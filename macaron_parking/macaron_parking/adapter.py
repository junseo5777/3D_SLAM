"""물리 속도·조향을 기존 macaron /erp_writes의 PWM·도 단위로 변환합니다."""

import math
import numpy as np


class MacaronCalibration:
    def __init__(self, speeds=(0.,), forward_pwm=(0.,), reverse_pwm=(0.,),
                 forward_input_sign=-1., steering_input_sign=0.,
                 confirmed=False, max_pwm=150., max_steer_deg=21.):
        self.speeds = np.asarray(speeds, dtype=float)
        self.forward_pwm = np.asarray(forward_pwm, dtype=float)
        self.reverse_pwm = np.asarray(reverse_pwm, dtype=float)
        self.forward_input_sign = forward_input_sign
        self.steering_input_sign = steering_input_sign
        self.confirmed = confirmed
        self.max_pwm, self.max_steer_deg = max_pwm, max_steer_deg
        if (self.speeds.ndim != 1 or not len(self.speeds)
                or self.forward_pwm.shape != self.speeds.shape
                or self.reverse_pwm.shape != self.speeds.shape
                or not np.isfinite(np.concatenate((self.speeds, self.forward_pwm, self.reverse_pwm))).all()
                or self.speeds[0] != 0 or self.forward_pwm[0] != 0 or self.reverse_pwm[0] != 0
                or (np.diff(self.speeds) <= 0).any()
                or (np.diff(self.forward_pwm) < 0).any() or (np.diff(self.reverse_pwm) < 0).any()
                or (self.forward_pwm < 0).any() or (self.reverse_pwm < 0).any()
                or not 0 < max_pwm <= 150 or not 0 < max_steer_deg <= 21
                or (self.forward_pwm > max_pwm).any() or (self.reverse_pwm > max_pwm).any()):
            raise ValueError('invalid PWM calibration table or output bounds')

    def readiness_failure(self):
        if not self.confirmed:
            return 'output_calibration_not_confirmed'
        if len(self.speeds) < 2 or self.forward_pwm[-1] <= 0 or self.reverse_pwm[-1] <= 0:
            return 'measured_speed_pwm_tables_required'
        if self.forward_input_sign not in (-1., 1.) or self.steering_input_sign not in (-1., 1.):
            return 'physical_direction_signs_required'
        return ''

    def convert(self, speed, steer):
        if not math.isfinite(speed) or not math.isfinite(steer):
            raise ValueError('nonfinite desired command')
        if speed == 0:
            return 0., 0.
        failure = self.readiness_failure()
        if failure:
            raise ValueError(failure)
        if abs(speed) > self.speeds[-1] + 1e-9:
            raise ValueError('desired speed exceeds measured calibration range')
        table = self.forward_pwm if speed > 0 else self.reverse_pwm
        pwm = float(np.interp(abs(speed), self.speeds, table))
        pwm *= self.forward_input_sign * (1 if speed > 0 else -1)
        angle = self.steering_input_sign * math.degrees(steer)
        return pwm, float(np.clip(angle, -self.max_steer_deg, self.max_steer_deg))


def adapter_failure(calibration, localization, mode, command, parking_status,
                    ros_now, wall_now, command_timeout=0.3, status_timeout=0.3,
                    reverse_observable=False):
    """계획부와 독립적으로 입력 지연·누락을 검사하여 PWM을 0으로 제한합니다."""
    failure = calibration.readiness_failure() or localization.failure(ros_now, wall_now) or mode.failure(wall_now)
    if failure:
        return failure
    from .safety import LocalizationGate
    if command is None or not LocalizationGate.fresh(command[2:], ros_now, wall_now, command_timeout):
        return 'parking_command_stale'
    if (parking_status is None or parking_status[0] != 'TRACKING'
            or not 0 <= wall_now - parking_status[1] <= status_timeout):
        return 'parking_not_tracking'
    if len(parking_status) < 3 or abs(parking_status[2] - command[2]) > 1e-6:
        return 'parking_command_status_mismatch'
    if command[0] < 0 and not reverse_observable:
        return 'reverse_observation_required'
    return ''
