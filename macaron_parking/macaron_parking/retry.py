"""선택적 한 번의 재주차에서 실제 전진 구간 후 후진으로 마무리합니다."""

import heapq
import math
import time
import numpy as np
from .hybrid_astar import PlanResult, _Node, normalize_angle


def plan_retry(planner, start, goal, min_forward=0.40, max_reverse=3.0, cancel_check=None):
    if min_forward <= 0 or max_reverse <= 0:
        raise ValueError('retry distances must be positive')
    started = time.monotonic()
    if cancel_check is not None and cancel_check():
        return PlanResult(False, reason='retry planning cancelled')
    planner._prepare()
    if not planner._path_is_free(np.array([start, goal])):
        return PlanResult(False, reason='retry start/goal collision or unknown ground')
    root = _Node(*start, 0., 1., 0., 0., None)
    queue = [(0., 0, root, 0., 0.)]
    closed = set()
    counter = expansions = 0
    while queue and expansions < planner.max_expansions:
        if cancel_check is not None and cancel_check():
            return PlanResult(False, reason='retry planning cancelled', expansions=expansions,
                              elapsed=time.monotonic() - started)
        if time.monotonic() - started > planner.time_limit:
            return PlanResult(False, reason='retry time limit reached', expansions=expansions)
        _, _, node, forward_distance, reverse_distance = heapq.heappop(queue)
        key = (*planner._state_index(*node.pose), node.phase,
               min(int(forward_distance / planner.step), int(math.ceil(min_forward / planner.step))),
               int(round(reverse_distance / planner.step)))
        if key in closed:
            continue
        closed.add(key)
        expansions += 1
        if forward_distance >= min_forward and max_reverse > reverse_distance:
            bridge = planner._connect_direction(node.pose, goal, -1.,
                                                max_reverse - reverse_distance)
            if bridge is not None:
                poses, directions = planner._assemble_single_tree(node, *bridge)
                if directions[0] == 1 and directions[-1] == -1:
                    return PlanResult(True, poses, directions, reason='one forward/reverse retry',
                                      expansions=expansions, elapsed=time.monotonic() - started)
        end_x, end_y, end_yaw, margins = planner._sweep(node, 1.)
        for gi, gear in enumerate(planner._gears):
            if (node.phase == 1 and gear > 0) or (gear < 0 and forward_distance < min_forward):
                continue
            reverse = reverse_distance + (planner.step if gear < 0 else 0.)
            if reverse > max_reverse:
                continue
            forward = forward_distance + (planner.step if gear > 0 else 0.)
            for si, steer in enumerate(planner._steers):
                if margins[gi, si] < 0:
                    continue
                cost = node.cost + planner._step_cost(node, float(steer), float(gear), margins[gi, si])
                child = _Node(float(end_x[gi, si]), float(end_y[gi, si]),
                              normalize_angle(float(end_yaw[gi, si])), float(steer), float(gear),
                              float(gear) * planner.step, cost, node,
                              phase=1 if gear < 0 else 0)
                estimate = math.hypot(child.x - goal[0], child.y - goal[1])
                counter += 1
                heapq.heappush(queue, (cost + planner.heuristic_weight * estimate,
                                      counter, child, forward, reverse))
    return PlanResult(False, reason='retry search exhausted', expansions=expansions)
