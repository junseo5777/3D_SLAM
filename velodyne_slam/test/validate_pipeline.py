"""구조를 아는 합성 방에서 실제 FAST-LIO, 지도 저장, NDT를 검사한다.

연결된 로봇 노드와 분리하려면 ROS_DOMAIN_ID=91로 실행한다.
하드웨어 드라이버는 실행하지 않으며 이 결과로 센서 보정값을 검증할 수는 없다.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
import psutil
import yaml
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


def room_scan(x_offset=0.0):
    """VLP와 유사한 16개 링으로 주변의 여섯 평면을 관측하며 time 단위는 초다."""
    dtype = np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4'),('time','<f4'),('ring','<u2')])
    points = []
    for i in range(360):
        yaw = i * math.pi / 180
        for ring in range(16):
            pitch = (-15 + ring*2)*math.pi/180
            v = np.array([math.cos(pitch)*math.cos(yaw), math.cos(pitch)*math.sin(yaw), math.sin(pitch)])
            # 센서 초기 좌표계에서 비대칭 직육면체 형태의 방을 만든다.
            lo, hi = np.array([-5.,-4.,-1.]), np.array([6.,5.,2.])
            distance = min((hi[k] if v[k]>0 else lo[k])/v[k] for k in range(3) if abs(v[k])>1e-8)
            p = v*distance
            p[0] += x_offset
            points.append((*p, 30., i/3600.+ring*0.000002304, ring))
    array = np.array(points,dtype=dtype)
    msg = PointCloud2()
    msg.header.frame_id='velodyne';msg.height=1;msg.width=len(array)
    msg.fields=[PointField(name=name,offset=dtype.fields[name][1],datatype=PointField.UINT16 if name=='ring' else PointField.FLOAT32,count=1) for name in dtype.names]
    msg.point_step=dtype.itemsize;msg.row_step=msg.point_step*msg.width;msg.is_dense=True;msg.data=array.tobytes()
    return msg


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '91':
        raise RuntimeError('Run this synthetic test with ROS_DOMAIN_ID=91')
    directory=Path(tempfile.mkdtemp(prefix='lio_ndt_validation_'))
    calibration=directory/'calibration.yaml'
    # 카트의 뒤집힌 IMU 장착 방향을 재현한다. RViz는 yaw가 0인 초기 방향을 보낸다.
    calibration.write_text('calibrated: true\ntranslation: [0.0, 0.0, 0.0]\nrotation: [-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0]\ntime_offset_lidar_to_imu: 0.0\n')
    map_path=directory/'room.pcd'
    # 합성 지도 원점은 IMU/라이다 위치이며 카트 장착 오프셋만큼 뒷바퀴 중심이 떨어져 있다.
    cart = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/cart.yaml').read_text())
    assert cart['lidar_rpy_degrees'] == [0.0, 0.0, 0.0]
    expected_base = -np.array(cart['lidar_xyz'])
    rclpy.init();node=rclpy.create_node('synthetic_pipeline_validation')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    lidar_pub=node.create_publisher(PointCloud2,'/validation/points',QoSProfile(depth=10))
    imu_pub=node.create_publisher(Imu,'/validation/imu',qos_profile_sensor_data)
    initial_pub=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
    scan=room_scan();bad_scan=room_scan(20.0);records={};statuses=[];poses=[];orientations=[];pose_times=[];pose_stamps=[]
    subscriptions=[]
    for topic,cls in [('/fast_lio/odometry',Odometry),('/fast_lio/cloud_body',PointCloud2),('/fast_lio/cloud_world',PointCloud2)]:
        subscriptions.append(node.create_subscription(cls,topic,lambda msg,t=topic:records.setdefault(t,[]).append(time.monotonic()),10))
    subscriptions.append(node.create_subscription(String,'/localization/status',lambda msg:statuses.append(msg.data),QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    def on_pose(msg):
        poses.append([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
        pose_times.append(time.monotonic())
        pose_stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        q=msg.pose.orientation
        orientations.append([q.x,q.y,q.z,q.w])
    subscriptions.append(node.create_subscription(PoseStamped,'/localization/pose',on_pose,10))
    results={'kind':'synthetic_stationary_room','hardware':False,'directory':str(directory)}
    process=None
    try:
        for mode in ('mapping','localization'):
            records.clear();statuses.clear();poses.clear();orientations.clear();pose_times.clear();pose_stamps.clear()
            with (directory/f'{mode}.log').open('w') as log:
                process=subprocess.Popen(['ros2','launch','velodyne_slam',f'{mode}.launch.py','start_sensors:=false',
                    'input_topic:=/validation/points','imu_topic:=/validation/imu',f'extrinsics_file:={calibration}',f'map_path:={map_path}'],
                    stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                start=time.monotonic();next_imu=start;next_cloud=start;initial_sent=False;peak_rss=0;cpu_start={};cpu_last={}
                def tick():
                    nonlocal next_imu,next_cloud
                    current=time.monotonic()
                    if current>=next_imu:
                        imu=Imu();imu.header.frame_id='imu_link';imu.header.stamp=node.get_clock().now().to_msg()
                        imu.linear_acceleration.z=-9.80665;imu.orientation.w=1.;imu_pub.publish(imu);next_imu=current+0.01
                    if current>=next_cloud:
                        active_scan = bad_scan if mode=='localization' and 10 < current-start < 22 else scan
                        active_scan.header.stamp=rclpy.time.Time(nanoseconds=node.get_clock().now().nanoseconds-100000000).to_msg()
                        lidar_pub.publish(active_scan);next_cloud=current+0.1
                    rclpy.spin_once(node,timeout_sec=0.001)
                while time.monotonic()-start < (22 if mode == 'mapping' else 36):
                    if process.poll() is not None:raise RuntimeError(f'{mode} launch exited; see {directory}')
                    tick()
                    if mode=='localization' and not initial_sent and time.monotonic()-start>5:
                        initial=PoseWithCovarianceStamped();initial.header.frame_id='map';initial.header.stamp=node.get_clock().now().to_msg();initial.pose.pose.orientation.w=1.
                        initial.pose.pose.position.x = float(expected_base[0] + 0.6)
                        initial.pose.pose.position.y = float(expected_base[1])
                        initial.pose.pose.position.z = float(expected_base[2])
                        initial_pub.publish(initial);initial_sent=True
                    children=psutil.Process(process.pid).children(recursive=True)
                    memory=0
                    for child in children:
                        try:
                            memory+=child.memory_info().rss
                            cpu=child.cpu_times();value=cpu.user+cpu.system
                            cpu_start.setdefault(child.pid,value);cpu_last[child.pid]=value
                        except psutil.Error:pass
                    peak_rss=max(peak_rss,memory)
                result={'counts':{k:len(v) for k,v in records.items()},
                        'rates_hz':{k:(len(v)-1)/(v[-1]-v[0]) for k,v in records.items() if len(v)>1},
                        'peak_children_rss_mb':peak_rss/1024**2,
                        'average_cpu_cores':sum(cpu_last[k]-cpu_start[k] for k in cpu_last)/(time.monotonic()-start)}
                assert result['counts'].get('/fast_lio/odometry',0)>50,result
                mounting = tf_buffer.lookup_transform('base_link', 'velodyne', rclpy.time.Time())
                translation = mounting.transform.translation
                assert np.allclose([translation.x, translation.y, translation.z],
                                   cart['lidar_xyz'], atol=1e-5)
                assert abs(mounting.transform.rotation.w) > 0.99999
                result['base_to_lidar_tf_verified'] = True
                if mode=='mapping':
                    client=node.create_client(Trigger,'/save_map')
                    assert client.wait_for_service(timeout_sec=3)
                    future=client.call_async(Trigger.Request());deadline=time.monotonic()+5
                    while not future.done() and time.monotonic()<deadline:tick()
                    assert future.done() and future.result().success, 'Map save failed'
                    result['save_response']=future.result().message
                    assert map_path.is_file() and map_path.stat().st_size>1000
                    node.destroy_client(client)
                else:
                    result['localization_pose_count']=len(poses);result['last_status']=statuses[-3:]
                    result['pose_rate_hz']=(len(pose_times)-1)/(pose_times[-1]-pose_times[0])
                    result['max_pose_gap_seconds']=float(np.max(np.diff(pose_times)))
                    result['pose_stamps_monotonic']=bool(np.all(np.diff(pose_stamps)>=0))
                    result['saw_candidate_verification']=any('verifying_candidate' in s for s in statuses)
                    assert result['pose_rate_hz'] > 8.0,result
                    assert result['max_pose_gap_seconds'] < 0.5,result
                    assert result['pose_stamps_monotonic'] and result['saw_candidate_verification'],result
                    assert abs(poses[0][0]-(expected_base[0]+0.6)) < 0.05,result
                    result['last_pose']=poses[-1] if poses else None
                    assert len(poses)>5,result
                    assert np.linalg.norm(np.array(poses[-1])-expected_base)<0.4,result
                    result['last_base_orientation_xyzw']=orientations[-1]
                    # IMU가 뒤집혀 있어도 카트는 지도 +X를 향하며 위아래가 올바르다.
                    assert abs(orientations[-1][3])>0.99,result
                    result['saw_relocalizing']=any('relocalizing' in s for s in statuses)
                    result['saw_recovered']=any('recovered' in s for s in statuses)
                    result['saw_relaxed_recovery']=any('recovery_level=1' in s for s in statuses)
                    result['saw_strict_tracking_restored']=any(
                        'strict_tracking_restored' in s for s in statuses)
                    assert result['saw_relocalizing'] and result['saw_recovered'],result
                    assert result['saw_relaxed_recovery'] and result['saw_strict_tracking_restored'],result
                results[mode]=result
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
                process=None
        results['passed']=True
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        node.destroy_node();rclpy.shutdown()
        (directory/'statuses.json').write_text(json.dumps(statuses,indent=2))
        (directory/'results.json').write_text(json.dumps(results,indent=2))
        print(json.dumps(results,indent=2))


if __name__=='__main__':main()
