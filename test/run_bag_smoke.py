"""Bounded existing-bag integration run. Uses own ROS domain; no hardware commands."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from collections import Counter
# RMW selection occurs during rclpy import on this installation.
os.environ['ROS_DOMAIN_ID']='137'
os.environ['RMW_IMPLEMENTATION']='rmw_cyclonedds_cpp'
os.environ['ROS_LOCALHOST_ONLY']='1'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--seconds',type=float,default=95)
    ap.add_argument('--output',required=True)
    a=ap.parse_args()
    out=Path(a.output); out.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_DOMAIN_ID']='137'
    os.environ['RMW_IMPLEMENTATION']='rmw_cyclonedds_cpp'
    os.environ['ROS_LOCALHOST_ONLY']='1'
    rclpy.init(); n=Node('bridge_bag_test_observer')
    processes=[]; files=[]; counts=Counter(); events=[]; pose_count=[0]
    statusfile=(out/'status.jsonl').open('w'); eventfile=(out/'events.jsonl').open('w')
    files += [statusfile,eventfile]
    def status(m):
        d=json.loads(m.data); counts[d['source']]+=1
        statusfile.write(m.data+'\n')
    def event(m):
        d=json.loads(m.data); events.append(d); eventfile.write(m.data+'\n')
    def pose(m): pose_count[0]+=1
    n.create_subscription(String,'/rtk_vio_bridge/status',status,100)
    n.create_subscription(String,'/rtk_vio_bridge/events',event,100)
    n.create_subscription(PoseStamped,'/rtk_vio_bridge/pose',pose,100)
    def start(label,cmd):
        f=(out/(label+'.log')).open('w'); files.append(f)
        p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append((label,p)); return p
    try:
        start('bridge',['ros2','run','rtk_vio_bridge','bridge','--ros-args',
            '-p','use_sim_time:=true','-p','origin_configured:=true','-p','extrinsics_configured:=true',
            '-p','origin_lat:=25.0','-p','origin_lon:=121.0','-p','origin_alt:=30.0',
            '-p','manual_alignment_enabled:=true','-p','field_from_vio_yaw_deg:=22.918311805',
            '-p','fix_topic:=/test/gps/fix','-p','quality_topic:=/test/gps/diagnostics',
            '-p','heading_topic:=/test/heading','-p','heading_quality_topic:=/test/heading_diagnostics'])
        start('synthetic',['ros2','run','rtk_vio_bridge','synthetic_rtk','--ros-args','-p','use_sim_time:=true'])
        start('openvins',['ros2','launch','ov_msckf','subscribe.launch.py',
            'config_path:=/home/tucker/openVINS_ws/src/open_vins/config/rs_d435i_user/estimator_config_mower_dyn.yaml',
            'rviz_enable:=false','verbosity:=WARNING'])
        # Allow ROS discovery before playback.
        deadline=time.monotonic()+4
        while time.monotonic()<deadline: rclpy.spin_once(n,timeout_sec=.1)
        start('bag',['ros2','bag','play','/home/tucker/rosbag2/d435i_vio_test_longdist',
            '--clock','100','--disable-keyboard-controls','--topics','/camera/camera/imu',
            '/camera/camera/infra1/image_rect_raw','/camera/camera/infra2/image_rect_raw'])
        deadline=time.monotonic()+a.seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=.1)
            for label,p in processes:
                if p.poll() is not None: raise RuntimeError(f'{label} exited {p.returncode}')
    finally:
        for _,p in reversed(processes):
            if p.poll() is None: os.killpg(p.pid,signal.SIGINT)
        for _,p in processes:
            try: p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGTERM); p.wait(timeout=5)
        result=dict(status_counts=dict(counts),pose_count=pose_count[0],
                    event_counts=dict(Counter(e['event'] for e in events)),
                    recovered=[e for e in events if e['event']=='rtk_recovered'],
                    scope='existing longdist prefix + synthetic RTK from VIO, NOT accuracy validation')
        (out/'summary.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2))
        for f in files: f.close()
        n.destroy_node(); rclpy.shutdown()
    assert pose_count[0]>0, 'no output poses'
    assert all(counts[s]>0 for s in ('RTK_ANCHORED','VIO_CONTINUATION','RECOVERING')), 'missing states'
    assert result['recovered'], 'no successful recovery'


if __name__=='__main__': main()
