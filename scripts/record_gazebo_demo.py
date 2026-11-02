"""Record the real supported-base smoke test from an observer camera.

Run in a terminal sourced with /opt/ros/humble/setup.bash. The original robot
and room remain unchanged; only a camera is added to a temporary world.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np


def camera_world(source, destination):
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    camera = ET.fromstring('''<model name="portfolio_observer"><static>true</static>
      <pose>1.8 -2.8 1.4 0 0.346 1.976</pose><link name="camera_link">
      <sensor name="observer" type="camera"><always_on>true</always_on>
      <update_rate>20</update_rate><topic>/portfolio/observer</topic><camera>
      <horizontal_fov>0.85</horizontal_fov><image><width>1280</width><height>800</height>
      <format>R8G8B8</format></image><clip><near>0.05</near><far>30</far></clip>
      </camera></sensor></link></model>''')
    world.append(camera)
    tree.write(destination, encoding='utf-8', xml_declaration=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    frames = out / 'frames'
    frames.mkdir(exist_ok=True)
    root = Path(__file__).resolve().parents[1]
    package = root / 'src/beni_sim'
    sys.path.insert(0, str(package))
    env = os.environ.copy()
    env.update({'ROS_LOCALHOST_ONLY':'1', 'ROS_DOMAIN_ID':'96',
                'IGN_PARTITION':'portfolio_beni_'+str(os.getpid()),
                'ROS_LOG_DIR':str(out/'ros_logs'),
                'PYTHONPATH':str(package)+os.pathsep+env.get('PYTHONPATH','')})
    # Set before importing rclpy so this process joins only the isolated test.
    os.environ.update({key:env[key] for key in ['ROS_LOCALHOST_ONLY','ROS_DOMAIN_ID','IGN_PARTITION','ROS_LOG_DIR']})
    world = out / 'observer_room.sdf'
    camera_world(package/'worlds/room.sdf', world)
    processes = []
    handles = []
    def start(command, name):
        handle = (out/(name+'.log')).open('w');handles.append(handle)
        process = subprocess.Popen(command, env=env, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process
    try:
        start(['ign','gazebo','-r','-s','--headless-rendering',str(world)],'gazebo')
        topics = ['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                  '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                  '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                  '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                  '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                  '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
                  '/portfolio/observer@sensor_msgs/msg/Image[ignition.msgs.Image']
        start(['/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge',*topics],'bridge')
        start([sys.executable,'-c','from beni_sim.drive_guard import main; main()'],'guard')
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from beni_sim.smoke_test import SmokeTest
        class RecordedTest(SmokeTest):
            def __init__(self):
                super().__init__()
                self.frames=[]
                self.create_subscription(Image,'/portfolio/observer',self.capture,qos_profile_sensor_data)
            def capture(self,msg):
                array=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)
                channels=4 if msg.encoding in ['rgba8','bgra8'] else 3
                image=array[:,:msg.width*channels].reshape(msg.height,msg.width,channels)
                if msg.encoding=='rgb8': image=cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
                elif msg.encoding=='rgba8': image=cv2.cvtColor(image,cv2.COLOR_RGBA2BGR)
                elif msg.encoding=='bgra8': image=cv2.cvtColor(image,cv2.COLOR_BGRA2BGR)
                name=f'{len(self.frames):05d}.jpg'
                cv2.imwrite(str(frames/name),image,[cv2.IMWRITE_JPEG_QUALITY,94])
                self.frames.append({'file':name,'stamp_s':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9})
        rclpy.init();node=RecordedTest()
        try:
            result=node.check()
            node.pump(1.0)
            assert len(node.frames)>30, 'Observer camera did not provide enough frames'
            timestamps=node.frames
        finally:
            node.destroy_node()
            rclpy.shutdown()
        result.update({'observer_frames':len(timestamps),'capture':'Gazebo Fortress observer camera',
                       'scope':'Supported base smoke test; odometry measurements, not balancing'})
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        with (out/'frame_timestamps.csv').open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=['file','stamp_s']);writer.writeheader();writer.writerows(timestamps)
        duration=timestamps[-1]['stamp_s']-timestamps[0]['stamp_s']
        fps=(len(timestamps)-1)/duration
        subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-framerate',str(fps),'-i',str(frames/'%05d.jpg'),
                        '-vf','scale=960:600,fps=20','-an','-c:v','libx264','-crf','22','-pix_fmt','yuv420p',
                        '-movflags','+faststart',str(out/'gazebo-supported-base.mp4')],check=True)
        print(json.dumps(result,indent=2),flush=True)
    finally:
        for process in reversed(processes):
            if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        for process in processes:
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        for handle in handles:handle.close()


if __name__=='__main__':main()
