"""Exercise installed/build-tree nodes with Best Effort cameras and IMU; no hardware."""
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu


def exercise(binary, package, cameras, imu, via_launch=False):
    with tempfile.TemporaryDirectory(prefix='vins-foxy-smoke-') as directory:
        directory = Path(directory)
        text = (package / 'config/euroc/euroc_stereo_imu_config.yaml').read_text()
        replacements = {
            'num_of_cam': str(cameras), 'imu': str(imu),
            'output_path': '"' + str(directory / 'results') + '"',
            'image0_topic': '"/smoke/cam0"', 'image1_topic': '"/smoke/cam1"',
            'imu_topic': '"/smoke/imu"',
            'cam0_calib': '"' + str(package / 'config/euroc/cam0_mei.yaml') + '"',
            'cam1_calib': '"' + str(package / 'config/euroc/cam1_mei.yaml') + '"',
        }
        for key, value in replacements.items():
            text, matches = re.subn(r'^' + key + r':.*$', key + ': ' + value,
                                   text, flags=re.MULTILINE)
            assert matches == 1, key
        config = directory / 'config.yaml'
        config.write_text(text)
        command = ([str(binary), '--ros-args', '-p', 'config_file:=' + str(config)]
                   if not via_launch else [
                       'ros2', 'launch', 'vins_fusion_ros2', 'vins_fusion_ros2.launch.py',
                       'config_file:=' + str(config), 'use_sim_time:=false'])
        with (directory / 'node.log').open('w+') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            node = rclpy.create_node('foxy_sensor_smoke')
            received = []
            node.create_subscription(Image, '/image_track', received.append, 10)
            left = node.create_publisher(Image, '/smoke/cam0', qos_profile_sensor_data)
            right = node.create_publisher(Image, '/smoke/cam1', qos_profile_sensor_data)
            imu_pub = node.create_publisher(Imu, '/smoke/imu', qos_profile_sensor_data)
            failure = None
            try:
                deadline = time.monotonic() + 12
                while time.monotonic() < deadline:
                    assert process.poll() is None, 'node exited during startup'
                    if (node.count_subscribers('/smoke/cam0') and
                            (cameras == 1 or node.count_subscribers('/smoke/cam1')) and
                            (not imu or node.count_subscribers('/smoke/imu'))):
                        break
                    rclpy.spin_once(node, timeout_sec=0.05)
                else:
                    raise AssertionError('sensor subscriptions did not appear')
                # Give DDS endpoints time to match, then send six textured frames.
                for _ in range(5):
                    rclpy.spin_once(node, timeout_sec=0.05)
                texture = np.random.RandomState(12).randint(0, 256, (480, 752), np.uint8)
                stamp = node.get_clock().now().nanoseconds
                for frame in range(6):
                    frame_ns = stamp + frame * 100000000
                    if imu:
                        for sample in range(21):
                            msg = Imu()
                            sample_ns = frame_ns - 10000000 + sample * 5000000
                            msg.header.stamp.sec = sample_ns // 1000000000
                            msg.header.stamp.nanosec = sample_ns % 1000000000
                            msg.linear_acceleration.z = 9.81007
                            msg.orientation.w = 1.0
                            imu_pub.publish(msg)
                    for publisher, offset in [(left, frame), (right, frame - 5)][:cameras]:
                        msg = Image()
                        msg.header.stamp.sec = frame_ns // 1000000000
                        msg.header.stamp.nanosec = frame_ns % 1000000000
                        msg.width, msg.height, msg.step = 752, 480, 752
                        msg.encoding = 'mono8'
                        msg.data = np.roll(texture, offset, axis=1).tobytes()
                        publisher.publish(msg)
                    until = time.monotonic() + 0.15
                    while time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=0.02)
                    assert process.poll() is None, 'node exited while processing sensors'
                deadline = time.monotonic() + 5
                while len(received) < 2 and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                assert len(received) >= 2, 'no tracked images from Best Effort input'
                assert all(image.encoding == 'bgr8' for image in received)
                assert (directory / 'results/vio.csv').is_file()
            except BaseException as error:
                failure = error
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                    failure = failure or AssertionError('shutdown hung')
                node.destroy_node()
            log.seek(0)
            output = log.read()
            if failure:
                raise AssertionError(str(failure) + '\n' + output) from failure
            assert process.returncode == 0, output
            print('PASS: cameras=%d imu=%d launch=%s tracked_images=%d' % (
                cameras, imu, via_launch, len(received)), flush=True)


def main():
    binary, package = Path(sys.argv[1]), Path(sys.argv[2])
    via_launch = '--launch' in sys.argv[3:]
    rclpy.init()
    try:
        for cameras, imu in [(2, 1), (1, 1), (2, 0)]:
            exercise(binary, package, cameras, imu, via_launch)
    finally:
        rclpy.shutdown()
    bad = subprocess.run([str(binary), '--ros-args', '-p',
                          'config_file:=/not/a/real/vins-config.yaml'],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, timeout=10)
    assert bad.returncode == 1 and 'Cannot open VINS config_file' in bad.stdout, bad.stdout
    print('PASS: invalid configuration exits cleanly', flush=True)


if __name__ == '__main__':
    main()
