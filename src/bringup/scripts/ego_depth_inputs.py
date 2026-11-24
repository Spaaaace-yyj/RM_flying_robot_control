#!/usr/bin/env python3
"""Pair D435 depth with calibrated rays and the VINS pose at exposure time."""
from array import array
from collections import deque

import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener


def cam0_to_root(transform, rotated):
    """T_calibrated_cam0_root from T_physical_infra2_root."""
    if rotated:
        # Premultiply by Rz(pi) in optical coordinates, including the baseline.
        t, q = transform.translation, transform.rotation
        t.x, t.y = -t.x, -t.y
        q.x, q.y, q.z, q.w = -q.y, q.x, q.w, -q.z
    return transform


def physical_depth(image, rotated):
    if not rotated:
        return image
    # Revert the driver's 180-degree pixel rotation. Work on byte pairs so
    # padded rows and big-endian 16UC1 also keep their original pixel values.
    rows = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.step)
    pixels = rows[:, :2 * image.width].reshape(image.height, image.width, 2)
    image.data = array('B', pixels[::-1, ::-1].tobytes())
    image.step = 2 * image.width
    return image


class DepthInputs(Node):
    def __init__(self):
        super().__init__('ego_depth_inputs')
        self.rotated = self.declare_parameter('images_rotated_180', True).value
        self.root = self.declare_parameter('camera_root', 'camera_link').value
        self.infra = self.declare_parameter('cam0_physical', 'camera_infra2_optical_frame').value
        self.cam0 = self.declare_parameter('cam0_calibrated', 'camera').value
        self.local = self.declare_parameter('local_frame', 'odom').value
        self.need_bridge = self.declare_parameter('publish_camera_bridge', True).value
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.pending = deque(maxlen=24)
        self.depth_out = self.create_publisher(Image, 'ego_input/depth', 5)
        self.info_out = self.create_publisher(CameraInfo, 'ego_input/camera_info', 5)
        self.pose_out = self.create_publisher(PoseStamped, 'ego_input/camera_pose', 5)
        self.depth = message_filters.Subscriber(
            self, Image, '/camera/camera/depth/image_rect_raw', qos_profile=qos_profile_sensor_data)
        self.info = message_filters.Subscriber(
            self, CameraInfo, '/camera/camera/depth/camera_info', qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.TimeSynchronizer([self.depth, self.info], 12)
        self.sync.registerCallback(self.receive)
        self.timer = self.create_timer(0.02, self.flush)

    def receive(self, img, info):
        if (img.header.frame_id != info.header.frame_id or not img.header.frame_id
                or img.width != info.width or img.height != info.height
                or img.width == 0 or img.height == 0 or img.encoding != '16UC1'
                or img.step < 2 * img.width or len(img.data) != img.step * img.height
                or Time.from_msg(img.header.stamp).nanoseconds == 0):
            self.get_logger().warning('Depth and CameraInfo geometry/encoding mismatch',
                                      throttle_duration_sec=5.0)
            return
        if self.pending and Time.from_msg(img.header.stamp) < Time.from_msg(self.pending[-1][0].header.stamp):
            self.pending.clear()
        self.pending.append((img, info))

    def flush(self):
        if self.need_bridge:
            try:
                ext = self.tf.lookup_transform(self.infra, self.root, Time())
            except TransformException as exc:
                self.get_logger().warning(f'Waiting for D435 static TF: {exc}', throttle_duration_sec=5.0)
                return
            bridge = TransformStamped()
            bridge.header.stamp = self.get_clock().now().to_msg()
            bridge.header.frame_id, bridge.child_frame_id = self.cam0, self.root
            bridge.transform = cam0_to_root(ext.transform, self.rotated)
            self.broadcaster.sendTransform(bridge)
            self.need_bridge = False

        # VINS publishes backend TF after the image arrives. Retry without blocking
        # the executor so it can receive those transforms, retaining a bounded window.
        while self.pending:
            img, info = self.pending[0]
            try:
                pose_tf = self.tf.lookup_transform(
                    self.local, img.header.frame_id, Time.from_msg(img.header.stamp))
            except TransformException as exc:
                self.get_logger().warning(f'Waiting for TF at depth exposure: {exc}', throttle_duration_sec=5.0)
                newest = self.pending[-1][0]
                if len(self.pending) == self.pending.maxlen or (
                    len(self.pending) > 1 and self.tf.can_transform(
                        self.local, newest.header.frame_id, Time.from_msg(newest.header.stamp))
                ):
                    self.pending.popleft()
                    continue
                return
            self.pending.popleft()
            pose = PoseStamped()
            pose.header.stamp = img.header.stamp
            pose.header.frame_id = self.local
            t = pose_tf.transform.translation
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = t.x, t.y, t.z
            pose.pose.orientation = pose_tf.transform.rotation
            self.info_out.publish(info)
            self.pose_out.publish(pose)
            self.depth_out.publish(physical_depth(img, self.rotated))


def main():
    rclpy.init()
    node = DepthInputs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
