#!/usr/bin/env python3
"""A small example script that uses MoveIt 2 to follow a target inside Ignition Gazebo"""

from geometry_msgs.msg import Pose, PoseStamped, Transform
from moveit2 import MoveIt2Interface
from rclpy.node import Node
from typing import Tuple
import rclpy
import tf2_ros


def quat_mul(quat_0: Tuple[float, float, float, float],
             quat_1: Tuple[float, float, float, float],
             xyzw: bool = True) -> Tuple[float, float, float, float]:
    """
    Multiply two quaternions
    """
    # TODO: Replace custom quaternion multiplication with tf2's do_transform() once it is implemented for use in Python.

    if xyzw:
        x0, y0, z0, w0 = quat_0
        x1, y1, z1, w1 = quat_1
        return (x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
                -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0)
    else:
        w0, x0, y0, z0 = quat_0
        w1, x1, y1, z1 = quat_1
        return (-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0)


class MoveItFollowTarget(Node):

    def __init__(self):

        super().__init__("ex_moveit_follow_target")

        # Create a subscriber for target pose
        self.previous_target_pose_ = Pose()
        self.target_pose_sub_ = self.create_subscription(PoseStamped, "/target_pose",
                                                         self.target_pose_callback, 1)

        # Create MoveIt2 interface node
        self.moveit2_ = MoveIt2Interface(robot_model="kinova_j2s7s300")

        # Create tf2 buffer and listener for transform lookup
        self.tf2_buffer_ = tf2_ros.Buffer()
        self.tf2_listener_ = tf2_ros.TransformListener(buffer=self.tf2_buffer_,
                                                       node=self)

        # Spin up multi-threaded executor
        self.executor_ = rclpy.executors.MultiThreadedExecutor(3)
        self.executor_.add_node(self)
        self.executor_.add_node(self.moveit2_)
        self.executor_.add_node(self.tf2_listener_.node)
        self.executor_.spin()

    def target_pose_callback(self, pose_msg):
        """
        Plan and execute kinematic path each time target's position is changed
        """

        # Plan trajectory only if target was moved
        if self.previous_target_pose_ != pose_msg.pose:
            # Set position goal (in world coordinate system)
            self.moveit2_.set_position_goal([pose_msg.pose.position.x,
                                             pose_msg.pose.position.y,
                                             pose_msg.pose.position.z],
                                            frame=pose_msg.header._frame_id)

            # Set orientation goal (transformed into robot frame)
            transform = self.lookup_transform_sync(target_frame=self.moveit2_.arm_base_link,
                                                   source_frame=pose_msg.header._frame_id)
            orientation_goal = quat_mul((pose_msg.pose.orientation.x,
                                         pose_msg.pose.orientation.y,
                                         pose_msg.pose.orientation.z,
                                         pose_msg.pose.orientation.w),
                                        (transform.rotation.x,
                                         transform.rotation.y,
                                         transform.rotation.z,
                                         transform.rotation.w))
            self.moveit2_.set_orientation_goal(orientation_goal)

            # Plan and execute
            self.moveit2_.plan_kinematic_path()
            self.moveit2_.execute()

            # Update for next callback
            self.previous_target_pose_ = pose_msg.pose

    def lookup_transform_sync(self,
                              target_frame: str,
                              source_frame: str) -> Transform:
        """
        Lookup transform from `source_frame` to `target_frame` using tf2
        """

        while rclpy.ok():
            if self.tf2_buffer_.can_transform(target_frame=target_frame,
                                              source_frame=source_frame,
                                              time=rclpy.time.Time(),
                                              timeout=rclpy.time.Duration(seconds=1,
                                                                          nanoseconds=0)):
                transform_stamped = self.tf2_buffer_.lookup_transform(target_frame=target_frame,
                                                                      source_frame=source_frame,
                                                                      time=rclpy.time.Time())
                return transform_stamped.transform

            print(f'Lookup of transform from "{source_frame}"'
                  f' to "{target_frame}" failed, retrying...')


def main(args=None):

    rclpy.init(args=args)

    _target_follower = MoveItFollowTarget()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
