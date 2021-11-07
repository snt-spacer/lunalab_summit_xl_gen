#!/usr/bin/env python3
"""A small example script that uses MoveIt 2 to follow a target inside Ignition Gazebo"""

from geometry_msgs.msg import Pose, PoseStamped, Transform
from moveit2 import MoveIt2Interface
from rclpy.node import Node
import rclpy

import tf2_ros


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

        # Plan trajectory only if target was moved
        if self.previous_target_pose_ != pose_msg.pose:
            # TODO: Do a proper transformation on target pose and not just position [Python interface for tf2 does not currently have this implemented - and its not worth implementing custom functions just for this example]
            transform = self.lookup_transform_sync(target_frame="robot_j2s7s300_link_base",
                                                   source_frame=pose_msg.header._frame_id)
            self.moveit2_.set_pose_goal([pose_msg.pose.position.x+transform.translation.x,
                                         pose_msg.pose.position.y+transform.translation.y,
                                         pose_msg.pose.position.z+transform.translation.z],
                                        [pose_msg.pose.orientation.x,
                                         pose_msg.pose.orientation.y,
                                         pose_msg.pose.orientation.z,
                                         pose_msg.pose.orientation.w])
            self.moveit2_.plan_kinematic_path()
            self.moveit2_.execute()

            # Update for next callback
            self.previous_target_pose_ = pose_msg.pose

    def lookup_transform_sync(self,
                              target_frame: str,
                              source_frame: str) -> Transform:

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
