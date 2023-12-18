#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
    Twist,
    TwistWithCovariance,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class SetpointPublisher(Node):
    def __init__(self):
        super().__init__("SetpointPublisher")
        self.listeners = {}
        self.listeners["imu_listener"] = self.create_subscription(
            Imu, "/sensors/imu", self.imu_callback, 10
        )

        self.pub = self.create_publisher(Odometry, "/state", 10)

        self.prev_time = self.get_clock().now()
        self.curr_time = self.get_clock().now()

        self.vx = 0
        self.vy = 0
        self.vz = 0

        self.xx = 0
        self.xy = 0
        self.xz = 0

        self.r = 0
        self.p = 0
        self.y = 0

        self.r_prev = 0
        self.p_prev = 0
        self.y_prev = 0

    def imu_callback(self, msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        r, p, y = euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        )

        self.curr_time = self.get_clock().now()
        dt = (self.curr_time - self.prev_time).nanoseconds / 1000000000
        self.prev_time = self.curr_time

        self.vx += ax * dt
        self.vy += ay * dt
        self.vz += az * dt

        self.xx += self.vx * dt
        self.xy += self.vy * dt
        self.xz += self.vz * dt

        r_prime = (r - self.r_prev) / dt
        p_prime = (p - self.p_prev) / dt
        y_prime = (y - self.y_prev) / dt

        self.r_prev = r
        self.p_prev = p
        self.y_prev = y

        pose = Pose()
        position = Point()
        position.x = self.xx
        position.y = self.xy
        position.z = self.xz
        quat = quaternion_from_euler(r, p, y)
        orientation = Quaternion()
        orientation.x, orientation.y, orientation.z, orientation.w = (
            quat[0],
            quat[1],
            quat[2],
            quat[3],
        )
        pose.position = position
        pose.orientation = orientation

        twist = Twist()
        twist.linear.x = self.vx
        twist.linear.y = self.vy
        twist.linear.z = self.vz
        twist.angular.x = r_prime
        twist.angular.y = p_prime
        twist.angular.z = y_prime

        odom = Odometry(
            pose=PoseWithCovariance(pose=pose), twist=TwistWithCovariance(twist=twist)
        )

        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    imu = SetpointPublisher()
    rclpy.spin(imu)
    imu.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
