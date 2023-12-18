import time

import rclpy
import tasks.utilities as utilities
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from tasks.task import Task
from tf2_ros import TransformListener
from tf_transformations import quaternion_from_euler

# Note: odom is global base_link is local


class MoveToPoseGlobalTask(Task):
    # Move to pose given in global coordinates
    def __init__(self, x, y, z, roll, pitch, yaw):
        super().__init__(task_name="move_to_global_task", outcomes=["done"])

        self.coords = [x, y, z, roll, pitch, yaw]

    def execute(self, ud):
        self.initial_state = self.state

        # pose from userdata if available
        arg_names = ["x", "y", "z", "roll", "pitch", "yaw"]
        for i in range(len(arg_names)):
            if arg_names[i] in ud:
                self.coords[i] = ud[arg_names[i]]

        self.desired_pose = Pose()
        self.desired_pose.position = Point(
            x=self.coords[0], y=self.coords[1], z=self.coords[2]
        )
        quat = quaternion_from_euler(self.coords[3], self.coords[4], self.coords[5])
        orientation = Quaternion()
        orientation.x, orientation.y, orientation.z, orientation.w = (
            quat[0],
            quat[1],
            quat[2],
            quat[3],
        )
        self.desired_pose.orientation = orientation

        # calls run
        return super().execute(ud)

    def run(self, ud):
        print("moving to ", self.desired_pose)
        rate = self.task_state.create_rate(15)

        # loop continues as long as conditional evaluates to false
        while not (
            self.state
            and utilities.stopped_at_pose(
                self.state.pose.pose, self.get_desired_pose(), self.state.twist.twist
            )
        ):
            rclpy.spin_once(self.task_state)
            print("not yet ", self.state)
            self.publish_desired_pose(self.get_desired_pose())
            time.sleep(0.5)
        print("i have arrived")
        return "done"

    def get_desired_pose(self) -> Pose:
        return self.desired_pose


class MoveToPoseLocalTask(MoveToPoseGlobalTask):
    # Move to pose given in local coordinates
    def __init__(self, x, y, z, roll, pitch, yaw):
        super().__init__(x, y, z, roll, pitch, yaw)

    def run(self, ud):
        # Transform the desired pose from base_link frame to odom frame
        # aka convert to global
        self.desired_pose = utilities.transform("base_link", "odom", self.desired_pose)

        return super().run(ud)


class AllocateVelocityLocalTask(Task):
    def __init__(self, x, y, z, roll, pitch, yaw):
        super().__init__(task_name="allocate_local_velocity", outcomes=["done"])
        linear = Vector3(x=x, y=y, z=z)
        angular = Vector3(x=roll, y=pitch, z=yaw)
        self.desired_twist = Twist(linear=linear, angular=angular)

    def run(self, ud):
        self.publish_desired_twist(self.desired_twist)
        return "done"


class AllocateVelocityGlobalTask(AllocateVelocityLocalTask):
    # Given global velocity, we first convert into local velocity
    def __init__(self, x, y, z, roll, pitch, yaw):
        super().__init__(x, y, z, roll, pitch, yaw)
        odom_global = Odometry()
        odom_global.twist.twist = self.desired_twist
        odom_local = utilities.transform("odom", "base_link", odom_global)
        self.desired_twist = odom_local.twist.twist
