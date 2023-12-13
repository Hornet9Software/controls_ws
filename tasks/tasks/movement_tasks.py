from task import Task
from geometry_msgs.msg import Pose, Quaternion, Twist, Point, Vector3
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
import utilities
import rclpy

class MoveToPoseGlobalTask(Task):
    def __init__(self, x, y, z, roll, pitch, yaw):
        super().__init__(task_name='move_to_global_task', outcomes=['done'])

        self.coords = [x,y,z,roll,pitch,yaw]
    

    def execute(self, ud):
        self.initial_state = self.state

        # pose from userdata if available
        arg_names = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        for i in range(len(arg_names)):
            if arg_names[i] in ud:
                self.coords[i] = ud[arg_names[i]]

        self.desired_pose = Pose()
        self.desired_pose.position = Point(x=self.coords[0], y=self.coords[1], z=self.coords[2])
        self.desired_pose.orientation = Quaternion(
            *quaternion_from_euler(
                self.coords[3],
                self.coords[4],
                self.coords[5]
            )
        )

        # calls run
        return super().execute(ud)
    
    def run(self, ud):
        print("moving to ", self.desired_pose)
        rate = self.task_state.create_rate(15)

        # loop continues as long as conditional evaluates to false
        while not(
            self.state and utilities.stopped_at_pose(
                self.state.pose.pose,
                self.get_desired_pose(),
                self.state.twist.twist
            )
        ):
            self.publish_desired_pose(self.get_desired_pose())
            rate.sleep()
        
        return 'done'
    
    def get_desired_pose(self) -> Pose:
        return self.desired_pose