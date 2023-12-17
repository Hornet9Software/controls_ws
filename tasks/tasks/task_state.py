from typing import Callable as _Callable, Optional, Union
from dependency_injector.providers import Injection
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from custom_msgs.msg import CVObject


# Note: Because these are rclpy Nodes, for them to be 'alive' and usable, after creating them, we still have to spin_once/spin
class TaskState(Node):
    STATE_TOPIC  = '/state'
    DESIRED_POSE_TOPIC = '/controls/desired_pose'
    DESIRED_TWIST_VELOCITY_TOPIC = '/controls/desired_twist'
    CV_TOPICS = []

    def __init__(self, task_name : str):
        super().__init__(f'{task_name}_node')
        self.state_listener = self.create_subscription(Odometry, self.STATE_TOPIC, self._on_receive_state, 10)
        self.desired_pose_publisher = self.create_publisher(Pose, self.DESIRED_POSE_TOPIC, 10)
        self.desired_twist_velocity_publisher = self.create_publisher(Twist, self.DESIRED_TWIST_VELOCITY_TOPIC, 10)
        self.state = None
        self.cv_data = {}
        for cv_topic in self.CV_TOPICS:
            topic = f'/cv_data/{cv_topic}'
            self.cv_data[cv_topic] = None
            self.create_subscription(CVObject, topic, self._on_receive_cv_data, 10)
        

    def _on_receive_state(self, state : Odometry):
        self.state = state
        

    def _on_receive_cv_data(self, cv_object : CVObject):
        self.cv_data[cv_object.label] = cv_object
