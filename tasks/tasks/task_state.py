import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from custom_msgs.msg import CVObject


class TaskState:
    STATE_TOPIC = '/state'
    DESIRED_POSE_TOPIC = 'controls/desire_pose'
    DESIRED_TWIST_VELOCITY_TOPIC = 'controls/desired_twist'
    CV_DATA_TOPICS = []

    def __init__(self):
        self.state_listener = Node.create_subscription(Odometry, self.STATE_TOPIC, self._on_receive_state,)
        self.desired_pose_publisher = rclpy.Node.create_publisher(Pose, self.DESIRED_POSE_TOPIC, 10)
        self.desired_twist_velocity_publisher = rclpy.Node.create_publisher(Twist, self.DESIRED_TWIST_VELOCITY_TOPIC, 10)
        self.state = None
        self.cv_data = {}
        for topic in self.CV_DATA_TOPICS:
            name = topic
            self.cv_data[name] = None
            Node.create_subscription(CVObject, name, self._on_receive_cv_data)
        

    
    def _on_receive_cv_data(self, cv_data):
        self.cv_data[CVObject.header.something] = cv_data
