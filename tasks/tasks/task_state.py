# from custom_msgs.msg import CVObject
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


# Note: Because these are rclpy Nodes, for them to be 'alive' and usable, after creating them, we still have to spin_once/spin
class TaskState(Node):
    STATE_TOPIC = "/state"
    DESIRED_POSE_TOPIC = "/controls/desired_pose"
    DESIRED_TWIST_VELOCITY_TOPIC = "/controls/desired_twist"
    CV_TOPICS = ["gate"]
    DEPTH_TOPIC = "/sensors/depth"

    def __init__(self, task_name: str):
        super().__init__(f"{task_name}_node")
        self.state_listener = self.create_subscription(
            Odometry, self.STATE_TOPIC, self._on_receive_state, 10
        )

        self.depth_listener = self.create_subscription(
            Float32, self.DEPTH_TOPIC, self._on_receive_depth, 10
        )

        self.desired_pose_publisher = self.create_publisher(
            Pose, self.DESIRED_POSE_TOPIC, 10
        )

        self.desired_twist_velocity_publisher = self.create_publisher(
            Twist, self.DESIRED_TWIST_VELOCITY_TOPIC, 10
        )

        self.depth = None
        self.state = None
        self.cv_data = {}
        for objectName in self.CV_TOPICS:
            self.cv_data[objectName] = {}
            for signal in ["bearing", "lateral", "distance"]:
                topic = "/object/" + objectName + "/" + signal
                self.cv_data[objectName][signal] = None
                self.create_subscription(
                    Float32,
                    topic,
                    lambda msg: self._on_receive_cv_data(msg, objectName, signal),
                    10,
                )

    def _on_receive_state(self, state: Odometry):
        self.state = state

    def _on_receive_depth(self, msg):
        self.depth = msg.data

    def _on_receive_cv_data(self, msg, objectName, signal):
        self.cv_data[objectName][signal] = msg.data
