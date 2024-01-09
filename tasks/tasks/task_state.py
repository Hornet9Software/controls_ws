from std_msgs.msg import Float32
from imu_msg.msg import Imu
from custom_msgs.msg import State, Correction
from rclpy.node import Node


class TaskState(Node):
    STATE_TOPIC = (
        "/sensors/imu/processed"  # once sensor fusion is up, this should be "/state".
    )
    CORRECTION_TOPIC = "/controls/correction"
    CV_OBJECTS = ["gate"]
    DEPTH_TOPIC = "/sensors/depth"  # no need for this once sensor fusion is up

    def __init__(self, task_name: str):
        super().__init__(f"{task_name}_node")
        self.state_listener = self.create_subscription(
            State, self.STATE_TOPIC, self._on_receive_state, 10
        )

        self.depth_listener = self.create_subscription(
            Float32, self.DEPTH_TOPIC, self._on_receive_depth, 10
        )

        self.correction_publisher = self.create_publisher(
            Correction, self.CORRECTION_TOPIC, 10
        )

        self.depth = None
        self.state = None
        self.cv_data = {}
        for objectName in self.CV_OBJECTS:
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

    def _on_receive_state(self, msg):
        self.state = msg

    def _on_receive_depth(self, msg):
        self.depth = msg.data

    def _on_receive_cv_data(self, msg, objectName, signal):
        self.cv_data[objectName][signal] = msg.data
