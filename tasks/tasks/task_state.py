import numpy as np
import rclpy

from std_msgs.msg import Float32, Float32MultiArray
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

        self.cv_listeners = {}
        self.cv_data = {}
        for objectName in self.CV_OBJECTS:
            self.cv_data[objectName] = None
            topic = "/object/" + objectName + "/bearing_lateral_distance"
            self.cv_listeners[objectName] = self.create_subscription(
                Float32MultiArray,
                topic,
                lambda msg: self._on_receive_cv_data(msg, objectName),
                10,
            )

        # Initialize self.state
        while (self.state is None) or (self.depth is None):
            rclpy.spin_once(self)

    def _on_receive_state(self, msg):
        self.state = msg

    def _on_receive_depth(self, msg):
        self.depth = msg.data

    def _on_receive_cv_data(self, msg, objectName):
        msgData = np.array(msg.data).tolist()
        self.cv_data[objectName] = {}
        self.cv_data[objectName]["bearing"] = msgData[0]
        self.cv_data[objectName]["lateral"] = msgData[1]
        self.cv_data[objectName]["distance"] = msgData[2]

        self.get_logger().info(self.cv_data.__repr__())
