import threading

import numpy as np
import rclpy
from custom_msgs.msg import Correction, State
from imu_msg.msg import Imu
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray


class TaskState(Node):
    STATE_TOPIC = (
        "/sensors/imu/corrected"  # once sensor fusion is up, this should be "/state".
    )
    CORRECTION_TOPIC = "/controls/correction"
    CV_OBJECTS = [
        "gate",
        "orange_flare",
        "blue_flare",
        "red_flare",
        "yellow_flare",
        "blue_drum",
        "red_drum",
    ]
    DEPTH_TOPIC = "/sensors/depth"  # no need for this once sensor fusion is up

    def __init__(self):
        super().__init__(f"task_state_node")
        self.state_listener = self.create_subscription(
            Imu, self.STATE_TOPIC, self._on_receive_state, 10
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

    @classmethod
    def create_task_state(cls, task_name):
        return cls(task_name)

    def stop_spinning(self):
        self.destroy_node()

    def __del__(self):
        self.stop_spinning()

    def _on_receive_state(self, msg):
        self.state = [msg.roll_pitch_yaw.x, msg.roll_pitch_yaw.y, msg.roll_pitch_yaw.z]

    def _on_receive_depth(self, msg):
        self.depth = msg.data

    def _on_receive_cv_data(self, msg, objectName):
        msgData = np.array(msg.data).tolist()
        self.cv_data[objectName] = {}
        self.cv_data[objectName]["bearing"] = msgData[0]
        self.cv_data[objectName]["lateral"] = msgData[1]
        self.cv_data[objectName]["distance"] = msgData[2]

        self.get_logger().info(self.cv_data.__repr__())
