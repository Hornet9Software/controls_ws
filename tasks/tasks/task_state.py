import threading

import numpy as np
import rclpy
from imu_msg.msg import Imu
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

from custom_msgs.msg import Correction, State


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

        self.depth = -1.0
        self.state = [0.0, 0.0, 0.0]

        self.cv_listeners = {}
        self.cv_data = {}

        self.cv_data["gate"] = None
        self.gate_listener = self.create_subscription(
            Float32MultiArray,
            "/object/gate/bearing_lateral_distance",
            lambda msg: self._on_receive_cv_data(msg, "gate"),
            100,
        )

        self.cv_data["orange_flare"] = None
        self.orange_flare_listener = self.create_subscription(
            Float32MultiArray,
            "/object/orange_flare/bearing_lateral_distance",
            lambda msg: self._on_receive_cv_data(msg, "orange_flare"),
            100,
        )

        self.cv_data["blue_flare"] = None
        self.blue_flare_listener = self.create_subscription(
            Float32MultiArray,
            "/object/blue_flare/bearing_lateral_distance",
            lambda msg: self._on_receive_cv_data(msg, "blue_flare"),
            100,
        )

        self.cv_data["red_flare"] = None
        self.red_flare_listener = self.create_subscription(
            Float32MultiArray,
            "/object/red_flare/bearing_lateral_distance",
            lambda msg: self._on_receive_cv_data(msg, "red_flare"),
            100,
        )

        self.cv_data["yellow_flare"] = None
        self.yellow_flare_listener = self.create_subscription(
            Float32MultiArray,
            "/object/yellow_flare/bearing_lateral_distance",
            lambda msg: self._on_receive_cv_data(msg, "yellow_flare"),
            100,
        )

        self.cv_data["blue_drum"] = None
        self.blue_drum_listener = self.create_subscription(
            Float32MultiArray,
            "/object/blue_drum/bearing_lateral_distance",
            lambda msg: self._on_receive_cv_data(msg, "blue_drum"),
            100,
        )

        self.cv_data["red_drum"] = None
        self.red_drum_listener = self.create_subscription(
            Float32MultiArray,
            "/object/red_drum/bearing_lateral_distance",
            lambda msg: self._on_receive_cv_data(msg, "red_drum"),
            100,
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

        # self.get_logger().info(self.cv_data.__repr__())
