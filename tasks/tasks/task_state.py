import threading

import numpy as np
import rclpy
from custom_msgs.msg import Correction, State
from imu_msg.msg import Imu
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray


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

        self.depth = -1.0
        self.state = [0.0, 0.0, 0.0]

        self.cv_listeners = {}
        self.cv_data = {}

        self.cv_data["gate"] = None
        self.gate_listener = self.create_subscription(
            Float64MultiArray,
            "/object/gate/bearing_lateral_distance",
            self._on_receive_cv_data_gate,
            100,
        )

        self.cv_data["orange_flare"] = None
        self.orange_flare_listener = self.create_subscription(
            Float64MultiArray,
            "/object/orange_flare/bearing_lateral_distance",
            self._on_receive_cv_data_orange_flare,
            100,
        )

        self.cv_data["blue_flare"] = None
        self.blue_flare_listener = self.create_subscription(
            Float64MultiArray,
            "/object/blue_flare/bearing_lateral_distance",
            self._on_receive_cv_data_blue_flare,
            100,
        )

        self.cv_data["red_flare"] = None
        self.red_flare_listener = self.create_subscription(
            Float64MultiArray,
            "/object/red_flare/bearing_lateral_distance",
            self._on_receive_cv_data_red_flare,
            100,
        )

        self.cv_data["yellow_flare"] = None
        self.yellow_flare_listener = self.create_subscription(
            Float64MultiArray,
            "/object/yellow_flare/bearing_lateral_distance",
            self._on_receive_cv_data_yellow_flare,
            100,
        )

        self.cv_data["blue_drum"] = None
        self.blue_drum_listener = self.create_subscription(
            Float64MultiArray,
            "/object/blue_drum/bearing_lateral_distance",
            self._on_receive_cv_data_blue_drum,
            100,
        )

        self.cv_data["red_drum"] = None
        self.red_drum_listener = self.create_subscription(
            Float64MultiArray,
            "/object/red_drum/bearing_lateral_distance",
            self._on_receive_cv_data_red_drum,
            100,
        )

    def stop_spinning(self):
        self.destroy_node()

    def __del__(self):
        self.stop_spinning()

    def _on_receive_state(self, msg):
        self.state = [msg.roll_pitch_yaw.x, msg.roll_pitch_yaw.y, msg.roll_pitch_yaw.z]

    def _on_receive_depth(self, msg):
        self.depth = msg.data

    def clear_cv_data(self, object_name):
        self.cv_data[object_name] = None

    def _on_receive_cv_data_gate(self, msg):
        msgData = np.array(msg.data).tolist()
        self.cv_data["gate"] = {}
        self.cv_data["gate"]["time"] = msgData[0]
        self.cv_data["gate"]["bearing"] = msgData[1]
        self.cv_data["gate"]["lateral"] = msgData[2]
        self.cv_data["gate"]["distance"] = msgData[3]

        # self.get_logger().info(self.cv_data.__repr__())

    def _on_receive_cv_data_orange_flare(self, msg):
        msgData = np.array(msg.data).tolist()
        self.cv_data["orange_flare"] = {}
        self.cv_data["orange_flare"]["time"] = msgData[0]
        self.cv_data["orange_flare"]["bearing"] = msgData[1]
        self.cv_data["orange_flare"]["lateral"] = msgData[2]
        self.cv_data["orange_flare"]["distance"] = msgData[3]

        # self.get_logger().info(self.cv_data.__repr__())

    def _on_receive_cv_data_blue_flare(self, msg):
        msgData = np.array(msg.data).tolist()
        self.cv_data["blue_flare"] = {}
        self.cv_data["blue_flare"]["time"] = msgData[0]
        self.cv_data["blue_flare"]["bearing"] = msgData[1]
        self.cv_data["blue_flare"]["lateral"] = msgData[2]
        self.cv_data["blue_flare"]["distance"] = msgData[3]

        # self.get_logger().info(self.cv_data.__repr__())

    def _on_receive_cv_data_red_flare(self, msg):
        msgData = np.array(msg.data).tolist()
        self.cv_data["red_flare"] = {}
        self.cv_data["red_flare"]["time"] = msgData[0]
        self.cv_data["red_flare"]["bearing"] = msgData[1]
        self.cv_data["red_flare"]["lateral"] = msgData[2]
        self.cv_data["red_flare"]["distance"] = msgData[3]

        # self.get_logger().info(self.cv_data.__repr__())

    def _on_receive_cv_data_yellow_flare(self, msg):
        msgData = np.array(msg.data).tolist()
        self.cv_data["yellow_flare"] = {}
        self.cv_data["yellow_flare"]["time"] = msgData[0]
        self.cv_data["yellow_flare"]["bearing"] = msgData[1]
        self.cv_data["yellow_flare"]["lateral"] = msgData[2]
        self.cv_data["yellow_flare"]["distance"] = msgData[3]

        # self.get_logger().info(self.cv_data.__repr__())

    def _on_receive_cv_data_blue_drum(self, msg):
        msgData = np.array(msg.data).tolist()
        self.cv_data["blue_drum"] = {}
        self.cv_data["blue_drum"]["time"] = msgData[0]
        self.cv_data["blue_drum"]["bearing"] = msgData[1]
        self.cv_data["blue_drum"]["lateral"] = msgData[2]
        self.cv_data["blue_drum"]["distance"] = msgData[3]

        # self.get_logger().info(self.cv_data.__repr__())

    def _on_receive_cv_data_red_drum(self, msg):
        msgData = np.array(msg.data).tolist()
        self.cv_data["red_drum"] = {}
        self.cv_data["red_drum"]["time"] = msgData[0]
        self.cv_data["red_drum"]["bearing"] = msgData[1]
        self.cv_data["red_drum"]["lateral"] = msgData[2]
        self.cv_data["red_drum"]["distance"] = msgData[3]

        # self.get_logger().info(self.cv_data.__repr__())
