import subprocess
import threading

import numpy as np
import rclpy
from controls_core.params import *
from controls_core.PIDManager import NormalPIDManager
from imu_msg.msg import Imu
from rclpy.node import Node
from sshkeyboard import listen_keyboard
from std_msgs.msg import Float32


class Teleop(Node):

    def __init__(self):
        super().__init__("teleop")

        self.create_timer(0.1, self._controlLoop)

        self.pid_manager = NormalPIDManager()

        self.currXYZ = [0.0, 0.0, 0.0]
        self.currRPY = [0.0, 0.0, 0.0]

        self.targetXYZ = [0.0, 0.0, -1.0]
        self.targetRPY = [0.0, 0.0, 0.0]

        self.depthListener = self.create_subscription(
            Float32, "/sensors/depth", self._depthProcessing, 10
        )

        self.imuListener = self.create_subscription(
            Imu, "/sensors/imu/corrected", self._imuProcessing, 10
        )

        self.keyboard_thread = threading.Thread(target=listen_keyboard, args=(self.on_press,self.on_release), daemon=True)
        self.keyboard_thread.start()

    def _imuProcessing(self, msg: Imu):
        self.currRPY = [
            msg.roll_pitch_yaw.x,
            msg.roll_pitch_yaw.y,
            msg.roll_pitch_yaw.z,
        ]

    def _depthProcessing(self, msg: Float32):
        self.currXYZ[2] = msg.data

    def on_press(self, key):
        if key == "w":
            self.targetXYZ[1] += 0.2
        elif key == "s":
            self.targetXYZ[1] += -0.2
        elif key == "a":
            self.targetXYZ[0] += -0.2
        elif key == "d":
            self.targetXYZ[0] += 0.2
        elif key == "h":
            self.targetXYZ[0] = 0.0
            self.targetXYZ[1] = 0.0
        elif key == "left":
            self.targetRPY[2] += 0.1
        elif key == "right":
            self.targetRPY[2] += -0.1
        elif key == "up":
            self.targetXYZ[2] += 0.1
        elif key == "down":
            self.targetXYZ[2] += -0.1

    def on_release(self, key):
        if key == "w":
            self.targetXYZ[1] = 0
        elif key == "s":
            self.targetXYZ[1] = 0
        elif key == "a":
            self.targetXYZ[0] = 0
        elif key == "d":
            self.targetXYZ[0] = 0

    def _controlLoop(self):
        self.pid_manager.correctVehicle(
            self.currRPY, self.targetRPY, self.currXYZ, self.targetXYZ
        )


def main(args=None):
    rclpy.init()

    teleop = Teleop()
    try:
        rclpy.spin(teleop)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        subprocess.run(["cansend", "can0", "000#7F7F7F7F7F7F7F7F"])
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
