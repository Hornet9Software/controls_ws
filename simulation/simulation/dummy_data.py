import math
import random
from abc import ABC, abstractmethod

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from imu_msg.msg import Imu
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Header


class IMU_Generator(ABC):
    @abstractmethod
    def update(self):
        pass  # pragma: no cover


class IMU_SHM(IMU_Generator):
    def __init__(self, amplitude=[0, 0, 1], wavelength=10) -> None:
        self.count = 0
        self.amplitude = np.array(amplitude)
        self.wavelength = wavelength

    def update(self):
        self.roll_pitch_yaw = self.amplitude * np.sin(
            self.count / self.wavelength * np.pi
        )
        self.count += 1
        return self.roll_pitch_yaw


class IMU_Constant(IMU_Generator):
    def __init__(self, roll_pitch_yaw) -> None:
        self.roll_pitch_yaw = roll_pitch_yaw

    def update(self):
        return self.roll_pitch_yaw


class DummyData(Node):
    def __init__(self, imuGenerator, yoloFps=40, depthHz=50, imuHz=50):
        super().__init__(node_name="dummy_data_publisher")

        self.yoloUpdatePeriod = 1 / yoloFps
        self.depthUpdatePeriod = 1 / depthHz
        self.imuUpdatePeriod = 1 / imuHz

        self.yoloPublisher = self.create_publisher(
            Float32MultiArray, "/object/gate/yolo", 10
        )
        self.yoloTimer = self.create_timer(self.yoloUpdatePeriod, self.yoloDummyData)

        self.depthPublisher = self.create_publisher(Float32, "/sensors/depth", 10)
        self.depthTimer = self.create_timer(self.depthUpdatePeriod, self.depthDummyData)

        self.imuGenerator = imuGenerator
        self.imuPublisher = self.create_publisher(Imu, "/sensors/imu", 10)
        self.imuTimer = self.create_timer(self.imuUpdatePeriod, self.imuDummyData)

    def yoloDummyData(self):
        xMin = random.uniform(0.2 * 640, 0.8 * 640)
        xMax = random.uniform(xMin, 640)
        x = (xMin + xMax) / 2.0
        w = xMax - xMin

        # Assuming AUV is centred vertically to gate
        y = 240
        h = random.uniform(0.2 * 480, 0.95 * 480)

        msg = Float32MultiArray()
        msg.data = [x, y, w, h]
        self.yoloPublisher.publish(msg)
        print("PUBLISHED BBOX:", msg.data)

    def depthDummyData(self):
        depth = random.uniform(-2.0, 0.0)
        msg = Float32()
        msg.data = depth
        self.depthPublisher.publish(msg)
        print("PUBLISHED DEPTH:", depth)

    def imuDummyData(self):
        roll_pitch_yaw = self.imuGenerator.update()

        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.frame_id = str(0)

        imu_msg.linear_acceleration = Vector3()
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0

        imu_msg.roll_pitch_yaw = Vector3()
        imu_msg.roll_pitch_yaw.x = roll_pitch_yaw[0]
        imu_msg.roll_pitch_yaw.y = roll_pitch_yaw[1]
        imu_msg.roll_pitch_yaw.z = roll_pitch_yaw[2]
        self.imuPublisher.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)

    imuGen = IMU_Constant(roll_pitch_yaw=[0.0, 0.0, np.pi / 2.0])
    # imuGen = IMU_Constant(roll_pitch_yaw=[0.0, 0.0, 0.0])
    pub = DummyData(imuGenerator=imuGen)

    try:
        rclpy.spin(pub)
    except KeyboardInterrupt:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
