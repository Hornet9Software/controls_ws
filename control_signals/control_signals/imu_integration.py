import rclpy
import math
import numpy as np
from rclpy.node import Node
from imu_msg.msg import Imu
from custom_msgs.msg import State
from std_msgs.msg import Float32, Float32MultiArray


class IMUIntegration(Node):
    RAW_TOPIC = "/sensors/imu/corrected"
    PROCESSED_TOPIC = "/sensors/imu/processed"

    def __init__(self):
        super().__init__("imu_control_signals_processor")

        self.imuListener = self.create_subscription(
            Imu, self.RAW_TOPIC, self._onReceiveIMU, 10
        )

        self.imuPublisher = self.create_publisher(State, self.PROCESSED_TOPIC, 10)

        self.prev_time = self.get_clock().now()
        self.curr_time = self.get_clock().now()

        self.vx = 0
        self.vy = 0
        self.vz = 0

        self.xx = 0
        self.xy = 0
        self.xz = 0

        self.ang_vx_prev = 0
        self.ang_vy_prev = 0
        self.ang_vz_prev = 0
        """
        REMEMBER TO SWAP ROLL AND PITCH
        """

    def _onReceiveIMU(self, msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        self.curr_time = self.get_clock().now()
        dt = (self.curr_time - self.prev_time).nanoseconds / 1000000000
        self.prev_time = self.curr_time

        self.vx += ax * dt
        self.vy += ay * dt
        self.vz += az * dt

        self.xx += self.vx * dt
        self.xy += self.vy * dt
        self.xz += self.vz * dt

        # Swap roll and pitch axis due to vehicle forward being defined as +y
        ang_vx = msg.angular_velocity.y
        ang_vy = msg.angular_velocity.x
        ang_vz = msg.angular_velocity.z

        ang_ax = (ang_vx - self.ang_vx_prev) / dt
        ang_ay = (ang_vy - self.ang_vy_prev) / dt
        ang_az = (ang_vz - self.ang_vz_prev) / dt

        self.ang_vx_prev = ang_vx
        self.ang_vy_prev = ang_vy
        self.ang_vz_prev = ang_vz

        state = State()

        state.linear_position.x = self.xx
        state.linear_position.y = self.xy
        state.linear_position.z = self.xz

        state.angular_position.x = msg.roll_pitch_yaw.y
        state.angular_position.y = msg.roll_pitch_yaw.x
        state.angular_position.z = msg.roll_pitch_yaw.z

        state.linear_velocity.x = self.vx
        state.linear_velocity.y = self.vy
        state.linear_velocity.z = self.vz

        state.angular_velocity.x = ang_vx
        state.angular_velocity.y = ang_vy
        state.angular_velocity.z = ang_vz

        state.linear_acceleration.x = ax
        state.linear_acceleration.y = ay
        state.linear_acceleration.z = az

        state.angular_acceleration.x = ang_ax
        state.angular_acceleration.y = ang_ay
        state.angular_acceleration.z = ang_az

        self.imuPublisher.publish(state)


def main(args=None):
    rclpy.init(args=args)
    IMUProcessor = IMUIntegration()

    try:
        rclpy.spin(IMUProcessor)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
