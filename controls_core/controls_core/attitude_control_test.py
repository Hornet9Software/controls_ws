import rclpy
from controls_core.thruster_allocator import ThrustAllocator
from controls_core.utilities import quat_to_list
from nav_msgs.msg import Odometry
from rclpy.node import Node
from simple_pid import PID
from tf_transformations import euler_from_quaternion
from thrusters.thrusters import ThrusterControl

thrusterControl = ThrusterControl()
thrustAllocator = ThrustAllocator()

# [Roll, Pitch, Yaw]
targetOrientation = [0, 0, 0]
ROLL_IDX = 0
YAW_IDX = 2


class AttitudeControlTestPublisher(Node):
    def __init__(self):
        super().__init__("attitude_control_test_publisher_node")
        self.state_subscriber = self.create_subscription(
            Odometry, "/state", self.attitudeControl, 10
        )

        self.rollPID = PID(Kp=1.0, Ki=0.0, Kd=0.0, setpoint=targetOrientation[ROLL_IDX])
        self.yawPID = PID(Kp=1.0, Ki=0.0, Kd=0.0, setpoint=targetOrientation[YAW_IDX])

    def attitudeControl(self, msg: Odometry):
        currAttQuat = msg.pose.pose.orientation
        currRoll, _, currYaw = euler_from_quaternion(quat_to_list(currAttQuat))
        rollAcc = self.rollPID(currRoll)
        yawAcc = self.yawPID(currYaw)

        angular_acc = [rollAcc, 0, yawAcc]
        self.get_logger().info(f"PID angular acceleration: {angular_acc}")
        # FL-FR-ML-MR-RL-RR
        thrustValues = thrustAllocator.getThrustPWMs([0, 0, 0], angular_acc)
        thrusterControl.setThrusters(thrustValues=thrustValues)


def main():
    rclpy.init(args=None)
    publisher = AttitudeControlTestPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()
