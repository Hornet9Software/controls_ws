import rclpy
from controls_core.attitude_control import AttitudeControl
from controls_core.params import cameraSteerPID, rollPID
from controls_core.thruster_allocator import ThrustAllocator
from rclpy.node import Node
from std_msgs.msg import Float32
from thrusters.thrusters import ThrusterControl

thrusterControl = ThrusterControl()
thrustAllocator = ThrustAllocator()


class Steer(Node):
    def __init__(self, topic, lin_acc_mag=1.0) -> None:
        super().__init__("steer_node")

        self.bearing_subscriber = self.create_subscription(
            Float32, topic, self.steer, 10
        )

        self.bearing = [0, 0, 0]
        self.linear_acc = [0, 0, lin_acc_mag]
        self.attitudeControl = AttitudeControl(rollPID=rollPID, yawPID=cameraSteerPID)

    def steer(self, msg: Float32):
        self.bearing[2] = msg.data

        attCorr = self.attitudeControl.getSteer(self.bearing)
        thrustValues = thrustAllocator.getThrustPWMs(self.linear_acc, attCorr)
        thrusterControl.setThrusters(thrustValues=thrustValues)


def main(args=None):
    rclpy.init(args=args)
    node = Steer(topic="/object/gate/bearing")

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        thrusterControl.killThrusters()
    finally:
        rclpy.try_shutdown()

    # from rclpy.signals import SignalHandlerOptions
    # while True:
    #     try:
    #         rclpy.spin_once(node, timeout_sec=0.1)
    #     except KeyboardInterrupt:
    #         node.kill()
    #         node.destroy_node()
    #         rclpy.shutdown()
    #         return
