import controls.controls_utils as utils
import rclpy
from controls.pid_manager import PIDManager
from geometry_msgs.msg import Pose, Twist
from rclpy.logging import get_logger
from rclpy.node import Node


class DesiredStateHandler(Node):
    # Receive desired states and run control algos.
    DESIRED_POSE_TOPIC = "controls/desired_pose"
    DESIRED_TWIST_TOPIC = "controls/desired_twist"

    # TO-DO: rate.sleep() doesnt work well. Update to use time.sleep()
    REFRESH_RATE = 15  # Hz

    # TO-DO: Initialize these if needed
    MAX_POWER = {}
    MAX_TWIST = {}

    # Variables are dictionaries that map directions to their value (local reference frame)
    pose = None
    twist = None
    event_id = 0

    def __init__(self):
        super().__init__("DesiredStateHandler")
        # Publisher no need to spin
        self.pid_manager = PIDManager()

        self.create_subscription(Pose, self.DESIRED_POSE_TOPIC, self._on_pose_received)
        self.create_subscription(
            Twist, self.DESIRED_TWIST_TOPIC, self._on_twist_receive
        )

    def _on_pose_received(self, pose):
        # Convert from global frame to local frame
        self.pose = utils._parse_pose(utils.transform("odom", "base_link", pose))

    def _on_twist_receive(self, twist):
        # Twist received in local frame
        self.twist = utils._parse_twist(twist)

    def _reset_data(self):
        # Reset all state data
        self.pose = None
        self.twist = None

    def _validate_status(self):
        # Validates desired state data that was received.
        # Returns false if multiple states are received (pose and twist)
        # or if no states received
        # If previous status was invalid and new status valid, increment event_id

        if self.pose and self.twist:
            self.pid_manager.soft_estop()
            get_logger().info("Conflicting desired state received. Halting...")
            return False
        # no states
        elif not self.pose and not self.twist:
            # If not explicitly stopped
            if not self.pid_manager.halted:
                get_logger.info(
                    f"No states receieved. Halting... Event {self.event_id}"
                )
            self.pid_manager.soft_estop()
            return False
        elif self.pid_manager.halted:
            get_logger.info(f"Received desired state. Event {self.event_id}")
            self.event_id += 1

        return True

    def run(self):
        rate = self.create_rate(self.REFRESH_RATE)
        while rclpy.ok():
            # Spin once for subscribers to receive info
            rclpy.spin_once(self)
            if self._validate_status():
                if self.pose:
                    self.pid_manager.position_control(self.pose)
                elif self.twist:
                    self.pid_manager.velocity_control(self.twist)
            rate.sleep()
            self._reset_data()
