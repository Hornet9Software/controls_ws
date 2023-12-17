import rclpy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Pose, Twist
import controls.controls_utils as utils
from rclpy.node import Node

class PIDManager(Node):
    # Manages publishing to PID loops
    # pub dictionaries contain mappings from directions to their corresponding
    # publishers

    pub_pose = {}
    pub_pose_enable = {}
    pub_vel = {}
    pub_vel_enable = {}
    pub_control_effort = {}
    halted = True

    def __init__(self):
        super().__init__("PIDManager")
        for axis in utils.get_axes():
            # Create publishers
            self.pub_pose[axis] = self.create_publisher(Float64, self.get_pose_pid_topic(axis), 3)
            self.pub_pose_enable[axis] = self.create_publisher(Bool, self.get_pose_pid_enable_topic(axis), 3)
            self.pub_vel[axis] = self.create_publisher(Float64, self.get_vel_pid_topic(axis), 3)
            self.pub_vel_enable[axis] = self.create_publisher(Bool, self.get_vel_pid_enable_topic(axis), 3)
            self.pub_control_effort[axis] = self.create_publisher(Float64, utils.get_controls_move_topic(axis), 3)
    
    def soft_estop(self):
        # Disable all PID loops and publish 0 everywhere
        self.disable_loops()
        utils._publish_data_dictionary(self.pub_pose, utils._parse_pose(Pose()))
        utils._publish_data_dictionary(self.pub_vel, utils._parse_twist(Twist()))
        utils._publish_data_constant(self.pub_control_effort, 0.0)
        self.halted = True

    def disable_loops(self):
        # Disable
        utils._publish_data_constant(self.pub_pose_enable, False)
        utils._publish_data_constant(self.pub_vel_enable, False)
    
    def enable_loops(self):
        self.halted = False
        utils._publish_data_constant(self.pub_pose_enable, True)
        utils._publish_data_constant(self.pub_vel_enable, True)

    def get_pose_pid_topic(self, axis):
        return 'controls/' + axis + '_pos/setpoint'
    
    def get_vel_pid_topic(self, axis):
        return 'controls/' + axis + '_vel/setpoint'
    
    def get_pose_pid_enable_topic(self, axis):
        return 'controls/enable/' + axis + '_pos'
    
    def get_vel_pid_enable_topic(self, axis):
        return 'controls/enable/' + axis + '_vel'
    

    def position_control(self, pose : dict):
        # Enable PID loops
        # Position PID generate set-points for velocity loops, which in trun produce control efforts
        self.enable_loops()
        utils._publish_data_dictionary(self.pub_pose, pose)

    def velocity_control(self, twist):
        # Enable velocity control without position control.
        # Velocity PID loops produce control efforts based on desired twist
        self.enable_loops()
        utils._publish_data_constant(self.pub_pose_enable, False)
        utils._publish_data_dictionary(self.pub_vel, twist)