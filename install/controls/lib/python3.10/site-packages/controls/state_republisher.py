import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs
from tf2_ros import TransformListener, Buffer
# from rcply.qos import QoSProfile
import controls.controls_utils as utils

class StateRepublisher(Node):
    STATE_TOPIC = '/state'
    LOCAL_STATE_TOPIC = '/state_local'

    def __init__(self):
        super().__init__('state_republisher_node')
        
        # Dictionary of pose and twist publishers
        self._pub_pose = {} 
        self._pub_twist = {}

        # Create pose/twist publishers for each axis, storing them in their respective dicts
        for axis in utils.get_axes():
            self._pub_pose[axis] = self.create_publisher(Float64, self._get_pose_topic(axis),10)
            self._pub_pose[axis].publish(Float64(0)) # not sure why there a need to publish 0 when initialising state republisher
            self._pub_twist[axis] = self.create_publisher(Float64, self._get_twist_topic(axis), 10)
            self._pub_twist[axis].publish(Float64(0))

        # Create local state publisher
        self._pub_local_state = self.create_publisher(Odometry, self.LOCAL_STATE_TOPIC, 10)

        # Subscribe to global state
        self.create_subscription(Odometry, self.STATE_TOPIC, self._receive_odometry)

        # Data redistribution node, so just keep alive
        rclpy.spin(self)

    # Return pose/twist topics based on given axis
    def _get_pose_topic(self, axis):
        return '/controls/state/pose/' + axis
    
    def _get_twist_topic(self,axis):
        return 'controls/state/twist/' + axis
    
    # Global state callback
    def _receive_odometry(self, odometry):
        try:
            # transform_stamp = self.tf_buffer.lookup_transform('odom','base_link', Time(), Duration(seconds=0.5))
            # local_pose = tf2_geometry_msgs.do_transform_pose(odometry.pose.pose, transform_stamp.transform)
            # utils._publish_data_dictionary(self._pub_pose, self._parse_pose(local_pose))
            # local_twist = tf2_geometry_msgs.do_transform_twist(odometry.twist.twist, transform_stamp.transform)
            # self._publish_data_dictionary(self._pub_twist, self._parse_twist(local_twist))
            
            # Transform global state into local state
            local_odom = utils.transform('odom', 'base_link', odometry)
            local_pose = local_odom.pose.pose
            local_twist = local_odom.twist.twist
            utils._publish_data_dictionary(self._pub_pose, utils._parse_pose(local_pose))
            utils._publish_data_dictionary(self._pub_twist, utils._parse_twist(local_twist))

            local_state = Odometry()
            local_state.header.frame_id = 'base_link'
            local_state.pose.pose = local_pose
            local_state.twist.twist = local_twist
            self._pub_local_state.publish(local_state)

        except Exception as e:
            self.get_logger().error(f"Error transforming odometry: {str(e)}")