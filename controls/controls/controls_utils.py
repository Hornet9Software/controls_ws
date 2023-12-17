from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovariance, Point, PointStamped, Twist, TwistWithCovariance, Voctor3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import tf2_geometry_msgs
import tf2_ros
from tf_transformations import euler_from_quaternion
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.logging import get_logger
from typing import Union, List, Dict


def get_axes():
    return ['x','y','z','roll','pitch','yaw']

def get_controls_move_topic(axis):
    return '/control_effort/' + axis

def transform(origin_frame: str, dest_frame: str, poseORodom: Union[Pose, PoseStamped, Odometry]) -> Union[Pose, PoseStamped, Odometry]:
    tfBuffer = tf2_ros.Buffer()
    trans = tfBuffer.lookup_transform(dest_frame, origin_frame, Time(), Duration(seconds=0.5))

    if isinstance(poseORodom, Pose):
        transformed = tf2_geometry_msgs.do_transform_pose(poseORodom,trans)
        return transformed
    
    if isinstance(poseORodom, PoseStamped):
        transformed = tf2_geometry_msgs.do_transform_pose_stamped(poseORodom,trans)
        return transformed

    if isinstance(poseORodom, Odometry):
        temp_pose_stamped = PoseStamped()
        temp_pose_stamped.pose = poseORodom.pose.pose
        transform_pose_stamped = tf2_geometry_msgs.do_transform_pose_stamped(temp_pose_stamped, trans)

        # Twist as Points
        twist_poseORodom = poseORodom.twist.twist
        temp_twist_point_linear = Point()
        temp_twist_point_angular = Point()
        temp_twist_point_linear.x = twist_poseORodom.linear.x
        temp_twist_point_linear.y = twist_poseORodom.linear.y
        temp_twist_point_linear.z = twist_poseORodom.linear.z

        temp_twist_point_angular.x = twist_poseORodom.angular.x
        temp_twist_point_angular.y = twist_poseORodom.angular.y
        temp_twist_point_angular.z = twist_poseORodom.angular.z 

        twist_point_linear_stamped = PointStamped(point=temp_twist_point_linear)
        twist_point_angular_stamped = PointStamped(point=temp_twist_point_angular) 

        # transforming points
        transformed_twist_point_linear_stamped = tf2_geometry_msgs.do_transform_point(twist_point_linear_stamped, trans)
        transformed_twist_point_angular_stamped = tf2_geometry_msgs.do_transform_point(twist_point_angular_stamped, trans)    

        # converting points back to twist
        transformed_twist = Twist()
        transformed_twist.linear.x = transformed_twist_point_linear_stamped.point.x
        transformed_twist.linear.y = transformed_twist_point_linear_stamped.point.y
        transformed_twist.linear.z = transformed_twist_point_linear_stamped.point.z

        transformed_twist.angular.x = transformed_twist_point_angular_stamped.point.x
        transformed_twist.angular.y = transformed_twist_point_angular_stamped.point.y
        transformed_twist.angular.z = transformed_twist_point_angular_stamped.point.z

        transformed_odometry = Odometry(header = Header(frame_id = dest_frame),child_frame_id = dest_frame,
                                        pose=PoseWithCovariance(pose=transform_pose_stamped.pose),
                                        twist=TwistWithCovariance(twist=transformed_twist))
        
        return transformed_odometry
    
    get_logger.info("Invalid message type passed to transform()")
    return None


def _parse_pose(pose : Pose):
    """Converts a POSE message to a dictionary that maps direction to value.
    Does a convertion from quaternion to euler to convert orientation data to euler angles used by PID loops.

    
    Args:
        pose: geometry_msgs Pose

    Returns:
        Dictionary that maps direction to value for each axis in pose
    """
    
    pose_dict = {
        'x': pose.position.x,
        'y': pose.position.y,
        'z': pose.position.z,
    }

    # perform conversion from quaternion to euler angles

    pose_dict['roll'],pose_dict['pitch'],pose_dict['yaw'] = euler_from_quaternion(
        [pose.orientation.x,
         pose.orientation.y,
         pose.orientation.z,
         pose.orientation.w])
    
    return pose_dict

def _parse_twist(twist):
    """Converts a TWIST message to a dictionary that maps direction to value

    Args:
        twist: geometry_msgs Twist

    Returns:
        Dictionary that maps direction to value for each axis in twist.
    """
    twist_dict = {
        'x': twist.linear.x,
        'y': twist.linear.y,
        'z': twist.linear.z,
        'roll': twist.angular.x,
        'pitch': twist.angular.y,
        'yaw': twist.angular.z}
    return twist_dict

def _publish_data_dictionary(publishers, vals, indexes = get_axes()):
    for axis in indexes:
        publishers[axis].publish(vals[axis])

def _publish_data_constant(publishers, value, indexes = get_axes()):
    for axis in indexes:
        publishers[axis].publish(value)