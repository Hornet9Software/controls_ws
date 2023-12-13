from cmath import nan
import numpy as np
import rclpy
import tf2_geometry_msgs
import tf2_ros
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import Vector3, Pose, PoseStamped, PoseWithCovariance, \
Twist, TwistWithCovariance, TwistStamped, Point, \
Quaternion, PointStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_multiply, quaternion_from_euler
from copy import deepcopy
from task import Task
from typing import Union, List, Dict
from std_msgs.msg import Header
from rclpy.logging import get_logger


def linear_distance(p1 : Point, p2 : Point) -> float:
    # Linear distance between 2 points
    v1 = np.array([p1.x, p1.y, p1.z])
    v2 = np.array([p2.x, p2.y, p2.z])

    # Vector math. Subtract 2 vectors and obtain the length of resulting vector
    distance = np.linalg.norm(v2 - v1)
    return distance


def angular_distance_quat(q1 : Quaternion, q2 : Quaternion) -> Vector3:
    # A quaternion consists of q=a+bi+cj+dk
    # Recall how a + bi can be used to represent points on the real and imaginary axes
    # A quaternion is simply another way to represent points in space, specifically 3D space
    # Find angular distance between 2 quaternions

    rpy1 = euler_from_quaternion([q1.x, q1.y, q1.z, q1.w])
    rpy2 = euler_from_quaternion([q2.x, q2.y, q2.z, q2.w])

    return angular_distance_rpy(rpy1, rpy2)


def angular_distance_rpy(rpy1 : tuple[float, float, float], rpy2 : tuple[float, float, float]) -> Vector3:
    # Find difference between 2 RPYs
    roll = np.fabs(rpy1[0] - rpy2[0])
    pitch = np.fabs(rpy1[1] - rpy2[1])
    yaw = np.fabs(rpy1[2] - rpy2[2])
    return Vector3(roll, pitch, yaw)


def reached_pose(current_pose : Pose, desired_pose : Pose, linear_tolerance = 0.1, angular_tolerance = 3) -> bool:
    # Compute linear distance between 2 points and check if it is within tolerance
    # Because we used vectors, sign does not matter!
    linear = linear_distance(current_pose.position, desired_pose.position) < linear_tolerance
    # Obtain RPY difference
    angular_dist = angular_distance_quat(current_pose.orientation, desired_pose.orientation)
    # Check if all of them are lesser than the tolerance
    angular = np.all(np.array([angular_dist.x, angular_dist.y, angular_dist.z]) < (np.ones(3) * angular_tolerance))
    return linear and angular


def at_desired_velocity(current_twist : Twist, desired_twist : Twist, linear_tolerance = 0.1, angular_tolerance = 0.3) -> bool:
    # Check if travelling at desired velocity
    linear_curr_velocity = np.linalg.norm([current_twist.linear.x, current_twist.linear.y, current_twist.linear.z])
    linear_des_velocity = np.linalg.norm([desired_twist.linear.x, desired_twist.linear.y, desired_twist.linear.z])
    linear = np.fabs(linear_curr_velocity - linear_des_velocity) < linear_tolerance

    angular_curr_velocity = np.linalg.norm([current_twist.angular.x, current_twist.angular.y, current_twist.angular.z])
    angular_des_velocity = np.linalg.norm([desired_twist.angular.x, desired_twist.angular.y, desired_twist.angular.z])
    angular = np.fabs(angular_curr_velocity - angular_des_velocity) < angular_tolerance
    return linear and angular


def stopped_at_pose(current_pose : Pose, desired_pose : Pose, current_twist : Twist) -> bool:
    at_desired_pose = reached_pose(current_pose, desired_pose, 0.2, 12)
    at_desired_vel = at_desired_velocity(current_twist, Twist(), 0.6, 6)

    return at_desired_pose and at_desired_vel


# Unsure what this does
def transform_pose(listener, base_frame, target_frame, pose : Pose):
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = base_frame

    return listener.transformPose(target_frame, pose_stamped).pose


def transform(origin_frame : str, dest_frame : str, poseORodom : Union[Pose, PoseStamped, Odometry]) -> Union[Pose, PoseStamped, Odometry]:
    # Transform poseORodom from origin_frame to dest_frame
    # Buffer stores and manages the history of coordinate frame transforms
    tfBuffer = tf2_ros.Buffer()
    # Use tfBuffer to look up the transformation from source to dest frame.
    # Transformation is essentially spacial relationship between frames
    # The translation/displacement between the frams and their orientation/rotation
    trans = tfBuffer.lookup_transform(dest_frame, origin_frame, Time(), Duration(seconds=0.5))

    if isinstance(poseORodom, Pose):
        # Perform the transformation
        # Apply a transformation on source frame pose gives us pose of dest frame
        transformed = tf2_geometry_msgs.do_transform_pose(poseORodom, trans)
        return transformed
    
    if isinstance(poseORodom, PoseStamped):
        transformed = tf2_geometry_msgs.do_transform_pose_stamped(poseORodom, trans)
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
        transformed_twise_point_linear_stamped = tf2_geometry_msgs.do_transform_point(twist_point_linear_stamped, trans)
        transformed_twise_point_angular_stamped = tf2_geometry_msgs.do_transform_point(twist_point_angular_stamped, trans)


        # converting points back to twist
        transformed_twist = Twist()
        transformed_twist.linear.x = transformed_twise_point_linear_stamped.point.x
        transformed_twist.linear.y = transformed_twise_point_linear_stamped.point.y
        transformed_twist.linear.z = transformed_twise_point_linear_stamped.point.z

        transformed_twist.angular.x = transformed_twise_point_angular_stamped.point.x
        transformed_twist.angular.y = transformed_twise_point_angular_stamped.point.y
        transformed_twist.angular.z = transformed_twise_point_angular_stamped.point.z
        
        transformed_odometry = Odometry(header=Header(frame_id=dest_frame), child_frame_id=dest_frame,
                                        pose=PoseWithCovariance(pose=transform_pose_stamped.pose),
                                        twist=TwistWithCovariance(twist=transformed_twist))
        
        return transformed_odometry
    
    get_logger.info("Invalid message type passed to transfor()")
    return None



def add_poses(pose_list : List[Pose]) -> Pose:
    # Add a list of poses
    p_sum = Point(0,0,0)
    q_sum = [0,0,0,1]

    for pose in pose_list:
        p_sum.x += pose.position.x
        p_sum.y += pose.position.y
        p_sum.z += pose.position.z

        q_sum = quaternion_multiply(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.z, pose.orientation.w], q_sum
        )

    return Pose(p_sum, Quaternion(*q_sum))

def parse_pose(pose : Pose) -> Dict:
    pose_dict = {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z}
    pose_dict['roll'], pose_dict['pitch'], pose_dict['yaw'] = euler_from_quaternion(
        [pose.orientation.x,
         pose.orientation.y,
         pose.orientation.z,
         pose.orientation.w]
    )

    return pose_dict

def cv_object_position(cv_obj_data):
    pass


class ObjectVisibleTask(Task):
    def __init__(self, image_name, timeout):
        super().__init__(task_name=image_name, outcomes=['undetected', 'detected'], input_keys=['image_name'], output_keys=['image_name'])

        self.image_name = image_name
        self.timeout = timeout

    def run(self, ud):
        milli_secs = 10
        rate = self.task_state.create_rate(milli_secs)
        total = 0
        while total < self.timeout * 1000:
            if cv_object_position(self.cv_data[self.image_name]) is not None:
                return 'detected'
            
            total += milli_secs
            rate.sleep()
        return 'undetected'
    
class MutableTask(Task):
    def __init__(self, mutablePose):
        super().__init__(task_name='mutable_task', outcomes=['done'], input_keys=['x', 'y', 'z', 'roll', 'pitch', 'yaw'])
        # super().__init__(task_name=name, outcomes=['done'], input_keys=['x','y','z','roll','pitch','yaw'])
        self.mutablePose = mutablePose

    def run(self, ud):
        if ud.x != nan and ud.y != nan and ud.z != nan:
            self.mutablePose.setPoseCoords(
                ud.x,
                ud.y,
                ud.z,
                ud.roll,
                ud.pitch,
                ud.yaw
            )

        # Checks if preemption request has been made for current state
        # Preemption refers to interuppting execution of current state in favor of another
        if self.preempt_requested():
            # Perform cleanup and transition to another state
            self.service_preempt()

        return 'done'
    


class MutablePose:
    def __init__(self):
        self.pose = None
    
    def setPoseCoords(self, x, y, z, roll, pitch, yaw):
        quat = Quaternion(*quaternion_from_euler(roll, pitch, yaw))
        point = Point(x,y,z)
        self.pose = Pose(point, quat)
    
    def setPose(self, newPose):
        self.pose = deepcopy(newPose)
    
    def getPose(self):
        return self.pose
    
    def getPoseEuler(self):
        return parse_pose(self.pose)