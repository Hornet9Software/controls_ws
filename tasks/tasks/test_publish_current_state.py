import rclpy
from rclpy.node import Node
from tasks.movement_tasks import MoveToPoseGlobalTask
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion
from tf_transformations import quaternion_from_euler



def main(args=None):
    rclpy.init(args=args)
    node = Node('haha')
    desired_state = Odometry()
    desired_state.pose.pose.position.x = 1.0
    desired_state.pose.pose.position.y = 2.0
    desired_state.pose.pose.position.z = 3.0
    desired_orientation = quaternion_from_euler(4.0,5.0,6.0)
    desired_state.pose.pose.orientation.x = desired_orientation[0]
    desired_state.pose.pose.orientation.y = desired_orientation[1]
    desired_state.pose.pose.orientation.z = desired_orientation[2]
    desired_state.pose.pose.orientation.w = desired_orientation[3]
    state_pub = node.create_publisher(Odometry, '/state', 10)
    state_pub.publish(desired_state)
    rclpy.shutdown()

if __name__ == 'main':
    main()