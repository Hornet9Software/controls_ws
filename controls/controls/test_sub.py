import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
class MinimalSubscriber:
    def __init__(self):
        
        self.node = Node("sub")
        self.node.create_subscription(String, '/string', self.say_hello, 10)
        rclpy.spin(self.node)
    
    def say_hello(self, message):
        output = message.data
        self.node.get_logger().info(output)
    
    def run(self):
        rate = self.node.create_rate(15)
        while rclpy.ok():
            print("hello from run")
            rate.sleep()

def main():
    rclpy.init()
    sub = MinimalSubscriber()
    sub.run()
    rclpy.shutdown()

if __name__ == 'main':
    main()