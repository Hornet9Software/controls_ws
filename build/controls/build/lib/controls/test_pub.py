import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("pub_node")
        self.pub = self.create_publisher(String, '/string', 10)
    
    def say_hello(self, message):
        self.get_logger().info(message.data)
    
    def run(self, input):
        self.pub.publish(input)

def main():
    rclpy.init()
    pub = MinimalPublisher()
    string = String()
    string.data = "kon"
    pub.run(string)
    rclpy.shutdown()

if __name__ == 'main':
    main()