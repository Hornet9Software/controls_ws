import rclpy
from rclpy.node import Node
from custom_msgs.msg import ThrusterSpeeds

class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")
        self.publisher = self.create_publisher(ThrusterSpeeds, 'speeds', 10)
        self.timer = self.create_timer(1.0, self.publisher_callback)

    def publisher_callback(self):
        msg = ThrusterSpeeds()
        msg.header.frame_id = "hello"
        msg.speeds[0] = 25

        self.publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    test = TestNode()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()