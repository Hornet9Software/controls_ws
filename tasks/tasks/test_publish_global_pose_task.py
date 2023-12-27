import rclpy
from tasks.movement_tasks import Steer


def main(args=None):
    rclpy.init(args=args)
    test = Steer(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    test.execute(ud=[])
    rclpy.shutdown()


if __name__ == "main":
    main()
