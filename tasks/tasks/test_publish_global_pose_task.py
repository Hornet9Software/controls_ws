import rclpy
from tasks.movement_tasks import MoveToPoseGlobalTask


def main(args=None):
    rclpy.init(args=args)
    test = MoveToPoseGlobalTask(1.,2.,3.,4.,5.,6.)
    test.execute(ud=[])
    rclpy.shutdown()

if __name__ == 'main':
    main()