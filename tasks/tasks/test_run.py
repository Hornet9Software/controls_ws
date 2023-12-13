import rclpy
from tasks.movement_tasks import MoveToPoseGlobalTask
from tf2_ros import TransformListener

def main(args=None):
    rclpy.init(args=args)
    test = MoveToPoseGlobalTask(1.,2.,3.,4.,5.,6.)
    test.execute(ud=[])
    rclpy.shutdown()

if __name__ == 'main':
    main()