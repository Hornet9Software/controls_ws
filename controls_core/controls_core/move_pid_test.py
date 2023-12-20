import rclpy
from driver import Driver

upthrust = -1


def moveLeft():
    rclpy.init(args=None)
    driver = Driver()
    driver.drive(
        [-1, 0, upthrust],
        [0, 0, 0],
    )
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()


def moveRight():
    rclpy.init(args=None)
    driver = Driver()
    driver.drive(
        [1, 0, -upthrust],
        [0, 0, 0],
    )
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()


def moveFront():
    rclpy.init(args=None)
    driver = Driver()
    driver.drive(
        [0, 1, -upthrust],
        [0, 0, 0],
    )
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()


def moveBack():
    rclpy.init(args=None)
    driver = Driver()
    driver.drive(
        [0, -1, -upthrust],
        [0, 0, 0],
    )
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()
