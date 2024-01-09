import rclpy
import math
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray


class CVControlSignals(Node):
    OBJECTS = ["gate"]
    OBJECT_DIMENSIONS = {"gate": (1.5, 1.0)}

    def __init__(self):
        super().__init__("cv_control_signals_processor")
        self.objectPublishers = {}
        self.listeners = {}

        """
        TODO Change to dynamically read parameters from yaml file.
        Should read the following details:
        - Image size
        - Camera matrix
        -
        """
        self.IMAGE_WIDTH_PIXELS = 1.0
        self.IMAGE_HEIGHT_PIXELS = 1.0
        self.IMAGE_CENTROID = (
            self.IMAGE_WIDTH_PIXELS / 2.0,
            self.IMAGE_HEIGHT_PIXELS / 2.0,
        )
        self.HFOV = math.radians(50.7)
        self.VFOV = math.radians(37.7)

        for objectName in self.OBJECTS:
            self.listeners[objectName] = self.create_subscription(
                Float32MultiArray,
                "/object/" + objectName + "/yolo",
                lambda msg: self._onReceiveYOLO(objectName, msg),
                10,
            )
            self.objectPublishers[objectName] = {}
            for signal in ["bearing", "lateral", "distance"]:
                topic = "/object/" + objectName + "/" + signal
                self.objectPublishers[objectName][signal] = self.create_publisher(
                    Float32, topic, 10
                )

    def _onReceiveYOLO(self, objectName, msg):
        objectWidth = self.OBJECT_DIMENSIONS[objectName][0]
        objectHeight = self.OBJECT_DIMENSIONS[objectName][1]

        x_centre = msg.data[0]
        y_centre = msg.data[1]
        w = msg.data[2]
        h = msg.data[3]

        xMin = x_centre - w / 2.0
        xMax = x_centre + w / 2.0
        yMin = y_centre - h / 2.0
        yMax = y_centre + h / 2.0

        print("XMIN XMAX YMIN YMAX: ", xMin, xMax, yMin, yMax)

        objectWidthPixels = xMax - xMin
        objectHeightPixels = yMax - yMin
        objectCentroid = ((xMin + xMax) / 2.0, (yMin + yMax) / 2.0)

        # assuming AUV is perfectly centred vertically
        metrePerPixelVertical = objectHeight / objectHeightPixels

        imageHeight = metrePerPixelVertical * self.IMAGE_HEIGHT_PIXELS

        print("IMAGE HEIGHT: ", imageHeight)

        # VARIABLES
        # D: perpendicular distance to plane of projection of gate

        D = imageHeight / (2.0 * math.tan(self.VFOV / 2.0))

        print("D: ", D)

        metresPerPixelHorizontal = (
            2.0 * D * math.tan(self.HFOV / 2.0) / self.IMAGE_WIDTH_PIXELS
        )

        print("METRES PER PIXEL HORIZONTAL: ", metresPerPixelHorizontal)

        deltaXPixels = objectCentroid[0] - self.IMAGE_CENTROID[0]
        deltaX = metresPerPixelHorizontal * deltaXPixels

        objectOnLeft = False
        if deltaX < 0:
            objectOnLeft = True
            deltaX *= -1

        bearing = math.atan2(deltaX, D)

        if objectOnLeft:
            bearing += math.pi / 2.0
        else:
            bearing = (math.pi / 2) - bearing

        print("BEARING: ", bearing)

        distance = math.hypot(D, deltaX)

        print("DISTANCE: ", distance)

        projectedWidth = metresPerPixelHorizontal * objectWidthPixels
        print("PROJECTED WIDTH: ", projectedWidth)
        print("\n============\n")

        lateral = math.acos(min(projectedWidth, objectWidth) / objectWidth)

        for signal in ["distance", "lateral", "bearing"]:
            outMsg = Float32()
            outMsg.data = eval(signal)
            self.objectPublishers[objectName][signal].publish(outMsg)


def main(args=None):
    rclpy.init(args=args)
    cvProcessor = CVControlSignals()

    try:
        rclpy.spin(cvProcessor)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
