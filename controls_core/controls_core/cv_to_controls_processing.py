#!/usr/bin/env python3

import rclpy
import math
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray


class CVToControlsSignals(Node):
    def __init__(self, objects=[]):
        super().__init__("cv_to_controls_signals_processor")
        self.objects = objects
        self.objectPublishers = {}
        self.listeners = {}

        self.IMAGE_WIDTH_PIXELS = 1.0
        self.IMAGE_HEIGHT_PIXELS = 1.0
        self.IMAGE_CENTROID = (
            self.IMAGE_WIDTH_PIXELS / 2.0,
            self.IMAGE_HEIGHT_PIXELS / 2.0,
        )
        self.OBJECT_DIMENSIONS = {"gate": (0.150, 0.100)}
        self.HFOV = math.radians(50.7)
        self.VFOV = math.radians(37.7)

        for objectName in objects:
            self.listeners[objectName] = self.create_subscription(
                Float32MultiArray,
                "/object/" + objectName + "/yolo",
                lambda msg: self.bbox_callback(objectName, msg),
                10,
            )
            self.objectPublishers[objectName] = {}
            for signal in ["bearing", "lateral", "distance"]:
                topic = "/object/" + objectName + "/" + signal
                self.objectPublishers[objectName][signal] = self.create_publisher(
                    Float32, topic, 10
                )

    def bbox_callback(self, objectName, msg):
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

        objectWidthPixels = xMax - xMin
        objectHeightPixels = yMax - yMin
        objectCentroid = ((xMin + xMax) / 2.0, (yMin + yMax) / 2.0)

        # assuming AUV is perfectly centred vertically
        metrePerPixelVertical = objectHeight / objectHeightPixels

        imageHeight = metrePerPixelVertical * self.IMAGE_HEIGHT_PIXELS

        # VARIABLES
        # D: perpendicular distance to plane of projection of gate

        D = imageHeight / (2.0 * math.tan(self.VFOV / 2.0))

        metresPerPixelHorizontal = (
            2.0 * D * math.tan(self.HFOV / 2.0) / self.IMAGE_WIDTH_PIXELS
        )

        deltaXPixels = objectCentroid[0] - self.IMAGE_CENTROID[0]
        deltaX = metresPerPixelHorizontal * deltaXPixels

        objectOnLeft = False
        if deltaX < 0:
            objectOnLeft = True
            deltaX *= -1

        bearing = math.atan2(deltaX, D)
        if objectOnLeft:
            bearing += math.pi / 2.0

        distance = math.hypot(D, deltaX)

        projectedWidth = metresPerPixelHorizontal * objectWidthPixels
        lateral = math.acos(projectedWidth / objectWidth)

        for signal in ["distance", "lateral", "bearing"]:
            outMsg = Float32()
            outMsg.data = eval(signal)
            self.objectPublishers[objectName][signal].publish(outMsg)


def main(args=None):
    rclpy.init(args=args)
    cvProcessor = CVToControlsSignals(objects=["gate"])

    try:
        rclpy.spin(cvProcessor)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
