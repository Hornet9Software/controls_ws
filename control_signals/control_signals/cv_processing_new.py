import math

import cv2
import cv_bridge
import message_filters
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32, Float32MultiArray
from vision_msgs.msg import Detection2DArray


class CVControlSignals(Node):
    CAMERAS = ["bottom"]
    OBJECT_MAP = {
        0: "gate",
        1: "orange_flare",
        2: "blue_flare",
        3: "red_flare",
        4: "yellow_flare",
        5: "blue_drum",
        6: "red_drum",
    }

    OBJECT_DIMENSIONS = {
        "gate": (1.5, 1.0),
        "orange_flare": (0.15, 1.5),
        "red_flare": (0.15, 1.5),
        "blue_flare": (0.15, 1.5),
        "yellow_flare": (0.15, 1.5),
        "blue_drum": (0.15, 1.5),
        "red_drum": (0.15, 1.5),
    }
    IMAGE_WIDTH_PIXELS = 640
    IMAGE_HEIGHT_PIXELS = 480
    IMAGE_CENTROID = (IMAGE_HEIGHT_PIXELS / 2.0, IMAGE_HEIGHT_PIXELS / 2.0)
    QUEUE_SIZE = 10
    HFOV = math.radians(51.96292634)
    VFOV = math.radians(40.98386431)

    def __init__(self):
        super().__init__("cv_control_signals_processor")
        self.object_publishers = {}
        self.detection_listeners = {}

        self._bridge = cv_bridge.CvBridge()

        for camera in self.CAMERAS:
            self.detection_listeners[camera] = self.create_subscription(
                Detection2DArray,
                f"{camera}/rect/detections_output",
                lambda msg: self.detections_callback(camera, msg),
                10,
            )

        for object_name in self.OBJECT_MAP.values():
            #             if object_name == "orange_flare":
            #                 continue
            self.object_publishers[object_name] = self.create_publisher(
                Float32MultiArray,
                "/object/" + object_name + "/bearing_lateral_distance",
                10,
            )

    def detections_callback(self, camera, detections_msg):
        for detection in detections_msg.detections:
            x_centre = detection.bbox.center.position.x
            y_centre = detection.bbox.center.position.y
            w = detection.bbox.size_x
            h = detection.bbox.size_y

            object_name = self.OBJECT_MAP[int(detection.results[0].hypothesis.class_id)]
            #             if object_name == "orange_flare":
            #                 continue

            object_width = self.OBJECT_DIMENSIONS[object_name][0]
            object_height = self.OBJECT_DIMENSIONS[object_name][1]

            x_min = x_centre - w / 2.0
            x_max = x_centre + w / 2.0
            y_min = y_centre - h / 2.0
            y_max = y_centre + h / 2.0

            self.get_logger().info(
                "{} XMIN XMAX YMIN YMAX: {}, {}, {}, {}".format(
                    object_name, x_min, x_max, y_min, y_max
                )
            )

            object_width_pixels = x_max - x_min
            object_height_pixels = y_max - y_min
            object_centroid = (x_centre, y_centre)

            # assuming AUV is perfectly centred vertically
            metre_per_pixel_vertical = object_height / object_height_pixels

            image_height = metre_per_pixel_vertical * self.IMAGE_HEIGHT_PIXELS

            # D: perpendicular distance to plane of projection of gate

            D = image_height / (2.0 * math.tan(self.VFOV / 2.0))

            metres_per_pixel_horizontal = (
                2.0 * D * math.tan(self.HFOV / 2.0) / self.IMAGE_WIDTH_PIXELS
            )

            delta_x_pixels = object_centroid[0] - self.IMAGE_CENTROID[0]
            delta_x = metres_per_pixel_horizontal * delta_x_pixels

            object_on_left = False
            if delta_x < 0:
                object_on_left = True
            delta_x *= -1

            bearing = math.atan2(delta_x, D)

            distance = math.hypot(D, delta_x)

            projected_width = metres_per_pixel_horizontal * object_width_pixels

            lateral = math.acos(min(projected_width, object_width) / object_width)

            out_msg = Float32MultiArray()
            out_msg.data = [bearing, lateral, distance]
            self.object_publishers[object_name].publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    cv_processor = CVControlSignals()
    try:
        rclpy.spin(cv_processor)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
