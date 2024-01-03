#!/usr/bin/env python3

import math
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from scipy.spatial.transform import Rotation as R


class Simulation(Node):
    def __init__(self, linPos_0=[0.0, 0.0, 0.0], angPos_0=[0.0, 0.0, 0.0], dt=0.01):
        super().__init__("simulation")

        self.dt = dt

        # (x, y, z, 1) of left and right endpoint
        self.gate = np.array([[10, 10, 0.5, 1.0], [11.5, 10, 0.5, 1.0]])

        self.HFOV = math.radians(50.7)
        self.leftBoundaryTf = R.from_euler("z", -self.HFOV / 2)
        self.rightBoundaryTf = R.from_euler("z", self.HFOV / 2)

        self.linPos = np.array(linPos_0)
        self.angPos = np.array(angPos_0)

        self.linVel = np.array([0.0, 0.0, 0.0])
        self.angVel = np.array([0.0, 0.0, 0.0])

        self.linAcc = np.array([0.0, 0.0, 0.0])
        self.angAcc = np.array([0.0, 0.0, 0.0])

        self.accListener = self.create_subscription(
            Float32MultiArray, "/sim/acc", self._accCallback, 10
        )

        self.depthPub = self.create_publisher(Float32, "/sensors/depth", 10)
        self.yoloPub = self.create_publisher(Float32MultiArray, "/object/gate/yolo", 10)

        self.physicsClock = self.create_timer(self.dt, self.physicsLoop)

    def _accCallback(self, msg):
        self.linAcc = np.array(msg.data[:3])
        self.angAcc = np.array(msg.data[3:])

    # returns three unit vectors: left boundary of cone, centre of cone, right boundary of cone
    def getCone(self):
        centre = R.from_euler("z", self.angPos[2]).apply([1, 0, 0])
        left = self.leftBoundaryTf.apply(centre)
        right = self.rightBoundaryTf.apply(centre)

        return [left, centre, right]

    def physicsLoop(self):
        self.linVel += self.linAcc * self.dt
        self.angVel += self.angAcc * self.dt

        self.linPos += self.linVel * self.dt
        self.angPos += self.angPos * self.dt

        depth = Float32()
        depth.data = self.linPos[2]
        self.depthPub.publish(depth)

        left, centre, right = self.getCone()

        f = 1.0 / (2.0 * math.tan(self.HFOV / 2.0))
        cx = 0.5
        cy = 0.5

        P = np.array([[f, 0.0, cx], [0.0, f, cy], [0.0, 0.0, 1.0]])

        world_to_camera_rot = R.from_euler("xyz", self.angPos).as_matrix()
        world_to_camera_T = np.expand_dims(self.linPos, axis=1)
        lastRow = np.zeros((1, world_to_camera_rot.shape[1] + 1))
        lastRow[0, -1] = 1
        tf = np.concatenate((world_to_camera_rot, world_to_camera_T), axis=1)
        T = np.concatenate((tf, lastRow), axis=0)

        proj = np.matmul(P, T)
        pixels = np.matmul(proj, self.gate.T).T

        left = pixels[0]
        right = pixels[1]

        centroid = (left + right) / 2.0
        w = math.fabs(left[0] - right[0])
        h = math.fabs(left[1] - right[1])

        yoloBox = Float32MultiArray()
        yoloBox.data = [centroid[0], centroid[1], w, h]
        self.yoloPub.publish(yoloBox)


def main(args=None):
    rclpy.init(args=args)
    sim = Simulation()

    try:
        rclpy.spin(sim)
    except KeyboardInterrupt:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
