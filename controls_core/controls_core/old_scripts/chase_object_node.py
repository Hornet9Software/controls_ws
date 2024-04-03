import numpy as np
from imu_msg.msg import Imu
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from thrusters.thrusters import ThrusterControl

from controls_core.attitude_control import AttitudeControl
from controls_core.params import *
from controls_core.position_control import PositionControl
from controls_core.thruster_allocator import ThrustAllocator

thrusterControl = ThrusterControl()
thrustAllocator = ThrustAllocator()
attitudeControl = AttitudeControl(rollPID, pitchPID, cameraSteerPID)
positionControl = PositionControl(distancePID, lateralPID, depthPID)


class ChaseObjectParams:
    def __init__(self, object_name, threshold_dist, linear_acc) -> None:
        self.object_name = object_name
        self.threshold_dist = threshold_dist
        self.linear_acc = linear_acc


class ChaseObjectNode(Node):
    def __init__(
        self,
        targetXYZ,
        targetRPY,
        chase_object_params: ChaseObjectParams,
        testRPYControl=False,
    ):
        super().__init__("depth_control_test")

        self.targetXYZ = targetXYZ
        self.targetRPY = targetRPY
        self.chase_object_params = chase_object_params
        self.testRPYControl = testRPYControl

        self.create_timer(0.1, self._controlLoop)

        self.currXYZ = [0.0, 0.0, 0.0]
        self.currRPY = [0.0, 0.0, 0.0]

        self.depthListener = self.create_subscription(
            Float32, "/sensors/depth", self._depthProcessing, 10
        )

        self.imuListener = self.create_subscription(
            Imu, "/sensors/imu/corrected", self._imuProcessing, 10
        )

        self.object_listener = self.create_subscription(
            Float32MultiArray,
            f"/object/{self.chase_object_params.object_name}/bearing_lateral_distance",
            lambda msg: self._on_receive_cv_data(
                msg, self.chase_object_params.object_name
            ),
            10,
        )

        self.cv_data = {self.chase_object_params.object_name: {}}

    def _on_receive_cv_data(self, msg: Float32MultiArray, objectName):
        msgData = np.array(msg.data).tolist()
        self.cv_data[objectName]["bearing"] = msgData[0]
        self.cv_data[objectName]["lateral"] = msgData[1]
        self.cv_data[objectName]["distance"] = msgData[2]

        self.targetRPY[2] = msgData[0]
        # self.get_logger().info(self.cv_data.__repr__())
        # self.get_logger().info(f"Target RPY changed to {self.targetRPY}")

    def _imuProcessing(self, msg: Imu):
        self.currRPY = [
            msg.roll_pitch_yaw.x,
            msg.roll_pitch_yaw.y,
            msg.roll_pitch_yaw.z,
        ]

    def _depthProcessing(self, msg: Float32):
        self.currXYZ[2] = msg.data

    def _controlLoop(self):
        if self.testRPYControl:
            # self.currRPY = attitudeControl.correctIMU(self.currRPY)
            self.currRPY[2] = 0.0
            angularAcc = attitudeControl.getAttitudeCorrection(
                currRPY=self.currRPY, targetRPY=self.targetRPY
            )
            # self.get_logger().info(f"PID angular acceleration: {angularAcc}")
        else:
            angularAcc = [0.0, 0.0, 0.0]

        linearAcc = positionControl.getPositionCorrection(
            currXYZ=self.currXYZ, targetXYZ=self.targetXYZ
        )

        # CV Control
        obj_data = self.cv_data[self.chase_object_params.object_name]

        if obj_data:
            curr_dist = obj_data["distance"]
            self.get_logger().info(f"Curr Distance: {curr_dist}")

            if (
                abs(attitudeControl.getAngleError(self.currRPY[2], self.targetRPY[2]))
                < 0.10
                or curr_dist < self.chase_object_params.threshold_dist
            ):

                linearAcc[1] = self.chase_object_params.linear_acc

                if curr_dist < self.chase_object_params.threshold_dist:
                    angularAcc[2] = 0.0

                # self.get_logger().info(f"Moving")
            else:
                linearAcc[1] = 0.0
                # self.get_logger().info(f"Stopped")

            # self.get_logger().info(f"Curr Depth: {self.currXYZ[2]}")
            # self.get_logger().info(f"Target Depth: {self.targetXYZ[2]}")
            # self.get_logger().info(
            #     f"Target RPY: {self.targetRPY}, Curr RPY: {self.currRPY}"
            # )
            # self.get_logger().info(f"Correction: {linearAcc}")

        # FL-FR-ML-MR-RL-RR
        thrustValues = thrustAllocator.getThrustPWMs(linearAcc, angularAcc)
        thrusterControl.setThrusters(thrustValues=thrustValues)
