import numpy as np
from controls_core.PID import PID

rollPID = PID(Kp=25.0, Ki=0.0, Kd=5.0, sample_time=0.1)
pitchPID = PID(Kp=25.0, Ki=0.0, Kd=10.0, sample_time=0.1)
yawPID = PID(Kp=35.0, Ki=0.0, Kd=5.0, sample_time=0.1)
cameraSteerPID = PID(Kp=10.0, Ki=0.0, Kd=5.0, sample_time=0.1)

# pitchPID = PID(Kp=0.0, Ki=0.0, Kd=0.0, sample_time=0.1)

distancePID = PID(Kp=5.0, Ki=0.0, Kd=5.0, sample_time=0.1)
lateralPID = PID(Kp=5.0, Ki=0.0, Kd=5.0, sample_time=0.1)
depthPID = PID(Kp=10.0, Ki=0.0, Kd=5.0, sample_time=0.1)


def bound_angle(angle):
    """
    Bound angle to [-pi, pi]
    """
    angle = angle % (2 * np.pi)
    if angle < -np.pi:
        return 2 * np.pi + angle
    elif angle > np.pi:
        return angle - 2 * np.pi
    else:
        return angle


def angle_error(curr_angle, target_angle):

    currAngle = bound_angle(curr_angle)
    targetAngle = bound_angle(target_angle)

    return bound_angle(currAngle - targetAngle)


def angle_abs_error(curr_angle, target_angle):
    return abs(angle_error(curr_angle, target_angle))


UPTHRUST = 1
IMU_ZERO = [-0.013709831, 0.016739033, 1.1965]

left_cam_dev = "/dev/video0"
bottom_cam_dev = "/dev/video2"
right_cam_dev = "/dev/video4"

weights_file = "front_yolov8n_300124_1.pt"
front_weights_file_path = (
    f"/home/bb/poolTest_ws/src/camera_ws/camera/camera/weights/{weights_file}"
)

# If "raw" is set to False, no feed at all
CAM_FLAGS = {
    "left": {"raw": True, "calibrated": False, "yolo": False, "dev": "/dev/video0"},
    "right": {"raw": False, "calibrated": False, "yolo": False, "dev": "/dev/video2"},
    "bottom": {"raw": False, "calibrated": False, "yolo": False, "dev": "/dev/video4"},
}

CAM_PARAMS = [
    {"width": 640},
    {"height": 480},
    {"codec": "unknown"},
    {"loop": 0},
    # {"latency": 2000},
    {"framerate": 30.0},
]
