from controls_core.PID import PID

rollPID = PID(Kp=25.0, Ki=0.0, Kd=5.0, sample_time=0.1)
pitchPID = PID(Kp=25.0, Ki=0.0, Kd=10.0, sample_time=0.1)
yawPID = PID(Kp=25.0, Ki=0.0, Kd=5.0, sample_time=0.1)

# pitchPID = PID(Kp=0.0, Ki=0.0, Kd=0.0, sample_time=0.1)

# Camera Steer PID
yawPID_camera = PID(Kp=25.0, Ki=0.0, Kd=20.0, sample_time=0.1)

distancePID = PID(Kp=5.0, Ki=0.0, Kd=5.0, sample_time=0.1)
lateralPID = PID(Kp=-5.0, Ki=0.0, Kd=-5.0, sample_time=0.1)
depthPID = PID(Kp=25.0, Ki=0.0, Kd=5.0, sample_time=0.1)

UPTHRUST = 1
IMU_ZERO = [-0.06480778207369386, 0.13629260216830133, 0.9820359710025255]

left_cam_dev = "/dev/video4"
bottom_cam_dev = "/dev/video2"
right_cam_dev = "/dev/video0"

weights_file = "front_yolov8n_300124_1.pt"
front_weights_file_path = (
    f"/home/bb/poolTest_ws/src/camera_ws/camera/camera/weights/{weights_file}"
)

# If "raw" is set to False, no feed at all
CAMERA_FLAGS = {
    "left": {"raw": True, "calibrated": True, "yolo": True},
    "right": {"raw": False, "calibrated": True, "yolo": True},
    "bottom": {"raw": False, "calibrated": True, "yolo": True},
}
