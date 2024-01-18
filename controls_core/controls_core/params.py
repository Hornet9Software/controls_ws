from controls_core.PID import PID

rollPID = PID(Kp=25.0, Ki=0.0, Kd=5.0, sample_time=0.1)
yawPID = PID(Kp=25.0, Ki=0.0, Kd=5.0, sample_time=0.1)
distancePID = PID(Kp=5.0, Ki=0.0, Kd=5.0, sample_time=0.1)
lateralPID = PID(Kp=-5.0, Ki=0.0, Kd=-5.0, sample_time=0.1)
depthPID = PID(Kp=25.0, Ki=0.0, Kd=5.0, sample_time=0.1)

UPTHRUST = 1
IMU_ZERO = [-0.06439203646190189, -0.03839329625251654, 1.0272925130643187]

left_cam_dev = "/dev/video4"
right_cam_dev = "/dev/video2"
bottom_cam_dev = "/dev/video0"
# bottom_cam_dev = "DISABLE"
