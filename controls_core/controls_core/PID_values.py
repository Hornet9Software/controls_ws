from controls_core.PID import PID

rollPID = PID(Kp=1.0, Ki=0.0, Kd=0.0, sample_time=0.1)
yawPID = PID(Kp=10.0, Ki=0.0, Kd=5.0, sample_time=0.1)
cameraSteerPID = PID(Kp=10.0, Ki=0.0, Kd=5.0, sample_time=0.1)
