class PID:
    def __init__(
        self,
        Kp=1.0,
        Ki=0.0,
        Kd=0.0,
        bias=0,
        integral_max=9999,
        sample_time=0.1,
    ):
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        self.bias = bias
        self.iMax = integral_max
        self.dt = sample_time

        self.error = 0
        self.integral = 0
        self.derivative = 0
        self.prevError = 0

        self.firstLoop = True

    def compute(self, setpoint, process_variable):
        self.error = setpoint - process_variable
        self.integral += self.error
        self.integral = min(max(self.integral, -self.iMax), self.iMax)
        self.integral *= self.dt

        self.derivative = 0 if self.firstLoop else (self.error - self.prevError)
        self.derivative /= self.dt
        self.prevError = self.error
        self.firstLoop = False

        return (
            (self.kp * self.error)
            + (self.ki * self.integral)
            + (self.kd * self.derivative)
            + self.bias
        )

