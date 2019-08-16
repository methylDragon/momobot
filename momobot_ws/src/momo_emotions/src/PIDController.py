import time

class PIDController:
    def __init__(self, Kp=0.25, Ki=0.0, Kd=0.0, anti_windup=10.0, cmd_freq=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # Set max integral correction per timestep
        self.anti_windup = anti_windup

        # Set delay between updates (seconds)
        self.cmd_freq = cmd_freq

        self.current_time = time.time()
        self.prev_time = self.current_time

        self.reset()


    def reset(self):
        self.setpoint = 0.0

        self.p_ = 0.0
        self.i_ = 0.0
        self.d_ = 0.0

        self.prev_error = 0.0

    def compute(self, setpoint, measured_value):
        ''' Compute PID correction wrt. measured_value - setpoint '''
        self.current_time = time.time()
        delta_time = self.current_time - self.prev_time

        if delta_time >= self.cmd_freq:
            self.setpoint = setpoint
            error = self.setpoint - measured_value

            delta_error = error - self.prev_error
            self.accumulated_error = error * delta_time

            # Limit the integration to prevent absolutely wrecking yourself
            if self.accumulated_error < -self.anti_windup:
                self.accumulated_error = -self.anti_windup
            if self.accumulated_error > self.anti_windup:
                self.accumulated_error = self.anti_windup

            self.i_ = self.i_ + self.accumulated_error
            self.d_ = delta_error / delta_time

            self.prev_error = error
            self.prev_time = self.current_time

            return self.Kp * error + self.Ki * self.i_ + self.Kd * self.d_

    def set_kp(self, kp):
        self.Kp = kp

    def set_ki(self, ki):
        self.Ki = ki

    def set_kd(self, kd):
        self.Kd = kd

    def set_anti_windup(self, anti_windup):
        self.anti_windup = anti_windup
