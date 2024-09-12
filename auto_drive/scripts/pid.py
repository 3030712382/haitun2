# -*-encoding:utf-8-*-
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.now_error = 0
        self.last_error = 0
        self.sum_error = 0
        self.delta_t = 1
    def clear_error(self):
        self.now_error = 0
        self.last_error = 0
        self.sum_error = 0
    def PID_cal_angle(self,measured_value, target_value):
        self.now_error =  target_value - measured_value
        if self.now_error>180:
            self.now_error -= 360
        elif self.now_error<-180:
            self.now_error += 360
        delta_error = self.now_error - self.last_error
        self.last_error = self.now_error
        self.sum_error += self.now_error
        output = self.Kp * self.now_error + self.Ki * self.sum_error * self.delta_t + self.Kd * delta_error / self.delta_t
        return output
    def PID_Calculate(self, measured_value, target_value):
        now_error = target_value - measured_value
        delta_error = now_error - self.last_error
        self.last_error = now_error
        self.sum_error += now_error
        output = self.Kp * now_error + self.Ki * self.sum_error * self.delta_t + self.Kd * delta_error / self.delta_t
        return output

