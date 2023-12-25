class PIDController():
    def __init__(self,kp=1.0,ki=1.0,kd=1.0,Ts=0.01, limMin=0.0, limMax=100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.Ts = Ts

        self.proportional = 0
        self.integral = 0
        self.derivative = 0

        self.limMax = limMax
        self.limMin = limMin    
        self.prev_error = 0



    def control(self, error):

        self.proportional = self.kp*error

        self.integral = self.integral + self.ki*self.Ts*(error)

        if (self.integral>self.limMax):
            self.integral = self.limMax
        elif (self.integral<self.limMin):
            self.integral = self.limMin

        self.derivative = self.kd * (error-self.prev_error)/self.Ts

        u = self.proportional + self.integral + self.derivative

        if (u>self.limMax):
            u = self.limMax
        elif (u<self.limMin):
            u = self.limMin

        self.prev_error = error

        return u