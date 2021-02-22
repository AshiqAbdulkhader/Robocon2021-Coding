import math
from scipy.interpolate import interp1d
from simple_pid import PID

# class PID:

    # def __init__(self, Input, Output, Setpoint, KP, KI, KD, DIRECT):
    #     self.input = Input
    #     self.output = Output
    #     self.setpoint = Setpoint
    #     self.kp = KP
    #     self.ki = KI
    #     self.kd = KD
    #     self.mode = DIRECT

    # def Compute(self):
    #     pass

# HELPER FUNCTIONS

def RAD_TO_DEG(angle):
    return (angle * 180)/math.pi

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)



# Pin configuration
FR_FWD = 1 # Front Right Wheel's Motor for Forward direction
FR_BWD = 2 # Front Right Wheel's Motor for Backward direction
FL_FWD = 3 # Front Left Wheel's Motor for Forward direction
FL_BWD = 4 # Front Left Wheel's Motor for Backward direction
BR_FWD = 5 # Back Right Wheel's Motor for Forward direction
BR_BWD = 6 # Back Right Wheel's Motor for Backward direction
BL_FWD = 7 # Back Left Wheel's Motor for Forward direction
BL_BWD = 8 # Back Left Wheel's Motor for Backward direction
EN_FR = 9 # Enable pin: to control speed of front right wheel
EN_FL = 10 # Enable pin: to control speed of front left wheel
EN_BR = 11 # Enable pin: to control speed of back right wheel
EN_BL = 12 # Enable pin: to control speed of back left

ax, ay, az = 0, 0, 0
gx, gy, gz = 0, 0, 0
X_angle, Y_angle = 0, 0
minVal = 265
maxVal = 402

# PID
# modify kp, ki and kd for optimal performance
kp = 5
ki = 1
kd = 0.01
Setpoint = 0;
myPID = PID(kp, ki, kd, Setpoint) # Creates a PID controller
Input = 0
Output = myPID(Input)


def calc_pid(FR, FL, BR, BL): # function to apply PID and correct wheel speeds
    print("\nEnter the 6 MPU outputs (3 accel, 3 gyro)")
    ax, ay, az, gx, gy, gz = map(float, input().split()) # get accelerometer and gyroscope values from MPU6050
    # change_limits = interp1d([minVal, maxVal], [-90, 90])
    xAng = translate(ax, minVal, maxVal, -90, 90)
    yAng = translate(ay, minVal, maxVal, -90, 90)
    zAng = translate(az, minVal, maxVal, -90, 90)
    X_angle = RAD_TO_DEG(math.atan2(-yAng, -zAng) + math.pi)
    Y_angle = RAD_TO_DEG(math.atan2(-xAng, -zAng) + math.pi)
    X_angle =translate(X_angle, 0, 360, -180, 180)
    Y_angle =translate(Y_angle, 0, 360, -180, 180)
    print("X_angle: ", X_angle)
    print("Y_angle: ", Y_angle)
    myPID.output_limits = (-90, 90)
    if X_angle == 0:
        return
    else:
        Input = X_angle
        Output = myPID(Input)
        # myPID.sample_time = 1
        if X_angle > 0: # right deviation
            if (FR + Output) < 255:
                FR += Output
            else:
                BR += Output
            return [FR, FL, BR, BL]
        elif X_angle < 0: # left deviation
            if (FL + Output) < 255:
                FL += Output
            else:
                BL += Output
            return [FR, FL, BR, BL]


Speed_FR = 0
Speed_FL = 0
Speed_BR = 0
Speed_BL = 0
Speed_inp = 0

def setup():
    # myPID.SetMode(AUTOMATIC) # PID mode to AUTOMATIC
    print("Initializing the sensor")
    # Serial.println(sensor.testConnection() ? "Successfully Connected" : "Connection failed")
    print("Taking Values from the sensor")


def loop():
    while(True):
        print("\nEnter the current speed:")
        Speed_inp = int(input())
        #Speed_FR = Serial.read();
        #Speed_FL = Serial.read();
        #Speed_BR = Serial.read();
        #Speed_BL = Serial.read();
        Speed_FR = Speed_FL = Speed_BR = Speed_BL = Speed_inp
        [Speed_FR, Speed_FL, Speed_BR, Speed_BL] = calc_pid(Speed_FR, Speed_FL, Speed_BR, Speed_BL)
        print("AnalogWrite to EN_FR: ", Speed_FR)
        print("AnalogWrite to EN_FL: ", Speed_FL)
        print("AnalogWrite to EN_BR: ", Speed_BR)
        print("AnalogWrite to EN_BL: ", Speed_BL)


if __name__ == "__main__":
    setup()
    loop()
