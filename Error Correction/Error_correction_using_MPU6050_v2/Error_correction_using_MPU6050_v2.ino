// Include libraries
#include <PID_v1.h>
#include <MPU6050.h>
#include<wire.h>

// Pin configuration

#define FR_FWD 1 // Front Right Wheel's Motor for Forward direction
#define FR_BWD 2 // Front Right Wheel's Motor for Backward direction
#define FL_FWD 3 // Front Left Wheel's Motor for Forward direction
#define FL_BWD 4 // Front Left Wheel's Motor for Backward direction
#define BR_FWD 5 // Back Right Wheel's Motor for Forward direction
#define BR_BWD 6 // Back Right Wheel's Motor for Backward direction
#define BL_FWD 7 // Back Left Wheel's Motor for Forward direction
#define BL_BWD 8 // Back Left Wheel's Motor for Backward direction
#define EN_FR 9 // Enable pin: to control speed of front right wheel
#define EN_FL 10 // Enable pin: to control speed of front left wheel
#define EN_BR 11 // Enable pin: to control speed of back right wheel
#define EN_BL 12 // Enable pin: to control speed of back left

// MPU6050 values
MPU6050 sensor;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float X_angle,Y_angle;
int minVal = 265;
int maxVal = 402;
//PID
double kp = 5, ki = 1, kd = 0.01, input = 0, output = 0, setpoint = 0; // modify kp, ki and kd for optimal performance
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);             // Creates a PID controller

void calc_pid(int *FR,int *FL,int *BR,int *BL)                         // function to apply PID and correct wheel speeds
{
    sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);                   // get accelerometer and gyroscope values from MPU6050
    int xAng = map(ax, minVal, maxVal, -90, 90);
    int yAng = map(ay, minVal, maxVal, -90, 90);
    int zAng = map(az, minVal, maxVal, -90, 90);
    X_angle = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
    Y_angle = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
    
    return;
}

int Speed_FR = 0, Speed_FL = 0, Speed_BR = 0, Speed_BL = 0,Speed_inp=0;
void setup() 
{
    pinMode(FR_FWD, OUTPUT);
    pinMode(FR_BWD, OUTPUT);
    pinMode(FL_FWD, OUTPUT);
    pinMode(FL_BWD, OUTPUT);
    pinMode(BR_FWD, OUTPUT);
    pinMode(BR_BWD, OUTPUT);
    pinMode(BL_FWD, OUTPUT);
    pinMode(BL_BWD, OUTPUT);
    pinMode(EN_FR, OUTPUT);
    pinMode(EN_FL, OUTPUT);
    pinMode(EN_BR, OUTPUT);
    pinMode(EN_BL, OUTPUT);

    Wire.begin();
    Serial.begin(9600);
    myPID.SetMode(AUTOMATIC);                                           // PID mode to AUTOMATIC
    Serial.println("Initializing the sensor");
    sensor.initialize();
    Serial.println(sensor.testConnection() ? "Successfully Connected" : "Connection failed");
    delay(1000);
    Serial.println("Taking Values from the sensor");
    delay(1000);
}

void loop() 
{
    while(Serial.available()>0)
    {
        Speed_inp = Serial.read();
        //Speed_FR = Serial.read();
        //Speed_FL = Serial.read();
        //Speed_BR = Serial.read();
        //Speed_BL = Serial.read();
        Speed_FR = Speed_FL = Speed_BR = Speed_BL = Speed_inp;
        calc_pid(&Speed_FR, &Speed_FL, &Speed_BR, &Speed_BL);
        analogWrite(EN_FR, Speed_FR);
        analogWrite(EN_FL, Speed_FL);
        analogWrite(EN_BR, Speed_BR);
        analogWrite(EN_BL, Speed_BL);
    }
    
}
