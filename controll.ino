#include <Wire.h>
#include <Zumo32U4.h>

// zumo classes
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4IMU imu;

// time variables
#define SAMPLERATE 10          // 10 millis =  100 Hz
unsigned long lastMillis = 0;
unsigned long lastMicros = 0;
float dt_time = SAMPLERATE/1000.0;


// Odometry settings
#define GEAR_RATIO 51.45      // Motor gear ratio 100.37
#define WHEELS_DISTANCE 98    // Distance between tracks in mm
#define WHEEL_DIAMETER 38.5   // Wheels diameter measured 38.5
#define ENCODER_PPR 12        // Encoder pulses per revolution
#define GYRO_SCALE 0.07        // 70 mdps/LSB 

float encoder2dist = (WHEEL_DIAMETER*3.14/(ENCODER_PPR*GEAR_RATIO)) / 1000.0;  // conversition of encoder pulses to distance in m

float theta = 0.0; 
float posx = 0.0;
float posy = 0.0;

// imu Fusion
float gyroAngle=0;
int32_t gyroOffset_z = -16;
float gyroz=0;
boolean motorsState = 0;


// controll variables:
float LeftMotorSpeed_Cntrl_CMD = 0.0, prevLeftMotorSpeed = 0.0;
float  leftWheelSpeed = 0.0, rightWheelSpeed = 0.0;
float RightMotorSpeed_Cntrl_CMD = 0.0, prevRightMotorSpeed = 0.0;
float desiredPosY = 0.0, desiredPosX = 0.0;

float integralRight = 0.0, integralLeft = 0.0;
float prevErrorLeft = 0.0, prevErrorRight = 0.0;
float cEr_left = 0.0, cEr_right = 0.0;
float w_last = 0.0;
float error_left = 0.0, error_right = 0.0;


float pidController(float error, float prevError, float &integral) {
  float Kp = 1500.0, Ki = 500.0 , Kd = 0.0, outMin = -400.0 , outMax = 400.0;

  float proportional = Kp * error;  // Calculate the proportional term
  integral += Ki * error * dt_time;  // Calculate the integral term
  float derivative = Kd * (error - prevError) / dt_time;  // Calculate the derivative term
  float output = proportional + integral + derivative;  // Calculate the output
  
  // Limit the output to the minimum and maximum values
  if (output > outMax) {
    output = outMax;
  } else if (output < outMin) {
    output = outMin;
  }
  
  return output;  // Return the output
}

void P2P_CTRL(float X_Desired, float Y_Desired, float &Left_Motor, float &Right_Motor) {
    
    float Vr[2] = {cos(theta), sin(theta)};  // car direction vector
    float Vt[2] = {X_Desired-posx, Y_Desired-posy}; // car direction vector

    float dir = (Vt[1]*Vr[0] - Vt[0]*Vr[1] > 0) ? 1 : -1; //cross(Vr,Vt)
    float dotVector = (Vt[0]*Vr[0] + Vt[1]*Vr[1])/(sqrt(pow(Vt[0],2) + pow(Vt[1],2))*sqrt(pow(Vr[0],2) + pow(Vr[1],2)));
    float theta_t;
    
    // calculate the desired angle change for the target point
    if ((dir == 0) && (dotVector == -1)) {  // vectors align in reverse... singularity
        theta_t = 3.14/2.0;
    } else {
        theta_t = acos(dotVector) * dir / 1000;
    }

    float w_forward = sqrt(pow(Vt[0],2) + pow(Vt[1],2));

    // set limits on max speed command
//    float WheelRadius = WHEEL_DIAMETER/2/1000;  // [m]
    float w_Max = 0.2;  // max speed [m/s]
    

    if (w_forward > w_Max) {
        w_forward = w_Max;
    }
    
    w_last = w_forward;

    Left_Motor = w_forward + theta_t*2;
    Right_Motor = w_forward - theta_t*2;

    Serial.print("  Theta_t = ");
    Serial.print(theta_t);



}


// the setup function runs once when you press reset or power the board
void setup() {

  // init imu
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  
  // initialize serial:
  Serial.begin(115200);

  // update motors command to stop
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  delay(1);
  // take time stamp
  lastMillis = millis();
  lastMicros = micros();
  delay(1);
  // calculate gyro offset
//  gyroOffset(); // recalibration of gyro.
//  delay(1);
  
}

// the loop function runs over and over again foreverdoub
void loop() {
  // get desired position if available
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil(',');
    desiredPosX = float(inputString.toFloat()); // r(t)
    
    inputString = Serial.readStringUntil('\n');
    desiredPosY = float(inputString.toFloat()); // r(t)
 
  }

  desiredPosX = 1.0;
  desiredPosY = 0.0;

  if (millis() - lastMillis >= SAMPLERATE){
    lastMillis = millis();

    // calculate dt sample
    unsigned long dtMicros = micros()-lastMicros;
    lastMicros = micros();
    dt_time = float(dtMicros)/1000000.0;

   
    
    P2P_CTRL(desiredPosX, desiredPosY, LeftMotorSpeed_Cntrl_CMD, RightMotorSpeed_Cntrl_CMD);

//    desired_vel = 0.2
//    LeftMotorSpeed_Cntrl_CMD = desired_vel ;
//    RightMotorSpeed_Cntrl_CMD = desired_vel ;

    
    Serial.print("  P2P output: ");
    Serial.print(LeftMotorSpeed_Cntrl_CMD);
    Serial.print(", ");
    Serial.print(RightMotorSpeed_Cntrl_CMD);
    Serial.print(" ");
    odometry();
    
//    gyroIntegration();
    
    // calculate the error between <Wheel RadS = LeftMotorSpeed> and the output from P2P_CTRL = <LeftMotorSpeed_Cntrl_CMD>
    float errorLeft = LeftMotorSpeed_Cntrl_CMD - leftWheelSpeed;

    // calculate the error between <Wheel RadS = RightMotorSpeed> and the output from P2P_CTRL = <RightMotorSpeed_Cntrl_CMD>
    float errorRight = RightMotorSpeed_Cntrl_CMD - rightWheelSpeed;
    
    // insert error to pidController, insert the controll signal to the motors.
    cEr_left = pidController(errorLeft, prevErrorLeft, integralLeft);

    cEr_right = pidController(errorRight, prevErrorRight, integralRight); 
    

    motors.setLeftSpeed(cEr_left); // -400 --> 400
    motors.setRightSpeed(cEr_right);


    prevErrorLeft = errorLeft;
    prevErrorRight = errorRight;

    Serial.print(" cEr_right = ");
    Serial.print(cEr_right);
    Serial.print(" cEr_left = ");
    Serial.print(cEr_left);
//    
    
//    
    Serial.print(" Pos = (");
    Serial.print(posx);
    Serial.print(", ");
    Serial.print(posy);
    Serial.print(") theta= ");
    Serial.print(theta);
    
//    
//    Serial.print(" Desired pos = (");
//    Serial.print(desiredPosX);
//    Serial.print(", ");
//    
//    Serial.print(desiredPosY);
//    Serial.print(")");
  }
  

    motorsState = (cEr_left || cEr_right) ==  0 ? 0 : 1; //  check if motors are still, for gyro

}

//
//// gyroIntegration
//// calculate the gyro angle
//void gyroIntegration(void){
//  imu.readGyro();  // 
//  gyroz = ((float) (imu.g.z - (int16_t)gyroOffset_z))*GYRO_SCALE; 
//  if (motorsState) gyroAngle+=(gyroz*dt_time); // integrate when in motion to stop accumulation of error in rest.
//}
//
//// gyro calibration
//void gyroOffset(){
//  delay(1); // delay before starting gyro readings for offset
//  int32_t total = 0;
//  for (uint16_t i = 0; i < 1024; i++) // calibration of the gyro - measuring the bias by assuming the result should be 0.
//  {
//    // Wait for new data to be available, then read it.
//    while(!imu.gyroDataReady()) {}
//    imu.readGyro();
//
//    // Add the Z axis reading to the total.
//    total += imu.g.z;
//  }
//  gyroOffset_z = total / 1024;
//  Serial.println(gyroOffset_z);
//}



void odometry(){
    //encoder read
    int16_t countsLeft = encoders.getCountsAndResetLeft();
    int16_t countsRight = encoders.getCountsAndResetRight();
    
    
    float dx_1 = countsRight*encoder2dist; // [m]
    float dx_2 = countsLeft*encoder2dist; // [m]
    
    float d_theta = float(dx_1-dx_2)/(WHEELS_DISTANCE/1000.0);
    posx += cos(theta+d_theta/2)*(dx_1+dx_2)/2;
    posy += sin(theta+d_theta/2)*(dx_1+dx_2)/2;
    theta += d_theta;

    

    leftWheelSpeed = float(dx_2 / dt_time); 
    rightWheelSpeed = float(dx_1 / dt_time); 
  

    
    
    Serial.print("Speed: ");
    Serial.print(leftWheelSpeed);
    Serial.print(", ");
    Serial.println(rightWheelSpeed);
    
}
