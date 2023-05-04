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
#define WHEELS_DISTANCE 98    // Distance between tracks
#define WHEEL_DIAMETER 37.5   // Wheels diameter measured 38.5
#define ENCODER_PPR 12        // Encoder pulses per revolution
#define GYRO_SCALE 0.07        // 70 mdps/LSB 

float encoder2dist = WHEEL_DIAMETER*3.14/(ENCODER_PPR*GEAR_RATIO);  // conversition of encoder pulses to distance in mm

float theta = 0; 
float posx = 0;
float posy = 0;

// imu Fusion
float gyroAngle=0;
int32_t gyroOffset_z = -16;
float gyroz=0;
boolean motorsState = 0;


// controll variables:
double cntrlCMDLeft = 0.0, prevLeftMotorSpeed = 0.0;
double cntrlCMDRight = 0.0, prevLeftMotorSpeed = 0.0;
double W_Left = 0.0, W_Right = 0.0;


double errorLeft = 0.0, errorRight = 0.0, cEr_left = 0.0, cEr_right = 0.0;

double integralRight = 0.0, integralLeft = 0.0;

float LowPassFilter(float currentReading, float previousReading, float alpha) {
  float filteredReading = alpha * currentReading + (1 - alpha) * previousReading;
  return filteredReading;
}


double pidControllerLeft(double error, double dt, double prevError, double &integral) {
  double prev_error = 0.0, Kp = 5.0, Ki = 0.1 , Kd = 0.5, outMin = -200.0 , outMax = 200.0;


  double proportional = Kp * error;  // Calculate the proportional term
  integral += Ki * error * dt;  // Calculate the integral term
  double derivative = Kd * (error - prevError) / dt;  // Calculate the derivative term
  double output = proportional + integral + derivative;  // Calculate the output
  
  // Limit the output to the minimum and maximum values
  if (output > outMax) {
    output = outMax;
  } else if (output < outMin) {
    output = outMin;
  }
  
  return output;  // Return the output
}

double pidControllerRight(double error, double dt, double prevError, double &integral) {
  double prev_error = 0.0, Kp = 5.0, Ki = 0.1 , Kd = 0.5, outMin = -200.0 , outMax = 200.0;


  double proportional = Kp * error;  // Calculate the proportional term
  integral += Ki * error * dt;  // Calculate the integral term
  double derivative = Kd * (error - prevError) / dt;  // Calculate the derivative term
  double output = proportional + integral + derivative;  // Calculate the output
  
  // Limit the output to the minimum and maximum values
  if (output > outMax) {
    output = outMax;
  } else if (output < outMin) {
    output = outMin;
  }
  
  return output;  // Return the output
}


void P2P_CTRL(double X_Desired, double Y_Desired, double &Left_Motor, double &Right_Motor) {
    float Vr[] = {cos(PHI), sin(PHI)};  // car direction vector
    float Vt[] = {X_Desired-POS_X, Y_Desired-POS_Y}; // car direction vector

    float dir = (Vt[1]*Vr[0] - Vt[0]*Vr[1] > 0) ? 1 : -1; //cross(Vr,Vt)
    float dotVector = (Vt[0]*Vr[0] + Vt[1]*Vr[1])/(sqrt(pow(Vt[0],2) + pow(Vt[1],2))*sqrt(pow(Vr[0],2) + pow(Vr[1],2)));

    // calculate the desired angle change for the target point
    if ((dir == 0) && (dotVector == -1)) {  // vectors align in reverse... singularity
        theta_t = PI/2;
    } else {
        theta_t = acos(dotVector) * dir;
    }

    float w_forward = sqrt(pow(Vt[0],2) + pow(Vt[1],2));

    // set limits on max speed command
    float WheelRadius = WHEEL_DIAMETER/2/1000;  // [m]
    float v_Max = 1;  // [m/s]
    float w_Max = v_Max/(WheelRadius*2*PI);  // [rad/sec^2] V=2*pi*r*w (w -  rotation/sec)

    if (w_forward > w_Max) {
        w_forward = w_Max;
    }

    Left_Motor = w_forward + theta_t*2;
    Right_Motor = w_forward - theta_t*2;




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
  Serial.begin(9600);

  // update motors command to stop
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  delay(1);
  // take time stamp
  lastMillis = millis();
  lastMicros = micros();
  
  // calculate gyro offset
  gyroOffset(); // recalibration of gyro.
  
}

// the loop function runs over and over again forever
void loop() {
  // get desired position if available
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil(', ');
    double desiredPosX = (float)inputString.toFloat(); // r(t)
    
    inputString = Serial.readStringUntil('\n');
    double desiredPosY = (float)inputString.toFloat(); // r(t)
  }

    

  if (millis() - lastMillis >= SAMPLERATE){
    lastMillis = millis();

    // calculate dt sample
    unsigned long dtMicros = micros()-lastMicros;
    lastMicros = micros();
    dt_time = float(dtMicros)/1000000.0;

    // read imu, calculate rotation angle

    // gets desired posx posy, curren pos x pos y, theta  (globals), edits LeftMotorSpeed_Cntrl_CMD and RightMotorSpeed_Cntrl_CMD.
    P2P_CTRL(desiredPosX, desiredPosY, LeftMotorSpeed_Cntrl_CMD, RightMotorSpeed_Cntrl_CMD);




    // calculate the error between <Wheel RadS = LeftMotorSpeed> and the output from P2P_CTRL = <LeftMotorSpeed_Cntrl_CMD>
    double errorLeft = LeftMotorSpeed_Cntrl_CMD - LeftMotorSpeed;

    // calculate the error between <Wheel RadS = RightMotorSpeed> and the output from P2P_CTRL = <RightMotorSpeed_Cntrl_CMD>
    double errorRight = RightMotorSpeed_Cntrl_CMD - RightMotorSpeed;
    
    // insert error to pidController, insert the controll signal to the motors.
    cEr_left = pidController(errorLeft, dt_time, prevError1, integralLeft);

    cEr_right = pidController(errorRight, dt_time, prevError1, integralRight); 
    

    motors.setLeftSpeed(cEr_left);
    motors.setRightSpeed(cEr_right);


    // insert <Wheel RadS> to Encoder position estimation - this is odometry
    gyroIntegration();
    odometry();


    prevErrorLeft = errorLeft;
    prevErrorRight = errorRight;


  }
  
    Serial.print("Control1 = ");
    Serial.print(controlsignal_1);
    Serial.print(" Control2 = ");
    Serial.print(controlsignal_2);
    Serial.print("Control3 = ");
    Serial.print(controlsignal_3);
    Serial.print(" Control4 = ");
    Serial.print(controlsignal_4);
    
    Serial.print(" Pos = (");
    Serial.print(posx);
    Serial.print(", ");
    Serial.print(posy);
    Serial.print(") ");
    
    Serial.print(" Desired pos = (");
    Serial.print(desiredPosX);
    Serial.print(", ");
    
    Serial.print(desiredPosY);
    Serial.println(")");
    





    motorsState = (controlsignal_1 || controlsignal_2) ==  0 ? 0 : 1; //  check if motors are still, for gyro

}

// gyroIntegration
// calculate the gyro angle
void gyroIntegration(void){
  imu.readGyro();  // 
  gyroz = ((float) (imu.g.z - (int16_t)gyroOffset_z))*GYRO_SCALE; 
  if (motorsState) gyroAngle+=(gyroz*dt_time); // integrate when in motion to stop accumulation of error in rest.
}

// gyro calibration
void gyroOffset(){
  delay(1); // delay before starting gyro readings for offset
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++) // calibration of the gyro - measuring the bias by assuming the result should be 0.
  {
    // Wait for new data to be available, then read it.
    while(!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  gyroOffset_z = total / 1024;
  Serial.println(gyroOffset_z);
}



void odometry(void){
    //encoder read
    int16_t countsLeft = encoders.getCountsAndResetLeft();
    int16_t countsRight = encoders.getCountsAndResetRight();
    float dx_1 = countsRight*encoder2dist;
    float dx_2 = countsLeft*encoder2dist;
    float d_theta = float(dx_1-dx_2)/WHEELS_DISTANCE;
    posx += cos(theta+d_theta/2)*(dx_1+dx_2)/2;
    posy += sin(theta+d_theta/2)*(dx_1+dx_2)/2;
    theta += d_theta;
}
