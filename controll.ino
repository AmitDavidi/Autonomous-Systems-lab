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

// infinity points:

//float pointsX[40] =  {0.0, 0.03208225617155205, 0.0633335987602945, 0.09294463440875371, 0.12014845284759577, 0.1442404894687629, 0.16459677317873128, 0.18069008692207644, 0.19210362232627445, 0.1985417748196108, 0.19983779963431395, 0.19595813040845356, 0.18700324853708297, 0.17320508075688776, 0.15492099236553092, 0.1326245316481591, 0.10689316522556025, 0.07839332197201504, 0.04786313285751162, 0.016093313743345217, -0.016093313743345078, -0.04786313285751149, -0.07839332197201498, -0.10689316522556014, -0.132624531648159, -0.15492099236553084, -0.17320508075688767, -0.18700324853708294, -0.19595813040845356, -0.19983779963431392, -0.19854177481961083, -0.1921036223262745, -0.1806900869220765, -0.16459677317873134, -0.14424048946876294, -0.12014845284759593, -0.09294463440875383, -0.06333359876029461, -0.03208225617155213, -4.898587196589413e-17};
//float pointsY[40] =  {0.0, -0.0004050345323210414, -0.0029623154374641336, -0.008755371102152572, -0.017706515024622, -0.029156389950552055, -0.0423866006901744, -0.056837518566891655, -0.07211973225187653, -0.08794397723209986, -0.10402655977279358, -0.11997756354302339, -0.1351552823930171, -0.14846149779161807, -0.15806126685897606, -0.16106850013878748, -0.15341472258503872, -0.13046864767574942, -0.08922533766745615, -0.03192771407321133, 0.031927714073211065, 0.08922533766745591, 0.13046864767574934, 0.1534147225850387, 0.16106850013878746, 0.15806126685897612, 0.1484614977916181, 0.13515528239301716, 0.11997756354302341, 0.10402655977279361, 0.08794397723209996, 0.0721197322518766, 0.056837518566891704, 0.042386600690174435, 0.02915638995055208, 0.01770651502462206, 0.008755371102152607, 0.002962315437464144, 0.0004050345323210424, -0.0};

// circle points
//float pointsX[40] =  {0.15, 0.14805753939568692, 0.1422804662920718, 0.1328184038479815, 0.11991641451052518, 0.10390865302643992, 0.08520971200967338, 0.06430388421045813, 0.04173261958746791, 0.018080502038298484, -0.006039891016412235, -0.03000385406640665, -0.053190733056380314, -0.07499999999999997, -0.09486680633930657, -0.11227661222566512, -0.12677851283156918, -0.13799691654882362, -0.14564127261390777, -0.1495135962201315, -0.1495135962201315, -0.1456412726139078, -0.13799691654882362, -0.12677851283156924, -0.11227661222566519, -0.09486680633930665, -0.07500000000000007, -0.05319073305638038, -0.030003854066406682, -0.006039891016412271, 0.01808050203829835, 0.04173261958746781, 0.06430388421045806, 0.08520971200967334, 0.10390865302643988, 0.1199164145105251, 0.13281840384798144, 0.1422804662920718, 0.14805753939568692, 0.15};
//float pointsY[40] =  {0.0, 0.024061692128664037, 0.04750019907022087, 0.06970847580656528, 0.09011133963569683, 0.10818036710157217, 0.12344757988404845, 0.13551756519155733, 0.14407771674470582, 0.14890633111470808, 0.14987834972573544, 0.14696859780634014, 0.1402524364028122, 0.1299038105676658, 0.11619074427414819, 0.09946839873611933, 0.08016987391917019, 0.058794991479011266, 0.035897349643133715, 0.012069985307508913, -0.012069985307508808, -0.03589734964313361, -0.05879499147901124, -0.0801698739191701, -0.09946839873611925, -0.11619074427414812, -0.12990381056766576, -0.1402524364028122, -0.14696859780634014, -0.14987834972573544, -0.1489063311147081, -0.14407771674470587, -0.13551756519155736, -0.12344757988404848, -0.1081803671015722, -0.09011133963569694, -0.06970847580656538, -0.047500199070220954, -0.024061692128664092, -3.6739403974420595e-17};

// square points
//float pointsX[40] = {0.0, 0.05, 0.1, 0.15000000000000002, 0.2, 0.25, 0.30000000000000004, 0.35000000000000003, 0.4, 0.45, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.45, 0.4, 0.35000000000000003, 0.30000000000000004, 0.25, 0.2, 0.15000000000000002, 0.1, 0.05, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//float pointsY[40] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.05, 0.1, 0.15000000000000002, 0.2, 0.25, 0.30000000000000004, 0.35000000000000003, 0.4, 0.45, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.45, 0.4, 0.35000000000000003, 0.30000000000000004, 0.25, 0.2, 0.15000000000000002, 0.1, 0.05};

// Odometry settings
#define GEAR_RATIO 51.45      // Motor gear ratio 100.37
#define WHEELS_DISTANCE 98    // Distance between tracks in mm
#define WHEEL_DIAMETER 37.5   // Wheels diameter measured 38.5
#define ENCODER_PPR 12        // Encoder pulses per revolution
#define GYRO_SCALE 0.07        // 70 mdps/LSB 

float encoder2dist = (WHEEL_DIAMETER*3.14/(ENCODER_PPR*GEAR_RATIO)) / 1000.0;  // conversition of encoder pulses to distance in m

float theta = 0.0; 


// imu Fusion
float gyroAngle=0;
int32_t gyroOffset_z = -16;
float gyroz=0;
boolean motorsState = 0;

float pointsX[40] = {0.0, 0.05, 0.1, 0.15000000000000002, 0.2, 0.25, 0.30000000000000004, 0.35000000000000003, 0.4, 0.45, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.45, 0.4, 0.35000000000000003, 0.30000000000000004, 0.25, 0.2, 0.15000000000000002, 0.1, 0.05, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float pointsY[40] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.05, 0.1, 0.15000000000000002, 0.2, 0.25, 0.30000000000000004, 0.35000000000000003, 0.4, 0.45, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.45, 0.4, 0.35000000000000003, 0.30000000000000004, 0.25, 0.2, 0.15000000000000002, 0.1, 0.05};


//
int pointIdx = 1;
float posx;
float posy;

// controll variables:
float LeftMotorSpeed_Cntrl_CMD = 0.0, prevLeftMotorSpeed = 0.0;
float  leftWheelSpeed = 0.0, rightWheelSpeed = 0.0;
float lastrightWheelSpeed = 0.0, lastleftWheelSpeed = 0.0;
float RightMotorSpeed_Cntrl_CMD = 0.0, prevRightMotorSpeed = 0.0;
float desiredPosY = pointsY[1] , desiredPosX = pointsX[1];

float integralRight = 0.0, integralLeft = 0.0;
float prevErrorLeft = 0.0, prevErrorRight = 0.0;
float cEr_left = 0.0, cEr_right = 0.0;
float w_last = 0.0;
float error_left = 0.0, error_right = 0.0;
int switched = 0;

float pidController(float error, float prevError, float &integral, float Kp) {
  float Ki = 500.0 , Kd = 0.0, outMin = -400.0 , outMax = 400.0;

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

    float dir = (Vt[1]*Vr[0] - Vt[0]*Vr[1] > 0) ? 1.0 : -1.0; //cross(Vr,Vt)
    
    float dotVector = (Vt[0]*Vr[0] + Vt[1]*Vr[1])/(sqrt(pow(Vt[0],2) + pow(Vt[1],2))*sqrt(pow(Vr[0],2) + pow(Vr[1],2)));
    float theta_t = 0.0;
    
    // calculate the desired angle change for the target point
    if ((Vt[1]*Vr[0] - Vt[0]*Vr[1] - 0 < 0.001) && (dotVector == -1)) {  // vectors align in reverse... singularity
        theta_t = float(3.14/2.0);
    } else {
        theta_t = float(acos(dotVector) * dir);
    }

    float w_forward = sqrt(pow(Vt[0],2) + pow(Vt[1],2));
    
    
    // set limits on max speed command
//    float WheelRadius = WHEEL_DIAMETER/2/1000;  // [m]
    float w_Max = 0.4;  // max speed [m/s]
    

    if (w_forward > w_Max) {
        w_forward = float(w_Max);
    }

    if(w_forward - w_last > 0.1) {
      w_forward = w_last + 0.1;
    }
    w_last = w_forward;

    Left_Motor = (float(w_forward - theta_t*0.25));
    Right_Motor = float(w_forward + theta_t*0.25);
    
    if(Left_Motor != Left_Motor) Left_Motor = 0.02;
    if(Right_Motor != Right_Motor) Right_Motor = 0.02;
      


}
// low pass filtering function
double LowPassFilter(double currentReading, double previousReading, double alpha) {
  if(abs(currentReading - previousReading) > 60000) {
    currentReading = previousReading; // bad ENCODER_COUNTS read, overflow, skip this reading. (needed ?)
  }
  double filteredReading = double(alpha * currentReading + (1.0 - alpha) * previousReading);
  return filteredReading;
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

  theta = atan2(pointsY[1] - pointsY[0], pointsX[1] - pointsX[0]); // calc initial angle
  posx = pointsX[0];
  posy = pointsY[0];
  
  
  
}



// the loop function runs over and over again foreverSdoub
void loop() {
  
  

  if (millis() - lastMillis >= SAMPLERATE){

   
    
    if( sqrt(pow(posx - desiredPosX, 2) + pow(posy - desiredPosY, 2)) < 0.03 ) {
      pointIdx = (pointIdx+1) % 40;
      desiredPosX = pointsX[pointIdx];
      desiredPosY = pointsY[pointIdx];
      
    }
    


    
    
    Serial.print(" desired = (");
    Serial.print(desiredPosX);
    Serial.print(", ");
    Serial.print(desiredPosY);
    Serial.print(")");
    
    lastMillis = millis();

    // calculate dt sample
    unsigned long dtMicros = micros()-lastMicros;
    lastMicros = micros();
    dt_time = float(dtMicros)/1000000.0;
    
   
   
      
    P2P_CTRL(desiredPosX, desiredPosY, LeftMotorSpeed_Cntrl_CMD, RightMotorSpeed_Cntrl_CMD);


 
    odometry();

    
//    gyroIntegration();
    
    // calculate the error between <Wheel RadS = LeftMotorSpeed> and the output from P2P_CTRL = <LeftMotorSpeed_Cntrl_CMD>
    float errorLeft = LeftMotorSpeed_Cntrl_CMD - leftWheelSpeed;

    // calculate the error between <Wheel RadS = RightMotorSpeed> and the output from P2P_CTRL = <RightMotorSpeed_Cntrl_CMD>
    float errorRight = RightMotorSpeed_Cntrl_CMD - rightWheelSpeed;



    
    // insert error to pidController, insert the controll signal to the motors.
    cEr_left = pidController(errorLeft, prevErrorLeft, integralLeft, 2000);

    cEr_right = pidController(errorRight, prevErrorRight, integralRight, 2000); 
    
 

    motors.setLeftSpeed(cEr_left); // -400 --> 400
    motors.setRightSpeed(cEr_right);

    
    Serial.print(" desired = (");
    Serial.print(desiredPosX);
    Serial.print(", ");
    Serial.print(desiredPosY);
    Serial.print(")");
    
    Serial.print(" Pos = (");
    Serial.print(cEr_left);
    Serial.print(", ");
    Serial.print(cEr_right);
    Serial.println(")");
    
  
    prevErrorLeft = errorLeft;
    prevErrorRight = errorRight;

    
    
    Serial.print(" Pos = (");
    Serial.print(posx);
    Serial.print(", ");
    Serial.print(posy);
    Serial.println(")");
    
    
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
    
    float d_theta = float(dx_1-dx_2)/(WHEELS_DISTANCE / 1000.0);
    posx += cos(theta+d_theta/2)*(dx_1+dx_2)/2;
    posy += sin(theta+d_theta/2)*(dx_1+dx_2)/2;
    theta += d_theta;

    

    leftWheelSpeed = float(dx_2 / dt_time); 
    rightWheelSpeed = float(dx_1 / dt_time); 

    leftWheelSpeed = LowPassFilter(leftWheelSpeed, lastleftWheelSpeed, 0.5);
    rightWheelSpeed = LowPassFilter(rightWheelSpeed, lastrightWheelSpeed, 0.5);

    lastrightWheelSpeed = rightWheelSpeed;
    lastleftWheelSpeed = leftWheelSpeed;
    
    
}
