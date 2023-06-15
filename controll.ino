#include <Wire.h>
#include <Zumo32U4.h>

// zumo classes
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4IMU imu;

// time variables
#define SAMPLERATE 10          // 10 millis =  100 Hz
// Odometry settings
#define GEAR_RATIO 51.45      // Motor gear ratio 100.37
#define WHEELS_DISTANCE 98    // Distance between tracks in mm
#define WHEEL_DIAMETER 37.5   // Wheels diameter measured 38.5
#define ENCODER_PPR 12        // Encoder pulses per revolution
#define GYRO_SCALE 0.07        // 70 mdps/LSB 



unsigned long lastMillis = 0;
unsigned long lastMicros = 0;
float dt_time = SAMPLERATE/1000.0;

// infinity points:
float pointsX[40] =  {0.0, 0.07796279852044666, 0.15034585288772137, 0.21719158547807443, 0.27804329311496895, 0.331374919654008, 0.375175233291167, 0.40769678418515204, 0.4282388474624953, 0.4380465723780955, 0.43978844550784507, 0.43438963052241786, 0.42041136805942303, 0.39748811585457783, 0.3680960037173226, 0.335427069760104, 0.295518922240434, 0.23993788657416784, 0.17059485297182114, 0.09363333199658815, 0.01316676327986243, -0.06872081131573661, -0.1520287232859072, -0.23894690388736683, -0.32769558848069114, -0.4065089581103317, -0.46853728792986066, -0.5146048746259917, -0.5454010215198498, -0.5594005526637246, -0.5546623017199586, -0.532279264168877, -0.49575365042523034, -0.44780863260248954, -0.38982752768980655, -0.32269897112301027, -0.24753158656569446, -0.1663221409137514, -0.08244848233731378, -1.2246467991473532e-16};
float pointsY[40] =  {-0.06, -0.06097065217830785, -0.0668716574196292, -0.0799390184802036, -0.09995710911601352, -0.1252915774822026, -0.1537273363718332, -0.1828552620066096, -0.21019745637179454, -0.2340088977241921, -0.2552460306252849, -0.27715975280112715, -0.3007143464819011, -0.3228008357055886, -0.3383103027430125, -0.3427959127754232, -0.330622709240164, -0.2853305277190237, -0.19135498708153148, -0.052461746692965126, 0.10717682367309084, 0.2547717012557486, 0.3670127106597232, 0.4364509036850294, 0.46254658791851405, 0.4519960315518679, 0.4195066532525018, 0.3750620654831845, 0.32272806491399, 0.2648867682386831, 0.20571098843630753, 0.15040120488758826, 0.10133233082784882, 0.05820566707903888, 0.02049037227055775, -0.011424533992903371, -0.03616216296944064, -0.052060080232308494, -0.05894547951670264, -0.06};


// circle points
//float pointsX[40] =  {0.15, 0.14805753939568692, 0.1422804662920718, 0.1328184038479815, 0.11991641451052518, 0.10390865302643992, 0.08520971200967338, 0.06430388421045813, 0.04173261958746791, 0.018080502038298484, -0.006039891016412235, -0.03000385406640665, -0.053190733056380314, -0.07499999999999997, -0.09486680633930657, -0.11227661222566512, -0.12677851283156918, -0.13799691654882362, -0.14564127261390777, -0.1495135962201315, -0.1495135962201315, -0.1456412726139078, -0.13799691654882362, -0.12677851283156924, -0.11227661222566519, -0.09486680633930665, -0.07500000000000007, -0.05319073305638038, -0.030003854066406682, -0.006039891016412271, 0.01808050203829835, 0.04173261958746781, 0.06430388421045806, 0.08520971200967334, 0.10390865302643988, 0.1199164145105251, 0.13281840384798144, 0.1422804662920718, 0.14805753939568692, 0.15};
//float pointsY[40] =  {0.0, 0.024061692128664037, 0.04750019907022087, 0.06970847580656528, 0.09011133963569683, 0.10818036710157217, 0.12344757988404845, 0.13551756519155733, 0.14407771674470582, 0.14890633111470808, 0.14987834972573544, 0.14696859780634014, 0.1402524364028122, 0.1299038105676658, 0.11619074427414819, 0.09946839873611933, 0.08016987391917019, 0.058794991479011266, 0.035897349643133715, 0.012069985307508913, -0.012069985307508808, -0.03589734964313361, -0.05879499147901124, -0.0801698739191701, -0.09946839873611925, -0.11619074427414812, -0.12990381056766576, -0.1402524364028122, -0.14696859780634014, -0.14987834972573544, -0.1489063311147081, -0.14407771674470587, -0.13551756519155736, -0.12344757988404848, -0.1081803671015722, -0.09011133963569694, -0.06970847580656538, -0.047500199070220954, -0.024061692128664092, -3.6739403974420595e-17};

// square points
//float pointsX[40] = {0.0, 0.05, 0.1, 0.15000000000000002, 0.2, 0.25, 0.30000000000000004, 0.35000000000000003, 0.4, 0.45, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.45, 0.4, 0.35000000000000003, 0.30000000000000004, 0.25, 0.2, 0.15000000000000002, 0.1, 0.05, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//float pointsY[40] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.05, 0.1, 0.15000000000000002, 0.2, 0.25, 0.30000000000000004, 0.35000000000000003, 0.4, 0.45, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.45, 0.4, 0.35000000000000003, 0.30000000000000004, 0.25, 0.2, 0.15000000000000002, 0.1, 0.05};


float encoder2dist = (WHEEL_DIAMETER*3.14/(ENCODER_PPR*GEAR_RATIO)) / 1000.0;  // conversition of encoder pulses to distance in m

float theta = 0.0; 


// imu Fusion
float gyroAngle=0;
int32_t gyroOffset_z = -16;
float gyroz=0;
boolean motorsState = 0;
//


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

float pidController(float error, float prevError, float &integral) {
  float Kp = 1500, Ki = 450.0, Kd = 0.0, outMin = -400.0 , outMax = 400.0;

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
    float w_Max = 0.5;  // max speed [m/s]
    

    if (w_forward > w_Max) {
        w_forward = float(w_Max);
    }

    if(w_forward - w_last > 0.1) {
      w_forward = w_last + 0.1;
    }
    
    w_last = w_forward;

    Left_Motor = float(w_forward - theta_t*0.35);
    Right_Motor = float(w_forward + theta_t*0.35);
    
    if(Left_Motor != Left_Motor) Left_Motor = 0.02; // nan handling
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

    float dist2nextp = sqrt(pow(posx - desiredPosX, 2) + pow(posy - desiredPosY, 2));
    
    
    if(  dist2nextp < 0.035 ) {
      
      if(sqrt(pow(posx - pointsX[pointIdx + 2], 2) + pow(posy - pointsY[pointIdx + 2], 2) < dist2nextp)) {
        // skip next
          pointIdx = (pointIdx+2) % 40; 
          desiredPosX = pointsX[pointIdx];
          desiredPosY = pointsY[pointIdx];
      }
    
    else {
          pointIdx = (pointIdx+1) % 40;
          desiredPosX = pointsX[pointIdx];
          desiredPosY = pointsY[pointIdx];
      }
      
    }
    
    lastMillis = millis();

    // calculate dt sample
    unsigned long dtMicros = micros()-lastMicros;
    lastMicros = micros();
    dt_time = float(dtMicros)/1000000.0;
    
   
   
     odometry();
    P2P_CTRL(desiredPosX, desiredPosY, LeftMotorSpeed_Cntrl_CMD, RightMotorSpeed_Cntrl_CMD);
 
   
    
//    gyroIntegration();
//
//    LeftMotorSpeed_Cntrl_CMD = 0.3;
//    RightMotorSpeed_Cntrl_CMD =0.3;
//    Serial.print(" desired Speed = (");
//    Serial.print(LeftMotorSpeed_Cntrl_CMD);
//    Serial.print(", ");
//    Serial.print(RightMotorSpeed_Cntrl_CMD);
//    Serial.print(")");
    
    // calculate the error between <Wheel RadS = LeftMotorSpeed> and the output from P2P_CTRL = <LeftMotorSpeed_Cntrl_CMD>
    float errorLeft = LeftMotorSpeed_Cntrl_CMD - leftWheelSpeed;

    // calculate the error between <Wheel RadS = RightMotorSpeed> and the output from P2P_CTRL = <RightMotorSpeed_Cntrl_CMD>
    float errorRight = RightMotorSpeed_Cntrl_CMD - rightWheelSpeed;



    
    // insert error to pidController, insert the controll signal to the motors.
    cEr_left = pidController(errorLeft, prevErrorLeft, integralLeft);

    cEr_right = pidController(errorRight, prevErrorRight, integralRight); 
    
   

    motors.setLeftSpeed(cEr_left); // -400 --> 400
    motors.setRightSpeed(cEr_right);

        
   
//    
//    Serial.print(" SpeedP2P = (");
//    Serial.print(leftWheelSpeed);
//    Serial.print(", ");
//    Serial.print(rightWheelSpeed);
//    Serial.print(")");
//  
    prevErrorLeft = errorLeft;
    prevErrorRight = errorRight;

    
//
//    Serial.print(" Pos = (");
//    Serial.print(posx);
//    Serial.print(", ");
//    Serial.print(posy);
//    Serial.println(")");
    
    
  }
  

//    motorsState = (cEr_left || cEr_right) ==  0 ? 0 : 1; //  check if motors are still, for gyro

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
    
    theta += d_theta*0.665*0.9; // band-aid
   


    

    leftWheelSpeed = float(dx_2 / dt_time); 
    rightWheelSpeed = float(dx_1 / dt_time); 
//
    leftWheelSpeed = LowPassFilter(leftWheelSpeed, lastleftWheelSpeed, 0.2);
    rightWheelSpeed = LowPassFilter(rightWheelSpeed, lastrightWheelSpeed, 0.2);

    lastrightWheelSpeed = rightWheelSpeed;
    lastleftWheelSpeed = leftWheelSpeed;
    
    
}
