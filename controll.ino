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
// X coordinates: {0.0, 0.0634239196565645, 0.12659245357374926, 0.18925124436041021, 0.2511479871810792, 0.3120334456984871, 0.3716624556603276, 0.42979491208917164, 0.4861967361004687, 0.5406408174555976, 0.5929079290546404, 0.6427876096865393, 0.690079011482112, 0.7345917086575333, 0.7761464642917568, 0.8145759520503357, 0.8497254299495144, 0.8814533634475821, 0.9096319953545183, 0.9341478602651067, 0.9549022414440739, 0.9718115683235417, 0.984807753012208, 0.9938384644612541, 0.998867339183008, 0.9998741276738751, 0.9968547759519424, 0.9898214418809327, 0.9788024462147787, 0.963842158559942, 0.9450008187146685, 0.9223542941045814, 0.8959937742913359, 0.8660254037844385, 0.8325698546347714, 0.795761840530832, 0.7557495743542583, 0.7126941713788627, 0.6667690005162917, 0.6181589862206051, 0.5670598638627709, 0.5136773915734063, 0.4582265217274105, 0.4009305354066136, 0.3420201433256689, 0.28173255684142967, 0.2203105327865408, 0.1580013959733499, 0.09505604330418244, 0.031727933498067656, -0.03172793349806786, -0.09505604330418263, -0.15800139597335008, -0.22031053278654056, -0.28173255684142984, -0.34202014332566866, -0.4009305354066138, -0.4582265217274103, -0.5136773915734064, -0.5670598638627706, -0.6181589862206053, -0.6667690005162915, -0.7126941713788629, -0.7557495743542582, -0.7957618405308321, -0.8325698546347713, -0.8660254037844388, -0.895993774291336, -0.9223542941045814, -0.9450008187146683, -0.9638421585599422, -0.9788024462147787, -0.9898214418809327, -0.9968547759519423, -0.9998741276738751, -0.998867339183008, -0.9938384644612541, -0.9848077530122081, -0.9718115683235417, -0.9549022414440739, -0.9341478602651068, -0.9096319953545182, -0.881453363447582, -0.8497254299495144, -0.8145759520503358, -0.7761464642917566, -0.7345917086575332, -0.690079011482112, -0.6427876096865396, -0.5929079290546402, -0.5406408174555974, -0.4861967361004688, -0.4297949120891719, -0.37166245566032724, -0.31203344569848707, -0.2511479871810794, -0.18925124436040974, -0.12659245357374904, -0.06342391965656452, -2.4492935982947064e-16}
// Y coordinates: {0.0, -0.0001271812711245652, -0.0010023959374544482, -0.003301760563327848, -0.007572012108672764, -0.01419717505048331, -0.02339177439275036, -0.03521638093720103, -0.04960745132427667, -0.06641284400403508, -0.08542612454239125, -0.10641543773294154, -0.1291452202602256, -0.15339080879464034, -0.17894698238223625, -0.20563183347292835, -0.2332873245444573, -0.26177765834393346, -0.2909863039314099, -0.32081225135329056, -0.3511658453402114, -0.3819643784191086, -0.41312749987436, -0.44457240874089987, -0.4762087362727625, -0.5079329778930923, -0.5396223009179413, -0.5711275296288647, -0.6022650938995561, -0.6328077254863499, -0.6624737056996658, -0.6909145238515346, -0.7177009198596322, -0.7423074889580905, -0.7640963656486452, -0.7823010332768865, -0.796012086415865, -0.8041678574295031, -0.8055542148640122, -0.7988194550505646, -0.7825117499241013, -0.755147487352797, -0.7153180594831372, -0.6618389008853709, -0.5939365679191723, -0.5114569718724238, -0.4150622253123621, -0.3063697687663588, -0.18798311558873548, -0.06337609503472316, 0.06337609503472356, 0.18798311558873587, 0.30636976876635913, 0.4150622253123617, 0.5114569718724241, 0.5939365679191719, 0.661838900885371, 0.7153180594831371, 0.7551474873527969, 0.7825117499241014, 0.7988194550505646, 0.8055542148640122, 0.804167857429503, 0.7960120864158651, 0.7823010332768865, 0.7640963656486452, 0.74230748895809, 0.717700919859632, 0.6909145238515346, 0.6624737056996659, 0.6328077254863497, 0.602265093899556, 0.5711275296288647, 0.5396223009179415, 0.5079329778930921, 0.4762087362727624, 0.4445724087409, 0.41312749987436015, 0.38196437841910846, 0.3511658453402114, 0.32081225135329067, 0.2909863039314097, 0.26177765834393335, 0.2332873245444573, 0.2056318334729285, 0.17894698238223605, 0.1533908087946403, 0.1291452202602256, 0.10641543773294165, 0.0854261245423912, 0.06641284400403508, 0.04960745132427667, 0.035216380937201086, 0.023391774392750305, 0.01419717505048331, 0.007572012108672767, 0.0033017605633278197, 0.0010023959374544464, 0.00012718127112456524, -0.0}

// circle points
// X coordinates: {1.0, 0.9979866764718844, 0.9919548128307953, 0.9819286972627067, 0.9679487013963562, 0.9500711177409454, 0.9283679330160726, 0.9029265382866212, 0.8738493770697849, 0.8412535328311812, 0.8052702575310586, 0.766044443118978, 0.7237340381050701, 0.6785094115571322, 0.6305526670845225, 0.5800569095711982, 0.5272254676105024, 0.4722710747726827, 0.41541501300188644, 0.3568862215918719, 0.2969203753282749, 0.23575893550942728, 0.17364817766693041, 0.1108381999010111, 0.04758191582374218, -0.01586596383480803, -0.07924995685678854, -0.14231483827328523, -0.20480666806519074, -0.26647381369003503, -0.32706796331742166, -0.3863451256931287, -0.4440666126057741, -0.5000000000000002, -0.5539200638661103, -0.6056096871376668, -0.654860733945285, -0.7014748877063214, -0.7452644496757547, -0.7860530947427875, -0.8236765814298327, -0.8579834132349771, -0.8888354486549234, -0.9161084574320696, -0.9396926207859083, -0.9594929736144974, -0.975429786885407, -0.9874388886763943, -0.9954719225730846, -0.9994965423831851, -0.9994965423831851, -0.9954719225730846, -0.9874388886763943, -0.975429786885407, -0.9594929736144974, -0.9396926207859084, -0.9161084574320696, -0.8888354486549235, -0.857983413234977, -0.8236765814298328, -0.7860530947427874, -0.7452644496757548, -0.7014748877063213, -0.6548607339452852, -0.6056096871376666, -0.5539200638661105, -0.4999999999999996, -0.44406661260577396, -0.3863451256931287, -0.3270679633174219, -0.26647381369003464, -0.20480666806519054, -0.14231483827328523, -0.07924995685678879, -0.01586596383480761, 0.04758191582374238, 0.11083819990101086, 0.17364817766692997, 0.23575893550942748, 0.2969203753282749, 0.35688622159187167, 0.4154150130018868, 0.4722710747726829, 0.5272254676105024, 0.5800569095711979, 0.6305526670845228, 0.6785094115571323, 0.7237340381050701, 0.7660444431189778, 0.8052702575310587, 0.8412535328311812, 0.8738493770697849, 0.9029265382866211, 0.9283679330160727, 0.9500711177409454, 0.9679487013963562, 0.9819286972627068, 0.9919548128307953, 0.9979866764718844, 1.0}
// Y coordinates: {0.0, 0.0634239196565645, 0.12659245357374926, 0.18925124436041021, 0.2511479871810792, 0.3120334456984871, 0.3716624556603276, 0.42979491208917164, 0.4861967361004687, 0.5406408174555976, 0.5929079290546404, 0.6427876096865393, 0.690079011482112, 0.7345917086575333, 0.7761464642917568, 0.8145759520503357, 0.8497254299495144, 0.8814533634475821, 0.9096319953545183, 0.9341478602651067, 0.9549022414440739, 0.9718115683235417, 0.984807753012208, 0.9938384644612541, 0.998867339183008, 0.9998741276738751, 0.9968547759519424, 0.9898214418809327, 0.9788024462147787, 0.963842158559942, 0.9450008187146685, 0.9223542941045814, 0.8959937742913359, 0.8660254037844385, 0.8325698546347714, 0.795761840530832, 0.7557495743542583, 0.7126941713788627, 0.6667690005162917, 0.6181589862206051, 0.5670598638627709, 0.5136773915734063, 0.4582265217274105, 0.4009305354066136, 0.3420201433256689, 0.28173255684142967, 0.2203105327865408, 0.1580013959733499, 0.09505604330418244, 0.031727933498067656, -0.03172793349806786, -0.09505604330418263, -0.15800139597335008, -0.22031053278654056, -0.28173255684142984, -0.34202014332566866, -0.4009305354066138, -0.4582265217274103, -0.5136773915734064, -0.5670598638627706, -0.6181589862206053, -0.6667690005162915, -0.7126941713788629, -0.7557495743542582, -0.7957618405308321, -0.8325698546347713, -0.8660254037844388, -0.895993774291336, -0.9223542941045814, -0.9450008187146683, -0.9638421585599422, -0.9788024462147787, -0.9898214418809327, -0.9968547759519423, -0.9998741276738751, -0.998867339183008, -0.9938384644612541, -0.9848077530122081, -0.9718115683235417, -0.9549022414440739, -0.9341478602651068, -0.9096319953545182, -0.881453363447582, -0.8497254299495144, -0.8145759520503358, -0.7761464642917566, -0.7345917086575332, -0.690079011482112, -0.6427876096865396, -0.5929079290546402, -0.5406408174555974, -0.4861967361004688, -0.4297949120891719, -0.37166245566032724, -0.31203344569848707, -0.2511479871810794, -0.18925124436040974, -0.12659245357374904, -0.06342391965656452, -2.4492935982947064e-16}
//

// square points
//float pointsX[40] = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.0, 0, 0, 0, 0, 0, 0, 0, 0,/ 0};
//float pointsY[40] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1};

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


float pointsX[40] = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float pointsY[40] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1};

int pointIdx = 0;
float posx = 0.0;
float posy = 0.0;

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
int switched = 0;

float pidController(float error, float prevError, float &integral) {
  float Kp = 2000.0, Ki = 500.0 , Kd = 0.0, outMin = -400.0 , outMax = 400.0;

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
    float w_Max = 0.7;  // max speed [m/s]
    

    if (w_forward > w_Max) {
        w_forward = float(w_Max);
    }
    
    w_last = w_forward;

    Left_Motor = float(w_forward - theta_t*0.05);
    Right_Motor = float(w_forward + theta_t*0.05);
    
    if(Left_Motor != Left_Motor) Left_Motor = 0.02;
    if(Right_Motor != Right_Motor) Right_Motor = 0.02;
      


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
  
  

  if (millis() - lastMillis >= SAMPLERATE){

   
    
    if( sqrt(pow(posx - desiredPosX, 2) + pow(posy - desiredPosY, 2)) < 0.05 ) {
      pointIdx = (pointIdx+1) % 40;
      desiredPosX = pointsX[pointIdx];
      desiredPosY = pointsY[pointIdx];
      
    }
    


    
    
    Serial.print(" desired = ");
    Serial.print(desiredPosX);
    Serial.print(", ");
    Serial.print(desiredPosY);
    
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
    cEr_left = pidController(errorLeft, prevErrorLeft, integralLeft);

    cEr_right = pidController(errorRight, prevErrorRight, integralRight); 
    
 

    motors.setLeftSpeed(cEr_left); // -400 --> 400
    motors.setRightSpeed(cEr_right);


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
  

    
    
}
