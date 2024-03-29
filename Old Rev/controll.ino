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

//float pointsX[40] =  {0.0, 0.0777758950280772, 0.14968017421997012, 0.21592741876609028, 0.2761826398646339, 0.32893939431918306, 0.3721488416531952, 0.40402774809181546, 0.42390383009972943, 0.4331875836555179, 0.43480460770968343, 0.4297641558975248, 0.41648663861914936, 0.39452773368477434, 0.3664954639509472, 0.33574921481341286, 0.29787608967181184, 0.24360076837784536, 0.17483960470749133, 0.09808333596644024, 0.017616767249714525, -0.06447605958006643, -0.14836584148222967, -0.23658973645598905, -0.3273734434273823, -0.4081094978767071, -0.4714976700996641, -0.5185296040662654, -0.5500264961447429, -0.5643843904618862, -0.5595212904425362, -0.536614281531643, -0.4994226865185669, -0.4508350242404613, -0.3922630530246315, -0.32455962437334535, -0.2487957532776786, -0.16698781958150263, -0.08263538582968323, -1.2246467991473532e-16};
//float pointsY[40] =  {-0.065, -0.06596715766559996, -0.07182714648845993, -0.08477656770727211, -0.1045980109122184, -0.12965829436602114, -0.15770740592569965, -0.18625205080572463, -0.21268896685030314, -0.23518797727785395, -0.2548443332247266, -0.27526107312975784, -0.2976165248568476, -0.3187714266411339, -0.33357339737597685, -0.33780630131112777, -0.3262132011382946, -0.28192710342982785, -0.18871262399093908, -0.05018195181879319, 0.10945661854726277, 0.257414064346341, 0.370416134948919, 0.44086041178689883, 0.46753619938280955, 0.45673293691890354, 0.4235360623169565, 0.37815988710823795, 0.32462674458535923, 0.26528846563924136, 0.2045319088826457, 0.14790969440907967, 0.09793554202873378, 0.05422559752517244, 0.01612365538673921, -0.016065435789108254, -0.04099971219650916, -0.057015569301139234, -0.06394198500399476, -0.065};
//#define NUM_POINTS 40
//
//float pointsX[100] =  {0.0, 0.03132172318132593, 0.06176464720897457, 0.09128553775206395, 0.11988440409335124, 0.14758565174608715, 0.17441586728077454, 0.2003838025928694, 0.22546748928195834, 0.24961033495041982, 0.2727246979520838, 0.29469972826998864, 0.315410458812775, 0.3347263343959656, 0.3525186381985981, 0.3686671747353809, 0.3830670529131308, 0.39563656053980883, 0.4063269154101035, 0.4151338993921347, 0.4221096464270131, 0.4273699671416627, 0.43108946258227393, 0.4334764286006948, 0.4347266398120687, 0.4349697722500451, 0.4342342000612856, 0.43245052325340827, 0.42949170790844626, 0.4252279058630182, 0.4195729304358399, 0.41251213267825143, 0.4041135178635126, 0.3945277336847743, 0.3839787943294745, 0.3727388801962179, 0.36107011905316977, 0.349113124445838, 0.3367316142411049, 0.3234069646694004, 0.3083428640398958, 0.29080561651125114, 0.27044906992351475, 0.2473708833283304, 0.2219453437305926, 0.19463005674484588, 0.16585186365742502, 0.13596729186113668, 0.1052612629262677, 0.07395771126883785, 0.04222977777077008, 0.010205219622085156, -0.022034104112213314, -0.05445866912911566, -0.08710250009658388, -0.12007479959507615, -0.15355965207828332, -0.18777745180389563, -0.22287177506215522, -0.2587169998228749, -0.29475202155120483, -0.33003738627518664, -0.36358104693302484, -0.3946794553010884, -0.4230229603346142, -0.44859106030529683, -0.4714976700996644, -0.49188025642782335, -0.50984216142633, -0.5254278882788285, -0.538614252696924, -0.5493107383063325, -0.5573709186275244, -0.5626205758906567, -0.56490435542383, -0.5641406993709392, -0.5603620358605593, -0.5537182904299341, -0.5444416011818789, -0.5327925950170608, -0.519013960872972, -0.5033050799444148, -0.4858168029077732, -0.4666583770363836, -0.44590877731495493, -0.42362782609315863, -0.39986537426156765, -0.374668552669337, -0.3480878814165508, -0.3201832311025565, -0.29103048250517766, -0.2607292468185104, -0.22941110949630242, -0.19724658837955286, -0.16444779395239995, -0.13126358308772806, -0.097965706608346, -0.06482780636477459, -0.032102196475238584, -1.2246467991473532e-16};
//float pointsY[100] =  {-0.065, -0.065062419204694, -0.06548315134038694, -0.06656500691071816, -0.0685365162558281, -0.07154947611534196, -0.07568564843618172, -0.08096711930648003, -0.08736691063427954, -0.09481886266245171, -0.10322687811354164, -0.1124734448461617, -0.12242690623784988, -0.1329468610126548, -0.14388737590121778, -0.15509816714455954, -0.1664244078756276, -0.1777063576374229, -0.18878066634212776, -0.1994859568774625, -0.20967575509648553, -0.21924095998587115, -0.22814018720511722, -0.23642871714484254, -0.2442688062391835, -0.25190468782527037, -0.2596016144582572, -0.2675714413154057, -0.27591648086825754, -0.2846094593376177, -0.2935051423795299, -0.3023676300408863, -0.3108999656628538, -0.318771426641134, -0.3256450466405305, -0.33120989751285496, -0.33521589167726973, -0.33748772290724155, -0.337863342882528, -0.33600843764311, -0.33117827181263687, -0.3221548994400535, -0.3074957873287124, -0.2859209702145984, -0.2565876195348462, -0.219198277336839, -0.1740215759062562, -0.12188301251675357, -0.06412731743942607, -0.0025312841910295376, 0.06084481084369382, 0.12385579814930961, 0.1844867562496054, 0.2410406494061057, 0.29225869453558495, 0.33734894838432583, 0.3759179306707726, 0.4078222721544247, 0.4329925879127435, 0.4513334781114645, 0.4628110174074546, 0.46769087198148424, 0.46668013452226154, 0.46079619473859523, 0.4510911357640315, 0.43845131900811474, 0.4235360623169562, 0.4068009541967783, 0.38854689381064833, 0.36896856332013594, 0.34819826614873206, 0.3263486130312984, 0.303556088313459, 0.28002068645968425, 0.2560282900678218, 0.2319399300335789, 0.20814369159605742, 0.1849873126692429, 0.16272341843323734, 0.14149009024372586, 0.1213262944758281, 0.102205637589282, 0.08407130070651048, 0.06686291666882971, 0.05053366632836889, 0.03505960648101835, 0.020443947781985508, 0.006718314022375715, -0.00605800711322009, -0.017800753571150435, -0.028406018658416625, -0.03775945931000286, -0.04575073836927897, -0.05229387404343139, -0.05735230106485865, -0.06096450414715534, -0.06326324634739033, -0.06448075540293251, -0.06493523793356942, -0.065};
//#define NUM_POINTS 100
//
//// circle points
//float pointsX[40] =  {0.15, 0.14805753939568692, 0.1422804662920718, 0.1328184038479815, 0.11991641451052518, 0.10390865302643992, 0.08520971200967338, 0.06430388421045813, 0.04173261958746791, 0.018080502038298484, -0.006039891016412235, -0.03000385406640665, -0.053190733056380314, -0.07499999999999997, -0.09486680633930657, -0.11227661222566512, -0.12677851283156918, -0.13799691654882362, -0.14564127261390777, -0.1495135962201315, -0.1495135962201315, -0.1456412726139078, -0.13799691654882362, -0.12677851283156924, -0.11227661222566519, -0.09486680633930665, -0.07500000000000007, -0.05319073305638038, -0.030003854066406682, -0.006039891016412271, 0.01808050203829835, 0.04173261958746781, 0.06430388421045806, 0.08520971200967334, 0.10390865302643988, 0.1199164145105251, 0.13281840384798144, 0.1422804662920718, 0.14805753939568692, 0.15};
//float pointsY[40] =  {0.0, 0.024061692128664037, 0.04750019907022087, 0.06970847580656528, 0.09011133963569683, 0.10818036710157217, 0.12344757988404845, 0.13551756519155733, 0.14407771674470582, 0.14890633111470808, 0.14987834972573544, 0.14696859780634014, 0.1402524364028122, 0.1299038105676658, 0.11619074427414819, 0.09946839873611933, 0.08016987391917019, 0.058794991479011266, 0.035897349643133715, 0.012069985307508913, -0.012069985307508808, -0.03589734964313361, -0.05879499147901124, -0.0801698739191701, -0.09946839873611925, -0.11619074427414812, -0.12990381056766576, -0.1402524364028122, -0.14696859780634014, -0.14987834972573544, -0.1489063311147081, -0.14407771674470587, -0.13551756519155736, -0.12344757988404848, -0.1081803671015722, -0.09011133963569694, -0.06970847580656538, -0.047500199070220954, -0.024061692128664092, -3.6739403974420595e-17};
//#define NUM_POINTS 40


// square points
float pointsX[40] = {0.0, 0.05, 0.1, 0.15000000000000002, 0.2, 0.25, 0.30000000000000004, 0.35000000000000003, 0.4, 0.45, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.45, 0.4, 0.35000000000000003, 0.30000000000000004, 0.25, 0.2, 0.15000000000000002, 0.1, 0.05, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float pointsY[40] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.05, 0.1, 0.15000000000000002, 0.2, 0.25, 0.30000000000000004, 0.35000000000000003, 0.4, 0.45, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.45, 0.4, 0.35000000000000003, 0.30000000000000004, 0.25, 0.2, 0.15000000000000002, 0.1, 0.05};
#define NUM_POINTS 40


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
  float Kp = 1500, Ki = 400.0, Kd = 0.0, outMin = -400.0 , outMax = 400.0;

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
  delay(2000);
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
          pointIdx = (pointIdx+2) ; 
          desiredPosX = pointsX[pointIdx];
          desiredPosY = pointsY[pointIdx];
      }
    
    else {
          pointIdx = (pointIdx+1) ;
          desiredPosX = pointsX[pointIdx];
          desiredPosY = pointsY[pointIdx];
      }
      
    }
    if(pointIdx >= NUM_POINTS) {
      
      motors.setLeftSpeed(0); // -400 --> 400
      motors.setRightSpeed(0);
      delay(100000);
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

    
    // calculate the error between <Wheel RadS = LeftMotorSpeed> and the output from P2P_CTRL = <LeftMotorSpeed_Cntrl_CMD>
    float errorLeft = LeftMotorSpeed_Cntrl_CMD - leftWheelSpeed;

    // calculate the error between <Wheel RadS = RightMotorSpeed> and the output from P2P_CTRL = <RightMotorSpeed_Cntrl_CMD>
    float errorRight = RightMotorSpeed_Cntrl_CMD - rightWheelSpeed;



    
    // insert error to pidController, insert the controll signal to the motors.
    cEr_left = pidController(errorLeft, prevErrorLeft, integralLeft);

    cEr_right = pidController(errorRight, prevErrorRight, integralRight); 
    
   

    motors.setLeftSpeed(cEr_left); // -400 --> 400
    motors.setRightSpeed(cEr_right);

        
   

    Serial.print(" Speed = (");
    Serial.print(leftWheelSpeed);
    Serial.print(", ");
    Serial.print(rightWheelSpeed);
    Serial.print(")");
    
    Serial.print(" pos = (");
    Serial.print(posx);
    Serial.print(", ");
    Serial.print(posy);
    Serial.print(")");
    Serial.print(" theta = ");
    Serial.println(theta);
    
    
  
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
    
    theta += d_theta*0.665*0.89; // band-aid 0.89 perfect infinity 0.885 perfect circle



    

    leftWheelSpeed = float(dx_2 / dt_time); 
    rightWheelSpeed = float(dx_1 / dt_time); 
//
    leftWheelSpeed = LowPassFilter(leftWheelSpeed, lastleftWheelSpeed, 0.5);
    rightWheelSpeed = LowPassFilter(rightWheelSpeed, lastrightWheelSpeed, 0.5);

    lastrightWheelSpeed = rightWheelSpeed;
    lastleftWheelSpeed = leftWheelSpeed;
    
    
}
