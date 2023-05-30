#include <ArduinoJson.h>

const int SENSOR_1_PIN = 0;
const int UPDATE_INTERVAL_MS = 10;

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;     // whether the string is complete


int M0_PWM = 5;     // Set up the M0_PWM pin
int M0_PHASE = 6;   // Set up the M0_PHASE pin
int potPin = 0;     // Set up the potentiometer pin
int potVal;         // Declare a variable to hold the potentiometer value
int motorSpeed;     // Declare a variable to hold the motor speed

#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensor;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);



void setup(void)
{
  Wire.begin();

  Serial.begin(115200);
  while (!Serial) continue; // Wait for serial connection

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }

  // init motor
  pinMode(M0_PWM, OUTPUT);     
  pinMode(M0_PHASE, OUTPUT);  
  digitalWrite(M0_PHASE, LOW); 
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);


}



void loop(void)
{

  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();
  
  int max_dist_val = min(distance, 1000);
  motorSpeed = map(max_dist_val, 0, 1000, 0, 255);
  
  // Set the motor speed based on the potentiometer value
  analogWrite(M0_PWM, motorSpeed);


  static unsigned long last_update_time = 0;
  unsigned long current_time = millis();
  if (current_time - last_update_time >= UPDATE_INTERVAL_MS) {

    last_update_time = current_time;
    
    StaticJsonDocument<64> doc;
    doc["sensor1"] = distance;
    serializeJson(doc, Serial);
    Serial.println();
  }

  delay(5);


}
