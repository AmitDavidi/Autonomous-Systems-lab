#define ENCODER_PINA 2
#define ENCODER_PINB 3
#define COUNTS_PER_REVOLUTION 12
#define GEAR_RATIO 30

// encoder variables
int M0_PWM = 5;     // Set up the M0_PWM pin
int M0_PHASE = 6;   // Set up the M0_PHASE pin
int potPin = 0;     // Set up the potentiometer pin
int potVal;         // Declare a variable to hold the potentiometer value
int motorSpeed;     // Declare a variable to hold the motor speed
int static_variable_for_plotter = 10;
int static_variable2_for_plotter = -10;

// encoder variables
volatile int encoderCounts = 0;
unsigned long prevTime = 0;
float prevPosition = 0.0;

// control variables
float desiredRPM = 0.0, prevRPM = 0.0;
float error = 0.0, prevError = 0.0, integral = 0.0 , Kp_const = 1.0, Ki_const = 5.0 , dt_const = 0.01, outMin_const = 0 , outMax_const = 255.0;


// EncoderA ISR
void encoderA() {
  // look for a low-to-high on channel B
  if (digitalRead(ENCODER_PINA) == HIGH) {
    // check channel A to see which way encoder is turning
    digitalRead(ENCODER_PINB) ? encoderCounts++ : encoderCounts--;
  } 
  else {
    // check channel A to see which way encoder is turning
    digitalRead(ENCODER_PINB) ? encoderCounts-- : encoderCounts++;
  }
}  // End EncoderA ISR


// EncoderB ISR
void encoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(ENCODER_PINB) == HIGH) {
    // check channel A to see which way encoder is turning
    digitalRead(ENCODER_PINA) ? encoderCounts-- : encoderCounts++;
  } 
  else {
    // check channel A to see which way encoder is turning
    digitalRead(ENCODER_PINA) ? encoderCounts++ : encoderCounts--;
  }

}  // End EncoderB ISR

float pidController(float error, float prevError, float integral, float Kp, float Ki, float dt, float outMin, float outMax) {
  float proportional = Kp * error;  // Calculate the proportional term
  integral += Ki * error * dt;  // Calculate the integral term
  float output = proportional + integral;  // Calculate the output
  
  // Limit the output to the minimum and maximum values
  if (output > outMax) {
    output = outMax;
  } else if (output < outMin) {
    output = outMin;
  }
  
  return output;  // Return the output
}

// low pass filtering function
float LowPassFilter(float currentReading, float previousReading, float alpha) {
  float filteredReading = alpha * currentReading + (1 - alpha) * previousReading;
  return filteredReading;
}


void setup() {

  Serial.begin(115200);
  
  // init DC motor
  pinMode(M0_PWM, OUTPUT);     
  pinMode(M0_PHASE, OUTPUT);   
  digitalWrite(M0_PHASE, LOW); 
  
  // init leds
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  
  // initialize encoders, attache ISR functions
  pinMode(ENCODER_PINA, INPUT);
  pinMode(ENCODER_PINB, INPUT);

  // Attached interrupt to encoder pins
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA), encoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINB), encoderB, CHANGE);
}


void loop() {

  // get desired position if available
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    desiredRPM = (float)inputString.toFloat(); // r(t)
  }

  // ------ Plot wheel RPM ------
  // Calculate time difference
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0;  // convert to seconds

  // Calculate position difference
  float currentPosition = (float)encoderCounts / (COUNTS_PER_REVOLUTION * GEAR_RATIO);
  float deltaPosition = currentPosition - prevPosition;
  
  // Calculate velocity in RPM
  float currentRPM = (deltaPosition / deltaTime) * 60.0;
  

  float error = desiredRPM - currentRPM;

  float control_signal = pidController(error, prevError, integral, Kp_const, Ki_const, dt_const, outMin_const, outMax_const);



  analogWrite(M0_PWM, control_signal);


  // --- Print values ---
  // Serial.print(currTime / 1000.0);
  // Serial.print(", ");

  /* --------- limits for the plotter ----------- */
  Serial.print(static_variable_for_plotter);
  Serial.print(", ");

  Serial.print(static_variable2_for_plotter);
  Serial.print(", ");

  /* -------------------------------------------- */

  // Serial.print(currPosition);
  // Serial.print(", ");
  
  Serial.print(currentRPM);
  Serial.print(", ");

  Serial.print(desiredRPM);
  Serial.println(",");
    
  // Update previous values
  prevTime = currentTime;
  prevPosition = currentPosition;
  prevRPM = currentRPM;
  prevError = error;
  delay(10);


}





