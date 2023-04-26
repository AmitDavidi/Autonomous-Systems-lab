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
int desired_wheel_position = 0;
double error, prev_error = 0.0, integral = 0.0 , Kp = 0.1, Ki = 0.1 , Kd = 0.0, dt = 0.01, outMin = -40.0 , outMax = 40.0;


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

double pidController(double error, double prevError, double integral, double Kp, double Ki, double Kd, double dt, double outMin, double outMax) {
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
    desired_wheel_position = inputString.toInt(); // r(t)
  }

  // ------ Plot wheel position ------
  // Calculate time difference
  unsigned long currTime = millis();
  float deltaTime = (currTime - prevTime) / 1000.0;  // convert to seconds

  // Calculate position difference
  float currPosition = (float)encoderCounts / (COUNTS_PER_REVOLUTION * GEAR_RATIO);
  float deltaPosition = currPosition - prevPosition;

  // Calculate velocity
  //float velocity = (deltaPosition / deltaTime) * 60.0;
  float error = desired_wheel_position - currPosition;


  float control_signal = pidController(error, prev_error, integral, Kp, Ki, Kd, dt, outMin, outMax);
  if(control_signal < 0) {
    control_signal *= -1;
    digitalWrite(M0_PHASE, HIGH);
  }
  else {
    digitalWrite(M0_PHASE, LOW);
    control_signal = map(control_signal, 0, 255, 255, 0);
  }


  if(abs(currPosition - desired_wheel_position) > 0.05) {
    analogWrite(M0_PWM, control_signal);
  }
  else {
    digitalWrite(M0_PHASE, HIGH);
    analogWrite(M0_PWM, 255);
  }


  // --- Print values ---
  // Serial.print(currTime / 1000.0);
  // Serial.print(", ");

  Serial.print(static_variable_for_plotter);
  Serial.print(", ");

  Serial.print(static_variable2_for_plotter);
  Serial.print(", ");

  Serial.print(currPosition);
  Serial.print(", ");

  // Serial.println(velocity);
  // Serial.print(", ");

  Serial.println(desired_wheel_position);
  Serial.print(",");
    
  // Update previous values
  prevTime = currTime;
  prevPosition = currPosition;

  delay(10);


}





