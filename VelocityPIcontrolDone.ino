#define ENCODER_PINA 2
#define ENCODER_PINB 3
#define COUNTS_PER_REVOLUTION 12
#define GEAR_RATIO 30

// encoder variables
int M0_PWM = 5;    // Set up the M0_PWM pin
int M0_PHASE = 6;  // Set up the M0_PHASE pin
int potPin = 0;    // Set up the potentiometer pin
int potVal;        // Declare a variable to hold the potentiometer value
int motorSpeed;    // Declare a variable to hold the motor speed
int IN1 = 6;
int IN2 = 5;
int reverse_flag = 0;

// encoder variables
volatile int encoderCounts = 0;

// control variables
double desiredRPM = 0.0, prevRPM = 0.0, currentRPM = 0.0, control_signal = 0.0, prev_control_signal = 0.0;
double error = 0.0, prevError = 0.0;
unsigned long currentTime = 0, prevTime = 0;
double deltaTime = 0.0, currentPosition = 0.0, deltaPosition = 0.0, prevPosition = 0.0;

// EncoderA ISR
void encoderA() {
  // look for a low-to-high on channel B
  if (digitalRead(ENCODER_PINA) == HIGH) {
    // check channel A to see which way encoder is turning
    digitalRead(ENCODER_PINB) ? encoderCounts++ : encoderCounts--;
  } else {
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
  } else {
    // check channel A to see which way encoder is turning
    digitalRead(ENCODER_PINA) ? encoderCounts++ : encoderCounts--;
  }

}  // End EncoderB ISR



double PIController() {

  double Kp = 1.00;
  double Ki = 8.0;
  double dt = 0.01;
  double outMax = 255.0;
  double outMin = -255.0;
  static double integral = 0.0;
  double proportional = Kp * error;     // Calculate the proportional term
  integral += double(Ki * error * dt);  // Calculate the integral term
  double output = proportional + integral;  // Calculate the output

  // Limit the output to the minimum and maximum values
  if (output > outMax) {
    output = outMax;
  } else if (output < outMin) {
    output = outMin;
  }

  return output;  // Return the output
}

// low pass filtering function
double LowPassFilter(double currentReading, double previousReading, double alpha) {
  if(abs(currentReading - previousReading) > 60000) {
    currentReading = previousReading; // bad ENCODER_COUNTS read, integer overflow, skip this reading.
  }
  double filteredReading = double(alpha * currentReading + (1.0 - alpha) * previousReading);
  return filteredReading;
}


void setup() {

  Serial.begin(9600);

  // init DC motor
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  // initialize encoders, attache ISR functions
  pinMode(ENCODER_PINA, INPUT);
  pinMode(ENCODER_PINB, INPUT);

  // Attached interrupt to encoder pins
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA), encoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINB), encoderB, CHANGE);
  delay(1);
}


void loop() {
  // get desired position if available
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    desiredRPM = (double)inputString.toDouble();  // r(t)
  }

  // ------ Plot wheel RPM ------
  // Calculate time difference
  currentTime = millis();
  deltaTime = double(currentTime - prevTime) / 1000.0;  // convert to seconds

  // Calculate position difference
  currentPosition = (double)encoderCounts / (COUNTS_PER_REVOLUTION * GEAR_RATIO);
  

  deltaPosition = (double)currentPosition - prevPosition;

  // Calculate velocity in RPM
  currentRPM = (double)(deltaPosition / deltaTime) * 60.0;

  currentRPM = LowPassFilter(currentRPM, prevRPM, 0.3);  // filter the measured velocity

  error = double(desiredRPM - currentRPM);

  control_signal = PIController();

  // Set motor direction and speed
  analogWrite(IN2, control_signal);
  analogWrite(IN1, 255 - control_signal);


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
