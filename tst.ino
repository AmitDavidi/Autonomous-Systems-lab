#define ENCODER_PINA 2
#define ENCODER_PINB 3
#define COUNTS_PER_REVOLUTION 12
#define GEAR_RATIO 30

// encoder variables
int motorSpeed;     // Declare a variable to hold the motor speed
int IN1 = 6;
int IN2 = 5;

// encoder variables
volatile int encoderCounts = 0;
unsigned long prevTime = 0;
float prevPosition = 0.0;

// control variables
int desired_wheel_position = 0;
double error, prev_error = 0.0;
double integral = 0.0;

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

double pidController(double error, double prevError) {
  double Kp = 200.0, Ki = 100.3 , Kd = 10.2, dt = 0.01, outMin = -255.0 , outMax = 255.0;

  integral += Ki * error * dt;  
  double proportional = Kp * error;  
  double derivative = Kd * (error - prevError) / dt;  
  double output = proportional + integral + derivative;  
  
  // Limit the output to the minimum and maximum values
  if (output > outMax) {
    output = outMax;
  } else if (output < outMin) {
    output = outMin;
  }
  
  return output;  // Return the output
}



void setup() {

  Serial.begin(9600);
  
  // init DC motor
  pinMode(IN1, OUTPUT);     
  pinMode(IN2, OUTPUT);   
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  
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


  float control_signal = pidController(error, prev_error);
  if(abs(error) > 0.05){
    analogWrite(IN2, map(control_signal, -255.0, 255.0, 0.0, 255));
    analogWrite(IN1,   map(control_signal, -255.0, 255.0, 255, 0));
  }



  Serial.print("error: ");
  Serial.print(error);
  Serial.print(", ");
  // --- Print values ---
  Serial.print("IN1 = ");
   
  Serial.print(map(control_signal, -255.0, 255.0, 0, 255));
  Serial.print(", IN2 = ");

  Serial.print(map(control_signal, -255.0, 255.0, 255, 0.0));
  Serial.print(", ");

  Serial.print(currPosition);
  Serial.print(", ");

  Serial.println(desired_wheel_position);
  Serial.print(",");
    
  // Update previous values
  prevTime = currTime;
  prevPosition = currPosition;
  prev_error = error;

  delay(10);


}





