#define ENCODER_PINA 2
#define ENCODER_PINB 3
#define COUNTS_PER_REVOLUTION 12
#define GEAR_RATIO 30

// encoder variables
int M0_PWM = 5;     // Set up the M0_PWM pin IN2
int IN1 = 6;
int IN2 = 5;
int M0_PHASE = 6;   // Set up the M0_PHASE pin IN1
int potPin = 0;     // Set up the potentiometer pin
int potVal;         // Declare a variable to hold the potentiometer value
int motorSpeed;     // Declare a variable to hold the motor speed
int motorPin;

// encoder variables
volatile int encoderCounts = 0;
unsigned long prevTime = 0;
float prevPosition = 0.0;

// Encoder ISR functions - Interupt Service Routine
void encoderA();
void encoderB();

void setup() {

  Serial.begin(115200);

  pinMode(M0_PWM, OUTPUT);     // Set M0_PWM as an output pin
  pinMode(M0_PHASE, OUTPUT);   // Set M0_PHASE as an output pin
  digitalWrite(M0_PHASE, HIGH); // Set M0_PHASE to LOW to drive the motor in one direction
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  
  // initialize encoder, attache ISR functions
  pinMode(ENCODER_PINA, INPUT);
  pinMode(ENCODER_PINB, INPUT);

  // Attached interrupt to encoder pins
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA), encoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINB), encoderB, CHANGE);
}

void loop() {
   // Calculate time difference using millis()
  unsigned long currTime = millis();
  float deltaTime = (currTime - prevTime) / 1000.0;  // convert to seconds

  // Calculate position difference
  float currPosition = (float)encoderCounts / (COUNTS_PER_REVOLUTION * GEAR_RATIO);
  float deltaPosition = currPosition - prevPosition;
  
  // Calculate velocity in RPM
  float velocity = (deltaPosition / deltaTime) * 60.0;

  // Print values
  Serial.print(currTime / 1000.0);
  Serial.print(", ");
  Serial.print(currPosition);
  Serial.print(", ");
  Serial.print(velocity);

  // Update previous values
  prevTime = currTime;
  prevPosition = currPosition;



  // Read the value of the potentiometer
  potVal = analogRead(potPin);

  // Map the potentiometer value to a motor speed value between 0 and 255

  // Decide the motor spin direction based on the potentiometer value

  if (potVal < 500) {
    // Spin the motor in one direction
    motorSpeed = map(potVal, 0, 499, 0, 255);
        // IN2 = 1 IN1 = reversed PWM
    digitalWrite(IN2, HIGH);
    analogWrite(IN1, motorSpeed);

   

    digitalWrite(11, HIGH);
    digitalWrite(10, LOW);
    digitalWrite(9, LOW);

  } 
  else if (potVal > 524) {

    motorSpeed = map(potVal, 525, 1023, 255, 0);
    
     // IN1 = 1 IN2 = PWM
    digitalWrite(IN1, HIGH);
    analogWrite(IN2, motorSpeed);


    digitalWrite(9, HIGH);
    digitalWrite(10, LOW);
    digitalWrite(11, LOW);

  }
  else {
    digitalWrite(10, HIGH);
    digitalWrite(9, LOW);
    digitalWrite(11, LOW);

  }
  Serial.print(", ");
  Serial.println(potVal);
  
  delay(10);
  // Set the motor speed based on the potentiometer value
}

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
