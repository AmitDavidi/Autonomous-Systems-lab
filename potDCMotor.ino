int M0_PWM = 5;     // Set up the M0_PWM pin
int M0_PHASE = 6;   // Set up the M0_PHASE pin
int potPin = 0;     // Set up the potentiometer pin
int potVal;         // Declare a variable to hold the potentiometer value
int motorSpeed;     // Declare a variable to hold the motor speed

void setup() {
  pinMode(M0_PWM, OUTPUT);     // Set M0_PWM as an output pin
  pinMode(M0_PHASE, OUTPUT);   // Set M0_PHASE as an output pin
  digitalWrite(M0_PHASE, HIGH); // Set M0_PHASE to LOW to drive the motor in one direction
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // Read the value of the potentiometer
  potVal = analogRead(potPin);

  // Map the potentiometer value to a motor speed value between 0 and 255

  // Decide the motor spin direction based on the potentiometer value
  if (potVal < 500) {
    // Spin the motor in one direction
    digitalWrite(M0_PHASE, LOW);
    motorSpeed = map(potVal, 0, 511, 255, 0);
    digitalWrite(11, HIGH);
    digitalWrite(10, LOW);
    digitalWrite(9, LOW);

  } 
  else if (potVal > 524) {
    // Spin the motor in the other direction
    digitalWrite(M0_PHASE, HIGH);
    motorSpeed = map(potVal, 512, 1023, 255, 0);
    digitalWrite(9, HIGH);
    digitalWrite(10, LOW);
    digitalWrite(11, LOW);

  }
  else {
    digitalWrite(10, HIGH);
    digitalWrite(9, LOW);
    digitalWrite(11, LOW);

  }
  
  delay(10);
  Serial.println(motorSpeed);
  // Set the motor speed based on the potentiometer value
  analogWrite(M0_PWM, motorSpeed);
}
