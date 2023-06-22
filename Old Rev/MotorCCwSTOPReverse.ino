int IN1 = 6;
int IN2 = 5;
int motorSpeed;     // Declare a variable to hold the motor speed


void setup() {

  Serial.begin(115200);

  pinMode(IN1, OUTPUT);     // Set IN1 as an output pin
  pinMode(IN2, OUTPUT);   // Set IN2 as an output pin
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

}

void loop() {

    motorSpeed = 255;
    analogWrite(IN2,  255 - motorSpeed); 
    analogWrite(IN1,   motorSpeed);
    delay(5000);

    analogWrite(IN2,  0); 
    analogWrite(IN1,  0);
    delay(5000);

    motorSpeed = 0;
    analogWrite(IN2,  255 - motorSpeed); 
    analogWrite(IN1,   motorSpeed);
    delay(5000);

    analogWrite(IN2,  0); 
    analogWrite(IN1,  0);
    delay(5000);

}
