
//Encoder Example


// Define Pins

#define ENCODER_PINA 2
#define ENCODER_PINB 3
#define COUNTS_PER_REVOLUTION 12
#define GEAR_RATIO 30

// encoder variables

volatile int encoderCounts = 0;

unsigned long prevTime = 0;
float prevPosition = 0.0;


// Encoder ISR functions - Interupt Service Routine

void encoderA();
void encoderB();


void setup() {

  // initialize serial communication at 115200 bits per second:

  Serial.begin(115200);
  // 506 - 146 = 360
  // one wheel rotation = 360 counts.
  // 360 counts = 12 pulses per revolution * 30 revolutions = one flywheel rotation.
  // 30 enigne revolutions = 1 flywheel rotation.
  // that meants, current position = <encoder count> /


  // initialize encoder, attache ISR functions

  pinMode(ENCODER_PINA, INPUT);
  pinMode(ENCODER_PINB, INPUT);

  // Attached interrupt to encoder pins

  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA), encoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINB), encoderB, CHANGE);

  Serial.print("Encoder_Value");
}


void loop() {

  // print encoder position

  // Calculate time difference
  unsigned long currTime = millis();
  float deltaTime = (currTime - prevTime) / 1000.0;  // convert to seconds

  // Calculate position difference
  float currPosition = (float)encoderCounts / (COUNTS_PER_REVOLUTION * GEAR_RATIO);
  float deltaPosition = currPosition - prevPosition;


  // Calculate velocity
  float velocity = (deltaPosition / deltaTime) * 60.0;

  // Print values
  Serial.print(currTime / 1000.0);
  Serial.print(", ");
  Serial.print(currPosition);
  Serial.print(", ");
  Serial.println(velocity);

  // Update previous values
  prevTime = currTime;
  prevPosition = currPosition;
 
 
 // Serial.println(encoderCounts / 360); - rotations of the flywheel 

  delay(100);
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