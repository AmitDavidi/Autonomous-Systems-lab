const int ledPin = 10; // green led

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
}


// Main loop to continuously receive input and update LED brightness
void loop() {
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    int brightness = inputString.toInt();
    analogWrite(ledPin, brightness);
  }
}
