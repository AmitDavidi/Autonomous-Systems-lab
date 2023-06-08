const int ledPin = 10; // green led
int potPin = 0;     // Set up the potentiometer pin

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
}


// Main loop to continuously receive input and update LED brightness
void loop() {
    Serial.println(map(analogRead(potPin), 0, 1023, 0, 5));

    int brightness = map(analogRead(potPin), 0, 1023, 0, 255);
    analogWrite(ledPin, brightness);
}
