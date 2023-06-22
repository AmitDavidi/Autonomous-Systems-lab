// led pins are 9 10 11
const int ledPin1 = 9;
const int ledPin2 = 10;
const int ledPin3 = 11;

// Initialize the serial port communication
void setup() {
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    
    String inputString = Serial.readStringUntil('\n'); // read untill \n from serial
    inputString.trim(); // remove spaces
    
    int index = 0;
    String substring;
    int brightness_array[3] = {0, 0, 0};
    int i = 0;

    while (index < inputString.length() && i < 3) {

      int delimiterIndex = inputString.indexOf(',', index);
      
      // if next ',' not found - mark delimiterIndex as the end of string
      if (delimiterIndex == -1) {
        delimiterIndex = inputString.length();
      }
      substring = inputString.substring(index, delimiterIndex);
      brightness_array[i] = substring.toInt();
      i++;
      index = delimiterIndex + 1;
    }

    // Update the brightness of each LED
    analogWrite(ledPin1, brightness_array[0]);
    analogWrite(ledPin2, brightness_array[1]);
    analogWrite(ledPin3, brightness_array[2]);
  }
}

