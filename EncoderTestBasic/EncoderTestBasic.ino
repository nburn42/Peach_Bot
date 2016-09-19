/*
  DigitalReadSerial
 Reads a digital input on pin 2, prints the result to the serial monitor

 This example code is in the public domain.
 */

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // make the pushbutton's pin an input:
  pinMode(21, INPUT);
  pinMode(18, INPUT);
  pinMode(19, INPUT);
  pinMode(22, INPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input pin:
  int buttonState0 = digitalRead(21);
  int buttonState1 = digitalRead(20);
  int buttonState2 = digitalRead(19);
  int buttonState3 = digitalRead(18);
  int temp = 
    10000+
    buttonState3 * 1000 +
    buttonState2 * 100 +
    buttonState1 * 10 +
    buttonState0 * 1;
  
  // print out the state of the button:
  Serial.println(temp);
}



