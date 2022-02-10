/*
  Qwiic-Pro-Kit-Analog-Read

  Adapted from AnalogReadSerial example code.
  Reads an analog input on pin 1, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
*/

void setup() {  
  SerialUSB.begin(9600); // initialize serial communication at 9600 bits per second
}

void loop() {
  // read the input on analog pin 1:
  float rawEMG = analogRead(A1) * 3.3 / 1023.0;
  // print out the value you read:
  SerialUSB.print(rawEMG);
  SerialUSB.print(" ");
  SerialUSB.print(3.3);
  SerialUSB.print(" ");

  // read the input on analog pin 2:
  float emg = analogRead(A0) * 3.3 / 1023.0;
  // print the value you read:
  SerialUSB.println(emg);
}
