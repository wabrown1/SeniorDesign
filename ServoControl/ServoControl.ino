/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/faq/how-to-control-speed-of-servo-motor
 */

#include <ESP32Servo.h>
//#include <Servo.h>

Servo myServo;
unsigned long MOVING_TIME = 3000; // moving time is 3 seconds
unsigned long moveStartTime;
int startAngle = 30; // 30°
int stopAngle  = 90; // 90°

void setup() {
  myServo.attach(9);
  moveStartTime = millis(); // start moving

  // TODO: other code
}

void loop() {
  unsigned long progress = millis() - moveStartTime;

  if (progress <= MOVING_TIME) {
    long angle = map(progress, 0, MOVING_TIME, startAngle, stopAngle);
    myServo.write(angle); 
  }

  // TODO: other code
}





/*#include <ESP32Servo.h>

Servo myservo;  // create servo object to control a servo

// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
int servoPin = 13;
long timer1 = millis();
bool flag = 0;
int angle = 0;

void setup() {
  //pinMode(servoPin, OUTPUT);
  myservo.setPeriodHertz(50);
  myservo.attach(servoPin);
}

void loop() {

  



  // Sweep from 0 to 180 degrees:
  /*for (angle = 0; angle <= 180; angle += 1) {
    myservo.write(angle);
    delay(5);
  }
  // And back from 180 to 0 degrees:
  for (angle = 180; angle >= 0; angle -= 1) {
    myservo.write(angle);
    delay(30);
  }*/
  //delay(1000);
  /*
    myservo.write(0);
    delay(2000);
    myservo.write(180);
    delay(2000);*/
//}*/
