
#include <ESP32Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int servoPosition = 0;
int pos = 0;    // variable to store the servo position
int timer1 = 0;

void setup() {
  myservo.attach(13);  // attaches the servo on pin 13 to the servo object
}

void loop() {
  /*for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(3);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  delay(1);*/

  myservo.write(pos);
  if((millis() - timer1) > 6000){
    if(pos == 180){
      pos = 0;
      //myservo.write(pos);
    }
    else{
      pos = 180;
     // myservo.write(0);
    }
    timer1 = millis();
  }
}

// run on every iteration of main loop in filter fsm
void MoveServo(float sig, float avgHigh) {
  int steps = 10; // number of loop iterations to perform each time
  float speedDelay = 1;
  float maxDelay = 3;

  // return to starting position if the user is not flexing
  if (sig < (avgHigh / 2) && (servoPosition > 0)) {
    for (pos = servoPosition; pos >= servoPosition - steps; pos -= 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      if (pos >= 0) {
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
      }
      delay(2);
    }
  }
  else {
    if (sig >= avgHigh) { // 0 delay if the signal value is greater than the average high
      speedDelay = 0;
    }
    else {
      speedDelay = ((avgHigh - sig) / avgHigh) * maxDelay; // scale the delay between 0 and maxDelay
    }
    for (pos = servoPosition; pos <= servoPosition + steps; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      if (pos <= 180) {
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
      }
      delay(speedDelay);
    }
  }
}


/*

  #include <ESP32Servo.h>

  Servo myservo;  // create servo object to control a servo
  // twelve servo objects can be created on most boards

  int servoPin = 13;
  int pos = 0;    // variable to store the servo position
  long timer1 = millis();

  void setup() {
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);    // standard 50 hz servo
  myservo.attach(servoPin, 500, 2400); // attaches the servo on pin 18 to the servo object
  // using default min/max of 1000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep

  //myservo.attach(13);  // attaches the servo on pin 13 to the servo object
  }

  void loop() {

  if ((millis() - timer1) > 500) {
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(1);                       // waits 15ms for the servo to reach the position
    }
    for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    timer1 = millis();
  }

  //delay(1000);
  }*/
