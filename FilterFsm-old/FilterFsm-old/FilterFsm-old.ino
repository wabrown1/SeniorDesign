#include <ESP32Servo.h>

#define SAMPLE_RATE 500
#define BAUD_RATE 115200
#define INPUT_PIN A0

#define HIGH_VOLTAGE  3.3
#define ONBOARD_LED 2

// Moving average filter varaibles
# define WINDOW_SIZE 50
// window to hold to the values to be averaged for the MAF
float window[WINDOW_SIZE];
//int window[WINDOW_SIZE];
int MAFIndex = 0;
//long MAFSum = 0;
float MAFSum = 0;

long sampleTimer = millis();

int period = 2; // sample rate of 500 Hz
// variables used to calculate average high value in SampleHigh state
int sampleCount = 0;
float sumHigh = 0;
float averageHigh = 0;

enum {Waiting, SampleHigh, Running} State;

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
int servoPeriod = 30; // call the MoveServo function every 30ms
int servoTimer = 0;

int servoPosition = 0; // variable to store the servo position

void setup() {
  // Serial connection begin
  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(INPUT_PIN, INPUT);
  Serial.begin(BAUD_RATE);
  digitalWrite(ONBOARD_LED, LOW);
  State = Waiting;
  myservo.attach(13);  // attaches the servo on pin 13 to the servo object
}

float MovingAverage(float sample) {
  // remove old sample from MAFSum before adding in the new sample
  MAFSum = MAFSum - window[MAFIndex];
  // set current window index to the value from the input pin
  window[MAFIndex] = sample;
  //window[MAFIndex] = analogRead(INPUT_PIN);
  // Add the new sample to the MAF Sum
  MAFSum += window[MAFIndex];
  // Update the window index, resets back to 0 if MAFIndex > WINDOW_SIZE
  MAFIndex = (MAFIndex + 1) % WINDOW_SIZE;
  // returns the average of the values in the MAF window

  // output for given sample in voltage units
  return MAFSum / WINDOW_SIZE;
  // output for sample read directly from analog input pin
  //float MAFSample = ((MAFSum / WINDOW_SIZE) * HIGH_VOLTAGE) / 4096;
  //return MAFSample;
}

// Band-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: [74.5, 149.5] Hz.
// Filter is order 4, implemented as second-order sections (biquads).
// Reference:
// https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
// https://courses.ideate.cmu.edu/16-223/f2020/Arduino/FilterDemos/filter_gen.py
float EMGFilter(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - 0.05159732 * z1 - 0.36347401 * z2;
    output = 0.01856301 * x + 0.03712602 * z1 + 0.01856301 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -0.53945795 * z1 - 0.39764934 * z2;
    output = 1.00000000 * x + -2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - 0.47319594 * z1 - 0.70744137 * z2;
    output = 1.00000000 * x + 2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.00211112 * z1 - 0.74520226 * z2;
    output = 1.00000000 * x + -2.00000000 * z1 + 1.00000000 * z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

// Reads the value from the specified input pin and converts the value
// to voltage
float Sample(int inputPin) {
  return (analogRead(inputPin) * HIGH_VOLTAGE) / 4095;
}

// Function to be called in each state of the FSM to update the current
// state based on the input from the keyboard
// keyboard input 1 -> 49
// keyboard input 2 -> 50
// keyboard input 3 -> 51
void CheckState() {
  if (Serial.available()) {
    int keyboardInput = Serial.read();
    if (keyboardInput == 49) {  // 1
      State = Waiting;
      Serial.println("Waiting State");
    }
    else if (keyboardInput == 50) {// 2
      State = SampleHigh;
      Serial.println("Sample High State");
    }
    else if (keyboardInput == 51) {// 3
      State = Running;
      Serial.println("Running State");
    }
  }
}

// run on every iteration of main loop in filter fsm
void MoveServo(float sig, float avgHigh) {
  int steps = 10; // number of loop iterations to perform each time
  float speedDelay = 1;
  float maxDelay = 3;
  int pos = 0;  

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

void loop() {
  //Serial.println(State); // debug to check state switching from keyboard works
  switch (State) {
    case Waiting:
      CheckState();
      break;
    case SampleHigh:
      CheckState();
      digitalWrite(ONBOARD_LED, HIGH); // turn on led when in sample high state
      // find average high EMG value
      if ((period * sampleCount) < 5000) { // sample for 5 seconds
        if ((millis() - sampleTimer) > period) {
          float sensor_value = Sample(INPUT_PIN);
          float filteredSignal = EMGFilter(sensor_value);
          filteredSignal = abs(filteredSignal);
          float MAFSignal = MovingAverage(filteredSignal);
          sumHigh += MAFSignal;
          //sumHigh += filteredSignal;
          sampleCount++;
          //Serial.println(sensor_value);
          sampleTimer = millis();
        }
      }
      if ((period * sampleCount) == 5000) {
        digitalWrite(ONBOARD_LED, LOW);  // turn led on when done sampling for calculating the high average
        averageHigh = sumHigh / sampleCount;
        Serial.print("Average High: ");
        Serial.println(averageHigh);
        sampleTimer = millis();
        //sampleCount = 0;
        State = Running;
        Serial.println("Running State");
      }
      break;
    case Running:
      CheckState();
      float servoSignal;

      if ((millis() - sampleTimer) > period) {
        float sensor_value = Sample(INPUT_PIN);
        //float sensor_value = (analogRead(INPUT_PIN) * HIGH_VOLTAGE) / 4095;
        float filteredSignal = EMGFilter(sensor_value);
        filteredSignal = abs(filteredSignal);
        float MAFSignal = MovingAverage(filteredSignal);
        //Serial.println(filteredSignal);
        //Serial.println(MAFSignal);
        Serial.println(MAFSignal * 20);
        //Serial.println(filteredSignal*20);

        // turn on led (punch) if signal amplitude is greater than 50% of average high
        if (MAFSignal > (averageHigh / 2)) {
          //if (filteredSignal > (averageHigh / 2)) {
          digitalWrite(ONBOARD_LED, HIGH);
        }
        else {
          digitalWrite(ONBOARD_LED, LOW);
        }

        //MoveServo(MAFSignal, averageHigh); 
        servoSignal = MAFSignal;
        sampleTimer = millis();
      }

      /*if((millis() - servoTimer) > servoPeriod){
        MoveServo(servoSignal, averageHigh); 
        servoTimer = millis();
      }*/
      break;
    default:
      State = Waiting;
      CheckState();
      break;
  }

}
