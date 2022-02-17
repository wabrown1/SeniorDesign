
#define SAMPLE_RATE 125
#define BAUD_RATE 115200
#define INPUT_PIN A0

#define ONBOARD_LED 2
// high voltage from microcontroller, should be 3.3 or 5 volts
#define HIGH_VOLTAGE  5.0

int timer1 = millis();
bool LED_State = LOW;
int sampleCount = 0;
int period = 5;
float sumLow = 0;
float sumHigh = 0;
float averageLow = 0;
float averageHigh = 0;

enum {Waiting, SampleLow, SampleHigh, Running} State;

// Moving average filter varaibles
# define WINDOW_SIZE 5
// window to hold to the values to be averaged for the MAF
int window[WINDOW_SIZE];
int MAFIndex = 0;
long MAFSum = 0;

void setup() {
  // Serial connection begin
  Serial.begin(BAUD_RATE);
  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(INPUT_PIN, INPUT);
  digitalWrite(ONBOARD_LED, LOW);
  State = Waiting;

  // initialize the window to all zeros
  for (int i = 0; i < WINDOW_SIZE; i++) {
    window[i] = 0;
  }
}

float MovingAverage() {
  // remove old sample from MAFSum before adding in the new sample
  MAFSum = MAFSum - window[MAFIndex];
  // set current window index to the value from the input pin
  window[MAFIndex] = analogRead(INPUT_PIN);
  // Add the new sample to the MAF Sum
  MAFSum += window[MAFIndex];
  // Update the window index, resets back to 0 if MAFIndex > WINDOW_SIZE
  MAFIndex = (MAFIndex + 1) % WINDOW_SIZE;
  // returns the average of the values in the MAF window

  float MAFSample = ((MAFSum / WINDOW_SIZE) * HIGH_VOLTAGE) / 4096;
  return MAFSample;
  // return ((MAFSum / WINDOW_SIZE) * HIGH_VOLTAGE) / 4096;
  //return MAFSum/WINDOW_SIZE;

}




// add moving average filter at each sample state
void loop() {
  // state machine to control sampling and motor output
  switch (State) {
    case Waiting:
      State = SampleLow;
      break;
    case SampleLow:
      /*if ((period * sampleCount) < 5000) { // sample for 5 seconds*/
      if ((period * sampleCount) < 5000) { // sample for 5 seconds
        if ((millis() - timer1) > period) {
          //LED_State = !LED_State;
          //digitalWrite(ONBOARD_LED,LED_State);
          float sensor_value = (analogRead(INPUT_PIN) * HIGH_VOLTAGE) / 4096;
          sumLow   += sensor_value;
          sampleCount++;
          //Serial.println(sensor_value);
          timer1 = millis();
        }
        if ((period * sampleCount) == 5000) {
          digitalWrite(ONBOARD_LED, HIGH);  // turn led on when done sampling for calculating the low average
          averageLow = sumLow / sampleCount;
          Serial.print("Average Low: ");
          Serial.println(averageLow);
          timer1 = millis();
          sampleCount = 0;
          State = SampleHigh;
        }
      }
      break;
    case SampleHigh:
      // find average high EMG value
      if ((period * sampleCount) < 5000) { // sample for 5 seconds
        if ((millis() - timer1) > period) {
          //LED_State = !LED_State;
          //digitalWrite(ONBOARD_LED,LED_State);
          float sensor_value = (analogRead(INPUT_PIN) * HIGH_VOLTAGE) / 4096;
          sumHigh += sensor_value;
          sampleCount++;
          //Serial.println(sensor_value);
          timer1 = millis();
        }
        if ((period * sampleCount) == 5000) {
          digitalWrite(ONBOARD_LED, LOW);  // turn led on when done sampling for calculating the low average
          averageHigh = sumHigh / sampleCount;
          Serial.print("Average High: ");
          Serial.println(averageHigh);
          timer1 = millis();
          //sampleCount = 0;
          State = Running;
        }
        break;
      case Running:
        if ((millis() - timer1) > period) {
          //Serial.print("MAF Sample: ");
          float MAFSample = MovingAverage();
          //Serial.println(MAFSample);
          //Serial.print("Window values: ");
          // print values in window that are to be averaged
          /*for (int i = 0; i < WINDOW_SIZE; i++) {
            //Serial.print((window[i]*HIGH_VOLTAGE)/4096);
            Serial.println(" ");
          }*/
          // turn on led if muscle is being flexed
          if (MAFSample > abs((averageHigh) / 1/*1.1*/)) {
            digitalWrite(ONBOARD_LED, HIGH);
          }
          else {
            digitalWrite(ONBOARD_LED, LOW);
          }
          timer1 = millis();
        }
        break;
      default:
        State = Waiting;
        break;
      }
  }

}
