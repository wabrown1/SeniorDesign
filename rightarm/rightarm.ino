#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid     = "OH_KSU_IOT";
const char* password = "ESP32iot!";
const char* host = "192.168.1.132";

WiFiClient espClient;
PubSubClient client(espClient);

#define SAMPLE_RATE 500
//#define BAUD_RATE 115200
#define BAUD_RATE 9600
#define INPUT_PIN A7

#define HIGH_VOLTAGE  3.3
#define ONBOARD_LED 13

// Moving average filter varaibles
# define WINDOW_SIZE 25
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
int servoPeriod = 5; // call the MoveServo function every servoPeriod ms
long servoTimer = 0;
long notFlexedTimer = 0;
bool notFlexedFlag = false;

int servoPosition = 0; // variable to store the servo position
char servoString[30];

long stateTimer = 0;
long blinkLEDTimer = 0;
bool ledState = 0;

bool prevButtonPressed = false;

void setup() {
  // Serial connection begin
  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(INPUT_PIN, INPUT);
  Serial.begin(BAUD_RATE);
  digitalWrite(ONBOARD_LED, LOW);
  State = Waiting;
  myservo.attach(4);  // attaches the servo on pin 13 to the servo object

  // Wifi initialization
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  delay(100);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // MQTT Setup
  client.setServer(host, 1883);
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
  if (digitalRead(0)) {
    prevButtonPressed = false;
  }
  switch (State) {
    case Waiting:
      if (!digitalRead(0) && !prevButtonPressed) {
        State = SampleHigh;
        Serial.println("Sample High State");
        Serial.println("TEST");
        prevButtonPressed = true;
      }
      break;
    case SampleHigh:
      if (!digitalRead(0) && !prevButtonPressed) {
        State = Running;
        Serial.println("Running State");
        prevButtonPressed = true;
      }
      break;
    case Running:
      if (!digitalRead(0) && !prevButtonPressed) {
        State = SampleHigh;
        Serial.println("Sample High State");
        prevButtonPressed = true;
        // reset average high variables before returning to sample high state
        sampleCount = 0;
        sumHigh = 0;
        averageHigh = 0;
        MAFIndex = 0;
        MAFSum = 0;
      }
      break;
    default:
      State = Waiting;
      break;
  }
}

// run on every iteration of main loop in filter fsm
void MoveServo(float sig, float avgHigh) {
  if ((millis() - servoTimer) > servoPeriod) {
    // Speed Thresholds
    // Adjust the servo speed by chaning the servoPeriod based on the signal amplitude
    if ((sig > (avgHigh * .5)) && (servoPosition <= 35)) {
      servoPosition += 5;
      notFlexedFlag = false;

      if (sig >= avgHigh) {
        servoPeriod = 0;
      }
      else {
        // Scales the servoPeriod between 0 and 4 ms
        servoPeriod = (1 - (sig / avgHigh)) * 8;
      }
    }


    else if ((sig < (avgHigh * .4)) && (servoPosition >= 5)) {  // not flexed
      if (!notFlexedFlag) {
        notFlexedFlag = true;
        notFlexedTimer = millis();
      }
      else { // if a notflexed state was previously detected
        // move the servo back to starting position if 500ms have gone by without detecting a flex
        if ((millis() - notFlexedTimer) > 200) {
          servoPosition = 0;
          sprintf(servoString, "%d", servoPosition);
          client.publish("/reser/rightarm", servoString);
          // myservo.write(servoPosition);
          notFlexedTimer = millis();
          notFlexedFlag = false;
        }
      }
      //servoPosition -= 5;
    }
    // myservo.write(servoPosition);
    sprintf(servoString, "%d", servoPosition);
    Serial.print(servoPosition);
    Serial.print("   ");
    Serial.println(servoString);
    client.publish("/reser/rightarm", servoString);

    servoTimer = millis();
  }
}

void loop() {
  /*if ((millis() - stateTimer) > 200) {
    CheckState();
    stateTimer = millis();
    }*/
  //Serial.println(State); // debug to check state switching from keyboard works
  switch (State) {
    case Waiting:
      CheckState();
      break;
    case SampleHigh:
      CheckState();
      // flash an LED to know you are in the sampling state
      if ((millis() - blinkLEDTimer) > 500) {
        ledState = !ledState;
        //digitalWrite(ONBOARD_LED, ledState);
        if (ledState) {
          digitalWrite(ONBOARD_LED, HIGH);
        }
        else {
          digitalWrite(ONBOARD_LED, LOW);
        }
        blinkLEDTimer = millis();
      }

      //CheckState();
      //digitalWrite(ONBOARD_LED, HIGH); // turn on led when in sample high state
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
        Serial.print("MAF Signal: ");
        Serial.println(MAFSignal);
        //Serial.println(MAFSignal * 20);
        //Serial.println(filteredSignal*20);

        // turn on led (punch) if signal amplitude is greater than 50% of average high
        if (MAFSignal > (.4 * averageHigh)) {
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

      /*if ((millis() - servoTimer) > servoPeriod) {
        MoveServo(servoSignal, averageHigh);
        servoTimer = millis();
        }*/

      MoveServo(servoSignal, averageHigh);

      break;
    default:
      State = Waiting;
      CheckState();
      break;
  }
  if (!client.connected()) {
    reconnect();
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "Rightarm";
    // clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("/reser", "hello world");
      // ... and resubscribe
      // client.subscribe("/");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
