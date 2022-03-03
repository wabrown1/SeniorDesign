// EMG Filter - BioAmp EXG Pill
// https://github.com/upsidedownlabs/BioAmp-EXG-Pill

// Upside Down Labs invests time and resources providing this open source code,
// please support Upside Down Labs and open-source hardware by purchasing
// products from Upside Down Labs!

// Copyright (c) 2021 Upside Down Labs - contact@upsidedownlabs.tech

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#define SAMPLE_RATE 500
#define BAUD_RATE 115200
#define INPUT_PIN A0

#define HIGH_VOLTAGE  3.3
#define ONBOARD_LED 2

// Moving average filter varaibles
# define WINDOW_SIZE 5
// window to hold to the values to be averaged for the MAF
float window[WINDOW_SIZE];
//int window[WINDOW_SIZE];
int MAFIndex = 0;
//long MAFSum = 0;
float MAFSum = 0;

long timer1 = millis();

void setup() {
  // Serial connection begin
  pinMode(ONBOARD_LED, OUTPUT);
  Serial.begin(BAUD_RATE);
}

void loop() {
  // Calculate elapsed time
  static unsigned long past = 0;
  unsigned long present = micros();
  unsigned long interval = present - past;
  past = present;

  // Run timer
  static long timer = 0;
  timer -= interval;

  // Sample
  /*if (timer < 0) {
    timer += 1000000 / SAMPLE_RATE;
    float sensor_value = (analogRead(INPUT_PIN) * 3.3) / 4095;
    float filteredSignal = EMGFilter(sensor_value);
    Serial.println(filteredSignal);
  }*/

  if ((millis() - timer1) > 2) {
    float sensor_value = (analogRead(INPUT_PIN) * HIGH_VOLTAGE) / 4095;
    float filteredSignal = EMGFilter(sensor_value);
    filteredSignal = abs(filteredSignal);
    float MAFSignal = MovingAverage(filteredSignal);
    //Serial.println(filteredSignal);
    Serial.println(MAFSignal);

    if(MAFSignal > .1){
      digitalWrite(ONBOARD_LED, HIGH);
    }
    else{
      digitalWrite(ONBOARD_LED, LOW);
    }
    
    timer1 = millis();
  }

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
