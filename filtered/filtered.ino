const float sample_freq = 2000;
int kLength = 115; //length of FIR coefficients array, should also be length of data array
double rawData[115];
//double rawData[kLength] = {-20, 24, 10, 10, 0, 8, 16, 14, 26, 16, 6, 10, 0, 0, 16, -14, 0, 8, 18, 4, 18, 24, 8, 40, 48, 36, 18, 6, 8, 4, 0, 22, 62, 18, 16, 30, 34, 18, 8, 0, 12, 10, -2, 0, 0, 0, 18, 12, 16, 14, 10, 22, 32, 48, 0, 10, 0, -8, -2, -6, -12, 16, 0, 16, 8, 26, 16, 24, 32, 20, 0, -8, 0, -30, 4, 20, 6, 16, 6, 4, 20, 20, 24, 30, 20, 12, 40, -8, 32, 2, -48, -6, -14, 0, 16, 4, 6, 30, 8, 20, 32, 0, 6, 0, 16, -6, -12, 0, -16, 22, 22, 8, 12, 18, 10, 56, 30, 16, 32, 20, -6, 0, -8, -10, -10, 2, 0, 32};
double coefficients[115] = {-0.011338,-0.0065188,-0.0066222,-0.0053944,-0.0029425,0.00026297,0.0035024,0.0059758,0.0070513,0.0064549,0.0044052,0.0015627,-0.0011109,-0.002649,-0.0023507,-3.3055e-05,0.0038846,0.0084371,0.012392,0.014583,0.014298,0.011486,0.0068307,0.0015945,-0.0026983,-0.0047799,-0.0039704,-0.00042555,0.0047758,0.010001,0.013411,0.013554,0.0097857,0.00254,-0.0067331,-0.015904,-0.022733,-0.025506,-0.023552,-0.01754,-0.00938,-0.0018184,0.0022688,0.00065213,-0.0075642,-0.021553,-0.038745,-0.055274,-0.066751,-0.06923,-0.060133,-0.038972,-0.0075992,0.029968,0.068219,0.10115,0.1234,0.13126,0.1234,0.10115,0.068219,0.029968,-0.0075992,-0.038972,-0.060133,-0.06923,-0.066751,-0.055274,-0.038745,-0.021553,-0.0075642,0.00065213,0.0022688,-0.0018184,-0.00938,-0.01754,-0.023552,-0.025506,-0.022733,-0.015904,-0.0067331,0.00254,0.0097857,0.013554,0.013411,0.010001,0.0047758,-0.00042555,-0.0039704,-0.0047799,-0.0026983,0.0015945,0.0068307,0.011486,0.014298,0.014583,0.012392,0.0084371,0.0038846,-3.3055e-05,-0.0023507,-0.002649,-0.0011109,0.0015627,0.0044052,0.0064549,0.0070513,0.0059758,0.0035024,0.00026297,-0.0029425,-0.0053944,-0.0066222,-0.0065188,-0.011338
};

int dataIndex = 0;
long timer1 = micros();
#define INPUT_PIN A0
#define ONBOARD_LED 2

//const int len = sizeof(rawData)/sizeof(rawData[0]);

//double sum, sum_old;
//int thresh = 0;
//byte pd_state = 0;

double applyFilter(double sample) {
  // set the current data 
  rawData[dataIndex] = sample;
  // Update the data index, resets back to 0 if dataIndex > kLength
  dataIndex = (dataIndex + 1) % kLength;
  
  double y = 0;
  
  for(int i = 0; i < kLength; i++){
    // y = b0*x(n) + b1*x(n-1) + ... + bM*x(n-M)
    y += coefficients[i]*rawData[kLength-i];
  }
  return y;
  
    /*const int lenCoef = sizeof(coefficients)/sizeof(coefficients[0]);
    static int result[len] = {};

    for (int m = 0; m < len; m++) {
        double y = 0;
        for (int n = 0; n < lenCoef; n++) {
            if (m < n) {
                break;
            }
            y += coefficients[n] * data[m-n];
        }
        result[m] = y;
    }
    return result;*/
    //return y;
}
  
void setup() {
  Serial.begin(115200);
  pinMode(ONBOARD_LED, OUTPUT);
  //initialize data array to all zeros
  for(int i = 0; i < kLength; i++){
    rawData[i] = 0;
  }
  //int* filteredDataset = applyFilter(rawData);

  /*sum = 0;
  pd_state = 0;
  int period = 0;
  
  // Autocorrelation
  for(int i = 0; i < len; i++) {
 
    sum_old = sum;
    sum = 0;
    for(int k = 0; k < len - i; k++) {
      sum += filteredDataset[k] * filteredDataset[k+i];
    }
    
    // Peak Detect State Machine
    if (pd_state == 2 && (sum-sum_old) <= 0) {
      period = i;
      pd_state = 3;
    }
    
    if (pd_state == 1 && (sum > thresh) && (sum - sum_old) > 0) {
      pd_state = 2;
    }
    
    if (i == 0) {
      thresh = sum * 0.5;
      pd_state = 1;
    }
  }
  
  // Frequency identified in Hz
  if (period > 0 && period < INT_MAX) {
    Serial.println("Wohooo, here is your frequency:");
    Serial.println(sample_freq/period);
  } else {
    Serial.println("Yeah, too much noise ... ");
  }*/
}

void loop() {
  // 2kHz sampling rate
  if((micros()- timer1) >= 500){
    double sensor_value = (analogRead(INPUT_PIN));
    double filteredValue = applyFilter(sensor_value);
    Serial.println(filteredValue);
    if(abs(filteredValue) > 300){
      digitalWrite(ONBOARD_LED, HIGH);
    }
    else{
      digitalWrite(ONBOARD_LED, LOW);
    }
    
    timer1 = micros();
  }


  
}
