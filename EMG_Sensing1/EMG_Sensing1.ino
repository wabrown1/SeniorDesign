#define INPUT_PIN A0
#define HIGH_VOLTAGE 3.3

long timer1 = millis();

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  // 2 kHz sampling rate
  if((millis() - timer1) > 2){
    float sensor_value = (analogRead(INPUT_PIN) * HIGH_VOLTAGE) / 4095;
    //Serial.println(sensor_value);
    Serial.println(sensor_value);
    timer1 = millis();
  }
  
  //float sensor_value = (analogRead(A0) * 5.0) / 4096;
  //Serial.println(sensor_value);

  //float sensorValue = analogRead(A0);
  //Serial.println(sensorValue);
  //float millivolt = (sensorValue/4096)*5;

  /*Serial.print("Sensor Value: ");
    //Serial.println(sensorValue);

    Serial.print("Voltage: ");
    Serial.print(millivolt*1000);
    Serial.println(" mV");
    Serial.println("");*/
  delay(1);
}
