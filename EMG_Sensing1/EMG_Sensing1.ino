void setup()
{
  Serial.begin(115200);
}

void loop()
{
  float sensor_value = (analogRead(A0) * 5.0) / 4096;
  Serial.println(sensor_value);

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
