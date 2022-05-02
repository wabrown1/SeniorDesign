#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>
// #include <ArduinoJson.h>

const char* ssid     = "OH_KSU_IOT";
const char* password = "ESP32iot!";
const char* host = "192.168.1.132";

#define SAMPLE_RATE 500
#define BAUD_RATE 115200
#define INPUT_PIN A0
#define ONBOARD_LED 2

WiFiClient espClient;
PubSubClient client(espClient);

Servo rightservo;  // create servo object to control a servo
Servo leftservo;
Servo centerservo;

// twelve servo objects can be created on most boards
// int servoPeriod = 2; // call the MoveServo function every servoPeriod ms
// long servoTimer = 0;
int servoPosition = 0; // variable to store the servo position

int timer = millis();

void setup() {
  // Serial connection begin
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  rightservo.attach(13);  // attaches the servo on pin 13 to the servo object
  leftservo.attach(14);
  centerservo.attach(15);
  rightservo.write(servoPosition);
  
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
  client.setCallback(callback);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  if(millis() - timer >= 1000){
    // Serial.println("Running");
    timer = millis();
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  // get a number out of servoString
  int input = 0;
  for(int i = 0; i < length; i++){
    input += ((uint8_t)payload[i] - 48) * pow(10, length - 1 - i);
  }
  servoPosition = input;
  Serial.println(topic);
  Serial.println(servoPosition);
  Serial.println();
  rightservo.write(servoPosition);
  if (strcmp(topic, "/reser/rightarm") == 1){
    rightservo.write(servoPosition);
  }
  else if (strcmp(topic, "/reser/leftarm") == 1){
    leftservo.write(servoPosition);
  }
  else if (strcmp(topic, "/reser/frontback") == 1){
    centerservo.write(servoPosition);
  }
  
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "controller";
    // clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("/reser/rightarm", "hello world");
      // ... and resubscribe
      client.subscribe("/reser/rightarm");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
