#include <WiFi.h>
#include <WiFiSSLClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <MQTT.h>
#include <ArduinoJson.h>
#include "secrets.h"

const char topic[]  = "anders/hej";

const long send_interval = 10000;
unsigned long previous_send = 0;

bool status[7] = {0, 0, 0, 0, 0, 0, 0};
bool g_status[7] = {0, 0, 0, 0, 0, 0, 0};
unsigned long senaste_andring[7] = {0, 0, 0, 0, 0, 0, 0};
unsigned long g_statustid[7] = {0, 0, 0, 0, 0, 0, 0};
unsigned long statustid = 0;

WiFiSSLClient client;
WiFiUDP ntpUDP;
MQTTClient mqttClient;
NTPClient timeClient(ntpUDP);

void setup() {
  Serial.begin(9600);
  delay(2000);
  
  Serial.println("Hello world");
  client.appendCustomCACert(SECRET_CACERT);
  pinMode(LED_D0, OUTPUT);
  pinMode(LED_D1, OUTPUT);
  pinMode(LED_D2, OUTPUT); // main loop
  pinMode(LED_D3, OUTPUT); // 
  pinMode(LED_USER, OUTPUT); // mqtt
  pinMode(LED_RESET, OUTPUT); // wifi

  WiFi.begin(SECRET_SSID, SECRET_WEPKEY);
  mqttClient.begin(SECRET_MQTT_BROKER, SECRET_MQTT_PORT, client);
  
  timeClient.begin();
    
  do {
    digitalWrite(LED_RESET, HIGH);
    timeClient.update();
    delay(200);
    digitalWrite(LED_RESET, LOW);
  } while (WiFi.status() != WL_CONNECTED && timeClient.isTimeSet() != true);

  initInputs();
  
}



void loop() {
  digitalWrite(LED_D0, HIGH);

  delay(10);
  digitalWrite(LED_D0, LOW);
  digitalWrite(LED_D1, LOW);
  digitalWrite(LED_D2, LOW);
  digitalWrite(LED_D3, LOW);

  unsigned long ms = millis();

  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_USER, LOW); //mqtt can not be connected
    
    logMsg(">>> Loop: Network not connected. Trying to reconnect."); 
    digitalWrite(LED_RESET, HIGH);
    WiFi.begin(SECRET_SSID, SECRET_WEPKEY);
    delay(500);
    digitalWrite(LED_RESET, LOW);
    delay(3000);
    return;
  }
  else {
    digitalWrite(LED_RESET, HIGH); //Network OK
  }

  timeClient.update();

  if (!mqttClient.connected()) {
    logMsg(">>> Loop: MQTT not connected"); 

    digitalWrite(LED_USER, LOW);
    digitalWrite(LED_D0, HIGH);

    logMsg(">>> Loop: Trying to connect MQTT");

    connectMQTT();
    digitalWrite(LED_D3, HIGH);
    return;
  }
  else {
    digitalWrite(LED_USER, HIGH); //MQTT OK
  }

  mqttClient.loop();

  updateInputs();
    
  if ((ms - previous_send >= send_interval )) { //heartbeat
    digitalWrite(LED_D1, HIGH);
    previous_send = ms;

    String output = createHeartbeatMessage();
    logMsg("Sending message: " + output);

    digitalWrite(LED_D1, HIGH);
    mqttClient.publish(topic, output);
    digitalWrite(LED_D1, LOW);

  }

  delay(100);
}

void logMsg(String msg) {
    Serial.print(timeClient.getFormattedTime());
    Serial.print(" ");
    Serial.println(msg);
}

void flashError(int t, int loops, int led) {
  for (int x = 0; x < loops; x++) {
    digitalWrite(led, HIGH);
    delay(t);
    digitalWrite(led, LOW);
    delay(t);
  }
  delay(3000);
  return;
}

void initInputs() {
  pinMode(PIN_A0, INPUT); // Högtryckspump
  pinMode(PIN_A1, INPUT); // Tryckvakt
  pinMode(PIN_A2, INPUT); // Larm
  pinMode(PIN_A3, INPUT); // Matarpump
  pinMode(PIN_A4, INPUT); // Tanksensor
  pinMode(PIN_A5, INPUT); // Renspolning
  pinMode(PIN_A6, INPUT); // ?

  status[0] = digitalRead(PIN_A0);
  status[1] = true; // Få notis om den är false vid första avläsning
  status[2] = false; // FÅ noties om den är true vid första avläsning
  status[3] = digitalRead(PIN_A3);
  status[4] = digitalRead(PIN_A4);
  status[5] = digitalRead(PIN_A5);
  status[6] = digitalRead(PIN_A6);

  long t = timeClient.getEpochTime();

  for (int x = 0; x < 7; x++) {
    senaste_andring[x] = t;
  }

}

void updateInputs() {
  
  for (int x = 0; x < 7; x++) {
    g_status[x] = status[x];
  }

  status[0] = digitalRead(PIN_A0);
  status[1] = digitalRead(PIN_A1);
  status[2] = digitalRead(PIN_A2);
  status[3] = digitalRead(PIN_A3);
  status[4] = digitalRead(PIN_A4);
  status[5] = digitalRead(PIN_A5);
  status[6] = digitalRead(PIN_A6);
  
  statustid = timeClient.getEpochTime();

  for (int x = 0; x < 7; x++) {
    if (g_status[x] != status[x]) {
      g_statustid[x] = statustid - senaste_andring[x];
      senaste_andring[x] = statustid;
      String msg = createStatusMessage(x);
      sendMessage(msg);
      delay(100);
    }
  }

}

void sendMessage(String msg) {
  digitalWrite(LED_D0, HIGH);
  logMsg("Sending message: " + msg);

  mqttClient.publish(topic, msg);
  digitalWrite(LED_D0, LOW);
}

String createStatusMessage(int id) {
  const int capacity (JSON_OBJECT_SIZE(4) + (7 * JSON_OBJECT_SIZE(4)));
  StaticJsonDocument<capacity> doc;
  String output;

  JsonObject obj = doc.createNestedObject("h");
  obj["z"] = 0;
  obj["time"] = statustid;
  obj["id"] = id;
  obj["s"] = status[id];
  obj["c"] = senaste_andring[id];
  obj["t"] = g_statustid[id];

  serializeJson(doc, output);
  return output;
}

String createHeartbeatMessage() {
  const int capacity (JSON_OBJECT_SIZE(4) + (7 * JSON_OBJECT_SIZE(4)));
  StaticJsonDocument<capacity> doc;
  String output;

  JsonObject obj = doc.createNestedObject("h");

  obj["z"] = 1;
  obj["time"] = timeClient.getEpochTime();

  serializeJson(doc, output);
  return output;
}


void checkMessages() {
/*
  int messageSize = mqttClient.parseMessage();
  if (messageSize) {
    Serial.print(timeClient.getFormattedTime());
    Serial.print(" ");
    Serial.print("Received a message with topic '");
    Serial.print(mqttClient.messageTopic());
    Serial.print("', length ");
    Serial.print(messageSize);
    Serial.print(" bytes:");

    while (mqttClient.available()) {
      Serial.print((char)mqttClient.read());
    }
    Serial.println();

  }
  */
}


bool connectMQTT() {
  logMsg("Stopping WiFi Client");
  client.stop();
  logMsg("Attempting to connect to the MQTT broker");
  digitalWrite(LED_D2, HIGH);

  if (!mqttClient.connect(SECRET_CLIENTNAME, SECRET_MQTT_USER, SECRET_MQTT_PASS)) {
    digitalWrite(LED_D3, HIGH);
    logMsg("MQTT not connected!");
    flashError(1000, 6, LED_USER);
    delay(5000);
    return false;
  }

  digitalWrite(LED_USER, HIGH) ;
  logMsg("We are connected to the MQTT broker!");

  flashError(200, 3, LED_USER);

  return true;
}