
#include <SPI.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <SSLClient.h>
//#include <ArduinoMqttClient.h>
#include <MQTT.h>
#include <Arduino_ConnectionHandler.h>
#include <ArduinoJson.h>
#include "trust_anchors.h"
#include "secrets.h"

const char broker[] = SECRET_MQTT_BROKER;
const char clientname[] = "anders1";
int        port     = 8883;
const char topic[]  = "anders/hej";
const char configtopic[]  = "anders/config";

const long send_interval = 60000;
unsigned long previous_send = 0;

const long check_interval = 1000;
unsigned long previous_check = 0;

const long wifi_interval = 10000;
unsigned long previous_wifi = 0;

const long mqttconn_interval = 10000;
unsigned long previous_mqttconn = 0;

bool status[6] = {0, 0, 0, 0, 0, 0};
bool g_status[6] = {0, 0, 0, 0, 0, 0};
unsigned long senaste_andring[6] = {0, 0, 0, 0, 0, 0};
unsigned long g_statustid[6] = {0, 0, 0, 0, 0, 0};
unsigned long statustid = 0;

WiFiConnectionHandler conMan(SECRET_SSID, SECRET_WEPKEY);

SSLClient wifiSSLClient(conMan.getClient(), TAs, (size_t)TAs_NUM, A5);


MQTTClient mqttClient;

NTPClient timeClient(conMan.getUDP());

void setup() {
  Serial.begin(9600);
  delay(2000);

  Serial.println("Hello world");

  pinMode(LED_D0, OUTPUT);
  pinMode(LED_D1, OUTPUT);
  pinMode(LED_D2, OUTPUT); // main loop
  pinMode(LED_D3, OUTPUT); // 
  pinMode(LED_USER, OUTPUT); // mqtt
  pinMode(LED_RESET, OUTPUT); // wifi

  //wifiSSLClient.setTimeout(5);

  mqttClient.begin(broker, port, wifiSSLClient);
  
  conMan.addCallback(NetworkConnectionEvent::CONNECTED, onNetworkConnect);
  conMan.addCallback(NetworkConnectionEvent::DISCONNECTED, onNetworkDisconnect);
  conMan.addCallback(NetworkConnectionEvent::ERROR, onNetworkError);

  timeClient.begin();
    
  do {
    conMan.check();
    timeClient.update();
  } while (conMan.getStatus() != NetworkConnectionState::CONNECTED && timeClient.isTimeSet() != true);

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

  if (ms - previous_wifi >= wifi_interval) {
    Serial.print(timeClient.getFormattedTime());
    Serial.print(" ");
    Serial.println(">>> Loop: Checking conMan"); 
    conMan.check();
    previous_wifi = ms;
  }

  if (conMan.getStatus() != NetworkConnectionState::CONNECTED) {
    digitalWrite(LED_RESET, LOW);
    Serial.print(timeClient.getFormattedTime());
    Serial.print(" ");
    Serial.println(">>> Loop: Network not connected"); 
    return;
  }
  else {
    digitalWrite(LED_RESET, HIGH); //Network OK
  }

  timeClient.update();

  if (!mqttClient.connected()) {
    Serial.print(timeClient.getFormattedTime());
    Serial.print(" ");
    Serial.println(">>> Loop: MQTT not connected"); 
    digitalWrite(LED_USER, LOW);
    digitalWrite(LED_D0, HIGH);
    Serial.print(timeClient.getFormattedTime());
    Serial.print(" ");
    Serial.println(">>> Loop: Stopping SSL"); 
    //wifiSSLClient.removeSession();
    wifiSSLClient.stop();
    digitalWrite(LED_D1, HIGH);
    Serial.print(timeClient.getFormattedTime());
    Serial.print(" ");
    Serial.println(">>> Loop: Trying to connect MQTT");
    //mqttClient.begin(broker, port, wifiSSLClient);
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
    Serial.print(timeClient.getFormattedTime());
    Serial.print(" ");
    Serial.print("Sending message: ");
    Serial.println(output);

    digitalWrite(LED_D1, HIGH);
    mqttClient.publish(topic, output);
    digitalWrite(LED_D1, LOW);

  }

  delay(100);
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
  pinMode(PIN_A0, INPUT); // Tryckvakt
  pinMode(PIN_A1, INPUT); // Matarpump
  pinMode(PIN_A2, INPUT); // Tanksensor
  pinMode(PIN_A3, INPUT); // Högtryckspump
  pinMode(PIN_A4, INPUT); // Backspolning
  pinMode(PIN_A5, INPUT); // Larm

  status[0] = digitalRead(PIN_A0);
  status[1] = digitalRead(PIN_A1);
  status[2] = digitalRead(PIN_A2);
  status[3] = digitalRead(PIN_A3);
  status[4] = digitalRead(PIN_A4);
  status[5] = digitalRead(PIN_A5);

  long t = timeClient.getEpochTime();

  for (int x = 0; x < 6; x++) {
    senaste_andring[x] = t;
  }

}

void updateInputs() {
  
  for (int x = 0; x < 6; x++) {
    g_status[x] = status[x];
  }

  status[0] = digitalRead(PIN_A0);
  status[1] = digitalRead(PIN_A1);
  status[2] = digitalRead(PIN_A2);
  status[3] = digitalRead(PIN_A3);
  status[4] = digitalRead(PIN_A4);
  status[5] = digitalRead(PIN_A5);
  
  statustid = timeClient.getEpochTime();

  for (int x = 0; x < 6; x++) {
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
  Serial.print(timeClient.getFormattedTime());
  Serial.print(" ");
  Serial.print("Sending message to topic: ");
  Serial.print(topic);
  Serial.print(":");
  Serial.println(msg);

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


void onNetworkConnect() {
  digitalWrite(LED_RESET, HIGH);
  Serial.println(">>>> CONNECTED to network");
  flashError(200, 3, LED_RESET);
  timeClient.forceUpdate();  
}

void onNetworkDisconnect() {
  digitalWrite(LED_RESET, LOW);
  Serial.println(">>>> DISCONNECTED from network");
}

void onNetworkError() {
  digitalWrite(LED_RESET, LOW);
  Serial.println(">>>> ERROR");
  flashError(500, 8, LED_RESET);
}

bool connectMQTT() {
  Serial.print(timeClient.getFormattedTime());
  Serial.print(" ");

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);
  
  digitalWrite(LED_D2, HIGH);

  if (!mqttClient.connect(clientname, SECRET_MQTT_USER, SECRET_MQTT_PASS)) {
    digitalWrite(LED_D3, HIGH);
    Serial.print(timeClient.getFormattedTime());
    Serial.print(" ");
    Serial.println("MQTT not connected!");
    flashError(1000, 6, LED_USER);
    delay(5000);
    return false;
  }

  digitalWrite(LED_USER, HIGH) ;
  Serial.print(timeClient.getFormattedTime());
  Serial.print(" ");
  Serial.println("We are connected to the MQTT broker!");
  Serial.println();

  flashError(200, 3, LED_USER);
/*
  Subscript to toptic here
*/

  return true;
}