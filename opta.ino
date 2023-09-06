
#include <SPI.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <SSLClient.h>
#include <ArduinoMqttClient.h>
#include <Arduino_ConnectionHandler.h>
#include <ArduinoJson.h>
#include "trust_anchors.h"
#include "secrets.h"

const char broker[] = SECRET_MQTT_BROKER;
int        port     = 8883;
const char topic[]  = "anders/hej";
const char configtopic[]  = "anders/config";

const long send_interval = 60000;
unsigned long previous_send = 0;

const long check_interval = 1000;
unsigned long previous_check = 0;

const long wifi_interval = 60000;
unsigned long previous_wifi = 0;

const long mqttconn_interval = 30000;
unsigned long previous_mqttconn = 0;

const long serial_interval = 10000;
unsigned long previous_serial = 0;


int count = 0;

bool status[6] = {0, 0, 0, 0, 0, 0};
bool g_status[6] = {0, 0, 0, 0, 0, 0};
unsigned long senaste_andring[6] = {0, 0, 0, 0, 0, 0};
unsigned long g_statustid[6] = {0, 0, 0, 0, 0, 0};
bool nyinfo = 0;

WiFiConnectionHandler conMan(SECRET_SSID, SECRET_WEPKEY);

SSLClient wifiSSLClient(conMan.getClient(), TAs, (size_t)TAs_NUM, A5);
MqttClient mqttClient(wifiSSLClient);
NTPClient timeClient(conMan.getUDP());

void setup() {
  Serial.begin(9600);
  delay(1000);

  Serial.println("Hello world");

  pinMode(LED_D0, OUTPUT);
  pinMode(LED_D1, OUTPUT);
  pinMode(LED_D2, OUTPUT); // main loop
  pinMode(LED_D3, OUTPUT); // 
  pinMode(LED_USER, OUTPUT); // mqtt
  pinMode(LED_RESET, OUTPUT); // wifi

  conMan.addCallback(NetworkConnectionEvent::CONNECTED, onNetworkConnect);
  conMan.addCallback(NetworkConnectionEvent::DISCONNECTED, onNetworkDisconnect);
  conMan.addCallback(NetworkConnectionEvent::ERROR, onNetworkError);

  pinMode(PIN_A0, INPUT); // Tryckvakt
  pinMode(PIN_A1, INPUT); // Matarpump
  pinMode(PIN_A2, INPUT); // Tanksensor
  pinMode(PIN_A3, INPUT); // Högtryckspump
  pinMode(PIN_A4, INPUT); // Backspolning
  pinMode(PIN_A5, INPUT); // Larm

  timeClient.begin();

  mqttClient.setUsernamePassword(SECRET_MQTT_USER, SECRET_MQTT_PASS);

}



void loop() {
  digitalWrite(LED_D2, HIGH);
  delay(10);
  digitalWrite(LED_D2, LOW);

  unsigned long ms = millis();

  digitalWrite(LED_D0, LOW);
  digitalWrite(LED_D1, LOW);

  if (ms - previous_wifi >= wifi_interval) {
    conMan.check();
    previous_wifi = ms;
  }
  
  if (ms - previous_serial >= serial_interval) {
    if (!Serial) {
      Serial.begin(9600);
    }
    previous_serial = ms;
  }

  timeClient.update();

  updateInputs();

  if (mqttClient.connected()) {
    digitalWrite(LED_USER, HIGH);
    
    if (ms - previous_check >= check_interval) {
      digitalWrite(LED_D0, HIGH);
      previous_check = ms;
      mqttClient.poll();
      checkMessages();
    }

    if ((ms - previous_send >= send_interval ) || nyinfo == 1 ) {
      digitalWrite(LED_D1, HIGH);
      previous_send = ms;

      String output = createStatusMessage();
      Serial.print(timeClient.getFormattedTime());
      Serial.print(" ");
      Serial.print("Sending message to topic: ");
      Serial.print(topic);
      Serial.print(":");
      Serial.println(output);

      mqttClient.beginMessage(topic);
      mqttClient.print(output);
      mqttClient.endMessage();

      count++;
    }
  }
  else {
    digitalWrite(LED_USER, LOW);
    if (ms - previous_mqttconn >= mqttconn_interval) {
    connectMQTT();
    previous_mqttconn = ms;
  }
  }
  
  delay(50);
}

void updateInputs() {
  nyinfo = 0;
  for (int x = 0; x < 6; x++) {
    g_status[x] = status[x];
  }

  status[0] = digitalRead(PIN_A0);
  status[1] = digitalRead(PIN_A1);
  status[2] = digitalRead(PIN_A2);
  status[3] = digitalRead(PIN_A3);
  status[4] = digitalRead(PIN_A4);
  status[5] = digitalRead(PIN_A5);
  
  unsigned long tid = timeClient.getEpochTime();

  for (int x = 0; x < 6; x++) {
    if (g_status[x] != status[x]) {
      if (senaste_andring[x] > 0) {
        g_statustid[x] = tid - senaste_andring[x];
      }

      senaste_andring[x] = tid;
      nyinfo = 1;
    }
  }

 
}

String createStatusMessage() {
  const int capacity (JSON_ARRAY_SIZE(6) + (6 * JSON_OBJECT_SIZE(3)));
  StaticJsonDocument<capacity> doc;
  String output;

  JsonObject obj1 =  doc.createNestedObject();
  obj1["k"] = "tv"; // Tryckvakt
  obj1["s"] = status[0];
  obj1["c"] = senaste_andring[0];
  obj1["t"] = g_statustid[0];

  JsonObject obj2 = doc.createNestedObject();
  obj2["k"] = "mp"; // Matarpump
  obj2["s"] = status[1];
  obj2["c"] = senaste_andring[1];
  obj2["t"] = g_statustid[1];

  JsonObject obj3 = doc.createNestedObject();
  obj3["k"] = "ts"; // Tanksensor
  obj3["s"] = status[2];
  obj3["c"] = senaste_andring[2];
  obj3["t"] = g_statustid[2];

  JsonObject obj4 = doc.createNestedObject();
  obj4["k"] = "hp"; // högtryckspump
  obj4["s"] = status[3];
  obj4["c"] = senaste_andring[3];
  obj4["t"] = g_statustid[3];
  
  JsonObject obj5 = doc.createNestedObject();
  obj5["k"] = "bs"; // högtryckspump
  obj5["s"] = status[4];
  obj5["c"] = senaste_andring[4];
  obj5["t"] = g_statustid[4];

  JsonObject obj6 = doc.createNestedObject();
  obj5["k"] = "al"; // alarm
  obj5["s"] = status[5];
  obj5["c"] = senaste_andring[5];
  obj5["t"] = g_statustid[5];

  serializeJson(doc, output);

  return output;
}

void checkMessages() {
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
}


void onNetworkConnect() {
  digitalWrite(LED_RESET, HIGH);
  Serial.println(">>>> CONNECTED to network");
  connectMQTT();
}

void onNetworkDisconnect() {
  digitalWrite(LED_RESET, LOW);
  Serial.println(">>>> DISCONNECTED from network");
}

void onNetworkError() {
  digitalWrite(LED_RESET, LOW);
  Serial.println(">>>> ERROR");
}

void connectMQTT() {
  if (conMan.getStatus() != NetworkConnectionState::CONNECTED) { 
    return; 
  }
  
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    return;
  }

  Serial.println("We are connected to the MQTT broker!");
  Serial.println();

  Serial.print("Subscribing to topic: ");
  Serial.println(configtopic);
  Serial.println();

  mqttClient.subscribe(configtopic);


  Serial.print("Waiting for messages on topic: ");
  Serial.println(configtopic);
  Serial.println();
}