#include<ESP8266WiFi.h>
#include<ESP8266WebServer.h>
#include<WiFiManager.h>
#include <PubSubClient.h>

#include<SPI.h>
#include<LoRa.h>

#define SS 15
#define RST 16
#define DIO0 4

//MQTT Broker details
String device_id = "ESP0001";
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_user = "Esp8266Receive";
const char* mqtt_password = "Esp8266Receive";
const char* mqtt_clientId = "ESP_ESP0001";
const char* topic_publish = "DATN10119609_Publish";
const char* topic_subscribe = "DATN10119609_subscribe";

byte MasterNode = 0xFF;
byte Node1 = 0xBB;
byte Node2 = 0xCC;

String SenderNode = "";
String outgoing; 

byte msgCount = 0;            // count of outgoing messages


unsigned long previousMillis = 0;
unsigned long int previoussecs = 0;
unsigned long int currentsecs = 0;
unsigned long currentMillis = 0;
int interval = 1 ; // updated every 1 second
int Secs = 0;

String valueSensor = "";
String valueBattery = "";
String valueGPS = "";

WiFiManager wifiManager;

WiFiClient esp_client;
void callback(char* topic, byte* payload, unsigned int length);
PubSubClient mqtt_client(mqtt_server, mqtt_port, callback, esp_client);

void setup() {
  Serial.begin(115200);
  SetupWiFi();
  Serial.println("LoRa Receive !!!");
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(433E6)) { //915E6 or 433E6
    Serial.println("LoRa Receive begin failed !!!");
    while (1);
  }
  Serial.println("LoRa Receive begin sucsses !!!");
  mqtt_connect();
}

void SetupWiFi() {
  if (!wifiManager.autoConnect("ESP8266 LoRa Receive", "123456789")) {
    delay(3000);
    ESP.reset();
    delay(3000);
  }
}

void mqtt_connect() {
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    Serial.println("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt_client.connect(mqtt_clientId)) {
      Serial.println("MQTT Client Connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void mqtt_publish(char * data) {
  mqtt_connect();
  if (mqtt_client.publish(topic_publish, data))
    Serial.println("Publish \"" + String(data) + "\" ok");
  else
    Serial.println("Publish \"" + String(data) + "\" failed");
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println("");
}


void loop() {
  currentMillis = millis();
  currentsecs = currentMillis / 1000;
  if ((unsigned long)(currentsecs - previoussecs) >= interval) {
    Secs = Secs + 1;
    //Serial.println(Secs);
    if ( Secs >= 11 )
    {
      Secs = 0;
    }
    if ( (Secs >= 1) && (Secs <= 5) )
    {
      String message = "10";
      sendMessage(message, MasterNode, Node1);
    }
    if ( (Secs >= 6 ) && (Secs <= 10))
    {
      String message = "20";
      sendMessage(message, MasterNode, Node2);
    }
    previoussecs = currentsecs;
  }
  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
  if (!mqtt_client.loop())
    mqtt_connect();
}

void sendMessage(String outgoing, byte MasterNode, byte otherNode) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(otherNode);              // add destination address
  LoRa.write(MasterNode);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;  
  Serial.println("Message out: "+ outgoing);// increment message ID
}


void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return
  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  if ( sender == 0XBB )
    SenderNode = "Node1:";
  if ( sender == 0XCC )
    SenderNode = "Node2:";
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length
  String incoming = "";
  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }
  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("Error length !!!");
    Serial.println("Message length in: "+incomingLength);
    Serial.println("Message in: "+incoming);    
    Serial.println("Length: "+String(incoming.length()));
    Serial.println();
    //Serial.println("error: message length does not match length");
    ;
    return;                             // skip rest of function
  }
  // if the recipient isn't this device or broadcast,
  if (recipient != Node1 && recipient != MasterNode) {
    Serial.println("Error address !!!");
    // Serial.println("This message is not for me.");
    ;
    return;                             // skip rest of function
  }

  if (sender == 0XCC ) {
    valueSensor = getValue(incoming, ',', 0);
    valueBattery = getValue(incoming, ',', 1);
    valueGPS = getValue(incoming, ',', 2);
  }
  if (sender == 0XBB ) {
    valueSensor = getValue(incoming, ',', 0);
    valueBattery = getValue(incoming, ',', 1);
    valueGPS = getValue(incoming, ',', 2);
  }
  String pkt = "{";
  pkt += "\"device_id\": " + String(sender) + ", ";
  pkt += "\"sensor\": " + String(valueSensor) + ", ";
  pkt += "\"gps\": \"" + String(valueGPS) + "\", ";
  pkt += "\"battery\": " + String(valueBattery) + "";
  pkt += "}";

  Serial.print("Receive value: ");
  Serial.println(pkt);
  mqtt_publish((char*) pkt.c_str());
}
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
