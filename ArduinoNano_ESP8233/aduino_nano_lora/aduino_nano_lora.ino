#include<SPI.h>
#include<LoRa.h>
#include<TinyGPS++.h>
#include<SoftwareSerial.h>

//LoRa pins
#define SS 10
#define RST 9
#define DIO0 2

//HC-SR04 pins
#define TRIG_PIN 5
#define ECHO_PIN 6

//GPS pins
#define S_RX 7
#define S_TX 8

//Battery pin
#define BATTERY_PIN A0

byte msgCount = 0;            // count of outgoing messages
byte MasterNode = 0xFF;
byte Node1 = 0xBB;

String Mymessage = "";


float valueSensor = 40;
int valueBattery = 69;
String valueGPS = "20.942249/106.059653";

TinyGPSPlus gps;
SoftwareSerial serialGPS(S_RX,S_TX);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  serialGPS.begin(9600);
  while (!serialGPS);
  Serial.println("Serial GPS begin sucsses !!!");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(433E6)) { //915E6 or 433E6
    Serial.println("LoRa begin failed !!!");
    while (1);
  }
  Serial.println("LoRa-1 sender begin sucsses !!!");
}


void loop() {
  getValueBattery();
  getValueSensor();
  getValueGPS();
  // parse for a packet, and call onReceive with the result: 
  onReceive(LoRa.parsePacket());
  delay(300);
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return
  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length
  String incoming = "";
  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }
  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("Error length !!!");
    // Serial.println("error: message length does not match length");
    ;
    return;                             // skip rest of function
  }
  // if the recipient isn't this device or broadcast,
  if (recipient != Node1 && recipient != MasterNode) {
    Serial.println("Error address !!!");
    //Serial.println("This message is not for me.");
    ;
    return;                             // skip rest of function
  }
  
  int Val = incoming.toInt();
  if (Val == 10){
    Serial.println("Message in: " + incoming);
    Mymessage = Mymessage + valueSensor + "," + valueBattery + "," + valueGPS;
    sendMessage(Mymessage, MasterNode, Node1);
    delay(100);
    Mymessage = "";
  }
}

void sendMessage(String outgoing, byte MasterNode, byte Node1) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(MasterNode);              // add destination address
  LoRa.write(Node1);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

void getValueSensor(){
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  int duration = pulseIn(ECHO_PIN, HIGH);
  valueSensor = (duration / 2) / 29.1;
}

void getValueGPS(){
  while(serialGPS.available() > 0){
    if (gps.encode(serialGPS.read())){
      if(gps.location.isValid()){
        float lt = gps.location.lat();
        float lg = gps.location.lng();
        valueGPS = String(lt,6) + "/" + String(lg,6);
      }
    }
  }
}

void getValueBattery(){
  float rawV = (analogRead(BATTERY_PIN)*4.98)/1024;
  if(rawV > 4.19) valueBattery = 100;
  else if(rawV > 4.0 && rawV < 4.2) valueBattery = 81;
  else if(rawV > 3.9 && rawV < 4.1) valueBattery = 79;
  else if(rawV > 3.8 && rawV < 4.0) valueBattery = 62;
  else if(rawV > 3.7 && rawV < 3.9) valueBattery = 42;
  else if(rawV > 3.6 && rawV < 3.8) valueBattery = 12;
  else if(rawV > 3.5 && rawV < 3.7) valueBattery = 2;
  else if(rawV < 3.6) valueBattery = 0;
}
