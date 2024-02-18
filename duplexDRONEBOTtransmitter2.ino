#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "DFRobot_GNSS.h"
#include <Servo.h> // Συμπερίλαβε τη βιβλιοθήκη για εντολές σερβομηχανισμού
Servo myservo; // Δημιούργησε ένα αντικείμενο τύπου Servo


DFRobot_GNSS_I2C gnss(&Wire ,GNSS_DEVICE_ADDR);
// uncomment the section corresponding to your board
// BSFrance 2017 contact@bsrance.fr 

//  //LoR32u4 433MHz V1.0 (white board)
//  #define SCK     15
//  #define MISO    14
//  #define MOSI    16
//  #define SS      1
//  #define RST     4
//  #define DI0     7
//  #define BAND    433E6 
//  #define PABOOST true

//  //LoR32u4 433MHz V1.2 (white board)
  #define SCK     15
  #define MISO    14
  #define MOSI    16
  #define SS      8
  #define RST     4
  #define DI0     7
  #define BAND    433E6 
  #define PABOOST true 


unsigned long myTime;
  //LoR32u4II 868MHz or 915MHz (black board)
 // #define SCK     15
 // #define MISO    14
 // #define MOSI    16
 // #define SS      8
 // #define RST     4
 // #define DI0     7
 // #define BAND    868E6  // 915E6
 // #define PABOOST true 
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
int counter = 0;
unsigned long delayTime;
float Pressure;
float Altitude;
//int ldr = 4;
//int Red_Led = 6;
//int counter = 0;
//int Dummyvalue;
//long randNumber; //Create Random Number To Avoid Transmission Loss For First Digit
const int ledPin = 11;
volatile int ledPin2 = 6;
float SEC;
// Outgoing message variable
String outMessage;
String incoming;
// Message counter
byte msgCount = 0;

// Receive message variables
// Receive message variables
String contents = "";
//String buttonPressL = "button pressedl";
//bool rcvButtonStatel;
String buttonPress = "button pressed";
volatile bool rcvButtonState;
//String buttonPressS = "button presseds";
//bool rcvButtonStates;


// Source and destination addresses
//byte localAddress = 0xFF;  // address of this device
//byte destination = 0xBB;   // destination to send to

// Pushbutton variables
// Pushbutton variables

//int sendButtonStateL;
volatile int sendButtonState;
//int sendButtonStateS;


void setup() {
delay(2000);
  myservo.attach(9);
// Set LED as output
 pinMode(ledPin, OUTPUT);
 pinMode(ledPin2, OUTPUT);  
  delay(1000);
 // Serial.begin(9600);
  Serial1.begin(9600);
//  while(!gnss.begin()){
//    Serial.println("NO DevicesGPS !");
//    delay(1000);
//    }

  gnss.enablePower(); 
 gnss.setGnss(eGPS_BeiDou_GLONASS);

  gnss.setRgbOn();

 //  while (!Serial);
unsigned status;
randomSeed(analogRead(0));    
    // default settings
    status = bme.begin();  
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)

 // Serial.println("LoRa Sender");
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND)) {                      //!LoRa.begin(BAND,PABOOST
    Serial.println("Starting LoRa failed!");
    while (1);
  }
// if (!status) {
 //       Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
//        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
//        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
//        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
//        Serial.print("        ID of 0x60 represents a BME 280.\n");
//        Serial.print("        ID of 0x61 represents a BME 680.\n");
 //       while (1) delay(10);
//    }
  
 // myservo.write(90);
 // delay(100);
 
//    Serial.println("-- Default Test --");
    delayTime = 100;
    LoRa.setSpreadingFactor(10);
 //   pinMode(11, OUTPUT); 
//     pinMode(12, OUTPUT); 
     LoRa.setSyncWord(0xF3); 



 // Serial.println("LoRa Duplex with callback");

  // Set Receive Call-back function
  LoRa.onReceive(onReceive);

  // Place LoRa in Receive Mode
  LoRa.receive();
myservo.write(0);

  
 // Serial.println("LoRa init succeeded.");
  delay(2500);
  myservo.write(90);
 
}

void loop() {
myTime = millis();
SEC=myTime/1000;

//sTim_t utc = gnss.getUTC();
//  sTim_t date = gnss.getDate();
  sLonLat_t lat = gnss.getLat();
  sLonLat_t lon = gnss.getLon();
//  double high = gnss.getAlt();
//  uint8_t starUserd = gnss.getNumSatUsed();
//  double sog = gnss.getSog();
//  double cog = gnss.getCog();


 //float h = bme.readHumidity();
 float t = bme.readTemperature();
// float f = dht.readTemperature(true);
 float pressure = (bme.readPressure() / 100.0F); Pressure = pressure;
 int altitude =  (bme.readAltitude(SEALEVELPRESSURE_HPA));; Altitude = altitude; 
// String Datastring = String(Dummyvalue) + ("; Temperature") + String(t) + ("; Humidity") + String(h) + ("; Pressure") + String (Pressure) + ("; Altitude") + String (Altitude);
 //Serial.println(Datastring);
//String Datastring =("  ;SEC ")+String(SEC)+(";  Temp") + String(t) + ("; 1Alt ") + String(high) + ("; Press") + String (Pressure) + ("; 2Alt ") + String (Altitude)+("; lat degree =")+String(lat.latitudeDegree)+("; lon degree =")+String(lon.lonitudeDegree);

 String Datastring =("  ;SEC ")+String(SEC)+(";  Temp") + String(t) + ("; 2Alt ") + String(Altitude) + ("; Press") + String (Pressure) + ("; lat degree =")+String(lat.latitudeDegree)+("; lon degree =")+String(lon.lonitudeDegree);
 //Serial.println(Datastring);
 Serial1.println(Datastring);
//  Serial.print("Sending packet: ");
//  Serial.println(counter);
// send packet
outMessage = Datastring; 
sendMessage(outMessage);
  //delay(500);
if(SEC>250){analogWrite(13, 255);}
// Toggle button state
// Set Receive Call-back function
  //LoRa.onReceive(onReceive);

  // Place LoRa in Receive Mode
 LoRa.receive();
  
 delay(500);  
 
}

// Send LoRa Packet
void sendMessage(String Datastring) {
  LoRa.beginPacket();             // start packet
 // LoRa.write(destination);        // add destination address
//  LoRa.write(localAddress);       // add sender address
//  LoRa.write(msgCount);           // add message ID
//  LoRa.write(outgoing.length());  // add payload length
  LoRa.print(Datastring);           // add payload
  LoRa.endPacket();               // finish packet and send it
//  msgCount++;                     // increment message ID
}

// Receive Callback Function
void onReceive(int packetSize) {
 // pinMode(ledPin2, OUTPUT);   
 // myservo.attach(9);
  if (packetSize == 0) return;  // if there's no packet, return

  // Read packet header bytes:
//  int recipient = LoRa.read();        // recipient address
//  byte sender = LoRa.read();          // sender address
//  byte incomingMsgId = LoRa.read();   // incoming msg ID
//  byte incomingLength = LoRa.read();  // incoming msg length

  String incoming = "";  // payload of packet

  while (LoRa.available()) {        // can't use readString() in callback, so
    incoming += (char)LoRa.read();  // add bytes one by one
  }

//  if (incomingLength != incoming.length()) {  // check length for error
 //   Serial.println("error: message length does not match length");
 //   return;  // skip rest of function
 // }

  // If the recipient isn't this device or broadcast,
 // if (recipient != localAddress && recipient != 0xFF) {
 //   Serial.println("This message is not for me.");
//    return;  // skip rest of function
//  }

  // If message is for this device, or broadcast, print details:
//  Serial1.println("Received from: 0x" + String(sender, HEX));
//  Serial1.println("Sent to: 0x" + String(recipient, HEX));
//  Serial1.println("Message ID: " + String(incomingMsgId));
//  Serial1.println("Message length: " + String(incomingLength));
  Serial1.println("Message: " + incoming);                   //-----------------------------------------
//  Serial1.println("RSSI: " + String(LoRa.packetRssi()));
//  Serial1.println("Snr: " + String(LoRa.packetSnr()));
//  Serial1.println();

// Toggle button state
  if (incoming.equals(buttonPress)) {
    rcvButtonState = !rcvButtonState;
  }
 
  // Drive LED
  if (rcvButtonState == true) {
  // digitalWrite(ledPin, HIGH);
   digitalWrite(ledPin2, HIGH);
    Serial1.println("led on");
   myservo.write(0);
 
  //   analogWrite(13, 255);
  //  delay(19); //yes almost 0.5 cycle
  //  myservo.write(90); 
  }
   else 
   { //myservo.write(180);
 //    analogWrite(13, 255); 
 //digitalWrite(ledPin, LOW);
 digitalWrite(ledPin2, LOW);
  Serial1.println("led off");
 //   delay(9); //yes almost 0.5 cycle
  //  myservo.write(90);
   }
   //myservo.write(90);
  
 



}
