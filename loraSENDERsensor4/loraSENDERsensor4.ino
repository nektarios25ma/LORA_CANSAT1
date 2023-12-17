#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "DFRobot_GNSS.h"

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
int Dummyvalue;
long randNumber; //Create Random Number To Avoid Transmission Loss For First Digit

void setup() {
  Serial.begin(9600);
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

  Serial.println("LoRa Sender");
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
    
//    Serial.println("-- Default Test --");
    delayTime = 100;
    LoRa.setSpreadingFactor(10); 
}

void loop() {
//sTim_t utc = gnss.getUTC();
//  sTim_t date = gnss.getDate();
  sLonLat_t lat = gnss.getLat();
  sLonLat_t lon = gnss.getLon();
  double high = gnss.getAlt();
//  uint8_t starUserd = gnss.getNumSatUsed();
//  double sog = gnss.getSog();
//  double cog = gnss.getCog();



  randNumber = random(1000);
  int randNumber = random(100); Dummyvalue = randNumber;
// double ldrvalue = analogRead(ldr);
 //float h = bme.readHumidity();
 float t = bme.readTemperature();
// float f = dht.readTemperature(true);
 float pressure = (bme.readPressure() / 100.0F); Pressure = pressure;
 int altitude =  (bme.readAltitude(SEALEVELPRESSURE_HPA));; Altitude = altitude; 
// String Datastring = String(Dummyvalue) + ("; Temperature") + String(t) + ("; Humidity") + String(h) + ("; Pressure") + String (Pressure) + ("; Altitude") + String (Altitude);
 //Serial.println(Datastring);
String Datastring =("  Temp") + String(t) + ("; 1Alt ") + String(high) + ("; Press") + String (Pressure) + ("; 2Alt ") + String (Altitude)+("; lat degree =")+String(lat.latitudeDegree)+("; lon degree =")+String(lon.lonitudeDegree);
 Serial.println(Datastring);
//  Serial.print("Sending packet: ");
//  Serial.println(counter);
// send packet
  LoRa.beginPacket();
  LoRa.print(counter);
  LoRa.print(Datastring);
 // LoRa.print("   ");
//  LoRa.print(counter);
  LoRa.endPacket();
  counter++;
  //delay(500);

 //   delay(delayTime);
}
