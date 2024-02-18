#include <SPI.h>
#include <LoRa.h>

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


// LED connection
//const int ledPinblue = 10;
//const int ledPinred = 9;

// Outgoing message variable
String outMessage;

// Message counter
//byte msgCount = 0;

// Receive message variables
String contents = "";
String buttonPress = "button pressed";
bool rcvButtonState;
//String buttonPressR = "button pressedr";
//bool rcvButtonStater;
//String buttonPressS = "button presseds";
//bool rcvButtonStates;
// Source and destination addresses
//byte localAddress = 0xBB;  // address of this device
//byte destination = 0xFF;   // destination to send to

// Pushbutton variables
//int buttonPinL = 9;
//int sendButtonState;
int buttonPin = 10;
int sendButtonState;
//int buttonPinS = 11;
//int sendButtonStateS;

void setup() {
// Set pushbutton as input
   pinMode(buttonPin, INPUT_PULLUP);
Serial.begin(9600);
  while (!Serial);
  Serial.println("LoRa Receiver");
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND)) {                         //if (!LoRa.begin(BAND,PABOOST )) {
    Serial.println("Starting LoRa failed!");
    while (1);
  } 
  LoRa.setSpreadingFactor(10);                  //------------------------------------------------------------------
   LoRa.setSyncWord(0xF3);


  // Set pushbutton as input
 //  pinMode(buttonPinL, INPUT_PULLUP);
 
 // pinMode(buttonPinS, INPUT_PULLUP);

  // Set LED as output
  //pinMode(ledPin, OUTPUT);
 // pinMode(ledPin, OUTPUT);


  Serial.println("LoRa Duplex with callback");

  // Set Receive Call-back function
  LoRa.onReceive(onReceive);

  // Place LoRa in Receive Mode
  LoRa.receive();

  Serial.println("LoRa init succeeded.");
}

void loop() {

  // Get pushbutton state
//  sendButtonStateL = digitalRead(buttonPinL);
  sendButtonState = digitalRead(buttonPin);
//  sendButtonStateS = digitalRead(buttonPinS);

  // Send packet if button pressed
  if (sendButtonState == LOW) {

    // Compose and send message
    outMessage = buttonPress;
    sendMessage(outMessage);
    delay(10);

    // Place LoRa back into Receive Mode
  //  LoRa.receive();
  // outMessage ="";

  }
  // Send packet if button pressed
  /*if (sendButtonStateR == LOW) {

    // Compose and send message
    outMessage = buttonPressR;
    sendMessage(outMessage);
    delay(33);

    // Place LoRa back into Receive Mode
    LoRa.receive();
    outMessage ="";

  }
  // Send packet if button pressed
  if (sendButtonStateS == LOW) {

    // Compose and send message
    outMessage = buttonPressS;
    sendMessage(outMessage);
    delay(13);

    // Place LoRa back into Receive Mode
    LoRa.receive();
  //  outMessage ="";

  }*/
  LoRa.receive();
  delay(960);
 //  outMessage ="";
}

// Send LoRa Packet
void sendMessage(String outgoing) {
  LoRa.beginPacket();             // start packet
 // LoRa.write(destination);        // add destination address
 // LoRa.write(localAddress);       // add sender address
 // LoRa.write(msgCount);           // add message ID
//  LoRa.write(outgoing.length());  // add payload length
  LoRa.print(outgoing);           // add payload
  LoRa.endPacket();               // finish packet and send it
 // msgCount++;                     // increment message ID
}

// Receive Callback Function
void onReceive(int packetSize) {
  if (packetSize == 0) return;  // if there's no packet, return

  // Read packet header bytes:
  //int recipient = LoRa.read();        // recipient address
 // byte sender = LoRa.read();          // sender address
 // byte incomingMsgId = LoRa.read();   // incoming msg ID
 // byte incomingLength = LoRa.read();  // incoming msg length

  String incoming = "";  // payload of packet

  while (LoRa.available()) {        // can't use readString() in callback, so
    incoming += (char)LoRa.read();  // add bytes one by one
  }

 // if (incomingLength != incoming.length()) {  // check length for error
 //   Serial.println("error: message length does not match length");
 //   return;  // skip rest of function
//  }

  // If the recipient isn't this device or broadcast,
 // if (recipient != localAddress && recipient != 0xFF) {
//    Serial.println("This message is not for me.");
 //   return;  // skip rest of function
 // }

  // If message is for this device, or broadcast, print details:
 // Serial.println("Received from: 0x" + String(sender, HEX));
//  Serial.println("Sent to: 0x" + String(recipient, HEX));
//  Serial.println("Message ID: " + String(incomingMsgId));
//  Serial.println("Message length: " + String(incomingLength));
  Serial.println(incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
//  Serial.println("Snr: " + String(LoRa.packetSnr()));
//  Serial.println();


}