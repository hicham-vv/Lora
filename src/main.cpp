


#define Sender
// #define Master




#ifdef Sender
//******************************** LORA MESH CLIENT ************************************//
#include <SPI.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#include <EEPROM.h>

#define EEPROM_SIZE 1 // ESP32 max 512, Arduino Uno max 1024

// #define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
// #define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */

// ESP32
#define RFM95_CS 4
#define RFM95_RST 22
#define RFM95_INT 21 //*/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 868.0

int nodeIdSelf;
int nodeIdDestination = 3;
int startTimer;

// Singleton instance of the radio driver
RH_RF95 driver(RFM95_CS, RFM95_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHMesh *manager;

String message;
String messageResponse = "Alert Received!";

// ----------------------
// SETUP
// === LoRa
void setup_lora() {
//   driver.setTxPower(23);
// //   driver.setTxPower(23, false);
//   driver.setSpreadingFactor(7);
//   driver.setFrequency(RF95_FREQ);
//   driver.setCADTimeout(500);

  //  if (!driver.init()) {
  //   Serial.println("LoRa radio init failed");
  //   Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
  //   // while (1);
  // }
  // else{
  //   Serial.println("LoRa radio init OK!");
  // }
  // if (!driver.setFrequency(RF95_FREQ)) {
  //   Serial.println("setFrequency failed");
  //   // while (1);
  // }
  // else{
  //   Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  // }
  // driver.setSpreadingFactor(7);
  // driver.setTxPower(23);
  // driver.setCADTimeout(500);

  // EEPROM.begin(EEPROM_SIZE); 
  // nodeIdSelf = EEPROM.read(0);
  nodeIdSelf = 1;


  manager = new RHMesh(driver, nodeIdSelf);
  
  if (!manager->init()) {
      Serial.println(F("init failed"));
  } else {
      Serial.println("Mesh Node \"" + (String) nodeIdSelf + "\" Up and Running!");
  }

  driver.setTxPower(23, false);
  driver.setFrequency(RF95_FREQ);
  driver.setCADTimeout(500);
}

// ----------------------
// ACTIONS
// === LoRa
uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];

void sendMessage(String _message, int _nodeIdDestination, int _sendMessageCount) {
  for (int i=0; i<_sendMessageCount; i++) {
    Serial.println("Sending to RF95 Mesh Node \"" + (String) _nodeIdDestination + "\"!");
    // Send a message to a rf95_mesh_node
    // A route to the destination will be automatically discovered.

    char messageChar[_message.length() + 1];
    strcpy(messageChar, _message.c_str());
    Serial.println("START1");
    // Serial.println(_message);
    // Serial.println(messageChar);
    int errorLog = manager->sendtoWait((uint8_t*) messageChar, sizeof(messageChar), nodeIdDestination);
    Serial.println("START2");
    if (errorLog == RH_ROUTER_ERROR_NONE)
    {
      Serial.println("START3");
      // It has been reliably delivered to the next node.
      // Now wait for a reply from the ultimate server
      uint8_t len = sizeof(buf);
      uint8_t from;    
      if (manager->recvfromAckTimeout(buf, &len, 3000, &from))
      {
        Serial.print("Got Reply from ");
        Serial.print(from, HEX);
        Serial.println(":");
        Serial.println((char*)buf);
        Serial.println("lastRssi = " + (String) driver.lastRssi());
      }
      else
      {
        Serial.println("No reply, is rf95_mesh_node1, rf95_mesh_node2 and rf95_mesh_node3 running?");
      }
    }
    else
       Serial.println("sendtoWait failed. Are the intermediate mesh nodes running?");
  }
}

void listen_lora() {
  uint8_t len = sizeof(buf);
  uint8_t nodeIdFrom;
//   Serial.println("Listen-LORA");
//   delay(1000);
  if (manager->recvfromAck(buf, &len, &nodeIdFrom))  // if (manager->recvfromAckTimeout(buf, &len, 3000, &nodeIdFrom))
  {
    Serial.print("Got Message nodeIdFrom ");
    Serial.print(nodeIdFrom, HEX);
    Serial.println(":");
    Serial.println((char*)buf);

    Serial.println("lastRssi = " + (String) driver.lastRssi());
    
    // Send a reply back to the originator client
    char messageResponseChar[messageResponse.length() + 1];
    strcpy(messageResponseChar, messageResponse.c_str());
    if (manager->sendtoWait((uint8_t*) messageResponseChar, sizeof(messageResponseChar), nodeIdFrom) != RH_ROUTER_ERROR_NONE)
      Serial.println("sendtoWait failed");
  }
}

void setup() {
    Serial.begin(115200);
    pinMode(26, OUTPUT); //EN 3.3V
    // pinMode(25, OUTPUT); //EN 5V
    digitalWrite(26, HIGH);
    // digitalWrite(25, HIGH);

    // esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

    setup_lora();
    startTimer = millis();
    message = "NODE " + (String) nodeIdSelf; //Hello! from node " + (String) nodeIdSelf + ", to node " + (String) nodeIdDestination
}

void loop() 
{
  if (millis() - startTimer > 5000) {
    startTimer = millis();
    sendMessage(message, nodeIdDestination, 1);
    // //   digitalWrite(25, LOW); //EN 5V
    // digitalWrite(26, LOW); //EN 3.3V
    // esp_deep_sleep_start(); 
  }
  listen_lora();  

}


#endif


#ifdef Master


//******************************** LORA MESH SERVER ************************************//
// PRE-REQUISITES
// === LoRa
#include <SPI.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#include <EEPROM.h>

#define EEPROM_SIZE 1 // ESP32 max 512, Arduino Uno max 1024

// ESP32
#define RFM95_CS 4
#define RFM95_RST 22
#define RFM95_INT 21 //*/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 868.0

int nodeIdSelf;
 
// Singleton instance of the radio driver
RH_RF95 driver(RFM95_CS, RFM95_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHMesh *manager;

String messageResponse = "Alert Received!";

// ----------------------
// SETUP
// === LoRa
void setup_lora() {
//   driver.setSpreadingFactor(7);
  //   driver.setTxPower(23);
// //   driver.setTxPower(23, false);
//   driver.setFrequency(RF95_FREQ);
//   driver.setCADTimeout(500);

  // if (!driver.init()) {
  //   Serial.println("LoRa radio init failed");
  //   Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
  //   // while (1);
  // }
  // else{
  //   Serial.println("LoRa radio init OK!");
  // }
  // if (!driver.setFrequency(RF95_FREQ)) {
  //   Serial.println("setFrequency failed");
  //   // while (1);
  // }
  // else{
  //   Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  // }
  // driver.setSpreadingFactor(7);
  // driver.setTxPower(23);
  // driver.setCADTimeout(500);

  // EEPROM.begin(EEPROM_SIZE);
  // nodeIdSelf = EEPROM.read(0);
  nodeIdSelf = 3;


  manager = new RHMesh(driver, nodeIdSelf);
  if (!manager->init()) {
      Serial.println(F("init failed"));
  } else {
      Serial.println("Mesh Node \"" + (String) nodeIdSelf + "\" Up and Running!");
  }

  driver.setTxPower(23, false);
  driver.setFrequency(RF95_FREQ);
  driver.setCADTimeout(500); 
}

// ----------------------
// ACTIONS
// === LoRa
uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];

void listen_lora() {
  uint8_t len = sizeof(buf);
  uint8_t nodeIdFrom;
  
  if (manager->recvfromAck(buf, &len, &nodeIdFrom))
  {
    Serial.print("Got Message nodeIdFrom ");
    Serial.print(nodeIdFrom, HEX);
    Serial.println(":");
    Serial.println((char*)buf);

    Serial.println("lastRssi = " + (String) driver.lastRssi());
    
    // Send a reply back to the originator client
    char messageResponseChar[messageResponse.length() + 1];
    strcpy(messageResponseChar, messageResponse.c_str());
    if (manager->sendtoWait((uint8_t*) messageResponseChar, sizeof(messageResponseChar), nodeIdFrom) != RH_ROUTER_ERROR_NONE)
      Serial.println("sendtoWait failed");
  }
}

// ----------------------
// MAIN
void setup() {
    Serial.begin(115200);
    pinMode(26, OUTPUT); //EN 3.3V
    // pinMode(25, OUTPUT); //EN 5V
    digitalWrite(26, HIGH);
    // digitalWrite(25, HIGH);
    setup_lora();
}

void loop()
{
  listen_lora();
}

#endif