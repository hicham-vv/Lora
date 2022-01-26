
//*********************Sender***************//
#include <stdio.h>
#include <stdlib.h>


#include <Layer1_LoRa.h>
#include <LoRaLayer2.h>

#define LORA_CS 4
#define LORA_RST 22
#define LORA_IRQ 21
// #define LORA_FREQ 915E6
#define LORA_FREQ 868E6
#define LED 2
#define TX_POWER 20

// #define Sender
#define Receiver
// #define GetMacAdress



#ifdef Sender

char MAC[9] = "bf8660c0";

// uint8_t LOCAL_ADDRESS[ADDR_LENGTH] = {0xbf, 0x86, 0x61, 0x4c};
uint8_t RECEIVER[ADDR_LENGTH] = {0xbf, 0x86, 0x61, 0x08};

 // bf:86:61:08 Receiver MAC Adress 


Layer1Class *Layer1;
LL2Class *LL2;

int counter = 0;
long lastTransmit;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  delay(2000);
  pinMode(26, OUTPUT); //EN 3.3V
  digitalWrite(26, HIGH);

  SPI.begin(18, 19, 23, 4); // 

  Serial.println("* Initializing LoRa...");
  Serial.println("LoRa Sender");

  Layer1 = new Layer1Class();
  Layer1->setPins(LORA_CS, LORA_RST, LORA_IRQ);
  Layer1->setTxPower(TX_POWER);
  Layer1->setLoRaFrequency(LORA_FREQ);
  if (Layer1->init())
  {
    Serial.println(" --> Layer1 initialized");
    LL2 = new LL2Class(Layer1); // initialize Layer2
    LL2->setLocalAddress(MAC); // this should either be randomized or set using the wifi mac address
    LL2->setInterval(10000); // set to zero to disable routing packets
    if (LL2->init() == 0){
      Serial.println(" --> LoRaLayer2 initialized");
    }
    else{
      Serial.println(" --> Failed to initialize LoRaLayer2");
    }
  }
  else
  {
    Serial.println(" --> Failed to initialize LoRa");
  }
  lastTransmit = Layer1Class::getTime();
}

void loop() {
  LL2->daemon();
  int msglen = 0;
  int datagramsize = 0;

  struct Datagram datagram; 
  if (Layer1Class::getTime() - lastTransmit >= 5000){
    msglen = sprintf((char*)datagram.message, "%s,%i", "Lora", counter);

    memcpy(datagram.destination, RECEIVER, ADDR_LENGTH);
    datagram.type = 's'; // can be anything, but 's' for 'sensor'
    Serial.print("DATALen= ");
    Serial.println(msglen);
    datagramsize = msglen + HEADER_LENGTH;
    Serial.println(datagramsize);
    LL2->writeData(datagram, datagramsize);
    counter++;
    Serial.println((char*)datagram.message);
    lastTransmit = Layer1Class::getTime();
    Serial.println();
  }

}

#endif




#ifdef Receiver

//********************Receiver*********************//
#include <stdio.h>
#include <stdlib.h>

//LoRaLayer2
#include <Layer1_LoRa.h>
#include <LoRaLayer2.h>
// #define LL2_DEBUG

#define LORA_CS 4
#define LORA_RST 22
#define LORA_IRQ 21


#define LORA_FREQ 868E6
#define LED 2
#define TX_POWER 20

char MAC[9] = "bf866108";
// uint8_t LOCAL_ADDRESS[ADDR_LENGTH] = {0xbf, 0x86, 0x60, 0xb8};
// GATEWAY is the receiver 
// uint8_t SENDER[ADDR_LENGTH] = {0xc0, 0xd3, 0xf0, 0x0d};

Layer1Class *Layer1;
LL2Class *LL2;

int counter = 0;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  pinMode(26, OUTPUT); //EN 3.3V

  digitalWrite(26, HIGH);


  SPI.begin(18, 19, 23, 4); // not needed for ttgo-lora32-v1
  Serial.println("* Initializing LoRa...");
  Serial.println("LoRa Receiver");

  Layer1 = new Layer1Class();
  Layer1->setPins(LORA_CS, LORA_RST, LORA_IRQ);
  Layer1->setTxPower(TX_POWER);
  Layer1->setLoRaFrequency(LORA_FREQ);
  if (Layer1->init())
  {
    Serial.println(" --> Layer1 initialized");
    LL2 = new LL2Class(Layer1); // initialize Layer2
    LL2->setLocalAddress(MAC); // this should either be randomized or set using the wifi mac address
    LL2->setInterval(1); // set to zero to disable routing packets
    if (LL2->init() == 0){
      Serial.println(" --> LoRaLayer2 initialized");
    }
    else{
      Serial.println(" --> Failed to initialize LoRaLayer2");
    }
  }
  else
  {
    Serial.println(" --> Failed to initialize LoRa");
  }
}

void loop() {

  char routes[256];
  char neighbors[256];

  LL2->daemon();

  struct Packet packet = LL2->readData();
  if(packet.totalLength > HEADER_LENGTH)
  {
    Serial.println(((char *)packet.datagram.message));

    // LL2->getRoutingTable(routes);
    // Serial.printf("%s", routes);

    LL2->getNeighborTable(neighbors);
    Serial.println(neighbors);
  }
}

#endif






#ifdef GetMacAdress

//***************GET MAC Address*******************//
// #include <SPI.h>
#ifdef ESP32
  #include <WiFi.h>
#else
  #include <ESP8266WiFi.h>
#endif

void setup(){
  Serial.begin(115200);
  Serial.println();
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
}

void loop(){

}

#endif