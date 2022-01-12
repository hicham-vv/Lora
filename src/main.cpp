/*********
  Modified from the examples of the Arduino LoRa library
  More resources: https://randomnerdtutorials.com
*********/

#include <SPI.h>
#include <LoRa.h>

//define the pins used by the transceiver module
#define ss 4
#define rst 22
#define dio0 21

#define pin3V 26

#define pin5V 25



// #define Sensor
// #define Master
#define Slave







#ifdef Slave

unsigned char data[4]={};
float distance; //float distance;
int Measure;


unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
int d=0;
int sum=1;
String Index ="T";


void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial2.begin(9600);
  while (!Serial2);
  
  Serial.println("LoRa Sender");

  
  pinMode(pin3V,OUTPUT);
  digitalWrite(pin3V,HIGH);
  pinMode(pin5V,OUTPUT);
  digitalWrite(pin5V,HIGH);



  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  
  //replace the LoRa.begin(---E-) argument with your location's frequency 
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(866E6)) {
    Serial.print(".");
    delay(500);
  }
   // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xFF);
  Serial.println("LoRa Initializing OK!");
}

void loop() {


  Serial.print("Sending packet: ");

  //Send LoRa packet to receiver
  LoRa.beginPacket();
  LoRa.print("N1:10");
  LoRa.endPacket();
  delay(5000);
}

#endif

#ifdef Master

void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Receiver");

  pinMode(pin3V,OUTPUT);
  digitalWrite(pin3V,HIGH);

  pinMode(pin5V,OUTPUT);
  digitalWrite(pin5V,HIGH);


  
  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  
  //replace the LoRa.begin(---E-) argument with your location's frequency 
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(866E6)) {
    Serial.println(".");
    delay(500);
  }
   // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xFF);
  Serial.println("LoRa Initializing OK!");
  delay(500);
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      String LoRaData = LoRa.readString();
      Serial.println(LoRaData);
    }

    // // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}
#endif


#ifdef Sensor




byte Buffer[5];
unsigned BytesRead = 0;
unsigned DistanceInMM = 0;

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  Serial2.begin(9600);
  while (!Serial2);
  
  Serial.println("LoRa Sender");

  
  pinMode(pin3V,OUTPUT);
  digitalWrite(pin3V,HIGH);
  pinMode(pin5V,OUTPUT);
  digitalWrite(pin5V,HIGH);
}

void loop()
{
  if (Serial2.available())
  {
    // Put the character in the buffer
    Buffer[BytesRead++] = Serial2.read();
    // Validate the input so far:
    switch (BytesRead)
    {
      case 1:  // Start character
        if (Buffer[0] != 0xFF)
        {
          Serial.print("Invalid start character: ");
          Serial.println(Buffer[0], HEX);
          BytesRead = 0; // Start over
        }
        break;

      case 4:  // Checksum
        byte sum;  // DO NOT INITIALIZE LOCAL VARIABLES IN A 'case' CLAUSE.
        sum = Buffer[0] + Buffer[1] + Buffer[2];
        if (sum != Buffer[3])
        {
          Serial.print("Invalid checksum: 0xFF + 0x");
          Serial.print(Buffer[1], HEX);
          Serial.print(" + 0x");
          Serial.print(Buffer[2], HEX);
          Serial.print(" != 0x");
          Serial.println(Buffer[3], HEX);
          BytesRead = 0; // Start over
        }
        break;

      case 5: // End character
        if (Buffer[4] != 0xFF)
        {
          Serial.print("Invalid end character: ");
          Serial.println(Buffer[4], HEX);
          BytesRead = 0; // Start over
        }
        break;
    }
  }

  if (BytesRead == 5)
  {
    DistanceInMM = Buffer[1] * 256 + Buffer[2];
    BytesRead = 0; // Look for a new start next time

    if (DistanceInMM > 30)
    {
      Serial.print("distance=");
      Serial.print(DistanceInMM / 10.0, 1); // Convert to cm and show with 1 decimal place
      Serial.println("cm");
    }
    else
    {
      Serial.println("Below the lower limit");
    }
  }

  delay(5);
}
#endif