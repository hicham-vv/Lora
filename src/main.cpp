/*********
  Modified from the examples of the Arduino LoRa library
  More resources: https://randomnerdtutorials.com
*********/
#include<Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <math.h>

#include "esp_task_wdt.h"

#include <SPIFFS.h>

#include <WiFi.h>
#include <HTTPClient.h>


// const char * ssid = "Orange-80C3";
// const char * ssid = "HUAWEI-8e4e";
const char * ssid = "Redmi9TH";


// const char * password = "0GEYH04G5A2";
// const char * password = "ifran123";
const char * password = "luffy123";
// const char * password = "tantan-1";



#define debug

#define Master
// #define Slave
// #define slaveTest


String device = "0"; // write the device Number Here !



// #define HTTPsend
// #define Sensor


String fileName = "";

byte Buffer[5];
unsigned BytesRead = 0;
unsigned DistanceInMM = 0;



unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
int d=0;
int sum=1;
String Index ="T";

bool vDist = false;

bool confirm=false;

uint8_t distance=0;

uint8_t Dr=200; 
int8_t niveauR=0;

uint8_t distanceRef=123;

boolean ledState = false;


const char* serverName = "http://141.94.71.45:8080/datasnap/rest/Tdata/rep";


#define HardReset 14 // Hardware Reset MTMS



//define the pins used by the transceiver module
#define ss 4
#define rst 22
#define dio0 21

#define pin3V 26

#define pin5V 25

#define Led_esp 2

void blinkLed(uint16_t time_Out,uint16_t ms);


void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Disconnected from WiFi access point");
  pinMode(HardReset,OUTPUT);
  delay(500);
  pinMode(HardReset,LOW);
  esp_restart();
}


boolean a= true;

WiFiClient client;
HTTPClient http;

int compteur=0;




#ifdef Slave





void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial2.begin(9600);
  while (!Serial2);
  delay(1000);

  esp_task_wdt_init(15, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  Serial.println("LoRa Sender");

  esp_sleep_enable_timer_wakeup(10000000); // 10S
  // esp_sleep_enable_timer_wakeup(3600000000); // 1hrs




  pinMode(Led_esp,OUTPUT);
  digitalWrite(Led_esp,LOW);
  
  pinMode(pin3V,OUTPUT);
  digitalWrite(pin3V,HIGH);
  pinMode(pin5V,OUTPUT);
  digitalWrite(pin5V,HIGH);



  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  
  //Replace the LoRa.begin(---E-) argument with your location's frequency 
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
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  Serial.println("LoRa Initializing OK!");
  delay(1500);

}

void loop() {

  // for(int i=0;i<3;i++){
      unsigned prevmillis=millis();
      // while((millis()-prevmillis)<2500){
      //   if (Serial2.available())
      //   {
      //     // Put the character in the buffer
      //     Buffer[BytesRead++] = Serial2.read();
      //     // Validate the input so far:
      //     switch (BytesRead)
      //     {
      //       case 1:  // Start character
      //         if (Buffer[0] != 0xFF) // Invalid start character
      //         {
      //           BytesRead = 0; // Start over
      //         }
      //         break;

      //       case 4:  // Checksum
      //         byte sum;  // DO NOT INITIALIZE LOCAL VARIABLES IN A 'case' CLAUSE.
      //         sum = Buffer[0] + Buffer[1] + Buffer[2];
      //         if (sum != Buffer[3])
      //         {
      //           // Serial.println("Invalid checksum: 0xFF + 0x");
      //           BytesRead = 0; // Start over
      //         }
      //         break;
      //       case 5: // End character
      //         if (Buffer[4] != 0xFF)
      //         {
      //           BytesRead = 0; // Start over
      //         }
      //         break;
      //     }
      //   }
      //   if (BytesRead == 5)
      //   {
      //     DistanceInMM = Buffer[1] * 256 + Buffer[2];
      //     BytesRead = 0; // Look for a new start next time
      //     if (DistanceInMM > 30)
      //     {

      //       distance= DistanceInMM / 10;
      //       Serial.print("distance=");
      //       Serial.print(distance);
      //       Serial.println("cm");

      //     }
      //     else
      //     {
      //       Serial.println("distance=3cm");
      //       distance=3;
      //     }
          
      //     if(abs(Dr-distance)>15){
      //       confirm=true;
      //       Dr=distance;
      //     }
      //     if(confirm){
      //       float a=distanceRef-distance;
      //       a=a/distanceRef;
      //       a=a*100; 
      //       niveauR=int(a);
      //       if(niveauR<0){
      //         niveauR=0;
      //       }
      //       if(niveauR>=97){
      //         niveauR=100;
      //       }

      //       // Serial.print("Niveau de remplissage=");
      //       // Serial.print(niveauR);
      //       // Serial.println("%");
      //       vDist=true;
      //       confirm=false;
      //     }



      //   }
      // }
      niveauR=100;
      esp_task_wdt_reset();
      Serial.println("Scanning Done");
      if(true){
        //Send LoRa packet to receiver
        Serial.print("Let send Here the data : Niveau de remplissage = ");
        Serial.println(niveauR);
        // for(int i=0;i<3;i++){
          // Serial.print(i);Serial.print("...");
          LoRa.beginPacket();
          String loradata="N"+device+",";
          loradata=loradata+niveauR;
          LoRa.print(loradata);
          LoRa.endPacket();
          blinkLed(500,25);
          delay(9000);
          esp_task_wdt_reset();
        // }
        vDist=false;
      } 

  // }

  esp_task_wdt_delete(NULL);
  esp_task_wdt_deinit();
  digitalWrite(pin3V,LOW);
  digitalWrite(pin5V,LOW);
  digitalWrite(Led_esp,LOW);      
  delay(500);
  Serial.print("Going to sleep");
  delay(50);
  esp_deep_sleep_start();
  // esp_task_wdt_reset();
  // delay(10000);
  // delay(250);
  // esp_task_wdt_reset();
  // esp_light_sleep_start();
}

#endif

#ifdef slaveTest

void setup() {

  esp_task_wdt_init(8, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  //initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial2.begin(9600);
  while (!Serial2);
  esp_sleep_enable_timer_wakeup(5000000); // sleep timer 5 seconds
  Serial.println("LoRa Sender");
  pinMode(Led_esp,OUTPUT);
  digitalWrite(Led_esp,LOW);

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
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  Serial.println("LoRa Initializing OK!");
}

void loop() {

  unsigned prevmillis=millis();
  while((millis()-prevmillis)<2000){
    if (Serial2.available())
    {
      // Put the character in the buffer
      Buffer[BytesRead++] = Serial2.read();
      // Validate the input so far:
      switch (BytesRead)
      {
        case 1:  // Start character
          if (Buffer[0] != 0xFF) // Invalid start character
          {
            BytesRead = 0; // Start over
          }
          break;

        case 4:  // Checksum
          byte sum;  // DO NOT INITIALIZE LOCAL VARIABLES IN A 'case' CLAUSE.
          sum = Buffer[0] + Buffer[1] + Buffer[2];
          if (sum != Buffer[3])
          {
            // Serial.println("Invalid checksum: 0xFF + 0x");
            BytesRead = 0; // Start over
          }
          break;
        case 5: // End character
          if (Buffer[4] != 0xFF)
          {
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

        distance= DistanceInMM / 10;
        Serial.print("distance=");
        Serial.print(distance);
        Serial.println("cm");

      }
      else
      {
        Serial.println("distance=3cm");
        distance=3;
      }
      
      // if(abs(Dr-distance)>6){
      //   confirm=true;
      //   Dr=distance;
      // }
      // if(confirm){
        float a=distanceRef-distance;
        a=a/distanceRef;
        a=a*100; 
        niveauR=int(a);
        if(niveauR<0){
          niveauR=0;
        }
        if(niveauR>=97){
          niveauR=100;
        }

        // Serial.print("Niveau de remplissage=");
        // Serial.print(niveauR);
        // Serial.println("%");
        // vDist=true;
        // confirm=false;
      // }



    }
  }
    // if(vDist){
      //Send LoRa packet to receiver
      Serial.print("Let send Here the data : Niveau de remplissage = ");
      Serial.println(niveauR);
      LoRa.beginPacket();
      String loradata="N2,";
      loradata=loradata+niveauR;
      LoRa.print(loradata);
      digitalWrite(Led_esp,HIGH);      
      delay(1000);
      LoRa.endPacket();
      digitalWrite(pin3V,LOW);
      digitalWrite(pin5V,LOW);
      digitalWrite(Led_esp,LOW);      
      delay(1000);
      Serial.print("Going to sleep");
      delay(50);
      esp_deep_sleep_start();

      // vDist=false;
    // }
    

  }

#endif

#ifdef Master

void setup() {
  //initialize Serial Monitor
  esp_task_wdt_init(25, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  Serial.begin(115200);
  while (!Serial);
  delay(500);
  // if (!SPIFFS.begin(true)){
  //   #ifdef debug
  //   Serial.println("An Error has occurred while mounting SPIFFS");
  //   #endif
  //   blinkLed(2000,250);
  //   esp_restart();
  // }
  // else{
  //   #ifdef debug
  //   Serial.println("SPIFFS Begin");
  //   #endif
  //   #ifdef FormatMemory
  //   Serial.println("Formating ESP.....");
  //   SPIFFS.format();
  //   Serial.println("Formating ESP finish");
  //   #endif
  // }

  Serial.println("LoRa Receiver");

  pinMode(HardReset,INPUT_PULLUP);

  pinMode(Led_esp,OUTPUT);
  digitalWrite(Led_esp,LOW);

  pinMode(pin3V,OUTPUT);
  digitalWrite(pin3V,HIGH);
  

  pinMode(pin5V,OUTPUT);
  digitalWrite(pin5V,HIGH);

  WiFi.disconnect(true);
  // delay(100);
  // WiFi.begin(ssid, password);
  // Serial.println("Connecting");
  // unsigned long pmillis=millis();
  // while(WiFi.status() != WL_CONNECTED && (millis()-pmillis)<4500) {
  //   delay(500);
  //   Serial.print(".");
  // }
  // if((millis()-pmillis)>=4500){
  //   WiFi.disconnect();
  //   WiFi.reconnect();
  //   Serial.print("Can't Connect to WiFi: ");
  //   blinkLed(3000,500);
  //   pinMode(HardReset,OUTPUT);
  //   delay(500);
  //   pinMode(HardReset,LOW);
  //   esp_restart();
  // }
  digitalWrite(Led_esp,HIGH);
  esp_task_wdt_reset();
  delay(2000);
  digitalWrite(Led_esp,LOW);

  // WiFi.onEvent(WiFiStationDisconnected, SYSTEM_EVENT_STA_DISCONNECTED);
  
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
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  Serial.println("LoRa Initializing OK!");
  delay(1500);
  esp_task_wdt_reset();

}

void loop() {
  // try to parse packet
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      Serial.println("Received packet");
      // read packet
      while (LoRa.available()){
        String LoRaData = LoRa.readString();
        Serial.println(LoRa.packetRssi());
        Serial.println(LoRaData);
        
        String datatopost= "[{\"X\":\"";
        datatopost=datatopost+LoRaData+"\"}]";
        Serial.println(datatopost);
        // if(WiFi.status()== WL_CONNECTED){

        //   // Your Domain name with URL path or IP address with path
        //   if(a){
        //     http.begin(client, serverName);
        //     a=false;
        //   }

        //   http.addHeader("Content-Type", "application/json");
        //   int i=0;
        //   for(i=0;i<5;i++){
        //     int httpResponseCode = http.POST(datatopost);
        //     Serial.print("HTTP Response code: ");
        //     if(httpResponseCode==200){
        //       Serial.println("Good Server");
        //       Serial.println(httpResponseCode);
        //       blinkLed(500,25);
        //       digitalWrite(Led_esp,LOW);
        //       break;
        //     }else{
        //       Serial.println(httpResponseCode);
        //       Serial.println("Bad Server");
        //     }
        //     esp_task_wdt_reset();
        //   }
        //   if(i>=5){
        //     Serial.println("Can't Send data to server !!!!!!!!");
        //     delay(500);
        //     pinMode(HardReset,OUTPUT);
        //     delay(500);
        //     pinMode(HardReset,LOW);
        //     esp_restart();
        //   }
        // }
        // else {
        //   Serial.println("WiFi Disconnected");
        //     pinMode(HardReset,OUTPUT);
        //     delay(500);
        //     pinMode(HardReset,LOW);
        //   esp_restart();
        // }
      }
    }
  esp_task_wdt_reset();

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

#ifdef HTTPsend


void setup() {
  esp_task_wdt_init(25, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  Serial.begin(115200);
  while (!Serial);

  pinMode(Led_esp,OUTPUT);
  digitalWrite(Led_esp,LOW);

  WiFi.disconnect(true);
  delay(100);
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  unsigned long pmillis=millis();
  while(WiFi.status() != WL_CONNECTED && (millis()-pmillis)<4500) {
    delay(500);
    Serial.print(".");
  }
  if((millis()-pmillis)>=4500){
    WiFi.disconnect();
    WiFi.reconnect();
    Serial.print("Can't Connect to WiFi: ");
    esp_restart();
  }
  digitalWrite(Led_esp,HIGH);
  esp_task_wdt_reset();
  delay(2000);
  digitalWrite(Led_esp,LOW);


  esp_task_wdt_reset();


  WiFi.onEvent(WiFiStationDisconnected, SYSTEM_EVENT_STA_DISCONNECTED);

  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
    Serial.println("Received packet");
      // Your Domain name with URL path or IP address with path
      String datatopost= "[{\"X\":\"";
      datatopost=datatopost+"N600,50";
      datatopost=datatopost+"\"}]";

      Serial.println(datatopost);
      if(a){
        http.begin(client, serverName);
        a=false;
      }
      http.addHeader("Content-Type", "application/json");
      // int i=0;
      // for(i=0;i<5;i++){
      //   int httpResponseCode = http.POST(datatopost);
      //   Serial.print("HTTP Response code: ");
      //   if(httpResponseCode==200){
      //     Serial.println("Good Server");
      //     Serial.println(httpResponseCode);
      //     blinkLed(500,25);
      //     digitalWrite(Led_esp,LOW);
      //     break;
      //   }else{
      //     Serial.println(httpResponseCode);
      //     Serial.println("Bad Server");
      //   }
      //   esp_task_wdt_reset();
      // }
      // if(i>=5){
      //   Serial.println("Can't Send data to server !!!!!!!!");
      //   delay(500);
      //   esp_restart();
      // }

    delay(5000);
  esp_task_wdt_reset();
}

#endif






void blinkLed(uint16_t time_Out,uint16_t ms){
  unsigned long preMillis=millis();
  while((millis()-preMillis)<time_Out){
    ledState = !ledState;
    digitalWrite(Led_esp,ledState);
    delay(ms);
  }
}