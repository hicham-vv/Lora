
#include<Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <math.h>
#include <SPIFFS.h>
#include "SparkFunSX1509.h"
#include "esp_task_wdt.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <HardwareSerial.h>
#include "driver/uart.h"


// #define FormatMemory



#define debug

// #define MasterReceiver
// #define MasterTracBal


// #define Master
#define Slave
// #define slaveTest
// #define HTTPsend
// #define Sensor


String device = "0"; // write the device Number Here !


const char * ssid = "Orange-80C3";
// const char * ssid = "HUAWEI-8e4e";
// const char * ssid = "Redmi9TH";


// const char * password = "0GEYH04G5A2";
// const char * password = "ifran123";
// const char * password = "luffy123";
const char * password = "tantan-1";





#define NUMERO_PORTA_SERIALE UART_NUM_1
#define BUF_SIZE (1024 * 2)
#define RD_BUF_SIZE (1024)



#define U2RXD 14
#define U2TXD 12


#define Led_esp 2 // GPIO 2 ESP32
#define Reg_Enable 15 // GPIO 15 ESP32
#define QwiicEnable 12
#define HardReset 14 // Hardware Reset MTMS

//define the pins used by the transceiver module
#define ss 4
#define rst 22
#define dio0 21

#define pin3V 26
#define pin5V 25

void TaskPopHTTPPOST( void *pvParameters );
TaskHandle_t handle_taskPopHTTPPOST;

void GPSPopHTTPPOST(File file);


/******************-------SIM800--------********************/
bool powerUp();
bool powerDown();
void powerCycle();
bool gsmSetup();
bool gsmCheck(uint16_t waitInterval);
uint8_t getGsmStat(uint16_t timeOut);
void flushSim();
bool sendAtCom(long timeout, char const *atCom, char const *Rep, char const *Error, int nbRep);
bool gprsOn();
bool getImei();
void getICCID();
void httpGet();
bool fireHttpAction(long timeout, char const *Commande, char const *Rep, char const *Error);
bool httpPost(char *payload);



static void UART_ISR_ROUTINE(void *pvParameters);
static QueueHandle_t uart1_queue;
String fileName = "";

char LoraToPost[50] = ""; // last GPS data to post


static const char * TAG = "";      


SX1509 io; // Create an SX1509 object to be used throughout
const byte SX1509_ADDRESS = 0x3E;  // SX1509 I2C address
#define PWS 8 // SX Pin power stat
#define KEY 15 // SX pin Key 
#define NS 10 // SX pin Network stat
#define VIO 9 // SX pin VIO




/***S800***/
String imei = "";
String ICCID ="";
String ipAddress = "";
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
uint8_t noGsmCounter = 0;
bool gsmstatus =false;
bool OkToSend = false;
uint16_t httpTimeout = 30000;
bool httppostgood = false;
uint8_t i_http=0;
bool Activity = false; // Track button press in ISR
bool readytosleep = false;
bool ImeiGood=false;


byte Buffer[5];
unsigned BytesRead = 0;
unsigned DistanceInMM = 0;



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

#ifdef MasterReceiver

void setup() {
  //initialize Serial Monitor
  esp_task_wdt_init(25, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  Serial.begin(115200);
  while (!Serial);
  delay(500);
  Serial2.begin(9600);
  while (!Serial2);
  delay(500);

  Serial.println("LoRa Receiver");

  // pinMode(HardReset,INPUT_PULLUP);

  pinMode(Led_esp,OUTPUT);
  digitalWrite(Led_esp,LOW);

  pinMode(pin3V,OUTPUT);
  digitalWrite(pin3V,HIGH);
  

  pinMode(pin5V,OUTPUT);
  digitalWrite(pin5V,HIGH);


  digitalWrite(Led_esp,HIGH);
  esp_task_wdt_reset();
  delay(2000);
  digitalWrite(Led_esp,LOW);

  
  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  
  //replace the LoRa.begin(---E-) argument with your locat²n's frequency 
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(868000000)) {
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
  delay(500);
  esp_task_wdt_reset();
  previousMillis=millis();

}

void loop() {
  // try to parse packet
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      Serial.println("Received packet");
      // read packet
      while (LoRa.available()){
        String LoRaData = LoRa.readString();
        String RSSI=String(LoRa.packetRssi());
        Serial.print(LoRaData);Serial.print("\t");Serial.println(RSSI);
        LoRaData=LoRaData+","+RSSI;
        Serial.println(LoRaData);
        Serial2.print(LoRaData);
      }
      blinkLed(500,25);
      digitalWrite(Led_esp,LOW);
      delay(500);
      LoRa.beginPacket();
      LoRa.print("OK");
      delay(1000);
      LoRa.endPacket();
    }

    if((millis()-previousMillis)>1800000){
      esp_restart();
    }
  
  esp_task_wdt_reset();
}
#endif

#ifdef MasterTracBal

void setup() {
  //initialize Serial Monitor

  pinMode(Led_esp,OUTPUT);

  esp_task_wdt_init(120, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  Serial.begin(115200);
  while (!Serial);
  delay(500);


  Serial2.begin(9600);
  while (!Serial2);
  pinMode(Led_esp,OUTPUT);
  delay(500);
  if (!SPIFFS.begin(true)){
    #ifdef debug
    Serial.println("An Error has occurred while mounting SPIFFS");
    #endif
    blinkLed(1000,100);
    delay(10);
    // pinMode(HardReset,OUTPUT);
    // delay(500);
    // pinMode(HardReset,LOW);
    // delay(500);
    esp_restart();

  }
  else{
    #ifdef debug
    Serial.println("SPIFFS Begin");
    #endif
    #ifdef FormatMemory
    Serial.println("Formating ESP.....");
    SPIFFS.format();
    Serial.println("Formating ESP finish");
    #endif
  }

  esp_task_wdt_reset();
  pinMode(Reg_Enable,OUTPUT);
  digitalWrite(Reg_Enable,LOW);
  delay(2500);
  pinMode(Reg_Enable,INPUT);
  pinMode(QwiicEnable,INPUT);
  esp_task_wdt_reset();

  uint32_t frequency=100000;
  if(!Wire.begin(21,22,frequency)){
    #ifdef debug
    Serial.println("I2C KO !!!!, Please check the Qwiic Power or the 3.3V LP voltage");
    #endif
    blinkLed(5000,100); // Blink if I2C not begin
    delay(10);
    // pinMode(HardReset,OUTPUT);
    // delay(500);
    // pinMode(HardReset,LOW);
    // delay(500);
    esp_restart();
  }else{
    #ifdef debug
    Serial.println("I2C begin");
    #endif
  }
  esp_task_wdt_reset();

  /*********** SX1509 ***********/
  if (!io.begin(SX1509_ADDRESS)){
    #ifdef debug
    Serial.println("SX1509 Failed to communicate.");
    #endif
    blinkLed(5000,500);
    delay(10);
    // pinMode(HardReset,OUTPUT);
    // delay(500);
    // pinMode(HardReset,LOW);
    // delay(500);
    esp_restart();
  }
  else{
    #ifdef debug
    Serial.println("SX1509 Begin communication");
    #endif
  }
  esp_task_wdt_reset();
  delay(500);
  /*********** END SX1509 ***********/

  /***********  SIM800   ********/
  io.pinMode(VIO,OUTPUT);
  io.digitalWrite(VIO,HIGH);
  io.pinMode(PWS,INPUT);
  io.pinMode(NS,INPUT);

      //Configuro la porta Serial1 (tutti i parametri hanno anche un get per effettuare controll)
    uart_config_t Configurazione_UART1 = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(NUMERO_PORTA_SERIALE, &Configurazione_UART1);
 
 
 
    // void esp_log_level_set(const char *tag, esp_log_level_tlevel)
    esp_log_level_set(TAG, ESP_LOG_INFO);
 
 
   
    // esp_err_tuart_set_pin(uart_port_tuart_num, int tx_io_num, int rx_io_num, int rts_io_num, int cts_io_num)
    uart_set_pin(NUMERO_PORTA_SERIALE, U2TXD, U2RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
 
 
    // uart_driver_install(UART_NUM_1, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
    // uart_driver_install(Numero_porta, RXD_BUFFER, TXD_Buffer, event queue handle and size, flags to allocate an interrupt)
    uart_driver_install(NUMERO_PORTA_SERIALE, BUF_SIZE, BUF_SIZE, 20, &uart1_queue, 0);
 
 
    //Create a task to handler UART event from ISR
    xTaskCreate(UART_ISR_ROUTINE, "UART_ISR_ROUTINE", 2048, NULL, 12, NULL);

    // delay(1000);

    // xTaskCreatePinnedToCore(TaskPopHTTPPOST,"Http",10000,NULL,2,&handle_taskPopHTTPPOST,1);
    esp_task_wdt_reset();

}

void loop() {
  #ifdef debug
  Serial.print("Looping.....");
  #endif
  bool simON=true;
  gsmstatus=false;
  previousMillis=millis();
  uint8_t CompteurGSM=0;
  while (true){
      esp_task_wdt_reset();
      File root = SPIFFS.open("/");
      File file = root.openNextFile();
      if(file){
        if(gsmstatus){
          CompteurGSM=0;
          simON=true;
          if(file){
            GPSPopHTTPPOST(file);
          }
        }else{
          CompteurGSM++;
          powerDown();
          esp_task_wdt_reset();
          delay(500);
          bool pu=powerUp();
          if(pu){
            gsmstatus=gsmSetup();
            esp_task_wdt_reset();
          }else{
            blinkLed(4000,50);
            digitalWrite(Led_esp,LOW);
          }
        }
      }
      else{
        if(simON){
          bool pd=powerDown();
          if(!pd){
            powerDown();
          }
          simON=false;
        }
        #ifdef debug
        Serial.println("No more Data to send Ready to sleep is true");
        #endif
        gsmstatus=false;
        delay(30000);

      }
    esp_task_wdt_reset();
    delay(5000);
    unsigned long ms=millis()-previousMillis;
    if(ms>3600000 || CompteurGSM>5){
    #ifdef debug
    Serial.println("Let restart the Systeme");
    #endif
    delay(50);
    // pinMode(HardReset,OUTPUT);
    // delay(500);
    // pinMode(HardReset,LOW);
    delay(500);
    esp_restart();
    }
  }
}
#endif


#ifdef Slave





void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial2.begin(9600);
  while (!Serial2);
  delay(1000);

  esp_task_wdt_init(25, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  Serial.println("LoRa Sender");

  // esp_sleep_enable_timer_wakeup(10000000); // 5s
  // esp_sleep_enable_timer_wakeup(3600000000); // 1hrs
  esp_sleep_enable_timer_wakeup(14400000000); // 4hrs


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
  while (!LoRa.begin(868000000)) {
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
}

void loop() {
  unsigned prevmillis=millis();
  uint8_t CompteurMoyen=1;
  uint16_t Mdistance=0;
  while((millis()-prevmillis)<3000){
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
        Serial.println("distanceeee=3cm");
        distance=3;
      }
      Mdistance=Mdistance+distance;
      CompteurMoyen++;
    }
  }
  Mdistance=Mdistance/CompteurMoyen;
  Serial.print("Distance Moyenne=");Serial.println(Mdistance);
  float a=distanceRef-Mdistance;
  a=a/distanceRef;
  a=a*100;
  niveauR=int(a);
  if(niveauR<0){
    niveauR=0;
  }
  if(niveauR>=97){
    niveauR=100;
  }
  esp_task_wdt_reset();
  Serial.println("Scanning Done");
  Serial.print("Let send Here the data : Niveau de remplissage = ");
  Serial.println(niveauR);
  LoRa.beginPacket();
  String loradata="N"+device+",";
  loradata=loradata+niveauR;
  LoRa.print(loradata);
  delay(1000);
  LoRa.endPacket();
  blinkLed(500,25);
  bool confi=false;
  prevmillis=millis();
  while((millis()-prevmillis)<5000 &&  confi==false){
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      Serial.println("Received Confirmation");
      while (LoRa.available()){
        String confirmation = LoRa.readString();
        if (strstr(confirmation.c_str(), "OK")){
          Serial.println("Received Confirmation ** OK ** ");
          confi=true;
          break;
        }
      }
    }
  }
  if(confi==false){
    Serial.println("didn't receive any Confirmation Let resend Data");
    LoRa.beginPacket();
    LoRa.print(loradata);
    delay(1000);
    LoRa.endPacket();
  }
  esp_task_wdt_reset();
  esp_task_wdt_delete(NULL);
  esp_task_wdt_deinit();
  digitalWrite(pin3V,LOW);
  digitalWrite(pin5V,LOW);
  digitalWrite(Led_esp,LOW);      
  Serial.print("Going to sleep");
  delay(250);
  esp_deep_sleep_start();
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
  Serial.println("LoRa Receiver");

  pinMode(HardReset,INPUT_PULLUP);

  pinMode(Led_esp,OUTPUT);
  digitalWrite(Led_esp,LOW);

  pinMode(pin3V,OUTPUT);
  digitalWrite(pin3V,HIGH);
  

  pinMode(pin5V,OUTPUT);
  digitalWrite(pin5V,HIGH);

  // WiFi.disconnect(true);
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
  while (!LoRa.begin(867E6)) {
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
  delay(500);
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
        Serial.println(LoRa.rssi());
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



 static void UART_ISR_ROUTINE(void *pvParameters)
{
  int i=0;
    uart_event_t event;
    // size_t buffered_size;
    bool exit_condition = false;
    //Infinite loop to run main bulk of task
    while (1) {
        //Loop will continually block (i.e. wait) on event messages from the event queue
        if(xQueueReceive(uart1_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
         
            //Handle received event
            if (event.type == UART_DATA) {
 
                uint8_t UART1_data[100]={0};
                int UART1_data_length = 0;
                ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_1, (size_t*)&UART1_data_length));
                UART1_data_length = uart_read_bytes(UART_NUM_1, UART1_data, UART1_data_length, 100);
             
                // Serial.println("LEN= ");Serial.println(UART1_data_length);
 
                Serial.print("DATA=");
                String data= (char*)UART1_data;
                // for(byte i=0; i<UART1_data_length;i++){
                //   Serial.print((char)UART1_data[i]);
                // }
                Serial.println(data);
                String Si=String(i);
                fileName="/";
                fileName=fileName+data;
                fileName=fileName+Si+".txt";
                i++;
                File filewrite = SPIFFS.open(fileName,"w");
                String datatopost= "[{\"X\":\"";
                datatopost=datatopost+data+"\"}]";
                filewrite.println(datatopost);
                // Serial.println(datatopost);
                filewrite.close();
                // #ifdef debug
                // Serial.print("File wrote UART Event--> ");
                // Serial.println(fileName);
                // #endif 
                Serial.println("");
             
            }
           
            //Handle frame error event
            else if (event.type == UART_FRAME_ERR) {
                Serial.println("UART_FRAME_ERR");
            }
           
            //Keep adding else if statements for each UART event you want to support
            //else if (event.type == OTHER EVENT) {
                //TODO...
            //}
           
           
            //Final else statement to act as a default case
            else {
                //TODO...
            }      
        }
       
        //If you want to break out of the loop due to certain conditions, set exit condition to true
        if (exit_condition) {
            break;
        }
    }
   
    //Out side of loop now. Task needs to clean up and self terminate before returning
    vTaskDelete(NULL);
}



void GPSPopHTTPPOST(File file){
  httppostgood=false;
  i_http=0;
  File fileread =SPIFFS.open(file.name(),"r");
  String data= fileread.readStringUntil('\n');
  fileread.close();
  #ifdef debug
  Serial.print("File found:");Serial.println(file.name());
  Serial.println(data);
  Serial.println("if there is * let remplace it by imei ");
  #endif
  data.replace("*",imei);
  int dataLen=data.length();
  if(dataLen>7){
    // Confirm Format JSON
    if(data[0]!='['){
      data[0]='[';
    }
    if(data[1]!='{'){
      data[1]='{';
    }
    if(data[2]!='"'){
      data[2]='"';
    }
    if(data[dataLen-4]!='"'){
      data[dataLen-4]='"';
    }
    if(data[dataLen-3]!='}'){
      data[dataLen-3]='}';
    }
    if(data[dataLen-2]!=']'){
      data[dataLen-2]=']';
    }
    // End Confirm Format JSON

    for (int i=3;i<dataLen-4;i++){
      if(!isAlphaNumeric(data[i]) && data[i]!='.' && data[i]!='-'  && data[i]!='{' && data[i]!='"' && data[i]!=':' && data[i]!=',' && data[i]!='}'){
        data[i]='X';
      }
    }

    String Newdata="";
    Newdata =  data.substring(0,data.length()-1);
    #ifdef debug
    Serial.println("Checked New data");
    Serial.println(Newdata);
    #endif
    sprintf(LoraToPost,"%s",Newdata.c_str());
  }
  else{
    httppostgood=true;
    for(int i=0;i<2;i++){
      if(SPIFFS.remove(file.name())){
        #ifdef debug
        Serial.print("Remouved Bad file:");Serial.println(file.name());
        #endif
        break;
      }
    }
  }
  while(httppostgood == false && (gsmstatus==true) && i_http <2){
    if(httpPost(LoraToPost)){
      httppostgood=true;
      i_http=0;
      for(int i=0;i<2;i++){
        if(SPIFFS.remove(file.name())){
          #ifdef debug
          Serial.print("remouved file:");Serial.println(file.name());
          #endif
          blinkLed(500,25);
          digitalWrite(Led_esp,LOW);
          break;
        }
      }
    }
    else{
      i_http++;
      #ifdef debug
      Serial.println("Error Server");
      #endif
      delay(500);
    }
    if(i_http>=2){
      gsmstatus=false;
    }
  }
  delay(250);
}







bool powerUp(){
  io.pinMode(PWS,INPUT);
  byte r=io.digitalRead(PWS);
  if(r==HIGH){
    #ifdef debug
    Serial.println("Sim800 Already Powered Up");
    #endif
    return true;
  }else{
    delay(500);
    #ifdef debug
    Serial.println("Powering Up SIM800");
    #endif
    io.pinMode(KEY, OUTPUT); //PWR KEY
    delay(10);
    io.digitalWrite(KEY, LOW);
    delay(1200);
    io.pinMode(KEY,INPUT);
    delay(3500);
    unsigned long premillis=millis();
    while(r==LOW && (millis()-premillis)<5000){
      r=io.digitalRead(PWS);
      delay(500);
    }
    if(r==HIGH){
      #ifdef debug
      Serial.println("SIM800 Powered successfully ");
      #endif
      return true;
    }
    else{
      #ifdef debug
      Serial.println("Can't Power Up SIM800 ");
      #endif
      return false;
    }
  }
}
bool powerDown(){
  // Hardware Power Down
  io.pinMode(PWS,INPUT);
  byte r=io.digitalRead(PWS);
  if(r){
    #ifdef debug
    Serial.println("SIM800 ON let turn it OFF");
    #endif
    io.pinMode(KEY,OUTPUT);
    delay(10);
    io.digitalWrite(KEY,LOW);
    delay(2500);
    io.pinMode(KEY,INPUT);
    delay(500);
    r=io.digitalRead(PWS);
    unsigned premillis=millis();
    while(r && (millis()-premillis)<5000){
      r=io.digitalRead(PWS);
      delay(500);
    }
    if(!r){
      #ifdef debug
      Serial.println("SIM800 Turned OFF");
      #endif
      return true;
    }else{
      #ifdef debug
      Serial.println("Can't turn SIM800 OFF");
      #endif
      return false;
    }

  }else{
    #ifdef debug
    Serial.println("SIM800 Already OFF");
    #endif
    return true;
  }
}
void powerCycle(){
  if(io.digitalRead(PWS)==HIGH){
    #ifdef debug
    Serial.println("Power cycle SIM800");
    #endif
      powerDown();
      delay(3000);
      unsigned long premillis=millis();
      byte r=io.digitalRead(PWS);
      while( r==HIGH && (millis()-premillis)<5000){
        r=io.digitalRead(PWS);
        delay(500);
      }
      if(io.digitalRead(PWS)==LOW){
        delay(1000);  // Wait for at least 800ms after STATUS pin changed to low level
        io.pinMode(KEY,OUTPUT);
        delay(10);
        io.digitalWrite(KEY,LOW);
        delay(1200);
        io.pinMode(KEY,INPUT);
        delay(1000);
        premillis=millis();
        while(r==LOW && (millis()-premillis)<5000){
          r=io.digitalRead(PWS);
          delay(500);
        }
        if(r==HIGH){
          #ifdef debug
          Serial.println("GOOD power cycle ");
          #endif
        }
        else {
          #ifdef debug
          Serial.println("Bad power cycle ");
          #endif
        }
      }
      else{
        #ifdef debug
        Serial.println("Power Cycle Something Wrong");
        #endif
      }
  }
  else {
    #ifdef debug
    Serial.println("POWER STATUS is LOW can't operate powercycle let power up the SIM800 ");
    #endif
    powerUp();
  }
}
bool gsmSetup(){
  if(gsmCheck(20000)){
    if (gprsOn()){
      #ifdef debug
      Serial.println("GPRS ON");
      #endif
      return true;
    }
    else {      
      #ifdef debug
      Serial.println("GPRS OFF");
      #endif
      powerCycle();
      if(gsmCheck(20000)){
        if (gprsOn()){
          #ifdef debug
          Serial.println("GPRS ON");
          #endif
          return true;
        } 
        else {
          #ifdef debug
          Serial.println("GPRS OFF");
          #endif
          return false;
        }
      } else return false;
    }
  }
  else{
    powerCycle();
      if(gsmCheck(20000)){
        if (gprsOn()){
          #ifdef debug
          Serial.println("GPRS ON");
          #endif
          return true;
        } 
        else {
          #ifdef debug
          Serial.println("GPRS OFF");
          #endif
          return false;
        }
      } else return false;
  }
}
bool getImei()
{
  Serial2.setTimeout(1000);
  Serial2.println("AT+GSN");
  String tempGSM3 = Serial2.readString();
  String tempIMEI = tempGSM3;
  if (strstr(tempGSM3.c_str(), "OK"))
  {
    imei = strstr(tempIMEI.c_str(), "8");
    imei = imei.substring(0, 15);
    if(imei.length()==15){
      for (int i=0;i<imei.length();i++){
        if(!isAlphaNumeric(imei[i])){
          #ifdef debug
          Serial.println("IMEI Bad Char");
          #endif
          return false;
        }
      }
      #ifdef debug
      Serial.print("imei=");
      Serial.println(imei);
      #endif
      return true;
    }
    else{
      #ifdef debug
      Serial.println("Lenght imei is not correct ");
      #endif
      return false;
    }
  }
  else{
    #ifdef debug
    Serial.print("IMEI Is Wrong");
    #endif
    return false;
  }
}
void getICCID(){
  Serial2.setTimeout(1000);
  Serial2.println("AT+CCID");
  String tempGSM4 = Serial2.readString();
  String tempICCID = tempGSM4;
  if (strstr(tempGSM4.c_str(), "OK"))
  {
    ICCID = tempICCID.substring(10, 30);
  }
  else
  {
    ICCID = "000000";
  }
  #ifdef debug
  Serial.print("ICCID=");
  Serial.println(ICCID);
  #endif
}
bool gsmCheck(uint16_t waitInterval){
  #ifdef debug
  Serial.println("GSM Check");
  #endif
  unsigned long currMillis = millis();
  unsigned long prevMillis = millis();
  uint8_t gsmStatInt = getGsmStat(3000);
  while ((gsmStatInt != 1) && (gsmStatInt != 5) && ((currMillis-prevMillis) <= waitInterval) && ((io.digitalRead(PWS)==HIGH)))
  {
    gsmStatInt = getGsmStat(2000);
    String localdebug = "gsm status is " + String(gsmStatInt);
    #ifdef debug
    Serial.println(localdebug);
    #endif
    currMillis=millis();
    Serial.println(currMillis-prevMillis);
  }

  if (gsmStatInt == 1 || gsmStatInt == 5)
  {
    #ifdef debug
    Serial.println("GSM Connected");
    #endif
    return true ;
  }
  else
  {
    #ifdef debug
    Serial.println("GSM NOT Connected");
    #endif
    return false;
  }

  if (((currMillis - prevMillis) <= waitInterval))
  {
    if (io.digitalRead(PWS)==HIGH)
    {
      noGsmCounter = 0;
      return true;
    }
  }
  else
  {
    noGsmCounter++;
    if (noGsmCounter == 5)
    {
      powerCycle();
      noGsmCounter = 0;
    }
    return false;
  }
}
uint8_t getGsmStat(uint16_t timeOut)
{
  flushSim();
  Serial2.setTimeout(timeOut);
  Serial2.println("AT+CREG?");
  String tempGSM = Serial2.readString();
  int ind1 = tempGSM.indexOf(',');
  String gsmStat = tempGSM.substring(ind1 + 1, ind1 + 2);
  return gsmStat.toInt();
}
void flushSim(){
  unsigned long prems=millis();
  while(Serial2.available() && ((millis()-prems)<10000)){
    Serial2.read();
    delay(10);
  }
}
bool gprsOn()
{
  #ifdef debug
  Serial.println("turning ON GPRS...");
  #endif
  if(sendAtCom(5000, "AT+CFUN=1", "OK", "ERROR", 2)){
    Serial.println("OK1");
    if (sendAtCom(5000, "AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", "OK", "OK", 2)){
      Serial.println("OK2");
      if (sendAtCom(5000, "AT+SAPBR=1,1", "OK", "OK", 2)){
        Serial.println("OK3");
        // if (sendAtCom(5000, "AT+CIICR", "OK", "ERROR", 5)){
          // if (sendAtCom(5000, "AT+CIFSR", ">", "ERROR", 5)){
            return true;
      }else return false;
    }else return false;
  }else return false;
}
bool fireHttpAction(long timeout, char const *Commande, char const *Rep, char const *Error)
{
  flushSim();
  Serial2.setTimeout(timeout);
  Serial2.print(Commande);
  Serial2.println(1, DEC);
  if (Serial2.findUntil(Rep, Error))
  {
    flushSim();
    Serial2.setTimeout(1000);
    return true;
  }
  else
  {
    flushSim();
    Serial2.setTimeout(1000);
    return false;
  }
}
bool httpPost(char *payload){
  OkToSend = true;
  if (sendAtCom(3000, "AT+HTTPINIT", "OK", "ERROR", 5))
  { //"AT+HTTPINIT"
    if (sendAtCom(3000, "AT+HTTPPARA=\"CID\",1", "OK", "ERROR", 5)){
      if (sendAtCom(5000,"AT+HTTPPARA=\"URL\",\"http://tanger.geodaki.com:8080/datasnap/rest/Tdata/rep\"", "OK", "ERROR", 5)){
        Serial2.setTimeout(15000);
        flushSim();
        Serial2.print("AT+HTTPDATA=");
        delay(100);
        uint16_t Size = strlen(payload);
        Serial2.print(Size);
        Serial2.print(",");
        uint32_t maxTime = 30000;
        Serial2.println(maxTime);
        if(!Serial2.findUntil("DOWNLOAD", "ERROR"))
        {
        OkToSend=false;
        }
        Serial2.println(payload); // j'ai changé print à println //
        if(!Serial2.findUntil("OK", "OK"))
         {
          OkToSend=false;
         }
      }
      else  OkToSend = false;
    }
    else OkToSend = false;
  }
  else OkToSend = false;

  if (OkToSend)
  {
    if (fireHttpAction(httpTimeout, "AT+HTTPACTION=", ",200,", "ERROR"))
    {
      sendAtCom(10000, "AT+HTTPTERM", "OK", "ERROR", 5);
      #ifdef debug
      Serial.println("FireHttp is True");
      #endif
      return true;
    }
    else
    {
      sendAtCom(10000, "AT+HTTPTERM", "OK", "ERROR", 5);
      #ifdef debug
      Serial.println("FireHttp is Bad");
      #endif
      return false;
    }
  }
  else  return false;
}
bool sendAtCom(long timeout, char const *atCom, char const *Rep, char const *Error, int nbRep){
  flushSim();
  Serial2.setTimeout(timeout);
  for (uint16_t a = 0; a < strlen(atCom); a++)
  {
    Serial2.print(atCom[a]);
  }
  Serial2.println("");
  int compteur = 0;
  while ((!Serial2.findUntil(Rep, Error)) && (compteur < nbRep))
  {
    flushSim();
    for (uint16_t a = 0; a < strlen(atCom); a++)
    {
      Serial2.print(atCom[a]);
    }
    Serial2.println("");
    compteur++;
    delay(50);
  }
  if (compteur < nbRep){
    return true;}
  else{
    return false;
  }
  Serial2.setTimeout(1000);
}
