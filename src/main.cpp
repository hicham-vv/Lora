/*********
  Modified from the examples of the Arduino LoRa library
  More resources: https://randomnerdtutorials.com
*********/

#include <SPI.h>
#include <LoRa.h>

#include <WiFi.h>
#include <HTTPClient.h>


const char * ssid = "Orange-80C3";
// const char * ssid = "Redmi 9T";

const char * password = "0GEYH04G5A2";
// const char * password = "luffy123";



byte Buffer[5];
unsigned BytesRead = 0;
unsigned DistanceInMM = 0;



unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
int d=0;
int sum=1;
String Index ="T";

bool vDist = false;

uint8_t distance=0;




const char* serverName = "http://141.94.71.45:8081/datasnap/rest/Tdata/rep";


//define the pins used by the transceiver module
#define ss 4
#define rst 22
#define dio0 21

#define pin3V 26

#define pin5V 25






// #define Sensor
#define Master
// #define Slave
// #define HTTPsend









#ifdef Slave





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
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  Serial.println("LoRa Initializing OK!");
}

void loop() {

  unsigned prevmillis=millis();
  while((millis()-prevmillis)<1000){
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
            // Serial.print("Invalid checksum: 0xFF + 0x");
            Serial.println("distance= 0");

            distance=0;
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
    }
  }
    //Send LoRa packet to receiver
    Serial.print("Let send Here the data : Distance = ");
    Serial.println(distance);    
    LoRa.beginPacket();
    String loradata="N5,";
    loradata=loradata+distance;
    LoRa.print(loradata);
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


  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  unsigned long pmillis=millis();
  while(WiFi.status() != WL_CONNECTED && (millis()-pmillis)<5000) {
    delay(500);
    Serial.print(".");
  }
  if((millis()-pmillis)>=5000){
    Serial.print("Can't Connect to WiFi: ");
    ESP.restart();
  }

  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  
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
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  Serial.println("LoRa Initializing OK!");
  delay(500);
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("Received packet");
    // read packet
    while (LoRa.available()){
      String LoRaData = LoRa.readString();
      Serial.println(LoRaData);
      String datatopost= "[{\"X\":\"";
      datatopost=datatopost+LoRaData+"\"}]";
      Serial.println(datatopost);
      if(WiFi.status()== WL_CONNECTED){
        WiFiClient client;
        HTTPClient http;
        // Your Domain name with URL path or IP address with path
        http.begin(client, serverName);

        http.addHeader("Content-Type", "application/json");
        int httpResponseCode = http.POST(datatopost);
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        http.end();
      }
      else {
        Serial.println("WiFi Disconnected");
        ESP.restart();
      }
    }
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

#ifdef HTTPsend


// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastTime = 0;
// Timer set to 10 minutes (600000)
//unsigned long timerDelay = 600000;
// Set timer to 5 seconds (5000)
unsigned long timerDelay = 5000;

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
 
  Serial.println("Timer set to 5 seconds (timerDelay variable), it will take 5 seconds before publishing the first reading.");
}

void loop() {
  //Send an HTTP POST request every 10 minutes
    //Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED){
      WiFiClient client;
      HTTPClient http;
    
      // Your Domain name with URL path or IP address with path
      http.begin(client, serverName);

      // Specify content-type header
      // http.addHeader("Content-Type", "application/x-www-form-urlencoded");
      // // Data to send with HTTP POST
      // String httpRequestData = "api_key=tPmAT5Ab3j7F9&sensor=BME280&value1=24.25&value2=49.54&value3=1005.14";           
      // // Send HTTP POST request
      // int httpResponseCode = http.POST(httpRequestData);
      
      // If you need an HTTP request with a content type: application/json, use the following:
      http.addHeader("Content-Type", "application/json");
      int httpResponseCode = http.POST("{\"Hello Rachid \"}");

      // If you need an HTTP request with a content type: text/plain
      //http.addHeader("Content-Type", "text/plain");
      //int httpResponseCode = http.POST("Hello, World!");
     
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
        
      // Free resources
      http.end();
      delay(2000);
    }
    else {
      Serial.println("WiFi Disconnected");
    }
}

#endif