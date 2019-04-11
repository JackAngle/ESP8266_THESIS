#include "ESP8266WiFi.h"
#include "DHTesp.h"
#include <WebSocketsServer.h>
#include <WebSocketsClient.h>
#include <WebSockets.h>
#include <Hash.h>
#include <SoftwareSerial.h>
#include <ESPAsyncTCP.h>



//#define AP_MODE_TIMEOUT_OUT 4000
#define AP_MODE 0
#define STA_MODE 1
#define SERVER_SSID "THESIS"
#define SERVER_PASSWORD "mustbedone"
#define SOCKET_PORT 8000
#define SOCKET_SERVER_IP "192.168.4.1"

//#define INTERVAL_BETWEEN_SENSOR_READ 3000
#define AP_MODE_TIMEOUT 10*1000 
//#define AP_FIRST_PHASE_TIMEOUT  6*1000
#define STA_MODE_TIMEOUT 10*1000
#define AP_DEEP_SLEEP_TIME 20*1000000
//#define AP_SLEEP_BROADCAST_TIME 45*1000
#define DATA_ARRAY_MAXIMUM_LENGTH 500



// Structure which will be stored in RTC memory.

struct {
  uint32_t crc32;
  byte counter;
  int dataLength;
  char data[DATA_ARRAY_MAXIMUM_LENGTH];
} rtcData;

bool isServerConnected = false;
bool isWiFiConnected = false;
 
DHTesp dht;
WebSocketsClient clientSocket;
WebSocketsServer webSocket = WebSocketsServer(SOCKET_PORT);

static int count = 6;
unsigned long currentTime = 0;
//unsigned long lastSensorReadTime = 0;
unsigned long apStartTimer = 0;
unsigned long staStartTimer = 0;
//unsigned long apSleepTimer = 0;
//bool apFirstPhase = false;
//bool isAP = false;
//bool isSTA = false;
//bool isPingBroadcasted = false;



bool currentMode = AP_MODE;


  /*CLIENT_WEBSOCKETEVENT handler*/
void clientSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
String mBuffer = (char*)payload;
  switch(type) {
    case WStype_DISCONNECTED:
        if (isServerConnected){
          isServerConnected = false;
        }
        Serial.printf("[Client] Disconnected!\n");
        if(isWiFiConnected){
          connectToIP(SOCKET_SERVER_IP);
        }
      break;
    case WStype_CONNECTED: {    
        Serial.print("[This Client] Connected to: ");   
        Serial.println(SOCKET_SERVER_IP);    
        /*send message to server when Connected*/
        clientSocket.sendTXT("Connected");
      
    }
      break;
    case WStype_TEXT:
      Serial.printf("[Client] get text: %s\n", payload);

      /*Check if this is server*/
      if (!isServerConnected){
        if (mBuffer.startsWith("1")){//Check OpCode
          Serial.println("Right Header");
          String message = mBuffer.substring(mBuffer.indexOf('|') + 1);
          if (message == "Welcome to server!"){
            Serial.println("Yup, server desu~");
            isServerConnected = true;

            /*Send data to server*/
            loadRtcDataAndSendToServer();
            readSensorAndSendToServer();
        
          }else{
            /*Wrong message -> goodbye*/
            Serial.println("Wrong message");
            Serial.println(mBuffer);
            goodBye();
          }
        }else{
          Serial.println("Wrong Header");
          Serial.println(mBuffer);
          goodBye();
        }
      }

      /*Received AP's schedule -> response -> sleep*/
      if (mBuffer.startsWith("AP_SLEEP")){
        clientSocket.sendTXT("SLP_RCV");//a.k.a SLEEP_RECEIVED
        String sleepTimeBuffer = mBuffer.substring(mBuffer.indexOf(':') + 1);
        int sleepTime = sleepTimeBuffer.toInt();
        rtcData.counter = 2;
        writeRTC();
        Serial.println("STA Deep sleep...");
        ESP.deepSleep(sleepTime*1000, WAKE_RF_DEFAULT);
      }

      // send message to server
      // 
      break;
    case WStype_BIN:
      Serial.printf("[Client] get binary length: %u\n", length);
      hexdump(payload, length);

      // send data to server
      // webSocket.sendBIN(payload, length);
      break;
  }
  delay(1000);
}


/*WEBSOCKETEVENT HANDLER (server side)*/
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
String mBuffer = (char*)payload;
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Client Disconnected!\n", num);
            break;
        case WStype_CONNECTED:
            {
                IPAddress ip = webSocket.remoteIP(num);
                Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
        
        /* send message to client*/
        webSocket.sendTXT(num, "1|Welcome to server!");
            }
            break;
        case WStype_TEXT:
            Serial.printf("[%u] get Text: %s\n", num, payload);

            /*Received Data Packet from Client*/
            if(mBuffer.startsWith("DATA|")){
              /*Read & save to RTC data array*/
              String data = mBuffer.substring(mBuffer.indexOf('|') + 1);
//              int bufferLength = data.length() + 1;          
//              byte byteBuffer[bufferLength];
//              data.getBytes(byteBuffer, );
              bool found2Dot = false;
              int readingLength = data.length();
              if(mBuffer.endsWith("|END")){
                readingLength = readingLength - 4;
              }
              for (int i = 0; i < readingLength; i++){
                if (data.charAt(i) != ':'){
                  if (found2Dot){
                    rtcData.data[rtcData.dataLength + i - 1] = data.charAt(i);
                  }else{
                    rtcData.data[rtcData.dataLength + i] = data.charAt(i);
                  }
                }else{
                  found2Dot = true;
                }
              }
              /*Update data length*/
              //rtcData.data[rtcData.dataLength + data.length()] ;
              rtcData.dataLength += data.length() - 1;
              if(mBuffer.endsWith("|END")){
                rtcData.dataLength = rtcData.dataLength - 4;
              }
              
              Serial.println(data);
              Serial1.println(data);

              if(mBuffer.endsWith("|END")){
                /*Send sleep_time to client*/
                unsigned long clientSleepTime = AP_MODE_TIMEOUT - currentTime + apStartTimer + AP_DEEP_SLEEP_TIME/1000 - 1000;
                Serial.println(clientSleepTime);
                String request = "AP_SLEEP:" + String(clientSleepTime);
                Serial.println(request);
                webSocket.sendTXT(num, request);
              }
            }

            if(mBuffer.startsWith("SLP_RCV")){//Client received Sleep time
              webSocket.disconnect(num);
            }
                 
            
            // send message to client
            // webSocket.sendTXT(num, "message here");

            // send data to all connected clients
            // webSocket.broadcastTXT("message here");
            break;
        case WStype_BIN:
            Serial.printf("[%u] get binary length: %u\n", num, length);
            hexdump(payload, length);

            // send message to client
            // webSocket.sendBIN(num, payload, length);
            break;
    }
}


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(50);

  count = readRtcAndCounter(); 
  /*calculateCRC32*/
  uint32_t crcOfData = calculateCRC32((uint8_t*) &rtcData.data[0], sizeof(rtcData.data));
    Serial.print("CRC32 of data: ");
    Serial.println(crcOfData, DEC);
    Serial.print("CRC32 read from RTC: ");
    Serial.println(rtcData.crc32, DEC);
    if (crcOfData != rtcData.crc32) {
      Serial.println("CRC32 in RTC memory doesn't match CRC32 of data. Setting up first running...");
      /*Set data array to NULL*/
      for (int i = 0; i < DATA_ARRAY_MAXIMUM_LENGTH; i++){
        rtcData.data[i] = NULL;
      }
      rtcData.dataLength = 0;
      rtcData.counter = 6;
    } else {
      Serial.println("CRC32 check ok, data is probably valid.");     
    }
    
  /*Read counter from RTC and choose working mode*/
  if ((count <= 6)&&(count > 3)){
    currentMode = AP_MODE;
    apStartTimer = millis();
;
  } else if ((count <= 3)&&(count > 0)){
    currentMode = STA_MODE;
    staStartTimer = millis();
  } 

  /*Set up DHT22*/
  dht.setup(D5, DHTesp::DHT22);  

  if (currentMode == AP_MODE){
  /*Set up Soft-AP*/
    WiFi.mode(WIFI_AP); 
    Serial.print("Setting soft-AP ... ");
    Serial.println(WiFi.softAP(SERVER_SSID, SERVER_PASSWORD, 1, false, 8)? "AP's Ready" : "Failed!");

    Serial.print("Soft-AP IP address = ");
    Serial.println(WiFi.softAPIP());
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
  }else{
    /*Set up STATION mode*/
    WiFi.mode(WIFI_STA);
    delay(50);
    WiFi.begin(SERVER_SSID, SERVER_PASSWORD);
    
    /*Assign event handler*/
    clientSocket.onEvent(clientSocketEvent);
    Serial.println("Attached clientSocket");

    /* try every 2000ms again if connection has failed*/
    clientSocket.setReconnectInterval(2000);
  }

  Serial.print("Count Read: ");
  Serial.println(count);
  
}
 
void loop() {
  currentTime = millis();

/*AP MODE*/
  if (currentMode == AP_MODE){
    /*Check AP timeout*/
    if ( (unsigned long) (currentTime - apStartTimer) > (AP_MODE_TIMEOUT)){
      rtcData.counter = count - 1;
      writeRTC();
      Serial.println(rtcData.counter);         
      Serial.println("AP Deep sleep ");
      ESP.deepSleep(AP_DEEP_SLEEP_TIME, WAKE_RF_DEFAULT);
  }
 
    /*SERVER WORKS*/
    /*Check the number of connected device(s)*/
  if(WiFi.softAPgetStationNum() == 0){
   //TO-DO
  }else{    
    webSocket.loop();    
  }    
 }
/*STATION MODE*/  
  else{    
     /*Check STA timeout*/
    if ( (unsigned long) (currentTime - staStartTimer) > (STA_MODE_TIMEOUT)){
      if(!isWiFiConnected){
        rtcData.counter = count - 1;
        writeRTC();
        Serial.println(rtcData.counter);
        Serial.println("STA Deep sleep");
        ESP.deepSleep(random(10, 30)*1000000, WAKE_RF_DEFAULT);
      }else{
        staStartTimer = millis();
      }
  }
    
     /*SHOW WIFI STATUS*/
    if(!isWiFiConnected){
       if(WiFi.status() == WL_CONNECTED){
        Serial.print("Connected to WiFi AP! IP:");
        Serial.println(WiFi.localIP());
        isWiFiConnected = true;
       }
    }else{
      if(WiFi.status() != WL_CONNECTED){
        Serial.println("Disconnected from WiFi AP!");
        isWiFiConnected = false;
       }
    }
    
    /*CLIENT WORKS*/
       
    clientSocket.loop();
      
  }
}



///////////////FUNCTIONS////////////////////
void readSensorAndSendToServer(){  
        /**/
        float humidity = dht.getHumidity();
        float temperature = dht.getTemperature();
        String request = String(F("DATA|")) + ESP.getChipId()+ String(F(":"));
        if (dht.getStatusString() == "OK"){
          Serial.println("Reading succeeded"); 
          request += String(temperature*10, 0);
          request += String(humidity*10, 0);
        } else{
           Serial.println("Reading failed");
          request += "000";
          request += "000";          
        }  
        int value = analogRead(A0); 
        int percent = map(value, 0, 1023, 0, 100);
        if (percent < 100){
          if(percent > 0){//100>percent>0
            request = request + "0" + String(percent);
          }else{//percent == 0
             request += "000";     
          }
        }else{//percent == 100
          request += String(percent);
        }
        request += String(F("|END"));
        clientSocket.sendTXT(request);
        Serial.print("Request: ");
        Serial.println(request);       
}

void loadRtcDataAndSendToServer(){
  char dataBuffer[18] = "";
  int localCounter = 0;

  /*Read data buffer every 16 character*/
  while (localCounter < rtcData.dataLength){
    for (int i = 0; i < 17; i++){
      if (i < 7){
        dataBuffer[i] = char(rtcData.data[localCounter + i]);
        rtcData.data[localCounter + i] = '\0';
      }else if(i == 7){
        dataBuffer[i] = ':';
      }else{
        dataBuffer[i] = char(rtcData.data[localCounter + i - 1]);
        rtcData.data[localCounter + i - 1]='\0';
      }
    }
    String strBuffer = String(F("DATA|")) + String(dataBuffer);
    //Serial.println(strBuffer);
    clientSocket.sendTXT(strBuffer);
    localCounter += 16;
  }
  rtcData.dataLength = 0;
}

bool turnOffSoftAP(){
    bool result = WiFi.softAPdisconnect(true);
    return result;
  }

int readRtcAndCounter(){ //Read RTC memory and return counter variable
  int counter;
  if (ESP.rtcUserMemoryRead(0, (uint32_t*) &rtcData, sizeof(rtcData))) {
    Serial.println("Read: ");
    printMemory();
    Serial.println();
  }
  counter  = rtcData.counter;
  if ((counter == 0)||(counter > 6)){
    counter = 6;
  }
  return counter;
}


void connectToIP(String IP){
  Serial.print("Try to connect to: ");
  Serial.println(IP);
  clientSocket.begin(IP, SOCKET_PORT, "/");
}


void goodBye(){
  clientSocket.sendTXT("Gomen ne, Watashi no server ja nai!");
  Serial.println("Disconnect from this server");
  clientSocket.disconnect();
}

uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

//prints all rtcData, including the leading crc32
void printMemory() {
  char buf[3];
  uint8_t *ptr = (uint8_t *)&rtcData;
  for (size_t i = 0; i < sizeof(rtcData); i++) {
    sprintf(buf, "%d", ptr[i]);
    Serial.print(buf);
    if ((i + 1) % 32 == 0) {
      Serial.println();
    } else {
      Serial.print(" ");
    }
  }
  Serial.println();
}

void writeRTC(){
  //update crc32
  rtcData.crc32 = calculateCRC32((uint8_t*) &rtcData.data[0], sizeof(rtcData.data));
  // Write struct to RTC memory
  if (ESP.rtcUserMemoryWrite(0, (uint32_t*) &rtcData, sizeof(rtcData))) {
    Serial.println("Write: ");
    printMemory();
    Serial.println();
  }
}
