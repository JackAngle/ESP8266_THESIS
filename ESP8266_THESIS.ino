#include "ESP8266WiFi.h"
#include "DHTesp.h"
#include <WebSocketsServer.h>
#include <WebSocketsClient.h>
#include <WebSockets.h>
#include <Hash.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <ESPAsyncTCP.h>



//#define AP_MODE_TIMEOUT_OUT 4000
#define AP_MODE 0
#define STA_MODE 1
#define SERVER_SSID "THESIS"
#define SERVER_PASSWORD "mustbedone"
#define SOCKET_PORT 8000
#define SOCKET_SERVER_IP "192.168.4.1"
#define IP_HEADER "192.168.4."
#define INTERVAL_BETWEEN_SENSOR_READ 3000
#define INTERVAL_BETWEEN_AP_MODE 30000
#define AP_MODE_TIMEOUT 10*1000 
#define STA_MODE_TIMEOUT 20*1000
#define DEEP_SLEEP_TIME 30*1000000
#define AP_SLEEP_BROADCAST_TIME 45*1000
#define AP_WORKING_TIME 60*1000

String mIPAddress = IP_HEADER;
int ipTail = 1;
bool isServerConnected = false;
bool isWiFiConnected = false;
 
DHTesp dht;
WebSocketsClient clientSocket;
WebSocketsServer webSocket = WebSocketsServer(SOCKET_PORT);


unsigned long currentTime = 0;
unsigned long lastSensorReadTime = 0;
unsigned long apStartTimer = 0;
unsigned long nextApStartTimer = INTERVAL_BETWEEN_AP_MODE;
//unsigned long apStopTimer = 0;
unsigned long staStartTimer = 0;
//unsigned long pingBroadcastTimer = 0;
unsigned long apSleepTimer = 0;
bool isAP = false;
bool isSTA = false;
bool isPingBroadcasted = false;

unsigned long randomNumber = random(15000);
//unsigned long staStopTimer = 0;

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
      if (!isServerConnected){
        if (mBuffer.startsWith("1")){//Check OpCode
          Serial.println("Right Header");
          String message = mBuffer.substring(mBuffer.indexOf('|') + 1);
          if (message == "Welcome to server!"){
            Serial.println("Yup, server desu~");
            isServerConnected = true;
            readSensorAndSendToServer();
            //TO-DO get server's IP and paste into EEFROM
//            writeIPToEEPROM(mIPAddress);          
          }
          else{
          goodBye();
          }
        }else{
          Serial.println("Wrong Header");
          Serial.println(mBuffer);
          goodBye();
        }
      }
      if (mBuffer.startsWith("AP_SLEEP")){
        String sleepTimeBuffer = mBuffer.substring(mBuffer.indexOf(':') + 1);
        int sleepTime = sleepTimeBuffer.toInt();
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
            if(mBuffer.startsWith("DATA|")){
              /*Read & send tata to Raspberry via Serial1*/
              String data = mBuffer.substring(mBuffer.indexOf('|') + 1);
              Serial.println(data);
              Serial1.println(data);

              /*Send sleep_time to client*/
              unsigned long clientSleepTime = AP_WORKING_TIME - currentTime + apSleepTimer + DEEP_SLEEP_TIME/1000 - 8000;
              Serial.println(clientSleepTime);
              String request = "AP_SLEEP:" + String(clientSleepTime);
              Serial.println(request);
              webSocket.sendTXT(num, request);
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
  dht.setup(D5, DHTesp::DHT22);
  /*Set up Soft-AP*/
  WiFi.mode(WIFI_AP);
  Serial.print("Setting soft-AP ... ");
  Serial.println(WiFi.softAP(SERVER_SSID, SERVER_PASSWORD, 1, false, 8)? "Ready" : "Failed!");

  Serial.print("Soft-AP IP address = ");
  Serial.println(WiFi.softAPIP());

  
  //wifiServer.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
//  pingBroadcastTimer = millis();
  apStartTimer = millis();
  //nextApStartTimer = apStartTimer + INTERVAL_BETWEEN_AP_MODE;
  Serial.println(apStartTimer);
  Serial.println(staStartTimer);
  
}
 
void loop() {
 
 
  currentTime = millis();

  if (currentMode == AP_MODE){
    /*Check if this is AP*/    
    if(WiFi.softAPgetStationNum() != 0){
      if(!isAP){
        isAP = true;
        apSleepTimer = millis();
      }
    }

    /*AP Sleep control*/
    if(isAP){
//      if( (unsigned long) (currentTime - apSleepTimer) > (AP_SLEEP_BROADCAST_TIME)){
////        webSocket.broadcastTXT("AP_SLEEP: 20s left");
////      }  
      if( (unsigned long) (currentTime - apSleepTimer) > (AP_WORKING_TIME)){
          Serial.println("AP Deep sleep ");
          ESP.deepSleep(DEEP_SLEEP_TIME, WAKE_RF_DEFAULT);
        }
    }else{    
      /*AP -> STA*/
      if( (unsigned long) (currentTime - apStartTimer) > (AP_MODE_TIMEOUT)){
        if((WiFi.softAPgetStationNum() == 0)&&(!isAP)){
          turnOffSoftAP();
          Serial.println("Timer STA");
          convertToStationMode();
          currentMode = STA_MODE;
          staStartTimer = millis();
        }
      }
    }
    /*SERVER WORKS*/

    /*Check the number of connected device(s)*/
  if(WiFi.softAPgetStationNum() == 0){
   //TO-DO
  }else{
    if (Serial.available() > 0){
      String incoming ="";
      incoming = Serial.readString();

      int num = incoming.charAt(0) - 48;
    
      Serial.println(incoming);
      Serial.println("Sending to client");
      webSocket.sendTXT(num, incoming);
      Serial.println("Done!"); 
    }else{
//      if ((currentTime - pingBroadcastTimer) > 50000){
//        webSocket.broadcastPing();
//        isPingBroadcasted = true;
//        pingBroadcastTimer = millis();
//      }
      webSocket.loop();
    }
  }
    
  }else{
    
     /*SHOW WIFI STATUS*/
    if(!isWiFiConnected){
       if(WiFi.status() == WL_CONNECTED){
        Serial.println("Connected to WiFi AP!");
        isSTA = true;
        isWiFiConnected = true;
        //connectToIP("192.168.4.1");
       }
    }else{
      if(WiFi.status() != WL_CONNECTED){
        Serial.println("Disconnected from WiFi AP!");
        isSTA = false;
        isWiFiConnected = false;
       }
    }

    if(!isSTA){
      /*Sleep scheduler for STA*/
      if(((unsigned long)(currentTime - staStartTimer)) > STA_MODE_TIMEOUT){
        if(!isWiFiConnected){
          Serial.println("Timeout Deep sleep ");
          ESP.deepSleep(random(30, 45)*1000000, WAKE_RF_DEFAULT);
        }
      }
    }else{
      staStartTimer = millis();
    }
    
    /*CLIENT WORKS*/
    if (Serial.available() > 0){
      String incoming ="";
      incoming = Serial.readString();
      Serial.print(incoming);
      Serial.println("Sending to server");
      clientSocket.sendTXT(incoming);
      Serial.println("Done!");  
    }
    else{      
      clientSocket.loop();
      }
  }
}

void readSensorAndSendToServer(){  
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
        request += String(percent);
        clientSocket.sendTXT(request);
        Serial.print("Request: ");
        Serial.println(request);       
}

bool turnOffSoftAP(){
    bool result = WiFi.softAPdisconnect(true);
    return result;
  }

void convertToStationMode(){
  webSocket.close();
  Serial.print("Turned off soft-AP ... ");
  /*Convert to STATION mode*/
  WiFi.mode(WIFI_STA);
  dht.setup(D5, DHTesp::DHT22);
  currentMode = STA_MODE; 
  delay(50);
  WiFi.begin(SERVER_SSID, SERVER_PASSWORD);
  if (WiFi.status() == WL_CONNECTED) {
   
  Serial.print("Connected to WiFi. IP:");
  Serial.println(WiFi.localIP());
  }
  
  /*Assign event handler*/
  clientSocket.onEvent(clientSocketEvent);
  Serial.println("Attached clientSocket");

  /* try every 2000ms again if connection has failed*/
  clientSocket.setReconnectInterval(2000);
}

//void convertToApMode(){
//  clientSocket.disconnect();
//  WiFi.disconnect(false);
//  Serial.print("Turned off STA ... ");
//  /*Convert to STATION mode*/
//  /*Set up Soft-AP*/
//  WiFi.mode(WIFI_AP);
//  Serial.print("Setting soft-AP ... ");
//  Serial.println(WiFi.softAP(SERVER_SSID, SERVER_PASSWORD, 1, false, 8)? "Ready" : "Failed!");
//
//  Serial.print("Soft-AP IP address = ");
//  Serial.println(WiFi.softAPIP());
//  webSocket.begin();
//  webSocket.onEvent(webSocketEvent);
//  currentMode = STA_MODE; 
//  delay(50);
//}

void connectToIP(String IP){
  Serial.print("Try to connect to: ");
  Serial.println(IP);
  clientSocket.begin(IP, SOCKET_PORT, "/");
}

void scanIP(){
  mIPAddress = IP_HEADER;
  //mIPAddress += 209;
  mIPAddress += ipTail++;
  connectToIP(mIPAddress);
  
}

void goodBye(){
  clientSocket.sendTXT("Gomen ne, Watashi no server ja nai!");
  Serial.println("Disconnect from this server");
  clientSocket.disconnect();
}

void writeIPToEEPROM(String IP){
  EEPROM.begin(16);

    char mBuffer[16];
    IP.toCharArray(mBuffer, 16); 
    EEPROM.put(0, mBuffer);
    EEPROM.commit();
}

String readIPFromEEPROM(){
  char IP[16];
  EEPROM.get(0, IP);
  EEPROM.end();
  /*
  for (int i = 0; i < 4; i++){
    IP += (int) EEPROM.read(i);
    if (i != 3){
      IP += '.';
    }
  }
  */
  Serial.print("Got IP ");
  Serial.println(IP);
  String mIPAddress = IP;

  return mIPAddress;
}
