#include "ESP8266WiFi.h"
#include "DHTesp.h"
#include <WebSocketsServer.h>
#include <WebSocketsClient.h>
#include <WebSockets.h>
#include <Hash.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <ESPAsyncTCP.h>



#define AP_MODE_TIME_OUT 4000
#define AP_MODE 0
#define STA_MODE 1
#define SERVER_SSID "THESIS"
#define SERVER_PASSWORD "mustbedone"
#define SOCKET_PORT 8000
#define IP_HEADER "192.168.4."
#define INTERVAL_BETWEEN_SENSOR_READ 3000
#define INTERVAL_BETWEEN_AP_MODE 30000
#define AP_MODE_TIME 5000 
#define STA_MODE_TIME 10000
#define DEEP_SLEEP_TIME 10000000

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
unsigned long pingBroadcastTimer = 0;
bool isPingBroadcasted = false;

unsigned long randomNumber = random(15000);
//unsigned long staStopTimer = 0;

bool currentMode = AP_MODE;


//void updateApTimer(){
//  apStartTimer += INTERVAL_BETWEEN_AP_MODE;
//  apStopTimer = apStartTimer + 5000;
//  staStartTimer = apStopTimer;
//}
//
///*|   AP(5)   |   RANDOM(15)   |   STA(10)   |*/
//void updateAllTimer(){
//    updateApTimer();
//    staStartTimer = apStopTimer + random(15000);
//    staStopTimer = staStartTimer;
//  }

  /*CLIENT_WEBSOCKETEVENT handler*/
void clientSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
String mBuffer = (char*)payload;
  switch(type) {
    case WStype_DISCONNECTED:
        if (isServerConnected){
          isServerConnected = false;
        }
        Serial.printf("[Client] Disconnected!\n");

//          if (ipTail < 255){
//            scanIP();
//          } else{
//          ipTail = 0;
//          scanIP();
//          }
        if(isWiFiConnected){
          connectToIP("192.168.4.1");
        }
      break;
    case WStype_CONNECTED: {
     
        Serial.print("[This Client] Connected to: ");   
        Serial.println(mIPAddress);    

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
            //TO-DO get server's IP and paste into EEFROM
            writeIPToEEPROM(mIPAddress);          
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
            if(mBuffer.lastIndexOf(':') != -1){
              Serial1.print(mBuffer);
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
  //Serial.setDebugOutput(true);

//  for(uint8_t t = 4; t > 0; t--) {
//    Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
//    Serial.flush();
//    delay(1000);
//  }
  Serial.print("Setting soft-AP ... ");
  Serial.println(WiFi.softAP(SERVER_SSID, SERVER_PASSWORD, 1, false, 8)? "Ready" : "Failed!");

  Serial.print("Soft-AP IP address = ");
  Serial.println(WiFi.softAPIP());

  
  //wifiServer.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  pingBroadcastTimer = millis();
  apStartTimer = millis();
  //nextApStartTimer = apStartTimer + INTERVAL_BETWEEN_AP_MODE;
  Serial.println(apStartTimer);
  Serial.println(staStartTimer);
  
}
 
void loop() {
 
 
  currentTime = millis();
  if (currentMode == AP_MODE){/*AP -> STA*/
    if( (unsigned long) (currentTime - apStartTimer) > (AP_MODE_TIME+randomNumber)){
      if(WiFi.softAPgetStationNum() == 0){
        turnOffSoftAP();
        Serial.println("Timer STA");
        convertToStationMode();
        currentMode = STA_MODE;
        //staStartTimer = millis();
      }
//        apStartTimer = nextApStartTimer;
//        nextApStartTimer += INTERVAL_BETWEEN_AP_MODE;   
    }
  }else{
     /*SHOW WIFI STATUS*/
    if(!isWiFiConnected){
       if(WiFi.status() == WL_CONNECTED){
        Serial.println("Connected to WiFi AP!");
        isWiFiConnected = true;
        //connectToIP("192.168.4.1");
       }
    }else{
      if(WiFi.status() != WL_CONNECTED){
        Serial.println("Disconnected tfrom WiFi AP!");
        isWiFiConnected = false;
       }
    }
    
    /*STA -> AP (future will be AP>STA>SLEEP>AP)*/
    if(((unsigned long)(currentTime - apStartTimer)) > INTERVAL_BETWEEN_AP_MODE){
//      Serial.println("Deep sleep ");
//      ESP.deepSleep(DEEP_SLEEP_TIME, WAKE_RF_DEFAULT);
    }
  }

 
  if (currentMode == AP_MODE){
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
      if ((currentTime - pingBroadcastTimer) > 50000){
        webSocket.broadcastPing();
        isPingBroadcasted = true;
        pingBroadcastTimer = millis();
      }
      webSocket.loop();
    }
  }
    
  }else{
    /*CLIENT WORKS*/
    if (Serial.available() > 0){
      String incoming ="";
      incoming = Serial.readString();
      Serial.print(incoming);
      Serial.println("Sending to server");
      clientSocket.sendTXT(incoming);
      Serial.println("Done!");  
    }else if (((unsigned long)(millis() - lastSensorReadTime)) > INTERVAL_BETWEEN_SENSOR_READ) {
        if (isServerConnected){
        float humidity = dht.getHumidity();
        float temperature = dht.getTemperature();
        String request = ESP.getChipId()+ String(F(": "));
        if (dht.getStatusString() == "OK"){
          request += String(temperature, 1);
          request += String(humidity, 1);
          clientSocket.sendTXT(request);
        } else{
          request += "000";
          request += "000";
          clientSocket.sendTXT(request);
        }      
        Serial.println("sensor");  
      } 
      lastSensorReadTime = millis();
    }else{      
      clientSocket.loop();
      }
  }
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
