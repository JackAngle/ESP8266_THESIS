#include "ESP8266WiFi.h"
#include "DHTesp.h"
#include <WebSocketsServer.h>
#include <WebSocketsClient.h>
#include <WebSockets.h>
#include <Hash.h>
#include <EEPROM.h>

#define AP_MODE_TIME_OUT 4000
#define AP_MODE 0
#define STA_MODE 1
#define SERVER_SSID "THESIS"
#define SERVER_PASSWORD "mustbedone"
#define SOCKET_PORT 8000
#define IP_HEADER "192.168.4."

String mIPAddress = IP_HEADER;
int ipTail = 1;
 
const char* ssid = "Dont Ask";
const char* password =  "password";


const char* host = "192.168.4.1";
const uint16_t port = 23;
 
//WiFiServer wifiServer(23);
DHTesp dht;
WebSocketsClient clientSocket;
WebSocketsServer webSocket = WebSocketsServer(SOCKET_PORT);

unsigned long current = 0;
bool currentMode = AP_MODE;

/*CLIENT_WEBSOCKETEVENT handler*/
void clientSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
String mBuffer = (char*)payload;
  switch(type) {
    case WStype_DISCONNECTED:
      
        Serial.printf("[Client] Disconnected!\n");
        if (WiFi.status() != WL_CONNECTED){
          if (ipTail < 255){
            scanIP();
          } else{
          ipTail = 0;
          scanIP();
          }
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
      
      if (mBuffer.startsWith("1")){//Check OpCode
        Serial.println("Right Header");
        String message = mBuffer.substring(mBuffer.indexOf('|') + 1);
        if (message == "Welcome to server!"){
          Serial.println("Yup, server desu~");
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
  delay(5000);
}


/*WEBSOCKETEVENT HANDLER (server side)*/
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

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

  
  /*Time variable*/
  current = millis();
  


  //wifiServer.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

}
 
void loop() {
  if (currentMode == AP_MODE){
    /*SERVER WORKS*/

    /*Check the number of connected device(s)*/
  if(WiFi.softAPgetStationNum() == 0){
    if (((unsigned long)(millis() - current)) > AP_MODE_TIME_OUT){
      if (turnOffSoftAP() == true){
        convertToStationMode();
      }
    }
    delay(3000);
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
  EEPROM.begin(15);
  /*Convert to STATION mode*/
  WiFi.mode(WIFI_STA);
  dht.setup(D5, DHTesp::DHT22);
  currentMode = STA_MODE; 
  delay(50);
  WiFi.begin(SERVER_SSID, SERVER_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting..");
    delay(1000);
  }
  Serial.print("Connected to WiFi. IP:");
  Serial.println(WiFi.localIP());

  /*Assign event handler*/
  clientSocket.onEvent(clientSocketEvent);

  /* try every 2000 again if connection has failed*/
  clientSocket.setReconnectInterval(2000);
}

void scanIP(){
  mIPAddress = IP_HEADER;
  //mIPAddress += 209;
  mIPAddress += ipTail++;
  Serial.print("Try to connect to: ");
  Serial.println(mIPAddress);
  clientSocket.begin(mIPAddress, SOCKET_PORT, "/");
}

void goodBye(){
  clientSocket.sendTXT("Gomen ne, Watashi no server wa nai!");
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
