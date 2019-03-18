#include "ESP8266WiFi.h"
 
const char* ssid = "Dont Ask";
const char* password =  "password";
 
//WiFiServer wifiServer(80);

unsigned long current = 0;

void setup() {
 
  Serial.begin(115200);
  /*Set up Soft-AP*/
  WiFi.mode(WIFI_AP);
  
  Serial.print("Setting soft-AP ... ");
  Serial.println(WiFi.softAP("THESIS", "mustbedone", 1, false, 8)? "Ready" : "Failed!");

  Serial.print("Soft-AP IP address = ");
  Serial.println(WiFi.softAPIP());
  
  /*Time variable*/
  current = millis();
  
//  WiFi.mode(WIFI_STA);
//  delay(1000);
// 
//  WiFi.begin(ssid, password);
// 
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(1000);
//    Serial.println("Connecting..");
//  }
// 
//  Serial.print("Connected to WiFi. IP:");
//  Serial.println(WiFi.localIP());
// 
//  wifiServer.begin();


}
 
void loop() {
  if(1){
    /*If*/
    if (((unsigned long)(millis() - current)) > 30000){
      if (turningOffAP() == true){
        WiFi.mode(WIFI_STA);
        delay(1000);
        WiFi.begin(ssid, password);
        Serial.print("Connected to WiFi. IP:");
        Serial.println(WiFi.localIP());
        current = 100000000;
      }
    }
    delay(3000);
    Serial.println(millis());
  }

// if ( (unsigned long) (millis() - current) > 3000){
//  WiFiClient client = wifiServer.available();
// 
//  if (client) {
// 
//    while (client.connected()) {
// 
//      while (client.available()>0) {
//        char c = client.read();
//        Serial.write(c);
//      }
// 
//      delay(10);
//    }
// 
//    client.stop();
//    Serial.println("Client disconnected");
//  }
//  }
// else {
//    current = millis();
//    Serial.println("AP mode ativating");
//    WiFi.mode(WIFI_AP);
//    Serial.println("AP mode ativating....");
//    WiFi.softAP("Hahaha", "lalalala");
//    Serial.println("AP mode ativated");
//  }
//  delay(50);
}



bool turningOffAP(){
    Serial.print("Turning off soft-AP ... ");
    bool result = WiFi.softAPdisconnect(true);
    Serial.println(result? "Off" : "Failed!");
    return result;
  }
