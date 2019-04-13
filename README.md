# ESP8266_THESIS
Program which runs on NodeMCU of Thesis


The program has 2 main mode: Access Point (AP) & Station (STA)
AP_MODE:        |     WORK 10s           |                   DEEP_SLEEP 20s                |
STA_MODE:       |     WORK 10s           |            DEEP_SLEEP random(10, 30) (seconds)  |

Program's data is saved to RTC Memory. There's a (int) counter variable having value from 6 down to 1
6-5-4: AP_MODE
3-2-1: STA_MODE
When counter completed the first loop (6-5-4-3-2-1), it will randomly pick a value of (1, 6).

DEFAULT: 
- Check CRC32 if RTC memory's data is valid. If not, set all values to default.
- Load counter from RTC memory to decide working mode.
- At the end, couter will decrease by 1 in all working mode (except Raspberry's AP)

AP_MODE: 
- Set up a softAP: WiFi.softAP(SERVER_SSID, SERVER_PASSWORD, 1, false, 8);
- Set up webSocket with default IP "192.168.4.1" at port SOCKET_PORT.
- Receive data from Client(s) & save to RTC Memory (Maximum 31 data packets)
- If server receives final packet from client (which has "|END" footer), it will send its next wake-up time & tell client go to sleep.

* Raspberry's AP will send to client(s) random sleep time (15, 30)


STA_MODE
- Set up socketClient & try to connect to server's default IP "192.168.4.1" when connected to Server's WiFi.
- Load data from RTC & send to webSocket server if connected. Client after will send its data packet as final packet after sent all  data packet read from RTC memory.
 
