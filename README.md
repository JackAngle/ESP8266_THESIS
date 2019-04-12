# ESP8266_THESIS
Program which runs on NodeMCU of Thesis


The program has 2 main mode: Access Point (AP) & Station (STA)
AP_MODE:        |     WORK 10 seconds    |                   DEEP_SLEEP 20s                |
STA_MODE:       |     WORK 10s           |            DEEP_SLEEP random(10, 30) (seconds)  |

Program's data is saved to RTC Memory. There's a (int) counter variable having value from 6 down to 1
6-5-4: AP_MODE
3-2-1: STA_MODE
When counter completed the first loop (6-5-4-3-2-1), it will randomly pick a value of (1, 6).

AP_MODE: 
