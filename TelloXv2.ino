/*
 * TelloX: Make your Tello fly autonomously!
 * 
 * Version 2
 * by: RWSenser
 * 2019-04-28
 * 
   Notice: Some code based on Wifi101 library examples: 'ScanNetworks' and 'WifiUdpSendReceiveString'

MIT License

Copyright (c) 2019 rwsenser

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 * 
 */

 /*
  *
  *                        "It is not pretty -- but it works."
  *                        
  * This code is the result of a software engineering proof-of-concept effort that
  * turned out to be very capable.  Please use with caution -- use at your own risk.
  * 
  * You will need the Arduino IDE install (version 1.8x or better) and at least some
  * ability to change C++ code, compile, and download the program to the Feather M0
  * 
  * View this code as marginally tested, experimental code.
  * 
  * This code works with a Ryze/DJI Tello and an Adafruit Feather M0 Wifi (PID 3010)
  * and only requires the mininal DJI Tello API 1.0 (this means no camera support).
  *   . needed is a copy of the Tello SDK 1.0 document as it lists the supported commands
  *   . added is the 'wait <seconds>' command
  * 
  * Please attach a leash (10 foot +/- string) to your Tello for your first few autonoumous flights
  * as your normal Tello controller will be totally ignored.  The Tello and Feather M0 communicate via WiFi.
  * 
  * Perhaps try DEBUG_MODE first.
  * 
  * Above all: Be safe!
  * 
  */

// kinda useful for testing  without Tello connected
// DEBUG MODE:
#undef DEBUG_MODE
// #define DEBUG_MODE 
//++++++++++
// TelloX port: 
// 2019-04-26: Recode using "thread" style
//             each thread has a return value that provides the update to the current clock value to re-execute
//             Also added code to find Tellos via WiFi
//             Added 'wait' command to list of supported commands
//

// identified future todos:
// * add code to land tello on any Tello error messages
//
// wish list:
// * add mode to read 'flightPlan' from different WiFi network before flight
//++++++++++

#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// FLIGHTPLAN:
const String flightPlan[] = { 
  "wait 5", "takeoff",
  "wait 5", "cw 90",
  "wait 5", "forward 50",
  "wait 5", "ccw 180",
  "wait 5", "forward 50",
  "wait 5", "cw 90",
  "wait 5", "land",
  "wait 5", "land",
  "", ""};
// end FLIGHTPLAN  
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// unless you are coding, do not change below this line

// global variables:
String ssid;
const unsigned int localPort = 8888;      // local port to listen on
const char TelloIP[] = "192.168.10.1";
const unsigned int TelloPort = 8889;
const char TelloSDKon[] = "command";
int numCommands;  
int status = WL_IDLE_STATUS;
char packetBuffer[255]; 
char cmdBuffer[255];
int cnt;
bool waited_thread3;
WiFiUDP Udp;
// thread control fields
long thread1_LED_clock = 0;
long thread2_INI_clock = 0;
long thread3_CNN_clock = 0;
long thread4_CMD_clock = 0;
bool thread2_INI_active = false;
bool thread3_CNN_active = false;
bool thread4_CMD_active = false;
// LED constants
const int LED_DELAY_OK = 50;
const int LED_DELAY_DONE = 2500;
const int LED_DELAY_ERROR = 5000;
const int LED_DELAY_SCAN = 500;
int LED_delay = LED_DELAY_SCAN;

void setup() {
  // Configure pins for Adafruit ATWINC1500 Feather
  WiFi.setPins(8,7,4,2); 
  pinMode(13, OUTPUT);  
  digitalWrite(13, LOW);        
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
#ifdef DEBUG_MODE
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println(">>DEBUG MODE<<");  
#endif
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true) {
      digitalWrite(13, !digitalRead(13));   
      delay(25);   
    };
  }
  // numCommands = flightPlan.length();
  thread2_INI_active = true;
  thread3_CNN_active = false;
  thread4_CMD_active = false;
  cnt = 0;
  // dispatcher now takes over from loop()
}

//
// some time you just have to blink!
//
long unsigned int thread1_LED () {
  digitalWrite(13, !digitalRead(13));
  return LED_delay;
}
//
// Connect to Tello while allowing blinking LED
//
long unsigned int thread2_INI () {
#ifdef DEBUG_MODE
  thread2_INI_active = false;
  thread3_CNN_active = false;
  thread4_CMD_active = true;
  LED_delay = LED_DELAY_OK;
  return 0;
}  
#else
  // find the tello!
  // attempt to connect to WiFi network:
  if ( status != WL_CONNECTED) {
    ssid = findTello();
    if (ssid.length() > 0) {
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid);
      // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
      status = WiFi.begin(ssid); //, pass);
    } else {
      Serial.println("Pause and scan again for Tello");      
    }
    // wait 10 seconds for connection:
    return 10000;
  }
  Serial.println("Connected to wifi/Tello");
  // printWiFiStatus();
  Serial.println("Turn on Tello SDK"); 
  // turn on the UDP access  
  Udp.begin(localPort);
  // notify Tello to enter SDK mode
  Udp.beginPacket(TelloIP, TelloPort);
  Udp.write(TelloSDKon);
  Udp.endPacket();  
  thread2_INI_active = false;
  thread3_CNN_active = true;    
  waited_thread3 = false;  
  return 0;  // bye bye
}
#endif   
//
// clunking way to wait 2000 millis, but the LED can blink 
// this could be done without this extra thread...
//
long unsigned int thread3_CNN () { 
  if (!waited_thread3) { 
    waited_thread3 = true;
    return 2000; // the needed delay
  }
  thread3_CNN_active = false;
  thread4_CMD_active = true;
  LED_delay = LED_DELAY_OK;  
  return 0;
}  
 
//
// Time to fly the Tello
//
long unsigned int thread4_CMD () {
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    Serial.print("Tello reply (");
    Serial.print(packetSize);
    Serial.print("):");    
    Serial.println(packetBuffer);
  }
  String sCmd = flightPlan[cnt];  
  if (sCmd.length() > 0) {  
    int waitTime = 0;
    if (sCmd.startsWith("wait ")) {
      // process wait operation here
      Serial.print("Local: ");      
      Serial.println(sCmd); 
      const char* numPtr = sCmd.substring(5).c_str();
      waitTime = atoi(numPtr) * 1000;     
    } else {
      // otherwise send command to Tello
      for (int i=0; i < sCmd.length(); i++) {
        if (!isprint(sCmd[i])) break;
        cmdBuffer[i] = sCmd[i];
        cmdBuffer[i+1] = '\0';
      }
      Serial.print("To Tello: ");
      Serial.println(cmdBuffer);
#ifdef DEBUG_MODE
      Serial.println("DEBUG_MODE, not sent!");
#else        
      Udp.beginPacket(TelloIP, TelloPort);
      Udp.write(cmdBuffer, strlen(cmdBuffer));
      Udp.endPacket();
      waitTime = 5000;  
    } 
    cnt++;  
    return(waitTime);
  } else {
    cnt = 0;
    Serial.println("**PAUSE & RESET**");
    LED_delay = LED_DELAY_DONE;
    thread4_CMD_active = false; // kill myself :)
  }
#endif
  return 0;
}

//++++++++++
void loop() {
  // dispatacher ((this would be much better done with a dispatcher class....))
  if (millis() >= thread1_LED_clock) {
    thread1_LED_clock = millis() + thread1_LED();
  }
  if (thread2_INI_active && millis() >= thread2_INI_clock) {
    thread2_INI_clock = millis() + thread2_INI();
  }  
  if (thread3_CNN_active && millis() >= thread3_CNN_clock) {
    thread3_CNN_clock = millis() + thread3_CNN();
  }  
  if (thread4_CMD_active && millis() >= thread4_CMD_clock) {
    thread4_CMD_clock = millis() + thread4_CMD();
  }
}
//++++++++++
//
// print the WiFi Status
// not really used
//
void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
//
// Find Tell SSID
//
String findTello() {
  // scan for nearby networks:
  String myTello = "";
  int myCnt = 0;
  // Serial.println("** Scan Networks **");
  int numSsid = WiFi.scanNetworks();
  if (numSsid == -1)
  {
    Serial.println("Couldn't get a wifi connection");
    while (true);
  }
  // print the network number and name for each network found:
  for (int thisNet = 0; thisNet < numSsid; thisNet++) {
    String SSIDname = WiFi.SSID(thisNet);
    // Serial.println(SSIDname);    
    if (SSIDname.startsWith("TELLO-")) {
      myTello = SSIDname;
      myCnt++;
    }  
  }
  if (myCnt > 1) {
    myTello = ""; // too many Tellos close by
    Serial.println("Error: Too many Tellos close by!");    
  }
  return myTello;
}
/* end of code */
