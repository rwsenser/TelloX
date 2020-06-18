/*
   TelloX: Make your Tello fly autonomously!

   Version 2b
   by: RWSenser
   2020-06-12 (restart)
   2019-04-28 (rework 2019-05-24 to add telnet support)

   Notice: Some code based on Wifi101 library examples: 'ScanNetworks' and 'WifiUdpSendReceiveString'

  MIT License

  Copyright (c) 2020 rwsenser

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

*/

/*

                          "It is not pretty -- but it works."

   This code is the result of a software engineering proof-of-concept effort that
   turned out to be very capable.  Please use this with caution -- use at your own risk.

   You will need the Arduino IDE installed (version 1.8x or better) and at least some
   ability to change C++ code, compile, and download this program to the Feather M0.
   Also needed is the Adafruit WiFi101 library installed in the IDE -- see Adafruit
   website(s).

   View this as partially tested, experimental code.

   This code works with a Ryze/DJI Tello and an Adafruit Feather M0 Wifi (PID 3010)
   and only requires the mininal DJI Tello API 1.0 (this means no camera support).
     . needed is a copy of the Tello SDK 1.0 document as it lists the supported commands
     . added are the 'wait <seconds>' and 'wtok <seconds> commands.  wtok waits for a
       reply from the Tello and times out after <seconds>.

   Testing has been done with the Feather M0 attached to the Tello, using a smallish 250 mah lipo. Be
   sure to watch the JST jack polarity.  Having to switch the red and black leads is common!


   Please attach a leash (10 foot +/- string) to your Tello for your first few autonomous flights
   as your normal Tello controller will be totally ignored.  The Tello and Feather M0 communicate via WiFi.

   Perhaps try DEBUG_MODE first. :)

   The 'telnet' mode is not used with AUTO_RUN.  A good first test is to have DEBUG on, AUTO_RUN on,
   TRANS off and SERIAL on.  With a USB cable attached, the M0 will function, write serial output
   to the Arduino IDE terminal and NOT, NOT, NOT communicate with the Tello.

   The 'telnet' mode requires its own WIFI network.

   There is a PC Java program provided that receives the after-the-fact TRANS log.  It uses UDP
   port 12345 and the PC firewall will need to open this port for UDP. 

   Above all: Be safe!

*/
// CONFIGURATION MODES:
// defaults: DEBUG off, AUTO_RUN on, TRANS_LOG off, LOCAL_SERIAL on 
// DEBUG MODE:  no commands sent to Tello
#undef DEBUG_MODE
// #define DEBUG_MODE
//
// AUTO RUN MODE: skip telnet access and after a delay, start the Tello 
#undef AUTO_RUN_MODE
#define AUTO_RUN_MODE
//
// TRANS LOG MODE: use UDP, port 12345, to send combined log
#undef TRANS_LOG_MODE
#define TRANS_LOG_MODE
//
// LOCAL_SERIAL_ON: sends log data to IDE serial port (needs USB cable)
#undef LOCAL_SERIAL_ON
// #define LOCAL_SERIAL_ON
//
//++++++++++
// todo:
// 2020-06-17:  1) Improve wait-for-OK code, current version continues on any returned value ... too simple
//              2) Improve watch dog coding (too much B.F.)
//   
// 2020-06-15:  1) Done, V1, Add code to wait for OK or Error or ... from Tello
//              2) Done, Add watch dog timmer to autoland at timeout
//
//TelloXv4 port:
// 2020-06-18: Substantially done!
// 2020-06-16: Add threads for wait processing and watch dog
// 2020-06-15: Code cleanup, add AUTO_RUN_MODE, and fix Millis()/Delay() error (off by times 2)
// 2020-06-11: Restart project
//TelloXv2 port:
// 2019-05-24: Lots of debugging ... keeping Wifi object used corretly...
//
// 2019-05-22: Switch Wifi to "TelloXlink" and begin to add telnet support.
//             Goal is to be able to load flightplans and dump logs.
//
// 2019-05-17: Add logging support, remove old log-like Serial.prints...
//             Uses UDP to hardcoded network and IPaddress/port???
// TelloX port:
// 2019-04-26: Recode using "thread" style
//             each thread has a return value that provides the update to the current clock value to re-execute
//             Also added code to find Tellos via WiFi
//             Added 'wait' command to list of supported commands
//

//++++++++++
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <cctype>

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// FLIGHTPLAN:
const long int watchDogMaxTime = 30 * 1000; // in m.s.
const String flightPlan[] = {
#if 0
  // ground test -- do me once as a test in DEBUG MODE, no Tello needed 
  "wait 1",
  "battery?",
  "wtok 5",
  "speed?",
  "wtok 5",
  "time?",
  "wtok 5", 
  "wait 15",
#else
   // flying test
  "wait 1",
  "battery?",
  "wtok 1",
  "speed?",
  "wtok 1",
  "time?",
  "wtok 1",  
  "wait 1",
  "takeoff",
  "wait 2",
  "cw 180",
  "wtok 2",
  "down 60",
  "wtok 2",    
  "ccw 180",
  "left 50", 
  "wtok 1", 
  "right 50",  
  "wtok 1",
  "land",
  "wtok 1",      
  "battery?",
  "wtok 1",
  "time?",
  "wtok 1",  
  "land",
  "land",
  "land",  
  // "wait 5", "cw 180",
  // "wait 5", "up 60",
  // "wait 5", "left 50",
  // "wait 5", "down 60",
  // "wait 5", "right 50"
  // "wait 5", "up 70",
  // "wait 5", "down 80",
  // "wait 30", "ccw 90",
  // "wait 5", "land",
  // "wait 5", "land",
  // "wait 5", "error",
  // "wait 5", "battery?",
#endif
  "", ""
};
// end FLIGHTPLAN
// TelloX base network location
const char TelloXlinkSSID[] = "TelloXlink";
const char TelloXlinkPW[] = "11111111";
const char TelloXlinkIP[] = "10.10.10.2";
const int TelloXlinkPort = 12345;
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
// unless you are a C++ coder, do not change below this line
//
// This is coded like a "script" and not so much like a "product"
//
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
bool telloActive = false;
bool telloPanic = false;
unsigned long int watchEndTicks = 0;
unsigned long int waitEndTicks = 0;
// should be an enum ....
const long replyOK = -1;
const long replyError = -2;
const long replyOther = -3;
const long replyNone = -4;
//
long replyValue = replyNone;
boolean titleShown = false;
bool waited_thread3;
WiFiUDP Udp;
WiFiServer telnetServer(23);
String TelloSSID = "";
// thread control fields
long thread_clock = 0;
long thread0_LED_clock = 0;
bool thread1a_telnet_active = false;
bool thread1b_telnet_active = false;
bool thread1c_telnet_active = false;
bool thread2_INI_active = false;
bool thread3_CNN_active = false;
bool thread4_CMD_active = false;
bool thread4_WAIT_active = false;
bool thread4_WAITOK_active = false;
bool thread4_listen_active = false;
bool thread5_cleanup_active = false;
// LED constants
const int LED_NO_SHIELD = 25;
const int LED_NO_TELLO = 25;
const int LED_DELAY_OK = 50;
const int LED_DELAY_DONE = 2500;
const int LED_DELAY_ERROR = 5000;
const int LED_DELAY_SCAN = 500;
int LED_delay = LED_DELAY_SCAN;
String logData = "";
const int LOG_REC_TEXT_SIZE = 64;
const int LOG_REC_ARRAY_SIZE = 256;
struct logRec {
  int count;
  long millis;
  char mode;
  char text[LOG_REC_TEXT_SIZE];
};
int logDataUsed = 0;
bool logWrapped = false;
logRec logData2[LOG_REC_ARRAY_SIZE];

// Fix millis() and delay() errors (needed with Adafruit M0....)
unsigned long Xmillis() {
  return millis() / 2;
}
void Xdelay(unsigned long v) {
  delay(v * 2);
}
// end Fix
//
// logEvent -- write to log
//
void logEvent(int cnt, String mode, String event) {
  logData2[logDataUsed].count = cnt;
  logData2[logDataUsed].millis = Xmillis();
  logData2[logDataUsed].mode = mode[0];
  strncpy(logData2[logDataUsed].text, event.c_str(), LOG_REC_TEXT_SIZE);
#ifdef LOCAL_SERIAL_ON 
  Serial.print(logData2[logDataUsed].count);
  Serial.print(",");
  Serial.print(logData2[logDataUsed].millis);
  Serial.print(",");
  Serial.print(logData2[logDataUsed].mode);
  Serial.print(",");
  Serial.println(logData2[logDataUsed].text);
#endif  
  logDataUsed++;
  if (logDataUsed >= LOG_REC_ARRAY_SIZE) {
    logDataUsed = 0;  // blah, wrapped the buffer!
    logWrapped = true;
  }
}
//
// logTrans -- transmit log
//
// not threaded ...
//
void logTrans(const char ssid[], const char pass[], const char ipAddress[], const int ipPort ) {
  int status = WL_IDLE_STATUS;
  int retryCnt = 0;
  int maxCnt = 60000 / LED_DELAY_SCAN;
  while ( status != WL_CONNECTED) {
    // Serial.print("Attempting to connect to SSID: ");
    // Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    // wait & blink
    Xdelay(LED_DELAY_SCAN);  // likely .5 secs
    retryCnt++;
    digitalWrite(13, !digitalRead(13));   
    if (retryCnt > maxCnt) {
      // give up
      return;
    }
  }
  // Serial.println("Connect to WiFi");
  Udp.begin(localPort);
  int top = logDataUsed;
  if (logWrapped) {
    logDataUsed =  LOG_REC_TEXT_SIZE;
    // say something
    String txt = "Warning: Buffer Wrapped";
    Udp.beginPacket(ipAddress, ipPort);
    Udp.write(txt.c_str(), strlen(txt.c_str()));
    Udp.endPacket();
    Xdelay(100);  // prevents lost UDP packets...    
  }
  for (int k = 0; k < top; k++) {
    String packet = String(logData2[k].count) + ',' + String(logData2[k].millis) + ',' +
                    logData2[k].mode + ',' + String(logData2[k].text) + '\n';
    Udp.beginPacket(ipAddress, ipPort);
    Udp.write(packet.c_str(), strlen(packet.c_str()));
    Udp.endPacket();
    Xdelay(100);  // prevents lost UDP packets...
#ifdef LOCAL_SERIAL_ON    
    Serial.print(packet);
#endif    
  }
  Udp.stop();
  WiFi.end();
  return;
}
//
// Find Tello SSID
//
String findTello() {
  // scan for nearby networks:
  String myTello = "";
  int myCnt = 0;
  int numSsid = WiFi.scanNetworks();
  if (numSsid == -1)
  {
    logEvent(cnt, "E", "Couldn't get a wifi connection");
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
    myTello = ""; 
    logEvent(cnt, "E", "Error: Too many Tellos close by!");
  }
  return myTello;
}
//
// start/restart
//
void startThreads(bool restart) {
  thread1a_telnet_active = true;
  thread1b_telnet_active = false;
  thread1c_telnet_active = false;
  thread2_INI_active = false;
  thread3_CNN_active = false;
  thread4_CMD_active = false;
  thread4_WAIT_active = false;
  thread4_WAITOK_active = false;  
  thread5_cleanup_active = false;  
  titleShown = false;
  status = WL_IDLE_STATUS;
  if (restart) {
    // leave cnt as is...
    logEvent(cnt, "P", "**RESTART**");
  } else {
    cnt = 0;
    logEvent(cnt, "P", "**BEGIN**");
  }
}
void setup() {
#ifdef LOCAL_SERIAL_ON
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#endif  
  // Configure pins for Adafruit ATWINC1500 Feather
  WiFi.setPins(8, 7, 4, 2);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  //Initialize serial
#ifdef LOCAL_SERIAL_ON  
  Serial.begin(9600);
  Xdelay(500);  // slight delay to let Serial start
#endif  
#ifdef DEBUG_MODE
  logEvent(cnt, "P", ">>DEBUG MODE<<");
#endif
#ifdef AUTO_RUN_MODE
  logEvent(cnt, "P", ">>AUTO RUN MODE<<");
#endif
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    // Serial.println("WiFi shield not present");
    logEvent(cnt, "P", ">>WiFi shield not present<<");    
    // don't continue:
    while (true) {
      digitalWrite(13, !digitalRead(13));
      Xdelay(LED_NO_SHIELD);
    };
  }
  // find Tello when NOT in debug mode
#ifdef DEBUG_MODE
  TelloSSID = "DEBUG";
#else
  // code not threaded...
  while (true) {
    TelloSSID = findTello();
    if (TelloSSID.length() > 0) {
      // Serial.print("TelloSSID: ");
      // Serial.println(TelloSSID);
      logEvent(cnt, "P", "Tello SSID: " + TelloSSID);
      break;
    }
    // Serial.println("No Tello found");
    logEvent(cnt, "P", ">>No Tello found<<");    
    for (int k = 0; k < 50; k++) { // wait 50 times LED_NO_TELL
      digitalWrite(13, !digitalRead(13));
      Xdelay(LED_NO_TELLO);
    }
  }
#endif
  //
  startThreads(false);
  // dispatcher now takes over
}
//
// some times you just have to blink!
//
long unsigned int thread0_LED () {
  digitalWrite(13, !digitalRead(13));
  return LED_delay;
}
//
// telnet connect!
//
long unsigned int thread1a_telnet () {
  telloActive = false;
  // attempt to connect to WiFi network:
  LED_delay = LED_DELAY_SCAN;
#ifndef AUTO_RUN_MODE
  // telnet then maybe run
  if ( status != WL_CONNECTED) {
    // Serial.println("Try connect to telnet link.");
    status = WiFi.begin(TelloXlinkSSID, TelloXlinkPW);
  }
  IPAddress localIP;
  localIP = WiFi.localIP();
  // Serial.print("Local IP: ");
  // Serial.println(localIP);
  // next thread...
  thread1a_telnet_active = false;
  thread1b_telnet_active = true;
  // wait 10 seconds for stable telnet connection:
  return 10000;
#else
  // autorun
  logEvent(cnt, "P", "**AUTO RUN waiting**");
  thread1a_telnet_active = false;
  thread2_INI_active = true;
  titleShown = false;
#ifdef DEBUG_MODE
  return 1000;    // just enough to read output
#else    
  return 5000;  // 5 seconds to get clear of Tello
#endif  
#endif
}
//
// telnet begin!
//
long unsigned int thread1b_telnet () {
  if (status != WL_CONNECTED) {
    // Serial.println("retry...");
    // prev thread...
    thread1a_telnet_active = true;
    thread1b_telnet_active = false;
    return 1000;
  }
  telnetServer.begin();
  // Serial.println("Telnet up.");
  logEvent(cnt, "P", "**Telnet up.**");
  thread1b_telnet_active = false;
  thread1c_telnet_active = true;
  return 500;
}
//
// telnet do commands!
//
long unsigned int thread1c_telnet () {
  // check for client:
  WiFiClient client = telnetServer.available();
  if (client) {
    if (!titleShown) {
      telnetServer.println("**TelloX**");
      telnetServer.println("**[enter 1 char of: R(un), D(isplay), L(og), C(lear Log), X(:exit) ]**");
      titleShown = true;
    }
    if (client.available() > 0) {// we have something to do!
      // Serial.println("Client active.");
      char ch = client.read();
      if (ch == 13 || isprint(ch)) {
        // Serial.print(">");
        // Serial.println(ch);
        telnetServer.write(ch); // echo, echo, echo, ....
        char msg[8] = {"Ch:X"};
        if (ch == 13) {
          msg[3] = '?';
        } else {
          msg[3] = ch;
        }
        logEvent(cnt, "T", msg);
        // switch failed here so using chained if ... else if ..   blah
        ch = toupper(ch);
        if (ch == 'R') { // Run
          telnetServer.println("**R: flightplan**");
          telnetServer.println("**Telnet dropped!**");
          Xdelay(100);
          client.stop();
          WiFi.end();
          thread1c_telnet_active = false;
          thread2_INI_active = true;
          titleShown = false;
          return 500;
        } else if (ch == 'L') { // Log
          telnetServer.println("**L: log display**");
          int top = logDataUsed;
          if (logWrapped) logDataUsed =  LOG_REC_TEXT_SIZE;
          for (int k = 0; k < top; k++) {
            String msg = String(logData2[k].count) + ',' + String(logData2[k].millis) + ',' +
                         logData2[k].mode + ',' + String(logData2[k].text);
            telnetServer.println(msg);
            // Serial.println(msg);
          }
        } else if (ch == 'D') {
          telnetServer.println("**D: flightplan**");
          int j = 0;
          while (flightPlan[j].length() > 0) {
            telnetServer.print("(" + String(j) + ")");
            telnetServer.println(flightPlan[j]);
            j++;
          }
        } else if (ch == 'C') {
          telnetServer.println("**C: clear log**");
          logDataUsed = 0;
          logWrapped = false;
        }  else if (ch == 'X') {
          telnetServer.println("**X: eXit Telnet**");
          telnetServer.println("**Telnet dropped!**");
          Xdelay(100);
          client.stop();
          WiFi.end();
          // restart
          startThreads(true);
        }
      }
    }
  }
  return 100;
}
//
// Connect to Tello while allowing blinking LED
// ((the fun starts here!))
//
long unsigned int thread2_INI () {
#ifdef DEBUG_MODE
  thread2_INI_active = false;
  thread3_CNN_active = true;
  thread4_CMD_active = false;
  LED_delay = LED_DELAY_OK;
  return 0;
#else
  // attempt to connect to Tello's WiFi network:
  status = WiFi.begin(TelloSSID); //, pass);
  if (status != WL_CONNECTED) {
    // Serial.println("Pause & Reconn Tello");
    // wait 10 seconds for connection:
    return 10000;
  }
  // Serial.println("Connected to wifi/Tello");
  logEvent(cnt, "P", "Connected:" + ssid);
  // printWiFiStatus();
  // turn on the UDP access
  Udp.begin(localPort);
  // notify Tello to enter SDK mode
  Udp.beginPacket(TelloIP, TelloPort);
  Udp.write(TelloSDKon);
  Udp.endPacket();
  logEvent(cnt, "P", TelloSDKon);
  thread2_INI_active = false;
  thread3_CNN_active = true;
  waited_thread3 = false;
  return 0;  // bye bye
#endif
}
//
// clunky way to wait, but the LED can blink
//
long unsigned int thread3_CNN () {
  if (!waited_thread3) {
    waited_thread3 = true;
    return 2000; // the needed delay
  }
  // time to go to work!
  telloActive = true;
  telloPanic = false;
  // let the dog watch, barks loudly it times out....
  watchEndTicks = Xmillis() + watchDogMaxTime;
  thread3_CNN_active = false;
  thread4_CMD_active = true;
  thread4_WAIT_active = false;
  thread4_WAITOK_active = false;  
  thread4_listen_active = true;
  LED_delay = LED_DELAY_OK;
  return 0;
}
//
// Time to fly the Tello
//
long unsigned int thread4_CMD () {
  // thread4_listen(); is now its own thread...
  if (telloPanic) {  // something has gone wrong, land now...
    // this is tooo brute force....
#ifndef DEBUG_MODE
      strcpy(cmdBuffer,"land");
      // be persistent!
      for (int k=0; k < 5; k++) {
        Udp.beginPacket(TelloIP, TelloPort);
        Udp.write(cmdBuffer, strlen(cmdBuffer));
        Udp.endPacket();
        logEvent(cnt, "F", cmdBuffer);
        Xdelay(100);
      }
#endif
    // show done
    logEvent(cnt, "Z", "**PANIC-END**");
    cnt = 0;
    telloActive = false;
    telloPanic = false;
#ifdef DEBUG_MODE
#else
    Udp.stop();
    WiFi.end();
#endif
    thread4_CMD_active = false; 
    thread4_WAIT_active = false;
    thread4_WAITOK_active = false;    
    thread4_listen_active = false;
    thread5_cleanup_active = true;
    LED_delay = LED_DELAY_OK;
    return 0;
  }
  // end brute force ....
  String sCmd = flightPlan[cnt];
  if (sCmd.length() > 0) {  
    int waitTime = 1000;   // just a cautionary wait (gives each command a second) ...
    bool waitOK = false;
    if (sCmd.startsWith("wait ")) {
      // process wait time here
      replyValue = replyNone;  // don't care about replyValue so just reset to None,
                               // it is processed in waitOK        
      logEvent(cnt, "L", sCmd);
      float fnum = sCmd.substring(5).toFloat();
      waitTime = fnum * 1000; // secs to m.s.
    } else if (sCmd.startsWith("wtok ")) {
      // wait for "OK" if timeout (for now) don't care ((WRONG))
      logEvent(cnt, "L", sCmd);
      waitOK = true;                 
      float fnum = sCmd.substring(5).toFloat();
      waitTime = fnum * 1000; // secs to m.s.
    } else {
      replyValue = replyNone;  // new command!!
      // otherwise send command to Tello
      for (int i = 0; i < sCmd.length(); i++) {
        if (!isprint(sCmd[i])) break;
        cmdBuffer[i] = sCmd[i];
        cmdBuffer[i + 1] = '\0';
      }
#ifdef DEBUG_MODE
      // Serial.println("DEBUG_MODE, not sent!");
      logEvent(cnt, "X", cmdBuffer);
#else
      Udp.beginPacket(TelloIP, TelloPort);
      Udp.write(cmdBuffer, strlen(cmdBuffer));
      Udp.endPacket();
      logEvent(cnt, "F", cmdBuffer);
#endif
    }
    cnt++;
    // handle wait but keep other threads alive ...
    thread4_CMD_active = false;
    if (waitOK) { // complex wait for reply
       thread4_WAIT_active = false;
      thread4_WAITOK_active = true;     
    } else {      // just wait
      thread4_WAIT_active = true;
      thread4_WAITOK_active = false;
    }        
    // don't change thread4_listen_active
    waitEndTicks = Xmillis() + waitTime; 
  } else {
    // flightplan done
    logEvent(cnt, "Z", "**END**");
    cnt = 0;
    telloActive = false;
    telloPanic = false;
#ifdef DEBUG_MODE
#else
    Udp.stop();
    WiFi.end();
#endif
    thread4_CMD_active = false; 
    thread4_WAIT_active = false;
    thread4_WAITOK_active = false;    
    thread4_listen_active = false;
    thread5_cleanup_active = true;
    LED_delay = LED_DELAY_OK;
  }
  return 0;
}

long unsigned int thread4_wait() {
  // either loop here or return to thread4_CMD
  if ((Xmillis() > waitEndTicks) || telloPanic) {
    // we are free!
    thread4_CMD_active = true;
    thread4_WAIT_active = false;
    // leave thread4_listen_active alone      
    return 0; 
  } else {
    // we are not free; play it again sam -- but allow other threads to run!
    return 0;  
  }
}
long unsigned int thread4_waitOK() {
  // either loop here or return to thread4_CMD
  if ((replyValue != replyNone) || telloPanic) { // == replyOK) {
                                               // this is too simple...
    replyValue = replyNone; // clear reply
    // we are forced free!
    thread4_CMD_active = true;
    thread4_WAITOK_active = false;  
    return 0;         
  }
  if (Xmillis() > waitEndTicks) {
    // we are free!
    thread4_CMD_active = true;
    thread4_WAIT_active = false;
    // leave thread4_listen_active alone     
    return 0; 
  } else {
    // we are not free; play it again sam -- but allow other threads to run!
    return 0;  
  }
}
long unsigned int thread4_listen () {
  // if there's data available, read a packet
#ifdef DEBUG_MODE
  replyValue = replyNone;
#else
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
      for (int j = 0; j < len; j++) {
        if (!isprint(packetBuffer[j])) packetBuffer[j] = ' ';
        packetBuffer[j] = toupper(packetBuffer[j]); // makes life simplier
      }
      if (memcmp(packetBuffer, "OK", 2) == 0) {
        replyValue = replyOK;
      } else if (memcmp(packetBuffer, "ERR", 3) == 0) {
        replyValue = replyError;
      } else if (isdigit(packetBuffer[0])) {
        replyValue = atoi(packetBuffer);
      } else {
        replyValue = replyOther;
      }
      if (len < 16) {
        char lBuf[16];
        char tBuf[16];
        ltoa(replyValue, lBuf, 10);
        switch (replyValue) {
          case (replyOK): strcpy(tBuf,"OK"); break;
          case (replyError): strcpy(tBuf,"Error"); break;
          case (replyOther): strcpy(tBuf,"Other"); break;
          case (replyNone): strcpy(tBuf,"none"); break;
          default: strcpy(tBuf,"(num)");
        }
        logEvent(cnt, "T", String(packetBuffer) + " (" + String(packetSize) + ") decoded: " + String(lBuf) + " " + tBuf);
      }
    } else {
      logEvent(cnt, "T", String(" (0)"));
    }
  }
#endif
  return 0;
}

long unsigned int thread5_cleanup_restart() {
  // Serial.print("Log Dump: \n");
  logEvent(cnt, "Z", "**CLEANUP**");  
  // With WiFi connection ended, Log to different network using UDP (not telnet!)
#ifdef TRANS_LOG_MODE 
  logEvent(cnt, "Z", "Log Dump");  
  logTrans(
    TelloXlinkSSID,
    TelloXlinkPW,
    TelloXlinkIP,
    TelloXlinkPort); // transmit log via UDP
#endif
#ifdef AUTO_RUN_MODE
  // Serial.println("**FINAL STOP**");
  // threads off
  while (1) {
    digitalWrite(13, !digitalRead(13));
    Xdelay(LED_DELAY_DONE);
  }
#else
  // Serial.println("**PAUSE & RESET**");
  // restart
  startThreads(true);
#endif
  return 0;
}

//++++++++++
void loop() {
  // dispatacher ((this would be much better done with a dispatcher class....))
  // enforce WatchDog timer
  if (telloActive && (Xmillis() > watchEndTicks)) {
    // Tello flying and it's over time, so panic!
    telloPanic = true;
  }
  //
  if (Xmillis() >= thread0_LED_clock) {
    thread0_LED_clock = Xmillis() + thread0_LED();
  }
  // the return value is the delay....
  if (thread1a_telnet_active && Xmillis() >= thread_clock) {
    thread_clock = Xmillis() + thread1a_telnet();
  }
  if (thread1b_telnet_active && Xmillis() >= thread_clock) {
    thread_clock = Xmillis() + thread1b_telnet();
  }
  if (thread1c_telnet_active && Xmillis() >= thread_clock) {
    thread_clock = Xmillis() + thread1c_telnet();
  }
  if (thread2_INI_active && Xmillis() >= thread_clock) {
    thread_clock = Xmillis() + thread2_INI();
  }
  if (thread3_CNN_active && Xmillis() >= thread_clock) {
    thread_clock = Xmillis() + thread3_CNN();
  }
  if (thread4_WAIT_active) {  // this is a very intense thread, no delay
    thread4_wait();
  } 
  if (thread4_WAITOK_active) {  // this is a very intense thread, no delay
    thread4_waitOK();
  }     
  if (thread4_listen_active) {  // this is a very intense thread, no delay
    thread4_listen();
  }
  // this thread after prev three....
  if (thread4_CMD_active && Xmillis() >= thread_clock) {
    thread_clock = Xmillis() + thread4_CMD();
  }
  if (thread5_cleanup_active && Xmillis() >= thread_clock) {
    thread_clock = Xmillis() + thread5_cleanup_restart();
  }
}
//++++++++++
//
#if 0
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
#endif
/* end of code */
