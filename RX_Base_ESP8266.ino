#include "Arduino.h"
#include "RCSwitch.h"
#include "ESP8266WebServer.h"
#include "TimeLib.h"
#include "NTPClient.h"
#include "WiFiUdp.h"
#include "EEPROM.h"
#include "SoftwareSerial.h"
#include <user_interface.h>
#include "uptime.h"

RCSwitch mySwitch = RCSwitch();

ESP8266WebServer server(80);

// By default 'pool.ntp.org' is used with 60 seconds update interval and no offset
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP,"europe.pool.ntp.org", 3600, 10*60000); // pool, TZ-Offset in ms, update interval in ms

IPAddress ip( 192, 168, 1, 190 );	// make RX Base available at this IP
IPAddress dns( 192, 168, 1, 1 );	// your local network router
IPAddress gateway( 192, 168, 1, 1 );	// your local network router
IPAddress subnet( 255, 255, 255, 0 );


// GSM
#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
SoftwareSerial gsmSerial(D3, D4); // auf dem board D1 und D2 !!?
#define SerialAT gsmSerial
TinyGsm modem(SerialAT);
String adminGsmNumber = "+491787122394";

// nur RCSwitch kompiliert auf ESP8266.
// D5 -> 220Ohm -> LED -> GND
// D6 -> Receiver's pin DATA
// D7 -> Buzzer -> GND
// D8 leer
// 3V3 -> Receiver VCC

// 5V an Power In
// G an common GND
// D4 leer (ist LED_BUILTIN)
// D3 -> Button Switch -> GND (variable ist D8 aber geht zu auf dem board D3!!?)
// D2 -> R 10kOhm -> Serial (ist TX) -> Abzw. über 5.7kOhm an GND -> RX
// D1 -> R 10kOhm -> Serial (ist RX) -> TX

// SIM800L
// GND -> | Cap 470uF und 10uF (-)   -> GND
// VCC -> |                        | -> VCC 4,1V LM2596

// Kelko Keramik Capacitor kommt 10nF zwischen GND und VCC des Receivers!
int receiverPin = D6;
int buzzerPin = D7;
int ledPin = D5;
int buttonPin = D8; // auf dem board D3!!?

int armed = 0;
int alarm = 0;
int notified = 0;

double networkConnectionTime = 0;
unsigned int wifiConnectedWorkaround = 1;
unsigned int delayedInitProceduresDone = 0;

unsigned long lastTimeOutput = 0;
String logBuffer;
double node_timeout[32];
double node_volts_timeout[32];

// LED_BUILTIN is used as status LED
int statusLedStatus = 0;
unsigned long lastTimeLedUpdate = 0; // will store last time the Status LED was updated

const int nodes_count = 16;
float     node_volts[17]; // = {1.07,1.07,1.07};
double    node_volts_millis[17];
int       node_alarm[17] = {0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // zero based, but zero isnt used, 16+1 elements
int       node_alarm_histogram[17][24]; // replace with <CircularBuffer.h> eventually

volatile unsigned short eeAddr;
    typedef struct LogObj {
       unsigned long epoch;
       char message[12];
    };

// http://forum.arduino.cc/index.php?topic=396450.0
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;

int gsmModemPresent = 0;
int gsmConnected = 0;
String gsmOperator = "";

void setup() {
  wifi_on();

  Serial.begin (115200);
  Serial.println(""); Serial.println("");
  
  mySwitch.enableReceive(D6);

  EEPROM.begin(32000);
  eeAddr = log_eeAddr();

  BlinkAndBuzz(LED_BUILTIN, buzzerPin, 100);
  Blink(ledPin, 100);

  pinMode(buttonPin,INPUT_PULLUP); // button


  // HTTP Server:
  server.on("/", handleRoot);
  server.on("/disarm", []() {
    tripwire_disarm();
    // server.send(200, "text/plain", "Ausgeschaltet.");
    server.sendHeader("Location", String("/"), true);
    server.send ( 302, "text/plain", "");
    BlinkAndBuzz(ledPin, buzzerPin, 100);
  });
  server.on("/arm", []() {
    tripwire_arm();
    // server.send(200, "text/plain", "Scharf geschaltet.");
    server.sendHeader("Location", String("/"), true);
    server.send ( 302, "text/plain", "");
    BlinkAndBuzz(ledPin, buzzerPin, 100);
  });
  server.on("/alarm_reset", []() {
    tripwire_alarm_reset();
    // server.send(200, "text/plain", "Alarm ausgeschaltet.");
    server.sendHeader("Location", String("/"), true);
    server.send ( 302, "text/plain", "");
  });
  server.on("/wifi_off", []() {
    wifi_off();
    server.sendHeader("Location", String("/"), true);
    server.send ( 302, "text/plain", "");
  });
  server.on("/clearlog", []() {
    log_clearlog();
    server.sendHeader("Location", String("/"), true);
    server.send ( 302, "text/plain", "");
  });
  server.onNotFound(handleNotFound);
  if(wifiConnectedWorkaround){
    server.begin();
    Serial.println("HTTP server started");
  }else{
    Serial.println("No network. No local HTTP server.");
  }

  // NTP
  if(wifiConnectedWorkaround){
    timeClient.begin();
    timeClient.update();
  }

  // GSM
  SerialAT.begin(38400);
  delay(1000);
  gsm_on();

  log("Restarted!");
 
  BlinkAndBuzz(LED_BUILTIN, buzzerPin, 100);
  delay(100);
  BlinkAndBuzz(LED_BUILTIN, buzzerPin, 100);

  // Start with WiFi powered down
  // wifi_off();

  Serial.println ("setup done.");
}

int total_gsm_connect_retries = 0;
void gsm_on() {
  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  Serial.println("Initializing GSM modem SIM800L...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  if( modemInfo.indexOf("800") >= 0 ){
    Serial.print("GSM Modem: ");
    Serial.println(modemInfo);
    gsmModemPresent = 1;
  }else{
    Serial.println("GSM Modem not found.");
    return;
  }

  // more config
  modem.receiveCallerIdentification(1); // configure incomign RINGS to be accompanied by +CLIP (caller ID) info
  // modem.deleteAllSmsMessages(DeleteAllSmsMethod::All);
  // gsmSerial.flush(); // usually "SMS Ready", etc is left in the buffer now

  // GSM: network
  if(! modem.isNetworkConnected() ){
    int retries = 5;
    while( retries > 0 ){
        // GSM: Unlock your SIM card with a PIN
        if ( modem.getSimStatus() != 3 ){
            Serial.print("GSM: Unlocking SIM...");
            int retries = 5;
            while( retries > 0 ){
              Serial.print(".");
              modem.simUnlock("3041");
              delay(500);
              
              if ( modem.getSimStatus() != 3 ){
                 Serial.println(".OK");
                 break;
              }
              
              retries--;
            }
        }
      
        // after sim code, modem will try to connect to GSM network, usually that shoud succeed
        // but with low power this will reset the SIM800L
        Serial.print("GSM: Waiting for network");
        for (int i = 1; i <= 10; i++) {
           delay(1000);
           Serial.print(".");
        }
  
        if (modem.isNetworkConnected()) {
          Serial.print("Connected: ");
      
          String cop = modem.getOperator();
          Serial.println(cop);
          gsmConnected = 1;
          total_gsm_connect_retries = 0;
          break; // while
        }else{
          Serial.println("NOT CONNECTED.");
          gsmConnected = 0;
        }
        
        retries--;
        total_gsm_connect_retries++;
        Serial.print("Retry "); Serial.print(5- retries); Serial.print(": ");
    }
  }
}

void handleCommand(const String& inBuffer, int viaSms){
     if(alarm){ Serial.print("<<ALARM>> "); } // start every line with ALARM when alarm

     String SMS = "";
     if( inBuffer.equals("aus") ){
        tripwire_disarm();
        Serial.println("Tripwire ist AUS");
     }else if( inBuffer.equals("scharf") ){
        tripwire_arm();
        Serial.println("Tripwire ist SCHARF");
     }else if( inBuffer.equals("reset") ){
        tripwire_alarm_reset();
        Serial.println("Tripwire alarm reset");
     }else if( inBuffer.equals("wifi") ){
        wifi_on();
        Serial.println("WiFi ist AN");
     }else if( inBuffer.equals("wifiaus") ){
        wifi_off();
        Serial.println("WiFi ist AUS");
     }else if( inBuffer.equals("gsm") ){
        gsm_on();
        Serial.println("GSM ist AN");
     }else if( inBuffer.equals("log") ){
        LogObj logMessage;
        Serial.print("Log: (");
        Serial.print( (eeAddr - 140) / 16);
        Serial.print(" entries, addr:");
        Serial.print( eeAddr );
        Serial.println(")");
        int done = 0;
        for (int eeIndex = 140 ; eeIndex <= 31800 ; eeIndex += 16) {
          EEPROM.get(eeIndex, logMessage);
          if(logMessage.epoch == 0){ done++; continue; }
          if(logMessage.epoch == 0 && done >= 3){ break; }
          
          // Serial.print(logMessage.epoch);
          tmElements_t sevenIntegersInAStructure;
          breakTime(logMessage.epoch, sevenIntegersInAStructure);
          Serial.print(sevenIntegersInAStructure.Year + 1970); Serial.print("-");
          Serial.print(sevenIntegersInAStructure.Month);  Serial.print("-");
          Serial.print(sevenIntegersInAStructure.Day);    Serial.print(" ");
          // Serial.print(sevenIntegersInAStructure.Wday);
          Serial.print(sevenIntegersInAStructure.Hour);   Serial.print(":");
          Serial.print(sevenIntegersInAStructure.Minute); Serial.print(":");
          Serial.print(sevenIntegersInAStructure.Second);
          Serial.print(" ");
          Serial.println(logMessage.message);
        }
     }else if( inBuffer.equals("clearlog") ){
        log_clearlog();
     }else if( inBuffer.equals("status") ){
        Serial.print("Tripwire: ");
        SMS.concat("Tripwire: ");
        if(alarm){
           Serial.print("!!ALARM!!");
           SMS.concat("ALRM!");
        }else{
           if(armed){
              Serial.print("SCHARF");
              SMS.concat("SCHARF");
           }else{
              Serial.print("AUS");
              SMS.concat("AUS");
           }
        }
        Serial.print(" | ");
        SMS.concat(" / ");
        Serial.print( uptime() );
        SMS.concat( uptime() );
        if(wifiConnectedWorkaround){
           Serial.print(", WiFi: verbunden");
           SMS.concat(", WiFi verb.");
        }else{
           Serial.print(", WiFi: AUS");
           SMS.concat(", WiFi AUS");
        }

        if( gsmConnected ){
           int csq = modem.getSignalQuality();
           Serial.print(", GSM: verbunden ("); Serial.print(csq); Serial.println("db)");
           SMS.concat(", GSM verb.("); SMS.concat(csq); SMS.concat("db)");
        }else{
          Serial.println(", GSM: getrennt");
          SMS.concat(", GSM getr.");
        }

        SMS.concat(" / ");
        int active_nodes_cnt = 0;
        int tripped_cnt = 0;
        int clear_cnt = 0;
        for (int i=1; i < nodes_count; i++){
            if(node_volts[i] <= 1 && !node_alarm[i]){ continue; }

            active_nodes_cnt++;
            Serial.print("Node ");
            Serial.print(i);
            if(node_alarm[i]){
               Serial.print(": MOTN!! | ");
               tripped_cnt++;
            }else{
               Serial.print(": --OK-- | ");
               clear_cnt++;
            }
            Serial.print(node_volts[i]);
            Serial.print("V (vor ");
            Serial.print( (int) (millis() - node_volts_millis[i]) / (60 * 1000) );
            Serial.print(" Minuten) | ");
    
            for (int h=0; h <= 24; h++){
               if(node_alarm_histogram[i][h]){
                  Serial.print(node_alarm_histogram[i][h]);
               }else{
                  Serial.print("_");
               }
               Serial.print(".");
            }
   
            Serial.println("");
        }
        SMS.concat(active_nodes_cnt); SMS.concat(" nodes:");
        SMS.concat(tripped_cnt); SMS.concat(" tripped,");
        SMS.concat(clear_cnt); SMS.concat(" clear.");
        if(viaSms){
           modem.sendSMS(adminGsmNumber, SMS);
        }
     }else{
        if( inBuffer.startsWith("sim ") ){
           String simBuffer = inBuffer;
           simBuffer.remove(0,3); // can only remove() on local String
           Serial.print("> SIM800: "); Serial.println(simBuffer);
           gsmSerial.println(simBuffer);
        }else{
           Serial.print("Kommando nicht verstanden: "); Serial.println(inBuffer);
           Serial.println("Kommandos: aus | scharf | reset | status | wifi | wifiaus | log | clearlog | gsm");
        }
     }
}

double lastTimeCheckedSMS = 0;
void loop() {
  if( gsmSerial.available() > 1){
     String gsmBuffer = gsmSerial.readString();
     gsmBuffer.trim(); // remove whitespace
     Serial.println( gsmBuffer );

     // GSM Interface: Anrufe/Trigger
     if( gsmBuffer.startsWith("RING") ){
           //  +CLIP: "+4917812345678",145,"",0,"",0
           if( gsmBuffer.indexOf("+CLIP:") ){
              int from  = gsmBuffer.indexOf("+CLIP:");
              int until = gsmBuffer.indexOf(',');
              String number = gsmBuffer.substring(from + 8, until - 1);

              Serial.print("Incoming call from: ");
              Serial.println(number);

              if(number == adminGsmNumber){
                   Serial.println(" Hello Admin!");

                   delay(2000);

                   modem.callAnswer();

                   if(armed){
                    if(alarm){
                      // Play DTMF, duration 3x 100ms
                      modem.dtmfSend('*', 80); delay(80); modem.dtmfSend('*', 80); delay(100); modem.dtmfSend('*', 80);
                      delay(200);
                      modem.dtmfSend('*', 100); delay(80); modem.dtmfSend('*', 100); delay(80); modem.dtmfSend('*', 80);
                    }else{
                      // Play DTMF, duration 500ms
                      modem.dtmfSend('1', 500);
                      delay(500);
                      modem.dtmfSend('1', 500);
                    }
                   }else{
                      // Play DTMF, duration 1000ms
                      modem.dtmfSend('1', 1000);
                   }
              }
              delay(1000);
              modem.callHangup();
           }
     }else if( gsmBuffer.startsWith("+CNMI") ){
         Serial.println("Incoming SMS");
         lastTimeCheckedSMS = 0; // trigger processing of SMS
     }else if( gsmBuffer.startsWith("+CMTI") ){
         Serial.println("Incoming SMS");
         lastTimeCheckedSMS = 0; // trigger processing of SMS
     }
  }

  // GSM Interface: SMS
  if(lastTimeCheckedSMS == 0 || millis() - lastTimeCheckedSMS > 120000){
       // reset check interval
       lastTimeCheckedSMS = millis();

       if(! gsmModemPresent){ return; }
       
       Serial.println("GSM housekeeping...");

       // check missed SMS
       int hasSMS = 0;
       while(1){
          int index = modem.newMessageIndex(0); // 0 for reading latest, 1 (doesnt work!) for reading oldest unread message first
          if(index > 0){
            hasSMS = 1;
            Serial.print(" SMS found in slot: "); Serial.println( (int)index );
            String message = modem.readSMS(index);
            String number = modem.getSenderID(index);

            Serial.print(" SMS From: "); Serial.println(number);
            Serial.print(" SMS Message: "); Serial.println(message);

            if(number == adminGsmNumber){
                   Serial.println(" Hello Admin!");
                   message.trim();
                   message.toLowerCase();
                   handleCommand(message, 1); // second arg is "viaSms = true"
            }
          }else{
            break;
          }
       }

       if( hasSMS ){
           Serial.println(" SMS: deleting all slots. ");
            modem.deleteAllSmsMessages(DeleteAllSmsMethod::All);
       }

       // check GSM is connected
       if(! modem.isNetworkConnected() ){
          gsmConnected = 0;
          gsm_on();
       }
  }

  // Serielles Interface
  // irgend etwas im Sketch verursacht, dass sämtliche Serial Command libraries nicht gehen...
  if( Serial.available() > 1){
     String inBuffer = Serial.readString();
     inBuffer.trim(); // remove whitespace
     inBuffer.toLowerCase();
     handleCommand(inBuffer,0);
  }

  if (mySwitch.available()) {   
    unsigned int value = mySwitch.getReceivedValue();   
    if (value == 0) {
      Serial.println("Unknown encoding");
    } else {
      Serial.print("Received ");
      Serial.print( value );
      Serial.print(" / ");
      Serial.print( mySwitch.getReceivedBitlength() );
      Serial.print("bit ");
      Serial.print("Protocol: ");
      Serial.println( mySwitch.getReceivedProtocol() );

      // analyze incoming signal
      if(value > 0 && value < 32000){
        if(value % 1000){
          // has 100l added => is the second "OFF" alert
          
        }else{
          if(armed){
            alarm = 1; // !!Alarm!!
          }

          int nodeId = value / 1000;

          // deduplicate incoming ALARM pulses
          // nur einen alarm pro node akzeptieren, alle 5sek (2.4 sekunden ist der timer auf dem PIR, 3 sek der re-trigger timout des PIR)
          // noch knapper: überlegen, wie lange ein send() dauert, dann denn Schwellwert hier etwas darüber setzen
          // (es geht nur darum, die repeatTransmits der RCSwitch lib zu filtern)
          if(!node_timeout[nodeId] || millis() - node_timeout[nodeId] > 3000){
              node_timeout[nodeId] = millis();

              Serial.print(" Node Alert: ");
              Serial.print("NodeId: "); Serial.print( nodeId ); Serial.println(" > MOTION");

              // save alarm into local per-node alarm array
              node_alarm[nodeId] = 1;

              // increment histogram frame for this node and hour
              node_alarm_histogram[nodeId][ timeClient.getHours() ]++;

              // log event
              char msg[12]; sprintf(msg, "MOTN Node %d", nodeId);
              log(msg);

              // once we have an alarm, report all subsequent sensor trippings
              if(armed && alarm && notified){
                 Serial.println("Send SMS");
                 String SMS = "Alarm! More motion on node "; SMS.concat(nodeId);
                 modem.sendSMS(adminGsmNumber, SMS);
              }           
    
              Blink(ledPin, 200);
          }else{
           //  Serial.println("Resend pulse. Ignored.");
          }
        }
      } else if (value >= 32000 && value <= 65535) {
        value -= 32000;
        int nodeId = value / 1000;

        // deduplicate incoming VOLTAGE pulses
        if(!node_volts_timeout[nodeId] || millis() - node_volts_timeout[nodeId] > 3000){ // nur einen pulse pro node akzeptieren, alle 5sek (2.4 sekunden ist der timer auf dem PIR, 3 sek der re-trigger timout des PIR)
            node_volts_timeout[nodeId] = millis();  
           
            Serial.print(" Node Voltage: ");
            Serial.print("NodeId:"); Serial.print( nodeId ); Serial.print(", ");
            
            float volts = ( (float)value - (nodeId * 1000) ) / 100;
            Serial.print("Volts: "); Serial.println(volts, DEC);

            // save volts into local voltage array
            // as node send a voltage on power up and every x minutes, we also use it as a heartbeat
            node_volts[nodeId] = volts;
            node_volts_millis[nodeId] = millis();
        }else{
          //  Serial.println("Resend pulse. Ignored.");
        }
      }else{
         Serial.println(" Value out of Tripwire's encoding scheme range.");

         Blink(ledPin, 200);

         String msg = "RX:"; msg.concat(value); msg.concat(",");
         msg.concat(mySwitch.getReceivedBitlength()); msg.concat(",");
         msg.concat(mySwitch.getReceivedProtocol());
         log(msg);
      }
    }
    mySwitch.resetAvailable();
  }

  // sirene/alarm/buzzer on und off
  if(armed){
    if(alarm){
      if(!notified){
        // local alarm: buzzer
        analogWriteFreq(520);
        analogWrite(buzzerPin,1023); //  ESP8266, analogWrite is 0-1023 on these devices!

        log("!! ALARM !!");

        // GSM: send SMS
        String SMS;
        int active_nodes_cnt = 0;
        int tripped_cnt = 0;
        int clear_cnt = 0;
        for (int i=1; i < nodes_count; i++){
            if(node_volts[i] <= 1 && !node_alarm[i]){ continue; }

            active_nodes_cnt++;
            if(node_alarm[i]){
               SMS.concat("Alarm Node "); SMS.concat(i); SMS.concat(", short status: ");
               tripped_cnt++;
            }else{
               clear_cnt++;
            }
        }
        SMS.concat(active_nodes_cnt); SMS.concat(" nodes:");
        SMS.concat(tripped_cnt); SMS.concat(" tripped,");
        SMS.concat(clear_cnt); SMS.concat(" clear.");
        modem.sendSMS(adminGsmNumber, SMS);

        // GSM: call
        int res = modem.callNumber(adminGsmNumber); // blocks for max 60 secs
        if( res ){
           modem.callHangup();
        }

        notified = 1;
      }
    }else{
      analogWrite(buzzerPin,0);
    }
  }


  // NTP
  if(wifiConnectedWorkaround){
      timeClient.update();
  }
  // time client doesn need to have network to output something  
  if(lastTimeOutput == 0 || millis() - lastTimeOutput > 60000){
     if(alarm){ Serial.print("<<ALARM>> "); }
     Serial.print("--- "); Serial.println(timeClient.getFormattedTime());
     lastTimeOutput = millis();

   // Serial.print("BUFFER:");
   // Serial.println(logBuffer);
  }
  // turn-over/clean histogram slate/hour-frame
  if(timeClient.getMinutes() == 0 && timeClient.getSeconds() == 0){
        int h = timeClient.getHours();
        for (int i=1; i < nodes_count; i++){
           if(node_alarm_histogram[i][h]){
                Serial.print(timeClient.getFormattedTime()); Serial.print(": Clear histogram of hour-frame "); Serial.print(h); Serial.print(" on Node "); Serial.println(i);
                node_alarm_histogram[i][h] = 0;        
           }
        }
  }


  // HTTP Server
  if(wifiConnectedWorkaround){
     server.handleClient();
  }

  // Status LED
  if(armed){
      if( statusLedStatus == 0 && millis() - lastTimeLedUpdate >= 2900 ){
          lastTimeLedUpdate = millis();
          LedOn(LED_BUILTIN);
          statusLedStatus = 1;
      }else if( statusLedStatus == 1 && millis() - lastTimeLedUpdate >= 100 ){
          lastTimeLedUpdate = millis();
          LedOff(LED_BUILTIN);
          statusLedStatus = 0;
      }
  }else{
      if( statusLedStatus == 1 ){
        digitalWrite(LED_BUILTIN, HIGH); // esp8266 oddness, HIGH is OFF,
        statusLedStatus = 0;
      }
  }

  if( digitalRead(buttonPin) == LOW ){
    Serial.print("Button pressed:");
    BlinkAndBuzz(LED_BUILTIN, buzzerPin, 100);

    delay(500); // simple debounce

    if(wifiConnectedWorkaround == 1){
       BlinkAndBuzz(LED_BUILTIN, buzzerPin, 100);
       delay(150);
       BlinkAndBuzz(LED_BUILTIN, buzzerPin, 100);

       Serial.println(" WiFi off");
       wifi_off();
    }else{
      Serial.println(" WiFi on");
      BlinkAndBuzz(LED_BUILTIN, buzzerPin, 100);
      wifi_on();
    }
  }

  if( delayedInitProceduresDone != 1 && millis() > 60000 ){
    Serial.println("Procedures after 1 Minute uptime...");
    delayedInitProceduresDone = 1;

    // power down wifi after a short startup period
    if(wifiConnectedWorkaround == 1){
       wifi_off();
    }
  }
}


void tripwire_disarm() {
    armed = 0;

    if(alarm){
      tripwire_alarm_reset();
    }

    // log event
    log("Disarmed");
}

void tripwire_arm() {
    // going from disarmed to armed clears all alarm flags collected during freewheeling disarmed state
    if(! armed){
      tripwire_alarm_reset();
    }

    armed = 1;

    // log event
    log("Armed");
}

void tripwire_alarm_reset() {
    // reset general alarm flag(s)
    alarm = 0;
    notified = 0;

    // reset per-node alarm array
    for (int i=1; i < nodes_count; i++){
       node_alarm[i] = 0;
    }

    // log event
    log("Alarm OFF");
}

void log(char buf[]) {
    LogObj logMessage = { timeClient.getEpochTime(), buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11] };
  
    EEPROM.put(eeAddr, logMessage); //log
    EEPROM.commit();
    eeAddr += sizeof(logMessage);

    if(eeAddr > 31800){
      log_clearlog();
    }
  
    // Serial.print("log:EEPROM Addr index at ");
    // Serial.println(eeAddr);
}
void log( const String& text ){
    LogObj logMessage = { timeClient.getEpochTime(), text.charAt(0), text.charAt(1), text.charAt(2), text.charAt(3), text.charAt(4), text.charAt(5), text.charAt(6), text.charAt(7), text.charAt(8), text.charAt(9), text.charAt(10), text.charAt(11) };
  
    EEPROM.put(eeAddr, logMessage); //log
    EEPROM.commit();
    eeAddr += sizeof(logMessage);

    if(eeAddr > 31800){
      log_clearlog();
    }
}


unsigned short log_eeAddr(){
    LogObj logMessage;
    unsigned short lastAddr = 140;
    unsigned int done = 0;
    for (int eeIndex = 140 ; eeIndex < 31800 ; eeIndex += 16) { // peek into data to find last entry
        EEPROM.get(eeIndex, logMessage);
        // Serial.print(eeIndex); Serial.print(" - "); Serial.print(logMessage.epoch); Serial.print(" - "); Serial.println(logMessage.message);
        if(logMessage.epoch == 0){ break; } // break on first empty log buffer line
        lastAddr = eeIndex;
    }
    lastAddr += 16;

    Serial.print("log_eeAddr is "); Serial.println(lastAddr);
    return lastAddr;
}

void log_clearlog() {
    for (int i = 140 ; i < 32000 ; i++) {
      EEPROM.write(i, 0); // format with all-zeros
    }
    EEPROM.commit();
    eeAddr = 140; // reset
    log("log cleared");
    Serial.println("Log cleared");
}

void wifi_on() {
  // WiFi.softAPdisconnect(true); // same as mode(WIFI_STA) ??
  // https://www.bakke.online/index.php/2017/05/22/reducing-wifi-power-consumption-on-esp8266-part-3/
  Serial.print("Bringing up WiFi."); 

  /*
  WiFi.forceSleepWake();
  delay( 1 );
  WiFi.persistent( false ); // Disable the WiFi persistence.  The ESP8266 will not load and save WiFi settings in the flash memory.
  WiFi.mode( WIFI_STA );
  WiFi.config(ip, dns, gateway, subnet);
  WiFi.begin(WIFI_SSID, WIFI_PASS); // from Adafruit's config.h
  */

  wifi_fpm_do_wakeup();
  wifi_fpm_close();
  wifi_set_opmode(STATION_MODE);
  WiFi.config(ip, dns, gateway, subnet);
  WiFi.begin(WIFI_SSID, WIFI_PASS); // from Adafruit's config.h
  wifi_station_connect();


  // we separate Wifi connection
  while(WiFi.localIP().toString() == "0.0.0.0"){
    if(networkConnectionTime >= 500 * 10){
       Serial.print("Error connecting to WiFi. Continuing without network. Status:");
       Serial.println(WiFi.status());
       wifiConnectedWorkaround = 0;
       break;
    }else{
       Serial.print(".");      
    }
    delay(500);

    networkConnectionTime += 500;
  }
  Serial.println();   
  if(wifiConnectedWorkaround){
    Serial.print("WiFi connected: IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println(WiFi.gatewayIP().toString());
    Serial.println(WiFi.subnetMask());
  }

  wifiConnectedWorkaround = 1;

  // log event
  log("WiFi on");
}

void wifi_off() {
  // https://www.bakke.online/index.php/2017/05/21/reducing-wifi-power-consumption-on-esp8266-part-2/
  Serial.println("Switching WiFi off.");
//  WiFi.mode(WIFI_OFF); WiFi.forceSleepBegin(); delay(1);


    // prevent crash: https://github.com/esp8266/Arduino/issues/4082
    wifi_station_disconnect();
    bool stopped;
    do  {
      stopped = wifi_station_get_connect_status() == DHCP_STOPPED;
      if (!stopped){
        delay(100);
      }
    } while (!stopped);
    wifi_set_opmode(NULL_MODE);
    wifi_set_sleep_type(MODEM_SLEEP_T);
    wifi_fpm_open();
    wifi_fpm_do_sleep(0xFFFFFFF);

  
    wifiConnectedWorkaround = 0;
    // log event
    log("WiFi off");
}

// esp8266 oddness, LOW is ON, https://forum.arduino.cc/index.php?topic=474723.0 , https://github.com/nodemcu/nodemcu-devkit-v1.0/issues/16#issuecomment-244625860
void Blink(byte PIN, byte DELAY_MS) {
  pinMode(PIN, OUTPUT); // re-power pin

  if(PIN == LED_BUILTIN){ digitalWrite(PIN,LOW); }else{ digitalWrite(PIN,HIGH); } // esp8266 oddness
  delay(DELAY_MS);
  if(PIN == LED_BUILTIN){ digitalWrite(PIN,HIGH); }else{ digitalWrite(PIN,LOW); } // esp8266 oddness
  delay(DELAY_MS);

  pinMode(PIN, INPUT); // save power again
}


void BlinkAndBuzz(byte ledpin, byte buzzerpin, byte delay_ms) {
    pinMode(ledpin, OUTPUT);
    pinMode(buzzerpin, OUTPUT);

    analogWriteFreq(520);
    analogWrite(buzzerpin,512);
    
    if(ledpin == LED_BUILTIN){ digitalWrite(ledpin,LOW); }else{ digitalWrite(ledpin,HIGH); } // esp8266 oddness
    delay(delay_ms);
    if(ledpin == LED_BUILTIN){ digitalWrite(ledpin,HIGH); }else{ digitalWrite(ledpin,LOW); } // esp8266 oddness

    analogWrite(ledpin,0);
    
    delay(delay_ms);

    pinMode(ledpin, INPUT);
    pinMode(buzzerpin, INPUT);
}

void LedOn(byte ledpin){
  pinMode(ledpin, OUTPUT); // re-power pin
  if(ledpin == LED_BUILTIN){ digitalWrite(ledpin,LOW); }else{ digitalWrite(ledpin,HIGH); } // esp8266 oddness
}
void LedOff(byte ledpin){
  if(ledpin == LED_BUILTIN){ digitalWrite(ledpin,HIGH); }else{ digitalWrite(ledpin,LOW); } // esp8266 oddness
  pinMode(ledpin, INPUT);
}

// buzz a piezo speaker on an ESP8266 pin, analogWrite is 0-1023 on these devices!
void espbuzz(uint8_t buzzerpin, unsigned int frequency, unsigned long duration, unsigned int volume) {
  pinMode (buzzerpin, OUTPUT);
  
  analogWriteFreq(frequency);
  analogWrite(buzzerpin,volume);
  delay(duration);
  analogWrite(buzzerpin,0);

  pinMode (buzzerpin, INPUT);
}

// HTTP Server
double lastTimeCheckedCSQ = 0;
int gsmCSQ = 0;
void handleRoot() {
    String html = "<html><head><meta http-equiv='refresh' content='8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>Tripwire Status</title></head><body style='font-family: Arial;'>\n\n";
    html += "<style type='text/css'>\n";
    html += "table { border-collapse: collapse; }\n";
    html += "th,td { border: 1px solid #ccc; padding: 5px 8px; text-align: center; }\n";
    html += "th small { color: #555; }\n";
    html += "td.left { text-align: left; }\n";
    html += "td span.histogram { color: #aaa; padding-right: 1px; }\n";
    html += "</style>\n";
    html += "<h1>Tripwire</h1>\n\n";
    // html += "URI: "+ server.uri() +"<br>\n";
    // html += "Method: "+ (server.method() == HTTP_GET) ? "GET" : "POST";
    // html += "<br>Arguments: "+ server.args();
    html += "<br>Sicherheits-Status: ";
    if(alarm){
      html += "<span style='padding:5px 8px; border:2px solid #e00; background: #faa;'>!!ALARM!!</span>";
    }else if(armed){
      html += "<span style='padding:5px 8px; border:2px solid #0af; background:#bdf;'>SCHARF</span>";
    }else{
      html += "<span style='padding:5px 8px; border:2px solid #bbb; background:#ddd;'>AUS</span>";
    }
    html += "<br><br><a href='/disarm'>Ausschalten</a> | <a href='/arm'>Scharf schalten</a> | <a href='/alarm_reset'>Reset alarm</a> \n";
    html += "<br><br><div style='font-size:0.8em;margin: 15px 0;'>System: ";
    html.concat( uptime() );
    html += ". WiFi verbunden. ";

    if( gsmConnected ){
       if(lastTimeCheckedCSQ == 0 || millis() - lastTimeCheckedCSQ > 25000){ // web interface updates every 5s, limit update-rate here a bit
           // reset check interval
           lastTimeCheckedCSQ = millis();
           gsmCSQ = modem.getSignalQuality();
       }
       html.concat(" GSM verbunden ("); html.concat(gsmCSQ); html.concat("db).");
    }else{
      html += ", GSM getrennt.";
    }
    html += " | <a href='/wifi_off'>WiFi ausschalten</a> </div>\n";

    html += "<table style='border:1px solid #ccc;'>\n";
    html += "<thead><tr><th>Node</th><th>Status</th><th style='align:left;'>Battery</th><th><small>24h Alarm Histogram</small></th></tr></thead>\n";
    for (int i=1; i < nodes_count; i++){
        if(node_volts[i] <= 1 && !node_alarm[i]){ continue; }
        
        html += "<tr>\n<td>Node ";
        html.concat(i);
        if(node_alarm[i]){
           html.concat("</td><td style='background:#f00;'>MOTN</td>\n");
        }else{
           html.concat("</td><td style='background:#0f0;'>0</td>\n");
        }
        html.concat("<td class='left'>");
        html.concat(node_volts[i]);
        html.concat("V <small>vor \n");
        html.concat( (int) (millis() - node_volts_millis[i]) / (60 * 1000) );
        html.concat(" Minuten</small></td>\n");

        html.concat("<td>");
        for (int h=0; h <= 24; h++){
           if(node_alarm_histogram[i][h]){
              html.concat(node_alarm_histogram[i][h]);
           }else{
              html.concat("<span class='histogram'>_</span>");
           }
           html.concat("<span class='histogram'>.</span>");
        }
        html.concat("</td>");

        html.concat("\n</tr>\n");
        
      //  char volts[5];
      //  dtostrf(node_volts[i], 2, 2,volts);
    }
    html += "</table>\n<div style='margin-top:15px;color:#aaa;font-size:0.8em;'>\n";

    html.concat("Log: (");
    html.concat( (eeAddr - 140 - 16) / 16);
    html.concat(" entries) - <a href='/clearlog'>clear log</a><br>\n");
    LogObj logMessage;
    for (int eeIndex = eeAddr - 16 ; eeIndex >= (eeAddr - 16*25) && eeIndex >= 140  ; eeIndex -= 16) {
      EEPROM.get(eeIndex, logMessage);

      // Serial.print(logMessage.epoch);
      tmElements_t sevenIntegersInAStructure;
      breakTime(logMessage.epoch, sevenIntegersInAStructure);
      html.concat(sevenIntegersInAStructure.Year + 1970); html.concat("-");
      html.concat(sevenIntegersInAStructure.Month);  html.concat("-");
      html.concat(sevenIntegersInAStructure.Day);    html.concat(" - ");
      // html.concat(sevenIntegersInAStructure.Wday);
      html.concat(sevenIntegersInAStructure.Hour);   html.concat(":");
      html.concat(sevenIntegersInAStructure.Minute); html.concat(":");
      html.concat(sevenIntegersInAStructure.Second);
      html.concat(" ");
      html.concat(logMessage.message);
      html.concat("<br>\n");
    }
    html.concat("...</div>");
      
    server.send(200, "text/html", html);
}
void handleNotFound() {
    String html = "File Not Found\n\n";
    html += "URI: ";
    html += server.uri();
    html += "\nMethod: ";
    html += (server.method() == HTTP_GET) ? "GET" : "POST";
    html += "\nArguments: ";
    html += server.args();
    html += "\n";
    for (uint8_t i = 0; i < server.args(); i++) {
      html += " " + server.argName(i) + ": " + server.arg(i) + "\n";
    }
    server.send(404, "text/plain", html);
}
