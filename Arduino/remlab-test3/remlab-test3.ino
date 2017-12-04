/*
  Arduino control for Remote Lab
  Test 3
  
  - Testing comms between PC-based java program and Arduino - using simple serial comms
  - Version 1a : Basic LED blinking
  - Version 1c : Includes a set of temperature sensors
*/

#include <OneWire.h>
#include <DallasTemperature.h>


String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// Thermometers
#define NUM_SENS 11
                                      // Addresses for each sensor
DeviceAddress ThermAddrs[NUM_SENS] = {
  { 0x28, 0x74, 0x0F, 0x3E, 0x06, 0x00, 0x00, 0xB9 },
  { 0x28, 0xFF, 0xC4, 0x9C, 0x54, 0x14, 0x00, 0xB4 },
  { 0x28, 0xFF, 0x0E, 0x8B, 0x54, 0x14, 0x00, 0x95 },
  { 0x28, 0xFF, 0x4E, 0xFE, 0x54, 0x14, 0x00, 0x55 },
  { 0x28, 0xFF, 0x33, 0xE9, 0x54, 0x14, 0x00, 0x6A },
  { 0x28, 0xFF, 0x6B, 0xE4, 0x54, 0x14, 0x00, 0x5C },
  { 0x28, 0xFF, 0xFB, 0x9B, 0x54, 0x14, 0x00, 0xD9 },
  { 0x28, 0xFF, 0x27, 0x4E, 0x73, 0x04, 0x00, 0x24 },
  { 0x28, 0xFF, 0x2F, 0xEA, 0x54, 0x14, 0x00, 0xBF },
  { 0x28, 0xFF, 0xAF, 0x8A, 0x54, 0x14, 0x00, 0xE4 },
  { 0x28, 0xFF, 0xFF, 0xE6, 0x54, 0x14, 0x00, 0xF3 }};
int   nextSens;                       // next Sensor to read
float mostRecentTemp[NUM_SENS];       // most recent temperature readings
char charBuf[20];


// -----------------------------------------
// Get TMP Function
float getTemperature(DeviceAddress deviceAddress) {
      sensors.requestTemperatures();
      float tempC = sensors.getTempC(deviceAddress);
      return tempC;
}   


// -----------------------------------------
// Initialise the system

void setup(){
  Serial.begin(115200);
  digitalWrite(13,LOW);

  // Start up the temperature sensor library and configure the resolution to 10 bit
  sensors.begin();
  for (int i=0; i<NUM_SENS; i++) {
    mostRecentTemp[i] = -127.0;
    sensors.setResolution(ThermAddrs[i], 10);
  }
  nextSens=0;
}


// -----------------------------------------
// Main Loop

void loop(){

  String response;

  // read in the next temperature
  mostRecentTemp[nextSens] = getTemperature(ThermAddrs[nextSens]);
  nextSens++;
  if (nextSens >= NUM_SENS) nextSens=0;


  // has a message arrived?
  if (stringComplete) {

    String processingString = inputString;
    inputString = "";
    stringComplete = false;

    // Check and process the incoming message
      
    // ------- Message=<rlab://13on>; Response=<OK-13on>; Turn pin 13 LED on
    if(processingString.startsWith("rlab://13on")) {
      Serial.println("OK-13on");
      Serial.flush();
      digitalWrite(13,HIGH);
    }

    // ------- Message=<rlab://13of>; Response=<OK-13of>; Turn pin 13 LED off
    else if(processingString.startsWith("rlab://13of")) {
      Serial.println("OK-13of");
      Serial.flush();
      digitalWrite(13,LOW);
    }

    // ------- Message=<rlab://13fl?xx>; Response=<OK-13fl>; Flash the pin 13 LED for xx mSec
    else if(processingString.startsWith("rlab://13fl")) {
      Serial.println("OK-13fl");
      Serial.flush();
      int dl = processingString.substring(12).toInt();
      digitalWrite(13,HIGH);
      delay(dl);
      digitalWrite(13,LOW);
    }

    // ------- Message=<rlab://tmpc?xx>; Response=<OK-tmpc?yy>; Return temp of sensor xx (as float in degC)
    else if(processingString.startsWith("rlab://tmpc")) {
      response = "OK-tmpc?";
      int sens = processingString.substring(12,14).toInt();
      if ((sens>=NUM_SENS)||(sens<0)) {
        response += "invalidSensor";
      }
      else {
        response += "s";
        response += sens;
        response += "=";
        dtostrf(mostRecentTemp[sens], 1, 2, charBuf);
        response += charBuf;
      }
      Serial.println(response);
      Serial.flush();
    }

    // ------- Message=<rlab://tall>; Response=<OK-tall?yy>; Return temps of all sensors (as float in degC)
    else if(processingString.startsWith("rlab://tall")) {
      response = "OK-tall?";
      for (int sens=0; sens<NUM_SENS; sens++) {
        if (sens>0) response += "&";
        response += "s";
        response += sens;
        response += "=";
        dtostrf(mostRecentTemp[sens], 1, 2, charBuf);
        response += charBuf;
      }
      Serial.println(response);
      Serial.flush();
    }

    // ------- Message=<rlab://xxx>; Response=<NOK-Unknown>; Valid type, but unknown message
    else if(processingString.startsWith("rlab://")) {
      Serial.println("NOK-unknown");
      Serial.flush();
    }

    // ------- Message=unknown
    else {
      Serial.println("NOK-invalid");
      Serial.flush();
    }
  }
}


// -----------------------------------------
// Serial read - triggered by incoming data

void serialEvent() {
    
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

