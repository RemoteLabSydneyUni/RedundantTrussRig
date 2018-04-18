// ************************************************************************************
//   Redundant Truss Hardware Controller - Distance Sensor Data Acquisition
// ----------------------------------------------------------------------------------
//   Version        :   0.0
//   Origin Date    :   09 Mar 2018
//   Created by     :   YR DENG
// ----------------------------------------------------------------------------------
//   Program Description
//  - Due to the compatibility issue of I2C communication & Arduino Mega, this is the 
//    DAQ program for Distance Sensor VL53L0X DAQ based on Arduino UNO and providing data to 
//    the controller (Mega) via Serial communication 
// ----------------------------------------------------------------------------------
//   Version Notes
//  0.0     Initial Build. 
//          - I2C comm to sensor
//          - Serial comm to master controller (this is the slave)
// ----------------------------------------------------------------------------------
//
// ************************************************************************************

// --------------- Library --------------------------------------
#include <Wire.h>
#include <VL53L0X.h>

// --------------- Variable Declaration -------------------------

VL53L0X sensor;
int measured = 0;
boolean tOutFlag = true;
boolean led = false;
unsigned long prevTimeLED = 0L;
unsigned long prevTimeRead = 0L;
char inputString[64] = "";           // Serial Comm - a string to hold incoming data
int inputPointer;                    // Serial Comm - input store pointer
boolean stringComplete = false;      // Serial Comm - whether the string is complete

// --------------- Setup ----------------------------------------
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  digitalWrite(13,LOW);
  
  sensor.init();
  sensor.setTimeout(100);
  tOutFlag = true;
  
  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();
  prevTimeLED = millis();
  prevTimeRead = millis();
}

// --------------- Main Operation -------------------------------
void loop()
{
  if (millis()< prevTimeRead){
      prevTimeRead = millis();
  }else if ((millis()-prevTimeRead)>50){
    measured = sensor.readRangeContinuousMillimeters();
    tOutFlag = sensor.timeoutOccurred();
    if(!tOutFlag){
      if (millis()< prevTimeLED){
        prevTimeLED = millis();
      }else if ((millis()-prevTimeLED)>250){
        digitalWrite(13,led);
        led = !led;
        prevTimeLED = millis();
  //      Serial.print(measured);
  //      Serial.print(" at ");
  //      Serial.println(millis()); // for debugging
      }
    }else{
      digitalWrite(13,HIGH);
    }
    prevTimeRead = millis();
  }
  delay(1);
}

// --------------- Serial Event Handling ------------------------
void serialEvent() {
    
  while (Serial.available() && !stringComplete) {  
    // get the new byte:
    char inChar = (char)Serial.read();
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if ((inChar == '\n')||(inputPointer == 63)) {
      stringComplete = true;
      inputPointer = 0;
    }else if (inputPointer < 63){
    // allocate to the inputString buffer
    // move pointer to next location
      inputString[inputPointer] = inChar;
      inputPointer++;
    }
  }
  
  // *************************
  // Interpret input
  if (stringComplete) {
    char request[5] = "";
    for (int i = 0; i<4;i++){
      request[i] = inputString[i];
    }
    if (strcmp(request,"REQV") == 0){
      Serial.print("Cpl-");
      Serial.print(measured);
      Serial.print(";");
      Serial.print(tOutFlag);
      Serial.print(";");
      Serial.println("0");
      Serial.flush();
    }else{
      Serial.println("Err-request");
      Serial.flush();
    }
    for (int i=0;i<64;i++){
      inputString[i] = '\0';
    }
    stringComplete = false;
  }
}
