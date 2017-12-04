/*
  Arduino control for Remote Lab
  Test 1
  
  - Testing comms between PC-based java program and Arduino - using simple serial comms
  - Version 1a : Basic LED blinking
*/

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

// -----------------------------------------
// Initialise the system

void setup(){
  Serial.begin(115200);
  digitalWrite(13,LOW);
}


// -----------------------------------------
// Main Loop

void loop(){

  // when a newline arrives:
  if (stringComplete) {

    String processingString = inputString;
    inputString = "";
    stringComplete = false;

    String response = "NOK";
    if(processingString.startsWith("rlab://")) { // OK is a message I know (this is general code you can reuse)
      
      // Process the incoming message and send response
      if(processingString.substring(7,11) == "13on")      response = "OK-13on";
      else if(processingString.substring(7,11) == "13of") response = "OK-13of";
      else if(processingString.substring(7,11) == "13fl") response = "OK-13fl";
    }
    else response = "OK-unknown";

    // Return response
    Serial.println(response);
    Serial.flush();

    // Process the incoming message and send response
    if(processingString.startsWith("rlab://")) { // OK is a message I know (this is general code you can reuse)
      if(processingString.substring(7,11) == "13on") {
          digitalWrite(13,HIGH);
      } else if(processingString.substring(7,11) == "13of") {
          digitalWrite(13,LOW);
      } else if(processingString.substring(7,11) == "13fl") {
          int dl = processingString.substring(13,18).toInt();
          digitalWrite(13,HIGH);
          delay(dl);
          digitalWrite(13,LOW);
      }
    }
  }
}


// -----------------------------------------
// Serial read - triggered by incoming data

void serialEvent() {
    
  while (Serial.available() && !stringComplete) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

