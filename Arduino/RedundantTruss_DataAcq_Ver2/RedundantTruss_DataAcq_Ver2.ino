// ************************************************************************************
//   Redundant Truss Hardware Controller - Data Acquisition
// ----------------------------------------------------------------------------------
//   Version    :   2.0
//   Date       :   20 Sep 2017
//   Modified by:   YR DENG
// ----------------------------------------------------------------------------------
//   Program Description
//  - To handle the data acquisition from the strain gauges and send to rig for indication
// ----------------------------------------------------------------------------------
//   Version Notes
//  0.0     Initial Build.
//          - Basic infrastructure for Arduino-Sahara Interface
//          - Interface for HX711 strain gauge amplifier
//          - Basic operation modes and handshaking
//  0.1     Add watchdog check
//  1.0     Upgrade
//          - Remove Calibration function from Arduino
//          - Add initialization stages
//  2.0     Add Deformation gauge reading
// ----------------------------------------------------------------------------------
//
// ************************************************************************************

// --------------- Library --------------------------------------
#include <HX711.h>

// --------------- Constant Declaration -------------------------
const int SLEEP_TIME_DEFAULT = 100;    //(ms)
const int PIN_REQ = 10;
const int PIN_DAT = 8;
const int PIN_CLK = 9;

// --------------- Variable Declaration -------------------------
// - Strain Gauge Informations
// - Readings, calibration factors (Scale & offest)
float strain[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float scale[10]  = {3570.3, 3733.8, 3574.6, 3402.7, 3144.2, 3539.0, 3320.6, 3378.4, 1.0, 1.0};
long offset[10] = { -9635L, -230437L, -91675L, -202624L, -68536L, -75539L, 6875L, 50370L, 0L, 0L};

// - Data Transfer Flags
boolean scaleChk[10] = {false, false, false, false, false, false, false, false, false, false};
boolean offsetChk[10] = {false, false, false, false, false, false, false, false, false, false};

// - Gauge Health
boolean faultChk[10] = {false, false, false, false, false, false, false, false, false, false};
int retryCount = 0;

// - Operating Mode
int mode;

// - Current Gauge Tracker
int currentGauge;

// - Serial Communication
char inputString[64] = "";           // Serial Comm - a string to hold incoming data
int inputPointer;                    // Serial Comm - input store pointer
boolean stringComplete = false;      // Serial Comm - whether the string is complete

// - Time Management
unsigned long prevTime;
unsigned long initTime;

// - Deformation Gauge
char defoResultChr[7];
float defoResult = 99.99;
int defoDecimal = 9;
byte defoResultTransfer[14];
boolean defoFault = false;

// --------------- Gauge Declaration ----------------------------
HX711 gauge[10] = {HX711(32, 42), HX711(33, 43), HX711(34, 44), HX711(35, 45), HX711(36, 46), HX711(37, 47), HX711(38, 48), HX711(39, 49), HX711(30, 40), HX711(31, 41)};
// Note: Last 2 gauge are not installed. Placeholder for future expansion. Avoid Pin 30 & 40 in future installation. Suspect faulty.

// --------------- Function: Pre-calibration Check --------------
int preCalCheck() {
  for (int i = 0; i < 8; i++) {
    if (scaleChk[i] == false) {
      return 11;
    }
  }
  for (int i = 0; i < 8; i++) {
    if (offsetChk[i] == false) {
      return 11;
    }
  }
  // All green, ready to calibrate
  return 12;
}

// --------------- Function: Calibraiton ------------------------
void calibration() {
  for (int i = 0; i < 8; i++) {
    gauge[i].set_scale(scale[i]);
    scaleChk[i] = false;
    gauge[i].set_offset(offset[i]);
    offsetChk[i] = false;
  }
}

// --------------- Function: Reading Retrieving -----------------
boolean gaugeRead(int index) {
  if (gauge[index].is_ready()) {
    strain[index] = gauge[index].get_units(1);
    return true;
  } else {
    return false;
  }
}

// --------------- Function: Deformation Retrieving -------------
boolean defoRead() {
  int i = 0;
  int j = 0;
  int k = 0;
  float sign = 0.0;
  unsigned long internalTimeCheck;
  digitalWrite(PIN_REQ, HIGH);
  internalTimeCheck = millis();
  for (i = 0; i < 13; i++) {
    k = 0;
    for (j = 0; j < 4; j++) {
      while (digitalRead(PIN_CLK) == LOW) {         // waiting for clock signal, return if overtime
        if ((millis() - internalTimeCheck) > 2000) {
          return true;
        }
      }
      internalTimeCheck = millis();
      while (digitalRead(PIN_CLK) == HIGH) {
        if ((millis() - internalTimeCheck) > 2000) {           // waiting for clock signal, return if overtime
          return true;
        }
      }
      digitalWrite(PIN_REQ, LOW);
      bitWrite(k, j, (digitalRead(PIN_DAT) & 0x1)); //Reading raw data bits, and reverse order
    }
    // extract data
    defoResultTransfer[i] = k;
  }
  // sign = defoResultTransfer[4]; decimal = ~[11]; units = ~[12]
  // assemble measurement
  for (int lp = 0; lp < 6; lp++) {
    defoResultChr[lp] = defoResultTransfer[lp + 5] + '0';
    defoResultChr[6] = 0;
  }
  defoDecimal = (int)defoResultTransfer[11];
  if((int)defoResultTransfer[4]==8){
    sign = -1.0;
  }else{
    sign = 1.0;
  }
  defoResult = atol(defoResultChr) / pow(10, defoDecimal)*sign;
  digitalWrite(PIN_REQ, LOW);
  return false;
}


// --------------- Function: Loop Timecheck ---------------------
boolean timeCheck() {
  unsigned long timeDiff;
  // check time elapsed from last check against default delay time
  if (millis() >= prevTime) {
    timeDiff = millis() - prevTime;
    if (timeDiff < SLEEP_TIME_DEFAULT) {
      return false;
    } else {
      // update time mark
      prevTime = millis();
      return true;
    }
  } else {
    // handle rollover
    prevTime = millis();
    return false;
  }
}

// --------------- Function: Gauge Scheduling -------------------
void scheduling() {
  if (currentGauge >= 0 && currentGauge <= 7) {
    if (gaugeRead(currentGauge)) {
      // success, move the gauge pointer to next gauge
      faultChk[currentGauge] = false;
      currentGauge++;
      // reset retry count
      retryCount = 0;
    } else {
      // fail, remain current gauge and try in next cycle
      if (retryCount > 10) {
        // declare faulty and move to next gauge
        faultChk[currentGauge] = true;
        currentGauge++;
        retryCount = 0;
      } else {
        // retry
        retryCount++;
      }
    }
  } else if (currentGauge == 8) {
    defoFault = defoRead();
//    Serial.print(millis());
//    Serial.print(";");
//    Serial.print(defoResult, 3);
//    Serial.print(";");
//    Serial.println(defoFault);
    currentGauge = 0;
  }
}

// --------------- Function: Communication Processing -----------
// - Provide strain gauge readings to Rig Client (REQV)
void requestValue(int addr) {
  switch (addr) {
    case 1:
      Serial.print("Cpl-");
      for (int i = 0; i < 4; i++) {
        Serial.print(strain[i]);
        Serial.print(";");
      }
      Serial.println(strain[4]);
      break;
    case 2:
      Serial.print("Cpl-");
      for (int i = 0; i < 4; i++) {
        Serial.print(strain[i + 5]);
        Serial.print(";");
      }
      Serial.println(strain[9]);
      break;
    case 3:
      Serial.print("Cpl-");
      for (int i = 0; i < 10; i++) {
        Serial.print((int)faultChk[i]);
        Serial.print(";");
      }
      Serial.print(mode);
      Serial.print(";");
      Serial.println(0);
      break;
    case 4:
      Serial.print("Cpl-");
      Serial.print(defoResult, 3);
      Serial.print(";");
      Serial.println(defoFault);
      break;
    default:
      Serial.println("Err-addr");
  }
}

// - Handling Rig Client Request for mode change (SETV)
void setValue(int addr, float val) {
  int intVal = (int)val;
  switch (addr) {
    case 1:
      mode = intVal;
      Serial.println("Cpl");
      break;
    default:
      Serial.println("Err-addr");
  }
}

// - Handling Rig Client Input for scales (SETS)
void setScale(int addr, float val) {
  if (addr >= 10 && addr <= 19) {
    scale[addr - 10] = val;
    scaleChk[addr - 10] = true;
    Serial.println("Cpl");
  } else {
    Serial.println("Err-addr");
  }
}

// - Handling Rig Client Input for offsets (SETO)
void setOffset(int addr, long val) {
  if (addr >= 10 && addr <= 19) {
    offset[addr - 10] = val;
    offsetChk[addr - 10] = true;
    Serial.println("Cpl");
  } else {
    Serial.println("Err-addr");
  }
}

// --------------- Main: System Setup & Initializtion -----------

void setup() {
  // initialize communication
  Serial.begin(115200);
  //Serial.println("Comm Started");
  // initialize Pins
  pinMode(32, INPUT);                  // Pin 32: Strain Gauge 1, DATA channel
  pinMode(33, INPUT);                  // Pin 33: Strain Gauge 2, DATA channel
  pinMode(34, INPUT);                  // Pin 34: Strain Gauge 3, DATA channel
  pinMode(35, INPUT);                  // Pin 35: Strain Gauge 4, DATA channel
  pinMode(36, INPUT);                  // Pin 36: Strain Gauge 5, DATA channel
  pinMode(37, INPUT);                  // Pin 37: Strain Gauge 6, DATA channel
  pinMode(38, INPUT);                  // Pin 38: Strain Gauge 7, DATA channel
  pinMode(39, INPUT);                  // Pin 39: Strain Gauge 8, DATA channel

  pinMode(42, OUTPUT);                 // Pin 42: Strain Gauge 1, CLK signal
  digitalWrite(42, LOW);
  pinMode(43, OUTPUT);                 // Pin 43: Strain Gauge 2, CLK signal
  digitalWrite(43, LOW);
  pinMode(44, OUTPUT);                 // Pin 44: Strain Gauge 3, CLK signal
  digitalWrite(44, LOW);
  pinMode(45, OUTPUT);                 // Pin 45: Strain Gauge 4, CLK signal
  digitalWrite(45, LOW);
  pinMode(46, OUTPUT);                 // Pin 46: Strain Gauge 5, CLK signal
  digitalWrite(46, LOW);
  pinMode(47, OUTPUT);                 // Pin 47: Strain Gauge 6, CLK signal
  digitalWrite(47, LOW);
  pinMode(48, OUTPUT);                 // Pin 48: Strain Gauge 7, CLK signal
  digitalWrite(48, LOW);
  pinMode(49, OUTPUT);                 // Pin 49: Strain Gauge 8, CLK signal
  digitalWrite(49, LOW);

  pinMode(PIN_REQ, OUTPUT);            // Deformation Gauge Pins
  pinMode(PIN_CLK, INPUT_PULLUP);
  pinMode(PIN_DAT, INPUT_PULLUP);
  digitalWrite(PIN_REQ, LOW);

  //Serial.println("Pin Mode Set");
  // initialize mode
  mode = 0;

  // initialize tracker
  currentGauge = 0;
  retryCount = 0;
  prevTime = millis();

  // initialize chk flags
  for (int i = 0; i < 10; i++) {
    scaleChk[i] = false;
    offsetChk[i] = false;
    faultChk[i] = true;
  }
  defoFault = true;
  //Serial.println("Starting init gauges");
  // initialize gauges
  for (int i = 0; i < 8; i++) {
    gauge[i].read();
  }

  defoResult = 99.99;

}

// --------------- Main: Function loop -----------------------------

void loop() {
  // Time check & Scan Management
  int count = 0;
  if (timeCheck()) {
    // mode 0: idle waiting for RigClient
    if (mode == 11) {                       // mode 11: Retrieve calibration data from RigClient
      mode = preCalCheck();
    } else if (mode == 12) {                 // mode 12: calibration
      calibration();
      currentGauge = 0;
      for (int i = 0; i < 8; i++) {
        faultChk[i] = true;
      }
      mode = 13;
      initTime = millis();
    } else if (mode == 13) {                 // mode 13: initialize readings
      scheduling();
      for (int i = 0; i < 8; i++) {         // - check reading status, note gauge 8 & 9 not installed
        if (faultChk[i]) {
          count++;
        }
      }
      if ((count == 0) || (millis() - initTime > 5000)) {
        mode = 14;                          // mode 14: idle waitng for RigClient
      }
    } else if (mode == 20) {                 // mode 20: operation
      scheduling();
    } else if (mode == 31) {                 // mode 31: cleanup
      delay(10);
      mode = 0;
    }
    //    Serial.print(mode);
    //    Serial.print(";");
    //    Serial.println(millis());
  }
  // if time check is false, i.e. in sleeping time, do nothing
  delayMicroseconds(100);
}

// --------------- Serial Communication ----------------------------

void serialEvent() {

  while (Serial.available() && !stringComplete) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if ((inChar == '\n') || (inputPointer == 63)) {
      stringComplete = true;
      inputPointer = 0;
    } else if (inputPointer < 63) {
      // allocate to the inputString buffer
      // move pointer to next location
      inputString[inputPointer] = inChar;
      inputPointer++;
    }
  }

  // *************************
  // Interpret input
  if (stringComplete) {
    char msgHeader[8] = "";
    char msgReq[5] = "";
    char msgAddr[3] = "";
    char msgVal[40] = "";
    for (int i = 0; i < 7; i++) {
      msgHeader[i] = inputString[i];
    }
    for (int i = 0; i < 4; i++) {
      msgReq[i] = inputString[i + 7];
    }
    for (int i = 0; i < 2; i++) {
      if ((inputString[i + 17] < '+') || (inputString[i + 17] > '9') || (inputString[i + 17] == '/') ) {
        break;
      }
      msgAddr[i] = inputString[i + 17];
    }
    for (int i = 0; i < 40; i++) {
      if ((inputString[i + 24] < '+') || (inputString[i + 24] > 'z') || ((inputString[i + 24] > ';') && (inputString[i + 24] < 'A')) || (inputString[i + 24] == '/') || ((inputString[i + 24] > 'Z') && (inputString[i + 24] < 'a'))) {
        break;
      }
      msgVal[i] = inputString[i + 24];
    }

    int intAddr = atoi(msgAddr);
    float flVal = atof(msgVal);
    long lonVal = atol(msgVal);
    if (strcmp(msgHeader, "rlab://") != 0) {
      Serial.println("Err-input");
      Serial.flush();
    } else {
      if (strcmp(msgReq, "REQV") == 0) {
        requestValue(intAddr);
        Serial.flush();
      } else if (strcmp(msgReq, "SETV") == 0) {
        setValue(intAddr, flVal);
        Serial.flush();
      } else if (strcmp(msgReq, "SETS") == 0) {
        setScale(intAddr, flVal);
        Serial.flush();
      } else if (strcmp(msgReq, "SETO") == 0) {
        setOffset(intAddr, lonVal);
        Serial.flush();
      } else {
        Serial.println("Err-command");
        Serial.flush();
      }
    }
    for (int i = 0; i < 64; i++) {
      inputString[i] = '\0';
    }
    stringComplete = false;
  }
}
