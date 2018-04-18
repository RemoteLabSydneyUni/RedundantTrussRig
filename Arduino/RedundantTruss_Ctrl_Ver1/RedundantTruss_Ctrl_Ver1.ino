// ************************************************************************************
//   Redundant Truss Hardware Controller - Actuator Control
// ----------------------------------------------------------------------------------
//   Version        :   1.1
//   Origin Date    :   18 Sep 2017
//   Modified Date  :   10 Mar 2018
//   Modified by    :   YR DENG
// ----------------------------------------------------------------------------------
//   Program Description
//  - To handle the load & angle control of the truss
// ----------------------------------------------------------------------------------
//   Version Notes
//  0.0     Initial Build. 
//          - Basic infrastructure for Arduino-Sahara Interface
//          - Chibi-OS infrastructure
//          - Interface for HX711 strain gauge amplifier
//          - Basic operation modes and handshaking
//
// 1.0      Upgrade
//          - Add handler for local (physical) maintenance switches
//          - Add interface to VL53L0X Distance sensor
//          - Modify angle controller to close-loop with distance sensor input
//          - Modify hand-shaking with RigClient
//          - Remove calibration feature from Arduino
//          Refine
//          - Rename some variables (more readable)
//          - Shift the calibration for angle control to RigClient (remove from Arduino)
// 1.1      Change Distance input from i2c to serial comm from another Arduino
//          
// ----------------------------------------------------------------------------------
//
// ************************************************************************************

// --------------- Library --------------------------------------
#include <HX711.h>
#include <ChibiOS_AVR.h>

#include <VL53L0X.h>      // Ver1.0
#include <Wire.h>         // Ver1.0

// --------------- Constant Declaration -------------------------
//  - Performance Parameter Definition
const float TENSION_DEADBAND = 1.0;        // Acceptable bias between tension reading and target (N)
const float TENSION_MAX_DEV = 20.0;        // Acceptable max deviation from
const int DISTANCE_DEADBAND_START = 15;    // Acceptable bias between distance reading and target (mm) to start motor action
const int DISTANCE_DEADBAND_STOP = 5;      // Acceptable bias between distance reading and target (mm) to stop motor action 
const int MAX_DISTANCE = 320;              // Maximum allowed distance (end of screw)
const int MIN_DISTANCE = 75;               // Minimum allowed distance (end of screw)
const float MAX_SET_TENSION = 325.0;       // Maximum allowed loading target
const float MAX_INTERLOCK_TENSION = 100.0;       // Maximum allowed loading target when changing angle
const float MIN_SET_TENSION = 25;          // Minimum loading (instrument readable)
const int SLEEP_TIME_DEFAULT = 1000;       // Default Sleep time (ms)

// --------------- Variable Declaration -------------------------
// - Angle Input ( Distance Sensor)
VL53L0X sensor; 
static int curDistance;                     // Current Distantce
static int tgtDistance;
static int avgDistance[5];
static int avgDistancePointer = 0;

static int rawDistance;                     // Distance from serial
static int rawTimeOut;                      // Timeout from serial
static boolean reqOut = false;              // request out flag
static boolean commFail= false;             // indicate the link to sensor arduino fails

// - Angle control (Screw drive)
static boolean angleMode;
static long angleOutput;                    // Target Steps to move in Auto mode
static long angleStepTarget;                // Target Steps to move in Manual mode
static boolean angleDirection;              // Target Direction to move 

static boolean angleCtrlActionTracker;

// - Load Sensor Input (Load Cell)
static float loadScale = 6997.683;          // scale of load strain gauge (With default value)
static boolean loadScaleChk = false;
static long loadOffset = -41324L;           // offset of load strain gauge (With default value)
static boolean loadOffsetChk = false;
static float curLoad;
static float tgtLoad;
static float avgLoad[5];                    // rolling average for load input
static int avgLoadPointer = 0;              // RA pointer

// - Load control (Winch)
static boolean loadMode;
static long loadOutput;                     // Target Steps to move in Auto mode
static long loadStepTarget;                 // Target Steps to move in Manual mode
static boolean loadDirection;               // Target Direction to move

// - Serial Communication
static char inputString[64] = "";           // Serial Comm - a string to hold incoming data
static int inputPointer;                    // Serial Comm - input store pointer
static boolean stringComplete = false;      // Serial Comm - whether the string is complete

// - Serial Communication with input arduino
static char inputString1[64] = "";          // Serial Comm - a string to hold incoming data
static int inputPointer1;                   // Serial Comm - input store pointer
static boolean stringComplete1 = false;     // Serial Comm - whether the string is complete


// - Overall Operation Status
static int operationMode;                   // Operating mode
static boolean faultTension = true;        // Indication of Tension reading loss
static boolean faultDistance = true;       // Indication of Distance reading loss
static boolean loadCaliRequest = false;     // Request flag for calibration (load drive)

static boolean angleInpPerm = false;        // Processing Permissive for angle input handling
static boolean angleCtrlPerm = false;       // Processing Permissive for angle control handling
static boolean loadInpPerm = false;         // Processing Permissive for load input handling
static boolean loadCtrlPerm = false;        // Processing Permissive for load control handling

static boolean snapshotInterlock = false;

static unsigned long timeoutTimer;

static int angleInputWD = 0;
// - Debug
static int debugReading = 0;

// --------------- Strain Gauge Declaration ---------------------
HX711 loadGauge = HX711(30,40);

// --------------- Thread: Operation Mode Management ------------
// Operation modes: Nn
// N: Action. 0 = idle; 1 = initialization; 2 = operation; 3 = cleanup
// n: detail stages (for detail tasks and handshaking)

static THD_WORKING_AREA(waMode,512);
static void thModeManager(void *arg){
  while (!chThdShouldTerminateX()) {
    if (operationMode == 0){
      // idle, do nothing
      setPermissive(false,false,false,false);
      digitalWrite(44,LOW);
    }else if (operationMode == 11){
      // Init 1: waiting for information from Rig
      setPermissive(false,false,false,false);
      digitalWrite(44,HIGH);      
      if ((loadScaleChk == false)||(loadOffsetChk == false)){
        operationMode = 11;
      }else{
        loadGauge.set_scale(loadScale);
        loadScaleChk = false;
        loadGauge.set_offset(loadOffset);
        loadOffsetChk = false;
        timeoutTimer = millis();
        operationMode = 12;
      }      
    }else if (operationMode == 12){
      // Init 2: initialize input
      setPermissive(true,false,true,false);
      digitalWrite(44,HIGH);
      if (((!faultTension)&&(!faultDistance))||((millis()-timeoutTimer)>10000)) {
        tgtDistance = min(max(curDistance,MIN_DISTANCE),MAX_DISTANCE);
        tgtLoad = min(max(curLoad,MIN_SET_TENSION),MAX_SET_TENSION);
        operationMode = 13;
      } else{
        operationMode = 12;
      }
    }else if (operationMode == 13){
      // Init 3: waiting for Rig Client to put to service
      setPermissive(true,false,true,false);
      digitalWrite(44,HIGH);
      tgtDistance = min(max(curDistance,MIN_DISTANCE),MAX_DISTANCE);
      tgtLoad = min(max(curLoad,MIN_SET_TENSION),MAX_SET_TENSION);
      
    }else if (operationMode == 20){
      setPermissive(true,true,true,true);
      digitalWrite(44,HIGH);
    }else if (operationMode == 31){
      // Cleanup
      setPermissive(true,false,true,true);
      digitalWrite(44,HIGH);
      angleOutput = 0;                                  // call stop of angel control
      angleStepTarget = 0;                              // call stop of angel control
      tgtLoad = MIN_SET_TENSION;                        // release load
      if ((curLoad <= (MIN_SET_TENSION + TENSION_DEADBAND))|| faultTension||loadMode){
        operationMode = 0;
      }
    }
    chThdSleepMilliseconds(10);
  }
}
// --------------- Thread: Angle input polling ------------------
static THD_WORKING_AREA(waAngleInputRequest,256);
static void thAngleInputRequesting(void *arg){
  while (!chThdShouldTerminateX()) {
   if (angleInpPerm){                                   // With permission to run, check the previous request status
      commFail = reqOut;                                // previous request sent but no response, a.k.a comm timeout
      Serial1.println("REQV");                          // Send command this round
      reqOut = true;                                    // raise sent flag
    }    
    chThdSleepMilliseconds(100);
  }
}

// --------------- Thread: Angle input handling -----------------
static THD_WORKING_AREA(waAngleInput,512);
static void thAngleInputHandling(void *arg){
  while (!chThdShouldTerminateX()) {
    if (angleInpPerm){                                  
      int count = 0;
      int val = 0;
      if (!rawTimeOut){
        avgDistance[avgDistancePointer] = rawDistance;
        for (int i = 0; i<3;i++){
          if ((avgDistance[i] <= 2000) && (avgDistance[i] >=20)){
            val = val + avgDistance[i];
            count++;
          }
        }
        if (count == 0){
          curDistance = avgDistance[avgDistancePointer];
          faultDistance = true;
        }else{
          curDistance = val / count;
          faultDistance = false;
        }
        debugReading = avgDistance[avgDistancePointer]+millis();
        // move pointer
        if (avgDistancePointer == 2){
          avgDistancePointer = 0;
        }else{
          avgDistancePointer++;
        }
      }else{
        debugReading = -1;
        faultDistance = true;
      }
      if(angleInputWD > 10){
        faultDistance = true;
      }
      if(angleInputWD <999){
        angleInputWD++;
      }
    }    
    chThdSleepMilliseconds(100);
  }
}

// --------------- Thread: Load input handling ------------------
static THD_WORKING_AREA(waLoadInput,512);
static void thLoadInputHandling(void *arg){
  while (!chThdShouldTerminateX()) {
    if (loadInpPerm){
      float val = 0.0;
      int count = 0;
      if (loadGauge.is_ready()){
        // Store value in the array
        avgLoad[avgLoadPointer] = loadGauge.get_units(1);
        // analyze value                     
        for (int i = 0; i<5; i++){
          // filter out bad value
          if ((avgLoad[i]<= 500 + TENSION_DEADBAND)&& (avgLoad[i]>= 0)){
            val = val + avgLoad[i];
            count++;
          }
        }
        //Analyze result
        if (count == 0){    // all value bad
          curLoad = avgLoad[avgLoadPointer];      // return last reading
          faultTension = true;                    // flag out bad
        }else{
          curLoad = val / (float) count;          // return average of last good values
          faultTension = false;                   // flag out good health
        }
        // Interlock when current reading is healthy
        if ((avgLoad[avgLoadPointer]<= 500 + TENSION_DEADBAND)&& (avgLoad[avgLoadPointer]>= MAX_INTERLOCK_TENSION+ 5)){
          snapshotInterlock = true;
        }else{
          snapshotInterlock = false;
        }
        // move pointer
        if (avgLoadPointer == 4){
          avgLoadPointer = 0;
        }else{
          avgLoadPointer++;
        }
      }
    }
    chThdSleepMilliseconds(25);
  }
}

// --------------- Thread: Angle control ------------------------
static THD_WORKING_AREA(waAngleCtrl,256);
static void thAngleCtrl(void *arg){
  while (!chThdShouldTerminateX()) {
    angleMode = digitalRead(23);                                            // Check Local Switch
    if (angleCtrlPerm){
      if (angleMode){                                                       // Manual Mode: Operatable regardless health status of input (angleMode = true)
        tgtDistance = min(max(curDistance,MIN_DISTANCE),MAX_DISTANCE);      // Distance SP track PV, avoid bumping
        if (abs(angleStepTarget)>0){                                        // When new command arrived
          angleDirection = (angleStepTarget >= 0);                          // - Check direction
          angleOutput = abs(angleStepTarget);                               // - Pass user specified motor step target to motor driver
          angleStepTarget = 0;                                              // - clear register for next command
        }                                                                   // When no new command, do nothing
      }else{                                                                // Auto Mode: Only operatable when both angle & load sensor online (angleMode = false)
        if ((!faultTension) && (!faultDistance)){
          if (curLoad >= (MAX_SET_TENSION+ 5)){                             // - safety lock, angle drive will not move when load is too high
            angleOutput = 0;
          }else{  
            angleOutput = calcAngleOutput();                                // - Calculate motor step target and pass to motor driver
          }
        }else{
          angleOutput = 0;
        }
        angleStepTarget = 0;                                                // - Clear Stepping target (in case any trial to stepping motor in Auto mode
      }
    }
    chThdSleepMilliseconds(300);
  }  
}

// --------------- Thread: Load control -------------------------
static THD_WORKING_AREA(waLoadCtrl,256);
static void thLoadCtrl(void *arg){
  while (!chThdShouldTerminateX()) {
  loadMode = digitalRead(22);                                               // Check Local Switch
    if (loadCtrlPerm){
      if (loadMode){                                                        // Manual Mode: Operatable regardless health status of input (loadMode = true)
        tgtLoad = min(max(curLoad,MIN_SET_TENSION),MAX_SET_TENSION);        // Load SP track PV, avoid bumping
        if(abs(loadStepTarget)>0){                                          // When new command arrived
          loadDirection = (loadStepTarget >=0);                             // - Check direction
          loadOutput = abs(loadStepTarget);                                 // - Pass user specified motor step target to motor driver
          loadStepTarget =0;                                                // - clear register for next command
      }                                                                     // When no new command, do nothing
      }else{                                                                // Auto Mode: Only operatable when load sensor online (loadMode = false)
        if (!faultTension){
          loadOutput = calcLoadOutput();                                    // - Calculate motor step target and pass to motor driver
        }else{
          loadOutput = 0;
        }
        loadStepTarget =0;                                                  // - Clear Stepping target (in case any trial to stepping motor in Auto mode
      }
    }
    chThdSleepMilliseconds(300);
  }  
}

// --------------- Thread: Angle Drive ---------------------------
static THD_WORKING_AREA(waAngleDrive,128);
static void thAngleMotorDrive(void *arg){
  while (!chThdShouldTerminateX()) {
    if (!snapshotInterlock){
      if (angleDirection){
        digitalWrite(51,LOW);
      }else{
        digitalWrite(51,HIGH);
      }
      if (angleOutput > 0){
        digitalWrite(50,HIGH);           
        delayMicroseconds(5);
        digitalWrite(50,LOW);
        angleOutput--;
      }
    }
    chThdSleepMicroseconds(500);
  }
}

// --------------- Thread: Load Drive ---------------------------
static THD_WORKING_AREA(waLoadDrive,128);
static void thLoadMotorDrive(void *arg){
  while (!chThdShouldTerminateX()) {
    if (loadDirection){
      digitalWrite(53,LOW);
    }else{
      digitalWrite(53,HIGH);
    }
    if (loadOutput > 0){
      digitalWrite(52,HIGH);           
      delayMicroseconds(5);
      digitalWrite(52,LOW);
      loadOutput--;
    }
    chThdSleepMicroseconds(500);
  }
}

// --------------- Thread: Comm scan ----------------------------
static THD_WORKING_AREA(waComm,512);
static void thCommScan(void *arg){
  while (!chThdShouldTerminateX()) {
    if (serialEventRun){
      serialEventRun();
    }
    chThdSleepMilliseconds(10);
  }
}

// --------------- Thread: Main ---------------------------------
void chMain() {  
      chThdCreateStatic(waComm, sizeof(waComm),NORMALPRIO+40, thCommScan, NULL);
      chThdCreateStatic(waMode, sizeof(waMode),NORMALPRIO+30, thModeManager, NULL);
      chThdCreateStatic(waLoadCtrl, sizeof(waLoadCtrl),NORMALPRIO+15, thLoadCtrl, NULL);
      chThdCreateStatic(waAngleCtrl, sizeof(waAngleCtrl),NORMALPRIO+10, thAngleCtrl, NULL);
      chThdCreateStatic(waLoadInput, sizeof(waLoadInput),NORMALPRIO+9, thLoadInputHandling, NULL);
      chThdCreateStatic(waAngleInputRequest, sizeof(waAngleInputRequest),NORMALPRIO+7, thAngleInputRequesting, NULL);
      chThdCreateStatic(waAngleInput, sizeof(waAngleInput),NORMALPRIO+8, thAngleInputHandling, NULL);
      chThdCreateStatic(waLoadDrive, sizeof(waLoadDrive),NORMALPRIO+6, thLoadMotorDrive, NULL);
      chThdCreateStatic(waAngleDrive, sizeof(waAngleDrive),NORMALPRIO+5, thAngleMotorDrive, NULL);
      
      chThdSleepMilliseconds(10);
}
// --------------- Initialization -------------------------------
void setup() {
  
  // initialize serial port.
  Serial.begin(115200);
  Serial1.begin(115200);
  
  // initialize pins - Driver
  pinMode(50,OUTPUT);                  // Pin 50: Screw Drive Motor, Step pin
  digitalWrite(50,LOW);
  pinMode(51,OUTPUT);                  // Pin 51: Screw Drive Motor, Direction pin
  digitalWrite(51,LOW);
  pinMode(52,OUTPUT);                  // Pin 52: Winch Motor, Step pin
  digitalWrite(52,LOW);
  pinMode(53,OUTPUT);                  // Pin 53: Winch Motor, Direction pin
  digitalWrite(53,LOW);

  // initialize pins - Communication
  pinMode(30,INPUT);                   // Pin 30: Load Strain Gauge, DATA channel
  pinMode(40,OUTPUT);                  // Pin 40: Load Strain Gauge, CLK signal
  digitalWrite(40,LOW);

  // initialize pine - Local Switch
  pinMode(22,INPUT);                   // Pin 22: local switch for Winch Motor
  pinMode(23,INPUT);                   // Pin 23: local switch for Screw Motor

  //Indication Pilot Light
  pinMode(44, OUTPUT);
  digitalWrite(44,LOW);
   
  // initialize variables
  operationMode = 0;
  tgtLoad = MIN_SET_TENSION;

  loadScaleChk = false;
  loadOffsetChk = false;
  faultTension = true;
  faultDistance = true;
  angleInpPerm = false;
  angleCtrlPerm = false; 
  loadInpPerm = false;  
  loadCtrlPerm = false;

  snapshotInterlock = false;
  reqOut = false;
  commFail = false;

  angleCtrlActionTracker = false;

  // initialize strain gauges
  loadGauge.read();

  // initialize distance sensor
  Wire.begin();

  sensor.init();
  sensor.setTimeout(10);
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensor.setMeasurementTimingBudget(20000);
  sensor.startContinuous(100);

  // Now start the main operating process
  chBegin(chMain);
  while(1) {  }
}
// --------------- Main Loop (not in use) -----------------------
void loop() {
}
// --------------- Comm Handling: Serial event ------------------
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
        char msgHeader[8] = "";
        char msgReq[5] = "";
        char msgAddr[3] = "";
        char msgVal[40] = "";
        for (int i = 0; i<7;i++){
          msgHeader[i] = inputString[i];
        }
        for (int i = 0; i<4;i++){
          msgReq[i] = inputString[i+7];
        }
        for (int i = 0; i<2;i++){
          if ((inputString[i+17] < '+') || (inputString[i+17] > '9') ||(inputString[i+17] == '/') ) {
            break; 
          }
          msgAddr[i] = inputString[i+17];
        }
        for (int i = 0; i<40;i++){
          if ((inputString[i+24] < '+') || (inputString[i+24] > 'z') ||((inputString[i+24] > ';')&&(inputString[i+24] < 'A'))||(inputString[i+24] == '/')||((inputString[i+24] > 'Z')&&(inputString[i+24] < 'a'))) {
            break; 
          }
          msgVal[i] = inputString[i+24];
        }
        
        int intAddr = atoi(msgAddr);
//        float flVal = atof(msgVal);
//        long lonVal = atol(msgVal);
        if(strcmp(msgHeader,"rlab://") != 0) {                                           
          Serial.println("Err-input");
          Serial.flush();
        }else{
          if (strcmp(msgReq,"REQV") == 0){
            requestValue(intAddr);
            Serial.flush();
          }else if (strcmp(msgReq,"SETV") == 0){
            setValue(intAddr,msgVal);
            Serial.flush();
          }else{
            Serial.println("Err-command");
            Serial.flush();
          }              
        }
        for (int i=0;i<64;i++){
          inputString[i] = '\0';
        }
        stringComplete = false;
    }
}
// --------------- Comm Handling: Serial event (with input arduino)
void serialEvent1() {
    
    while (Serial1.available() && !stringComplete1) {
      // get the new byte:
      char inChar = (char)Serial1.read();
      // if the incoming character is a newline, set a flag
      // so the main loop can do something about it:
      if ((inChar == '\n')||(inputPointer1 == 63)) {
        stringComplete1 = true;
        inputPointer1 = 0;
      }else if (inputPointer1 < 63){
      // allocate to the inputString buffer
      // move pointer to next location
        inputString1[inputPointer1] = inChar;
        inputPointer1++;
      }
    }
    
    // *************************
    // Interpret input
    if (stringComplete1) {
      char responseHeader[4] = "";
      char responseMsg[60] = "";
      char* currptr;
      char *endptr;
      for (int i = 0; i<3;i++){
        responseHeader[i] = inputString1[i];
      }
      for (int j = 0; j<59;j++){
        responseMsg[j]= inputString1[j+4];
      }
      if (strcmp(responseHeader,"Cpl") == 0){
        currptr = responseMsg;
        rawDistance = (int)strtol(currptr, &endptr, 10);
        currptr = endptr+1;
        rawTimeOut = (int)strtol(currptr, &endptr, 10);
        currptr = endptr+1;
        angleInputWD = (int)strtol(currptr, &endptr, 10);
      }
      for (int i=0;i<64;i++){
        inputString1[i] = '\0';
      }
      stringComplete1 = false;
      // off the comm checking flags
      reqOut = false;  
    }

}

// --------------- Subroutine: provide value to comm ------------
void requestValue(int addr){
   switch (addr){
    case 1:
      Serial.print("Cpl-");
      Serial.print(curDistance);
      Serial.print(";");
      Serial.print(tgtDistance);
      Serial.print(";");
      Serial.println(angleMode);
      break;
    case 2:
      Serial.print("Cpl-");
      Serial.print(curLoad);
      Serial.print(";");
      Serial.print(tgtLoad);
      Serial.print(";");
      Serial.println(loadMode);
      break;
    case 3:
      Serial.print("Cpl-");
      Serial.print(faultTension);
      Serial.print(";");
      Serial.print(faultDistance);
      Serial.print(";");      
      Serial.print(operationMode);
      Serial.print(";");
      Serial.println(0);
      break;
    case 4:                             // for debugging
      Serial.print("Cpl-");
      Serial.print(angleStepTarget);
      Serial.print(";");
      Serial.print(loadStepTarget);
      Serial.print(";");      
      Serial.print(loadOutput);
      Serial.print(";");
      Serial.print(angleOutput);
      Serial.print(";");
      Serial.print(debugReading);
      Serial.print(";");
      Serial.print(commFail);
      Serial.print(";");
      Serial.print(rawDistance);
      Serial.print(";");
      Serial.print(rawTimeOut);
      Serial.print(";");
      Serial.print(snapshotInterlock);
      Serial.print(";");
      Serial.println(angleInputWD);    
      break;
    default:
      Serial.println("Err-addr");
   }
}

// --------------- Subroutine: Handle comm request --------------
void setValue(int addr, char val[]){
  switch (addr){
    int intVal;
    // case 00-09: General Purpose
    case 1:                                                     // Set Operation Mode
      intVal = atoi(val);
      if((intVal == 11) || (intVal == 20) || (intVal == 31)){ 
        operationMode = intVal;
        Serial.println("Cpl");
      }else{
        Serial.println("Err-Value");
      }
      break;      
    // case 10-19: Load Control  
    case 11:                                                    // set load target
      tgtLoad = min(max(atof(val),MIN_SET_TENSION),MAX_SET_TENSION);
      Serial.println("Cpl");
      break;
    case 12:                                                    // set load control motor stepping target
      loadStepTarget = atol(val);
      Serial.println("Cpl");
      break;
    case 13:                                                    // set load cell scale
      loadScale = atof(val);
      loadScaleChk = true;
      Serial.println("Cpl");
      break;
    case 14:                                                    // set load cell offset
      loadOffset = atol(val);
      loadOffsetChk =true;
      Serial.println("Cpl");
      break;
      
    // case 20-29: Angle Control
    case 21:                                                    // set angle(distance) target
      tgtDistance = min(max(atoi(val),MIN_DISTANCE),MAX_DISTANCE);
      Serial.println("Cpl");
      break;
    case 22:                                                    // set angle control motor stepping target
      angleStepTarget = atol(val);
      Serial.println("Cpl");
      break;
    default:
      Serial.println("Err-addr");
   }
}

// --------------- Subroutine: Angle output calculation ----------
long calcAngleOutput(){
  long outputVal;
  int diff;
  if (tgtDistance > curDistance){
    angleDirection = false;
  }else{
    angleDirection = true;
  }
  diff = abs(tgtDistance - curDistance);
  if (diff> DISTANCE_DEADBAND_START){
    angleCtrlActionTracker = true;
  }
  if (diff<= DISTANCE_DEADBAND_STOP){
    angleCtrlActionTracker = false;
  }
  if (angleCtrlActionTracker){
      outputVal = min((long)(diff*40),400);
  }else{
    outputVal = 0;
  }
  return outputVal;
}
// --------------- Subroutine: Load output calculation ----------
long calcLoadOutput(){
  long outputVal;
  float diff;
  if (tgtLoad > curLoad){
    loadDirection = true;
  }else{
    loadDirection = false;
  }
  diff = abs(tgtLoad - curLoad);
  if (diff> TENSION_DEADBAND){
    if (diff < 20.0){
      outputVal = min((long)(diff*5.0),1000);
    }else if (diff< 50.0){
      outputVal = min((long)((diff-20.0)*10.0)+100,5000);
    }else{
      outputVal = min((long)((diff-50)*100.0) + 400,10000);
    }
  }else{
    outputVal = 0;
  }
  return outputVal;
}

// --------------- Subroutine: Permissive Check -----------------
void setPermissive(boolean angleI, boolean angleC, boolean loadI, boolean loadC){
  angleInpPerm = angleI;
  angleCtrlPerm = angleC; 
  loadInpPerm = loadI;  
  loadCtrlPerm = loadC;
}
