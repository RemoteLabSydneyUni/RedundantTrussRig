// ************************************************************************************
//   Redundant Truss Hardware Controller - Actuator Control
// ----------------------------------------------------------------------------------
//   Version    :   0.0
//   Date       :   18 Sep 2017
//   Modified by:   YR DENG
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
// ----------------------------------------------------------------------------------
//
// ************************************************************************************

// --------------- Library --------------------------------------
#include <HX711.h>
#include <ChibiOS_AVR.h>

// --------------- Constant Declaration -------------------------
//  - Performance Parameter Definition
const float TENSION_DEADBAND = 5.0;        // Acceptable bias between tension reading and target
const float TENSION_MAX_DEV = 20.0;        // Acceptable max deviation from
const long MAX_ALLOW_STEP = 380000L;           // Maximum allowed step (end of screw)
const long MIN_ALLOW_STEP = 0L;               // Maximum allowed step (end of screw)
const float MAX_SET_TENSION = 200.0;        // Maximum allowed loading target
const float MIN_SET_TENSION = 5.6;         // Minimum loading (instrument readable)
const int SLEEP_TIME_DEFAULT = 1000;        // Default Sleep time (ms)

// --------------- Variable Declaration -------------------------
// - Angle control (Screw drive)
static long curAngleStep;                    // Absolute Current Step count for the Screw Drive
static long tgtAngleStep;                    // Absolute Target Step count for the Screw Drive
static long refAngleStep;                    // Reference step count (0 point)
static boolean refStepChk = false;
static boolean curStepChk = false;
static int angleMode; 

// - Load control (Winch)
static float loadScale = 1.0;               // scale of load strain gauge
static boolean loadScaleChk = false;
static long loadOffset = 0L;                // offset of load strain gauge
static boolean loadOffsetChk = false;
static long loadOutput;                      // Target Steps to move in Auto mode
static long loadTarget;
static boolean loadDirection;               // Target Direction to move in Auto mode
static int loadMode;
static float curLoad;
static float tgtLoad;
static float avgLoad[10];                   // rolling average for load input
static int avgLoadPointer = 0;              // RA pointer

// - Serial Communication
static char inputString[64] = "";           // Serial Comm - a string to hold incoming data
static int inputPointer;                    // Serial Comm - input store pointer
static boolean stringComplete = false;      // Serial Comm - whether the string is complete

// - Overall Operation Status
static int operationMode;                         // Operating mode
static boolean manOverride = false;         // Operating Mode, True = Manual, False = Auto;
static boolean faultTension = false;        // Indication of Tension reading loss

// --------------- Strain Gauge Declaration ---------------------
HX711 loadGauge = HX711(30,40);

// --------------- Thread: Operation Mode Management ------------
static THD_WORKING_AREA(waMode,128);
static void thModeManager(void *arg){
  while (!chThdShouldTerminateX()) {
    if (operationMode == 1){
      // idle, do nothing
    }else if (operationMode == 2){
      operationMode = initMode();
    }else if (operationMode == 3){
      // idle, wait for command from Rig
    }else if (operationMode == 4){
      // working condition
    }else if (operationMode == 5){
      // Cleanup
      tgtAngleStep = curAngleStep; // call stop of angel control
      tgtLoad = MIN_SET_TENSION;   // release load
      if ((curLoad <= (MIN_SET_TENSION + TENSION_DEADBAND))||(faultTension == true)){
        operationMode = 1;
      }
    }
    chThdSleepMilliseconds(10);
  }
}

// --------------- Thread: Load input handling ------------------
static THD_WORKING_AREA(waInput,512);
static void thInputHandling(void *arg){
  while (!chThdShouldTerminateX()) {
    if ((operationMode == 4) || (operationMode == 5)){
      float val = 0.0;
      int count = 0;
      if (loadGauge.is_ready()){
        // Store value in the array
        avgLoad[avgLoadPointer] = loadGauge.get_units(1);
        // analyze value                     
        for (int i = 0; i<10; i++){
          // filter out bad value
          if ((avgLoad[i]<= MAX_SET_TENSION + TENSION_DEADBAND)&& (avgLoad[i]>= MIN_SET_TENSION)){
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
        // move pointer
        if (avgLoadPointer == 9){
          avgLoadPointer = 0;
        }else{
          avgLoadPointer++;
        }
      }
    }
    chThdSleepMilliseconds(25);
  }
}

// --------------- Thread: Load control -------------------------
static THD_WORKING_AREA(waLoadCtrl,256);
static void thLoadCtrl(void *arg){
  while (!chThdShouldTerminateX()) {
    if ((operationMode == 4) || (operationMode == 5)){
      if (faultTension ==  false){                      // With good load input
        if (loadMode == 0){                             // Manual Mode
          if (loadTarget != 0){                         // only set load output once per user command
            loadOutput = abs(loadTarget);               // - calculate step from specified steps from user
            if (loadTarget >= 0){
              loadDirection = true;
            }else{
              loadDirection = false;
            }
          }
          tgtLoad = curLoad;                            // setpoint tracking
          loadTarget =0;                                // reset step target
        }else if (loadMode == 1){                       // Auto Mode
          loadOutput = calcLoadOutput();
        }else if (loadMode == 2){                       // Calibration Mode
          if ((loadScaleChk == true)&&(loadOffsetChk == true)){
            loadGauge.set_scale(loadScale);
            loadScaleChk = false;
            loadGauge.set_offset(loadOffset);
            loadOffsetChk = false;
            loadMode = 0;
          }
        }
      }else{                                            // with BAD input, lock in Manual mode
        loadMode = 0;
        if (loadTarget != 0){                           // only set load output once per user command
          loadOutput = abs(loadTarget);                 // - calculate step from specified steps from user
          if (loadTarget >= 0){
            loadDirection = true;
          }else{
            loadDirection = false;
          }
        }
        loadTarget =0;                                 //reset step target
      }
    }
    chThdSleepMilliseconds(100);
  }  
}

// --------------- Thread: Load Drive ---------------------------
static THD_WORKING_AREA(waDrive,128);
static void thMotorDrive(void *arg){
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

// --------------- Thread: Angle control ------------------------
static THD_WORKING_AREA(waAngleCtrl,256);
static void thAngleCtrl(void *arg){
  while (!chThdShouldTerminateX()) {
    if ((operationMode == 4) || (operationMode == 5)){
      if ((angleMode == 0)||(angleMode == 1)){        // Manual Mode, bypass safety check
        if (tgtAngleStep > curAngleStep){
          digitalWrite(51,LOW);
          digitalWrite(50,HIGH);           
          delayMicroseconds(5);
          digitalWrite(50,LOW);
          curAngleStep++;
        }else if (tgtAngleStep < curAngleStep){
          digitalWrite(51,HIGH);
          digitalWrite(50,HIGH);           
          delayMicroseconds(5);
          digitalWrite(50,LOW);
          curAngleStep--;
        }
      }else if ((faultTension = false)&&(angleMode == 1)){ // Automode, when load reading healthy, safety check      
        if ((curLoad <= tgtLoad+TENSION_MAX_DEV)&&(curLoad <=MAX_SET_TENSION)){   //safety check
          if (tgtAngleStep > curAngleStep){
            digitalWrite(51,LOW);
            digitalWrite(50,HIGH);           
            delayMicroseconds(5);
            digitalWrite(50,LOW);
            curAngleStep++;
          }else if (tgtAngleStep < curAngleStep){
            digitalWrite(51,HIGH);
            digitalWrite(50,HIGH);           
            delayMicroseconds(5);
            digitalWrite(50,LOW);
            curAngleStep--;
          }
        }
      }else if ((faultTension = true)&&(angleMode == 1)){  // Automode, when load reading fail, kick to manual mode
        angleMode = 0;  
      }else if (angleMode == 2){                      // Calibration Mode
        refAngleStep = curAngleStep;                  // - reset reference point
        tgtAngleStep = curAngleStep;
        angleMode = 0;
      }
    }
    chThdSleepMicroseconds(500);
  }  
}

// --------------- Thread: Local Panel --------------------------

// --------------- Thread: Comm scan ----------------------------
static THD_WORKING_AREA(waComm,128);
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
      chThdCreateStatic(waMode, sizeof(waMode),NORMALPRIO+30, thModeManager, NULL);
      chThdCreateStatic(waInput, sizeof(waInput),NORMALPRIO+8, thInputHandling, NULL);
      chThdCreateStatic(waComm, sizeof(waComm),NORMALPRIO+40, thCommScan, NULL);
      chThdCreateStatic(waLoadCtrl, sizeof(waLoadCtrl),NORMALPRIO+15, thLoadCtrl, NULL);
      chThdCreateStatic(waAngleCtrl, sizeof(waAngleCtrl),NORMALPRIO+10, thAngleCtrl, NULL);
      chThdCreateStatic(waDrive, sizeof(waDrive),NORMALPRIO+5, thMotorDrive, NULL);
      chThdSleepMilliseconds(10);
}
// --------------- Initialization -------------------------------
void setup() {
  
  // initialize serial port.
  Serial.begin(115200);
  
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
   
  // initialize variables
  operationMode = 1;

  refStepChk = false;
  loadScaleChk = true;
  loadOffsetChk = false;
  faultTension = true;
  curStepChk = false;

  // initialize strain gauges
  loadGauge.read();

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
        float flVal = atof(msgVal);
        long lonVal = atol(msgVal);
        if(strcmp(msgHeader,"rlab://") != 0) {                                           
          Serial.println("Err-input");
          Serial.flush();
        }else{
          if (strcmp(msgReq,"REQV") == 0){
            requestValue(intAddr);
            Serial.flush();
          }else if (strcmp(msgReq,"SETV") == 0){
            setValue(intAddr,flVal);
            Serial.flush();
          }else if (strcmp(msgReq,"SETM") == 0){
            setMode(intAddr,(int)flVal);
            Serial.flush();
          }else if (strcmp(msgReq,"SETS") == 0){
            setStep(intAddr,lonVal);
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
// --------------- Subroutine: provide value to comm ------------
void requestValue(int addr){
   switch (addr){
    case 1:
      Serial.print("Cpl-");
      Serial.print(curAngleStep);
      Serial.print(";");
      Serial.print(tgtAngleStep);
      Serial.print(";");
      Serial.print(refAngleStep);
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
      Serial.print(operationMode);
      Serial.print(";");
      Serial.println(0);
      break;
    default:
      Serial.println("Err-addr");
   }
}

// --------------- Subroutine: Handle comm request --------------
void setValue(int addr, float val){
  switch (addr){
    case 11:                   // set load target
      tgtLoad = min(max(val,MIN_SET_TENSION),MAX_SET_TENSION);
      Serial.println("Cpl");
      break;
    case 12:                   // set load sensor scale
      loadScale = val;
      loadScaleChk = true;
      Serial.println("Cpl");
      break;
    default:
      Serial.println("Err-addr");
   }
}

void setMode(int addr, int val){
  switch (addr){
    case 1:
      operationMode = min(max(val,0),5);
      Serial.println("Cpl");
      break;
    case 11:
      loadMode = min(max(val,0),2);
      Serial.println("Cpl");
      break;
    case 21:
      angleMode = min(max(val,0),2);
      Serial.println("Cpl");
      break;
    default:
      Serial.println("Err-addr");
   }
}

void setStep(int addr, long val){
  switch (addr){
    case 11:
      loadOffset = val;
      loadOffsetChk =true;
      Serial.println("Cpl");
      break;
    case 12:
      loadTarget = val;
      Serial.println("Cpl");
      break;
    case 21:
      tgtAngleStep = min(max(val,MIN_ALLOW_STEP),MAX_ALLOW_STEP);
      Serial.println("Cpl");
      break;
    case 22:
      refAngleStep = min(max(val,MIN_ALLOW_STEP),MAX_ALLOW_STEP);
      refStepChk = true;
      Serial.println("Cpl");
      break;
    case 23:
      curAngleStep = min(max(val,MIN_ALLOW_STEP),MAX_ALLOW_STEP);
      curStepChk = true;
      Serial.println("Cpl");
      break;
    default:
      Serial.println("Err-addr");
   }
}
// --------------- Subroutine: initialization check -------------
int initMode(){
  if ((loadScaleChk == false)||(loadOffsetChk == false)||(refStepChk == false)||(curStepChk == false)){
    return 2;
  }else{
    loadGauge.set_scale(loadScale);
    loadScaleChk = false;
    loadGauge.set_offset(loadOffset);
    loadOffsetChk = false;
    tgtAngleStep= curAngleStep;
    refStepChk = false;
    curStepChk = false;    
    return 3;
  }
}
// --------------- Subroutine: Load output calculation ----------
long calcLoadOutput(){
  long outputVal;
  if (tgtLoad > curLoad){
    loadDirection = true;
  }else{
    loadDirection = false;
  }
  if (abs(tgtLoad - curLoad)> TENSION_DEADBAND){
    outputVal = min((long)(abs(tgtLoad - curLoad)/0.02),25);
  }else{
    outputVal = 0;
  }
  return outputVal;
}

