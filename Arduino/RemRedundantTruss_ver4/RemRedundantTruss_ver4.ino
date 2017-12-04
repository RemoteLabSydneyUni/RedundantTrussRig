/* --------------------------------------------------------------------------------------------------------
 *  Redundant Truss Lab Apparatus Remote Control Program
 *  -------------------------------------------------------------------------------------------------------
 *  Version:
 *    0.0         Initial Trial             YR.DENG           16-FEB-2016
 *    
 *  Note:
 *    0.0   : Structure for data acquisition and control with Arduino Mega 2560 R3
 *    1.0   : Combine winch control and output; Remove Strain Gauge input for stability
 *    1.a   : Test for S-lock with Strain Gauge
 *    2.0   : Re-configure the comm structure
 *    4.0   : Change to only works on Absolute step count for screw drive
 *            Add operation mode handling initialization and cleanup
 *            Restructure comm for strain gauge calibration
*/
//------------------------------------------------------------------------------
// Libraries
//------------------------------------------------------------------------------
#include <ChibiOS_AVR.h>
#include <HX711.h>

//------------------------------------------------------------------------------
// Performance Parameter Definition
//------------------------------------------------------------------------------
const int SLEEP_TIME_IN = 5;                // Scan frequency for Tension Sensor input (ms)
const int SLEEP_TIME_CTRL_WINCH = 10;       // Scan frequency for Winch Motor Controller (ms)  
const int SLEEP_TIME_CTRL_SDRIVE = 20;      // Scan frequency for Screw Drive Control and Output Pulse(ms)
const int SLEEP_TIME_SG = 400;              // Scan frequency for Strain Gauge Input (ms)
const float TENSION_DEADBAND = 10.0;        // Acceptable bias between tension reading and target
const float TENSION_MAX_DEV = 100.0;        // Acceptable max deviation from
const int MAX_ALLOW_STEP = 20000;           // Maximum allowed step (end of screw)
const int MIN_ALLOW_STEP = 0;               // Maximum allowed step (end of screw)
const float MAX_SET_TENSION = 500.0;        // Maximum allowed loading target
const float MIN_SET_TENSION = 10.0;         // Minimum loading (instrument readable)
const int SLEEP_TIME_DEFAULT = 1000;        // Default Sleep time (ms)

//------------------------------------------------------------------------------
// Static Variable Declaration
//------------------------------------------------------------------------------
static int intStepOutput;                   // Target Steps to move (Tension Control <-> Winch Motor Pulse Generator)
static boolean boolStepDirection;           // Target Direction to move (Tension Control <-> Winch Motor Pulse Generator)
static int intStepCount;                    // Absolute Current Step count for the Screw Drive (Screw Drive Control -> Remote Comm)
static int intSetScrew;                     // Absolute Target Step count for the Screw Drive (Remote Comm -> Screw Drive Control)
static int intStepRef;                      // Reference (0-point) Step Count

static char inputString[256] = "";           // Serial Comm - a string to hold incoming data
static int inputPointer;                    // Serial Comm - input store pointer
static boolean stringComplete = false;      // Serial Comm - whether the string is complete

static float flSetTension;                  // Target Tension (Remote Comm -> Tension Control)
static float flTension;                     // Current Tension reading (Tension Input -> Tension Control & Remote Comm)
static boolean boolManOverride;             // Operating Mode, True = Manual, False = Auto;
static boolean boolTIFault;                 // Indication of Tension reading loss

static float flStrain[10];                  // Strain Gauge Readings
static float flStrainScale[10]  = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};    
                                            // Strain Gauge scales (Remote Comm -> SG scaling)
static float flStrainBias[10] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
                                            // Strain Gauge Bias (Remote Comm -> SG scaling)

static boolean boolIsSetZero = false;       // Recalibration flag from Remote

static int intMode;                         // Operating mode
static boolean boolScaleLoaded = false;
static boolean boolBiasLoaded = false;
static boolean boolStepLoaded = false;      // Track last status restore progress
static boolean boolCommDown = false;        // Watchdog flag
static boolean boolIsSetScale = false;      // Flag to Indicate the scale recalibration completion

static unsigned long wdTimeComm = 0;
static unsigned long wdTimer = 0;
static boolean wdDisabled = true;


//------------------------------------------------------------------------------
// Non-volatile variable declaration
//------------------------------------------------------------------------------
boolean boolBtn1sts = false;
boolean boolBtn2sts = false;
boolean boolBtn3sts = false;
boolean boolBtn4sts = false;
boolean boolBtn5sts = false;

//------------------------------------------------------------------------------
// Strain Gauges Declaration
//------------------------------------------------------------------------------
HX711 SG[10] = {HX711(30,40), HX711(31,41),HX711(32,42),HX711(33,43),HX711(34,44),HX711(35,45),HX711(36,46),HX711(37,47),HX711(38,48),HX711(39,49)};

//------------------------------------------------------------------------------
// Process - Mode Monitoring
//------------------------------------------------------------------------------
static THD_WORKING_AREA(waMM,128);

static void thModeMonitor(void *arg){
  while(!chThdShouldTerminateX()){
    // Check mode
    // Mode 0: Move to Mode 1 (idle)
    if (intMode == 0){
      intMode = 1;
    }
    // Mode 1: do nothing (idle), wait for command from RigHW
    // Mode 2: receive stored status from RigHW
    if ((intMode == 2)&&(boolScaleLoaded)&&(boolStepLoaded)&&(boolBiasLoaded)){
      intMode = 3;
    }
    // Mode 3: do nothing (idle), wait for command from RigHW
    // Mode 4: Re-calibrate strain gauges
    // move to mode 5 when calibration done
    if ((intMode == 4) && (boolIsSetScale)){
      intMode = 5;
    }
    // Mode 5: Operation
    // If comm fail (watchdog flagged), move to Mode 5
    if ((intMode == 5) && (boolCommDown)){
      intMode = 6;
    }
    // Mode 6: clean up
    // Return to idle mode when cleanup complete
    if (intMode == 6){
      flSetTension = MIN_SET_TENSION;
      intSetScrew = intStepRef;
      if (intStepCount == intStepRef){
        intMode = 1;
          boolIsSetScale = false;
          wdDisabled = true;
          boolCommDown = false;
          boolScaleLoaded = false;
          boolStepLoaded = false;
          boolBiasLoaded = false; 
      }
    }
    // Sleep to next cycle
    chThdSleepMilliseconds(100);
  }
}
//------------------------------------------------------------------------------
// Process - Pin Monitoring
//------------------------------------------------------------------------------
static THD_WORKING_AREA(waPM,128);

static void thPinMonitor(void *arg){
  while (!chThdShouldTerminateX()) {
    
    // Check Manual Override Status
    // rising edge detection - Manual Override button
    if (boolBtn1sts != digitalRead(26)){
      if (digitalRead(26)){
        // when rising edge detected,toggle Manual Override status 
        if (!boolManOverride){
          boolManOverride = true;                    // Enable Manual Override Status
        }else{
          boolManOverride = false;                   // Disable Manual Override Status, recalibrate screw drive status
          intStepRef = intStepCount;                 // Calibration: reset the zero point for the screw drive
          intSetScrew = intStepCount;                // Calibration: reset the zero point for the screw drive
        }   
      }
     boolBtn1sts = digitalRead(26); 
    }
    
    // Remote recalibration command handling
    if (boolIsSetZero){                             // Recalibration triggered from remote       
      intStepRef = intStepCount;                    // Calibration: reset the zero point for the screw drive
      intSetScrew = intStepCount;                   // Calibration: reset the zero point for the screw drive
      boolIsSetZero = false;                        // Reset recalibration flag       
    }
    digitalWrite(27,boolManOverride);               // Update local indication.

    // Sleep to next cycle
    chThdSleepMilliseconds(200);
  }
}

//------------------------------------------------------------------------------
// Process - Tension Input
//------------------------------------------------------------------------------
static THD_WORKING_AREA(waTI,256);
static void thTensionIn(void *arg){
  while (!chThdShouldTerminateX()) {
    
    // Read data from the DATA pin

    /* Simulation code
     *  to simulate the tension response
     */
    
    // Sleep to next cycle
    chThdSleepMilliseconds(SLEEP_TIME_IN);
  } 
}
//------------------------------------------------------------------------------
// Process - Strain Gauge Input
//------------------------------------------------------------------------------
static THD_WORKING_AREA(waSG,512);

static void thStrainIn(void *arg){
  while (!chThdShouldTerminateX()) {
    // Calibrate strain gauge bridge when receive command
    if (intMode == 4){
      for (int i=0;i<10;i++){
        SG[i].set_scale(flStrainScale[i]);
        SG[i].set_offset(flStrainBias[i]);
      }
      boolIsSetScale = true;
    }
    if (intMode == 5){
      // Reading data
      for (int addr=0;addr<10;addr++){
        if (SG[addr].is_ready()){
          // Read data from the DATA pin
          flStrain[addr] = SG[addr].get_units(1);
        }
      }
    }
    // Sleep to next cycle
    chThdSleepMilliseconds(SLEEP_TIME_SG);
  } 
}

//------------------------------------------------------------------------------
// Process - Winch Motor Control
//------------------------------------------------------------------------------
static THD_WORKING_AREA(waWMC,256);
static void thWinchCtrl(void *arg){
    while (!chThdShouldTerminateX()) {
      if ((intMode == 5)||(intMode == 6)){
        // Check whether Manual Override in place
        if (boolManOverride) {
          //Manual Mode: Direction judged based on input, 
          //             motor run predefined number of step per cycle when run button is triggered every time
          // Rising edge detection - Backward button (tighten)
          if (boolBtn3sts != digitalRead(25)){
            if (digitalRead(25)){          //Rising edge                
              intStepOutput = 49;
              boolStepDirection = false;
            }
            boolBtn3sts = digitalRead(25);
          }
          // Rising edge detection - Forward button (release)
          if (boolBtn2sts != digitalRead(24)){
            if (digitalRead(24)){          //Rising edge
              intStepOutput = 49;
              boolStepDirection = true;                
            }
            boolBtn2sts = digitalRead(24);
          }
        }else if(!boolTIFault){
          // Automatic mode: Calculate the output pulse and direction required in this cycle
          if ((flTension > (flSetTension + TENSION_DEADBAND)) || (flTension < (flSetTension - TENSION_DEADBAND))){
            if (flTension > flSetTension){
              boolStepDirection = true;          
            }else {
              boolStepDirection = false;
            }
            intStepOutput = funcStepCalc();
          }
        }
        digitalWrite(53,boolStepDirection);
        for (int i =0; i<intStepOutput;i++){
          digitalWrite(52,HIGH);           
          delayMicroseconds(2);
          digitalWrite(52,LOW);
          delayMicroseconds(500);
        }
        intStepOutput = 0;
      }
    // Sleep to next cycle
    chThdSleepMilliseconds(SLEEP_TIME_CTRL_WINCH);
  } 
}

//------------------------------------------------------------------------------
// Process - Screw Drive Motor Control
//------------------------------------------------------------------------------
static THD_WORKING_AREA(waSDC,256);
static void thSDCtrl(void *arg){
    while (!chThdShouldTerminateX()) {
      if ((intMode == 5)||(intMode == 6)){
        // Check whether Manual Override in place
        if (boolManOverride) {
          //Manual Mode: Direction judged based on input,
          //             motor run predefined 1 step per cycle when run button is triggered once
          // Rising edge detection - Backward button
          if (boolBtn5sts != digitalRead(23)){
            if (digitalRead(23)){          //Rising edge                
              digitalWrite(51,HIGH);
              digitalWrite(50,HIGH);           
              delayMicroseconds(2);
              digitalWrite(50,LOW);
              intStepCount = intStepCount - 1;
            }
            boolBtn5sts = digitalRead(23);
          }
          // Rising edge detection - Forward button
          if (boolBtn4sts != digitalRead(22)){
            if (digitalRead(22)){          //Rising edge
              digitalWrite(51,LOW);
              digitalWrite(50,HIGH);           
              delayMicroseconds(2);
              digitalWrite(50,LOW);
              intStepCount = intStepCount + 1;               
            }
            boolBtn4sts = digitalRead(22);
          }
        }else if (!boolTIFault){
          //Auto Mode: Move to the target step with 1 step per cycle, when tension is within limit
          if((flTension <= flSetTension + TENSION_MAX_DEV) && (flTension >= flSetTension - TENSION_MAX_DEV)){
  
            if (intStepCount < intSetScrew){
              digitalWrite(51,LOW);
              digitalWrite(50,HIGH);           
              delayMicroseconds(2);
              digitalWrite(50,LOW);
              intStepCount = intStepCount + 1;
            }else if (intStepCount > intSetScrew){
              digitalWrite(51,HIGH);
              digitalWrite(50,HIGH);           
              delayMicroseconds(2);
              digitalWrite(50,LOW);
              intStepCount = intStepCount - 1;
            }         
          }
        }
      }
    // Sleep to next cycle
    chThdSleepMilliseconds(SLEEP_TIME_CTRL_SDRIVE);
  } 
}
//------------------------------------------------------------------------------
// Process - Serial Comm Scan
//------------------------------------------------------------------------------
static THD_WORKING_AREA(waComm,1536);
static void thCommScan(void *arg){
  while (!chThdShouldTerminateX()) {
    if (serialEventRun){
      serialEventRun();
    }
    chThdSleepMilliseconds(10);
  }
}
//------------------------------------------------------------------------------
// Process - Watchdog
//------------------------------------------------------------------------------
static THD_WORKING_AREA(waWDog,256);
static void thWatchDog(void *arg){
  while (!chThdShouldTerminateX()) {
    if (!wdDisabled){
      wdTimer = millis() - wdTimeComm;
      if (wdTimer > 30000){
        boolCommDown = true;
      }
    }
    chThdSleepMilliseconds(1500);
  }
}

//------------------------------------------------------------------------------
// Main Thread  - that generate other threads
//------------------------------------------------------------------------------
void chMain() {  
      chThdCreateStatic(waPM, sizeof(waPM),NORMALPRIO+30, thPinMonitor, NULL);
      chThdCreateStatic(waMM, sizeof(waMM),NORMALPRIO+30, thModeMonitor, NULL);
      chThdCreateStatic(waTI, sizeof(waTI),NORMALPRIO+10, thTensionIn, NULL);
      chThdCreateStatic(waWMC, sizeof(waWMC),NORMALPRIO+15, thWinchCtrl, NULL);
      chThdCreateStatic(waSDC, sizeof(waSDC),NORMALPRIO+5, thSDCtrl, NULL);
      chThdCreateStatic(waComm, sizeof(waComm),NORMALPRIO+40, thCommScan, NULL);
      chThdCreateStatic(waWDog, sizeof(waWDog),NORMALPRIO+2, thPinMonitor, NULL);
      chThdCreateStatic(waSG, sizeof(waSG),NORMALPRIO+1, thStrainIn, (void *)0);
      chThdSleepMilliseconds(10);
}


//------------------------------------------------------------------------------
// Setup the system
//------------------------------------------------------------------------------
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
  pinMode(30,INPUT);                   // Pin 30: Strain Gauge 1, DATA channel
  pinMode(31,INPUT);                   // Pin 31: Strain Gauge 2, DATA channel
  pinMode(32,INPUT);                   // Pin 32: Strain Gauge 3, DATA channel
  pinMode(33,INPUT);                   // Pin 33: Strain Gauge 4, DATA channel
  pinMode(34,INPUT);                   // Pin 34: Strain Gauge 5, DATA channel
  pinMode(35,INPUT);                   // Pin 35: Strain Gauge 6, DATA channel
  pinMode(36,INPUT);                   // Pin 36: Strain Gauge 7, DATA channel
  pinMode(37,INPUT);                   // Pin 37: Strain Gauge 8, DATA channel
  pinMode(38,INPUT);                   // Pin 38: Strain Gauge 9, DATA channel
  pinMode(39,INPUT);                   // Pin 39: Strain Gauge 10, DATA channel
  
  pinMode(40,OUTPUT);                  // Pin 40: Strain Gauge 1, CLK signal
  digitalWrite(40,LOW);
  pinMode(41,OUTPUT);                  // Pin 41: Strain Gauge 2, CLK signal
  digitalWrite(41,LOW);
  pinMode(42,OUTPUT);                  // Pin 42: Strain Gauge 3, CLK signal
  digitalWrite(42,LOW);
  pinMode(43,OUTPUT);                  // Pin 43: Strain Gauge 4, CLK signal
  digitalWrite(43,LOW);
  pinMode(44,OUTPUT);                  // Pin 44: Strain Gauge 5, CLK signal
  digitalWrite(44,LOW);
  pinMode(45,OUTPUT);                  // Pin 45: Strain Gauge 6, CLK signal
  digitalWrite(45,LOW);
  pinMode(46,OUTPUT);                  // Pin 46: Strain Gauge 7, CLK signal
  digitalWrite(46,LOW);
  pinMode(47,OUTPUT);                  // Pin 47: Strain Gauge 8, CLK signal
  digitalWrite(47,LOW);
  pinMode(48,OUTPUT);                  // Pin 48: Strain Gauge 9, CLK signal
  digitalWrite(48,LOW);
  pinMode(49,OUTPUT);                  // Pin 49: Strain Gauge 10, CLK signal
  digitalWrite(49,LOW);
 
  // initialize pins - Manual Override
  pinMode(22,INPUT);                   // Pin 22: Manual Override, SD motor forward
  pinMode(23,INPUT);                   // Pin 23: Manual Override, SD motor backward    
  pinMode(24,INPUT);                   // Pin 24: Manual Override, Winch motor forward
  pinMode(25,INPUT);                   // Pin 25: Manual Override, Winch motor backward
  pinMode(26,INPUT);                   // Pin 26: Manual Override Status
  pinMode(27,OUTPUT);                  // Pin 27: Manula Override Status Local Indication
  digitalWrite(27,LOW);

  // initialize variables
  intMode = 0;

  boolIsSetScale = false;
  wdDisabled = true;
  boolCommDown = false;
  boolScaleLoaded = false;
  boolStepLoaded = false;
  boolBiasLoaded = false; 

  // initialize strain gauges
  for (int i=0;i<10;i++){
    SG[i].read();
    SG[i].tare();
  }

  // Now start the main operating process
  chBegin(chMain);
  while(1) {  }
}


//------------------------------------------------------------------------------
// Main loop (no longer used)
//------------------------------------------------------------------------------
void loop() {
}

//------------------------------------------------------------------------------
// Sub-function: Winch Motor Step Calculation
//------------------------------------------------------------------------------
static int funcStepCalc(){
  float flError;
  if (flTension > flSetTension){
    flError = flTension - flSetTension;
  }else{
    flError = flSetTension - flTension;
  }
  if (flError >=TENSION_MAX_DEV){
    return 20;
  }else{
    return (int)(flError/TENSION_MAX_DEV *20);
  }
}

// ----------------------------------------------------------------------------------------------------
// Comm Process - Serial event Interpretation
// ----------------------------------------------------------------------------------------------------
  /*
   *  Serial command processing
   *  Detection flag: stringComplete
   *    - Interpret Command in several cases
   *      - to Drive output: 
   *      - to Provide value
   *  Command Structure
   *    - command format: rlab://xxxx?addr=xx&val=xxxxx
   *    - Valid Header: rlab://
   *    - Command type (char 7 to 10) : REQV (retrive reading from Arduino); 
   *                                    SETV (set target value)
   *  REQV command
   *    - No address used 
   *    - Data Segments Sequence : 01 = tension reading; 02 = tension target;
   *                               03 = SD current step; 04 = step target
   *                               05 = Max step; 06 = Min step; 07 = Max Tension; 08 = Min tension;                           
   *                               09 = Manual Override Status; 10 = Tension Input Fault Status;
   *                               11-20 = Strain readings;
   *                               21 = mode;22 = watchdog
   *                               
   *  SETV command
   *    - Address (char 17 to 18): 01 = tension target; 02 = step target; 03 = current step (for last data recovery) 
   *                               04 = target mode
   *                               99 = recalibration 
   *    - value (char 24 to end)
   *  SECA command
   *    - no address used
   *    - Data Segments Sequence: 01-10: Strain Scale; 11-20:Strain Bias
   *  RECA command
   *    - no address used
   *    - Retrieve scale and bias info for last state storage
   */
void serialEvent() {
    while (Serial.available() && !stringComplete) {  
      // get the new byte:
      char inChar = (char)Serial.read();
      // if the incoming character is a newline, set a flag
      // so the main loop can do something about it:
      if ((inChar == '\n')||(inputPointer == 255)) {
        stringComplete = true;
        inputPointer = 0;
      }else if (inputPointer < 255){
      // allocate to the inputString buffer
      // move pointer to next location
        inputString[inputPointer] = inChar;
        inputPointer++;
               Serial.println(inputString);
               Serial.println(inputPointer);
      }
    }
    
    // *************************
    // Interpret input
    if (stringComplete) {
        char msgHeader[8] = "";
        char msgReq[5] = "";
        char msgAddr[3] = "";
        char msgVal[233] = "";
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
        for (int i = 0; i<233;i++){
          if ((inputString[i+24] < '+') || (inputString[i+24] > 'z') ||((inputString[i+24] > ';')&&(inputString[i+24] < 'A'))||(inputString[i+24] == '/')||((inputString[i+24] > 'Z')&&(inputString[i+24] < 'a'))){
            break; 
          }
          msgVal[i] = inputString[i+24];
        }
        
        int intAddr = atoi(msgAddr);
        float flVal = atof(msgVal);
        if(strcmp(msgHeader,"rlab://") != 0) {                                           
          Serial.println("Err-input");
          Serial.flush();
        }else{
          if (strcmp(msgReq,"REQV") == 0){
            funcRequestValue(intAddr);
            Serial.flush();
          }else if (strcmp(msgReq,"RECA") == 0){
            funcRequestCali(intAddr);
            Serial.flush();  
          }else if (strcmp(msgReq,"SETV") == 0){
            funcSetValue(intAddr,flVal);
            Serial.flush();
          }else if (strcmp(msgReq,"SECA") == 0){
            funcGetLast(intAddr,msgVal);
            Serial.flush();
          }else{
            Serial.println("Err-command");
            Serial.flush();
          }              
        }
        for (int i=0;i<255;i++){
          inputString[i] = '\0';
        }
        stringComplete = false;
        // Watchdog
        if (wdDisabled == true){
          wdDisabled = false;
        }
        wdTimeComm = millis();
      }
}


// ----------------------------------------------------------------------------------------------------
// Comm Process - Request to read value
// ----------------------------------------------------------------------------------------------------
void funcRequestValue(int intAddr){
  switch(intAddr){
    case 1:
      Serial.print("Cpl-");
      Serial.print(flTension,2);
      Serial.print(";");
      Serial.print(flSetTension,2);
      Serial.print(";");
      Serial.print(intStepCount);
      Serial.print(";");
      Serial.print(intSetScrew);
      Serial.print(";");    
      Serial.print(MAX_ALLOW_STEP);
      Serial.print(";");
      Serial.print(MIN_ALLOW_STEP);
      Serial.print(";");
      Serial.print(MAX_SET_TENSION,2);
      Serial.print(";");
      Serial.print(MIN_SET_TENSION,2);
      Serial.print(";");
      Serial.print(boolManOverride);
      Serial.print(";");
      Serial.println(boolTIFault);
      break;
    case 2:
      for (int i = 0; i<9;i++){
        Serial.print(flStrain[i],2);
        Serial.print(";");  
      }
      Serial.println(flStrain[9],2);
      break;
    case 3:
      Serial.print(intMode);
      Serial.print(";");
      Serial.println(millis());
      break;
    default:
      Serial.println("Err-addr");   
  }
}


// ----------------------------------------------------------------------------------------------------
// Comm Process - Request to read calibration value
// ----------------------------------------------------------------------------------------------------
void funcRequestCali(int intAddr){
  switch(intAddr){
    case 1:
     Serial.print("Cpl-");
     for (int i = 0; i<9;i++){
       Serial.print(flStrainScale[i],2);
       Serial.print(";");  
     }
     Serial.println(flStrainScale[9],2);
     break;
    case 2:
      Serial.print("Cpl-");
      for (int i = 0; i<9;i++){
        Serial.print(flStrainBias[i],2);
        Serial.print(";");  
      }
      Serial.println(flStrainBias[9],2);
      break;
    default:
      Serial.println("Err-addr");
  }
}

// ----------------------------------------------------------------------------------------------------
// Comm Process - Request to set value
// ----------------------------------------------------------------------------------------------------
void funcSetValue(int intAddr, float Val){
  int intVal=(int)Val;
  switch(intAddr){
    case 1:
      flSetTension = min(max(Val,MIN_SET_TENSION),MAX_SET_TENSION);
      Serial.println(flSetTension);
      Serial.println("Cpl");
      break;
    case 2:
      intSetScrew = min(max(intVal,MIN_ALLOW_STEP),MAX_ALLOW_STEP);
      Serial.println("Cpl");
      break;
    case 3:
      intMode = intVal;
      Serial.println("Cpl");
      break;
    case 99:
      boolIsSetZero = true;
      Serial.println("Cpl");
      break;
    default:
      Serial.println("Err-addr");  
  }
}

// ----------------------------------------------------------------------------------------------------
// Comm Process - Request to recover last settings
// ----------------------------------------------------------------------------------------------------
void funcGetLast(int intAddr,char* msg){
  switch(intAddr){
    case 1:
      intStepCount = atoi(strtok(msg,";"));
      boolStepLoaded = true;
      Serial.println("Cpl");
      break;
    case 2:
      flStrainScale[0] = atof(strtok(msg,";"));
      for (int i = 1; i<10; i++){
         flStrainScale[i] = atof(strtok(NULL,";"));
      }
      boolScaleLoaded = true;
      Serial.println("Cpl");
      break;
    case 3: 
      flStrainBias[0] = atof(strtok(msg,";"));
      for (int j = 1; j<10; j++){
         flStrainBias[j] =atof(strtok(NULL,";"));
      }
      boolBiasLoaded = true;
      Serial.println("Cpl");
      break;
    default:
      Serial.println("Err-addr");
  }      
}

