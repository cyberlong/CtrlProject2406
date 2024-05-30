#include <HardwareSerial.h>


// Motor Pins
#define MOT1P 15
#define MOT1N 18
#define MOT2P 4
#define MOT2N 5
#define MOT1  0
#define MOT2  1

// PWM values
#define PWMfreq    5000
#define PWMresolution 8
#define PWM_M1_A      0
#define PWM_M1_B      1
#define PWM_M2_A      2
#define PWM_M2_B      3
#define maxPWM      255

// Pins of the encoders
#define CodA 14
#define CodB 27
#define CodC 26
#define CodD 25

// Pins of the Limit Switches
#define LS1  12
#define LS2  19

// UART Pins
#define rx2  16
#define tx2  17

// Motor Control
bool MOT1D = 0;     // Direction
bool MOT2D = 1;
uint8_t MOT1S = 0;  // PWM Signal
uint8_t MOT2S = 0;
bool MOT1O = 0;     // On/Off
bool MOT2O = 0;
volatile float MOT1CS = 0;     // Control Signal
volatile float MOT2CS = 0;

// Setup Control Parameters and variables
#define Clk 1e6 // 1Mhz
#define ms 1000
#define Tm 30   // ms
#define PPT 60.0                  // Pulses per turn
#define mmPT 12.0                 // Milimeter per turn
#define RPS2RPM 60                // rev per sec to rev per minute
#define RPS2RadPs 2 * 3.141592    // rev per sec to rad per sec
#define alph 0.2                  // Alpha for EMA
float toCTRL = 0;
uint8_t PWM = 0;
uint8_t codPins[][2] = {{CodA, CodB},{CodC, CodD}}; // encoder pins
volatile int encoderValue[]   = {0,0};              // the current value
volatile uint32_t last_time[] = {0,0};              // for debouncing the encoder
volatile float EMAprev[]      = {0,0};              // for EMAprev
volatile float linearValue[]  = {0,0};              // Axis linear value
float setPoint[]     = {0,0};                       // The setpoints
uint32_t staticTime[]= {0,0};                       // stabilize chepoint
volatile bool TmVis = false;

// PID params and Vars
#define PID_Kp 0.5
#define PID_Ki 3.0
#define PID_Kd 0.03
float PID_Emem = 0;
float PID_Imem = 0;

// System Objects
hw_timer_t* timer = NULL;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
HardwareSerial daServer(2);

// UART vars
const byte numChars = 64; // Max Parsing chars
char iStr[numChars]; // Message buffer
bool thereIsInput = false;
bool recvInProgress = false;
bool requestSP = false;
float newSP[] = {0,0};
bool  isWiFiOk = false;
float scrPos[] = {0,0};
char* ptr;
char* strings[10];


//    <===========================Asyncronous Functions==============================>

void IRAM_ATTR Measure(){
  // code :D
  linearValue[MOT1] = EMA(updtLinearPos(encoderValue[MOT1]), &EMAprev[MOT1]);
  linearValue[MOT2] = EMA(updtLinearPos(encoderValue[MOT2]), &EMAprev[MOT2]);

  PID(50, linearValue[MOT1], &MOT1CS);
  // PID(0, linearValue[MOT2], &MOT2CS);
  // CSMotCtrl(MOT2CS,MOT2,1);

  TmVis = !TmVis;
  digitalWrite(2,TmVis);
}


float updtLinearPos(int encVal){
  return (encVal/PPT) * mmPT;
}

void IRAM_ATTR setXas0(){
  setZero(&encoderValue[MOT1], &linearValue[MOT1], &EMAprev[MOT1]);
}

void IRAM_ATTR setYas0(){
  setZero(&encoderValue[MOT2], &linearValue[MOT2], &EMAprev[MOT2]);
}

void setZero(volatile int* encVal, volatile float* linVal, volatile float* EMAval){
  *encVal = 0;
  *linVal = 0;
  *EMAval = 0;
}

//      <<<=|Encoder Interrupts|=>>>
void IRAM_ATTR updateEncoderA(){
    // Serial.print("updateEncoderA running on core ");
    // Serial.println(xPortGetCoreID());
    portENTER_CRITICAL_ISR(&mux);
    UpDtEnc(&last_time[MOT1], &encoderValue[MOT1],codPins[MOT1]);
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR updateEncoderB(){
    // Serial.print("updateEncoderB running on core ");
    // Serial.println(xPortGetCoreID());
    portENTER_CRITICAL_ISR(&mux);
    UpDtEnc(&last_time[MOT2], &encoderValue[MOT2],codPins[MOT2]);
    portEXIT_CRITICAL_ISR(&mux);
}

//      <<<=|Update encoder Value Function|=>>>
void UpDtEnc(volatile uint32_t* Ltiem, volatile int* encVal, uint8_t pins[]){
  if ((millis() - (*Ltiem)) < 10)  // debounce time is 10ms
    // If it's called before the cooldown period, it doesn't update
    return;
  
  int MSB = digitalRead(pins[0]); //MSB = most significant bit
  int LSB = digitalRead(pins[1]); //LSB = least significant bit
  
  // This represents the current encoder state
  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  // And with this we unify with the previous value
  int sum  = (*Ltiem << 2) | encoded; //adding it to the previous encoded value
  // Then we compare on a lookup table and add or substract from the encoder
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) (*encVal) ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) (*encVal) --;

  *Ltiem = encoded; //store this value for next time
}


//    <===========================SETUP==============================>

void setup() {
  Serial.begin(9600);
  Serial.println("Starting Setup");
  // Start Serial port with Server from UART
  daServer.begin(9600,SERIAL_8N1,rx2,tx2);
  // Timer at Clk hz, 1 Mhz is used to have 1us steps
  timer = timerBegin(Clk);
  // interrupt using the address of the ISR
  timerAttachInterrupt(timer, &Measure);
  // Configure the pins as needed
  ConfigurePins();
  // Just some time bc why not?
  delay(2000);
  // start axes at 0
  calibrate();
  // call an interrupt every n us, 
  // our Tm is 30 ms so that's that
  timerAlarm(timer,Tm*ms,true,0);
  // end
  Serial.println("Finished Setup");
}

void calibrate(){
  try {
    Serial.println("Calibrating X");
    bool a = digitalRead(LS1);
    SingleMOTCtrl(MOT1, maxPWM, 1, 0);
    while(a){
      a = digitalRead(LS1);
      // Serial.println(a);
      delay(20);
    } SingleMOTCtrl(MOT1, 0, 0, 1);
  } catch(...){
    Serial.println("Error on Cali X");
  }

  try {
    Serial.println("Calibrating Y");
    bool b = digitalRead(LS2);
    SingleMOTCtrl(MOT2, maxPWM, 1, 0);
    while(b){
      b = digitalRead(LS2);
      // Serial.println(b);
      delay(20);
    } SingleMOTCtrl(MOT2, 0, 0, 1);
  } catch(...){
    Serial.println("Error on Cali Y");
  }

  Serial.println("Ending Calibration");
}


//    <===========================LOOPING==============================>

void loop() {
  // serialEvent();

  if (thereIsInput && !recvInProgress){
    // serialAssign();
    // do smth
    // Serial.println(iStr);
    
    // // reset
    // serialAssign();
    // iStrClean();
    thereIsInput = false;
  }

  ///


  ///

  CSMotCtrl(MOT1CS,MOT1,1);

  // SPHandler(&setPoint[MOT1],linearValue[MOT1],&staticTime[MOT1]);
  // SPHandler(&setPoint[MOT2],linearValue[MOT2],&staticTime[MOT2]);
  // serialInform();

  // Debug
  // db_LinearLCDprint();
  db_values();
  db_stats();

  delay(100);
}

void iStrClean(){
  for (int i = 0 ; i < numChars ; i++){
    iStr[i] = '\0';
  }
}

char* serialEvent() {
  char daStr[64];
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc; // char buffer
 
  while (daServer.available() > 0 && thereIsInput == false) {
    rc = daServer.read();

    if (recvInProgress == true) {       // while the end marker isn't sent
      if (rc != endMarker) {          // it accumulates the recieved chars
        daStr[ndx] = rc;    // in the Message buffer
        ndx++;
        if (ndx >= numChars) {
            ndx = numChars - 1;
        }
      }
      else {
        daStr[ndx] = '\0';  // if the end marker is sent
        recvInProgress = false;     // ends the string
        ndx = 0;                    // and the new data flag
        thereIsInput = true;             // is sent to true
      }
    } else if (rc == startMarker) { // if the starter marker is sent
        recvInProgress = true;    // receiving process starts
    }
  }
  return daStr;
}

  // Str format: "a:112,b:124,c:e\n";
  // We finish a message with '\n' from println();
  // Every bit of information we separate with a comma ',';
  // This is basically a CSV RipOff.
void serialInform(){
  String forward = "";
  // forward += S_ReqSP + ':'; // Request a Setpoint
  forward += '<';
  forward += requestSP; 
  forward += '>';

  daServer.println(forward);

  if (0) Serial.println(forward);
}

void serialAssign(){
  byte index = 0;
  ptr = strtok(iStr, ",");  // delimiter
  while (ptr != NULL)
  {
     strings[index] = ptr;
     index++;
     ptr = strtok(NULL, ",");
  }

  newSP[0]  = atof(strings[0]);
  newSP[1]  = atof(strings[1]);
  isWiFiOk  = atoi(strings[2]);
  scrPos[0] = atof(strings[3]);
  scrPos[1] = atof(strings[4]);
}

/* It handles the Setpoints */
void SPHandler(float* currentSP, volatile float PV, uint32_t* StaTime){
  if( !(*currentSP + 1 > PV && PV > *currentSP - 1) ){ // if outside
    *StaTime = millis();
  }
  
  if( (*StaTime - millis()) >= 1000 ){
    // Update SetPoint
    *currentSP = 50; //test
  }
}

void CSMotCtrl(float CS, bool MOT, bool isOn){
  try {
    uint8_t daPWM  = abs(CS);
    bool direction = CS > 0 ? 1 : 0;
    SingleMOTCtrl(MOT, daPWM, isOn, direction);
  } catch(...){
    Serial.println("Error on CS");
  }
}

void PID (float SeP, float PV, volatile float* CtrlVar){
  try {
    float En = SeP - PV;
    float csP = PID_Kp*En;
    float csD = PID_Kd*(En - PID_Emem);
    float Eni = En + PID_Imem;
    float csI = PID_Ki*Eni;
    float cs = csP + csI + csD;
    if (cs >  100) cs =  100.0;
    if (cs < -100) cs = -100.0;
    *CtrlVar = 255.0 * cs/100.0;
    PID_Emem = En;
    PID_Imem = Eni;
  } catch(...){
    Serial.println("Error on PID");
  }
}

float EMA(float current,volatile float *EMApv){
  float daNew = alph*current + (1-alph)* (*EMApv); // The equation
  *EMApv = daNew; // Updates the Prev value
  return daNew;
}


//    <===========================PERIPHERALS==============================>

void SingleMOTCtrl(bool MOT , uint8_t PWM, bool state, bool direction){
  analogWrite(!MOT * MOT1P + MOT * MOT2P, state *  direction * PWM); // positive
  analogWrite(!MOT * MOT1N + MOT * MOT2N, state * !direction * PWM); // negative
}

void ConfigurePins(){
  // mot pins
  int channel[] = {PWM_M1_A, PWM_M1_B, PWM_M2_A, PWM_M2_B};
  int outputs[] = {MOT1P   , MOT1N   , MOT2P   , MOT2N};
  for (int i = 0; i < 4; i++){ // asigning exits
      // PWM setup
    // ledcSetup(channel[i], PWMfreq, PWMresolution);
    ledcAttach(outputs[i],PWMfreq,PWMresolution);
    pinMode(outputs[i],OUTPUT);
  } pinMode(2,3); // Debugging purposes

    // Prepare the pins and interrupts of the encoders
  pinMode(CodA, INPUT_PULLUP); // Turn pullup resistor on
  pinMode(CodB, INPUT_PULLUP); // Turn pullup resistor on
  pinMode(CodC, INPUT_PULLUP); // Turn pullup resistor on
  pinMode(CodD, INPUT_PULLUP); // Turn pullup resistor on
  attachInterrupt(digitalPinToInterrupt(CodC), updateEncoderB, CHANGE);  // Interrupt
  attachInterrupt(digitalPinToInterrupt(CodD), updateEncoderB, CHANGE);  // Interrupt
  attachInterrupt(digitalPinToInterrupt(CodA), updateEncoderA, CHANGE);  // Interrupt
  attachInterrupt(digitalPinToInterrupt(CodB), updateEncoderA, CHANGE);  // Interrupt
  // Prepare the pins and interrupts of the limit switches
  attachInterrupt(LS1, setXas0, FALLING);  // Interrupt
  attachInterrupt(LS2, setYas0, FALLING);  // Interrupt
  pinMode(LS1, INPUT_PULLUP); // Turn pullup resistor on
  pinMode(LS2, INPUT_PULLUP); // Turn pullup resistor on
}


//    <===========================DEBUG==============================>

void db_AxisLCDprint(){
  String a = "";
  a += "X:";
  a += encoderValue[MOT1];
  a += ",Y:";
  a += encoderValue[MOT2];
  Serial.println(a);
}

void db_LinearLCDprint(){
  String a = "";
  a += "X: ";  
  a += linearValue[MOT1];
  a += ",Y:";
  a += linearValue[MOT2];
  Serial.println(a);
}

void db_values(){
  String a = "";
  a += "Next SP: [";
  a += String(newSP[0]);
  a += ", ";
  a += String(newSP[1]);
  a += "]   ; Screen pos: [";
  a += String(scrPos[0]);
  a += ", ";
  a += String(scrPos[1]);
  a += "]   ; WiFi condition: ";
  a += isWiFiOk;

  Serial.println(a);
}

void db_stats(){
  try {
    String a = "";
    a += "MOT1:";
    a += String(linearValue[MOT1]);
    a += ",CS1:";
    a += String(MOT1CS);
    a += ",MOT2:";
    a += String(linearValue[MOT2]);
    a += ",CS2:";
    a += String(MOT2CS);

    Serial.println(a);
  } catch(...){
    Serial.println("Error in Db");
  }
}
