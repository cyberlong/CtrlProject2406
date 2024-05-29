#include <HardwareSerial.h>


// Motor Pins
#define MOT1P 18
#define MOT1N 19
#define MOT2P 16
#define MOT2N 17
#define MOT1  0
#define MOT2  1

// Pins of the encoders
#define CodA 25
#define CodB 26
#define CodC 32
#define CodD 33

// Pins of the Limit Switches
#define LS1  27
#define LS2  35

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
volatile int MOT1CS = 0;     // Control Signal
volatile int MOT2CS = 0;

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

// PID params and Vars
#define PID_Kp 1
#define PID_Ki 2
#define PID_Kd 3
float PID_Emem = 0;
float PID_Imem = 0;

// System Objects
hw_timer_t* timer = NULL;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
HardwareSerial daServer(2);

// UART vars
String iStr = "";
bool thereIsInput = false;
bool requestSP = false;
char *ptr = NULL;
char *strings[8];
float newSP[] = {0,0};
bool  isWiFiOk = false;
char  ssid[20];
char  ipaddress[20];
float scrPos[] = {0,0};

//    <===========================Asyncronous Functions==============================>

void IRAM_ATTR Measure(){
  // code :D
  linearValue[MOT1] = EMA(updtLinearPos(encoderValue[MOT1]), &EMAprev[MOT1]);
  linearValue[MOT2] = EMA(updtLinearPos(encoderValue[MOT2]), &EMAprev[MOT2]);

  // PID(0, linearValue[MOT1], &MOT1CS);
  // PID(0, linearValue[MOT2], &MOT2CS);
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
  daServer.begin(9600);
  // Timer at Clk hz, 1 Mhz is used to have 1us steps
  timer = timerBegin(Clk);
  // interrupt using the address of the ISR
  timerAttachInterrupt(timer, &Measure);
  // Configure the pins as needed
  ConfigurePins();
  // Just some time bc why not?
  delay(2000);
  // call an interrupt every n us, 
  // our Tm is 30 ms so that's that
  timerAlarm(timer,Tm*ms,true,0);
  Serial.println("Finished Setup");
}


//    <===========================LOOPING==============================>

void loop() {

  serialEvent();

  if (thereIsInput){
    serialAssign();
    // do smth
    
    
    // reset
    iStr = "";
    thereIsInput = false;
  }

  SPHandler(&setPoint[MOT1],linearValue[MOT1],&staticTime[MOT1]);
  SPHandler(&setPoint[MOT2],linearValue[MOT2],&staticTime[MOT2]);

  CSMotCtrl(setPoint[MOT1],MOT1,1);
  CSMotCtrl(setPoint[MOT2],MOT2,1);

  serialInform();

  // Debug
  db_LinearLCDprint();

  delay(5);
}

void serialEvent(){
  while (daServer.available() > 0){
    char c = (char) daServer.read();
    if (c != '\n'){
      iStr += c;
    } else {
      iStr.trim();
      thereIsInput = true;
    }
  }
}

  // Str format: "a:112,b:124,c:e\n";
  // We finish a message with '\n' from println();
  // Every bit of information we separate with a comma ',';
  // This is basically a CSV RipOff.
void serialInform(){
  String forward = "";
  // forward += S_ReqSP + ':'; // Request a Setpoint
  forward += requestSP; 
  daServer.println(forward);

  if (0) Serial.println(forward);
}

void serialAssign(){
  byte index = 0;
  char buffer[sizeof(iStr)];
  iStr.toCharArray(buffer, iStr.length()+1);
  ptr = strtok(buffer, ",");  // delimiter
  while (ptr != NULL)
  {
     strings[index] = ptr;
     index++;
     ptr = strtok(NULL, ",");
  }

  newSP[0]  = atof(strings[0]);
  newSP[1]  = atof(strings[1]);
  isWiFiOk  = atoi(strings[2]);
  strncpy(ssid, strings[3],20);
  strncpy(ipaddress,strings[4],20);
  scrPos[0] = atof(strings[5]);
  scrPos[1] = atof(strings[6]);
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
  uint8_t daPWM  = abs(CS);
  bool direction = CS > 0 ? 1 : 0;
  SingleMOTCtrl(MOT, daPWM, isOn, direction);
}

void PID (float SeP, float PV, volatile int* CtrlVar){
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
}

float EMA(float current,volatile float *EMApv){
  float daNew = alph*current + (1-alph)* (*EMApv); // The equation
  *EMApv = daNew; // Updates the Prev value
  return daNew;
}


//    <===========================PERIPHERALS==============================>

void SingleMOTCtrl(bool MOT , uint8_t PWM, bool state, bool direction){
  analogWrite(MOT * MOT1P + !MOT * MOT2P, state *  direction * PWM); // positive
  analogWrite(MOT * MOT1N + !MOT * MOT2N, state * !direction * PWM); // negative
}

void ConfigurePins(){
  int outputs[] = {MOT1P, MOT1N, MOT2P, MOT2N};
  for (int i = 0; i < 4; i++){ // asigning exits
    pinMode(outputs[i],3);
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
  pinMode(LS1, INPUT_PULLUP); // Turn pullup resistor on
  pinMode(LS2, INPUT_PULLUP); // Turn pullup resistor on
  attachInterrupt(LS1, setXas0, FALLING);  // Interrupt
  attachInterrupt(LS2, setYas0, FALLING);  // Interrupt
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
  a += "X:";  
  a += linearValue[MOT1];
  a += ",Y:";
  a += linearValue[MOT2];
  Serial.println(a);
}
