#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LiquidCrystal_I2C.h>  
#include <ezButton.h>           
#include "EzSerial.h"
#include "Pages.h"
#include "CustomChars.h"

// Sensor pins
#define Xsense 15
#define Ysense 4

// Motor Pins
#define MOT1P 18
#define MOT1N 19
#define MOT2P 16
#define MOT2N 17
#define MOT1  1
#define MOT2  0

// Pins of the encoders
#define CodA 25
#define CodB 26
#define CodC 32
#define CodD 33

// Pins of the Limit Switches
#define LS1  27
#define LS2  35

// Motor Control
bool MOT1D = 0;     // Direction
bool MOT2D = 1;
uint8_t MOT1S = 0;  // PWM Signal
uint8_t MOT2S = 0;
bool MOT1O = 0;     // On/Off
bool MOT2O = 0;
int MOT1CS = 0;     // Control Signal
int MOT2CS = 0;

// Setup Control Parameters and variables
#define Clk 1e6 // 1Mhz
#define ms 1000
#define Tm 30   // ms
#define PPT 60.0                  // Pulses per turn
#define mmPT 12.0                 // Milimeter per turn
#define RPS2RPM 60              // rev per sec to rev per minute
#define RPS2RadPs 2 * 3.141592  // rev per sec to rad per sec
#define alph 0.2                // Alpha for EMA
float toCTRL = 0;
uint8_t PWM = 0;
uint8_t codPins[][2] = {{CodA, CodB},{CodC, CodD}}; // encoder pins
int encoderValue[]   = {0,0};           // the current value
uint32_t last_time[] = {0,0};           // for debouncing the encoder
float EMAprev[]      = {0,0};           // for EMAprev
float linearValue[]  = {0,0};           // Axis linear value
float setPoint[]     = {0,0};           // The setppointa
uint32_t staticTime[]= {0,0};           // stebilize chepoint

// PID params and Vars
#define PID_Kp 1
#define PID_Ki 2
#define PID_Kd 3
float PID_Emem = 0;
float PID_Imem = 0;

// WiFi connect
#define WbQty 4
const char *ssid[]     = {"Galaxy A51 5739","WUNAB_CSU","WUNAB_Jardin","TP-LINK_B0C8"};
const char *password[] = {"zasx8682"       ,NULL       ,NULL          ,"80742555"};
uint16_t timeConnecting = 0;
uint8_t tryIndex = 0;

// System Objects
hw_timer_t* timer = NULL;
WebServer server(80);
LiquidCrystal_I2C lcd(0x27, 16, 2);
ezButton ls1(LS1);
ezButton ls2(LS2);

// random vars
float lastRead = 0;
float dTh = 0;

//    <===========================Asyncronous Functions==============================>
// The ISR is placed before Setup bc Arduino Sucks
void IRAM_ATTR Measure(){
  // code :D
  linearValue[MOT1] = updtLinearPos(encoderValue[MOT1]);
  linearValue[MOT2] = updtLinearPos(encoderValue[MOT2]);

  PID(0, linearValue[MOT1], &MOT1CS);
  PID(0, linearValue[MOT2], &MOT2CS);
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

void setZero(int* encVal, float* linVal, float* EMAval){
  *encVal = 0;
  *linVal = 0;
  *EMAval = 0;
}

//      <<<=|Encoder Interrupts|=>>>
void IRAM_ATTR updateEncoderA(){
    UpDtEnc(&last_time[MOT1], &encoderValue[MOT1],codPins[MOT1]);
}

void IRAM_ATTR updateEncoderB(){
    UpDtEnc(&last_time[MOT2], &encoderValue[MOT2],codPins[MOT2]);
}

//      <<<=|Update encoder Value Function|=>>>
void UpDtEnc(uint32_t* Ltiem, int* encVal, uint8_t pins[]){
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
  BeginEzSerial(9600);
  // Timer at Clk hz, 1 Mhz is used to have 1us steps
  timer = timerBegin(Clk);
  // interrupt using the address of the ISR
  timerAttachInterrupt(timer, &Measure);
  // Prepare the pins and interrupts of the encoders
  pinMode(CodA, INPUT_PULLUP); // Turn pullup resistor on
  pinMode(CodB, INPUT_PULLUP); // Turn pullup resistor on
  pinMode(CodC, INPUT_PULLUP); // Turn pullup resistor on
  pinMode(CodD, INPUT_PULLUP); // Turn pullup resistor on
  attachInterrupt(CodC, updateEncoderB, CHANGE);  // Interrupt
  attachInterrupt(CodD, updateEncoderB, CHANGE);  // Interrupt
  attachInterrupt(CodA, updateEncoderA, CHANGE);  // Interrupt
  attachInterrupt(CodB, updateEncoderA, CHANGE);  // Interrupt
  // Prepare the pins and interrupts of the limit switches
  pinMode(LS1, INPUT_PULLUP); // Turn pullup resistor on
  pinMode(LS2, INPUT_PULLUP); // Turn pullup resistor on
  attachInterrupt(LS1, setYas0, FALLING);  // Interrupt
  attachInterrupt(LS2, setXas0, FALLING);  // Interrupt
  // Initialize the LCD through I2C, along with custom chars
  LCDinit();
  // Configure the pins as needed
  ConfigurePins();
  // Starts the connection process to WiFi
  InitWiFiCon();
  // Builds the structure of the sites
  BuildSites();
  // Activates the server and gives the IP
  StartServer();
  // Just some time bc why not?
  delay(2000);
  // call an interrupt every n us, 
  // our Tm is 30 ms so that's that
  timerAlarm(timer,Tm*ms,true,0);
}

//    <===========================LOOPING==============================>
void loop() {
  server.handleClient(); // it explains itself



  SPHandler(&setPoint[MOT1],linearValue[MOT1],&staticTime[MOT1]);
  SPHandler(&setPoint[MOT2],linearValue[MOT2],&staticTime[MOT2]);

  CSMotCtrl(setPoint[MOT1],MOT1,1);
  CSMotCtrl(setPoint[MOT2],MOT2,1);

  // Debug
  db_AxisLCDprint();

  delay(5);
}

void SPHandler(float* currentSP, float PV, uint32_t* StaTime){
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

void PID (float SeP, float PV, int* CtrlVar){
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

float EMA(float current, float *EMApv){
  float daNew = alph*current + (1-alph)* (*EMApv); // The equation
  *EMApv = daNew; // Updates the Prev value
  return daNew;
}

//    <===========================WEB_PAGES==============================>
void Lobby(){ // Starting page
  server.send(200,"text/html", lobby);
}

void Notebook(){ // This is for the phone placed in the bed
  server.send(200,"text/html", notebook);
}

void Canvas(){ // This is for sending the path
  server.send(200,"text/html", canvas);
}

void OpenLoop(){ // Open Loop site
  
  if (server.hasArg("Xaxis") && server.arg("Xaxis") == "increment"){
    MOT2D = 0; MOT2O = 1; MOT2S = 255;
    lcd.setCursor(0, 0); lcd.print("Increment X     ");
  } else if (server.hasArg("Xaxis") && server.arg("Xaxis") == "decrement"){
    MOT2D = 1; MOT2O = 1; MOT2S = 255;
    lcd.setCursor(0, 0); lcd.print("Decrement X     ");
  } else if (server.hasArg("Xaxis") && server.arg("Xaxis") == "Stop"){
    MOT2D = 0; MOT2O = 0; MOT2S = 0;
    lcd.setCursor(0, 0); lcd.print("Stop X          ");
  }

  if (server.hasArg("Yaxis") && server.arg("Yaxis") == "increment"){
    MOT1D = 0; MOT1O = 1; MOT1S = 255;
    lcd.setCursor(0, 0); lcd.print("Increment Y     ");
  } else if (server.hasArg("Yaxis") && server.arg("Yaxis") == "decrement"){
    MOT1D = 1; MOT1O = 1; MOT1S = 255;
    lcd.setCursor(0, 0); lcd.print("Decrement Y     ");
  } else if (server.hasArg("Yaxis") && server.arg("Yaxis") == "Stop"){
    MOT1D = 0; MOT1O = 0; MOT1S = 0;
    lcd.setCursor(0, 0); lcd.print("Stop Y          ");
  }

  server.send(200,"text/html", openLoop);
}

//    <===========================PERIPHERALS==============================>
void SingleMOTCtrl(bool MOT , uint8_t PWM, bool state, bool direction){
  analogWrite(MOT * MOT1P + !MOT * MOT2P, state *  direction * PWM); // positive
  analogWrite(MOT * MOT1N + !MOT * MOT2N, state * !direction * PWM); // negative
}

void LCDinit(){
  lcd.init();//inicializar la pantalla lcd
  lcd.backlight();//Encender la luz de fondo
  lcd.createChar(NO, noks);
  lcd.createChar(OK, todobn);
}

void ConfigurePins(){
  int outputs[] = {MOT1P, MOT1N, MOT2P, MOT2N};
  for (int i = 0; i < 4; i++){ // asigning exits
    pinMode(outputs[i],3);
  } pinMode(2,3); // Debugging purposes

  int inputs[] = {Xsense, Ysense};
  for (int i = 0; i < 2; i++){ // asigning inputs
    pinMode(inputs[i],1);
  }
}

//    <===========================INITIALIZATION==============================>
void InitWiFiCon(){
  WiFi.mode(WIFI_STA); // Connect to WiFi
  lcd.setCursor (15, 0);
  lcd.write(NO);
  String ssidName;
  
  uint8_t Pos = 0;
  while (WiFi.status() != WL_CONNECTED){ // Loading anim
    tryIndex = ++tryIndex % WbQty;
    WiFi.begin(ssid[tryIndex], password[tryIndex]);

    lcd.setCursor (0, 0); // set 0 0... wow, don't you say, rlly??? awesome
    lcd.print(ssid[tryIndex]);      // web name is on LCD
    
    Serial.println();
    ssidName = ssid[tryIndex];
    Serial.println(ssidName); // and console
    
    uint8_t tries = 0;
    while (tries < 10){
      Serial.print(".");
      
      lcd.setCursor(Pos, 1);
      lcd.print(".");
      Pos++;
      if (Pos > 16){
        Pos = 0;
        lcd.setCursor(0, 1);
        lcd.print("                ");
      }
      
      if (WiFi.status() == WL_CONNECTED) break;

      tries++;
      delay(500);
    }

  } Serial.println();

  lcd.setCursor(0, 1);
  lcd.print("                ");

  Serial.print("Conectado a la red: ");
  Serial.println(ssidName);
  lcd.setCursor (15, 0);
  lcd.write(OK);
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  lcd.setCursor (0, 1); // Y'already kno what dis does 
  lcd.print(WiFi.localIP()); // LCD IP, noice
}

void StartServer(){
  server.on("/", Lobby);
  server.on("/notebook/", Notebook);
  server.on("/canvas/", Canvas);
  server.begin();
  Serial.println("Servidor HTTP iniciado");
}

//    <===========================DEBUG==============================>
void db_AxisLCDprint(){
  String a = "";
  a += "X: ";
  a += encoderValue[MOT1];
  a += "  ;  Y: ";
  a += encoderValue[MOT2];
  Serial.println(a);
}
