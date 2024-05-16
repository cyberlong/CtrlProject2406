#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LiquidCrystal_I2C.h>
#include "EzSerial.h"
#include "AS5600.h"
#include "Pages.h"
#include "CustomChars.h"

// Motor Control
bool MOT1D = 0; // Direction
bool MOT2D = 1;
uint8_t MOT1S = 0; // PWM Signal
uint8_t MOT2S = 0;
bool MOT1O = 0; // On/Off
bool MOT2O = 0;

// Setup Control Parameters and variables
#define Clk 1e6 // 1Mhz
#define ms 1000
#define Tm 30   //ms
float toCTRL = 0;
uint8_t PWM = 0;

// PID params and Vars
#define PID_Kp 1
#define PID_Ki 2
#define PID_Kd 3
float PID_Emem = 0;
float PID_Imem = 0;

// WiFi connect
// #define ssid      "TP-LINK_B0C8"
// #define password  "80742555"
#define WbQty 4
const char *ssid[]     = {"Galaxy A51 5739","WUNAB_CSU","WUNAB_Jardin","TP-LINK_B0C8"};
const char *password[] = {"zasx8682"       ,NULL       ,NULL          ,"80742555"};
uint16_t timeConnecting = 0;
uint8_t tryIndex = 0;

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

// System Objects
hw_timer_t* timer = NULL;
WebServer server(80);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// random vars
float lastRead = 0;
float dTh = 0;



// The ISR is placed before Setup bc Arduino Sucks
void IRAM_ATTR Measure(){
  // code :D
  SingleMOTCtrl(MOT1,MOT1S,MOT1O,MOT1D);
  SingleMOTCtrl(MOT2,MOT2S,MOT2O,MOT2D);
  PID(0);

}
//    <===========================SETUP==============================>
void setup() {
  BeginEzSerial(115200);
  // Timer at Clk hz, 1 Mhz is used to have 1us steps
  timer = timerBegin(Clk);
  // interrupt using the address of the ISR
  timerAttachInterrupt(timer, &Measure);
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
}

void PID (float SeP){
  float En = SeP - toCTRL;
  float csP = PID_Kp*En;
  float csD = PID_Kd*(En - PID_Emem);
  float Eni = En + PID_Imem;
  float csI = PID_Ki*Eni;
  float cs = csP + csI + csD;
  if (cs > 100) cs = 100.0;
  if (cs < 0)   cs = 0.0;
  PWM = 255.0 * cs/100.0;
  PID_Emem = En;
  PID_Imem = Eni;
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
