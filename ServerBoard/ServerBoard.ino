#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LiquidCrystal_I2C.h>
#include "Pages.h"
#include "CustomChars.h"

// WiFi connect
#define WbQty 4
const char *ssid[]     = {"Galaxy A51 5739","WUNAB_CSU","WUNAB_Jardin","TP-LINK_B0C8"};
const char *password[] = {"zasx8682"       ,NULL       ,NULL          ,"80742555"};
uint16_t timeConnecting = 0;
uint8_t tryIndex = 0;

// System Objects
WebServer server(80);
LiquidCrystal_I2C lcd(0x27, 16, 2);
HardwareSerial daController(2);

// UART vars
String iStr = "";
bool thereIsInput = false;
float nextSP[] = {0,0};
char *ptr = NULL;
char *strings[8];
bool plzSP = false;


//    <===========================SETUP==============================>
void setup() {
  Serial.begin(9600);
  // Start Serial port with Server from UART
  daController.begin(9600);
  // Initialize the LCD through I2C, along with custom chars
  LCDinit();
  // Starts the connection process to WiFi
  xTaskCreatePinnedToCore(
        InitWiFiConTask, /* Function to implement the task */
        "WiFiTask",     /* Name of the task */
        4096,           /* Stack size in words */
        NULL,           /* Task input parameter */
        1,              /* Priority of the task */
        NULL,           /* Task handle. */
        0);             /* Core where the task should run */
  // Builds the structure of the sites
  BuildSites();
  // Activates the server and gives the IP
  StartServer();
  // Just some time bc why not?
  delay(2000);
}

void InitWiFiConTask(void * parameter){
  // Serial.print("InitWiFiConTask running on core ");
  // Serial.println(xPortGetCoreID());
  InitWiFiCon();
  vTaskDelete(NULL); // Delete this task when done
}

//    <===========================LOOPING==============================>
void loop() {
  server.handleClient(); // it explains itself

  serialEvent();
  serialAssign();
  
  // When the Controller sends a message, what to do?
  if (thereIsInput){
    // do smth
    
    
    // reset
    iStr = "";
    thereIsInput = false;
  }

    serialInform();

    delay(5);
}

void serialEvent(){
  while (daController.available() > 0){
    char c = (char) daController.read();
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
  // Every pair of information we separate with a comma ',';
  // The key and the value are separated by a colon ':';
  // This is basically a Json RipOff.
void serialInform(){
  String forward = "";
  // forward += S_NextSPx + ':'; // next X SP
  forward += 420;
  forward += ','; //+ S_NextSPy + ':'; // next Y SP
  forward += 69;
  forward += ','; //+ S_WiFiOK + ':'; // is WiFi Ok?
  forward += WiFi.status() == WL_CONNECTED;
  forward += ','; //+ S_SSID + ':'; // what is the connected SSID
  forward += ssid[tryIndex];
  forward += ','; //+ S_IP + ':'; // Connection IP plz
  forward += WiFi.localIP();
  forward += ','; //+ S_ScreenX + ':'; // current X Screen Pos
  forward += 69;
  forward += ','; //+ S_ScreenY + ':'; // current Y Screen Pos
  forward += 420;
  
  daController.println(forward);

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

   plzSP = atoi(strings[0]);

   
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
  
  // if (server.hasArg("Xaxis") && server.arg("Xaxis") == "increment"){
  //   MOT2D = 0; MOT2O = 1; MOT2S = 255;
  //   lcd.setCursor(0, 0); lcd.print("Increment X     ");
  // } else if (server.hasArg("Xaxis") && server.arg("Xaxis") == "decrement"){
  //   MOT2D = 1; MOT2O = 1; MOT2S = 255;
  //   lcd.setCursor(0, 0); lcd.print("Decrement X     ");
  // } else if (server.hasArg("Xaxis") && server.arg("Xaxis") == "Stop"){
  //   MOT2D = 0; MOT2O = 0; MOT2S = 0;
  //   lcd.setCursor(0, 0); lcd.print("Stop X          ");
  // }

  // if (server.hasArg("Yaxis") && server.arg("Yaxis") == "increment"){
  //   MOT1D = 0; MOT1O = 1; MOT1S = 255;
  //   lcd.setCursor(0, 0); lcd.print("Increment Y     ");
  // } else if (server.hasArg("Yaxis") && server.arg("Yaxis") == "decrement"){
  //   MOT1D = 1; MOT1O = 1; MOT1S = 255;
  //   lcd.setCursor(0, 0); lcd.print("Decrement Y     ");
  // } else if (server.hasArg("Yaxis") && server.arg("Yaxis") == "Stop"){
  //   MOT1D = 0; MOT1O = 0; MOT1S = 0;
  //   lcd.setCursor(0, 0); lcd.print("Stop Y          ");
  // }

  server.send(200,"text/html", openLoop);
}

//    <===========================INITIALIZATION==============================>
void InitWiFiCon(){
  Serial.print("Free heap before WiFi init: ");
  Serial.println(ESP.getFreeHeap());

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

  Serial.print("Free heap after WiFi init: ");
  Serial.println(ESP.getFreeHeap());
}

void StartServer(){
  server.on("/", Lobby);
  server.on("/notebook/", Notebook);
  server.on("/canvas/", Canvas);
  server.begin();
  Serial.println("Servidor HTTP iniciado");
}

//    <===========================PERIPHERALS==============================>
void LCDinit(){
  lcd.init();//inicializar la pantalla lcd
  lcd.backlight();//Encender la luz de fondo
  lcd.createChar(NO, noks);
  lcd.createChar(OK, todobn);
}
