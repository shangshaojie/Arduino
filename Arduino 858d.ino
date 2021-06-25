//Arduino 585d热风枪，2021.6.25 极客杰
#include <EEPROM.h>
#include <PID_v1.h>
#include <Wire.h>
#include <max6675.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/URW_Gothic_L_Demi_16.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const byte encoder0PinA = 2; //1st Rotary encoder interrupt pin
const byte encoder0PinB = 4; //1st Rotary encoder pin
const byte encoder1PinA = 3; //2nd Rotary encoder interrupt pin
const byte encoder1PinB = 5; //2nd Rotary encoder pin
const byte airflowreg = 6; // turbofan regulator MOSFET
const byte sleepsw = 8; // sleep switch (handle holder switch)
const byte heaterreg = 7; // heater regulator TRIAC(via optocoupler)
const byte buzzer = A3 ; // BUZZER
const byte LED1 = 9; // LED
const byte pinSO  = 10; // Thermocouple driver
const byte pinCS  = 11; // Thermocouple driver
const byte pinSCK = 12; // Thermocouple driver
const byte encbutton1 = A2; // 1st rotary encoder button
const byte encbutton2 = A1; // 2nd rotary encoder button
static long rotaryCount = 0;
static long rotary1Count = 0;
long previousMillis = 0;
int WindowSize = 1000;
unsigned long windowStartTime;
int menu = 1;
volatile bool fired;
volatile bool up;
volatile bool fired1;
volatile bool up1;
bool swtrig = false;
const unsigned long measurementPeriod = 250;
float teplotaC;
unsigned long timer;

double Setpoint, Input;  // Temperature (must be in the same units)
double Output;  // 0-WindowSize, Part of PWM window where output is ACTIVE.
double Kp = 3; // Proportional Constant: Active 10 milliseconds for each degree low
double Ki = 1;  // Integral Constant: Increase to prevent offset (Input settles too high or too low)
double Kd = 2;  // Differential Constant:  Increase to prevent overshoot (Input goes past Setpoint)


PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
MAX6675 termoclanek(pinSCK, pinCS, pinSO);

struct AppSettings
{
  int tempadj = 0;
  int rotaryCountSet = 0;
  int rotary1CountSet = 0;
  int fanmultiplier = 0;
  int tempmultiplier = 0;
  int minAirflow = 0;  //failsafe rychlost turbofanu 2=10%
  int maxAirflow = 0; // max rychlost turbofanu 20=100%
  int minTemp = 0;  //minimalni teplota heateru 2=10%
  int maxTemp = 0; // maximalni teplota heateru 20=100%
} AppSettings;



const unsigned char fan [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0x00, 0x03, 0x80, 0x60, 0x00, 0x06, 0x00, 0x10, 0x00,
  0x08, 0x7f, 0x08, 0x00, 0x10, 0xff, 0x84, 0x00, 0x30, 0xff, 0xc2, 0x00, 0x20, 0xff, 0xc2, 0x00,
  0x20, 0xff, 0x81, 0x00, 0x40, 0x7e, 0x01, 0x00, 0x42, 0x3c, 0x61, 0x00, 0x47, 0x1d, 0xf9, 0x00,
  0x4f, 0xf7, 0xf9, 0x00, 0x4f, 0xdd, 0xf9, 0x00, 0x4f, 0xeb, 0xf9, 0x00, 0x47, 0xf3, 0xf9, 0x00,
  0x27, 0xf3, 0xf9, 0x00, 0x27, 0xf3, 0xf2, 0x00, 0x33, 0xf3, 0xe2, 0x00, 0x11, 0xc3, 0x84, 0x00,
  0x08, 0x00, 0x08, 0x00, 0x06, 0x00, 0x10, 0x00, 0x03, 0x80, 0x60, 0x00, 0x00, 0xff, 0x80, 0x00,
  0x00, 0x00, 0x00, 0x00
};

const unsigned char heater [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0c, 0x60, 0x00, 0x01, 0x8c, 0x30, 0x00,
  0x00, 0x86, 0x30, 0x00, 0x00, 0xc6, 0x10, 0x00, 0x00, 0xc6, 0x10, 0x00, 0x00, 0xc6, 0x30, 0x00,
  0x01, 0x86, 0x30, 0x00, 0x01, 0x8c, 0x30, 0x00, 0x01, 0x8c, 0x60, 0x00, 0x03, 0x18, 0x60, 0x00,
  0x03, 0x18, 0xc0, 0x00, 0x06, 0x30, 0xc0, 0x00, 0x06, 0x31, 0x80, 0x00, 0x04, 0x31, 0x80, 0x00,
  0x0c, 0x31, 0x80, 0x00, 0x0c, 0x31, 0x80, 0x00, 0x06, 0x31, 0x80, 0x00, 0x06, 0x38, 0xc0, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0x00, 0x7f, 0xff, 0xff, 0x00, 0x7f, 0xff, 0xff, 0x00,
  0x00, 0x00, 0x00, 0x00
};

const unsigned char sleepicon [] PROGMEM = {
  0x00, 0x00, 0x7f, 0x00, 0x00, 0x00, 0x07, 0xff, 0x80, 0x00, 0x00, 0x1f, 0xff, 0x80, 0x00, 0x00,
  0x7c, 0x1e, 0x00, 0x00, 0x00, 0xf0, 0x38, 0x00, 0x00, 0x01, 0xc0, 0x70, 0x00, 0x00, 0x03, 0x80,
  0xe0, 0x00, 0x00, 0x07, 0x01, 0xc0, 0x00, 0x00, 0x0e, 0x01, 0xc0, 0x00, 0x00, 0x1c, 0x03, 0x80,
  0x00, 0x00, 0x1c, 0x03, 0x80, 0x00, 0x00, 0x38, 0x03, 0x00, 0x00, 0x00, 0x30, 0x03, 0x00, 0x00,
  0x00, 0x70, 0x07, 0x00, 0x00, 0x00, 0x70, 0x07, 0x00, 0x00, 0x00, 0x60, 0x07, 0x00, 0x00, 0xe0,
  0x60, 0x07, 0x00, 0x00, 0xe0, 0x60, 0x03, 0x00, 0x00, 0xc0, 0x60, 0x03, 0x01, 0xff, 0xe0, 0xe0,
  0x03, 0x00, 0xfc, 0xe0, 0xe0, 0x03, 0x80, 0x3c, 0x00, 0x60, 0x01, 0xc0, 0x78, 0x00, 0x60, 0x01,
  0xc0, 0xf0, 0x00, 0x60, 0x00, 0xe0, 0xfc, 0x00, 0x60, 0x00, 0x70, 0xfe, 0x06, 0x70, 0x00, 0x38,
  0x00, 0x0f, 0x70, 0x00, 0x1e, 0x00, 0x3e, 0x30, 0x00, 0x0f, 0xc0, 0xfe, 0x38, 0x00, 0x03, 0xff,
  0xfc, 0x1c, 0x00, 0x00, 0xff, 0x9c, 0x1c, 0x00, 0x00, 0x00, 0x38, 0x0e, 0x00, 0x00, 0x00, 0x38,
  0x07, 0x00, 0x00, 0x00, 0x70, 0x03, 0x80, 0x00, 0x00, 0xe0, 0x01, 0xc0, 0x00, 0x03, 0xc0, 0x00,
  0xf0, 0x00, 0x07, 0x80, 0x00, 0x7c, 0x00, 0x1f, 0x00, 0x00, 0x1f, 0xc1, 0xfc, 0x00, 0x00, 0x07,
  0xff, 0xf0, 0x00, 0x00, 0x00, 0xff, 0x80, 0x00
};

const unsigned char disc [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1f, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x7e, 0x00,
  0x00, 0x3c, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x0e, 0x3e, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x0e, 0x3e,
  0x00, 0x00, 0x00, 0x7e, 0x00, 0x0e, 0x3f, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x0e, 0x3f, 0x80, 0x00,
  0x00, 0x7e, 0x00, 0x0e, 0x3f, 0x80, 0x00, 0x00, 0x7e, 0x00, 0x0e, 0x3f, 0x80, 0x00, 0x00, 0x7e,
  0x00, 0x0e, 0x3f, 0x80, 0x00, 0x00, 0x7e, 0x00, 0x0e, 0x3f, 0x80, 0x00, 0x00, 0x7e, 0x00, 0x00,
  0x3f, 0x80, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x7f, 0x80, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0x80,
  0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00,
  0x78, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x78, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x78, 0x00,
  0x00, 0x03, 0x80, 0x00, 0x00, 0x78, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x78, 0x00, 0x00, 0x03,
  0x80, 0x00, 0x00, 0x78, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x78, 0x00, 0x00, 0x03, 0x80, 0x00,
  0x00, 0x78, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x78, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x78,
  0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x78, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x78, 0x00, 0x00,
  0x03, 0x80, 0x00, 0x00, 0x78, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x78, 0x00, 0x00, 0x03, 0x80,
  0x00, 0x00, 0x38, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


// Interrupt Service Routine for a change to encoder0 pin A
void isr ()
{
  if (digitalRead (encoder0PinA))
    up = digitalRead (encoder0PinB);
  else
    up = !digitalRead (encoder0PinB);
  fired = true;
}

// Interrupt Service Routine for a change to encoder1 pin A
void isr1 ()
{
  if (digitalRead (encoder1PinA))
    up1 = digitalRead (encoder1PinB);
  else
    up1 = !digitalRead (encoder1PinB);
  fired1 = true;
}

void setup ()
{
  pinMode (encoder0PinA, INPUT);
  pinMode (encoder0PinB, INPUT);
  pinMode (encoder1PinA, INPUT);
  pinMode (encoder1PinB, INPUT);
  pinMode (encbutton1, INPUT_PULLUP);
  pinMode (encbutton2, INPUT_PULLUP);
  pinMode(airflowreg, OUTPUT);
  pinMode(heaterreg, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(sleepsw, INPUT);
  attachInterrupt (digitalPinToInterrupt (encoder0PinA), isr, CHANGE);  // interrupt 0 is pin 2
  attachInterrupt (digitalPinToInterrupt (encoder1PinA), isr1, CHANGE);  // interrupt 1 is pin 3
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetMode(AUTOMATIC);
  display.clearDisplay();
  display.setFont(&URW_Gothic_L_Demi_16);
  display.setCursor(6, 35);
  display.setTextColor(WHITE);
  display.print("HOTAIR PRO");
  display.setFont();
  display.setTextSize(1);
  display.setCursor(90, 2);
  display.print("v.1.21");
  display.setCursor(35, 55);
  display.print("designed by XPM");
  display.display();
  delay(1000);
  buzzertone1();
  RestoreSettings();
  rotaryCount = AppSettings.rotaryCountSet;
  rotary1Count = AppSettings.rotary1CountSet;
  timer  = millis();

}

void loop ()  {

  rotaryCount = constrain(rotaryCount, AppSettings.minAirflow, AppSettings.maxAirflow);
  rotary1Count = constrain(rotary1Count, AppSettings.minTemp, AppSettings.maxTemp);
  AppSettings.rotaryCountSet = constrain(AppSettings.rotaryCountSet, AppSettings.minAirflow, AppSettings.maxAirflow);
  AppSettings.rotary1CountSet = constrain(AppSettings.rotary1CountSet, AppSettings.minTemp, AppSettings.maxTemp);
  unsigned long currentTime = millis();

  //THERMOCOUPLE DRIVER TIMER FOR READING TEMPERATURE (the driver needs min. 250ms)//
  if (millis() - timer >= measurementPeriod) {
    timer += measurementPeriod;
    teplotaC = (termoclanek.readCelsius() + AppSettings.tempadj);
  }

  //PART OF THE TEMP PID CONTROL//
  Setpoint = rotary1Count;
  Input = teplotaC, 0;
  myPID.Compute();

  if (currentTime - windowStartTime > WindowSize)
  {
    windowStartTime += WindowSize;
  }
  //WHEN HANDLE SLEEP SWITCH OFF//
  if (digitalRead(sleepsw) == LOW) {
    updateMenu();
    swtrig = false;

    //TRIAC DRIVER OUTPUT CONTROL AFTER PID CALCULATION//
    if (Output < (currentTime - windowStartTime))
      digitalWrite(heaterreg, LOW);
    else
      digitalWrite(heaterreg, HIGH);


    //TURBOFAN MOSFET DRIVER OUTPUT//
    analogWrite(airflowreg, (rotaryCount * 255) / 100);


    //1st ROTARY ENCODER MAIN MENU CONTROL//
    if (menu == 1) {
      if (fired)
      {
        if (up)
          rotaryCount = rotaryCount + AppSettings.fanmultiplier;
        else
          rotaryCount = rotaryCount - AppSettings.fanmultiplier;
      }

      //2nd ROTARY ENCODER MAIN MENU CONTROL//
      if (fired1)
      {
        if (up1)
          rotary1Count = rotary1Count + AppSettings.tempmultiplier;
        else
          rotary1Count = rotary1Count - AppSettings.tempmultiplier;
      }
      fired = false;
      fired1 = false;
    }
    //1st ROTARY ENCODER THERMOCOUPLE ADJUST MENU CONTROL//
    if (menu == 2) {
      if (fired)
      {
        if (up)
          AppSettings.tempadj++;
        else
          AppSettings.tempadj--;
      }

      fired = false;
      fired1 = false;
    }
    //1st ROTARY ENCODER STARTUP SETTINGS MENU CONTROL//
    if (menu == 3) {
      if (fired)
      {
        if (up)
          AppSettings.rotaryCountSet = AppSettings.rotaryCountSet + AppSettings.fanmultiplier;
        else
          AppSettings.rotaryCountSet = AppSettings.rotaryCountSet - AppSettings.fanmultiplier;
      }

      //2nd ROTARY ENCODER STARTUP SETTINGS MENU CONTROL//
      if (fired1)
      {
        if (up1)
          AppSettings.rotary1CountSet = AppSettings.rotary1CountSet + AppSettings.tempmultiplier;
        else
          AppSettings.rotary1CountSet = AppSettings.rotary1CountSet - AppSettings.tempmultiplier;
      }
      fired = false;
      fired1 = false;
    }
    //1st ROTARY ENCODER MULTIPLIER MENU CONTROL//
    if (menu == 4) {
      if (fired)
      {
        if (up)
          AppSettings.fanmultiplier++;
        else
          AppSettings.fanmultiplier--;
      }

      //2nd ROTARY ENCODER MULTIPLIER MENU CONTROL//
      if (fired1)
      {
        if (up1)
          AppSettings.tempmultiplier++;
        else
          AppSettings.tempmultiplier--;
      }
      fired = false;
      fired1 = false;
    }
    //1st ROTARY ENCODER FAIL-SAFE MIN. MENU CONTROL//
    if (menu == 5) {
      if (fired)
      {
        if (up)
          AppSettings.minAirflow++;
        else
          AppSettings.minAirflow--;
      }

      //2nd ROTARY ENCODER FAIL-SAFE MIN. MENU CONTROL//
      if (fired1)
      {
        if (up1)
          AppSettings.minTemp++;
        else
          AppSettings.minTemp--;
      }
      fired = false;
      fired1 = false;
    }
    //1st ROTARY ENCODER FAIL-SAFE MAX. MENU CONTROL//
    if (menu == 6) {
      if (fired)
      {
        if (up)
          AppSettings.maxAirflow++;
        else
          AppSettings.maxAirflow--;
      }

      //1st ROTARY ENCODER FAIL-SAFE MAX. MENU CONTROL//
      if (fired1)
      {
        if (up1)
          AppSettings.maxTemp = AppSettings.maxTemp + AppSettings.tempmultiplier;
        else
          AppSettings.maxTemp = AppSettings.maxTemp - AppSettings.tempmultiplier;
      }
      fired = false;
      fired1 = false;
    }


    //TEMPERATURE ALARM//
    if (teplotaC > 600) {
      buzzertone4();
    }


  //1st ROTARY ENCODER MENU BUTTON//
    if (digitalRead(encbutton1) == LOW) {
      menu++;
      updateMenu();
      buzzertone2();
      delay(10);
      while (digitalRead(encbutton1) == LOW);
    }
  }


  //SLEEP DRIVE//

  if ( digitalRead(sleepsw) == HIGH && swtrig == false) {
    if (teplotaC < 100) {
      drawsleep();
      buzzertone3();
      analogWrite(airflowreg, 0);
      digitalWrite(heaterreg, LOW);
      analogWrite(LED1, 5);

      swtrig = true;
    } else {
      display.clearDisplay();
      display.drawBitmap(95, 15, fan, 25, 25, WHITE);
      display.fillRect(0, 0, (teplotaC * 91) / AppSettings.maxTemp , 13 , WHITE);
      display.setFont(&URW_Gothic_L_Demi_16);
      display.setCursor(5, 33);
      display.setTextColor(WHITE);
      display.print("CHLAZENI");
      display.setCursor(33, 58);
      display.print(teplotaC, 0);
      display.print("*C");
      display.display();
      analogWrite(airflowreg, 255);
      digitalWrite(heaterreg, LOW);
    }
  }

  delay(2);
}


void updateMenu() {

  switch (menu) {
    case 0:
      menu = 1;
      break;
    case 1:
      display.clearDisplay();
      display.drawBitmap(4, 14, fan, 25, 25, WHITE);
      display.drawBitmap(4, 40, heater, 25, 25, WHITE);
      display.fillRect(0, 0, 128, 12, WHITE);
      display.drawRect(0, 15, 128, 49, WHITE);
      display.setTextSize(1);
      display.setCursor(1, 2);
      display.setTextColor(BLACK);
      display.print("MAIN SCREEN");
      display.drawRoundRect(30, 17, 95, 20, 2, WHITE);
      display.fillRoundRect(32, 19, (rotaryCount * 91) / AppSettings.maxAirflow , 16 , 2, WHITE);
      if ( rotaryCount >= 35) {
        display.setTextColor(BLACK, WHITE); // 'inverted' text
      } else {
        display.setTextColor(WHITE);
      }
      display.setCursor(58, 27);
      display.setFont(&URW_Gothic_L_Demi_16);
      display.print(rotaryCount);
      display.print("%");
      display.drawRoundRect(30, 41, 95, 20, 2, WHITE);
      display.fillRoundRect(32, 43, (rotary1Count * 91) / AppSettings.maxTemp , 16 , 2, WHITE);
      if ( rotary1Count >= 95) {
        display.setTextColor(BLACK, WHITE); // 'inverted' text
        display.fillRoundRect(40, 44, 43, 14 , 1, WHITE);
      } else {
        display.setTextColor(WHITE);
      }
      display.setCursor(45, 57);
      display.print(rotary1Count);
      display.print("/");
      display.setFont();
      display.setTextSize(1);
      if ( rotary1Count >= 350) {
        display.setTextColor(BLACK, WHITE); // 'inverted' text
      } else {
        display.setTextColor(WHITE);
      }
      display.setCursor(94, 47);
      display.print(teplotaC, 0);
      if  (digitalRead(heaterreg)) {
        analogWrite(LED1, 40);
      } else {
        analogWrite(LED1, 0);
      }
      break;
    case 2:
      display.clearDisplay();
      display.fillRect(0, 0, 128, 12, WHITE);
      display.setTextSize(1);
      display.setCursor(1, 2);
      display.setTextColor(BLACK);
      display.print("THERMOCOUPLE ADJUST");
      display.setTextColor(WHITE);
      display.setCursor(5, 17);
      display.print("SENSOR: ");
      display.print(teplotaC, 1);
      display.print(" *C");
      display.setFont(&URW_Gothic_L_Demi_16);
      display.setCursor(52, 50);
      display.print(AppSettings.tempadj);
      display.setFont();
      display.drawRoundRect(2, 30, 120, 30, 3, WHITE);
      break;
    case 3:
      display.clearDisplay();
      display.drawBitmap(4, 14, fan, 25, 25, WHITE);
      display.drawBitmap(4, 40, heater, 25, 25, WHITE);
      display.fillRect(0, 0, 128, 12, WHITE);
      display.setTextSize(1);
      display.setCursor(1, 2);
      display.setTextColor(BLACK);
      display.print("STARUP SETTINGS");
      display.setTextColor(WHITE);
      display.setCursor(45, 27);
      display.setFont(&URW_Gothic_L_Demi_16);
      display.print(AppSettings.rotaryCountSet);
      display.print("%");
      display.setCursor(45, 57);
      display.print(AppSettings.rotary1CountSet);
      display.print("*c");
      display.setFont();
      break;
    case 4:
      display.clearDisplay();
      display.drawBitmap(4, 14, fan, 25, 25, WHITE);
      display.drawBitmap(4, 40, heater, 25, 25, WHITE);
      display.fillRect(0, 0, 128, 12, WHITE);
      display.setTextSize(1);
      display.setCursor(1, 2);
      display.setTextColor(BLACK);
      display.print("NASOBIC ENKODERU");
      display.setTextColor(WHITE);
      display.setCursor(45, 27);
      display.setFont(&URW_Gothic_L_Demi_16);
      display.print("x ");
      display.print(AppSettings.fanmultiplier);
      display.setCursor(45, 57);
      display.print("x ");
      display.print(AppSettings.tempmultiplier);
      display.setFont();
      break;
    case 5:
      display.clearDisplay();
      display.drawBitmap(4, 14, fan, 25, 25, WHITE);
      display.drawBitmap(4, 40, heater, 25, 25, WHITE);
      display.fillRect(0, 0, 128, 12, WHITE);
      display.setTextSize(1);
      display.setCursor(1, 2);
      display.setTextColor(BLACK);
      display.print("FAIL-SAFE MIN.");
      display.setTextColor(WHITE);
      display.setCursor(45, 27);
      display.setFont(&URW_Gothic_L_Demi_16);
      display.print(AppSettings.minAirflow);
      display.print("%");
      display.setCursor(45, 57);
      display.print(AppSettings.minTemp);
      display.print("*c");
      display.setFont();
      break;
    case 6:
      display.clearDisplay();
      display.drawBitmap(4, 14, fan, 25, 25, WHITE);
      display.drawBitmap(4, 40, heater, 25, 25, WHITE);
      display.fillRect(0, 0, 128, 12, WHITE);
      display.setTextSize(1);
      display.setCursor(1, 2);
      display.setTextColor(BLACK);
      display.print("FAIL-SAFE MAX.");
      display.setTextColor(WHITE);
      display.setCursor(45, 27);
      display.setFont(&URW_Gothic_L_Demi_16);
      display.print(AppSettings.maxAirflow);
      display.print("%");
      display.setCursor(45, 57);
      display.print(AppSettings.maxTemp);
      display.print("*c");
      display.setFont();
      break;
    case 7: 
      display.clearDisplay();
      display.drawBitmap(88, 15, disc, 50, 50, WHITE);
      display.fillRect(0, 0, 128, 12, WHITE);
      display.setTextSize(1);
      display.setCursor(1, 2);
      display.setTextColor(BLACK);
      display.print("SAVE TO EEPROM");
      display.setTextColor(WHITE);
      display.setCursor(0, 30);
      display.setFont(&URW_Gothic_L_Demi_16);
      display.print("Save");
      display.setCursor(0, 55);
      display.print("values?");
      display.setFont();
        //2nd ROTARY ENCODER EEPROM BUTTON (confirm save to eeprom)//
      if (digitalRead(encbutton2) == LOW) {
        savescreen();
        while (digitalRead(encbutton2) == LOW);
      }
      break;
    case 8:
      menu = 1;
      break;
  }
  display.display();

}


//SLEEP SCREEN//
void drawsleep() {
  display.clearDisplay();
  display.drawBitmap(80, 16, sleepicon, 40, 40, WHITE);
  display.setFont(&URW_Gothic_L_Demi_16);
  display.setCursor(10, 40);
  display.setTextColor(WHITE);
  display.print("SLEEP");
  display.display();
  delay(3000);
  display.clearDisplay();
  display.display();

}

//EEPROM CONTROL//
void SaveSettings()
{
  ClearEEPROM();
  int eeAddress = 0;
  EEPROM.put(eeAddress, AppSettings);
}

void RestoreSettings()
{
  int eeAddress = 0;
  EEPROM.get(eeAddress, AppSettings);
}

void ClearEEPROM()
{
  for (int i = 0; i < EEPROM.length(); i++)
  {
    EEPROM.write(i, 0);
  }
}


void savescreen() {
  display.clearDisplay();
  display.drawBitmap(88, 15, disc, 50, 50, WHITE);
  display.setFont(&URW_Gothic_L_Demi_16);
  display.setCursor(0, 46);
  display.print("SAVING..");
  display.display();
  SaveSettings();
  buzzertone3();
  display.clearDisplay();
  display.drawBitmap(88, 15, disc, 50, 50, WHITE);
  display.fillRoundRect(100, 40, 28, 15, 3, WHITE);
  display.setFont(&URW_Gothic_L_Demi_16);
  display.setCursor(0, 46);
  display.print("SAVED");
  display.display();
  delay(2000);

  menu = 1;
}


// BUZZER
void buzzertone1() {
  tone(buzzer, 2000); // Send 1KHz sound signal...
  delay(50);        // ...for 1 sec
  noTone(buzzer);     // Stop sound...
  delay(100);        // ...for 1sec
  tone(buzzer, 2500); // Send 1KHz sound signal...
  delay(50);        // ...for 1 sec
  noTone(buzzer);     // Stop sound...
  delay(100);        // ...for 1se
  tone(buzzer, 3000); // Send 1KHz sound signal...
  delay(50);        // ...for 1 sec
  noTone(buzzer);     // Stop sound...

}

// BUZZER
void buzzertone2() {
  tone(buzzer, 3000); // Send 1KHz sound signal...
  delay(50);        // ...for 1 sec
  noTone(buzzer);     // Stop sound...
  delay(50);        // ...for 1sec
}

// BUZZER
void buzzertone3() {
  tone(buzzer, 2000); // Send 1KHz sound signal...
  delay(500);        // ...for 1 sec
  noTone(buzzer);     // Stop sound...
  delay(100);        // ...for 1sec
  // ...for 1sec
}

// BUZZER
void buzzertone4() {
  tone(buzzer, 2000); // Send 1KHz sound signal...
  delay(100);        // ...for 1 sec
  noTone(buzzer);     // Stop sound...
  delay(100);        // ...for 1sec
  // ...for 1sec
}
