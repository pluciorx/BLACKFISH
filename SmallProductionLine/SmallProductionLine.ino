#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Debounce.h>
#include <LiquidCrystal_I2C.h>

// PANEL BUTTONS
#define BTN_PULL_OUT 22
#define BTN_PULL_IN 23
#define BTN_HEAT1 24
#define BTN_HEAT2 25
#define BTN_PROD_START 26
#define BTN_PROD_END 27
#define BTN_TAPE_FWD 28
#define BTN_TAPE_REV 29
#define BTN_FAIL_STOP 30

#define BTN_MENU_RIGHT 31
#define BTN_MENU_LEFT 32
#define BTN_MENU_UP 33
#define BTN_MENU_DOWN 34
#define BTN_MENU_ENTER 35

//Panel Lights
#define LED_PULL_OUT PIN_A8
#define LED_PULL_IN PIN_A9
#define LED_HEAT1 PIN_A10
#define LED_HEAT2 PIN_A11
#define LED_PROD_START PIN_A12
#define LED_PROD_END PIN_A13
#define LED_TAPE_FWD PIN_A14
#define LED_TAPE_REV PIN_A15

#define LIGHT_RED_COL 50
#define LIGHT_YELLOW_COL 51
#define LIGHT_GREEN_COL	52

//REMOTE CONTROL BUTTONS 
#define REMOTE_PULL_OUT 4 
#define REMOTE_PULL_IN 5
#define REMOTE_TAPE_FWD 6
#define REMOTE_TAPE_REV 7
#define REMOTE_PROD_START 8
#define REMOTE_PROD_STOP 11
#define REMOTE_RESERVED_1 14
#define REMOTE_RESERVED_2 15
#define REMOTE_ALLOW_REMOTE 16
#define REMOTE_START_SIGNAL 17

//OTHERS:

//DS18B20
OneWire oneWire(36);
DallasTemperature DSTemp(&oneWire);
float DSTempC[2] = { -255, -255 };

LiquidCrystal_I2C lcd(0x27, 20, 4);



Adafruit_Debounce btnB1(P1_B1, LOW);
Adafruit_Debounce btnB2(P1_B2, LOW);
Adafruit_Debounce btnB3(P1_B3, LOW);
Adafruit_Debounce btnB4(P1_B4, LOW);

// the setup function runs once when you press reset or power the board
void setup() {

}

// the loop function runs over and over again until power down or reset
void loop() {
  
}
