#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Debounce.h>
#include <LiquidCrystal_I2C.h>
#include <avdweb_VirtualDelay.h>

//---------- Control Panel Module ---------------
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

Adafruit_Debounce btnPullOut(BTN_PULL_OUT, LOW);
Adafruit_Debounce btnPullIn(BTN_PULL_IN, LOW);
Adafruit_Debounce btnHeat1(BTN_HEAT1, LOW);
Adafruit_Debounce btnHeat2(BTN_HEAT2, LOW);
Adafruit_Debounce btnProdStart(BTN_PROD_START, LOW);
Adafruit_Debounce btnProdEnd(BTN_PROD_END, LOW);
Adafruit_Debounce btnTapeFwd(BTN_TAPE_FWD, LOW);
Adafruit_Debounce btnTapeRev(BTN_TAPE_REV, LOW);
Adafruit_Debounce btnFailStop(BTN_FAIL_STOP, LOW);

#define BTN_MENU_RIGHT 31
#define BTN_MENU_LEFT 32
#define BTN_MENU_UP 33
#define BTN_MENU_DOWN 34
#define BTN_MENU_ENTER 35
Adafruit_Debounce btnMenuRight(BTN_MENU_RIGHT, LOW);
Adafruit_Debounce btnMenuLeft(BTN_PULL_IN, LOW);
Adafruit_Debounce btnMenuUp(BTN_MENU_UP, LOW);
Adafruit_Debounce btnMenuDown(BTN_MENU_DOWN, LOW);
Adafruit_Debounce btnMenuEnter(BTN_MENU_ENTER, LOW);

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
#define REM_PULL_OUT 4 
#define REM_PULL_IN 5
#define REM_TAPE_FWD 6
#define REM_TAPE_REV 7
#define REM_PROD_START 8
#define REM_PROD_STOP 11
#define REM_RESERVED_1 14
#define REM_RESERVED_2 15
#define REM_ALLOW_REMOTE 16
#define REM_START_SIGNAL 17
Adafruit_Debounce remPullOut(REM_PULL_OUT, LOW);
Adafruit_Debounce remPullIn(REM_PULL_IN, LOW);
Adafruit_Debounce remAllowRemote(REM_ALLOW_REMOTE, LOW);
Adafruit_Debounce remStartSignal(REM_START_SIGNAL, LOW);
Adafruit_Debounce remProdStart(REM_PROD_START, LOW);
Adafruit_Debounce remProdEnd(REM_PROD_STOP, LOW);
Adafruit_Debounce remTapeFwd(REM_TAPE_FWD, LOW);
Adafruit_Debounce remTapeRev(REM_TAPE_REV, LOW);
Adafruit_Debounce remFailStop(BTN_FAIL_STOP, LOW);


//LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

//------------ FOAM forming module -----------------
//DS18B20
#define DALLAS_SENSOR 36
OneWire oneWire(DALLAS_SENSOR);
DallasTemperature DSTemp(&oneWire);

#define FOAM_CHILLER_SIG 37
#define FOAM_PNEUMATIC_1 38
#define FOAM_PNEUMATIC_2 39
#define FOAM_HEAT_1 40
#define FOAM_HEAT_2 41
#define FOAM_BLOWER 42

//---------- Tape module ---------------
#define TAPE_ENGINE_INVERTER 12 
#define SIG_TAPE_RIGHT 43
#define SIG_TAPE_LEFT 44
#define TAPE_ENC_A 9
#define TAPE_ENC_B 10
#define TAPE_CURR_SENS PIN_A4

//puller SMALL
#define SIG_PULL_FWD 45
#define SIG_PULL_REV 46
#define PULL_CURR_SENS PIN_A5

//BLOWER 
#define BLOWER_PIN 49

//BUZZER
#define SPK_PIN 53

//STATIC CONFIG
#define HEATERS_START_DELAY 5000
#define HEATERS_LOWERING_DELAY 5000
#define BLOWER_SWITCH_OFF_DELAY 240000  //2 minuters blower cut off time

enum E_STATE {
	PIPE_LOAD,
	STARTING,
	PROCESS_RUN,
	COOLDOWN,
	
};
enum ES_START {
	HEATERS_ON,
	HEATERS_DOWN
};

VirtualDelay heaterStartDelay;
VirtualDelay pneumaticDelay;
VirtualDelay blowerSwitchOffDelay;

volatile ES_START _start_sub_state = ES_START::HEATERS_ON;
volatile E_STATE _state = E_STATE::STARTING;

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(115200);
	lcd.init(); // initialize the lcd
	lcd.clear();
	lcd.backlight();
	lcd.setCursor(0, 0);            // move cursor the first row
	lcd.print("  FOAM MASTER S  ");          // print message at the first row
	lcd.setCursor(0, 1);            // move cursor to the second row
	lcd.print(""); // print message at the second row
	lcd.setCursor(0, 2);            // move cursor to the third row
	lcd.print("software v0.1"); // print message at the second row

	btnPullOut.begin();
	btnPullIn.begin();
	btnHeat1.begin();
	btnHeat2.begin();
	btnProdStart.begin();
	btnProdEnd.begin();
	btnTapeFwd.begin();
	btnTapeRev.begin();
	btnFailStop.begin();

	btnMenuRight.begin();
	btnMenuLeft.begin();
	btnMenuUp.begin();
	btnMenuDown.begin();
	btnMenuEnter.begin();

	remPullOut.begin();
	remPullIn.begin();
	remAllowRemote.begin();
	remStartSignal.begin();
	remProdStart.begin();
	remProdEnd.begin();
	remTapeFwd.begin();
	remTapeRev.begin();
	remFailStop.begin();

	//pin setup
	pinMode(SPK_PIN, OUTPUT);

	pinMode(FOAM_HEAT_1, OUTPUT);
	pinMode(FOAM_HEAT_2, OUTPUT);

	pinMode(PULL_FWD, OUTPUT);
	pinMode(PULL_REV, OUTPUT);

	pinMode(BLOWER_PIN, OUTPUT);

	SetState(E_STATE::PIPE_LOAD);

}

// the loop function runs over and over again until power down or reset
void loop() {

	UpdateButtons();
	switch (_state)
	{
	case PIPE_LOAD:
	{
		lcd.setCursor(0, 0);
		lcd.print("Load Pipe....");          // print message at the first row
		DO_ONCE(Serial.println("Load Pipe...."));

		if (btnPullIn.isPressed())
		{
			Serial.println("Btn PULL IN ");
			digitalWrite(SIG_PULL_FWD, HIGH);
		}else digitalWrite(SIG_PULL_FWD, LOW);

		if (btnPullOut.isPressed())
		{
			Serial.println("Btn PULL REV ");
			digitalWrite(SIG_PULL_REV, HIGH);
		}else digitalWrite(SIG_PULL_REV, LOW);

		if (btnTapeFwd.isPressed())
		{
			Serial.println("Btn tape FWD");
			digitalWrite(SIG_TAPE_RIGHT, HIGH);
		}
		else digitalWrite(SIG_TAPE_RIGHT, LOW);

		if (btnTapeRev.isPressed())
		{
			Serial.println("Btn tape REV ");
			digitalWrite(SIG_TAPE_LEFT, HIGH);
		}
		else digitalWrite(SIG_TAPE_LEFT, LOW);


		if (btnProdStart.justPressed())
			SetState(E_STATE::STARTING);

	}break;
	case STARTING: {
		DO_ONCE(heaterStartDelay.start(HEATERS_START_DELAY)); //5s Wait before heater starts
		if(digitalRead(BLOWER_PIN) == LOW) digitalWrite(BLOWER_PIN, HIGH); // Make sure the blower is ALWAYS ON !!

		if (heaterStartDelay.elapsed())
		{
			
			DO_ONCE(pneumaticDelay.start(HEATERS_LOWERING_DELAY));
			lcd.setCursor(0, 0);            
			lcd.print("Heaters starting...");         
			Serial.println("Heaters start");
			digitalWrite(FOAM_HEAT_1, HIGH); 
			//here we can add potential delay to second heater
			digitalWrite(FOAM_HEAT_2, HIGH);

		}
		if (pneumaticDelay.elapsed())
		{

			Serial.println("Heaters Down");
			lcd.setCursor(0, 1);
			lcd.print("Heaters down...");
			digitalWrite(FOAM_PNEUMATIC_1, HIGH);
			digitalWrite(FOAM_PNEUMATIC_2, HIGH);
			SetState(E_STATE::PROCESS_RUN);
		}

		

	}break;
	case PROCESS_RUN:
	{

		lcd.setCursor(0, 0);
		lcd.print(" !! RUNNING !!..");
		Serial.println("RUNNING ");		

		while (!btnProdEnd.isPressed())
		{
			btnProdEnd.update();
			if (btnProdEnd.isPressed()) break;
			digitalWrite(SIG_TAPE_LEFT, HIGH);
			analogWrite(TAPE_ENGINE_INVERTER, 32);// ehre we replace the value from the variable stored in eeprom .
			lcd.setCursor(0, 1);
			lcd.print("Speed....");
		}
		SetState(E_STATE::COOLDOWN);
		
		

	}
	case COOLDOWN:
	{
		lcd.setCursor(0, 0);
		lcd.print("Cooldown..");
		Serial.println("Cooldown started");
		Serial.println("Heaters UP");
		lcd.setCursor(0, 1);
		lcd.print("Heaters STOP...");
	
		
		digitalWrite(FOAM_HEAT_1, LOW);
		//here we can add potential delay to second heater
		digitalWrite(FOAM_HEAT_2, LOW);
		
		
		digitalWrite(FOAM_PNEUMATIC_1, LOW);
		digitalWrite(FOAM_PNEUMATIC_2, LOW);		
		

		DO_ONCE(blowerSwitchOffDelay.start(BLOWER_SWITCH_OFF_DELAY));
		if (blowerSwitchOffDelay.elapsed())
		{
			digitalWrite(BLOWER_PIN, LOW); //2 minutes after all is finished we switch BLOWER OFF
			Serial.println("BLOWERS OFF");
		}

	}
	
	default:
		break;
	}
	
}

void UpdateButtons()
{
	btnPullOut.update();
	btnPullIn.update();
	btnHeat1.update();
	btnHeat2.update();
	btnProdStart.update();
	btnProdEnd.update();
	btnTapeFwd.update();
	btnTapeRev.update();
	btnFailStop.update();

	btnMenuRight.update();
	btnMenuLeft.update();
	btnMenuUp.update();
	btnMenuDown.update();
	btnMenuEnter.update();

	remPullOut.update();
	remPullIn.update();
	remAllowRemote.update();
	remStartSignal.update();
	remProdStart.update();
	remProdEnd.update();
	remTapeFwd.update();
	remTapeRev.update();
	remFailStop.update();
}

void SetState(E_STATE newState)
{
	Beep();
	_state = _state != newState ? newState : _state;
	lcd.clear();
}

void Beep()
{
	int x = 0;
	while (x < 50)
	{
		analogWrite(SPK_PIN, 255);
		delay(1);
		analogWrite(SPK_PIN, 0);
		x++;
		delay(1);

	}
}

