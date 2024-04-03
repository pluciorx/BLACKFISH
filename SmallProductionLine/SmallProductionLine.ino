#include <NewEncoder.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Debounce.h>
#include <LiquidCrystal_I2C.h>
#include <avdweb_VirtualDelay.h>

//---------- Control Panel Module ---------------
// PANEL BUTTONS
#define BTN_PULL_RIGHT 22
#define BTN_PULL_LEFT 23
#define BTN_HEAT1 24
#define BTN_HEAT2 25
#define BTN_PROD_START 26
#define BTN_PROD_END 27
#define BTN_TAPE_LEFT 28
#define BTN_TAPE_RIGHT 29
#define BTN_FAIL_STOP 30

Adafruit_Debounce btnPullRight(BTN_PULL_RIGHT, LOW);
Adafruit_Debounce btnPullLeft(BTN_PULL_LEFT,LOW);
Adafruit_Debounce btnHeat1(BTN_HEAT1, LOW);
Adafruit_Debounce btnHeat2(BTN_HEAT2, LOW);
Adafruit_Debounce btnProdStart(BTN_PROD_START, LOW);
Adafruit_Debounce btnProdEnd(BTN_PROD_END, LOW);
Adafruit_Debounce btnTapeRight(BTN_TAPE_RIGHT, LOW);
Adafruit_Debounce btnTapeLeft(BTN_TAPE_LEFT, LOW);
Adafruit_Debounce btnFailStop(BTN_FAIL_STOP, LOW);

#define BTN_MENU_RIGHT 31
#define BTN_MENU_LEFT 32
#define BTN_MENU_UP 33
#define BTN_MENU_DOWN 34
#define BTN_MENU_ENTER 35
Adafruit_Debounce btnMenuRight(BTN_MENU_RIGHT, LOW);
Adafruit_Debounce btnMenuLeft(BTN_MENU_LEFT, LOW);
Adafruit_Debounce btnMenuUp(BTN_MENU_UP, LOW);
Adafruit_Debounce btnMenuDown(BTN_MENU_DOWN, LOW);
Adafruit_Debounce btnMenuEnter(BTN_MENU_ENTER, LOW);

//Panel Lights
#define LED_PULL_RIGHT PIN_A8
#define LED_PULL_LEFT PIN_A9
#define LED_HEAT1 PIN_A10
bool isHeat1ON = false;
bool isHeat2ON = false;

#define LED_HEAT2 PIN_A11
#define LED_PROD_START PIN_A12
#define LED_PROD_END PIN_A13
#define LED_TAPE_RIGHT PIN_A14
#define LED_TAPE_LEFT PIN_A15

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
#define SIG_FOAM_HEAT_1 40
#define FOAM_HEAT_1_TEMP_AL1_TRIG  PIN_A1 // alarm dolny
#define FOAM_HEAT_1_TEMP_AL2_TRIG  PIN_A2 // alarm gorny 

#define SIG_FOAM_HEAT_2 41
#define FOAM_HEAT_2_TEMP_AL1_TRIG  PIN_A4 //alarm dolny H2
#define FOAM_HEAT_2_TEMP_AL2_TRIG  PIN_A5 //alarm gorny

#define FOAM_BLOWER 42

//---------- Tape module ---------------
#define TAPE_ENGINE_INVERTER 12 

#define SIG_TAPE_RIGHT 43
#define SIG_TAPE_LEFT 44

NewEncoder encTapeSpeed;
#define TAPE_ENC_A 9
#define TAPE_ENC_B 10
#define TAPE_CURR_SENS PIN_A6

//puller SMALL
#define SIG_PULL_LEFT 45
#define SIG_PULL_RIGHT 46
#define PULL_CURR_SENS PIN_A7

//BLOWER 
#define SIG_BLOWER_PIN 42

//BUZZER
#define SPK_PIN 53

//STATIC CONFIG
#define HEATERS_START_DELAY 5000
#define HEATERS_LOWERING_DELAY 6000
#define BLOWER_SWITCH_OFF_DELAY 20000  //2 minuters blower cut off time

#define INPUT_PULLDOWN

void(*resetFunc) (void) = 0;

enum E_STATE {
	PIPE_LOAD,
	STARTING,
	PROCESS_RUN,
	COOLDOWN,
	
};
//enum ES_START {
//	HEATERS_ON,
//	HEATERS_DOWN
//};

VirtualDelay heaterStartDelay;
VirtualDelay blowerSwitchOffDelay;

//volatile ES_START _start_sub_state = ES_START::HEATERS_ON;
volatile E_STATE _state = E_STATE::STARTING;

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(115200);
	lcd.init(); // initialize the lcd
	lcd.clear();
	lcd.backlight();
	lcd.clear();
	lcd.clear();
	lcd.setCursor(0, 0);            // move cursor the first row
	lcd.print("  FOAM MASTER S  ");          // print message at the first row
	lcd.setCursor(0, 1);            // move cursor to the second row
	lcd.print(""); // print message at the second row
	lcd.setCursor(0, 2);            // move cursor to the third row
	lcd.print("software v0.1"); // print message at the second row
	delay(1500);

	btnPullRight.begin();
	btnPullLeft.begin();
	btnHeat1.begin();
	btnHeat2.begin();
	btnProdStart.begin();
	btnProdEnd.begin();
	btnTapeLeft.begin();
	btnTapeRight.begin();
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

	pinMode(FOAM_HEAT_1_TEMP_AL1_TRIG, INPUT_PULLUP);
	pinMode(FOAM_HEAT_1_TEMP_AL2_TRIG, INPUT_PULLUP);
	pinMode(SIG_FOAM_HEAT_1, OUTPUT);
	digitalWrite(SIG_FOAM_HEAT_1, LOW);

	pinMode(FOAM_HEAT_2_TEMP_AL1_TRIG, INPUT_PULLUP);
	pinMode(FOAM_HEAT_2_TEMP_AL2_TRIG, INPUT_PULLUP);
	pinMode(SIG_FOAM_HEAT_2, OUTPUT);
	digitalWrite(SIG_FOAM_HEAT_2, LOW);

	pinMode(SIG_PULL_LEFT, OUTPUT);
	pinMode(SIG_PULL_RIGHT, OUTPUT);
	digitalWrite(SIG_PULL_LEFT, LOW);
	digitalWrite(SIG_PULL_RIGHT, LOW);


	pinMode(SIG_TAPE_RIGHT, OUTPUT);
	pinMode(SIG_TAPE_LEFT, OUTPUT);
	digitalWrite(SIG_TAPE_RIGHT, LOW);
	digitalWrite(SIG_TAPE_RIGHT, LOW);


	pinMode(SIG_BLOWER_PIN, OUTPUT);
	pinMode(FOAM_PNEUMATIC_1, OUTPUT);
	pinMode(FOAM_PNEUMATIC_2, OUTPUT);
	digitalWrite(SIG_BLOWER_PIN, LOW);
	digitalWrite(FOAM_PNEUMATIC_1, LOW);
	digitalWrite(FOAM_PNEUMATIC_2, LOW);

	//LED
	pinMode(LED_PULL_RIGHT, OUTPUT);
	pinMode(LED_PULL_LEFT, OUTPUT);
	digitalWrite(LED_PULL_RIGHT, LOW);
	digitalWrite(LED_PULL_LEFT, LOW);


	pinMode(LED_HEAT1, OUTPUT);
	pinMode(LED_HEAT2, OUTPUT);
	digitalWrite(LED_HEAT1, LOW);
	digitalWrite(LED_HEAT2, LOW);

	pinMode(LED_PROD_START, OUTPUT);
	pinMode(LED_PROD_END, OUTPUT);
	pinMode(LED_TAPE_RIGHT, OUTPUT);
	pinMode(LED_TAPE_LEFT, OUTPUT);

	pinMode(TAPE_ENGINE_INVERTER, OUTPUT);  // sets the pin as output

	analogWrite(TAPE_ENGINE_INVERTER, 255);
	digitalWrite(LED_PROD_START, LOW);
	digitalWrite(LED_PROD_END, LOW);
	digitalWrite(FOAM_PNEUMATIC_2, LOW);

	encTapeSpeed.begin(TAPE_ENC_A, TAPE_ENC_B);

	SetState(E_STATE::PIPE_LOAD);

}

// the loop function runs over and over again until power down or reset
void loop() {
	UpdateButtons();
	switch (_state)
	{
	case PIPE_LOAD:
	{
		Serial.println("Load Pipe....");
		lcd.setCursor(0, 0);
		lcd.print("Load Pipe...    ");          // print message at the first row
		analogWrite(TAPE_ENGINE_INVERTER, 255);

		
		digitalWrite(LED_PULL_RIGHT, LOW);
		digitalWrite(LED_PULL_LEFT, LOW);
		
		while (!btnProdStart.isPressed())
		{
			UpdateButtons();
			while (btnPullLeft.isPressed())
			{
				Serial.println("btnPullLeft pressed");
				digitalWrite(SIG_PULL_RIGHT, LOW);
				digitalWrite(LED_PULL_RIGHT, LOW);
				digitalWrite(SIG_PULL_LEFT, HIGH);
				digitalWrite(LED_PULL_LEFT, HIGH);

				btnPullLeft.update();
			}

			while (btnPullRight.isPressed())
			{
				Serial.println("btnPullRight pressed");
				digitalWrite(SIG_PULL_LEFT, LOW);
				digitalWrite(LED_PULL_LEFT, LOW);
				digitalWrite(SIG_PULL_RIGHT, HIGH);
				digitalWrite(LED_PULL_RIGHT, HIGH);
				btnPullRight.update();
			}

			while (btnTapeRight.isPressed())
			{
				analogWrite(TAPE_ENGINE_INVERTER, 255);
				Serial.println("btnTapeRight pressed");
				digitalWrite(SIG_TAPE_LEFT, LOW);
				digitalWrite(LED_TAPE_LEFT, LOW);
				digitalWrite(SIG_TAPE_RIGHT, HIGH);
				digitalWrite(LED_TAPE_RIGHT, HIGH);
				btnTapeRight.update();
			}

			while (btnTapeLeft.isPressed())
			{
				analogWrite(TAPE_ENGINE_INVERTER, 255);
				Serial.println("btnTapeLeft pressed");
				digitalWrite(SIG_TAPE_RIGHT, LOW);
				digitalWrite(LED_TAPE_RIGHT, LOW);
				digitalWrite(SIG_TAPE_LEFT, HIGH);
				digitalWrite(LED_TAPE_LEFT, HIGH);
				btnTapeLeft.update();
			}
			
		}

			Serial.println("btnProdStart pressed");
			digitalWrite(SIG_TAPE_RIGHT, LOW);
			digitalWrite(LED_TAPE_RIGHT, LOW);

			digitalWrite(SIG_TAPE_LEFT, LOW);
			digitalWrite(LED_TAPE_LEFT, LOW);

			digitalWrite(SIG_PULL_LEFT, LOW);
			digitalWrite(LED_PULL_LEFT, LOW);

			digitalWrite(SIG_PULL_RIGHT, LOW);
			digitalWrite(LED_PULL_RIGHT, LOW);

			SetState(E_STATE::STARTING);
	}break;
	case STARTING: {
		
		heaterStartDelay.start(HEATERS_START_DELAY); //5s Wait before heater starts
		Serial.println("BLOWER ON");
		lcd.setCursor(0, 0);
		lcd.print("Blower starting...");
		digitalWrite(SIG_BLOWER_PIN, HIGH); // Make sure the blower is ALWAYS ON !!
		digitalWrite(LED_PROD_START, HIGH);
		digitalWrite(LED_PROD_END, LOW);
		btnProdEnd.update();
		while (!btnProdEnd.isPressed())
		{
			if (heaterStartDelay.elapsed())
			{

				lcd.setCursor(0, 1);
				lcd.print("Heaters starting...");
				Serial.println("Heaters start");
				Serial.print("PID H1 (A2) AL1:"); Serial.println(digitalRead(FOAM_HEAT_1_TEMP_AL1_TRIG));
				Serial.print("PID H1 (A1) AL2:"); Serial.println(digitalRead(FOAM_HEAT_1_TEMP_AL2_TRIG));

				if (!digitalRead(FOAM_HEAT_1_TEMP_AL2_TRIG))
				{
					digitalWrite(SIG_FOAM_HEAT_1, HIGH);
					isHeat1ON = true;
					Serial.println("Heater 1 ON");
				}
				//here we can add potential delay to second heater
				Serial.print("PID H2 (A4) AL1:"); Serial.println(digitalRead(FOAM_HEAT_2_TEMP_AL1_TRIG));
				Serial.print("PID H2 (A5) AL2:"); Serial.println(digitalRead(FOAM_HEAT_2_TEMP_AL2_TRIG));

				if (!digitalRead(FOAM_HEAT_2_TEMP_AL2_TRIG))
				{
					digitalWrite(SIG_FOAM_HEAT_2, HIGH);
					isHeat2ON = true;
					Serial.println("Heater 2 ON");
				}
				Serial.println("Waiting for temp to reach treshold");
			}

			if (!digitalRead(FOAM_HEAT_2_TEMP_AL1_TRIG) && !digitalRead(FOAM_HEAT_1_TEMP_AL1_TRIG))
			{
				lcd.setCursor(0, 2);
				lcd.print("Heaters starting...");
				Serial.println("Heaters Down");
				lcd.setCursor(0, 2);
				lcd.print("Heaters down...");
				digitalWrite(FOAM_PNEUMATIC_1, HIGH);
				digitalWrite(FOAM_PNEUMATIC_2, HIGH);
				btnProdEnd.update();
				SetState(E_STATE::PROCESS_RUN);
				
				break;
			}
			btnProdEnd.update();
		}
		if (btnProdEnd.isPressed()) SetState(E_STATE::PIPE_LOAD);
		
		
		
	}break;
	case PROCESS_RUN:
	{
		
		lcd.setCursor(0, 0);
		lcd.print(" !! RUNNING !! ");
		Serial.println("RUNNING ");
		digitalWrite(LED_PROD_START, HIGH);
		digitalWrite(LED_PROD_END, LOW);

		while (!btnProdEnd.isPressed())
		{
			digitalWrite(SIG_TAPE_LEFT, HIGH);
			digitalWrite(SIG_TAPE_RIGHT, LOW);

			analogWrite(TAPE_ENGINE_INVERTER, 255);
			lcd.setCursor(0, 1);
			lcd.print("Speed:");

			if (!digitalRead(FOAM_HEAT_1_TEMP_AL2_TRIG) && digitalRead(FOAM_HEAT_1_TEMP_AL1_TRIG)) {
				Serial.println("Heater 1 ON");
				digitalWrite(SIG_FOAM_HEAT_1, HIGH);
				digitalWrite(LED_HEAT1, HIGH);
			}
			else {
				Serial.println("Heater 1 OFF");
				digitalWrite(LED_HEAT1, LOW);
				digitalWrite(SIG_FOAM_HEAT_1, LOW);
			}

			if (!digitalRead(FOAM_HEAT_2_TEMP_AL2_TRIG) && digitalRead(FOAM_HEAT_2_TEMP_AL1_TRIG)) {
				Serial.println("Heater 2 ON");
				digitalWrite(SIG_FOAM_HEAT_2, HIGH);
				digitalWrite(LED_HEAT2, HIGH);

			}
			else {
				Serial.println("Heater 2 OFF");
				digitalWrite(SIG_FOAM_HEAT_2, LOW);
				digitalWrite(LED_HEAT2, LOW);
			}
			btnProdEnd.update();
		}
		
		Serial.println("btnProdEnd pressed");
		
		SetState(E_STATE::COOLDOWN);
	
	}
	case COOLDOWN:
	{
		digitalWrite(LED_PROD_START, LOW);
		digitalWrite(LED_PROD_END, HIGH);
		lcd.setCursor(0, 0);
		lcd.print("Cooldown starting");
		
		Serial.println("Cooldown started");
		lcd.setCursor(0, 1);
		lcd.print("Heaters STOP... ");
		Serial.println("Heaters Stop");
		digitalWrite(SIG_FOAM_HEAT_1, LOW);
		digitalWrite(SIG_FOAM_HEAT_2, LOW);
		digitalWrite(LED_HEAT1, LOW);
		digitalWrite(LED_HEAT2, LOW);
	
		
		lcd.setCursor(0, 2);
		lcd.print("Heaters UP... ");
		Serial.println("Heaters Up");
		digitalWrite(FOAM_PNEUMATIC_1, LOW);
		digitalWrite(FOAM_PNEUMATIC_2, LOW);

		lcd.setCursor(0, 3);
		lcd.print("Tape Stop... ");
		Serial.println("Tape Stop");
		analogWrite(TAPE_ENGINE_INVERTER, 0);

		digitalWrite(LED_PROD_END, HIGH);
		digitalWrite(LED_PROD_START, LOW);
		btnProdStart.update();

		blowerSwitchOffDelay.start(BLOWER_SWITCH_OFF_DELAY);
		while (!blowerSwitchOffDelay.elapsed());
		digitalWrite(SIG_BLOWER_PIN, LOW); //2 minutes after all is finished we switch BLOWER OFF
		Serial.println("BLOWERS OFF");

		while (!btnProdStart.isPressed()) {
			
			btnProdStart.update();
		}
		SetState(E_STATE::PIPE_LOAD);
		
		
	}
	
	default:
		break;
	}
	
}


void UpdateButtons()
{
	btnPullRight.update();
	btnPullLeft.update();
	btnHeat1.update();
	btnHeat2.update();
	btnProdStart.update();
	btnProdEnd.update();
	btnTapeLeft.update();
	btnTapeRight.update();
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
	lcd.clear();
	Beep();
	_state = _state != newState ? newState : _state;
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

