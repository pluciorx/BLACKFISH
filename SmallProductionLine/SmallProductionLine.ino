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
#define BTN_TAPE_LEFT 29
#define BTN_TAPE_RIGHT 28
#define BTN_FAIL_STOP 30

Adafruit_Debounce btnPullRight(BTN_PULL_RIGHT, LOW);
Adafruit_Debounce btnPullLeft(BTN_PULL_LEFT, LOW);
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


bool isHeat1ON = false;
bool isHeat2ON = false;

#define LED_HEAT1 PIN_A10
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
//Adafruit_Debounce remAllowRemote(REM_ALLOW_REMOTE, LOW);
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
#define TAPE_SLOW_SPEED 32
#define TAPE_PROD_SPEED 64
#define SIG_TAPE_RIGHT 43
#define SIG_TAPE_LEFT 44

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
#define BLOWER_SWITCH_OFF_DELAY 5000  //2 minuters blower cut off time

//#define INPUT_PULLDOWN

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
VirtualDelay btnStop3sCounterl;
VirtualDelay blowerSwitchOffDelay;

//volatile ES_START _start_sub_state = ES_START::HEATERS_ON;
volatile E_STATE _state = E_STATE::STARTING;

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(115200);

	lcd.init(); // initialize the lcd	
	lcd.backlight();
	lcd.clear();
	//lcd.setCursor(0, 0);            // move cursor the first row
	//lcd.print("     BLACKFISH   ");          // print message at the first row
	//lcd.setCursor(0, 1);            // move cursor to the second row
	//lcd.print("   FOAM MASTER S   "); // print message at the second row
	//lcd.setCursor(0, 2);            // move cursor to the third row
	//lcd.print("v0.51"); // print message at the second row

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
	//remAllowRemote.begin();
	remStartSignal.begin();
	remProdStart.begin();
	remProdEnd.begin();
	remTapeFwd.begin();
	remTapeRev.begin();
	remFailStop.begin();

	//pin setup
	pinMode(SPK_PIN, OUTPUT);
	digitalWrite(SPK_PIN, LOW);

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
	digitalWrite(SIG_TAPE_LEFT, LOW);

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
	digitalWrite(LED_PROD_START, LOW);
	digitalWrite(LED_PROD_END, LOW);
	digitalWrite(LED_TAPE_RIGHT, LOW);
	digitalWrite(LED_TAPE_LEFT, LOW);

	pinMode(TAPE_ENGINE_INVERTER, OUTPUT);  // sets the pin as output
	//analogWrite(TAPE_ENGINE_INVERTER, 255);

	delay(200);

	SetState(E_STATE::PIPE_LOAD);
}

// the loop function runs over and over again until power down or reset
void loop() {
	UpdateButtons();
	switch (_state)
	{
	case PIPE_LOAD:
	{
		lcd.clear();
		lcd.backlight();
		
		lcd.setCursor(0, 0);
		lcd.print("Please press START");          // print message at the first row
		analogWrite(TAPE_ENGINE_INVERTER, TAPE_SLOW_SPEED);

		digitalWrite(LED_PULL_RIGHT, LOW);
		digitalWrite(LED_PULL_LEFT, LOW);
		digitalWrite(LED_PROD_START, LOW);
		digitalWrite(LED_PROD_END, HIGH);
		digitalWrite(SIG_FOAM_HEAT_2, LOW);

		digitalWrite(LED_HEAT2, LOW);
		digitalWrite(SIG_FOAM_HEAT_1, LOW);
		digitalWrite(LED_HEAT1, LOW);
		delay(100);

		int endCounter = 0;
		while (endCounter < 3)
		{
			UpdateButtons();

			if (btnHeat1.isPressed())
			{
				Serial.println("btnHeat1 pressed");
				digitalWrite(FOAM_PNEUMATIC_1, HIGH);
				digitalWrite(LED_HEAT1, HIGH);
			}
			else
			{
				digitalWrite(FOAM_PNEUMATIC_1, LOW);
				digitalWrite(LED_HEAT1, LOW);
			}

			if (btnHeat2.isPressed())
			{
				Serial.println("btnHeat2 pressed");
				digitalWrite(FOAM_PNEUMATIC_2, HIGH);
				digitalWrite(LED_HEAT2, HIGH);
			}
			else
			{
				digitalWrite(FOAM_PNEUMATIC_2, LOW);
				digitalWrite(LED_HEAT2, LOW);
			}

			if (btnPullLeft.isPressed())
			{
				Serial.println("btnPullLeft pressed");

				digitalWrite(SIG_PULL_RIGHT, LOW);
				digitalWrite(SIG_PULL_RIGHT, LOW);
				digitalWrite(LED_PULL_LEFT, HIGH);
			}
			else
			{
				digitalWrite(SIG_PULL_LEFT, LOW);
				digitalWrite(LED_PULL_LEFT, LOW);
			}

			if (btnPullRight.isPressed())
			{
				digitalWrite(SIG_PULL_LEFT, LOW);
				Serial.println("btnPullRight pressed");
				digitalWrite(SIG_PULL_RIGHT, HIGH);
				digitalWrite(LED_PULL_RIGHT, HIGH);
			}
			else
			{
				digitalWrite(SIG_PULL_RIGHT, LOW);
				digitalWrite(LED_PULL_RIGHT, LOW);
			}

			if (btnTapeRight.isPressed())
			{

				Serial.println("btnTapeRight pressed");
				digitalWrite(SIG_TAPE_RIGHT, HIGH);
				digitalWrite(SIG_TAPE_LEFT, LOW);
				digitalWrite(LED_TAPE_RIGHT, HIGH);
			}
			else
			{
				digitalWrite(SIG_TAPE_RIGHT, LOW);
				digitalWrite(LED_TAPE_RIGHT, LOW);
			}

			if (btnTapeLeft.isPressed())
			{

				Serial.println("btnTapeLeft pressed");
				digitalWrite(SIG_TAPE_RIGHT, LOW);
				digitalWrite(SIG_TAPE_LEFT, HIGH);
				digitalWrite(LED_TAPE_LEFT, HIGH);

			}
			else {
				digitalWrite(SIG_TAPE_LEFT, LOW);
				digitalWrite(LED_TAPE_LEFT, LOW);
			}

			btnProdStart.update();
			if (btnProdStart.isPressed())
			{
				Serial.println("Btn Start");
				if (endCounter == 0) btnStop3sCounterl.start(500);
				if (btnStop3sCounterl.elapsed())
				{

					btnStop3sCounterl.start(500);
					endCounter++;

				}
			}
			if (!btnProdStart.isPressed()) endCounter = 0;
		}
		Serial.println("btnProdStart pressed");
		SetState(E_STATE::STARTING);
	}break;
	case STARTING: {
		lcd.clear();
		heaterStartDelay.start(HEATERS_START_DELAY); //5s Wait before heater starts
		Serial.println("BLOWER ON");
		lcd.setCursor(0, 0);
		lcd.print("Blower starting...");
		digitalWrite(SIG_BLOWER_PIN, HIGH); // Make sure the blower is ALWAYS ON !!
		delay(100);
		digitalWrite(LED_PROD_START, HIGH);
		digitalWrite(LED_PROD_END, LOW);

		while (!btnProdEnd.isPressed())
		{
			UpdateButtons();
			if (heaterStartDelay.elapsed())
			{
				digitalWrite(SIG_BLOWER_PIN, HIGH); // Make sure the blower is ALWAYS ON !!
				lcd.setCursor(0, 1);
				lcd.print("Heaters starting...");
				Serial.println("Heaters start");
				Serial.print("PID H1 (A2) AL1:"); Serial.println(digitalRead(FOAM_HEAT_1_TEMP_AL1_TRIG));
				Serial.print("PID H1 (A1) AL2:"); Serial.println(digitalRead(FOAM_HEAT_1_TEMP_AL2_TRIG));

				if (digitalRead(FOAM_HEAT_1_TEMP_AL1_TRIG))
				{

					digitalWrite(SIG_FOAM_HEAT_1, HIGH);
					digitalWrite(LED_HEAT2, HIGH);
					isHeat1ON = true;
					Serial.println("Heater 1 ON");
				}
				else
				{
					digitalWrite(LED_HEAT2, LOW);
				}
				//here we can add potential delay to second heater
				Serial.print("PID H2 (A4) AL1:"); Serial.println(digitalRead(FOAM_HEAT_2_TEMP_AL1_TRIG));
				Serial.print("PID H2 (A5) AL2:"); Serial.println(digitalRead(FOAM_HEAT_2_TEMP_AL2_TRIG));

				if (digitalRead(FOAM_HEAT_2_TEMP_AL1_TRIG))
				{

					digitalWrite(SIG_FOAM_HEAT_2, HIGH);
					digitalWrite(LED_HEAT1, HIGH);
					isHeat2ON = true;
					Serial.println("Heater 2 ON");
				}
				else
				{
					digitalWrite(LED_HEAT1, HIGH);
				}
				Serial.println("Waiting for temp to reach treshold");

				if (btnTapeRight.isPressed())
				{

					Serial.println("btnTapeRight pressed");
					digitalWrite(SIG_TAPE_RIGHT, HIGH);
					digitalWrite(SIG_TAPE_LEFT, LOW);
					digitalWrite(LED_TAPE_RIGHT, HIGH);
				}
				else
				{
					digitalWrite(SIG_TAPE_RIGHT, LOW);
					digitalWrite(LED_TAPE_RIGHT, LOW);
				}

				if (btnTapeLeft.isPressed())
				{

					Serial.println("btnTapeLeft pressed");
					digitalWrite(SIG_TAPE_RIGHT, LOW);
					digitalWrite(SIG_TAPE_LEFT, HIGH);
					digitalWrite(LED_TAPE_LEFT, HIGH);

				}
				else {
					digitalWrite(SIG_TAPE_LEFT, LOW);
					digitalWrite(LED_TAPE_LEFT, LOW);
				}

				bool isH1Ready = false, isH2Ready = false;
				E_STATE nextState = E_STATE::PROCESS_RUN;
				while ((!isH1Ready || !isH2Ready) && !btnProdEnd.isPressed())
				{
					UpdateButtons();
					if (btnProdEnd.isPressed())
					{
						nextState = E_STATE::PIPE_LOAD;
						break;
					}
					if (digitalRead(FOAM_HEAT_2_TEMP_AL1_TRIG))
					{
						digitalWrite(SIG_FOAM_HEAT_2, LOW);
						digitalWrite(LED_HEAT1, LOW);
						Serial.println("H2 reached temp");
						isH2Ready = true;
					}
					else
					{
						digitalWrite(SIG_FOAM_HEAT_2, HIGH);
						digitalWrite(LED_HEAT1, HIGH);
					}

					if (digitalRead(FOAM_HEAT_1_TEMP_AL1_TRIG))
					{
						digitalWrite(SIG_FOAM_HEAT_1, LOW);
						digitalWrite(LED_HEAT2, LOW);
						Serial.println("H1 reached temp");
						isH1Ready = true;
					}
					else
					{
						digitalWrite(SIG_FOAM_HEAT_1, HIGH);
						digitalWrite(LED_HEAT2, HIGH);
					}
					if (btnTapeRight.isPressed())
					{

						Serial.println("btnTapeRight pressed");
						digitalWrite(SIG_TAPE_RIGHT, HIGH);
						digitalWrite(SIG_TAPE_LEFT, LOW);
						digitalWrite(LED_TAPE_RIGHT, HIGH);
					}
					else
					{
						digitalWrite(SIG_TAPE_RIGHT, LOW);
						digitalWrite(LED_TAPE_RIGHT, LOW);
					}

					if (btnTapeLeft.isPressed())
					{

						Serial.println("btnTapeLeft pressed");
						digitalWrite(SIG_TAPE_RIGHT, LOW);
						digitalWrite(SIG_TAPE_LEFT, HIGH);
						digitalWrite(LED_TAPE_LEFT, HIGH);

					}
					else {
						digitalWrite(SIG_TAPE_LEFT, LOW);
						digitalWrite(LED_TAPE_LEFT, LOW);
					}


				}

				SetState(nextState);

				digitalWrite(FOAM_PNEUMATIC_1, HIGH);
				digitalWrite(FOAM_PNEUMATIC_2, HIGH);
				break;
			}
		}
		if (btnProdEnd.isPressed()) SetState(E_STATE::PIPE_LOAD);



	}break;
	case PROCESS_RUN:
	{
		lcd.clear();
		btnProdEnd.update();
		lcd.setCursor(0, 0);
		lcd.print(" !! RUNNING !! ");
		Serial.println("RUNNING");
		digitalWrite(LED_PROD_START, HIGH);
		digitalWrite(LED_PROD_END, LOW);

		analogWrite(TAPE_ENGINE_INVERTER, TAPE_PROD_SPEED);
		delay(200);
		digitalWrite(SIG_TAPE_LEFT, HIGH);
		digitalWrite(SIG_TAPE_RIGHT, LOW);

		digitalWrite(SIG_BLOWER_PIN, HIGH); // Make sure the blower is ALWAYS ON !!		

		float endCounter = 0;
		while (endCounter < 3)
		{
			if (digitalRead(FOAM_HEAT_1_TEMP_AL1_TRIG)) {

				digitalWrite(SIG_FOAM_HEAT_1, LOW);
				digitalWrite(LED_HEAT2, LOW);
			}
			else {

				digitalWrite(LED_HEAT2, HIGH);
				digitalWrite(SIG_FOAM_HEAT_1, HIGH);
			}

			if (digitalRead(FOAM_HEAT_2_TEMP_AL1_TRIG)) {

				digitalWrite(SIG_FOAM_HEAT_2, LOW);
				digitalWrite(LED_HEAT1, LOW);

			}
			else {

				digitalWrite(SIG_FOAM_HEAT_2, HIGH);
				digitalWrite(LED_HEAT1, HIGH);
			}

			btnProdEnd.update();
			if (btnProdEnd.isPressed())
			{
				Serial.println("Btn STP");
				if (endCounter == 0) btnStop3sCounterl.start(100);
				if (btnStop3sCounterl.elapsed())
				{
					Serial.print("Counter stop:"); Serial.println(endCounter);
					btnStop3sCounterl.start(100);
					endCounter++;
				}
			}
			if (!btnProdEnd.isPressed()) endCounter = 0;
			btnProdEnd.update();
		}

		Serial.println("btnProdEnd pressed");

		digitalWrite(FOAM_PNEUMATIC_1, LOW);
		digitalWrite(FOAM_PNEUMATIC_2, LOW);

		digitalWrite(SIG_FOAM_HEAT_1, LOW);
		digitalWrite(SIG_FOAM_HEAT_2, LOW);

		digitalWrite(SIG_BLOWER_PIN, LOW);
		delay(500);

		digitalWrite(SIG_TAPE_LEFT, LOW);
		digitalWrite(SIG_TAPE_RIGHT, LOW);
		delay(1000);
		SetState(E_STATE::COOLDOWN);

	}break;
	case COOLDOWN:
	{
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("Cool down started");
		lcd.setCursor(0, 1);
		lcd.print("Cool down complete");

		lcd.setCursor(0, 3);
		lcd.print("--- Press start ---");
		int endCounter = 0;
		while (endCounter < 3)
		{
			UpdateButtons();
			if (btnProdStart.isPressed())
			{
				Serial.println("Btn Start");
				if (endCounter == 0) btnStop3sCounterl.start(200);
				if (btnStop3sCounterl.elapsed())
				{

					btnStop3sCounterl.start(200);
					endCounter++;

				}
			}
			if (!btnProdStart.isPressed()) endCounter = 0;
		}
		SetState(E_STATE::PIPE_LOAD);
	}break;
	default: {
		SetState(E_STATE::PIPE_LOAD);
	}
	}

}

void UpdateRemoteButtons()
{
	if (digitalRead(REM_ALLOW_REMOTE)) return;
	//Adafruit_Debounce remPullOut(REM_PULL_OUT, LOW);
	//Adafruit_Debounce remPullIn(REM_PULL_IN, LOW);
	//Adafruit_Debounce remAllowRemote(REM_ALLOW_REMOTE, LOW);
	//Adafruit_Debounce remStartSignal(REM_START_SIGNAL, LOW);
	//Adafruit_Debounce remProdStart(REM_PROD_START, LOW);
	//Adafruit_Debounce remProdEnd(REM_PROD_STOP, LOW);
	//Adafruit_Debounce remTapeFwd(REM_TAPE_FWD, LOW);
	//Adafruit_Debounce remTapeRev(REM_TAPE_REV, LOW);
	//Adafruit_Debounce remFailStop(BTN_FAIL_STOP, LOW);

	remPullOut.update();
	remPullIn.update();
	remStartSignal.update();
	remProdStart.update();
	remProdEnd.update();
	remTapeFwd.update();
	remTapeRev.update();
	remFailStop.update();

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
}

void SetState(E_STATE newState)
{
	lcd.clear();
	Beep();
	delay(100);
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

