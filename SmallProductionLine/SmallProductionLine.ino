#include <Adafruit_Debounce.h>
#include <LiquidCrystal_I2C.h>
#include <avdweb_VirtualDelay.h>
#include <SoftwareSerial.h>

#define ADDR_PANEL '0'
#define ADDR_ROLL '1'
#define ADDR_CTRL '2'
#define ADDR_PUSH '3'
#define ADDR_PULL '4'

#define ADDR_DEREG 'F'

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
#define INPUT_PULLDOWN

Adafruit_Debounce btnPullRight(BTN_PULL_RIGHT, HIGH);
Adafruit_Debounce btnPullLeft(BTN_PULL_LEFT, HIGH);
Adafruit_Debounce btnHeat1(BTN_HEAT1, HIGH);
Adafruit_Debounce btnHeat2(BTN_HEAT2, HIGH);

Adafruit_Debounce btnProdStart(BTN_PROD_START, HIGH);

Adafruit_Debounce btnProdEnd(BTN_PROD_END, HIGH);

Adafruit_Debounce btnTapeRight(BTN_TAPE_RIGHT, HIGH);
Adafruit_Debounce btnTapeLeft(BTN_TAPE_LEFT, HIGH);
Adafruit_Debounce btnFailStop(BTN_FAIL_STOP, HIGH);


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

#define LED_HEAT_2 PIN_A10
#define LED_HEAT_1 PIN_A11
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
//#define DALLAS_SENSOR 36
//OneWire oneWire(DALLAS_SENSOR);
//DallasTemperature DSTemp(&oneWire);

#define HEATERS_EN 37
#define FOAM_PNEUMATIC_1 38
#define FOAM_PNEUMATIC_2 39
#define SIG_FOAM_HEAT_1 40
#define FOAM_HEAT_1_TEMP_AL1_TRIG  PIN_A1 // alarm dolny

#define SIG_FOAM_HEAT_2 41
#define FOAM_HEAT_2_TEMP_AL1_TRIG  PIN_A4 //alarm dolny H2

#define FOAM_BLOWER 42

//---------- Tape module ---------------
#define TAPE_ENGINE_INVERTER 12 
#define TAPE_SLOW_SPEED 32
#define TAPE_PROD_SPEED 64
#define SIG_TAPE_RIGHT 43
#define SIG_TAPE_LEFT 44

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


#define SIG_TAPE_BREAK_PIN PIN_A7
#define SIG_PIPE_BREAK_PIN PIN_A6
//ENCODERS 
#define ENC_TAPE_A 2
#define ENC_TAPE_B 3
#define ENC_PIPE_A 9
#define ENC_PIPE_B 10

//MOTOR SPD POT
#define PIN_MOTOR_SPD A0
int speed = 0;
int _prevSpeed = 0;

//pipe end detection
long prev_Enc_Tape_Counter = 360;
long Enc_Tape_counter = 0;
int Enc_Tape_aState;
int Enc_Tape_aLastState;

long prev_Enc_Pipe_Counter = 360;
long Enc_Pipe_counter = 0;
int Enc_Pipe_aState;
int Enc_Pipe_aLastState;

String commandBuffer;

//for production line we have: push and pull
const int maxSlaves = 4;
struct SlaveInfo {
	char ID;
	bool isHealthy;
	unsigned long lastCheckedTime; // Timestamp of last health check
	unsigned long lastOkTime; // Timestamp of last health check

	byte slaveType;
};

SlaveInfo registeredSlaves[maxSlaves];
bool isPushRegistered = false;
bool isPullRegistered = false;

bool isPullStateReadyToStart = false;
bool isPushStateReadyToStart = false;

int currentSlaveIndex = -1;
unsigned long pingSentTime = 0;

int numRegisteredSlaves = 0;

const int healthCheckInterval = 1500; //10s TTL check 



void(*resetFunc) (void) = 0;

enum E_STATE {
	COMMS_CHECK,
	PIPE_LOAD,
	PIPE_END,
	FOAM_END,
	STARTING,
	PROCESS_RUN,
	COOLDOWN,
	EMERGENCY_STOP,

};

VirtualDelay heaterStartDelay;
VirtualDelay btnStop3sCounterl;
VirtualDelay blowerSwitchOffDelay;
VirtualDelay pipePresenceDelay;

//volatile ES_START _start_sub_state = ES_START::HEATERS_ON;
volatile E_STATE _state = E_STATE::STARTING;


// the setup function runs once when you press reset or power the board
void setup() {

	Serial.begin(115200);
	Serial.println("Foam Master V2025.04.07");
	//Serial1.is RS485
	Serial1.begin(19200);

	lcd.init(); // initialize the lcd	
	lcd.backlight();
	lcd.clear();
	lcd.setCursor(0, 0);            // move cursor the first row
	lcd.print("     BLACKFISH   ");          // print message at the first row
	lcd.setCursor(0, 1);            // move cursor to the second row
	lcd.print("   FOAM MASTER PRO  "); // print message at the second row
	lcd.setCursor(0, 2);            // move cursor to the third row
	lcd.print("     V2025.04.05"); // print message at the second row
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

	pinMode(HEATERS_EN, OUTPUT);
	pinMode(FOAM_HEAT_1_TEMP_AL1_TRIG, INPUT);

	pinMode(SIG_FOAM_HEAT_1, OUTPUT);
	digitalWrite(SIG_FOAM_HEAT_1, LOW);

	pinMode(FOAM_HEAT_2_TEMP_AL1_TRIG, INPUT);

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

	pinMode(LED_HEAT_2, OUTPUT);
	pinMode(LED_HEAT_1, OUTPUT);
	digitalWrite(LED_HEAT_2, LOW);
	digitalWrite(LED_HEAT_1, LOW);

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

	pinMode(SIG_TAPE_BREAK_PIN, INPUT);
	pinMode(SIG_PIPE_BREAK_PIN, INPUT);
	pinMode(ENC_PIPE_A, INPUT);
	pinMode(ENC_PIPE_B, INPUT);
	Enc_Pipe_aLastState = digitalRead(ENC_PIPE_A);

	pinMode(ENC_TAPE_A, INPUT);
	pinMode(ENC_TAPE_B, INPUT);
	Enc_Tape_aLastState = digitalRead(ENC_TAPE_A);

	//pinMode(SIG_TAPE_BREAK_PIN, INPUT);
	delay(200);

	for (int i = 0; i < maxSlaves; i++) {
		registeredSlaves[i].ID = ADDR_DEREG;
		registeredSlaves[i].isHealthy = false;
		registeredSlaves[i].lastCheckedTime = 0;
		registeredSlaves[i].lastOkTime = 0;
	}

	SetState(E_STATE::COMMS_CHECK);
}

// the loop function runs over and over again until power down or reset
void loop() {
	UpdateButtons();
	HandleComms();
	switch (_state)
	{
	case E_STATE::COMMS_CHECK:
	{
		lcd.clear();
		lcd.backlight();
		lcd.setCursor(0, 0);
		lcd.print("-  LINE NOT READY  -");          // print message at the first row

		lcd.setCursor(0, 1);
		lcd.print("PUSH:- ");
		lcd.setCursor(8, 1);
		lcd.print("PULL:- ");

		//CheckSlaves();
		while (1)
		{
			HandleComms();
			CheckSlaves();
			checkAndDeregisterSlaves();			
			UpdateButtons();
			
			if (isPullStateReadyToStart && isPushStateReadyToStart)
			{
				break;
			}

			
		}


		SetState(E_STATE::PIPE_LOAD);

	}break;
	case PIPE_LOAD:
	{
		UpdateButtons();
		lcd.clear();
		lcd.backlight();
		lcd.setCursor(0, 0);
		lcd.print("-   System ready   -");          // print message at the first row
		CheckSlaves();

		lcd.setCursor(0, 2);
		lcd.print("SPD:");
		lcd.setCursor(0, 3);
		lcd.print("    Press START    ");
		analogWrite(TAPE_ENGINE_INVERTER, TAPE_SLOW_SPEED);

		digitalWrite(LED_PULL_RIGHT, LOW);
		digitalWrite(LED_PULL_LEFT, LOW);
		digitalWrite(LED_PROD_START, LOW);
		digitalWrite(LED_PROD_END, HIGH);
		digitalWrite(SIG_FOAM_HEAT_2, LOW);

		digitalWrite(LED_HEAT_1, LOW);
		digitalWrite(SIG_FOAM_HEAT_1, LOW);
		digitalWrite(LED_HEAT_2, LOW);
		delay(100);
		HandleComms();
		int endCounter = 0;
		while (endCounter < 3)
		{
			HandleComms();
			UpdateButtons();
			ReadAndUpdateSpeed();
			if (btnHeat1.isPressed())
			{
				Serial.println("btnHeat1 pressed");
				digitalWrite(FOAM_PNEUMATIC_1, HIGH);
				digitalWrite(LED_HEAT_2, HIGH);
			}
			else
			{
				digitalWrite(FOAM_PNEUMATIC_1, LOW);
				digitalWrite(LED_HEAT_2, LOW);
			}

			if (btnHeat2.isPressed())
			{
				Serial.println("btnHeat2 pressed");
				digitalWrite(FOAM_PNEUMATIC_2, HIGH);
				digitalWrite(LED_HEAT_1, HIGH);
			}
			else
			{
				digitalWrite(FOAM_PNEUMATIC_2, LOW);
				digitalWrite(LED_HEAT_1, LOW);
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

			HandleTapeMovement();

			btnProdStart.update();
			if (btnProdStart.isPressed())
			{

				if (endCounter == 0) btnStop3sCounterl.start(300);
				if (btnStop3sCounterl.elapsed())
				{

					btnStop3sCounterl.start(300);
					endCounter++;

				}
			}
			if (!btnProdStart.isPressed()) endCounter = 0;
		}
		delay(100);

		bool isFoamDetected = !IsTapeBreakDetectedOnLaser();
		E_STATE nextState = E_STATE::STARTING;
		if (!isFoamDetected)
		{
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print("-   System hold    -");
			lcd.setCursor(0, 2);
			lcd.print("    LOAD FOAM  ");
			nextState = E_STATE::PIPE_LOAD;
		}

		while (IsTapeBreakDetectedOnLaser());

		Serial.println("btnProdStart pressed");
		SetState(nextState);

	}break;
	case STARTING: {
		lcd.clear();
		heaterStartDelay.start(HEATERS_START_DELAY); //5s Wait before heater starts
		Serial.println("BLOWER ON");
		lcd.setCursor(0, 0);
		lcd.print("Blower starting...");
		digitalWrite(SIG_BLOWER_PIN, HIGH); // Make sure the blower is ALWAYS ON !!
		delay(500);
		digitalWrite(HEATERS_EN, HIGH); // Make sure the blower is ALWAYS ON !!
		digitalWrite(LED_PROD_START, HIGH);
		digitalWrite(LED_PROD_END, LOW);

		while (!btnProdEnd.isPressed())
		{
			UpdateButtons();
			if (heaterStartDelay.elapsed())
			{
				digitalWrite(SIG_BLOWER_PIN, HIGH); // Make sure the blower is ALWAYS ON !!
				digitalWrite(HEATERS_EN, HIGH); // Make sure the blower is ALWAYS ON !!
				lcd.setCursor(0, 1);
				lcd.print("Heaters starting...");
				Serial.println("Heaters start");
				Serial.print("PID H1 (A1) AL1:"); Serial.println(digitalRead(FOAM_HEAT_1_TEMP_AL1_TRIG));

				if (digitalRead(FOAM_HEAT_1_TEMP_AL1_TRIG) == LOW)
				{

					digitalWrite(SIG_FOAM_HEAT_1, HIGH);
					digitalWrite(LED_HEAT_1, HIGH);
					isHeat1ON = true;
					Serial.println("Heater 1 ON");
				}
				else
				{
					digitalWrite(LED_HEAT_1, LOW);
				}
				//here we can add potential delay to second heater
				Serial.print("PID H2 (A4) AL1:"); Serial.println(digitalRead(FOAM_HEAT_2_TEMP_AL1_TRIG));

				if (digitalRead(FOAM_HEAT_2_TEMP_AL1_TRIG) == LOW)
				{

					digitalWrite(SIG_FOAM_HEAT_2, HIGH);
					digitalWrite(LED_HEAT_2, HIGH);
					isHeat2ON = true;
					Serial.println("Heater 2 ON");
				}
				else
				{
					digitalWrite(LED_HEAT_2, HIGH);
				}
				Serial.println("Waiting for temp to reach treshold");

				HandleTapeMovement();

				bool isH1Ready = false, isH2Ready = false;
				E_STATE nextState = E_STATE::PROCESS_RUN;
				while (!isH1Ready || !isH2Ready)
				{

					if (!isH2Ready && digitalRead(FOAM_HEAT_2_TEMP_AL1_TRIG) == HIGH)
					{
						digitalWrite(SIG_FOAM_HEAT_2, LOW);
						digitalWrite(LED_HEAT_2, LOW);
						Serial.println("H2 reached temp");
						isH2Ready = true;
					}
					else
					{
						//isH2Ready = false;
						digitalWrite(SIG_FOAM_HEAT_2, HIGH);
						digitalWrite(LED_HEAT_2, HIGH);
					}

					if (!isH1Ready && digitalRead(FOAM_HEAT_1_TEMP_AL1_TRIG) == HIGH)
					{
						digitalWrite(SIG_FOAM_HEAT_1, LOW);
						digitalWrite(LED_HEAT_1, LOW);
						Serial.println("H1 reached temp");
						isH1Ready = true;
					}
					else
					{
						//isH1Ready = false;
						digitalWrite(SIG_FOAM_HEAT_1, HIGH);
						digitalWrite(LED_HEAT_1, HIGH);
					}

					UpdateButtons();
					if (btnHeat1.isPressed())
					{
						Serial.println("btnHeat1 pressed");
						digitalWrite(FOAM_PNEUMATIC_1, HIGH);
						digitalWrite(LED_HEAT_2, HIGH);
					}
					else
					{
						digitalWrite(FOAM_PNEUMATIC_1, LOW);
						digitalWrite(LED_HEAT_2, LOW);
					}

					if (btnHeat2.isPressed())
					{
						Serial.println("btnHeat2 pressed");
						digitalWrite(FOAM_PNEUMATIC_2, HIGH);
						digitalWrite(LED_HEAT_1, HIGH);
					}
					else
					{
						digitalWrite(FOAM_PNEUMATIC_2, LOW);
						digitalWrite(LED_HEAT_1, LOW);
					}

					if (btnProdEnd.isPressed())
					{
						Serial.println("Warmup interrupted");
						nextState = E_STATE::COOLDOWN;
						break;
					}

					HandleTapeMovement();
				}

				if (nextState == E_STATE::PROCESS_RUN)
				{
					lcd.clear();
					btnProdEnd.update();
					lcd.setCursor(0, 1);
					lcd.print("  READY TO START ?");
					while (1)
					{
						UpdateButtons();
						if (btnProdStart.isPressed()) {

							nextState = E_STATE::PROCESS_RUN;
							break;
						}
						if (btnProdEnd.isPressed()) {

							nextState = E_STATE::PIPE_LOAD;
							break;
						}

						HandleTapeMovement();
					}
				}
				SetState(nextState);
				break;
			}
		}
		//if (btnProdEnd.isPressed()) SetState(E_STATE::PIPE_LOAD);

	}break;
	case PROCESS_RUN:
	{
		lcd.clear();
		btnProdEnd.update();
		lcd.setCursor(0, 0);
		lcd.print("   !! RUNNING !! ");
		Serial.println("RUNNING");

		DoHeaters(HIGH);

		digitalWrite(LED_PROD_START, HIGH);
		digitalWrite(LED_PROD_END, LOW);

		analogWrite(TAPE_ENGINE_INVERTER, TAPE_PROD_SPEED);
		delay(200);
		digitalWrite(SIG_TAPE_LEFT, HIGH);
		digitalWrite(SIG_TAPE_RIGHT, LOW);

		digitalWrite(SIG_BLOWER_PIN, HIGH); // Make sure the blower is ALWAYS ON !!		
		digitalWrite(HEATERS_EN, HIGH); // Make sure the blower is ALWAYS ON !!

		delay(400);
		pipePresenceDelay.start(500);

		prev_Enc_Pipe_Counter = -150; // we will stop only after full rotation has passed.
		prev_Enc_Tape_Counter = -150;
		Enc_Pipe_counter = 0;
		Enc_Tape_counter = 0;

		E_STATE nextState = E_STATE::COOLDOWN;
		long endCounter = 0;

		while (endCounter < 2)
		{
			IsPipeEncRotating();
			IsTapeEncRotating();

			if (pipePresenceDelay.elapsed())
			{
				//if (IsPipeEndDetectedOnEncoder() || IsTapeBreakDetectedOnEncoder()) {

				if (IsTapeBreakDetectedOnLaser()) {

					nextState = E_STATE::FOAM_END;
					Serial.println("Foam Missing;");
					break;
				}
				if (IsPipeBreakDetectedOnLaser()) {

					nextState = E_STATE::PIPE_END;
					Serial.println("Pipe Missing;");
					break;
				}
				pipePresenceDelay.start(300);
			}

			/*if (digitalRead(FOAM_HEAT_1_TEMP_AL1_TRIG)) {
				digitalWrite(LED_HEAT_1, LOW);
			}
			else {
				digitalWrite(LED_HEAT_1, HIGH);
				digitalWrite(SIG_FOAM_HEAT_1, HIGH);
			}

			if (digitalRead(FOAM_HEAT_2_TEMP_AL1_TRIG)) {
				digitalWrite(LED_HEAT_2, LOW);
			}
			else {

				digitalWrite(SIG_FOAM_HEAT_2, HIGH);
				digitalWrite(LED_HEAT_2, HIGH);
			}*/

			btnProdEnd.update();
			if (btnProdEnd.isPressed())
			{
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

		SetState(nextState);

	}break;
	case COOLDOWN:
	{
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("-  System stopped  -");
		DoCoolDownAndStopTape();

		lcd.setCursor(0, 3);
		lcd.print("    Press START   ");
		int endCounter = 0;
		while (endCounter < 3)
		{
			UpdateButtons();
			if (btnProdStart.isPressed())
			{
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
	case FOAM_END:
	case PIPE_END:
	{
		lcd.clear();
		lcd.setCursor(0, 0);
		if (_state == E_STATE::FOAM_END) lcd.print("-  Foam end found  -");
		if (_state == E_STATE::PIPE_END) lcd.print("-  Pipe end found  -");
		lcd.setCursor(0, 2);
		lcd.print("    Please load  ");
		lcd.setCursor(0, 3);
		lcd.print("    Press START     ");

		DoCoolDownAndStopTape();

		int endCounter = 0;
		while (endCounter < 3)
		{
			UpdateButtons();
			if (btnProdStart.isPressed())
			{
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


bool HandleComms()
{
	if (Serial1.available()) {
		String data = Serial1.readStringUntil('\n');
		//data = data.trim(); // Remove any leading or trailing whitespace
		if (data.length() < 2) {
			Serial.println("Invalid data received: " + data);

		}

		char slaveID = data.charAt(data.lastIndexOf(':')+1);
		String message = data.substring(0, data.lastIndexOf(':'));

		Serial.println("Received data: " + data);
		Serial.println("Slave ID: " + slaveID);
		Serial.println("Message: " + message);

		if (message.startsWith("REG_PUSH")) {
			if (registerSlave(slaveID, ADDR_PUSH)) {
				Serial.println("Slave registered successfully REG_PUSH node: " + String(slaveID));

			}
			else {
				Serial.println("Can't register node: " + String(slaveID));
			}
			isPushRegistered = true;
			lcd.setCursor(0, 1);
			lcd.print("PUSH:OK");
		}
		else if (message.startsWith("REG_PULL")) {
			if (registerSlave(slaveID, ADDR_PULL)) {
				Serial.println("Slave registered successfully REG_PULL node: " + String(slaveID));
			}
			else {
				Serial.println("Can't register node: " + String(slaveID));
			}

			isPullRegistered = true;
			lcd.setCursor(8, 1);
			lcd.print("PULL:OK");
		}


		else if (message.startsWith("PONG")) {
			//Serial.println("=> Pong received");
			updateSlaveHealth(slaveID, true);
		}
		else if (message.startsWith("ACK")) {
			//Serial.println(" = > ACK received");
			updateSlaveHealth(slaveID, true);

		}
		else if (message.startsWith("AREAD")) {
			//Serial.println("=> AREAD received");
			updateSlaveHealth(slaveID, true);

		}
		else {
			Serial.println("Unknown message received: " + message);
		}

	}
}

void DoHeaters(bool state)
{
	digitalWrite(FOAM_PNEUMATIC_1, state);
	digitalWrite(FOAM_PNEUMATIC_2, state);
}

void HandleEmergency()
{
	btnFailStop.update();
	if (btnFailStop.isPressed())
	{

		Serial.println("EMERGENCY STOP");
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("-     EMERGENCY!   -");
		lcd.setCursor(0, 1);
		lcd.print("        STOP     ");
		
		DoCoolDownAndStopTape();
		sendCommand(-1, "EMERGENCY STOP");
		while (digitalRead(BTN_FAIL_STOP) != HIGH) {
			delay(10);
		}
		Serial.println("EMERGENCY STOP released");
		resetFunc();
	}

}

void DoCoolDownAndStopTape()
{
	digitalWrite(SIG_TAPE_LEFT, LOW);
	digitalWrite(SIG_TAPE_RIGHT, LOW);

	digitalWrite(FOAM_PNEUMATIC_1, LOW);
	digitalWrite(FOAM_PNEUMATIC_2, LOW);

	digitalWrite(SIG_FOAM_HEAT_1, LOW);
	digitalWrite(SIG_FOAM_HEAT_2, LOW);

	digitalWrite(SIG_BLOWER_PIN, LOW);
	digitalWrite(HEATERS_EN, LOW); // Make sure the blower is ALWAYS ON !!
	delay(500);	//Leave it here
}


bool IsTapeBreakDetectedOnLaser()
{

	if (digitalRead(SIG_TAPE_BREAK_PIN))
	{
		Serial.println("TAPE BREAK DETECTED ");
		return true;
	}

	return false;

}

bool IsPipeBreakDetectedOnLaser()
{

	if (digitalRead(SIG_PIPE_BREAK_PIN))
	{
		Serial.println("PIPE BREAK DETECTED L");
		return true;
	}

	return false;

}

bool IsTapeBreakDetectedOnEncoder()
{
	/*Serial.print("Tape counter:"); Serial.println(Enc_Tape_counter);
	Serial.print("Tape prev counter:"); Serial.println(prev_Enc_Tape_Counter);*/
	Serial.print("Tape Diff:"); Serial.println(Enc_Tape_counter - prev_Enc_Tape_Counter);
	if (Enc_Tape_counter - prev_Enc_Tape_Counter < 5)
	{
		Serial.println("TAPE BREAK DETECTED");
		return true;
	}
	prev_Enc_Tape_Counter = Enc_Tape_counter;
	return false;
}

bool IsPipeEndDetectedOnEncoder()
{
	/*Serial.print("Pipe counter:"); Serial.println(Enc_Pipe_counter);
	Serial.print("Pipe prev counter:"); Serial.println(prev_Enc_Pipe_Counter);*/
	Serial.print("Pipe Diff:"); Serial.println(Enc_Pipe_counter - prev_Enc_Pipe_Counter);
	if (Enc_Pipe_counter - prev_Enc_Pipe_Counter < 5)
	{
		Serial.println("PIPE END DETECTED");
		return true;
	}
	prev_Enc_Pipe_Counter = Enc_Pipe_counter;
	return false;
}

void HandleTapeMovement()
{
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
	HandleEmergency();
	ReadAndUpdateSpeed();

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

static bool IsPipeEncRotating() {
	Enc_Pipe_aState = digitalRead(ENC_PIPE_A); // Reads the "current" state of the outputA
	// If the previous and the current state of the outputA are different, that means a Pulse has occured
	if (Enc_Pipe_aState != Enc_Pipe_aLastState) {
		// If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
		if (digitalRead(ENC_PIPE_B) != Enc_Pipe_aState) {
			Enc_Pipe_counter++;
		}
		/*else {
			Enc_Pipe_counter--;
		}*/
		/*Serial.print("Enc Pipe Position: ");
		Serial.println(Enc_Pipe_counter);*/
	}
	Enc_Pipe_aLastState = Enc_Pipe_aState; // Updates the previous state of the outputA with the current state
}

static bool IsTapeEncRotating() {
	Enc_Tape_aState = digitalRead(ENC_TAPE_A); // Reads the "current" state of the outputA
	// If the previous and the current state of the outputA are different, that means a Pulse has occured
	if (Enc_Tape_aState != Enc_Tape_aLastState) {
		// If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
		if (digitalRead(ENC_TAPE_B) != Enc_Tape_aState) {
			Enc_Tape_counter++;
		}
		/*else {
			Enc_Tape_counter--;
		}*/
		/*Serial.print("Enc Type Position: ");
		Serial.println(Enc_Tape_counter);*/
	}
	Enc_Tape_aLastState = Enc_Tape_aState; // Updates the previous state of the outputA with the current state

}




