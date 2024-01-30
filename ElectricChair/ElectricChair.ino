#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
// Pin definitions
//buttons (2 x 4 )

#define P1_B1 12
#define P1_B2 11
#define P1_B3 10
#define P1_B4 9

#define P2_B1 7
#define P2_B2 6
#define P2_B3 5
#define P2_B4 4

const byte buttonCount = 8;
byte buttonPins[] = { P1_B1, P1_B2, P1_B3, P1_B4, P2_B1, P2_B2, P2_B3, P2_B4 };
byte buttonState[buttonCount];
byte buttonActive[buttonCount];
// end buttons
// 
//Com
#define COM_BAUD_Debug 115200
#define COM_BAUD_PC 115200

//motors (4 motors
//Chair 1 
#define CHAIR1_LEFT_UPPER_LIMIT 52
#define CHAIR1_LEFT_LOWER_LIMIT 50
#define CHAIR1_LEFT_EN 48
#define CHAIR1_LEFT_STEP 44
#define CHAIR1_LEFT_DIR 42

#define CHAIR1_RIGHT_UPPER_LIMIT 22
#define CHAIR1_RIGHT_LOWER_LIMIT 24
#define CHAIR1_RIGHT_EN 26
#define CHAIR1_RIGHT_STEP 30
#define CHAIR1_RIGHT_DIR 32

//Chair 2 
#define CHAIR2_RIGHT_UPPER_LIMIT 23
#define CHAIR2_RIGHT_LOWER_LIMIT 25
#define CHAIR2_RIGHT_EN 27
#define CHAIR2_RIGHT_STEP 31
#define CHAIR2_RIGHT_DIR 33

#define CHAIR2_LEFT_UPPER_LIMIT 53
#define CHAIR2_LEFT_LOWER_LIMIT 51
#define CHAIR2_LEFT_EN 49
#define CHAIR2_LEFT_STEP 45
#define CHAIR2_LEFT_DIR 43

byte motorControlPins[] = { 
	CHAIR1_LEFT_EN		,
	CHAIR1_LEFT_STEP	, 
	CHAIR1_LEFT_DIR		,
	CHAIR1_RIGHT_EN		,
	CHAIR1_RIGHT_STEP	,
	CHAIR1_RIGHT_DIR	,
	CHAIR2_LEFT_EN		,
	CHAIR2_LEFT_STEP	,
	CHAIR2_LEFT_DIR		,
	CHAIR2_RIGHT_EN		,
	CHAIR2_RIGHT_STEP	,
	CHAIR2_RIGHT_DIR	,
 };
long maxTravel = 200000; // max distance you could be away from zero switch
long maxBackup = 200; // max distance to correct limit switch overshoot

AccelStepper motorC1L = AccelStepper(AccelStepper::FULL3WIRE, CHAIR1_LEFT_STEP, CHAIR1_LEFT_DIR);
AccelStepper motorC1R = AccelStepper(AccelStepper::FULL3WIRE, CHAIR1_RIGHT_STEP, CHAIR1_RIGHT_DIR);
AccelStepper motorC2L = AccelStepper(AccelStepper::FULL3WIRE, CHAIR2_LEFT_STEP, CHAIR2_LEFT_DIR);
AccelStepper motorC2R = AccelStepper(AccelStepper::FULL3WIRE, CHAIR2_RIGHT_STEP, CHAIR2_RIGHT_DIR);


enum E_STATE {
	SETUP = 'S',
	HOMING = 'H',
	READY = 'R',	
	VIBRATE = 'V'
};

E_STATE _state = E_STATE::SETUP;
E_STATE _prevState = E_STATE::SETUP;

void setup() {
	
	Serial.begin(COM_BAUD_Debug);
	//Serial1 to talk with host
	Serial1.begin(COM_BAUD_PC);
	Serial.println("<================= Starting =================>");
	
	// Set limit switch inputs
	pinMode(CHAIR2_LEFT_UPPER_LIMIT, INPUT_PULLUP);
	pinMode(CHAIR1_LEFT_LOWER_LIMIT, INPUT_PULLUP);
	pinMode(CHAIR2_RIGHT_UPPER_LIMIT, INPUT_PULLUP);
	pinMode(CHAIR1_RIGHT_LOWER_LIMIT, INPUT_PULLUP);

	for (int i = 0; i < 12; i++) {
		pinMode(motorControlPins[i], OUTPUT);
		digitalWrite(motorControlPins[i], LOW);
	}

	for (int i = 0; i < buttonCount; i++) {
		pinMode(buttonPins[i], INPUT_PULLUP);
		digitalWrite(buttonPins[i], HIGH); //redundant but just in case
	}

	SetState(E_STATE::HOMING);
}


void loop() {
	
	switch (_state)
	{
		case HOMING:{
			
			SetState(E_STATE::READY);
		}break;
		case READY: {
			// we check if any of the buttons are pressed first and send msg to host 
			HandlePanelPress();
		}break;	
	}
}

void SetState(E_STATE newState)
{
	Serial.print("State:"); Serial.println((char)newState);
	
	if (_state != newState)
	{
		_prevState = _state;
		_state = newState;		
	}	
}

void HandlePanelPress()
{
	for (byte i = 0; i < buttonCount; i++)
	{
		buttonState[i] = !digitalRead(buttonPins[i]); //inversion for ease read
		if (buttonActive[i] == false && buttonState[i] == true)
		{
			//check if we need to repeat msg or only when it's changed ?
			buttonActive[i] = true;
			char cMsg[20];
			sprintf(cMsg, "button|%d", i + 1);
			Serial1.println(cMsg);
			Serial.print("==>"); Serial.println(cMsg);
		}
		else
			buttonActive[i] = false;

	}
}