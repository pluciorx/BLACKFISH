#include <Arduino.h>

// Pin definitions
//buttons (2 x 4 )
// 
//Defines so the device can do a self reset
#define SYSRESETREQ    (1<<2)
#define VECTKEY        (0x05fa0000UL)
#define VECTKEY_MASK   (0x0000ffffUL)
#define AIRCR          (*(uint32_t*)0xe000ed0cUL) // fixed arch-defined address
#define REQUEST_EXTERNAL_RESET (AIRCR=(AIRCR&VECTKEY_MASK)|VECTKEY|SYSRESETREQ)

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

//Pedals
int p1_counter = 0;
int p1_prevCounter = 0;
int p1_angle = 0;
bool p1_prevA = 1, p1_prevB = 1;

int p2_counter = 0;
int p2_prevCounter = 0;
int p2_angle = 0;
bool p2_prevA = 1, p2_prevB = 1;

#define PEDAL1_A 31
#define PEDAL1_B 33
#define PEDAL2_A 35
#define PEDAL2_B 37
byte pedalResistance = 0; 

//Com
#define COM_BAUD_Debug 115200
#define COM_BAUD_PC 115200
#define MAX_INPUT 200U

char input_line[MAX_INPUT];

//motors (4 for chairs 2 for pedals)
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

//TODO: ADD oil valve motor.


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

bool IsVibrationEnabled = false;


//AccelStepper motorC1L = AccelStepper(AccelStepper::FULL3WIRE, CHAIR1_LEFT_STEP, CHAIR1_LEFT_DIR);
//AccelStepper motorC1R = AccelStepper(AccelStepper::FULL3WIRE, CHAIR1_RIGHT_STEP, CHAIR1_RIGHT_DIR);
//AccelStepper motorC2L = AccelStepper(AccelStepper::FULL3WIRE, CHAIR2_LEFT_STEP, CHAIR2_LEFT_DIR);
//AccelStepper motorC2R = AccelStepper(AccelStepper::FULL3WIRE, CHAIR2_RIGHT_STEP, CHAIR2_RIGHT_DIR);


enum E_STATE {
	SETUP = 'S',
	HOMING = 'H',
	LISTENING = 'L',
	EXECUTE_CMD = 'E',
	READY = 'R',	
	VIBRATE = 'V',
	KILL = 'K'
};

enum CommandType {
	VibStart,
	VibStop,
	PedalResistance,
	Ping
};

E_STATE _state = E_STATE::SETUP;
E_STATE _prevState = E_STATE::SETUP;


void setup() {
	
	Serial.begin(COM_BAUD_Debug);
	//Serial1 to talk with host
	Serial1.begin(COM_BAUD_PC);
	Serial.println("<================= Starting =================>");
	
	//motors
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
	//end motors
	// 
	//pedals
	pinMode(PEDAL1_A, INPUT_PULLUP);
	pinMode(PEDAL1_B, INPUT_PULLUP);
	pinMode(PEDAL2_A, INPUT_PULLUP);
	pinMode(PEDAL2_B, INPUT_PULLUP);

	//DUMP whatever is in the serial and wait for the clean go.
	while (Serial1.available()) Serial2.read();
	//end pedals
	SetState(E_STATE::HOMING);
}



void loop() {
	
	switch (_state)
	{
		case HOMING:{
			
			
			//HomeChairs();
			//HomePedals();
			SetState(E_STATE::LISTENING);
		}break;
		case LISTENING: {
			if (ProcessIncommingMsg(Serial1)) SetState(E_STATE::EXECUTE_CMD);
			
			SetState(E_STATE::READY);
		}break;
		case EXECUTE_CMD: {
			ExecuteCMD();
			SetState(E_STATE::LISTENING);
		}break;

		case READY: {
			
			HandlePanelPress();
			HandlePedaling();
			
			SetState(E_STATE::LISTENING);
		}break;
		case KILL: {
			REQUEST_EXTERNAL_RESET;
		}break;
	}
}

bool ExecuteCMD()
{

	CommandType cmd = GetCMDFromInput();
	switch (cmd)
	{
	case VibStart:
		break;
	case VibStop:
		break;
	case PedalResistance:
		break;
	case Ping:
	{
		Serial1.println("pong");
	}break;
	default:
		break;
	}
	//startVibration
	//stopVibration
	//pedalResistance|ID
	Serial.println(input_line);
	return true;
}
CommandType GetCMDFromInput()
{
	if (input_line == "startVibration")
	{
		Serial.println("Vibration ON....");
		return CommandType::VibStart;
	}
	if (input_line == "stopVibration")
	{
		Serial.println("Vibration OFF");
		return CommandType::VibStop;
	}
	char* p;
	p = strstr(input_line, "pedalResistance|");
	if (p)
	{
		char level[1];
		 level[0] = input_line[strlen(input_line) - 1];
		
		 if (isdigit(level[0])) {     // checks if end_char is a number
			 pedalResistance = stringToByte(level);
			 if (pedalResistance > 0 && pedalResistance <= 4)
			 {  
				 Serial.print("Pedal Resistance:"); Serial.println(pedalResistance);
				 return CommandType::PedalResistance;
			 }  				 
		}		
		Serial.print("Invalid resistance:"); Serial.println(level[0]);
	}
}

byte stringToByte(char* src) {
	return byte(atoi(src));
}
bool ProcessIncommingMsg(UARTClass sourceSerial)
{
	while (sourceSerial.available())
	{
		byte chr1 = sourceSerial.read();
		if (commBuild(chr1, 1)) return true;		
	}
	return false;
}

bool commBuild(const char inByte, int serialNum)
{
	static unsigned int input_pos = 0;

	switch (inByte)
	{

	case '\n':   // end of text
	{
		input_line[input_pos] = 0;  // terminating null byte
		input_pos = 0;
		return true;
	}
	break;
	case '\r':   // discard carriage return - t should never get here
		break;

	default: {
		// keep adding if not full ... allow for terminating null byte
		if (input_pos < (MAX_INPUT - 1))
			input_line[input_pos++] = inByte;
		return false;
	}break;

	}  // end of switch
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
			buttonActive[i] = buttonState[i];
			char cMsg[20];
			sprintf(cMsg, "button|%d", i + 1);
			Serial1.println(cMsg);
			Serial.print("==>"); Serial.println(cMsg);
		}
		else
			buttonActive[i] = false;

	}
}

void SendPedalState(byte pedalNo, int pedalAngle)
{
	char cMsg[20];
	sprintf(cMsg, "pedal%d|%d", pedalNo, pedalAngle);
	Serial1.println(cMsg);
	Serial.print("==>"); Serial.println(cMsg);
	
	Serial.print("Position Pedal_"+String(pedalNo)+":");
	Serial.print(int(pedalAngle * (-1.8)));
	Serial.println("deg");
}

void HandlePedaling()
{
	bool A = digitalRead(PEDAL1_A), B = digitalRead(PEDAL1_B);

	p1_counter += (A ^ p1_prevA) | (B ^ p1_prevB) ? A ^ p1_prevB ? 1 : -1 : 0;

	p1_prevA = A;
	p1_prevB = B;
	if (p1_counter != p1_prevCounter)
	{
		SendPedalState(1, p1_counter);
		p1_prevCounter = p1_counter;
	}

	A = digitalRead(PEDAL2_A), B = digitalRead(PEDAL2_B);

	p2_counter += (A ^ p2_prevA) | (B ^ p2_prevB) ? A ^ p2_prevB ? 1 : -1 : 0;
	 
	p2_prevA = A;
	p2_prevB = B;

	if (p2_counter != p2_prevCounter)
	{
		SendPedalState(2, p2_counter);
		p2_prevCounter = p2_counter;
	}
	
}


