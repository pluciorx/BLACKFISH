#include <AccelStepper.h>

//Set DEBUG 1 to test commands by sending them trough debug serial.
#define DEBUG 1
// Pin definitions
//buttons (2 x 4 )
// 
//Defines so the device can do a self reset
#define SYSRESETREQ    (1<<2)
#define VECTKEY        (0x05fa0000UL)
#define VECTKEY_MASK   (0x0000ffffUL)
#define AIRCR          (*(uint32_t*)0xe000ed0cUL) // fixed arch-defined address
#define REQUEST_EXTERNAL_RESET (AIRCR=(AIRCR&VECTKEY_MASK)|VECTKEY|SYSRESETREQ)

#define P1_B1 52
#define P1_B2 53
#define P1_B3 50
#define P1_B4 51

const byte buttonCount = 4;
byte buttonPins[] = { P1_B1, P1_B2, P1_B3, P1_B4 };
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

#define PEDAL1_A 28
#define PEDAL1_B 26
#define PEDAL2_A 24
#define PEDAL2_B 22
byte pedalResistance = 0; 

//Com
#define COM_BAUD_Debug 115200
#define COM_BAUD_PC 115200
#define MAX_INPUT 50 //maximum length of the command..

char input_line[MAX_INPUT] ="";
char error[MAX_INPUT];

#define HOMING_SPEED 100
#define HOMING_ACCELERATION 100

//motors (4 for chairs 2 for pedals)
#define MOTORS_BASE_SPEED 500
#define	MOTORS_ACCEL 500

//Chair 1 
#define CHAIR1_LEFT_UPPER_LIMIT 48
#define CHAIR1_LEFT_ALARM 46
#define CHAIR1_LEFT_EN 44
#define CHAIR1_LEFT_STEP 2
#define CHAIR1_LEFT_DIR 3

#define CHAIR1_RIGHT_UPPER_LIMIT 49
#define CHAIR1_RIGHT_ALARM 47
#define CHAIR1_RIGHT_EN 45
#define CHAIR1_RIGHT_STEP 4
#define CHAIR1_RIGHT_DIR 5

//Chair 2 
#define CHAIR2_RIGHT_UPPER_LIMIT 40
#define CHAIR2_RIGHT_ALARM 38
#define CHAIR2_RIGHT_EN 36
#define CHAIR2_RIGHT_STEP 6
#define CHAIR2_RIGHT_DIR 7


#define CHAIR2_LEFT_UPPER_LIMIT 41
#define CHAIR2_LEFT_ALARM 39
#define CHAIR2_LEFT_EN 37
#define CHAIR2_LEFT_STEP 8
#define CHAIR2_LEFT_DIR 9



//Resistance
#define PEDALL_UPPER_LIMIT 34
#define PEDALL_EN 30
#define PEDALL_STEP 10
#define PEDALL_DIR 11

#define PEDALR_UPPER_LIMIT 33
#define PEDALR_EN 29
#define PEDALR_STEP 12 
#define PEDALR_DIR 13
#define PEDALS_MAX_MAXDISTANCE 20000
#define PEDALS_BASE_SPEED 500


byte motorControlPins[] = { 
	CHAIR1_LEFT_EN		,
	CHAIR1_LEFT_STEP	, 
	CHAIR1_LEFT_DIR		,
	CHAIR1_LEFT_ALARM	,

	CHAIR1_RIGHT_EN		,
	CHAIR1_RIGHT_STEP	,
	CHAIR1_RIGHT_DIR	,
	CHAIR1_RIGHT_ALARM,

	CHAIR2_LEFT_EN		,
	CHAIR2_LEFT_STEP	,
	CHAIR2_LEFT_DIR		,
	CHAIR2_LEFT_ALARM	,

	CHAIR2_RIGHT_EN		,
	CHAIR2_RIGHT_STEP	,
	CHAIR2_RIGHT_DIR	,
	CHAIR2_RIGHT_ALARM	,

	PEDALL_UPPER_LIMIT	,
	PEDALL_EN 			,
	PEDALL_STEP 		,
	PEDALL_DIR 			,

	PEDALR_UPPER_LIMIT	,
	PEDALR_EN 			,
	PEDALR_STEP 		,
	PEDALR_DIR 			,
 };

long maxTravel = 200000; // max distance you could be away from zero switch
long maxBackup = 200; // max distance to correct limit switch overshoot

bool IsVibrationEnabled = false;

AccelStepper motorC1L(AccelStepper::DRIVER, CHAIR1_LEFT_STEP, CHAIR1_LEFT_DIR);
AccelStepper motorC1R(AccelStepper::DRIVER, CHAIR1_RIGHT_STEP, CHAIR1_RIGHT_DIR);
AccelStepper motorC2L(AccelStepper::DRIVER, CHAIR2_LEFT_STEP, CHAIR2_LEFT_DIR);
AccelStepper motorC2R(AccelStepper::DRIVER, CHAIR2_RIGHT_STEP, CHAIR2_RIGHT_DIR);
AccelStepper motorPedalsL(AccelStepper::DRIVER, PEDALL_STEP, PEDALL_DIR);
AccelStepper motorPedalsR(AccelStepper::DRIVER, PEDALR_STEP, PEDALR_DIR);


enum E_STATE {
	SETUP = 'S',
	HOMING = 'H',
	LISTENING = 'L',
	EXECUTE_CMD = 'E',
	READY = 'R',	
	VIBRATE = 'V',
	KILL = 'K',
	ERROR = 'X'
};

enum CommandType {
	VibStart,
	VibStop,
	PedalResistance,
	Ping,
	Unknown
};

E_STATE _state = E_STATE::SETUP;
E_STATE _prevState = E_STATE::SETUP;


void setup() {
	
	Serial.begin(COM_BAUD_Debug);
	//Serial1 to talk with host
	Serial1.begin(COM_BAUD_PC);
	Serial.println(F("<================= Starting =================>"));
	
	//motors
	// Set limit switch inputs
	pinMode(CHAIR2_LEFT_UPPER_LIMIT, INPUT_PULLUP);
	pinMode(CHAIR1_LEFT_ALARM, INPUT_PULLUP);
	pinMode(CHAIR2_RIGHT_UPPER_LIMIT, INPUT_PULLUP);
	pinMode(CHAIR1_RIGHT_ALARM, INPUT_PULLUP);

	for (int i = 0; i < 12; i++) {
		pinMode(motorControlPins[i], OUTPUT);
		digitalWrite(motorControlPins[i], LOW);
	}

	for (int i = 0; i < buttonCount; i++) {
		pinMode(buttonPins[i], INPUT_PULLUP);
		digitalWrite(buttonPins[i], HIGH); //redundant but just in case
	}

	motorC1L.setEnablePin(CHAIR1_LEFT_EN);
	motorC1L.enableOutputs();
	motorC1L.setAcceleration(MOTORS_ACCEL);
	motorC1L.setMaxSpeed(MOTORS_BASE_SPEED);
	attachInterrupt(digitalPinToInterrupt(CHAIR1_LEFT_UPPER_LIMIT), stopMotorC1L, FALLING);
	//motorC1L.setSpeed(400);

	motorC1R.setEnablePin(CHAIR1_RIGHT_EN);
	motorC1R.enableOutputs();
	motorC1R.setAcceleration(MOTORS_ACCEL);
	motorC1R.setMaxSpeed(500);
	attachInterrupt(digitalPinToInterrupt(CHAIR1_RIGHT_UPPER_LIMIT), stopMotorC1R, FALLING);

	motorC2L.setEnablePin(CHAIR2_RIGHT_EN);
	motorC2L.enableOutputs();
	motorC2L.setAcceleration(MOTORS_ACCEL);
	motorC2L.setMaxSpeed(MOTORS_BASE_SPEED);
	attachInterrupt(digitalPinToInterrupt(CHAIR2_LEFT_UPPER_LIMIT), stopMotorC2L, FALLING);

	motorC2R.setEnablePin(CHAIR2_LEFT_EN);
	motorC2R.enableOutputs();
	motorC2R.setAcceleration(MOTORS_ACCEL);
	motorC2R.setMaxSpeed(MOTORS_BASE_SPEED);
	attachInterrupt(digitalPinToInterrupt(CHAIR2_RIGHT_UPPER_LIMIT), stopMotorC2R, FALLING);

	motorPedalsL.setEnablePin(PEDALL_EN);
	motorPedalsL.enableOutputs();
	motorPedalsL.setMaxSpeed(MOTORS_BASE_SPEED);
	attachInterrupt(digitalPinToInterrupt(PEDALL_UPPER_LIMIT), stopMotorPL, FALLING);

	motorPedalsR.setEnablePin(PEDALR_EN);
	motorPedalsR.enableOutputs();
	motorPedalsR.setMaxSpeed(MOTORS_BASE_SPEED);
	attachInterrupt(digitalPinToInterrupt(PEDALR_UPPER_LIMIT), stopMotorPR, FALLING);
	//end motors
	

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
			Serial.println(F("Homing started..."));
			while (!HandlePedalsHoming());
			while (!HandleChairsHoming());
			Serial.println(F("Homing Finished..."));
			SetState(E_STATE::LISTENING);
			
		}break;
		case LISTENING: {
#if DEBUG == 1
			
			if (ProcessIncommingMsg(Serial))
			{						
				SetState(E_STATE::EXECUTE_CMD);
			}else 
				SetState(E_STATE::READY);
#endif

#if DEBUG == 0 
			if (ProcessIncommingMsg(Serial1))
			{
				SetState(E_STATE::EXECUTE_CMD);
			}
			else 
				SetState(E_STATE::READY);
#endif

		}break;
		case EXECUTE_CMD: {
			CommandType cmd = GetCMDFromInput(input_line);
			
			if(cmd != CommandType::Unknown && !ExecuteCMD(cmd)) 
			{
				SetError("Command execution failure");
				SetState(E_STATE::ERROR);
				
			}
			SetState(E_STATE::LISTENING);
		}break;

		case READY: {
			
			HandlePanelPress();
			HandlePedaling();
			HandleVibrations();
			SetState(E_STATE::LISTENING);
		}break;
		case KILL: {
			REQUEST_EXTERNAL_RESET;

		}break;
		case ERROR: {
			Serial.print(F("ERROR has occured:")); Serial.println(error);

		}break;
	}
}

void SetError(char* errorInput )
{
	strncpy(error, errorInput, 100);
#if DEBUG == 1
	Serial.print(F("Error Set:")); Serial.println(error);
#endif
}

void HandleVibrations() //none blockin
{
#if DEBUG == 2
	Serial.print("Vibrations:"); Serial.println(IsVibrationEnabled);
#endif
	if (IsVibrationEnabled)
	{
		//here move the motors

	}
	else
	{
		//stop all the motors
		
	}
}

bool HandleChairsHoming() //can be blocking
{
	bool IsHomingFinished = false;
#if DEBUG >= 1
	Serial.println("Homing Chairs:");
	IsHomingFinished = true;
#endif
	

	return IsHomingFinished;
}

bool HandlePedalsHoming() //can be blocking
{
	bool IsHomingFinished = false;
#if DEBUG >= 1
	Serial.print("Homing Pedals:");
	IsHomingFinished = true;
#endif


	Serial.println("HOMING LEFT"); //print action
	motorPedalsL.setAcceleration(HOMING_SPEED); //defining some low acceleration
	motorPedalsL.setMaxSpeed(HOMING_SPEED); //set speed, 100 for test purposes
	motorPedalsL.move(-1 * PEDALS_MAX_MAXDISTANCE); ////set distance - negative value flips the direction
	while (!IsHomingFinished ) motorPedalsL.run();
	
	

	return IsHomingFinished;
}

void stopMotorC1L()
{
	StopMotor(motorC1L);
}

void stopMotorC1R()
{
	StopMotor(motorC1R);
}
void stopMotorC2L()
{
	StopMotor(motorC2L);
}
void stopMotorC2R()
{
	StopMotor(motorC2R);
}

void stopMotorPL()
{
	StopMotor(motorPedalsL);
}

void stopMotorPR()
{
	StopMotor(motorPedalsR);
}

void StopMotor(AccelStepper & stepper)
{
	stepper.setCurrentPosition(0); 
	Serial.println("STOP "); 
	stepper.stop(); 
	stepper.disableOutputs(); 
}

bool ExecuteCMD(CommandType cmd)
{		
	switch (cmd)
	{
	case VibStart: //startVibration
	{
		Serial.println(F("Vibration ON"));
		IsVibrationEnabled = true;
		
	}break;
	case VibStop://stopVibration
	{
		Serial.println(F("Vibration OFF"));
		IsVibrationEnabled = false;
	}break;
	case PedalResistance://pedalResistance|ID
	{
		Serial.print(F("Pedals resistance")); Serial.println(pedalResistance);

	}break;
	case Ping:
	{
		//reserved for future use...
		Serial.println(F("==>pong"));
		Serial1.println(F("pong"));
		return true;
	}break;
	case Unknown:
	{
		Serial.print(F("Unknown Command received:'")); Serial.print(input_line); Serial.println(F('\''));
		return true;
	}break;
	default:
	{
		return false;
	}break;
	}
}
CommandType GetCMDFromInput(const char* input)
{
	CommandType cmdFound = CommandType::Unknown;
	Serial.print(F("<==")); Serial.println(input_line);
	if (strcmp(input, "ping") == NULL)
	{
		cmdFound = CommandType::Ping;
	}

	if (strcmp(input, "startVibration") == NULL)
	{
		cmdFound = CommandType::VibStart;
	}

	if (strcmp(input, "stopVibration") == NULL)
	{
		cmdFound = CommandType::VibStop;
	}

	if (strstr(input, "pedalResistance|") != NULL)
	{
		char level[1];
		level[0] = input_line[strlen(input_line) - 1];	
		if (isdigit(level[0])) {     // checks if end_char is a number
			pedalResistance = stringToByte(level);
			if (pedalResistance >= 0 && pedalResistance <= 4)
			{				 
			 cmdFound = CommandType::PedalResistance;
			}
			else {
				Serial.print(F("Invalid resistance range:")); Serial.println(level[0]);
			}
		}
		else {
			Serial.print(F("Invalid resistance:")); Serial.println(level[0]);
		}
	}
	input_line[0] = '\0';
	return cmdFound;
}

byte stringToByte(char* src) {
	return byte(atoi(src));
}

bool ProcessIncommingMsg(UARTClass & sourceSerial)
{
	while (sourceSerial.available())
	{	
		byte chr1 = sourceSerial.read();
		if(commBuild(chr1, 1)) {
			return true;
		}
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
	{
		return false;
	}break;
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

#if DEBUG == 2
	Serial.print("State:"); Serial.println((char)newState);
#endif //  DEBUG = 1

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
			Serial.print(F("==>")); Serial.println(cMsg);
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
	Serial.print(F("==>")); Serial.println(cMsg);

#if DEBUG == 1 	
	Serial.print("Position Pedal_"+String(pedalNo)+":");
	Serial.print(int(pedalAngle * (-1.8)));
	Serial.println("deg");
#endif
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


