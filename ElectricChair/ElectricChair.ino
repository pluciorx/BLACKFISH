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
#define PANEL_LED 20

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
byte pedalPreviousResistance = 0; 

//Com
#define COM_BAUD_Debug 115200
#define COM_BAUD_PC 115200
#define MAX_INPUT 50 //maximum length of the command..

char input_line[MAX_INPUT] ="";
char error[MAX_INPUT];

#define PEDAL_HOMING_SPEED 500
#define PEDAL_HOMING_ACCELERATION 1000

#define PEDAL_MOTORS_BASE_SPEED 200
#define	PEDAL_MOTORS_ACCEL 200


#define CHAIR_HOMING_SPEED 1000
#define CHAIR_HOMING_ACCELERATION 1000

#define CHAIR_MOTORS_BASE_SPEED 500
#define	CHAIR_MOTORS_ACCEL 500

//motors (4 for chairs 2 for pedals)
#define CHAIR_MAX_DISTANCE 10000
#define CHAIR_BASE_POSITION 1000
#define MOTORS_BASE_SPEED 10000
#define	MOTORS_ACCEL 1000

//Chair 1 
#define CHAIR1_LEFT_UPPER_LIMIT 48
#define CHAIR1_LEFT_ALARM 46
#define CHAIR1_LEFT_EN 44
#define CHAIR1_LEFT_STEP 2
#define CHAIR1_LEFT_DIR 3
volatile bool IsC1LHomed = false;

#define CHAIR1_RIGHT_UPPER_LIMIT 49
#define CHAIR1_RIGHT_ALARM 47
#define CHAIR1_RIGHT_EN 45
#define CHAIR1_RIGHT_STEP 4
#define CHAIR1_RIGHT_DIR 5
volatile bool IsC1RHomed = false;

//Chair 2 
#define CHAIR2_RIGHT_UPPER_LIMIT 40
#define CHAIR2_RIGHT_ALARM 38
#define CHAIR2_RIGHT_EN 36
#define CHAIR2_RIGHT_STEP 6
#define CHAIR2_RIGHT_DIR 7
volatile bool IsC2RHomed = false;

#define CHAIR2_LEFT_UPPER_LIMIT 41
#define CHAIR2_LEFT_ALARM 39
#define CHAIR2_LEFT_EN 37
#define CHAIR2_LEFT_STEP 8
#define CHAIR2_LEFT_DIR 9
volatile bool IsC2LHomed = false;

//Resistance
#define PEDALL_UPPER_LIMIT 34
#define PEDALL_EN 30
#define PEDALL_STEP 10
#define PEDALL_DIR 11
volatile bool IsPLHomed = false;

#define PEDALR_UPPER_LIMIT 33
#define PEDALR_EN 29
#define PEDALR_STEP 12 
#define PEDALR_DIR 13
volatile bool IsPRHomed = false;
#


#define PEDALS_MAX_MAXDISTANCE 8000
#define PEDALS_LEVEL0 2000
#define PEDALS_LEVEL1 4000
#define PEDALS_LEVEL2 6000
#define PEDALS_LEVEL3 8000
#define PEDALS_BASE_SPEED 200

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

	PEDALL_EN 			,
	PEDALL_STEP 		,
	PEDALL_DIR 			,

	PEDALR_EN 			,
	PEDALR_STEP 		,
	PEDALR_DIR 			,
	PANEL_LED
 };


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
	MOVE_MOTORS = 'M',
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
	pinMode(CHAIR2_LEFT_UPPER_LIMIT, INPUT);
	pinMode(CHAIR1_LEFT_ALARM, INPUT);
	pinMode(CHAIR2_RIGHT_UPPER_LIMIT, INPUT);
	pinMode(CHAIR1_RIGHT_ALARM, INPUT);
	pinMode(PEDALL_UPPER_LIMIT, INPUT);
	pinMode(PEDALR_UPPER_LIMIT, INPUT);

	for (int i = 0; i < 24; i++) {
		pinMode(motorControlPins[i], OUTPUT);
		digitalWrite(motorControlPins[i], LOW);
	}

	for (int i = 0; i < buttonCount; i++) {
		pinMode(buttonPins[i], INPUT_PULLUP);
		digitalWrite(buttonPins[i], HIGH); //redundant but just in case
	}

	motorC1L.setEnablePin(CHAIR1_LEFT_EN);
	motorC1L.setPinsInverted(false, false, true);
	motorC1L.enableOutputs();
	motorC1L.setAcceleration(MOTORS_ACCEL);
	motorC1L.setMaxSpeed(MOTORS_BASE_SPEED);
	attachInterrupt(digitalPinToInterrupt(CHAIR1_LEFT_UPPER_LIMIT), stopMotorC1L, RISING);

	motorC1R.setEnablePin(CHAIR1_RIGHT_EN);
	motorC1R.setPinsInverted(false, false, true);
	motorC1R.enableOutputs();
	motorC1R.setAcceleration(MOTORS_ACCEL);
	motorC1R.setMaxSpeed(500);
	attachInterrupt(digitalPinToInterrupt(CHAIR1_RIGHT_UPPER_LIMIT), stopMotorC1R, RISING);

	motorC2L.setEnablePin(CHAIR2_RIGHT_EN);
	motorC2L.setPinsInverted(false, false, true);
	motorC2L.enableOutputs();
	motorC2L.setAcceleration(MOTORS_ACCEL);
	motorC2L.setMaxSpeed(MOTORS_BASE_SPEED);
	attachInterrupt(digitalPinToInterrupt(CHAIR2_LEFT_UPPER_LIMIT), stopMotorC2L, RISING);

	motorC2R.setEnablePin(CHAIR2_LEFT_EN);
	motorC2R.setPinsInverted(false, false, true);
	motorC2R.enableOutputs();
	motorC2R.setAcceleration(MOTORS_ACCEL);
	motorC2R.setMaxSpeed(MOTORS_BASE_SPEED);
	attachInterrupt(digitalPinToInterrupt(CHAIR2_RIGHT_UPPER_LIMIT), stopMotorC2R, RISING);

	motorPedalsL.setEnablePin(PEDALL_EN);
	motorPedalsL.setPinsInverted(false, false, true);
	motorPedalsL.enableOutputs();
	motorPedalsL.setMaxSpeed(MOTORS_BASE_SPEED);
	attachInterrupt(digitalPinToInterrupt(PEDALL_UPPER_LIMIT), stopMotorPL, RISING);

	motorPedalsR.setEnablePin(PEDALR_EN);
	motorPedalsR.setPinsInverted(false, false, true);
	motorPedalsR.enableOutputs();
	motorPedalsR.setMaxSpeed(MOTORS_BASE_SPEED);
	attachInterrupt(digitalPinToInterrupt(PEDALR_UPPER_LIMIT), stopMotorPR, RISING);
	//end motors
	

	//pedals
	pinMode(PEDAL1_A, INPUT_PULLUP);
	pinMode(PEDAL1_B, INPUT_PULLUP);
	pinMode(PEDAL2_A, INPUT_PULLUP);
	pinMode(PEDAL2_B, INPUT_PULLUP);
	
	//DUMP whatever is in the serial and wait for the clean go.
	while (Serial1.available()) Serial1.read();
	//end pedals
	SetState(E_STATE::HOMING);
	Serial.println("Setup finished.");
	digitalWrite(PANEL_LED, HIGH);
}

void loop() {
	switch (_state)
	{
		case HOMING:{

			Serial.println(F("Homing pedals started..."));
			if (!HandlePedalsHoming()) {
				Serial.println("Pedals homing Failed.");
			}
			Serial.println(F("Homing Pedals Finished..."));
			Serial.println(F("Homing chairs started..."));

			if (!HandleChairsHoming()) {
				Serial.println("Chairs homing Failed.");
			}
			
			Serial.println(F("Homing chairs Finished..."));
			SetState(E_STATE::LISTENING);
			
		}break;
		case LISTENING: {
#if DEBUG == 1
			
			if (ProcessIncommingMsg(Serial))
			{						
				SetState(E_STATE::EXECUTE_CMD);
			}else 
				SetState(E_STATE::MOVE_MOTORS);
#endif

#if DEBUG == 0 
			if (ProcessIncommingMsg(Serial1))
			{
				SetState(E_STATE::EXECUTE_CMD);
			}
			else 
				SetState(E_STATE::MOVE_MOTORS);
#endif

		}break;
		case MOVE_MOTORS: {

			if (motorC1L.isRunning()) motorC1L.run();
			if (motorC1R.isRunning()) motorC1R.run();
			if (motorC2L.isRunning()) motorC2L.run();
			if (motorC2R.isRunning()) motorC2R.run();
			if (motorPedalsL.isRunning()) motorPedalsL.run();
			if (motorPedalsR.isRunning()) motorPedalsR.run();

			SetState(E_STATE::READY);
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
	motorC1L.setAcceleration(CHAIR_HOMING_ACCELERATION);
	motorC1L.setMaxSpeed(CHAIR_HOMING_SPEED);
	motorC1L.moveTo(-1 * CHAIR_MAX_DISTANCE);

	motorC1R.setAcceleration(CHAIR_HOMING_ACCELERATION);
	motorC1R.setMaxSpeed(CHAIR_HOMING_SPEED);
	motorC1R.moveTo(-1 * CHAIR_MAX_DISTANCE);

	motorC2L.setAcceleration(CHAIR_HOMING_ACCELERATION);
	motorC2L.setMaxSpeed(CHAIR_HOMING_SPEED);
	motorC2L.moveTo(-1 * CHAIR_MAX_DISTANCE);

	motorC2R.setAcceleration(CHAIR_HOMING_ACCELERATION);
	motorC2R.setMaxSpeed(CHAIR_HOMING_SPEED);
	motorC2R.moveTo(-1 * CHAIR_MAX_DISTANCE);
	
	while (1)
	{

		if (motorC1L.distanceToGo() != 0)
		{
			motorC1L.run();
		}
		else IsC1LHomed = true;

		if (motorC1R.distanceToGo() != 0)
		{
			motorC1R.run();
		}
		else IsC1RHomed = true;

		if (motorC2L.distanceToGo() != 0)
		{
			motorC2L.run();
		}
		else IsC2LHomed = true;

		if (motorC2R.distanceToGo() != 0)
		{
			motorC2R.run();
		}
		else IsC2RHomed = true;

		if (IsC1LHomed && IsC1RHomed && IsC2LHomed && IsC2RHomed) break;
	}
	

	if (motorC1L.distanceToGo() == 0) motorC1L.moveTo(CHAIR_BASE_POSITION);
	if (motorC1R.distanceToGo() == 0) motorC1R.moveTo(CHAIR_BASE_POSITION);
	if (motorC2L.distanceToGo() == 0) motorC2L.moveTo(CHAIR_BASE_POSITION);
	if (motorC2R.distanceToGo() == 0) motorC2R.moveTo(CHAIR_BASE_POSITION);

	while (motorC1L.distanceToGo() != 0 || motorC1R.distanceToGo() != 0 || motorC2L.distanceToGo() != 0 || motorC2R.distanceToGo() != 0)
	{
		motorC1L.run();
		motorC1R.run();
		motorC2L.run();
		motorC2R.run();
	}

	motorC1L.setCurrentPosition(0);
	motorC1R.setCurrentPosition(0);
	motorC2L.setCurrentPosition(0);
	motorC2R.setCurrentPosition(0);

	return true;
}

bool HandlePedalsHoming() //can be blocking
{
	motorPedalsL.setAcceleration(PEDAL_HOMING_ACCELERATION); 
	motorPedalsL.setSpeed(PEDAL_HOMING_SPEED); 
	motorPedalsL.moveTo(-1 * PEDALS_MAX_MAXDISTANCE);
	
	motorPedalsR.setAcceleration(PEDAL_HOMING_ACCELERATION);
	motorPedalsR.setSpeed(PEDAL_HOMING_SPEED);
	motorPedalsR.moveTo(-1 * PEDALS_MAX_MAXDISTANCE);

	IsPLHomed = false;
	IsPRHomed = false;

	while (1)
	{	

		if (motorPedalsL.distanceToGo() !=0)
		{
			motorPedalsL.run();
		}else IsPLHomed = true;

		if (motorPedalsR.distanceToGo() != 0)
		{
			motorPedalsR.run();
		}
		else IsPRHomed = true;

		if (IsPRHomed && IsPLHomed) break;
	}

	if (motorPedalsL.distanceToGo() == 0) motorPedalsL.moveTo(PEDALS_LEVEL0);
	if (motorPedalsR.distanceToGo() == 0) motorPedalsR.moveTo(PEDALS_LEVEL0);

	while (motorPedalsR.distanceToGo() != 0 || motorPedalsL.distanceToGo() != 0)
	{
		motorPedalsR.run();
		motorPedalsL.run();
	}

	motorPedalsR.setCurrentPosition(0); 
	motorPedalsL.setCurrentPosition(0);
	return true;
}

void stopMotorC1L() //interrupt call
{	
	Serial.println("Stop:C1L");
	if (!IsC1LHomed) HandleEndStopHitMotor(motorC1L, CHAIR_MOTORS_BASE_SPEED, CHAIR_BASE_POSITION);
}

void stopMotorC1R() //interrupt call
{
	Serial.println("Stop:C1R");
	if (!IsC1RHomed)  HandleEndStopHitMotor(motorC1R, CHAIR_MOTORS_BASE_SPEED, CHAIR_BASE_POSITION);
}

void stopMotorC2L() //interrupt call
{
	Serial.println("Stop:C2L");
	if (!IsC2LHomed) HandleEndStopHitMotor(motorC2L, CHAIR_MOTORS_BASE_SPEED, CHAIR_BASE_POSITION);
}

void stopMotorC2R() //interrupt call
{
	Serial.println("Stop:C2R");
	if (!IsC2RHomed) HandleEndStopHitMotor(motorC2R, CHAIR_MOTORS_BASE_SPEED, CHAIR_BASE_POSITION);
}

void stopMotorPL() //interrupt call
{
	Serial.println("Stop:PEDAL_LEFT");
	if (!IsPLHomed) HandleEndStopHitMotor(motorPedalsL, PEDAL_MOTORS_BASE_SPEED, PEDALS_LEVEL0);
}

void stopMotorPR() //interrupt call
{
	Serial.println("Stop:PEDAL_RIGHT");
	if (!IsPRHomed) HandleEndStopHitMotor(motorPedalsR,PEDAL_MOTORS_BASE_SPEED, PEDALS_LEVEL0);
}

bool HandleEndStopHitMotor(AccelStepper& stepper,int desiredSpeed,int retraction)  //Sub interrupt call
{
	stepper.setCurrentPosition(0);
	stepper.setSpeed(desiredSpeed);
	stepper.move(retraction);
	Serial.print("Retracting to:"); Serial.println(retraction);
	
	//stepper.stop(); 
	
	return true;
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
		
		Serial.print(F("Pedals resistance:")); Serial.println(pedalResistance);
		SetPedalResistance(pedalResistance);
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

void SetPedalResistance(int pedalResistance)
{
	
	if (pedalPreviousResistance != pedalResistance)
	{
		Serial.print("Current :");
		Serial.println(motorPedalsL.currentPosition());
		
		int factor = 1;
		int jump = 0;
		if (pedalPreviousResistance > pedalResistance)
		{
			factor = -1;
			jump = pedalResistance - pedalPreviousResistance;

			Serial.print("Jump:"); Serial.println(jump);
		}
		if (pedalPreviousResistance < pedalResistance)
		{
			factor = 1;
			jump = pedalPreviousResistance - pedalResistance;
			Serial.print("Jump:"); Serial.println(jump);

		}

		factor *= abs(jump);
		Serial.print("Target:"); Serial.println(factor * PEDALS_LEVEL0);
		
		Serial.println(factor);
		Serial.println(pedalPreviousResistance);
		Serial.println(pedalResistance);


		motorPedalsR.setAcceleration(PEDAL_MOTORS_ACCEL);
		motorPedalsL.setAcceleration(PEDAL_MOTORS_ACCEL);
		motorPedalsR.setSpeed(PEDALS_BASE_SPEED);
		motorPedalsL.setSpeed(PEDALS_BASE_SPEED);
		
		switch (pedalResistance)
		{
		case 0: {
			motorPedalsL.moveTo(factor *  PEDALS_LEVEL0);
			motorPedalsR.moveTo(factor *  PEDALS_LEVEL0);
		}break;
		case 1: {
			motorPedalsL.moveTo(factor *  PEDALS_LEVEL0);
			motorPedalsR.moveTo(factor *  PEDALS_LEVEL0);
		}break;
		case 2: {
			motorPedalsL.moveTo(factor *  PEDALS_LEVEL0);
			motorPedalsR.moveTo(factor *  PEDALS_LEVEL0);
		}break;
		case 3: {
			motorPedalsL.moveTo(factor *  PEDALS_LEVEL0);
			motorPedalsR.moveTo(factor *  PEDALS_LEVEL0);
		}break;
		case 4: {
			motorPedalsL.moveTo(factor * PEDALS_LEVEL0);
			motorPedalsR.moveTo(factor * PEDALS_LEVEL0);
		}break;

		default:
			break;
		}
		pedalPreviousResistance = pedalResistance;

	}
	else Serial.println("No resistance change");
	
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
			pedalResistance = abs(stringToByte(level));
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


