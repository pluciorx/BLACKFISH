#include <AccelStepper.h>

//Set DEBUG 1 to test commands by sending them trough debug serial.
#define DEBUG 1
#define PEDALS_HOMING_ENABLED 0 
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

//Com
#define COM_BAUD_Debug 115200
#define COM_BAUD_PC 115200
#define MAX_INPUT 50 //maximum length of the command..

char input_line[MAX_INPUT] ="";
char error[MAX_INPUT];

#define CHAIR_MAX_SPEED 1000
#define CHAIR_HOMING_SPEED 200
#define CHAIR_HOMING_ACCELERATION 3000

#define CHAIR_BASE_SPEED 7000
#define	CHAIR_ACCEL 8000

//motors (4 for chairs 2 for pedals)
#define CHAIR_MAX_DISTANCE 9000
#define CHAIR_BASE_POSITION 4000

byte vibrationfactor = 1;
#define VIB_C1_L0_MIN -4000
#define VIB_C1_L0_MAX 2000
#define VIB_C1_L1_MIN -3500
#define VIB_C1_L1_MAX 1000
#define VIB_C1_L2_MIN -2500
#define VIB_C1_L2_MAX 1000
#define VIB_C1_L3_MIN -1500
#define VIB_C1_L3_MAX 500
#define VIB_C1_L4_MIN -200
#define VIB_C1_L4_MAX 200

//Chair 1 
#define CHAIR1_LEFT_UPPER_LIMIT 48
#define CHAIR1_LEFT_ALARM 46
#define CHAIR1_LEFT_EN 44
#define CHAIR1_LEFT_STEP 2
#define CHAIR1_LEFT_DIR 3
volatile bool IsC1LHomed = false;
bool C1LSubVibReady = true;

#define CHAIR1_RIGHT_UPPER_LIMIT 49
#define CHAIR1_RIGHT_ALARM 47
#define CHAIR1_RIGHT_EN 45
#define CHAIR1_RIGHT_STEP 4
#define CHAIR1_RIGHT_DIR 5
volatile bool IsC1RHomed = false;
bool C1RSubVibReady = true;
//Chair 2 
#define CHAIR2_RIGHT_UPPER_LIMIT 40
#define CHAIR2_RIGHT_ALARM 38
#define CHAIR2_RIGHT_EN 36
#define CHAIR2_RIGHT_STEP 6
#define CHAIR2_RIGHT_DIR 7
volatile bool IsC2RHomed = false;
bool C2LSubVibReady = true;

#define CHAIR2_LEFT_UPPER_LIMIT 41
#define CHAIR2_LEFT_ALARM 39
#define CHAIR2_LEFT_EN 37
#define CHAIR2_LEFT_STEP 8
#define CHAIR2_LEFT_DIR 9
volatile bool IsC2LHomed = false;
bool C2RSubVibReady = true;

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

#define PEDAL_HOMING_SPEED 1000
#define PEDAL_HOMING_ACCELERATION 3000
#define PEDALS_MAX_MAXDISTANCE 4000
#define PEDAL_BASE_POSITION -1000

#define PEDAL_MAX_SPEED 3000
#define PEDAL_BASE_SPEED 1000
#define	PEDAL_ACCEL 1200

#define PEDALS_LEVEL0 2000
#define PEDALS_LEVEL1 4000
#define PEDALS_LEVEL2 6000
#define PEDALS_LEVEL3 8000
byte pedalResistance = 0;
byte pedalPreviousResistance = 0;


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
	//SerialUSB to talk with host
	SerialUSB.begin(COM_BAUD_PC);
	
	Serial.println(F("<================= Starting =================>"));
	
	//motors
	// Set limit switch inputs
	pinMode(CHAIR1_LEFT_UPPER_LIMIT, INPUT);
	pinMode(CHAIR1_RIGHT_UPPER_LIMIT, INPUT);

	pinMode(CHAIR2_LEFT_UPPER_LIMIT, INPUT);
	pinMode(CHAIR2_RIGHT_UPPER_LIMIT, INPUT);
	
	pinMode(CHAIR1_LEFT_ALARM, INPUT);
	pinMode(CHAIR1_RIGHT_ALARM, INPUT);

	pinMode(PEDALL_UPPER_LIMIT, INPUT);
	pinMode(PEDALR_UPPER_LIMIT, INPUT);

	for (int i = 0; i < 24; i++) {
		pinMode(motorControlPins[i], OUTPUT);
		digitalWrite(motorControlPins[i], LOW);
	}

	for (int i = 0; i < buttonCount; i++) {
		pinMode(buttonPins[i], INPUT);
		digitalWrite(buttonPins[i], HIGH); //redundant but just in case
	}

	motorC1L.setEnablePin(CHAIR1_LEFT_EN);
	motorC1L.setPinsInverted(false, false, true);
	motorC1L.disableOutputs();
	delay(120);
	motorC1L.enableOutputs();
	motorC1L.setMaxSpeed(CHAIR_MAX_SPEED);
	

	motorC1R.setEnablePin(CHAIR1_RIGHT_EN);
	motorC1R.setPinsInverted(false, false, true);
	motorC1R.disableOutputs();
	delay(120);
	motorC1R.enableOutputs();
	motorC1R.setMaxSpeed(CHAIR_MAX_SPEED);
	

	motorC2L.setEnablePin(CHAIR2_RIGHT_EN);
	motorC2L.setPinsInverted(false, false, true);
	motorC2L.disableOutputs();
	delay(120);
	motorC2L.enableOutputs();
	motorC2L.setMaxSpeed(CHAIR_MAX_SPEED);
	

	motorC2R.setEnablePin(CHAIR2_LEFT_EN);
	motorC2R.setPinsInverted(false, false, true);
	motorC2R.disableOutputs();
	delay(120);
	motorC2R.enableOutputs();
	motorC2R.setMaxSpeed(CHAIR_MAX_SPEED);


	motorPedalsL.setEnablePin(PEDALL_EN);
	motorPedalsL.setPinsInverted(false, false, true);
	motorPedalsL.enableOutputs();
	motorPedalsL.setMaxSpeed(PEDAL_MAX_SPEED);
	attachInterrupt(digitalPinToInterrupt(PEDALL_UPPER_LIMIT), stopMotorPL, RISING);

	motorPedalsR.setEnablePin(PEDALR_EN);
	motorPedalsR.setPinsInverted(false, false, true);
	motorPedalsR.enableOutputs();
	motorPedalsR.setMaxSpeed(PEDAL_MAX_SPEED);
	attachInterrupt(digitalPinToInterrupt(PEDALR_UPPER_LIMIT), stopMotorPR, RISING);
	//end motors
	
	//pedals
	pinMode(PEDAL1_A, INPUT_PULLUP);
	pinMode(PEDAL1_B, INPUT_PULLUP);
	pinMode(PEDAL2_A, INPUT_PULLUP);
	pinMode(PEDAL2_B, INPUT_PULLUP);
	
	//DUMP whatever is in the serial and wait for the clean go.
	Serial.flush();
	SerialUSB.flush();

	SetState(E_STATE::HOMING);
	pinMode(PANEL_LED, OUTPUT);
	digitalWrite(PANEL_LED, HIGH);
	
	Serial.println("Setup finished.");
}

void loop() {
	switch (_state)
	{
		case HOMING:{
#if PEDALS_HOMING_ENABLED == 1
			Serial.println(F("Homing pedals started..."));
			if (!HandlePedalsHoming()) {
				Serial.println("Pedals homing Failed.");
			}
			Serial.println(F("Homing Pedals Finished..."));
#endif
			Serial.println(F("Homing chairs started..."));

			if (!HandleChairsHoming()) {
				Serial.println(F("Chairs homing Failed."));
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

			//this will override the  commands sent from programmer
		/*	if (ProcessIncommingMsg(SerialUSB))
			{
				SetState(E_STATE::EXECUTE_CMD);
			}
			else 
				SetState(E_STATE::MOVE_MOTORS);*/

		}break;
		case MOVE_MOTORS: {

			if (motorC1L.distanceToGo() != 0)
			{
				motorC1L.runSpeed();
			}
			else {
				C1LSubVibReady = true;
			}

			if (motorC1R.distanceToGo() != 0)
			{
				motorC1R.runSpeed();
				
			}else C1RSubVibReady = true;

			if (motorC2L.distanceToGo() != 0)
			{
				motorC2L.runSpeed();
			}
			else {
				C2LSubVibReady = true;
			}

			if (motorC2R.distanceToGo() != 0)
			{
				motorC2R.runSpeed();
			}
			else C2RSubVibReady = true;

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
			SetState(E_STATE::READY);
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
	if (IsVibrationEnabled)
	{
		long minRange, maxRange,speed,accel;

		switch (vibrationfactor)
		{
		case 1: {
			minRange = VIB_C1_L0_MIN;
			maxRange = VIB_C1_L0_MAX;

		}break;
		case 2: {
			minRange = VIB_C1_L1_MIN;
			maxRange = VIB_C1_L1_MAX;

		}break;
		case 3: {
			minRange = VIB_C1_L2_MIN;
			maxRange = VIB_C1_L2_MAX;

		}break;
		case 4: {
			minRange = VIB_C1_L3_MIN;
			maxRange = VIB_C1_L3_MAX;

		}break;
		default: {
			minRange = -100;
			maxRange = 100;
		}break;
		}
		speed = random(1, 4) * CHAIR_BASE_SPEED;
		accel = random(1, 4) * CHAIR_ACCEL;

		if (C1LSubVibReady == true && IsVibrationEnabled)
		{
			motorC1L.moveTo(random(minRange, maxRange));
			motorC1L.setSpeed(speed);
			C1LSubVibReady = false;
		}
		if (C1RSubVibReady == true && IsVibrationEnabled)
		{			
			motorC1R.moveTo(random(minRange, maxRange));
			motorC1R.setSpeed(speed);
			C1RSubVibReady = false;
		}
		if (C2LSubVibReady == true && IsVibrationEnabled)
		{

			motorC2L.moveTo(random(minRange, maxRange));
			motorC2L.setSpeed(speed);
			C2LSubVibReady = false;
		}
		if (C2RSubVibReady == true && IsVibrationEnabled)
		{
			motorC2R.moveTo(random(minRange, maxRange));
			motorC2R.setSpeed(speed);
			C2RSubVibReady = false;
		}
	}
}

bool HandleChairsHoming() //can be blocking
{

	//Just in case we are on the endstops
	CheckAndRetractMotors();

	motorC1L.setAcceleration(CHAIR_HOMING_ACCELERATION);
	motorC1L.setSpeed(CHAIR_HOMING_SPEED);
	motorC1L.moveTo(-1 * CHAIR_MAX_DISTANCE);

	motorC1R.setAcceleration(CHAIR_HOMING_ACCELERATION);
	motorC1R.setSpeed(CHAIR_HOMING_SPEED);
	motorC1R.moveTo(-1 * CHAIR_MAX_DISTANCE);
	
	motorC2L.setAcceleration(CHAIR_HOMING_ACCELERATION);
	motorC2L.setSpeed(CHAIR_HOMING_SPEED);
	motorC2L.moveTo(-1 * CHAIR_MAX_DISTANCE);

	motorC2R.setAcceleration(CHAIR_HOMING_ACCELERATION);
	motorC2R.setSpeed(CHAIR_HOMING_SPEED);
	motorC2R.moveTo(-1 * CHAIR_MAX_DISTANCE);
	//Moving motors towards the endstops
	
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
	

	if (motorC1L.distanceToGo() == 0) stopMotorC1L();
	if (motorC1R.distanceToGo() == 0) stopMotorC1R();
	if (motorC2L.distanceToGo() == 0) stopMotorC2L();
	if (motorC2R.distanceToGo() == 0) stopMotorC2R();

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

void CheckAndRetractMotors()
{
	/*detachInterrupt(digitalPinToInterrupt(CHAIR1_LEFT_UPPER_LIMIT));
	detachInterrupt(digitalPinToInterrupt(CHAIR1_RIGHT_UPPER_LIMIT));
	detachInterrupt(digitalPinToInterrupt(CHAIR2_LEFT_UPPER_LIMIT));
	detachInterrupt(digitalPinToInterrupt(CHAIR2_RIGHT_UPPER_LIMIT));*/

	if (digitalRead(CHAIR1_LEFT_UPPER_LIMIT) == HIGH)
	{
		Serial.println("Start at endstop C1L");
		motorC1L.setCurrentPosition(0);
		motorC1L.setAcceleration(CHAIR_HOMING_ACCELERATION);		
		motorC1L.moveTo(800);
		
		while (digitalRead(CHAIR1_LEFT_UPPER_LIMIT) == HIGH)
		{
			if (motorC1L.distanceToGo() != 0 || digitalRead(CHAIR1_LEFT_UPPER_LIMIT) == HIGH)
			{
				motorC1L.run();
			}
			else break;
		}
		
		motorC1L.move(500);
		
		while (motorC1L.distanceToGo() != 0)
		{
			motorC1L.run();
		}

		motorC1L.setCurrentPosition(0);
		Serial.println("C1L endstop cleared;");
	}
		
	if (digitalRead(CHAIR1_RIGHT_UPPER_LIMIT) == HIGH )
	{
		Serial.println("Start at endstop C1R");
		motorC1L.setCurrentPosition(0);
		motorC1R.setAcceleration(CHAIR_HOMING_ACCELERATION);
		motorC1R.moveTo(800);
	
		while (digitalRead(CHAIR1_RIGHT_UPPER_LIMIT) == HIGH )
		{
			if (motorC1R.distanceToGo() != 0 || digitalRead(CHAIR1_RIGHT_UPPER_LIMIT) == HIGH)
			{
				motorC1R.run();
			}
		}

		motorC1R.move(500);
		while (motorC1R.distanceToGo() != 0)
		{
			motorC1R.run();
		}
		
		motorC1R.setCurrentPosition(0);
		Serial.println("C1R endstop cleared;");
	}

	attachInterrupt(digitalPinToInterrupt(CHAIR1_LEFT_UPPER_LIMIT), stopMotorC1L, RISING);
	attachInterrupt(digitalPinToInterrupt(CHAIR1_RIGHT_UPPER_LIMIT), stopMotorC1R, RISING);
	attachInterrupt(digitalPinToInterrupt(CHAIR2_RIGHT_UPPER_LIMIT), stopMotorC2R, RISING);
	attachInterrupt(digitalPinToInterrupt(CHAIR2_LEFT_UPPER_LIMIT), stopMotorC2L, RISING);

}

bool HandlePedalsHoming() //can be blocking
{	
	motorPedalsL.setAcceleration(PEDAL_HOMING_ACCELERATION); 	
	motorPedalsL.moveTo(PEDALS_MAX_MAXDISTANCE);
	motorPedalsL.setSpeed(PEDAL_HOMING_SPEED);

	motorPedalsR.setAcceleration(PEDAL_HOMING_ACCELERATION);
	motorPedalsR.moveTo(PEDALS_MAX_MAXDISTANCE);
	motorPedalsR.setSpeed(PEDAL_HOMING_SPEED);

	IsPLHomed = false;
	IsPRHomed = false;

	while (1)
	{	
		if (motorPedalsL.distanceToGo()!= 0)
		{
			motorPedalsL.runSpeed();
			
		}else IsPLHomed = true;

		if (motorPedalsR.distanceToGo() != 0)
		{
			motorPedalsR.runSpeed();
			
		} else IsPRHomed = true;

		if (IsPRHomed && IsPLHomed)
		{
			break;
		}
	}
	Serial.println("Moved to endstop.");

	//Retract 
	if (motorPedalsL.distanceToGo() != 0) stopMotorPL();
	if (motorPedalsR.distanceToGo() != 0) stopMotorPR();

	while (motorPedalsR.distanceToGo() != 0 || motorPedalsL.distanceToGo() != 0)
	{
		motorPedalsR.runSpeed();
		motorPedalsL.runSpeed();
	}

	motorPedalsR.setCurrentPosition(0); 
	motorPedalsL.setCurrentPosition(0);
	return true;
}

void stopMotorC1L() //interrupt call
{	
	Serial.println("Stop:C1L");
	//detachInterrupt(digitalPinToInterrupt(CHAIR1_LEFT_UPPER_LIMIT));
	if (!IsC1LHomed) HandleEndStopHitMotor(motorC1L, CHAIR_HOMING_SPEED, CHAIR_BASE_POSITION);
}

void stopMotorC1R() //interrupt call
{
	Serial.println("Stop:C1R");
	//detachInterrupt(digitalPinToInterrupt(CHAIR1_RIGHT_UPPER_LIMIT));
	if (!IsC1RHomed)  HandleEndStopHitMotor(motorC1R, CHAIR_HOMING_SPEED, CHAIR_BASE_POSITION);
}

void stopMotorC2L() //interrupt call
{
	Serial.println("Stop:C2L");
	//detachInterrupt(digitalPinToInterrupt(CHAIR2_LEFT_UPPER_LIMIT));
	if (!IsC2LHomed) HandleEndStopHitMotor(motorC2L, CHAIR_HOMING_SPEED, CHAIR_BASE_POSITION);
}

void stopMotorC2R() //interrupt call
{
	Serial.println("Stop:C2R");
	//detachInterrupt(digitalPinToInterrupt(CHAIR2_RIGHT_UPPER_LIMIT));
	if (!IsC2RHomed) HandleEndStopHitMotor(motorC2R, CHAIR_HOMING_SPEED, CHAIR_BASE_POSITION);
}

void stopMotorPL() //interrupt call
{
	Serial.print("Stop:PEDAL_LEFT at:"); Serial.println(motorPedalsL.currentPosition());
	detachInterrupt(digitalPinToInterrupt(PEDALL_UPPER_LIMIT));
	if (!IsPLHomed) HandleEndStopHitMotor(motorPedalsL, -1 * PEDAL_HOMING_SPEED, PEDAL_BASE_POSITION);

}

void stopMotorPR() //interrupt call
{
	Serial.print("Stop:PEDAL_RIGHT at:"); Serial.println(motorPedalsR.currentPosition());
	detachInterrupt(digitalPinToInterrupt(PEDALR_UPPER_LIMIT));
	if (!IsPRHomed) HandleEndStopHitMotor(motorPedalsR,-1 * PEDAL_HOMING_SPEED,PEDAL_BASE_POSITION);

}

bool HandleEndStopHitMotor(AccelStepper& stepper,float desiredSpeed,long retraction)  //Sub interrupt call
{
	stepper.stop();
	stepper.setCurrentPosition(0); // endstop is at position 0 
	stepper.moveTo(retraction);
	stepper.setSpeed(desiredSpeed);
	Serial.print("Retracting to:"); Serial.println(retraction);

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
		SerialUSB.println(F("pong"));
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
	return true;
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


		motorPedalsR.setAcceleration(PEDAL_ACCEL);
		motorPedalsL.setAcceleration(PEDAL_ACCEL);
		motorPedalsR.setSpeed(PEDAL_BASE_SPEED);
		motorPedalsL.setSpeed(PEDAL_BASE_SPEED);
		
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
		vibrationfactor = pedalResistance;
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
#if DEBUG == 1
				Serial.print(F("Invalid resistance range:")); Serial.println(level[0]);
#endif
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

bool ProcessIncommingMsg(Stream& sourceSerial)
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
			SerialUSB.println(cMsg);
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
	SerialUSB.println(cMsg);
	Serial.print(F("==>")); Serial.println(cMsg);

//#if DEBUG == 1 	
//	Serial.print("Position Pedal_"); Serial.print(pedalNo); Serial.println(":");
//	Serial.print(int(pedalAngle * (-1.8)));
//	Serial.println("deg");
//#endif
}

void Console()
{
#if DEBUG == 1 

#endif
}

void HandlePedaling()
{
	bool A = digitalRead(PEDAL1_A), B = digitalRead(PEDAL1_B);

	p1_counter += (A ^ p1_prevA) | (B ^ p1_prevB) ? A ^ p1_prevB ? 1 : -1 : 0;

	p1_prevA = A;
	p1_prevB = B;
	if (p1_counter > p1_prevCounter)
	{
		int angle = int(p1_counter * (-1.8));
		if (angle == 361) {
			angle = 0;
			p1_counter =0;
		}
		SendPedalState(1, angle);
		p1_prevCounter = p1_counter;
	}

	A = digitalRead(PEDAL2_A), B = digitalRead(PEDAL2_B);

	p2_counter += (A ^ p2_prevA) | (B ^ p2_prevB) ? A ^ p2_prevB ? 1 : -1 : 0;
	 
	p2_prevA = A;
	p2_prevB = B;

	if (p2_counter > p2_prevCounter)
	{
		int angle = int(p2_counter * (-1.8));
		if (angle == 361) {
			angle = 0;
			p2_counter = 0;
		}
		SendPedalState(2, angle);
		p2_prevCounter = p2_counter;
	}	
}


