#include <AccelStepper.h>

//Set DEBUG 1 to test commands by sending them trough debug serial.
#define DEBUG 1
#define PEDALS_HOMING_ENABLED 0 
// Pin definitions
//buttons (2 x 4 )
// 

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

char input_line[MAX_INPUT] = "";
char error[MAX_INPUT];

#define CHAIR_MIN_PULSE_WIDTH 3

#define CHAIR_MAX_SPEED 600

#define CHAIR_HOMING_SPEED 200
#define CHAIR_HOMING_ACCELERATION 200

#define CHAIR_BASE_SPEED 300
#define	CHAIR_ACCEL 700

//motors (4 for chairs 2 for pedals)
#define CHAIR_MAX_DISTANCE -1600
#define CHAIR_BASE_POSITION 800

byte vibrationfactor = 1;
#define VIB_C1_L0_MIN -7500
#define VIB_C1_L0_MAX 6000
#define VIB_C1_L1_MIN -7000
#define VIB_C1_L1_MAX 5000
#define VIB_C1_L2_MIN -7000
#define VIB_C1_L2_MAX 4000
#define VIB_C1_L3_MIN -5000
#define VIB_C1_L3_MAX 4000
#define VIB_C1_L4_MIN -6000
#define VIB_C1_L4_MAX 3000

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
	MOTOR_RST = 'Y',
	S_ERROR = 'X'
};

enum CommandType {
	VibStart,
	VibStop,
	PedalResistance,
	Ping,
	Kill,
	MotorReset,
	Unknown
};

volatile E_STATE _state = E_STATE::SETUP;
volatile E_STATE _prevState = E_STATE::SETUP;

void setup() {

	Serial.begin(115200);
  
	Serial.println("<================= Starting =================>");

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
	motorC1L.setMinPulseWidth(CHAIR_MIN_PULSE_WIDTH);


	motorC1R.setEnablePin(CHAIR1_RIGHT_EN);
	motorC1R.setPinsInverted(false, false, true);
	motorC1R.disableOutputs();
	delay(120);
	motorC1R.enableOutputs();
	motorC1R.setMaxSpeed(CHAIR_MAX_SPEED);
	motorC1R.setMinPulseWidth(CHAIR_MIN_PULSE_WIDTH);

	motorC2L.setEnablePin(CHAIR2_RIGHT_EN);
	motorC2L.setPinsInverted(false, false, true);
	motorC2L.disableOutputs();
	delay(120);
	motorC2L.enableOutputs();
	motorC2L.setMaxSpeed(CHAIR_MAX_SPEED);
	motorC2L.setMinPulseWidth(CHAIR_MIN_PULSE_WIDTH);

	motorC2R.setEnablePin(CHAIR2_LEFT_EN);
	motorC2R.setPinsInverted(false, false, true);
	motorC2R.disableOutputs();
	delay(120);
	motorC2R.enableOutputs();
	motorC2R.setMaxSpeed(CHAIR_MAX_SPEED);
	motorC2R.setMinPulseWidth(CHAIR_MIN_PULSE_WIDTH);

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
	pinMode(PEDAL1_A, INPUT);
	pinMode(PEDAL1_B, INPUT);
	pinMode(PEDAL2_A, INPUT);
	pinMode(PEDAL2_B, INPUT);

	//DUMP whatever is in the serial and wait for the clean go.
	Serial.flush();


	SetState(E_STATE::HOMING);
	pinMode(PANEL_LED, OUTPUT);
	digitalWrite(PANEL_LED, HIGH);

	Serial.println("Setup finished.");
}

void loop() {
	switch (_state)
	{
	case HOMING: {
#if PEDALS_HOMING_ENABLED == 1
		Serial.println("Homing pedals started...");
		if (!HandlePedalsHoming()) {
			Serial.println("Pedals homing Failed.");
		}
		Serial.println("Homing Pedals Finished...");
#endif
		Serial.println("Homing chairs started...");


		if (!HandleChairsHoming()) {
			Serial.println("Chairs homing Failed.");
		}

		Serial.println("Homing chairs Finished...");
		
		SetState(E_STATE::LISTENING);

	}break;
	case LISTENING: {
#if DEBUG == 1

		if (ProcessIncommingMsg(Serial))
		{
			SetState(E_STATE::EXECUTE_CMD);
		}
		else
			SetState(E_STATE::READY);
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
		
		if (IsVibrationEnabled)
		{

			/*while (motorC1L.distanceToGo() != 0 || motorC1R.distanceToGo() != 0 || motorC2L.distanceToGo() != 0 || motorC2R.distanceToGo() != 0)
			{
				
				motorC1L.run();
				motorC1R.run();
				motorC2L.run();
				motorC2R.run();
				if (motorC1L.distanceToGo() == 0) C1LSubVibReady = true;
				if (motorC1R.distanceToGo() == 0) C1RSubVibReady = true;
				if (motorC2L.distanceToGo() == 0) C2LSubVibReady = true;
				if (motorC2R.distanceToGo() == 0) C2RSubVibReady = true;
			}*/

			/*if (motorC1L.distanceToGo() != 0 || motorC1R.distanceToGo() != 0 || motorC2L.distanceToGo() != 0 || motorC2R.distanceToGo() != 0)
			{
				delayMicroseconds(4);
				motorC1L.run();
				motorC1R.run();
				motorC2L.run();
				motorC2R.run();
				if (motorC1L.distanceToGo() == 0) C1LSubVibReady = true;
				if (motorC1R.distanceToGo() == 0) C1RSubVibReady = true;
				if (motorC2L.distanceToGo() == 0) C2LSubVibReady = true;
				if (motorC2R.distanceToGo() == 0) C2RSubVibReady = true;
			}*/
			
			for (int i = 0; i < 1000; i++)
			{
				
				motorC1L.run();
				motorC1R.run();
				motorC2L.run();
				motorC2R.run();
				if (motorC1L.distanceToGo() == 0) C1LSubVibReady = true;
				if (motorC1R.distanceToGo() == 0) C1RSubVibReady = true;
				if (motorC2L.distanceToGo() == 0) C2LSubVibReady = true;
				if (motorC2R.distanceToGo() == 0) C2RSubVibReady = true;
			}
			
			/*delayMicroseconds(4);
			if (motorC1L.distanceToGo() != 0)  motorC1L.run();
			else  C1LSubVibReady = true;

			if (motorC1R.distanceToGo() != 0)  motorC1R.run();
			else  C1RSubVibReady = true;

			if (motorC2L.distanceToGo() != 0)  motorC2L.run();
			else  C2LSubVibReady = true;

			if (motorC2R.distanceToGo() != 0)  motorC2R.run();
			else  C2RSubVibReady = true;*/
		}
		else {
			motorC1L.disableOutputs();
			motorC1R.disableOutputs();
			motorC2L.disableOutputs();
			motorC2R.disableOutputs();
		}
		//if (motorPedalsL.isRunning()) motorPedalsL.run();
		//if (motorPedalsR.isRunning()) motorPedalsR.run();

		SetState(E_STATE::LISTENING);
	}break;
	case EXECUTE_CMD: {
		CommandType cmd = GetCMDFromInput(input_line);

		if (cmd == CommandType::Unknown)
		{
			SetError("Command execution failure");
			SetState(E_STATE::S_ERROR);
		}else ExecuteCMD(cmd);

	}break;

	case READY: {

		HandlePanelPress();
		HandlePedaling();
		HandleVibrations();
		SetState(E_STATE::MOVE_MOTORS);
	}break;
	case KILL: {
		

	}break;
	case MOTOR_RST: {
		motorC1L.disableOutputs();
		delay(120);
		motorC1L.enableOutputs();

		motorC1R.disableOutputs();
		delay(120);
		motorC1R.enableOutputs();

		motorC2L.disableOutputs();
		delay(120);
		motorC2L.enableOutputs();

		motorC2R.disableOutputs();
		delay(120);
		motorC2R.enableOutputs();
		SetState(E_STATE::LISTENING);
	}break;
	case S_ERROR: {
		Serial.print(F("ERROR has occured:")); Serial.println(error);
		delay(1500);
		SetState(E_STATE::READY);
	}break;
	}
}

void SetError(char* errorInput)
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
		float minRange, maxRange;

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


		if (C1LSubVibReady)
		{
			float speed = CHAIR_BASE_SPEED * random (1,6) ;
			float accel = CHAIR_ACCEL / random(1, 3);
			float newPosition = random(minRange, maxRange);
			Serial.print("C1L New Range min/max:"); Serial.println(minRange); Serial.println(maxRange);
			Serial.print("C1L New vibration move s/a/p:");
			Serial.println(speed);
			Serial.println(accel);
			Serial.println(newPosition);
			motorC1L.enableOutputs();
			motorC1L.setAcceleration(accel);	
			//motorC1L.setSpeed(speed);
			motorC1L.moveTo(newPosition);
			

			C1LSubVibReady = false;
		}
		if (C1RSubVibReady)
		{
			float speed = CHAIR_BASE_SPEED * random(1, 6);
			float accel = CHAIR_ACCEL * random(1, 3);
			float newPosition = random(minRange, maxRange);
			Serial.println("C1R New Range min/max:"); Serial.println(minRange); Serial.println(maxRange);
			Serial.println("C1R New vibration move s/a/p:");
			Serial.println(speed);
			Serial.println(accel);
			Serial.println(newPosition);

			motorC1R.enableOutputs();
			motorC1R.setAcceleration(accel);		
			//motorC1R.setSpeed(speed);
			motorC1R.moveTo(newPosition);
			
			C1RSubVibReady = false;
		}
		if (C2LSubVibReady)
		{
			float speed = CHAIR_BASE_SPEED * random(1, 6);
			float accel = CHAIR_ACCEL * random(1, 3);
			float newPosition = random(minRange, maxRange);
			Serial.print("C2L New Range min/max:"); Serial.println(minRange); Serial.println(maxRange);
			Serial.print("C2L New vibration move s/a/p:");
			Serial.println(speed);
			Serial.println(accel);
			Serial.println(newPosition);
			motorC2L.enableOutputs();
			motorC2L.setAcceleration(accel);
			//motorC1L.setSpeed(speed);
			motorC2L.moveTo(newPosition);


			C2LSubVibReady = false;
		}
		if (C2RSubVibReady)
		{
			float speed = CHAIR_BASE_SPEED * random(1, 6);
			float accel = CHAIR_ACCEL * random(1, 3);
			float newPosition = random(minRange, maxRange);
			Serial.println("C2R New Range min/max:"); Serial.println(minRange); Serial.println(maxRange);
			Serial.println("C2R New vibration move s/a/p:");
			Serial.println(speed);
			Serial.println(accel);
			Serial.println(newPosition);

			motorC2R.enableOutputs();
			motorC2R.setAcceleration(accel);
			//motorC1R.setSpeed(speed);
			motorC2R.moveTo(newPosition);

			C2RSubVibReady = false;
		}


		/*if (C2LSubVibReady && IsVibrationEnabled)
		{

			motorC2L.moveTo(random(minRange, maxRange));
			motorC2L.setSpeed(speed);
			C2LSubVibReady = false;
		}
		if (C2RSubVibReady && IsVibrationEnabled)
		{
			motorC2R.moveTo(random(minRange, maxRange));
			motorC2R.setSpeed(speed);
			C2RSubVibReady = false;
		}*/
	}
}

bool HandleChairsHoming() //can be blocking
{

	//Just in case we are on the endstops
	CheckAndRetractMotors();

	attachInterrupt(digitalPinToInterrupt(CHAIR1_LEFT_UPPER_LIMIT), stopMotorC1L, RISING);
	attachInterrupt(digitalPinToInterrupt(CHAIR1_RIGHT_UPPER_LIMIT), stopMotorC1R, RISING);
	attachInterrupt(digitalPinToInterrupt(CHAIR2_RIGHT_UPPER_LIMIT), stopMotorC2R, RISING);
	attachInterrupt(digitalPinToInterrupt(CHAIR2_LEFT_UPPER_LIMIT), stopMotorC2L, RISING);

	motorC1L.setAcceleration(CHAIR_HOMING_ACCELERATION);
	motorC1L.move(CHAIR_MAX_DISTANCE);
	motorC1L.setSpeed(CHAIR_HOMING_SPEED);

	motorC1R.setAcceleration(CHAIR_HOMING_ACCELERATION);
	motorC1R.move(CHAIR_MAX_DISTANCE);
	motorC1R.setSpeed(CHAIR_HOMING_SPEED);

	motorC2L.setAcceleration(CHAIR_HOMING_ACCELERATION);
	motorC2L.move(CHAIR_MAX_DISTANCE);
	motorC2L.setSpeed(CHAIR_HOMING_SPEED);

	motorC2R.setAcceleration(CHAIR_HOMING_ACCELERATION);
	motorC2R.move(CHAIR_MAX_DISTANCE);
	motorC2R.setSpeed(CHAIR_HOMING_SPEED);
	//Moving motors towards the endstops

	while (1)
	{
		if (motorC1L.distanceToGo() != 0)
		{
			motorC1L.runSpeedToPosition();
		}
		else
		{
			IsC1LHomed = true;
		}

		if (motorC1R.distanceToGo() != 0)
		{
			motorC1R.runSpeedToPosition();
		}
		else
		{
			IsC1RHomed = true;
		}

		if (motorC2L.distanceToGo() != 0)
		{
			motorC2L.runSpeedToPosition();
		}
		else
		{
			IsC2LHomed = true;
		}

		if (motorC2R.distanceToGo() != 0)
		{
			motorC2R.runSpeedToPosition();
		}
		else
		{
			IsC2RHomed = true;
		}

		if (IsC1LHomed && IsC1RHomed && IsC2LHomed && IsC2RHomed)
		{
			break;
		}
	}

	Serial.println("all motors reached endstop or at max distance;");
	if (motorC1L.distanceToGo() == 0) stopMotorC1L();
	if (motorC1R.distanceToGo() == 0) stopMotorC1R();
	if (motorC2L.distanceToGo() == 0) stopMotorC2L();
	if (motorC2R.distanceToGo() == 0) stopMotorC2R();

	while (motorC1L.distanceToGo() != 0 || motorC1R.distanceToGo() != 0 || motorC2L.distanceToGo() != 0 || motorC2R.distanceToGo() != 0)
	{
		motorC1L.runSpeedToPosition();
		motorC1R.runSpeedToPosition();
		motorC2L.runSpeedToPosition();
		motorC2R.runSpeedToPosition();
	}

	motorC1L.setCurrentPosition(0);
	motorC1R.setCurrentPosition(0);
	motorC2L.setCurrentPosition(0);
	motorC2R.setCurrentPosition(0);

	return true;
}

void CheckAndRetractMotors()
{

	while (digitalRead(CHAIR1_LEFT_UPPER_LIMIT) == HIGH)
	{
		Serial.println("Start at endstop C1L");
		motorC1L.setCurrentPosition(0);
		motorC1L.move(100000);
		motorC1L.setSpeed(CHAIR_HOMING_SPEED);

		while (motorC1L.runSpeedToPosition() || digitalRead(CHAIR1_LEFT_UPPER_LIMIT) == HIGH);
		Serial.println("C1L endstop off");
		delay(500);
		motorC1L.move(500);
		motorC1L.setSpeed(CHAIR_HOMING_SPEED);

		Serial.println("C1L  move");
		while (motorC1L.distanceToGo() != 0)
		{
			motorC1L.runSpeedToPosition();
		}
		Serial.println("C1L  stop");

		motorC1L.setCurrentPosition(0);
		Serial.println("C1L endstop cleared;");

	}

	while (digitalRead(CHAIR1_RIGHT_UPPER_LIMIT) == HIGH)
	{
		Serial.println("Start at endstop C1R");
		motorC1R.setCurrentPosition(0);
		motorC1R.move(100000);
		motorC1R.setSpeed(CHAIR_HOMING_SPEED);

		while (motorC1R.runSpeedToPosition() || digitalRead(CHAIR1_RIGHT_UPPER_LIMIT) == HIGH);
		
		motorC1R.move(500);
		motorC1R.setSpeed(CHAIR_HOMING_SPEED);

		Serial.println("C1R  move");
		while (motorC1R.distanceToGo() != 0)
		{
			motorC1R.runSpeedToPosition();
		}
		Serial.println("C1R  stop");

		motorC1R.setCurrentPosition(0);
		Serial.println("C1R endstop cleared;");
	}

	Serial.println("Endstops clear.");
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
		if (motorPedalsL.distanceToGo() != 0)
		{
			motorPedalsL.runSpeed();

		}
		else IsPLHomed = true;

		if (motorPedalsR.distanceToGo() != 0)
		{
			motorPedalsR.runSpeed();

		}
		else IsPRHomed = true;

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
	Serial.print("Stop C1L:"); Serial.println(motorC1L.currentPosition());

	//detachInterrupt(digitalPinToInterrupt(CHAIR1_LEFT_UPPER_LIMIT));
	if (!IsC1LHomed) HandleEndStopHitMotor(motorC1L, CHAIR_HOMING_SPEED, CHAIR_BASE_POSITION);
}

void stopMotorC1R() //interrupt call
{
	Serial.print("Stop C1R:"); Serial.println(motorC1R.currentPosition());
	//detachInterrupt(digitalPinToInterrupt(CHAIR1_RIGHT_UPPER_LIMIT));
	if (!IsC1RHomed)  HandleEndStopHitMotor(motorC1R, CHAIR_HOMING_SPEED, CHAIR_BASE_POSITION);
}

void stopMotorC2L() //interrupt call
{
	Serial.print("Stop C2L:"); Serial.println(motorC2L.currentPosition());
	//detachInterrupt(digitalPinToInterrupt(CHAIR2_LEFT_UPPER_LIMIT));
	if (!IsC2LHomed) HandleEndStopHitMotor(motorC2L, CHAIR_HOMING_SPEED, CHAIR_BASE_POSITION);
}

void stopMotorC2R() //interrupt call
{
	Serial.print("Stop C2R:"); Serial.println(motorC2R.currentPosition());
	//detachInterrupt(digitalPinToInterrupt(CHAIR2_RIGHT_UPPER_LIMIT));
	if (!IsC2RHomed) HandleEndStopHitMotor(motorC2R, CHAIR_HOMING_SPEED, CHAIR_BASE_POSITION);
}

void stopMotorPL() //interrupt call
{
	Serial.print("Stop PEDAL_LEFT:"); Serial.println(motorPedalsL.currentPosition());
	detachInterrupt(digitalPinToInterrupt(PEDALL_UPPER_LIMIT));
	if (!IsPLHomed) HandleEndStopHitMotor(motorPedalsL, -1 * PEDAL_HOMING_SPEED, PEDAL_BASE_POSITION);

}

void stopMotorPR() //interrupt call
{
	Serial.print("Stop PEDAL_RIGHT:"); Serial.println(motorPedalsR.currentPosition());
	detachInterrupt(digitalPinToInterrupt(PEDALR_UPPER_LIMIT));
	if (!IsPRHomed) HandleEndStopHitMotor(motorPedalsR, -1 * PEDAL_HOMING_SPEED, PEDAL_BASE_POSITION);

}

bool HandleEndStopHitMotor(AccelStepper& stepper, float desiredSpeed, long retraction)  //Sub interrupt call
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
		
		SetState(E_STATE::READY);

	}break;
	case VibStop://stopVibration
	{

		Serial.println(F("Vibration OFF"));
		IsVibrationEnabled = false;
		SetState(E_STATE::READY);
	}break;
	case PedalResistance://pedalResistance|ID
	{
		Serial.print(F("Pedals resistance:")); Serial.println(pedalResistance);
		SetPedalResistance(pedalResistance);
		SetState(E_STATE::READY);
	}break;
	case Ping:
	{
		//reserved for future use...
		Serial.println(F("==>pong"));
		
		SetState(E_STATE::READY);
		return true;
	}break;
	case Kill:
	{
		//reserved for future use...
		SetState(E_STATE::KILL);
		return true;
	}break;
	case MotorReset:
	{
		//reserved for future use...
		SetState(E_STATE::MOTOR_RST);
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
			motorPedalsL.moveTo(factor * PEDALS_LEVEL0);
			motorPedalsR.moveTo(factor * PEDALS_LEVEL0);
		}break;
		case 1: {
			motorPedalsL.moveTo(factor * PEDALS_LEVEL0);
			motorPedalsR.moveTo(factor * PEDALS_LEVEL0);
		}break;
		case 2: {
			motorPedalsL.moveTo(factor * PEDALS_LEVEL0);
			motorPedalsR.moveTo(factor * PEDALS_LEVEL0);
		}break;
		case 3: {
			motorPedalsL.moveTo(factor * PEDALS_LEVEL0);
			motorPedalsR.moveTo(factor * PEDALS_LEVEL0);
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

	if (strcmp(input, "kill") == NULL)
	{
		cmdFound = CommandType::Kill;
	}

	if (strcmp(input, "motorReset") == NULL)
	{
		cmdFound = CommandType::MotorReset;
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
		if (commBuild(chr1, 1)) {
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
	_state = _state != newState ? newState : _state;
#if DEBUG == 2
	Serial.print("State:"); Serial.println((char)newState);
#endif //  DEBUG = 1
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
		
			Serial.print(F("==>")); Serial.println(cMsg);
		}
		else
			buttonActive[i] = false;
	}
#if DEBUG >= 1
	if (buttonActive[3] && buttonActive[0]) IsVibrationEnabled = true;
	if (buttonActive[1] && buttonActive[2]) IsVibrationEnabled = false;
#endif
}

void SendPedalState(byte pedalNo, int pedalAngle)
{
	char cMsg[20];
	sprintf(cMsg, "pedal%d|%d", pedalNo, pedalAngle);
	
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
		int angle = int(p1_counter * (1.8)); // do (-1.8) for oposit direction pedaling
		if (angle >= 361) {
			angle = 0;
			p1_counter = 0;
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
		int angle = int(p2_counter * (1.8));
		if (angle >= 361) {
			angle = 0;
			p2_counter = 0;
		}
		SendPedalState(2, angle);
		p2_prevCounter = p2_counter;
	}
}


