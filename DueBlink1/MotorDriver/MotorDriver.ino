#include <AccelStepper.h>

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
#define CHAIR1_LEFT_UPPER_LIMIT PIN_A0
#define CHAIR1_LEFT_STEP 2
#define CHAIR1_LEFT_DIR 3
#define CHAIR1_LEFT_EN 4
volatile bool IsC1LHomed = false;
bool C1LSubVibReady = true;

#define CHAIR1_RIGHT_UPPER_LIMIT PIN_A1
#define CHAIR1_RIGHT_STEP 5
#define CHAIR1_RIGHT_DIR 6
#define CHAIR1_RIGHT_EN 7
volatile bool IsC1RHomed = false;
bool C1RSubVibReady = true;
//Chair 2 
#define CHAIR2_RIGHT_UPPER_LIMIT PIN_A2
#define CHAIR2_RIGHT_STEP 8
#define CHAIR2_RIGHT_DIR 9
#define CHAIR2_RIGHT_EN PIN_A5
volatile bool IsC2RHomed = false;
bool C2LSubVibReady = true;

#define CHAIR2_LEFT_UPPER_LIMIT PIN_A3
#define CHAIR2_LEFT_STEP 10
#define CHAIR2_LEFT_DIR 11
#define CHAIR2_LEFT_EN PIN_A4

volatile bool IsC2LHomed = false;
bool C2RSubVibReady = true;

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

bool IsVibrationEnabled = false;

AccelStepper motorC1L(AccelStepper::DRIVER, CHAIR1_LEFT_STEP, CHAIR1_LEFT_DIR);
AccelStepper motorC1R(AccelStepper::DRIVER, CHAIR1_RIGHT_STEP, CHAIR1_RIGHT_DIR);
AccelStepper motorC2L(AccelStepper::DRIVER, CHAIR2_LEFT_STEP, CHAIR2_LEFT_DIR);
AccelStepper motorC2R(AccelStepper::DRIVER, CHAIR2_RIGHT_STEP, CHAIR2_RIGHT_DIR);

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


void setup() {
	Serial.begin(115200);

	Serial.println("<================= Starting =================>");

	pinMode(CHAIR1_LEFT_UPPER_LIMIT, INPUT);
	pinMode(CHAIR1_RIGHT_UPPER_LIMIT, INPUT);

	pinMode(CHAIR2_LEFT_UPPER_LIMIT, INPUT);
	pinMode(CHAIR2_RIGHT_UPPER_LIMIT, INPUT);

	for (int i = 0; i < 16; i++) {
		pinMode(motorControlPins[i], OUTPUT);
		digitalWrite(motorControlPins[i], LOW);
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

}

volatile E_STATE _state = E_STATE::SETUP;
volatile E_STATE _prevState = E_STATE::SETUP;
// the loop function runs over and over again forever
void loop() {
	switch (_state)
	{
	case HOMING: {
		Serial.println("Homing chairs started...");

		if (!HandleChairsHoming()) {
			Serial.println("Chairs homing Failed.");
		}

		Serial.println("Homing chairs Finished...");

		SetState(E_STATE::LISTENING);

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

bool HandleEndStopHitMotor(AccelStepper& stepper, float desiredSpeed, long retraction)  //Sub interrupt call
{
	stepper.stop();
	stepper.setCurrentPosition(0); // endstop is at position 0 
	stepper.moveTo(retraction);
	stepper.setSpeed(desiredSpeed);
	Serial.print("Retracting to:"); Serial.println(retraction);

	return true;
}

void SetState(E_STATE newState)
{
	_state = _state != newState ? newState : _state;
#if DEBUG == 2
	Serial.print("State:"); Serial.println((char)newState);
#endif //  DEBUG = 1
}