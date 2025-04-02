/* 
* This program should be installed only on the rotary elements of the roller.
*/

#include <FlexyStepper.h>
#include <SoftwareSerial.h>

#define DIAMETER_MIN 0
#define DIAMETER_MAX 100
int diameter = DIAMETER_MIN;

#define THICKNESS_MIN 0
#define THICKNESS_MAX 100
int thickness = THICKNESS_MIN;

#define ACCEL 500
#define MAX_SPD 50

//#define RS485_CONTROL D2
FlexyStepper motorThickness;
#define PIN_POT_THICKNESS A2
#define STEP_THICKNESS D2
#define DIR_THICKNESS D3

FlexyStepper motorDiameter;
#define PIN_POT_DIAMETER A3
#define STEP_DIAMETER D4
#define DIR_DIAMETER D5


#define LED_PIN 13 // Pin for the blinking LED

const char* fv = "     V 2025.03.29";

#define ADDR_PANEL '0'
#define ADDR_ROLL '1'
#define ADDR_CTRL '2'

#define ADDR_DEREG 'F'

const char nodeAddr = ADDR_ROLL;

enum SlaveState { IDLE, SEND_SENSOR_DATA, RECEIVE_COMMAND, DEREG };
SlaveState slaveState = SlaveState::IDLE;

unsigned long lastHostUpdate = 0;
const unsigned long healthCheckInterval = 2000UL; //3S TTL check 

void setup() {

	pinMode(PIN_POT_DIAMETER, INPUT);
	pinMode(PIN_POT_THICKNESS, INPUT);
	
	motorDiameter.connectToPins(STEP_DIAMETER, DIR_DIAMETER);
	motorThickness.connectToPins(STEP_THICKNESS, DIR_THICKNESS);	

	pinMode(LED_PIN, OUTPUT);

	Serial1.begin(14400);
	Serial.begin(115200);
	Serial.println("");

	Serial.println();
	delay(250);
	
	Serial.println("Slave Roller Node Setup Ready");
	Serial.print("Slave node Addr: ");
	Serial.println(nodeAddr);

	
	slaveState = SlaveState::DEREG;
	TestMotors();
	while (1);
}

void TestMotors()
{
	motorDiameter.setCurrentPositionInSteps(0);
	motorDiameter.setSpeedInStepsPerSecond(MAX_SPD);
	motorDiameter.setAccelerationInStepsPerSecondPerSecond(ACCEL);

	motorThickness.setCurrentPositionInSteps(0);
	motorThickness.setSpeedInStepsPerSecond(MAX_SPD);
	motorThickness.setAccelerationInStepsPerSecondPerSecond(ACCEL);

	motorDiameter.moveToPositionInSteps(100);
	motorThickness.moveToPositionInSteps(100);

	while (!motorDiameter.motionComplete() && !motorThickness.motionComplete())
	{
		motorDiameter.processMovement();
		motorThickness.processMovement();
	}
	motorDiameter.moveToPositionInSteps(0);
	motorThickness.moveToPositionInSteps(0);

	while (!motorDiameter.motionComplete() && !motorThickness.motionComplete())
	{
		motorDiameter.processMovement();
		motorThickness.processMovement();
	}

}


void loop() {
	if (Serial1.available()) {
		slaveState = SlaveState::RECEIVE_COMMAND;
	}

	switch (slaveState) {
	case DEREG:
	{
		RegisterRollerNode();

	}break;
	case IDLE: {
		if (millis() - lastHostUpdate > healthCheckInterval) {
			Serial.println("No communication with host for 3 seconds. Going to DEREG.");
			slaveState = SlaveState::DEREG;

		}

	}break;

	case SEND_SENSOR_DATA:
		// After sending, return to IDLE state
		slaveState = SlaveState::IDLE;
		break;

	case RECEIVE_COMMAND: {
		String command = Serial1.readStringUntil('\n');
		Serial1.flush();
		processCommand(command);

		slaveState = SlaveState::IDLE;
		break;
	}
	}

	// Blink the LED: HIGH when idle, LOW otherwise
	if (slaveState == IDLE) {
		digitalWrite(LED_PIN, HIGH);
	}
	else {
		digitalWrite(LED_PIN, LOW);
	}
}

void RegisterRollerNode() {
	static unsigned long lastAttempt = 0;
	unsigned long now = millis();

	if (now - lastAttempt >= 500) {  // every 2 seconds
		String message = String(nodeAddr) + "REG_ROLL";
		Serial1.println(message);
		Serial1.flush();
		Serial.println("=>:" + message);
		
		lastAttempt = now;
	}
	
}

void sendSensorTriggered(String sensor) {
	String message = String(nodeAddr) + sensor;
	Serial1.println(message);
	Serial1.flush();
}


void processCommand(String cmd) {
	cmd.trim();
	if (cmd.length() == 0 || cmd.charAt(0) != nodeAddr) {
		
		Serial.print("Foreign command:");
		Serial.println(cmd);
		return;
	}
	lastHostUpdate = millis();
	String message = cmd.substring(1);
	
	if (message.startsWith("PING")) {
		processPingCommand();
		return;
	}

	//if (message.startsWith("RL")) {
	//	//Relay command
	//	return;
	//}
	//if (message.startsWith("SETRSPD")) {
	//	processRSPEED(message);
	//	return;
	//}
	
	if (processAnalogReadCommand(cmd)) return;
	

	Serial.print("Unrecognized command for this node:");
	Serial.println(cmd);
}



