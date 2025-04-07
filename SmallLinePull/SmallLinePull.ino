// SmallLinePush.ino
#include <Adafruit_Debounce.h>
#include <SoftwareSerial.h>

#define ADDR_PANEL '0'
#define ADDR_ROLL '1'
#define ADDR_CTRL '2'
#define ADDR_PULL '3'
#define ADDR_PUSH '4'
#define ADDR_DEREG 'F'
const char nodeAddr = ADDR_PULL;

#define PIN_BTN_FORWARD D10
Adafruit_Debounce btnForward(PIN_BTN_FORWARD, LOW);
#define PIN_BTN_BACKWARD D11
Adafruit_Debounce btnBackward(PIN_BTN_BACKWARD, LOW);
#define PIN_BTN_RELEASE D8
Adafruit_Debounce btnUp(PIN_BTN_RELEASE, LOW);
#define PIN_BTN_HOLD D9	
Adafruit_Debounce btnDown(PIN_BTN_HOLD, LOW);
//#define INPUT_PULLDOWN

#define PIN_SEN_IN  A2
bool sensorInState = false;
bool sensorInStatePrev = false;
#define PIN_SEN_OUT A3
bool sensorOutState = false;
bool sensorOutStatePrev = false;

#define PIN_SEN_DOOR1  A4
bool sensorDoor1State = false;
bool sensorDoor1StatePrev = false;
#define PIN_SEN_DOOR2  A5
bool sensorDoor2State = false;
bool sensorDoor2StatePrev = false;

#define PIN_MOTOR_SPD A0
int motorSPDValue = 0;

#define PIN_RL_FORWARD  D4
#define PIN_RL_BACKWARD  D5
#define PIN_VALVE D6

enum SlaveState { IDLE, SEND_SENSOR_DATA, RECEIVE_COMMAND, DEREG };
SlaveState slaveState = SlaveState::IDLE;

enum TapeDirection { FORWARD, BACKWARD, STOP };

unsigned long lastHostUpdate = 0;
const unsigned long healthCheckInterval = 2000UL; //3S TTL check 

bool isProductionRunning = false;
bool isValveUp = false;

void setup() {

	pinMode(PIN_RL_FORWARD, OUTPUT);
	pinMode(PIN_RL_BACKWARD, OUTPUT);

	pinMode(PIN_SEN_IN, INPUT);
	pinMode(PIN_SEN_OUT, INPUT);
	pinMode(PIN_SEN_DOOR1, INPUT);
	pinMode(PIN_SEN_DOOR2, INPUT);

	pinMode(PIN_MOTOR_SPD, OUTPUT);
	pinMode(PIN_VALVE, OUTPUT);

	btnDown.begin();
	btnUp.begin();
	btnForward.begin();
	btnBackward.begin();

	Serial1.begin(19200);
	Serial.begin(115200);
	Serial.println("");

	Serial.println();
	delay(250);

	Serial.println("Pull node setup ready");
	Serial.print("Slave node Addr: ");
	Serial.println(nodeAddr);

	setMotorSpeed(0);
	slaveState = SlaveState::DEREG;
	digitalWrite(PIN_RL_FORWARD, LOW);
	digitalWrite(PIN_RL_BACKWARD, LOW);
}

// the loop function runs over and over again until power down or reset
void loop() {
	updateButtons();
	ReadINOUTSensors();
	if (Serial1.available()) {
		slaveState = SlaveState::RECEIVE_COMMAND;
	}

	switch (slaveState) {
	case DEREG:
	{
		RegisterNode();

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
		digitalWrite(PIN_LED, HIGH);
	}
	else {
		digitalWrite(PIN_LED, LOW);
	}
}

void RegisterNode() {
	static unsigned long lastAttempt = 0;
	unsigned long now = millis();

	if (now - lastAttempt >= 500) {  // every 2 seconds
		String message = String(nodeAddr) + "REG_PULL";
		Serial1.println(message);
		Serial1.flush();
		Serial.println("=>:" + message);

		lastAttempt = now;
	}

}

void sendSensorTriggered(String sensor) {
	Serial.println("Sensor triggered: " + sensor);
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

void setMotorSpeed(int speed) {

	// Set the motor speed
	// speed should be between 0 and 255
	Serial.println("Set motor speed: " + String(speed));
	analogWrite(PIN_MOTOR_SPD, speed);
}



void ReadINOUTSensors()
{
	sensorInState = digitalRead(PIN_SEN_IN);

	sensorOutState = digitalRead(PIN_SEN_OUT);

	sensorDoor1State = digitalRead(PIN_SEN_DOOR1);

	sensorDoor2State = digitalRead(PIN_SEN_DOOR2);

	if (sensorDoor1State != sensorDoor1StatePrev) {

		sendSensorTriggered("SENSOR_DOOR1");
		sensorDoor1StatePrev = sensorDoor1State;
	}
	if (sensorDoor2State != sensorDoor2StatePrev) {

		sendSensorTriggered("SENSOR_DOOR2");
		sensorDoor2StatePrev = sensorDoor2State;
	}

	if (sensorInState != sensorInStatePrev) {
		sendSensorTriggered("SENSOR_IN");
		sensorInStatePrev = sensorInState;
	}
	if (sensorOutState != sensorOutStatePrev) {
		sendSensorTriggered("SENSOR_OUT");
		sensorOutStatePrev = sensorOutState;
	}
}

void updateButtons() {
	//if (isProductionRunning) return;
	btnForward.update();
	btnBackward.update();
	btnUp.update();
	btnDown.update();

	if (btnDown.justReleased()) {
		closeValve();
	}
	if (btnUp.justReleased()) {
		openValve();
	}
	if (btnForward.isPressed()) {
		digitalWrite(PIN_RL_FORWARD, HIGH);
		//setmMotorDir(TapeDirection::FORWARD);
		//setMotorSpeed(128);
	}
	else
	{
		digitalWrite(PIN_RL_FORWARD, LOW);

	}

	if (btnBackward.isPressed()) {
		digitalWrite(PIN_RL_BACKWARD, HIGH);
		//setMotorSpeed(128);
	}
	else
	{
		digitalWrite(PIN_RL_BACKWARD, LOW);

	}


	/*if (!btnLeft.isPressed() && !btnRigh.isPressed()) {
		setMotorSpeed(0);
	}*/
}

void setmMotorDir(TapeDirection direction) {


	if (direction == STOP) {
		digitalWrite(PIN_RL_FORWARD, LOW);
		digitalWrite(PIN_RL_BACKWARD, LOW);
	}
	else {
		digitalWrite(PIN_RL_FORWARD, direction == FORWARD ? HIGH : LOW);
		digitalWrite(PIN_RL_BACKWARD, direction == FORWARD ? LOW : HIGH);
	}
}

void openValve() {
	// Open the valve
	Serial.println("Open valve");
	digitalWrite(PIN_VALVE, HIGH);

}

void closeValve() {
	Serial.println("Close valve");
	digitalWrite(PIN_VALVE, LOW);
}

