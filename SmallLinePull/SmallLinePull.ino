// SmallLinePush.ino
#include <Adafruit_Debounce.h>
#include <SoftwareSerial.h>

#define ADDR_PANEL '0'
#define ADDR_ROLL '1'
#define ADDR_CTRL '2'
#define ADDR_PUSH '3'
#define ADDR_PULL '4'

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
const unsigned long healthCheckInterval = 3000UL; //3S TTL check 

bool isProdReadyState = false;
bool isProdReadyStatePrev = false;
bool isValveUp = false;
bool isEngineRotating = false;

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

	Serial1.begin(14400);
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
	UpdateReadyState();
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
			
			Serial.println("PULL going DEREG.");
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
		String message = "REG_PULL";
		sendCommand(nodeAddr, message);	
	
		lastAttempt = now;
	}

}

void processEngineCommand(String cmd)
{
	
	int colonIndex = cmd.indexOf(':');
	if (colonIndex == -1) {
		Serial.println("Invalid ENG command format.");
		return;
	}

	String engineDir = cmd.substring(colonIndex + 1);
	
	if (engineDir.startsWith("F"))
	{
		engineMoveForward();
	}
	if (engineDir.startsWith("B"))
	{
		engineMoveBackward();
	}
	if (engineDir.startsWith("S"))
	{
		engineStop();
	}
}

void sendSensorState(String sensor,bool state) {	
	String message = sensor + ":" + String(state);
	sendCommand(nodeAddr, message);
	
}


void processCommand(String cmd) {
	cmd.trim();
	char slaveID = cmd.charAt(cmd.lastIndexOf(':') + 1);
	if (slaveID != nodeAddr) {

		return;
	}
	lastHostUpdate = millis();
	delay(50);
	String message = cmd.substring(0, cmd.lastIndexOf(':'));

	if (message.startsWith("PING")) {
		processPingCommand();
		return;
	}

	if (message.startsWith("ISRREQ")) {

		SendReadyStateToHost();
		return;
	}

	if (message.startsWith("ENG")) {
		// Send the readiness state to the host
		processEngineCommand(message);
		return;
	}

	if (message.startsWith("ESPD")) {
		processRSPEED(message);
		return;
	}

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

void UpdateReadyState()
{

	sensorInState = digitalRead(PIN_SEN_IN);

	sensorOutState = digitalRead(PIN_SEN_OUT);

	sensorDoor1State = digitalRead(PIN_SEN_DOOR1);

	sensorDoor2State = digitalRead(PIN_SEN_DOOR2);
	//we are responding to the host request if the module is ready to be operated.
	if (sensorDoor1State == LOW && sensorDoor2State == LOW && sensorInState == LOW && sensorOutState ==LOW ) {
		//both doors are closed
		if (isProdReadyState != isProdReadyStatePrev) Serial.println("Module Ready");
		isProdReadyState = true;

	}
	else {

		//engineStop();
		if (isProdReadyState != isProdReadyStatePrev) {
			SendHold();
		}

	}

	isProdReadyStatePrev = isProdReadyState;
}

void SendHold()
{
	sendCommand(nodeAddr, "HOLD");
	sendCommand(ADDR_PUSH, "ENG:S");
}

void SendReadyStateToHost()
{
	if (isProdReadyState) {

		String message = "ISRREP:1";
		sendCommand(nodeAddr, message);
	}
	else {
		String message = "ISRREP:0";
		sendCommand(nodeAddr, message);
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

	if (!isEngineRotating) {
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
	}


	/*if (!btnLeft.isPressed() && !btnRigh.isPressed()) {
		setMotorSpeed(0);
	}*/
}

//void setmMotorDir(TapeDirection direction) {
//
//
//	if (direction == STOP) {
//		digitalWrite(PIN_RL_FORWARD, LOW);
//		digitalWrite(PIN_RL_BACKWARD, LOW);
//	}
//	else {
//		digitalWrite(PIN_RL_FORWARD, direction == FORWARD ? HIGH : LOW);
//		digitalWrite(PIN_RL_BACKWARD, direction == FORWARD ? LOW : HIGH);
//	}
//}

void openValve() {
	// Open the valve
	Serial.println("Open valve");
	digitalWrite(PIN_VALVE, HIGH);

}

void closeValve() {
	Serial.println("Close valve");
	digitalWrite(PIN_VALVE, LOW);
}

