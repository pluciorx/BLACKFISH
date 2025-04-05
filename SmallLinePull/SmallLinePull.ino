// SmallLinePull.ino
#include <SoftwareSerial.h>
#include <Adafruit_Debounce.h>
#define ADDR_PANEL '0'
#define ADDR_ROLL '1'
#define ADDR_CTRL '2'
#define ADDR_PULL '3'
#define ADDR_PUSH '4'
#define ADDR_DEREG 'F'
const char nodeAddr = ADDR_PULL;

#define PIN_BTN_LEFT D10
Adafruit_Debounce btnLeft(PIN_BTN_LEFT, HIGH);
#define PIN_BTN_RIGHT D11
Adafruit_Debounce btnRigh(PIN_BTN_RIGHT, HIGH);
#define PIN_BTN_UP D8
Adafruit_Debounce btnUp(PIN_BTN_UP, HIGH);
#define PIN_BTN_DOWN D9	
Adafruit_Debounce btnDown(PIN_BTN_DOWN, HIGH);
#define INPUT_PULLDOWN

#define PIN_SENSOR_IN  D3
#define PIN_SENSOR_OUT D4

#define PIN_MOTOR_SPD A0
int motorSPDValue = 0;

#define PIN_RL1  D4
#define PIN_RL2  D5
#define PIN_VALVE D7


enum SlaveState { IDLE, SEND_SENSOR_DATA, RECEIVE_COMMAND, DEREG };
SlaveState slaveState = SlaveState::IDLE;

unsigned long lastHostUpdate = 0;
const unsigned long healthCheckInterval = 2000UL; //3S TTL check 

bool isProductionRunning = false;
// the setup function runs once when you press reset or power the board
void setup() {

	pinMode(PIN_RL1, OUTPUT);
	pinMode(PIN_RL2, OUTPUT);
	pinMode(PIN_SENSOR_IN, INPUT);
	pinMode(PIN_SENSOR_OUT, INPUT);

	pinMode(PIN_MOTOR_SPD, OUTPUT);
	pinMode(PIN_VALVE, OUTPUT);

	btnDown.begin();
	btnUp.begin();
	btnLeft.begin();
	btnRigh.begin();

	Serial1.begin(19200);
	Serial.begin(115200);
	Serial.println("");

	Serial.println();
	delay(250);

	Serial.println("Puller Node Setup Ready");
	Serial.print("Slave node Addr: ");
	Serial.println(nodeAddr);


	slaveState = SlaveState::DEREG;
}

// the loop function runs over and over again until power down or reset
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
		digitalWrite(PIN_LED, HIGH);
	}
	else {
		digitalWrite(PIN_LED, LOW);
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


void updateButtons() {
	if (isProductionRunning) return;
	btnLeft.update();
	btnRigh.update();
	btnUp.update();
	btnDown.update();

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