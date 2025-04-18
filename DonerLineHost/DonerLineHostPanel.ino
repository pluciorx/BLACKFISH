/*"BlackFish Doner roller line steering panel
 * This program should be installed only on the control panel of the roller.
 */

#include <Adafruit_Debounce.h>
#include <LiquidCrystal_I2C.h>

#include <Wire.h>
const char* fv = "     V 2025.03.31";

LiquidCrystal_I2C lcd(0x27, 20, 4);

// Kierunek obrotów
#define BUTTON_RIGHT_ROTATE  D2  // Przycisk chwilowy obroty w prawo
Adafruit_Debounce buttonRightRotate(BUTTON_RIGHT_ROTATE, HIGH);
#define BUTTON_LEFT_ROTATE   D3  // Przycisk chwilowy obroty w lewo
Adafruit_Debounce buttonLeftRotate(BUTTON_LEFT_ROTATE, HIGH);

// Regulacja predkosci
#define POT_SPEED_CONTROL    A3  // Potencjometr regulacji obrotów
int speed = 0;
int _prevSpeed = 0;
// Tryb funkcji ciaglej
#define BUTTON_FUNCTION      A1  // Przycisk chwilowy FUNKCYJNY
Adafruit_Debounce buttonFunction(BUTTON_FUNCTION, HIGH);

// Zmiana srednicy
#define BUTTON_DIAMETER_UP   D4  // Przycisk chwilowy zmiany srednicy na plus
Adafruit_Debounce btnDiameterUp(BUTTON_DIAMETER_UP, HIGH);
#define BUTTON_DIAMETER_DOWN D5  // Przycisk chwilowy zmiany srednicy na minus
Adafruit_Debounce btnDiameterDown(BUTTON_DIAMETER_DOWN, HIGH);

#define DIAMETER_MIN 0
#define DIAMETER_MAX 100
int diameter = DIAMETER_MIN;

// Zmiana grubosci
#define BUTTON_THICKNESS_UP   D6  // Przycisk chwilowy zmiany grubosci na plus
Adafruit_Debounce btnThicknessUp(BUTTON_THICKNESS_UP, HIGH);
#define BUTTON_THICKNESS_DOWN D7  // Przycisk chwilowy zmiany grubosci na minus
Adafruit_Debounce btnThicknessDown(BUTTON_THICKNESS_DOWN, HIGH);

#define THICKNESS_MIN 0
#define THICKNESS_MAX 100
int thickness = THICKNESS_MIN;

// Blokada trzymacza rury
#define BUTTON_PIPE_LOCK     D8  // Przycisk blokady trzymacza rury
Adafruit_Debounce buttonPipeLock(BUTTON_PIPE_LOCK, HIGH);

// Procedura zsuniêcia rury
#define BUTTON_PIPE_RELEASE  D9  // Przycisk procedury zsuniêcia rury
Adafruit_Debounce buttonPipeRelease(BUTTON_PIPE_RELEASE, HIGH);

// Presety
#define BUTTON_PRESET_1      D10 // Przycisk PRESET 1
Adafruit_Debounce buttonPreset1(BUTTON_PRESET_1, HIGH);
#define BUTTON_PRESET_2      D11 // Przycisk PRESET 2
Adafruit_Debounce buttonPreset2(BUTTON_PRESET_2, HIGH);
#define BUTTON_PRESET_3      D12 // Przycisk PRESET 3
Adafruit_Debounce buttonPreset3(BUTTON_PRESET_3, HIGH);
#define BUTTON_PRESET_4      D13 // Przycisk PRESET 4
Adafruit_Debounce buttonPreset4(BUTTON_PRESET_4, HIGH);

// Wylacznik bezpieczeñstwa
#define BUTTON_STOP          A2  // Przycisk STOP
Adafruit_Debounce buttonStop(BUTTON_STOP, HIGH);

String commandBuffer;

//for doner line only two slaves are needed
const int maxSlaves = 2;

//define the suitable types of the slaves for this host.
#define ADDR_PANEL '0'
#define ADDR_ROLL '1'
#define ADDR_CTRL '2'

#define ADDR_DEREG 'F'
const char nodeAddr = ADDR_PANEL;

struct SlaveInfo {
	char ID;
	bool isHealthy;
	unsigned long lastCheckedTime; // Timestamp of last health check
	unsigned long lastOkTime; // Timestamp of last health check

	byte slaveType;
};



SlaveInfo registeredSlaves[maxSlaves];
bool isMainRegistered = false;
bool isRollRegistered = false;

int currentSlaveIndex = -1;
unsigned long pingSentTime = 0;

int numRegisteredSlaves = 0;

const int healthCheckInterval = 1500; //10s TTL check 

enum MasterState { IDLE, PROCESS_COMMAND, RECEIVE_DATA, HEALTH_CHECK };

MasterState masterState = MasterState::IDLE;


void setup() {


	//pinMode(BUTTON_RIGHT_ROTATE, INPUT_PULLUP);
	//pinMode(BUTTON_LEFT_ROTATE, INPUT_PULLUP);
	//pinMode(BUTTON_FUNCTION, INPUT_PULLUP);
	pinMode(BUTTON_DIAMETER_UP, INPUT);
	pinMode(BUTTON_DIAMETER_DOWN, INPUT);
	pinMode(BUTTON_THICKNESS_UP, INPUT);
	pinMode(BUTTON_THICKNESS_DOWN, INPUT);
	//pinMode(BUTTON_PIPE_LOCK, INPUT_PULLUP);
	//pinMode(BUTTON_PIPE_RELEASE, INPUT_PULLUP);
	//pinMode(BUTTON_PRESET_1, INPUT_PULLUP);
	//pinMode(BUTTON_PRESET_2, INPUT_PULLUP);
	//pinMode(BUTTON_PRESET_3, INPUT_PULLUP);
	//pinMode(BUTTON_PRESET_4, INPUT_PULLUP);
	//pinMode(BUTTON_STOP, INPUT_PULLUP);
	InitButtons();
	// Potencjometr jako wejœcie analogowe (nie wymaga INPUT_PULLUP)
	//pinMode(POT_SPEED_CONTROL, INPUT);

	lcd.init();
	lcd.backlight();
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("      BLACKFISH   ");
	lcd.setCursor(0, 1);
	lcd.print("    Doner Roller   ");
	lcd.setCursor(0, 2);
	lcd.print(fv);
	delay(1000);

	lcd.clear();

	lcd.setCursor(0, 0);
	lcd.print("THI:-");
	lcd.setCursor(0, 1);
	lcd.print("DIA:-");

	lcd.setCursor(0, 2);
	lcd.print("                 ");
	lcd.setCursor(0, 2);
	lcd.print("SPD:");
	lcd.setCursor(0, 3);
	lcd.print("ZW:-");
	lcd.setCursor(8, 3);
	lcd.print("CTRL:-");
	
	// Initialize registered slaves array
	for (int i = 0; i < maxSlaves; i++) {
		registeredSlaves[i].ID = ADDR_DEREG;
		registeredSlaves[i].isHealthy = false;
		registeredSlaves[i].lastCheckedTime = 0;
		registeredSlaves[i].lastOkTime = 0;
	}
	Serial1.begin(19200);
	Serial.begin(115200);
	Serial.println("");

	Serial.println("Host Node Setup Ready");
	Serial.print("Host ID: ");
	Serial.println(nodeAddr);
	Serial.println(fv); // print message at the second row
	ReadAndUpdateSpeed();
	RequestDiameterAndThickness();
}

void loop() {
	CheckSlaves();
	checkAndDeregisterSlaves();
	updateButtons();
	HandleButtonPresses();
	switch (masterState)
	{
	case IDLE: {

		if (Serial1.available()) {
			masterState = MasterState::RECEIVE_DATA;

		}

		if (Serial.available()) {
			commandBuffer = Serial.readStringUntil('\n');
			masterState = MasterState::PROCESS_COMMAND;
		}

		//handle the inputs 
		if (isMainRegistered) ReadAndUpdateSpeed();
		HandleButtonPresses();

	} break;

	case PROCESS_COMMAND: {

		sendCommand(ADDR_ROLL, commandBuffer);
		masterState = MasterState::IDLE;
	}break;

	case RECEIVE_DATA: {
		String data = Serial1.readStringUntil('\n');

		if (data.length() < 2) {
			Serial.println("Invalid data received: " + data);
			masterState = MasterState::IDLE;
			break;
		}

		char slaveID = data.charAt(0);
		String message = data.substring(1);

		/*Serial.println("Received data: " + data);
		Serial.println("Slave ID: " + slaveID);
		Serial.println("Message: " + message);*/

		if (message.startsWith("REG_ROLL")) {
			if (registerSlave(slaveID, ADDR_ROLL)) {
				Serial.println("Slave registered successfully ROLL node: " + String(slaveID));
				RequestDiameterAndThickness();
			}
			else {
				Serial.println("Can't register node: " + String(slaveID));
			}
			isRollRegistered = true;
			lcd.setCursor(0, 3);
			lcd.print("ZW:OK");
		}
		else if (message.startsWith("REG_CTRL")) {
			if (registerSlave(slaveID, ADDR_CTRL)) {
				Serial.println("Slave registered successfully REG_CTRL node: " + String(slaveID));
			}
			else {
				Serial.println("Can't register node: " + String(slaveID));
			}
			isMainRegistered = true;
			lcd.setCursor(8, 3);
			lcd.print("CTRL:OK");
		}
		else if (message.startsWith("PONG")) {
			//Serial.println("=> Pong received");
			updateSlaveHealth(slaveID, true);
		}
		else if (message.startsWith("AREAD")) {
			//Serial.println("=> AREAD received");
			updateSlaveHealth(slaveID, true);

		}
		else {
			Serial.println("Unknown message received: " + message);
		}

		masterState = MasterState::IDLE;
	} break;

	}
}

void InitButtons()
{
	buttonRightRotate.begin();
	buttonLeftRotate.begin();
	buttonFunction.begin();
	btnDiameterUp.begin();
	btnDiameterDown.begin();
	btnThicknessUp.begin();
	btnThicknessDown.begin();
	buttonPipeLock.begin();
	buttonPipeRelease.begin();
	buttonPreset1.begin();
	buttonPreset2.begin();
	buttonPreset3.begin();
	buttonPreset4.begin();
	buttonStop.begin();
}
void HandleButtonPresses()
{
	if (btnDiameterDown.isPressed()) {
		Serial.print("Diameter down");
		DecreaseDiameter();
	}
	if (btnDiameterUp.isPressed()) {
		Serial.print("Diameter up");
		IncreaseDiameter();
	}
	if(btnThicknessDown.isPressed())
	{
		Serial.print("Thickness down");
		DecreaseThickness();
	}
	if (btnThicknessUp.isPressed())
	{
		Serial.print("Thickness up");
		IncreaseThickness();
	}
}

void CheckSlaves() {
	for (int i = 0; i < maxSlaves; i++) {
		if (millis() - registeredSlaves[i].lastCheckedTime >= healthCheckInterval) {
			if (registeredSlaves[i].ID != ADDR_DEREG) {
				SendPing(registeredSlaves[i].ID);
				registeredSlaves[i].lastCheckedTime = millis();
			}
		}
		updateSlaveScreen(i);
	}
}

void sendCommand(char slave_id, String cmd) {
	String message = String(slave_id) + cmd;
	Serial1.println(message);
	Serial1.flush();
	Serial.println("=>" + message);
}


bool deregisterSlave(char slaveID) {
	for (int i = 0; i < maxSlaves; i++) {
		if (registeredSlaves[i].ID == slaveID) {
			registeredSlaves[i].ID = ADDR_DEREG;
			registeredSlaves[i].isHealthy = false;
			registeredSlaves[i].lastCheckedTime = 0;
			registeredSlaves[i].lastOkTime = 0;
			numRegisteredSlaves--;
			return true;
		}
	}
	return false;
}

bool registerSlave(char slaveID, byte type) {
	// Check if the slave is already registered
	for (int i = 0; i < maxSlaves; i++) {
		if (registeredSlaves[i].ID == slaveID) {
			Serial.println("Slave " + String(slaveID) + " is already registered.");
			updateSlaveHealth(slaveID, true);
			RequestDiameterAndThickness();
			return true; // Slave already registered
		}
	}
	// Find an empty slot in the array
	for (int i = 0; i < maxSlaves; i++) {
		if (registeredSlaves[i].ID == ADDR_DEREG) {
			registeredSlaves[i].ID = slaveID;
			registeredSlaves[i].isHealthy = true;
			registeredSlaves[i].lastCheckedTime = millis();
			registeredSlaves[i].lastOkTime = millis();
			registeredSlaves[i].slaveType = type;

			numRegisteredSlaves++;
			Serial.println("Slave " + String(slaveID) + " registered successfully.");
			SendPing(slaveID);
			RequestDiameterAndThickness();
			return true; // Slave registered successfully

		}
	}
	Serial.println("No available slots for new slave.");
	return false; // No available slots
}



void SendPing(char slaveID)
{
	sendCommand(slaveID, "PING");
}


void updateSlaveScreen(int slaveIndex) {
	if (millis() - registeredSlaves[slaveIndex].lastOkTime > healthCheckInterval * 2) {

		switch (registeredSlaves[slaveIndex].slaveType) {
		case ADDR_CTRL:
			isMainRegistered = false;
			lcd.setCursor(8, 3);
			lcd.print("CTRL:- ");
			break;
		case ADDR_ROLL:
			isRollRegistered = false;
			lcd.setCursor(0, 3);
			lcd.print("ZW:- ");
			break;
		}
	}
	else {
		switch (registeredSlaves[slaveIndex].slaveType) {
		case ADDR_CTRL:
			isMainRegistered = true;
			lcd.setCursor(8, 3);
			lcd.print("CTRL:OK");
			break;
		case ADDR_ROLL:
			isRollRegistered = true;
			lcd.setCursor(0, 3);
			lcd.print("ZW:OK");
			break;
		}
	}
}


void checkAndDeregisterSlaves() {
	for (int i = 0; i < maxSlaves; i++) {
		if (registeredSlaves[i].ID != ADDR_DEREG &&
			millis() - registeredSlaves[i].lastOkTime > healthCheckInterval * 2) {
			Serial.println("Deregistering slave " + String(registeredSlaves[i].ID));
			deregisterSlave(registeredSlaves[i].ID);

		}
	}
}
void updateSlaveHealth(char slaveID, bool isHealthy) {
	for (int i = 0; i < maxSlaves; i++) {
		if (registeredSlaves[i].ID == slaveID) {
			registeredSlaves[i].isHealthy = isHealthy;
			registeredSlaves[i].lastCheckedTime = millis();
			if (isHealthy) registeredSlaves[i].lastOkTime = millis();
			break;
		}
	}
}


static void RequestAnalogRead(char slaveID, int analogPin) {
	sendCommand(slaveID, "AREAD_"+ String(analogPin));

}

void updateButtons()
{
	buttonRightRotate.update();
	buttonLeftRotate.update();
	buttonFunction.update();
	btnDiameterUp.update();
	btnDiameterDown.update();
	btnThicknessUp.update();
	btnThicknessDown.update();
	buttonPipeLock.update();
	buttonPipeRelease.update();
	buttonPreset1.update();
	buttonPreset2.update();
	buttonPreset3.update();
	buttonPreset4.update();
	buttonStop.update();
}

static void IncreaseDiameter() {
	if (!isRollRegistered) return;
	diameter++;
	updateDiameter(diameter);
	sendCommand(ADDR_ROLL, "SETDIA:" + String(diameter));
	RequestDiameterAndThickness();
}

static void DecreaseDiameter() {
	if (!isRollRegistered) return;
	diameter--;
	updateDiameter(diameter);
	sendCommand(ADDR_ROLL, "SETDIA:" + String(diameter));
	RequestDiameterAndThickness();
}


static void IncreaseThickness() {
	if (!isRollRegistered) return;
	thickness++;
	updateThickness(thickness);
	sendCommand(ADDR_ROLL, "SETTHI:" + String(thickness));
	RequestDiameterAndThickness();
}

static void DecreaseThickness() {
	if (!isRollRegistered) return;
	thickness--;
	updateThickness(thickness);
	sendCommand(ADDR_ROLL, "SETTHI:" + String(thickness));
	RequestDiameterAndThickness();
}


static void RequestDiameterAndThickness() {
	if (!isRollRegistered) return;
	Serial.println("Requesting diameter and thickness from slave...");
	RequestAnalogRead(ADDR_ROLL, 0); // Diameter
	RequestAnalogRead(ADDR_ROLL, 1); // Thickness
}


void updateDiameter(int value) {
	
	lcd.setCursor(0, 1);
	lcd.print("        ");
	lcd.setCursor(0, 1);
	lcd.print("DIA:" + String(diameter));
}
void updateThickness(int value) {
	
	lcd.setCursor(0, 0);
	lcd.print("        ");
	lcd.setCursor(0, 0);
	lcd.print("THI:" + String(thickness));
}


static int ReadAndUpdateSpeed() {
	
	speed = constrain(1024 - analogRead(POT_SPEED_CONTROL),0,1023);
	if (speed != _prevSpeed && abs(_prevSpeed - speed) >= 32) {
		_prevSpeed = speed;

		String message = "SETRSPD:" + String(speed);
		sendCommand(ADDR_ROLL, message);

		lcd.setCursor(4, 2);
		lcd.print(map(speed, 0, 1024, 0, 100));
		lcd.print("% ");

		return speed;

	}
	return _prevSpeed;
}