#include <SoftwareSerial.h>

#define RS485_TX 2
#define RS485_RX 3
#define RS485_CONTROL 4

SoftwareSerial RS485Serial(RS485_RX, RS485_TX);

enum MasterState { IDLE, PROCESS_COMMAND, RECEIVE_DATA, HEALTH_CHECK };

MasterState masterState = MasterState::IDLE;

String commandBuffer;

// Array to store registered slave IDs and their health status
const int maxSlaves = 10; // Adjust the maximum number of slaves as needed

struct SlaveInfo {
	String ID;
	bool isHealthy;
	unsigned long lastCheckedTime; // Timestamp of last health check
};

SlaveInfo registeredSlaves[maxSlaves];
int numRegisteredSlaves = 0;

const unsigned long healthCheckInterval = 60000; // Health check interval in milliseconds
int slaveId;

void setup() {
	pinMode(RS485_CONTROL, OUTPUT);
	digitalWrite(RS485_CONTROL, LOW); // Set to receive mode

	RS485Serial.begin(9600);

	// Initialize registered slaves array
	for (int i = 0; i < maxSlaves; i++) {
		registeredSlaves[i].ID = "";
		registeredSlaves[i].isHealthy = false;
		registeredSlaves[i].lastCheckedTime = 0;
	}
}

void loop() {
	switch (masterState)
	{
	case MasterState::IDLE:
		if (RS485Serial.available()) {
			masterState = MasterState::RECEIVE_DATA;
		}
		else if (Serial.available()) {
			commandBuffer = Serial.readStringUntil('\n');
			masterState = MasterState::PROCESS_COMMAND;
		}
		else
		{
			if (millis() - registeredSlaves[slaveId].lastCheckedTime >= healthCheckInterval) {
				masterState = MasterState::HEALTH_CHECK;
			}
			break;
		}

	case MasterState::PROCESS_COMMAND:
		sendCommand(commandBuffer);
		masterState = MasterState::IDLE;
		break;

	case MasterState::RECEIVE_DATA:
		String data = RS485Serial.readStringUntil('\n');
		if (data.startsWith("REG:")) {
			String slaveID = data.substring(4);
			if (registerSlave(slaveID)) {
				Serial.println("Slave " + slaveID + " registered successfully.");
			}
			else {
				Serial.println("Slave " + slaveID + " already registered.");
			}
		}
		else if (data.startsWith("HEALTH:")) {
			String slaveID = data.substring(6);
			bool isHealthy = data.substring(7) == "OK";
			updateSlaveHealth(slaveID, isHealthy);
		}
		masterState = MasterState::IDLE;
		break;

	case MasterState::HEALTH_CHECK:
		for (int i = 0; i < numRegisteredSlaves; i++) {
			if (millis() - registeredSlaves[i].lastCheckedTime >= healthCheckInterval) {
				sendCommand("PING:" + registeredSlaves[i].ID);
				registeredSlaves[i].lastCheckedTime = millis();
			}
		}
		masterState = MasterState::IDLE;
		break;
		}
	}
	// ... (rest of your loop code)
}

void sendCommand(String cmd) {
	digitalWrite(RS485_CONTROL, HIGH); // Set to transmit mode
	RS485Serial.println(cmd);
	RS485Serial.flush();
	digitalWrite(RS485_CONTROL, LOW); // Back to receive mode
}

bool registerSlave(String slaveID) {
	for (int i = 0; i < maxSlaves; i++) {
		if (registeredSlaves[i].ID == slaveID) {
			return false; // Slave already registered
		}
	}

	// Find an empty slot in the array
	for (int i = 0; i < maxSlaves; i++) {
		if (registeredSlaves[i].ID == "") {
			registeredSlaves[i].ID = slaveID;
			registeredSlaves[i].isHealthy = true; // Assume healthy when initially registered
			registeredSlaves[i].lastCheckedTime = millis();
			numRegisteredSlaves++;
			return true; // Slave registered successfully
		}
	}

	return false; // No available slots
}

void updateSlaveHealth(String slaveID, bool isHealthy) {
	for (int i = 0; i < maxSlaves; i++) {
		if (registeredSlaves[i].ID == slaveID) {
			registeredSlaves[i].isHealthy = isHealthy;
			break;
		}
	}
}