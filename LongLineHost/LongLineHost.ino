#include <107-Arduino-UniqueId.h>
#define RS485_CONTROL 2

String uniqueID;
enum MasterState { IDLE, PROCESS_COMMAND, RECEIVE_DATA, HEALTH_CHECK };

MasterState masterState = MasterState::IDLE;

String commandBuffer;

// Array to store registered slave IDs and their health status
const int maxSlaves = 20; // Adjust the maximum number of slaves as needed

struct SlaveInfo {
    String ID;
    bool isHealthy;
    unsigned long lastCheckedTime; // Timestamp of last health check
};

SlaveInfo registeredSlaves[maxSlaves];
int numRegisteredSlaves = 0;

const unsigned long healthCheckInterval = 10000; // Health check interval in milliseconds

void setup() {
    pinMode(RS485_CONTROL, OUTPUT);
    digitalWrite(RS485_CONTROL, LOW); // Set to receive mode

    for (int i = 0; i < OpenCyphalUniqueId.ID_SIZE; i++) {
        uniqueID += OpenCyphalUniqueId[i];
    }
    
    Serial.begin(9600);
    Serial.println("");
    Serial.println("Host Node setup ");

    Serial1.begin(115200);

    // Initialize registered slaves array
    for (int i = 0; i < maxSlaves; i++) {
        registeredSlaves[i].ID = "";
        registeredSlaves[i].isHealthy = false;
        registeredSlaves[i].lastCheckedTime = 0;
    }

    Serial.println("Host Node Setup Ready");
}

void loop() {
    switch (masterState)
    {
    case IDLE: {
        if (Serial1.available()) {
            masterState = MasterState::RECEIVE_DATA;
        }
        else if (Serial.available()) {
            commandBuffer = Serial.readStringUntil(';');
            masterState = MasterState::PROCESS_COMMAND;
        }
        else {
            // Iterate through registered slaves to see if any need a health check
            for (int i = 0; i < numRegisteredSlaves; i++) {
                if (millis() - registeredSlaves[i].lastCheckedTime >= healthCheckInterval) {
                    masterState = MasterState::HEALTH_CHECK;
                    break;
                }
            }
        }
    } break;

    case PROCESS_COMMAND: {
        sendCommand(commandBuffer);
        masterState = MasterState::IDLE;
    } break;

    case RECEIVE_DATA: {
        String data = Serial1.readStringUntil(';');
        Serial1.flush();
        if (data.startsWith("REG:")) {
            String slaveID = data.substring(4);
            if (registerSlave(slaveID)) {
                Serial.println("Slave " + slaveID + " registered successfully.");
            }
            else {
                Serial.println("Slave " + slaveID + " already registered.");
            }
        }
        else if (data.startsWith("PONG:")) {
            // For a PONG message, the expected format is "PONG:slaveID"
            String slaveID = data.substring(5); // Remove "PONG:" (5 characters)
            Serial.println(slaveID + " => Pong");
            updateSlaveHealth(slaveID, true);
		}
		else if (data.startsWith("ACK:")) {
			// For a ACK message, the expected format is "ACK:slaveID:command"
			String slaveID = data.substring(4);
			Serial.println(slaveID + " => ACK");
		}

        
        masterState = MasterState::IDLE;
    } break;

    case HEALTH_CHECK: {
        // Send a ping to any slave that is overdue for a health check
        for (int i = 0; i < numRegisteredSlaves; i++) {
            if (millis() - registeredSlaves[i].lastCheckedTime >= healthCheckInterval) {
                sendCommand(registeredSlaves[i].ID + ":PING");
                registeredSlaves[i].lastCheckedTime = millis();
            }
        }
        masterState = MasterState::IDLE;
    } break;
    }
}

void sendCommand(String cmd) {
	delay(5);
    digitalWrite(RS485_CONTROL, HIGH); // Set to transmit mode
	cmd += ';';                         // ALWAYS ADD A SEMICOLON TO THE END OF THE COMMAND
    Serial1.print(cmd);    
    Serial1.flush();
    Serial.print("RS485>: ");
    Serial.println(cmd);
    digitalWrite(RS485_CONTROL, LOW); // Back to receive mode
}

bool registerSlave(String slaveID) {
    // Check if the slave is already registered
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
