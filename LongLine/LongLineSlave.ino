#include <107-Arduino-UniqueId.h>
#include <SoftwareSerial.h>

#define RS485_CONTROL 2
#define SENSOR1_PIN D9
#define SENSOR2_PIN D8
#define SENSOR3_PIN D10
#define SENSOR4_PIN D11
#define RELAY1_PIN D4
#define RELAY2_PIN D5
#define RELAY3_PIN D6
#define RELAY4_PIN D7
#define LED_PIN 13 // Pin for the blinking LED

String uniqueID;

enum SlaveState { IDLE, SEND_SENSOR_DATA, RECEIVE_COMMAND };
SlaveState slaveState = SlaveState::IDLE;

void setup() {
    pinMode(RS485_CONTROL, OUTPUT);
    digitalWrite(RS485_CONTROL, LOW); // Set to receive mode

    pinMode(SENSOR1_PIN, INPUT_PULLUP);
    pinMode(SENSOR2_PIN, INPUT_PULLUP);
    pinMode(SENSOR3_PIN, INPUT_PULLUP);
    pinMode(SENSOR4_PIN, INPUT_PULLUP);

    pinMode(RELAY1_PIN, OUTPUT);
    pinMode(RELAY2_PIN, OUTPUT);
    pinMode(RELAY3_PIN, OUTPUT);
    pinMode(RELAY4_PIN, OUTPUT);

    pinMode(LED_PIN, OUTPUT);

    Serial1.begin(115200);
    Serial.begin(9600);
    Serial.println("");

    // Build the uniqueID from OpenCyphalUniqueId
    for (int i = 0; i < OpenCyphalUniqueId.ID_SIZE; i++) {
        uniqueID += OpenCyphalUniqueId[i];
    }

    Serial.println("Unique ID: " + uniqueID);
    Serial.println("Slave Node Setup Ready");
    RegisterNode();
    selfTestRelays();
    slaveState = SlaveState::IDLE;
}

void loop() {
    switch (slaveState) {
    case IDLE:
        if (digitalRead(SENSOR1_PIN) == LOW) {
            slaveState = SlaveState::SEND_SENSOR_DATA;
            sendSensorTriggered("SENSOR1");
        }
        else if (digitalRead(SENSOR2_PIN) == LOW) {
            slaveState = SlaveState::SEND_SENSOR_DATA;
            sendSensorTriggered("SENSOR2");
        }
        else if (digitalRead(SENSOR3_PIN) == LOW) {
            slaveState = SlaveState::SEND_SENSOR_DATA;
            sendSensorTriggered("SENSOR3");
        }
        else if (digitalRead(SENSOR4_PIN) == LOW) {
            slaveState = SlaveState::SEND_SENSOR_DATA;
            sendSensorTriggered("SENSOR4");
        }
        else if (Serial1.available()) {
            slaveState = SlaveState::RECEIVE_COMMAND;
        }
        break;

    case SEND_SENSOR_DATA:
        // After sending, return to IDLE state
        slaveState = SlaveState::IDLE;
        break;

    case RECEIVE_COMMAND: {
        String command = Serial1.readStringUntil(';');
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

void selfTestRelays() {
    Serial.println("Performing relay self-test...");
    // Test each relay individually
    for (int i = 1; i <= 4; i++) {
        int relayPin = RELAY1_PIN + i - 1;
        digitalWrite(relayPin, HIGH);
        delay(100); // Allow relay to activate
        digitalWrite(relayPin, LOW);
        delay(100); // Allow relay to deactivate

        Serial.print("Relay ");
        Serial.print(i);
        Serial.println(" tested successfully.");
    }
}

void RegisterNode() {
    digitalWrite(RS485_CONTROL, HIGH); // Set to transmit mode
    Serial1.print("REG:" + uniqueID);
	Serial1.println(';');
    Serial1.flush();
    digitalWrite(RS485_CONTROL, LOW); // Back to receive mode
}

void sendSensorTriggered(String sensor) {
    digitalWrite(RS485_CONTROL, HIGH); // Set to transmit mode
    // Send the unique ID and sensor info
    Serial1.print(uniqueID);
    Serial1.print(": ");
    Serial1.print(sensor);
	Serial1.println(';');
    Serial1.flush();
    digitalWrite(RS485_CONTROL, LOW); // Back to receive mode
}

void sendPong() {
    Serial.println("PONG>:" + uniqueID);
    digitalWrite(RS485_CONTROL, HIGH); // Set to transmit mode
    Serial1.print("PONG:" + uniqueID);
    Serial1.println(';');
    Serial1.flush();
    digitalWrite(RS485_CONTROL, LOW); // Back to receive mode
}

void ackCommand(String cmd) {
    Serial.println("ACK>:" + uniqueID);
	digitalWrite(RS485_CONTROL, HIGH); // Set to transmit mode
	Serial1.print("ACK:" + uniqueID);
    Serial1.print(cmd);
	Serial1.println(';');
	Serial1.flush();
	digitalWrite(RS485_CONTROL, LOW); // Back to receive mode
}

void processCommand(String cmd) {
    delay(5);
    // Process commands addressed to this slave (relay controls)
    if (cmd.startsWith(uniqueID)) {
        if (cmd.endsWith(":PING")) {
            sendPong();
            return;
		}
		else
        if (cmd.endsWith(":RL1 ON")) {
			
            digitalWrite(RELAY1_PIN, HIGH);
            ackCommand(cmd);
        }
        else if (cmd.endsWith(":RL1 OFF")) {
            digitalWrite(RELAY1_PIN, LOW);
            ackCommand(cmd);
        }
        else if (cmd.endsWith(":RL2 ON")) {
            digitalWrite(RELAY2_PIN, HIGH);
            ackCommand(cmd);
        }
        else if (cmd.endsWith(":RL2 OFF")) {
            digitalWrite(RELAY2_PIN, LOW);
            ackCommand(cmd);
        }
        else if (cmd.endsWith(":RL3 ON")) {
            digitalWrite(RELAY3_PIN, HIGH);
            ackCommand(cmd);
        }
        else if (cmd.endsWith(":RL3 OFF")) {
            digitalWrite(RELAY3_PIN, LOW);
            ackCommand(cmd);
        }
        else if (cmd.endsWith(":RL4 ON")) {
            digitalWrite(RELAY4_PIN, HIGH);
            ackCommand(cmd);
        }
        else if (cmd.endsWith(":RL4 OFF")) {
            digitalWrite(RELAY4_PIN, LOW);
            ackCommand(cmd);
        }
        else {
            Serial.print("Unrecognized command for this node:(");
			Serial.print(cmd);
            Serial.println(")");
        }
    }
    else {
        Serial.print("Invalid re(");
        Serial.print(cmd);
        Serial.println(")");
    }
}
