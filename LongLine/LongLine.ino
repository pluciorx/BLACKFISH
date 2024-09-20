#include <107-Arduino-UniqueId.h>
#include <SoftwareSerial.h>

#define RS485_TX 2
#define RS485_RX 3
#define RS485_CONTROL 4
#define SENSOR1_PIN D9
#define SENSOR2_PIN D8
#define SENSOR3_PIN D10
#define SENSOR4_PIN D11
#define RELAY1_PIN D4
#define RELAY2_PIN D5
#define RELAY3_PIN D6
#define RELAY4_PIN D7
#define LED_PIN 13 // Pin for the blinking LED

SoftwareSerial RS485Serial(RS485_RX, RS485_TX);

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

    pinMode(RELAY1_PIN,
        OUTPUT);
    pinMode(RELAY2_PIN, OUTPUT);
    pinMode(RELAY3_PIN, OUTPUT);
    pinMode(RELAY4_PIN, OUTPUT);

    pinMode(LED_PIN,
        OUTPUT);

    RS485Serial.begin(9600);
    Serial.println("");

    for (int i = 0; i < OpenCyphalUniqueId.ID_SIZE; i++) {
        uniqueID += OpenCyphalUniqueId[i];
    }

    Serial.println(uniqueID);
    selfTestRelays();
}

void loop() {
    switch (slaveState) {
    case SlaveState::IDLE:
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
        else if (RS485Serial.available()) {
            slaveState = SlaveState::RECEIVE_COMMAND;
        }
        break;

    case SlaveState::SEND_SENSOR_DATA:
        // After sending, return to IDLE state
        slaveState = SlaveState::IDLE;
        break;

    case SlaveState::RECEIVE_COMMAND:
        String command = RS485Serial.readStringUntil('\n');
        processCommand(command);
        slaveState = SlaveState::IDLE;
        break;
    }

    // Blink the LED based on the current state
    if (slaveState == SlaveState::IDLE) {
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
        digitalWrite(RELAY1_PIN + i - 1, HIGH);
        delay(200); // Delay to allow relay to activate
        digitalWrite(RELAY1_PIN + i - 1, LOW);
        delay(200); // Delay to allow relay to deactivate

        Serial.print("Relay ");
        Serial.print(i);
        Serial.println(" tested successfully.");
    }
}

void sendSensorTriggered(String sensor) {
    digitalWrite(RS485_CONTROL, HIGH); // Set to transmit mode
    RS485Serial.print(OpenCyphalUniqueId);
    RS485Serial.print(": ");
    RS485Serial.println(sensor);
    RS485Serial.flush();
    digitalWrite(RS485_CONTROL, LOW); // Back to receive mode
}

void processCommand(String cmd) {
    
    if (cmd.startsWith(uniqueID)) {
        if (cmd.endsWith("RL1 ON")) digitalWrite(RELAY1_PIN, HIGH);
        if (cmd.endsWith("RL1 OFF")) digitalWrite(RELAY1_PIN, LOW);
        if (cmd.endsWith("RL2 ON")) digitalWrite(RELAY2_PIN, HIGH);
        if (cmd.endsWith("RL2 OFF")) digitalWrite(RELAY2_PIN, LOW);
        if (cmd.endsWith("RL3 ON")) digitalWrite(RELAY3_PIN, HIGH);
        if (cmd.endsWith("RL3 OFF")) digitalWrite(RELAY3_PIN, LOW);
        if (cmd.endsWith("RL4 ON")) digitalWrite(RELAY4_PIN, HIGH);
        if (cmd.endsWith("RL4 OFF")) digitalWrite(RELAY4_PIN, LOW);
    }
}