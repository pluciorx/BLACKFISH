#include <107-Arduino-UniqueId.h>
#include <SoftwareSerial.h>

//#define RS485_CONTROL D2
#define SENSOR1_PIN D9
#define SENSOR2_PIN D8
#define SENSOR3_PIN D10
#define SENSOR4_PIN D11
#define RELAY1_PIN D4
#define RELAY2_PIN D5
#define RELAY3_PIN D6
#define RELAY4_PIN D7
#define LED_PIN 13 // Pin for the blinking LED

const char* fv = "     V 2025.03.28";
String uniqueID;
#define SL_TYPE_ROLL 0x1
#define SL_TYPE_MAIN 0x2

enum SlaveState { IDLE, SEND_SENSOR_DATA, RECEIVE_COMMAND,DEREG };
SlaveState slaveState = SlaveState::IDLE;
bool isRegisteredWithHost = false;
long lastHostUpdate = 0;
const unsigned long healthCheckInterval = 10000; //10s TTL check 

void setup() {

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
    Serial.begin(115200);
    Serial.println("");

    // Build the uniqueID from OpenCyphalUniqueId
    for (int i = 0; i < OpenCyphalUniqueId.ID_SIZE; i++) {
        uniqueID += OpenCyphalUniqueId[i];
    }

    Serial.println();
	delay(250);
    Serial.println("Unique ID: " + uniqueID);
    Serial.println("Slave Node Setup Ready");
    
    selfTestRelays();
    slaveState = SlaveState::DEREG;
}

void loop() {
    switch (slaveState) {
    case DEREG:
    {       
        while (!isRegisteredWithHost) {
            RegisterNode();
            slaveState == SlaveState::RECEIVE_COMMAND;
            if (Serial1.available())
            {
                String c = Serial1.readStringUntil('\n');
                processCommand(c);
            }
        }
        Serial.println("Registered going idle;");
        slaveState = SlaveState::IDLE;
    }break;
    case IDLE: {


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

        if (millis() - lastHostUpdate >= healthCheckInterval) {
            Serial.println("No communication with host for 10 seconds. Going to DEREG.");
            slaveState = SlaveState::DEREG;
            isRegisteredWithHost = false;
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
    static unsigned long lastAttempt = 0;
    unsigned long now = millis();

    if (now - lastAttempt >= 2000) {  // every 2 seconds
        Serial1.println("REG_ROLL:" + uniqueID);
        Serial1.flush();
        Serial.println("Attempting registration...");
        lastAttempt = now;
    }
}

void sendSensorTriggered(String sensor) {
    
    // Send the unique ID and sensor info
    Serial1.print(uniqueID);
    Serial1.print(": ");
    Serial1.println(sensor);	
    Serial1.flush();
   
}

void sendPong() {
    Serial.println("PONG>:" + uniqueID);  
    Serial1.println("PONG:" + uniqueID);    
    Serial1.flush();  
}

void ackCommand(String cmd) {
    Serial.println("ACK>:" + uniqueID);
	Serial1.print("ACK:" + uniqueID);
    Serial1.println(cmd);	
	Serial1.flush();
	
}

void sendResponse(String cmd) {
    Serial.println("ACK>:" + uniqueID);
    Serial1.print("ACK:" + uniqueID);
    Serial1.println(cmd);
    Serial1.flush();

}
void processPingCommand()
{
    isRegisteredWithHost = true;
    lastHostUpdate = millis();
    sendPong();
}
void processCommand(String cmd) {
  
    cmd.trim();
    // Process commands addressed to this slave (relay controls)
    if (cmd.startsWith(uniqueID)) {
        lastHostUpdate = millis();
        if (cmd.endsWith(":PING")) {
    
            processPingCommand();
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
        else if (cmd.endsWith("_AREAD")) {
            int colonIndex = cmd.lastIndexOf(':');
            if (colonIndex != -1) {
                String pinStr = cmd.substring(colonIndex + 1, cmd.indexOf('_', colonIndex));
               
                if (pinStr.charAt(0) == 'A') {
                    int pinNumber = pinStr.substring(1).toInt(); // Extract number part

                    // Construct Arduino A pin constant (e.g. A1, A2, etc.)
                    int analogPin = A0 + pinNumber; // This works for A0 to A7
                    int analogValue = analogRead(analogPin);

                    Serial.print("Reading from ");
                    Serial.print(pinStr);
                    Serial.print(": ");
                    Serial.println(analogValue);
                    
                }
            }
        }
        else {
            Serial.print("Unrecognized command for this node:");
			Serial.println(cmd);
           
        }
    }
    else {
        Serial.print("Foreign command:");
        Serial.println(cmd);
        
    }
}
