#include <107-Arduino-UniqueId.h>
#include <Adafruit_Debounce.h>
#include <LiquidCrystal_I2C.h>
#include <avdweb_VirtualDelay.h>
#include <Wire.h>
const char* fv = "     V 2025.03.28";

//#define RS485_CONTROL D2

LiquidCrystal_I2C lcd(0x27, 20, 4);

String uniqueID;

// Kierunek obrotów
#define BUTTON_RIGHT_ROTATE  D2  // Przycisk chwilowy obroty w prawo
#define BUTTON_LEFT_ROTATE   D3  // Przycisk chwilowy obroty w lewo

// Regulacja prêdkoœci
#define POT_SPEED_CONTROL    A3  // Potencjometr regulacji obrotów

// Tryb funkcji ci¹g³ej
#define BUTTON_FUNCTION      A1  // Przycisk chwilowy FUNKCYJNY

// Zmiana œrednicy
#define BUTTON_DIAMETER_UP   D4  // Przycisk chwilowy zmiany œrednicy na plus
#define BUTTON_DIAMETER_DOWN D5  // Przycisk chwilowy zmiany œrednicy na minus

// Zmiana gruboœci
#define BUTTON_THICKNESS_UP   D6  // Przycisk chwilowy zmiany gruboœci na plus
#define BUTTON_THICKNESS_DOWN D7  // Przycisk chwilowy zmiany gruboœci na minus

// Blokada trzymacza rury
#define BUTTON_PIPE_LOCK     D8  // Przycisk blokady trzymacza rury

// Procedura zsuniêcia rury
#define BUTTON_PIPE_RELEASE  D9  // Przycisk procedury zsuniêcia rury

// Presety
#define BUTTON_PRESET_1      D10 // Przycisk PRESET 1
#define BUTTON_PRESET_2      D11 // Przycisk PRESET 2
#define BUTTON_PRESET_3      D12 // Przycisk PRESET 3
#define BUTTON_PRESET_4      D13 // Przycisk PRESET 4

// Wy³¹cznik bezpieczeñstwa
#define BUTTON_STOP          A2  // Przycisk STOP wy³¹czaj¹cy zasilanie silnika

//define the suitable types of the slaves for this host.
#define SL_TYPE_ROLL 0x1
#define SL_TYPE_MAIN 0x2

String commandBuffer;

const int maxSlaves = 2; // Adjust the maximum number of slaves as needed 
                         //for doner line only two slaves are needed

struct SlaveInfo {
    String ID;
    bool isHealthy;
    unsigned long lastCheckedTime; // Timestamp of last health check
    unsigned long lastOkTime; // Timestamp of last health check

    byte slaveType;
};

SlaveInfo registeredSlaves[maxSlaves];
bool isMainRegistered = false;
bool isRollRegistered = false;

int numRegisteredSlaves = 0;

const unsigned long healthCheckInterval = 2000; //10s TTL check 

enum MasterState { IDLE, PROCESS_COMMAND, RECEIVE_DATA, HEALTH_CHECK };

MasterState masterState = MasterState::IDLE;


void setup() {
    Serial.begin(115200);
    Serial.println("");

    pinMode(BUTTON_RIGHT_ROTATE, INPUT_PULLUP);
    pinMode(BUTTON_LEFT_ROTATE, INPUT_PULLUP);
    pinMode(BUTTON_FUNCTION, INPUT_PULLUP);
    pinMode(BUTTON_DIAMETER_UP, INPUT_PULLUP);
    pinMode(BUTTON_DIAMETER_DOWN, INPUT_PULLUP);
    pinMode(BUTTON_THICKNESS_UP, INPUT_PULLUP);
    pinMode(BUTTON_THICKNESS_DOWN, INPUT_PULLUP);
    pinMode(BUTTON_PIPE_LOCK, INPUT_PULLUP);
    pinMode(BUTTON_PIPE_RELEASE, INPUT_PULLUP);
    pinMode(BUTTON_PRESET_1, INPUT_PULLUP);
    pinMode(BUTTON_PRESET_2, INPUT_PULLUP);
    pinMode(BUTTON_PRESET_3, INPUT_PULLUP);
    pinMode(BUTTON_PRESET_4, INPUT_PULLUP);
    pinMode(BUTTON_STOP, INPUT_PULLUP);

    // Potencjometr jako wejœcie analogowe (nie wymaga INPUT_PULLUP)
    pinMode(POT_SPEED_CONTROL, INPUT);

    for (int i = 0; i < OpenCyphalUniqueId.ID_SIZE; i++) {
        uniqueID += OpenCyphalUniqueId[i];
    }
    
    lcd.init(); // initialize the lcd	
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);            // move cursor the first row
    lcd.print("      BLACKFISH   ");          // print message at the first row
    lcd.setCursor(0, 1);            // move cursor to the second row
    lcd.print("    Doner Roller   "); // print message at the second row
    lcd.setCursor(0, 2);            // move cursor to the third row
    lcd.print(fv); // print message at the second row
    
    lcd.setCursor(0, 3);
    lcd.print("ZW:-");
    lcd.setCursor(8, 3);
    lcd.print("CTRL:-");
    delay(500);
   
    Serial1.begin(115200);

    // Initialize registered slaves array
    for (int i = 0; i < maxSlaves; i++) {
        registeredSlaves[i].ID = "";
        registeredSlaves[i].isHealthy = false;
        registeredSlaves[i].lastCheckedTime = 0;
        
    }

    Serial.println("Host Node Setup Ready");
    Serial.print(fv); // print message at the second row
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
    }break;

    case RECEIVE_DATA: {
        String data = Serial1.readStringUntil('\n');
        Serial1.flush();
        if (data.startsWith("REG_ROLL:")) {
            String slaveID = data.substring(9);
            if (registerSlave(slaveID, SL_TYPE_ROLL)) {
                Serial.println("Slave " + slaveID + " registered successfully as ROLL");
            }
            else {
                Serial.println("Slave " + slaveID + " already registered.");
            }
            isRollRegistered = true;
            lcd.setCursor(0, 3);
            lcd.print("ZW:OK");
            
        }
        if (data.startsWith("REG_MAIN:")) {
            String slaveID = data.substring(9);
            if (registerSlave(slaveID, SL_TYPE_MAIN)) {
                Serial.println("Slave " + slaveID + " registered successfully as MAIN");
            }
            else {
                Serial.println("Slave " + slaveID + " already registered.");
            }
            isMainRegistered = true;
            lcd.setCursor(8, 3);
            lcd.print("CTRL:OK");
            
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
            updateSlaveHealth(slaveID, true);
            Serial.println(slaveID + " => ACK");
        }

        masterState = MasterState::IDLE;
    } break;

    case HEALTH_CHECK: {
        // Send a ping to any slave that is overdue for a health check
        for (int i = 0; i < numRegisteredSlaves; i++) {
            if (millis() - registeredSlaves[i].lastCheckedTime >= healthCheckInterval) {
                
                if (registeredSlaves[i].slaveType == SL_TYPE_MAIN)
                {
                    isMainRegistered = false;
                    if (millis() - registeredSlaves[i].lastOkTime >= healthCheckInterval * 2 )
                    {
                        Serial.println("Slave " + registeredSlaves[i].ID + " is not responding. Deregistering...");
                        lcd.setCursor(8, 3);
                        lcd.print("CTRL:- ");
                    }
                }
                if (registeredSlaves[i].slaveType == SL_TYPE_ROLL)
                {
                    
                    isRollRegistered = false;
                    if (millis() - registeredSlaves[i].lastOkTime >= healthCheckInterval * 2)
                    {
						Serial.println("Slave " + registeredSlaves[i].ID + " is not responding. Deregistering...");
                        lcd.setCursor(0, 3);
                        lcd.print("ZW:- ");
                    }
                }

                registeredSlaves[i].isHealthy == false;
                SendPing(registeredSlaves[i].ID);
                registeredSlaves[i].lastCheckedTime = millis();
            }
        }
        masterState = MasterState::IDLE;
    } break;
    }
}
void sendCommand(String id, String cmd) {
    String cmd2 = id + ":" + cmd;
    Serial1.println(cmd2);
    Serial1.flush();
    Serial.print("=>");
    Serial.println(cmd2);

}

void sendCommand(String cmd) {
   
    Serial1.println(cmd);
    Serial1.flush();
    Serial.print("=>");
    Serial.println(cmd);
  
}
bool registerSlave(String slaveID, byte type) {
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
            registeredSlaves[i].isHealthy = true; 
            registeredSlaves[i].lastCheckedTime = millis();
            registeredSlaves[i].lastOkTime = millis();
            registeredSlaves[i].slaveType = type;
            
            numRegisteredSlaves++;
            SendPing(slaveID);
            return true; // Slave registered successfully
            
        }
    }
    
    return false; // No available slots
}


void SendPing(String slaveID)
{
    sendCommand(slaveID + ":PING");
}

void updateSlaveHealth(String slaveID, bool isHealthy) {
    for (int i = 0; i < maxSlaves; i++) {
        if (registeredSlaves[i].ID == slaveID) {

            registeredSlaves[i].isHealthy = isHealthy;
            if (isHealthy) registeredSlaves[i].lastOkTime = millis();
            
            break;
        }
    }
}
