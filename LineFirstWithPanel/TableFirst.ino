#include <ezButton.h>

#define PIN_RL1  D4
#define PIN_RL2  D5

#define PIN_BTN_FIRST_VALVE D8
#define PIN_BTN_MIDDLE_VALVES D9

ezButton btnToggleFirst(PIN_BTN_FIRST_VALVE, LOW);
ezButton btnToggleMiddle(PIN_BTN_MIDDLE_VALVES, LOW);

bool rl1State = true;  // HIGH means relay is off (assuming active LOW)
bool rl2State = true;

void setup() {
    Serial.begin(115200);
    Serial.println("");
    Serial.println("Foam Master TABLE MIDDLE V2025.04.09");
    btnToggleFirst.setDebounceTime(150);
	btnToggleMiddle.setDebounceTime(150);

    pinMode(PIN_RL1, OUTPUT);
    pinMode(PIN_RL2, OUTPUT);
    digitalWrite(PIN_RL1, LOW);
    digitalWrite(PIN_RL2, LOW);
}

void loop() {
    btnToggleFirst.loop();
    btnToggleMiddle.loop();
    if (btnToggleFirst.isPressed()) {
        rl1State = !rl1State;  // toggle the state
        digitalWrite(PIN_RL1, rl1State);
        Serial.print("RL1 toggled to ");
        Serial.println(rl1State ? "HIGH (OFF)" : "LOW (ON)");
    }

    if (btnToggleMiddle.isPressed()) {
        rl2State = !rl2State;  // toggle the state
        digitalWrite(PIN_RL2, rl2State);
        Serial.print("RL2 toggled to ");
        Serial.println(rl2State ? "HIGH (OFF)" : "LOW (ON)");
    }

    // you can later add btnToggleMiddle logic here as well
}
