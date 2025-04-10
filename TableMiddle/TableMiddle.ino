
#define PIN_RL1  D4
#define PIN_RL2  D5


// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(115200);
	Serial.println("");
	Serial.println("Foam Master TABLE MIDDLE V2025.04.09");

	pinMode(PIN_RL1, OUTPUT);
	pinMode(PIN_RL2, OUTPUT);
	digitalWrite(PIN_RL1, HIGH);
	digitalWrite(PIN_RL2, HIGH);
}

// the loop function runs over and over again until power down or reset
void loop() {
  
}
