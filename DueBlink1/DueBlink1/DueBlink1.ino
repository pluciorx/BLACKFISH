
#define BTN_PULL_RIGHT 22
#define BTN_PULL_LEFT 23
#define BTN_HEAT1 24
#define BTN_HEAT2 25
#define BTN_PROD_START 26
#define BTN_PROD_END A7
#define BTN_TAPE_LEFT 28
#define BTN_TAPE_RIGHT 29
#define BTN_FAIL_STOP 30

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(115200);

	pinMode(BTN_PROD_START, INPUT);
	pinMode(BTN_PROD_END, INPUT_PULLUP);
	Serial.println("GO");
	delay(5000);
	
}

// the loop function runs over and over again forever
void loop() {

	Serial.println(digitalRead(BTN_PROD_END));
	delay(5);
}
