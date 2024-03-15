#include <AccelStepper.h>

#define CHAIR_MAX_SPEED 1000
#define CHAIR_HOMING_SPEED 1000

#define CHAIR_BASE_SPEED 1500
#define	CHAIR_ACCEL 4000

//motors (4 for chairs 2 for pedals)
#define CHAIR_MAX_DISTANCE -16000
#define CHAIR_BASE_POSITION 7000
//Chair 1 
#define CHAIR1_LEFT_UPPER_LIMIT 48
#define CHAIR1_LEFT_EN 30
#define CHAIR1_LEFT_STEP 10
#define CHAIR1_LEFT_DIR 11
volatile bool IsC1LHomed = false;
bool C1LSubVibReady = true;

byte motorControlPins[] = {
	CHAIR1_LEFT_EN		,
	CHAIR1_LEFT_STEP	,
	CHAIR1_LEFT_DIR		
};
bool IsVibrationEnabled = false;
AccelStepper motorC1L(AccelStepper::DRIVER, CHAIR1_LEFT_STEP, CHAIR1_LEFT_DIR);
// the setup function runs once when you press reset or power the board
void setup() {


	Serial.begin(115200);
	for (int i = 0; i < 3; i++) {
		pinMode(motorControlPins[i], OUTPUT);
		digitalWrite(motorControlPins[i], LOW);
	}


	
	// initialize digital pin 13 as an output.
	motorC1L.setEnablePin(CHAIR1_LEFT_EN);
	motorC1L.setPinsInverted(false, false, true);
	motorC1L.setMinPulseWidth(20);
	motorC1L.disableOutputs();
	delay(120);
	motorC1L.enableOutputs();
	motorC1L.setMaxSpeed(CHAIR_MAX_SPEED);
	motorC1L.setMaxSpeed(100);
	motorC1L.setAcceleration(20);
	motorC1L.moveTo(500);
	Serial.println("Ready");
}

// the loop function runs over and over again forever
void loop() {
	// If at the end of travel go to the other end
	if (motorC1L.distanceToGo() == 0)
		motorC1L.moveTo(-motorC1L.currentPosition());

	motorC1L.run();
}
