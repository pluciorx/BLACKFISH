#include <AccelStepper.h>

#define CHAIR_MAX_SPEED 2500
#define CHAIR_HOMING_SPEED 1200

#define CHAIR_BASE_SPEED 1500
#define	CHAIR_ACCEL 4000

//motors (4 for chairs 2 for pedals)
#define CHAIR_MAX_DISTANCE -16000
#define CHAIR_BASE_POSITION 7000
//Chair 1 
#define CHAIR1_LEFT_UPPER_LIMIT 48
#define CHAIR1_LEFT_EN 44
#define CHAIR1_LEFT_STEP 2
#define CHAIR1_LEFT_DIR 3
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
	motorC1L.setMinPulseWidth(250);
	motorC1L.disableOutputs();
	delay(120);
	motorC1L.enableOutputs();
	motorC1L.setMaxSpeed(CHAIR_MAX_SPEED);
	
	Serial.println("Ready");
}

// the loop function runs over and over again forever
void loop() {
	if (IsVibrationEnabled)
	{
		while (motorC1L.distanceToGo() != 0 )
		{
			motorC1L.run();
      //delay(1);
			if (motorC1L.distanceToGo() == 0) C1LSubVibReady = true;
		}

		/*for (int i = 0; i < 1611; i++)
		{
			motorC1L.run();
			if (motorC1L.distanceToGo() == 0) C1LSubVibReady = true;
		}*/
		
	}
	if (C1LSubVibReady)
	{
		float speed = random(1, 4) * CHAIR_BASE_SPEED;
    
		float accel = random(1, 2)* CHAIR_ACCEL;
		float newPosition = random(-8000, 800);
		Serial.print("C1L New vibration move s/a/p:");
		Serial.println(speed);
		Serial.println(accel);
		Serial.println(newPosition);
		motorC1L.stop();
		motorC1L.setAcceleration(accel);
		motorC1L.setSpeed(speed);
		motorC1L.moveTo(newPosition);

		IsVibrationEnabled = true;
		C1LSubVibReady = false;
	}
}
