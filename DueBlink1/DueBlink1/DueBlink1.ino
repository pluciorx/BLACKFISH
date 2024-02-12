

#include <AccelStepper.h>

AccelStepper motor(AccelStepper::DRIVER, 10, 11);
// the setup function runs once when you press reset or power the board
void setup() {
	Serial.println("Test");
  // initialize digital pin 13 as an output.
	motor.setEnablePin(30);
	motor.setPinsInverted(false, false, true);
	motor.enableOutputs();
	motor.setAcceleration(400);
	motor.setMaxSpeed(500); 
	motor.move(5000);
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
	Serial.print(",,,");
}

// the loop function runs over and over again forever
void loop() {
	motor.run();
	Serial.println("xxx");
}
