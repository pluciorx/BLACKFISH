#include <AccelStepper.h>

#define dirPin1 D7
#define stepPin1 D6

#define dirPin2 D5
#define stepPin2 D4
#define motorInterfaceType 1

AccelStepper stepper1 = AccelStepper(AccelStepper::DRIVER, stepPin1, dirPin1);

AccelStepper stepper2 = AccelStepper(AccelStepper::DRIVER, stepPin2, dirPin2);

void setup() {
	pinMode(dirPin1, OUTPUT);
	pinMode(stepPin1, OUTPUT);

	pinMode(dirPin2, OUTPUT);
	pinMode(stepPin2, OUTPUT);



	stepper1.setMaxSpeed(100);
	stepper1.setSpeed(200);
	stepper1.setMinPulseWidth(20);


	stepper2.setMaxSpeed(100);
	stepper2.setSpeed(500);
	stepper2.setMinPulseWidth(20);

	Serial.begin(115200);
	Serial.println("Starting StepperTest");
}

void loop() {
	
	stepper2.runSpeed();
	stepper1.runSpeed();
	//stepper2.runSpeed();
}