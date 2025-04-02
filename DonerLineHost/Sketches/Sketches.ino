#include <AccelStepper.h>

#define dirPin1 D3
#define stepPin1 D2
#define dirPin2 D5
#define stepPin2 D4
#define motorInterfaceType 1

AccelStepper stepper1 = AccelStepper(AccelStepper::DRIVER, stepPin1, dirPin1);
AccelStepper stepper2 = AccelStepper(AccelStepper::DRIVER, stepPin2, dirPin2);

void setup() {
	stepper1.setMaxSpeed(100);
	stepper1.setSpeed(50);
	stepper2.setMaxSpeed(100);
	stepper2.setSpeed(50);
	Serial.begin(9600);
	Serial.println("Starting StepperTest");
}

void loop() {
	stepper1.runSpeed();
	stepper2.runSpeed();
}