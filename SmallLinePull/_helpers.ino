void SetRollerSpeed(int speed) {
		
	analogWrite(PIN_MOTOR_SPD, speed);
}

void ackCommand(String cmd) {
	String message = "ACK:" + cmd;
	sendCommand(nodeAddr, message);
}


void sendCommand(char slave_id, String cmd) {
	String message = cmd + ":" + String(slave_id);
	Serial1.println(message);
	Serial1.flush();
	Serial.println("=>" + message);
}

bool processRSPEED(String cmd) {
	int colonIndex = cmd.indexOf(':');
	if (colonIndex == -1) {
		Serial.println("Invalid SETRSPD command format.");
		return false;
	}

	String speedStr = cmd.substring(colonIndex + 1);
	int speedValue = speedStr.toInt();
	SetRollerSpeed(speedValue);

	Serial.print("Setting speed to: ");
	Serial.println(speedValue);

	return true;
}

bool processAnalogReadCommand(String cmd) {
	if (!cmd.endsWith("_AREAD")) return false;

	int colonIndex = cmd.lastIndexOf(':');
	if (colonIndex == -1) return false;

	String pinStr = cmd.substring(colonIndex + 1, cmd.indexOf('_', colonIndex));
	int pinNumber = pinStr.toInt();
	int analogValue = analogRead(pinNumber);

	Serial.print("Reading from pin ");
	Serial.print(pinNumber);
	Serial.print(": ");
	Serial.println(analogValue);
	sendCommand(nodeAddr, "AREAD:" + String(pinNumber) + ":" + String(analogValue));
	return true;
}

void processPingCommand()
{
	
	String message = "PONG";
	sendCommand(nodeAddr, message);
	//Serial.println("=>:" + message);
	
}

void engineMoveBackward()
{
	Serial.println("Engine backward");
	if (!isEngineRotating) {
		digitalWrite(PIN_RL_FORWARD, LOW);
		digitalWrite(PIN_RL_BACKWARD, HIGH);
		isEngineRotating = true;
	}
}

void engineMoveForward()
{
	Serial.println("Engine forward");
	if (!isEngineRotating) {
		digitalWrite(PIN_RL_FORWARD, HIGH);
		digitalWrite(PIN_RL_BACKWARD, LOW);
		isEngineRotating = true;
	}
}

void engineStop()
{
	Serial.println("Engine stop");
	if (isEngineRotating || isProductionRunning)
	{
		digitalWrite(PIN_RL_FORWARD, LOW);
		digitalWrite(PIN_RL_BACKWARD, LOW);
		isEngineRotating = false;
	
	}
}