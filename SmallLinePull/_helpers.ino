void SetRollerSpeed(int speed) {
	
}

void ackCommand(String cmd) {
	String message = String(nodeAddr) + "ACK:" + cmd;
	Serial1.println(message);
	Serial1.flush();
	Serial.println("=>:" + message);

}
void sendCommand(char slave_id, String cmd) {
	String message = String(slave_id) + cmd;
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
	// Perform the necessary actions to set the speed
	// For example, you can set a global variable or control a motor
	Serial.print("Setting speed to: ");
	Serial.println(speedValue);

	// Acknowledge the command
	ackCommand(cmd);
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
	String message = String(nodeAddr) + "PONG";
	Serial1.println(message);
	Serial1.flush();
	Serial.println("=>:" + message);
}
