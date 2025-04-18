void SendEngineForwardRequest()
{
	if (!isEngRotating) {
		sendCommand(ADDR_PULL, "ENG:F");
		delay(10);
		sendCommand(ADDR_PUSH, "ENG:F");
		isEngRotating = true;
	}

}

void SendProductionStartRequest()
{
	if (!isEngRotating) {
		sendCommand(ADDR_PULL, "ENG:b");
		delay(10);
		sendCommand(ADDR_PUSH, "ENG:b");
		isEngRotating = true;
	}

}

void SendEngineBackwardRequest()
{
	if (!isEngRotating) {
		sendCommand(ADDR_PULL, "ENG:B");
		delay(10);
		sendCommand(ADDR_PUSH, "ENG:B");
		isEngRotating = true;
	}

}

void SendEngineStopRequest(bool isForced)
{
	if (isEngRotating || isForced)
	{
		sendCommand(ADDR_PULL, "ENG:S");
		delay(10);
		sendCommand(ADDR_PUSH, "ENG:S");
		isEngRotating = false;
	}
}

void CheckSlaves() {
	for (int i = 0; i < maxSlaves; i++) {
		if (millis() - registeredSlaves[i].lastCheckedTime >= healthCheckInterval) {
			if (registeredSlaves[i].ID != ADDR_DEREG) {
				SendPing(registeredSlaves[i].ID);
				registeredSlaves[i].lastCheckedTime = millis();
			}

		}
		updateSlaveScreen(i);
	}
}
int ReadAndUpdateSpeed() {
	int adc_value = analogRead(PIN_MOTOR_SPD);
	speed = constrain(1024 - analogRead(PIN_MOTOR_SPD), 0, 1023);
	if (speed != _prevSpeed && abs(_prevSpeed - speed) >= 32) {
		_prevSpeed = speed;

		String message = "ESPD:" + String(map(adc_value, 0, 1023, 0, 255));
		if (isPushRegistered) sendCommand(ADDR_PUSH, message);
		delay(10);
		if (isPullRegistered) sendCommand(ADDR_PULL, message);

		lcd.setCursor(13, 2);

		lcd.print("      ");
		lcd.setCursor(13, 2);
		lcd.print("SPD:");
		lcd.print(map(speed, 0, 1024, 0, 100));
		lcd.print("% ");
		Serial.println("Speed: " + String(speed));
		return speed;

	}
	return _prevSpeed;
}

bool deregisterSlave(char slaveID) {
	for (int i = 0; i < maxSlaves; i++) {
		if (registeredSlaves[i].ID == slaveID) {
			registeredSlaves[i].ID = ADDR_DEREG;
			registeredSlaves[i].isHealthy = false;
			registeredSlaves[i].lastCheckedTime = 0;
			registeredSlaves[i].lastOkTime = 0;
			numRegisteredSlaves--;
			return true;
		}
	}
	return false;
}

bool registerSlave(char slaveID, byte type) {
	// Check if the slave is already registered
	for (int i = 0; i < maxSlaves; i++) {
		if (registeredSlaves[i].ID == slaveID) {
			//Serial.println("Slave " + String(slaveID) + " is already registered.");
			updateSlaveHealth(slaveID, true);

			return true; // Slave already registered
		}
	}
	// Find an empty slot in the array
	for (int i = 0; i < maxSlaves; i++) {
		if (registeredSlaves[i].ID == ADDR_DEREG) {
			registeredSlaves[i].ID = slaveID;
			registeredSlaves[i].isHealthy = true;
			registeredSlaves[i].lastCheckedTime = millis();
			registeredSlaves[i].lastOkTime = millis();
			registeredSlaves[i].slaveType = type;

			numRegisteredSlaves++;
			Serial.println("Slave " + String(slaveID) + " registered successfully.");
			SendPing(slaveID);

			return true; // Slave registered successfully

		}
	}
	Serial.println("No available slots for new slave.");
	return false; // No available slots
}



void SendPing(char slaveID)
{
	sendCommand(slaveID, "PING");
}


void sendCommand(char slave_id, String cmd) {
	String message = cmd + ":" + String(slave_id);
	Serial1.println(message);
	Serial1.flush();
	delay(3);
	Serial.println("=>" + message);
}

static void updateSlaveScreen(int slaveIndex) {
	if (millis() - registeredSlaves[slaveIndex].lastOkTime > healthCheckInterval * 2) {

		switch (registeredSlaves[slaveIndex].slaveType) {
		case ADDR_PUSH:
			isPushRegistered = false;
			lcd.setCursor(0, 2);
			lcd.print("IN:- ");
			break;
		case ADDR_PULL:
			isPullRegistered = false;
			lcd.setCursor(6, 2);
			lcd.print("OUT:- ");
			break;
		}
	}
	else {
		switch (registeredSlaves[slaveIndex].slaveType) {
		case ADDR_PUSH:
			isPushRegistered = true;
			lcd.setCursor(0, 2);
			lcd.print("IN:OK");
			break;
		case ADDR_ROLL:
			isPullRegistered = true;
			lcd.setCursor(8, 2);
			lcd.print("OUT:OK");
			break;
		}
	}
}

void sendReadinesRequest(char slaveID) {
	String message = "ISRREQ";
	sendCommand(slaveID, message);
}

void checkAndDeregisterSlaves() {
	for (int i = 0; i < maxSlaves; i++) {
		if (registeredSlaves[i].ID != ADDR_DEREG &&
			millis() - registeredSlaves[i].lastOkTime > healthCheckInterval * 2) {
			Serial.println("Deregistering slave " + String(registeredSlaves[i].ID));
			deregisterSlave(registeredSlaves[i].ID);

		}
	}
}
void updateSlaveHealth(char slaveID, bool isHealthy) {
	for (int i = 0; i < maxSlaves; i++) {
		if (registeredSlaves[i].ID == slaveID) {
			registeredSlaves[i].isHealthy = isHealthy;
			registeredSlaves[i].lastCheckedTime = millis();
			if (isHealthy) registeredSlaves[i].lastOkTime = millis();
			break;
		}
	}
}
