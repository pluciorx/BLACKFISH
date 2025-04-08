


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
static int ReadAndUpdateSpeed() {
	
	speed = constrain(1024 - analogRead(PIN_MOTOR_SPD), 0, 1023);
	if (speed != _prevSpeed && abs(_prevSpeed - speed) >= 32) {
		_prevSpeed = speed;

		String message = "SETRSPD:" + String(speed);
		if (isPushRegistered) sendCommand(ADDR_PUSH, message);
		if (isPullRegistered) sendCommand(ADDR_PULL, message);
		
		lcd.setCursor(4, 2);
		lcd.print(map(speed, 0, 1024, 0, 100));
		lcd.print("% ");

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
			Serial.println("Slave " + String(slaveID) + " is already registered.");
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
	Serial.println("=>" + message);
}

static void updateSlaveScreen(int slaveIndex) {
	if (millis() - registeredSlaves[slaveIndex].lastOkTime > healthCheckInterval * 2) {

		switch (registeredSlaves[slaveIndex].slaveType) {
		case ADDR_PUSH:
			isPushRegistered = false;
			lcd.setCursor(0, 1);
			lcd.print("PUSH:- ");
			break;
		case ADDR_PULL:
			isPullRegistered = false;
			lcd.setCursor(8, 1);
			lcd.print("PULL:- ");
			break;
		}
	}
	else {
		switch (registeredSlaves[slaveIndex].slaveType) {
		case ADDR_PUSH:
			isPushRegistered = true;
			lcd.setCursor(0, 1);
			lcd.print("PUSH:OK");
			break;
		case ADDR_ROLL:
			isPullRegistered = true;
			lcd.setCursor(8, 1);
			lcd.print("PULL:OK");
			break;
		}
	}
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
