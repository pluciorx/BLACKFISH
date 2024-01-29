
// Pin definitions
//buttons (2 x 4 )

#define P1_B1 12
#define P1_B2 11
#define P1_B3 10
#define P1_B4 9

#define P2_B1 7
#define P2_B2 6
#define P2_B3 5
#define P2_B4 4
// end buttons

//Com
#define COM_BAUD_Debug 115200
#define COM_BAUD_PC 115200

//motors
//Chair1 (4x3
#define CHAIR1_LEFT_UPPER_LIMIT 8
#define CHAIR1_LEFT_LOWER_LIMIT 9
#define CHAIR1_LEFT_EN 10
#define CHAIR1_LEFT_RST 11
#define CHAIR1_LEFT_STEP 12
#define CHAIR1_LEFT_DIR 13
#define CHAIR1_LEFT_INTERFACE_TYPE 1

#define CHAIR1_RIGHT_UPPER_LIMIT 8
#define CHAIR1_RIGHT_LOWER_LIMIT 9
#define CHAIR1_RIGHT_EN 10
#define CHAIR1_RIGHT_RST 11
#define CHAIR1_RIGHT_STEP 12
#define CHAIR1_RIGHT_DIR 13
#define CHAIR1_RIGHT_INTERFACE_TYPE 1
//Charir 2 
#define CHAIR1_RIGHT_UPPER_LIMIT 8
#define CHAIR1_RIGHT_LOWER_LIMIT 9
#define CHAIR1_RIGHT_EN 10
#define CHAIR1_RIGHT_RST 11
#define CHAIR1_RIGHT_STEP 12
#define CHAIR1_RIGHT_DIR 13
#define CHAIR1_RIGHT_INTERFACE_TYPE 1

#define CHAIR2_LEFT_UPPER_LIMIT 8
#define CHAIR2_LEFT_LOWER_LIMIT 9
#define CHAIR2_LEFT_EN 10
#define CHAIR2_LEFT_RST 11
#define CHAIR2_LEFT_STEP 12
#define CHAIR2_LEFT_DIR 13
#define CHAIR2_LEFT_INTERFACE_TYPE 1

enum E_STATE {
	HOMING,
	READY,
	SEND_PC,
	MOTOR
};

void setup() {
	Serial.begin(COM_BAUD_Debug, SERIAL_8N1);
	Serial1.begin(COM_BAUD_PC, SERIAL_8N1);
	
}

// the loop function runs over and over again until power down or reset
void loop() {
  
}
