#include "Tuyav.h"
#include "global.h"
#include <SPI.h>
#include <SdFat.h>
#include <sdios.h>
#include <FreeStack.h>
#include <vs1053_SdFat.h>
#include <avdweb_VirtualDelay.h>


#if defined(USE_MP3_REFILL_MEANS) && USE_MP3_REFILL_MEANS == USE_MP3_Timer1
#include <TimerOne.h>
#elif defined(USE_MP3_REFILL_MEANS) && USE_MP3_REFILL_MEANS == USE_MP3_SimpleTimer
#include <SimpleTimer.h>
#endif
SdFat sd;

vs1053 MP3player;
int16_t last_ms_char; // milliseconds of last recieved character from Serial port.
int8_t buffer_pos; // next position to recieve character from Serial port.

char buffer[6]; // 0-35K+null

Tuyav tuyav(&Serial1);
VirtualDelay tuyaYpdateDelay;
#define PIN_PLAY_SENSOR 22

//Initialize Time for updating Arbitrary Values
unsigned long currentTime = 0;
unsigned long previousTime = 0;
int updateDelay = 3000;    //3 seconds by default. Min 1 second or you will overflow the serial communication!
uint8_t currVolume = 50;
uint8_t prevVolume = 50;

bool isPlaying = false;

enum E_STATE {
	Playing,
	Idle
};

E_STATE _state;

void setup()
{
	//start serial for debugging

	pinMode(PIN_PLAY_SENSOR, INPUT_PULLUP);
	Serial.begin(115200);
	Serial.print(F("F_CPU = "));
	Serial.println(F_CPU);
	Serial.print(F("Free RAM = ")); // available in Version 1.0 F() bases the string to into Flash, to use less SRAM.
	Serial.print(FreeStack(), DEC);  // FreeRam() is provided by SdFatUtil.h

#if (0)
	// Typically not used by most shields, hence commented out.
	Serial.println(F("Applying ADMixer patch."));
	if (MP3player.ADMixerLoad("admxster.053") == 0) {
		Serial.println(F("Setting ADMixer Volume."));
		MP3player.ADMixerVol(-3);
	}
#endif

	setupMp3Player();

	last_ms_char = millis(); // stroke the inter character timeout.
	buffer_pos = 0; // start the command string at zero length.


	Serial1.begin(9600);
	Serial.println("");
	Serial.println("Blackfish Egg controller");
	tuyav.setDigitalInputs(PIN_UNUSED, PIN_UNUSED, PIN_UNUSED);                    //Set DigitalInputs
	tuyav.setAnalogInputs(PIN_UNUSED, PIN_UNUSED, PIN_UNUSED);                  //Set AnalogInputs
	tuyav.setDigitalOutputs(4, 5, 23, 25, PIN_UNUSED);  //SetDigitalOutputs
	tuyav.setAnalogOutputs(PIN_UNUSED, PIN_UNUSED, PIN_UNUSED);                  //Set AnalogOutputs (PWM digital pins)
	tuyav.initialize();

	tuyav.tuyaUpdate();
	_state = E_STATE::Idle;
}

void loop()
{
	// Below is only needed if not interrupt driven. Safe to remove if not using.
#if defined(USE_MP3_REFILL_MEANS) \
    && ( (USE_MP3_REFILL_MEANS == USE_MP3_SimpleTimer) \
    ||   (USE_MP3_REFILL_MEANS == USE_MP3_Polled)      )

	MP3player.available();
#endif

	//Should be called continuously 
	tuyav.tuyaUpdate();

	//check time
	currentTime = millis();
	switch (_state)
	{
	case Playing: {

		tuyav.setAV8("Playing");
		tuyav.tuyaUpdate();
		SdFile file;
		char filename[13] = "track001.mp3";
		sd.chdir("/", true);
		uint16_t count = 1;

		currVolume = constrain(tuyav.ANALOG_OUT[0], 0, 255);
		if (currVolume != prevVolume)
		{
			Serial.print("New Volume:"); Serial.println(currVolume);
			MP3player.setVolume(currVolume, currVolume);
			prevVolume = currVolume;
		}

		file.open(filename, O_READ);

		file.getName(filename, sizeof(filename));
		Serial.print("Playing:"); Serial.println(filename);
		if (isFnMusic(filename) && !MP3player.isPlaying()) {

			int8_t result = MP3player.playMP3(filename);
			
			Serial.print("Play result:"); Serial.println(result);

		}
		//	if (MP3player.getState())
			//file.close();


		if (MP3player.isPlaying() && digitalRead(PIN_PLAY_SENSOR) == HIGH && tuyav.DIGITAL_OUT[4] == HIGH)
		{
			Serial.println("Human out stopping sound");
			MP3player.stopTrack();
			isPlaying = false;
			_state = E_STATE::Idle;
		}

		if (MP3player.isPlaying() && (tuyav.DIGITAL_OUT[4] == LOW))
		{
			Serial.println("Tuya stop initiated. ");
			MP3player.stopTrack();
			isPlaying = false;
			_state = E_STATE::Idle;
		}


	}break;
	case Idle: {

		

		while (!isPlaying)
		{
			tuyaYpdateDelay.start(3000);
			if (tuyaYpdateDelay.elapsed())
			{
				//set arbitrary values (9 are available - read only in the app)
				tuyav.setUserValue(AV1, "Blackfish EGG");
				tuyav.setAV2("SW V1.0");
				tuyav.setAV3("V1.0");
				String AV4msg = "Update time:";
				tuyav.setUserValue(AV4, AV4msg);
				tuyav.setUserValue(AV5, String(currentTime - previousTime));
				tuyav.setAV6("ms ago");
				tuyav.setUserValue(AV7, "State");
				tuyav.setAV8("Playing");
				tuyav.setAV9("");
				tuyav.tuyaUpdate();
				tuyav.setAnalogOutputs(50, 50, 50);
				tuyaYpdateDelay.start(3000);
			}


			if (digitalRead(PIN_PLAY_SENSOR) == LOW)
			{
				Serial.println("Person detected !, playing file ");
				tuyav.setAV8("Playing");
				_state = E_STATE::Playing;
				isPlaying = true;
				
				break;

			}
			if (tuyav.DIGITAL_OUT[4] == true)
			{
				Serial.println("Tuya start playing file!");
				tuyav.setAV8("Playing");
				_state = E_STATE::Playing;
				isPlaying = true;
				break;
			}
		}

	}break;
	default:
		break;
	}


}

void setupMp3Player()
{
	//Initialize the SdCard.
	if (!sd.begin(SD_SEL, SPI_FULL_SPEED)) sd.initErrorHalt();
	// depending upon your SdCard environment, SPI_HAVE_SPEED may work better.
	if (!sd.chdir("/")) sd.errorHalt("sd.chdir");

	//Initialize the MP3 Player Shield
	uint8_t result = MP3player.begin();
	//check result, see readme for error codes.
	if (result != 0) {
		Serial.print(F("Error code: "));
		Serial.print(result);
		Serial.println(F(" when trying to start MP3 player"));
		if (result == 6) {
			Serial.println(F("Warning: patch file not found, skipping.")); // can be removed for space, if needed.			
		}
	}
}