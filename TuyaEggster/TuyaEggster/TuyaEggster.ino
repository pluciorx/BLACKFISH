#include <Adafruit_Debounce.h>
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
Tuyav tuyav(&Serial1);
VirtualDelay tuyaYpdateDelay;

uint16_t currVolume = 50;
uint16_t prevVolume = 50;
uint16_t currLowTone = 50;
uint16_t prevLowTone = 50;

bool isPlaying = false;
bool isTuyaInit = false;
bool isSensorInit = false;
#define PIN_PLAY_SENSOR 22
//ezButton btnHumanSensor(PIN_PLAY_SENSOR);
Adafruit_Debounce  btnPlay(PIN_PLAY_SENSOR);

enum E_STATE {
	Playing,
	Idle
};

E_STATE _state;

void setup()
{
	//start serial for debugging
	Serial.begin(115200);
	Serial.print(F("F_CPU = "));
	Serial.println(F_CPU);
	Serial.print(F("Free RAM = "));
	Serial.print(FreeStack(), DEC);
	btnPlay.begin(LOW);
	pinMode(PIN_PLAY_SENSOR, INPUT_PULLUP);

#if (0)
	// Typically not used by most shields, hence commented out.
	Serial.println(F("Applying ADMixer patch."));
	if (MP3player.ADMixerLoad("admxster.053") == 0) {
		Serial.println(F("Setting ADMixer Volume."));
		MP3player.ADMixerVol(-3);
	}
#endif

	setupMp3Player();

	Serial1.begin(9600);
	Serial.println("");
	Serial.println("Blackfish Egg controller");
	tuyav.setDigitalInputs(22, PIN_UNUSED, PIN_UNUSED);
	tuyav.setAnalogInputs(PIN_UNUSED, PIN_UNUSED, PIN_UNUSED);
	tuyav.setDigitalOutputs(4, 5, 23, 25, PIN_UNUSED);
	tuyav.setAnalogOutputs(PIN_UNUSED, PIN_UNUSED, PIN_UNUSED);
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

	switch (_state)
	{
	case Playing: {
		Serial.println("State:Playing");
		
		tuyav.tuyaUpdate();
		SdFile file;
		char filename[13] = "track001.mp3";
		sd.chdir("/", true);
		uint16_t count = 1;

		file.open(filename, O_READ);

		file.getName(filename, sizeof(filename));
		Serial.print("Playing:"); Serial.println(filename);
		if (isFnMusic(filename) && !MP3player.isPlaying()) {
			int8_t result = MP3player.playMP3(filename);
			Serial.print("Play result:"); Serial.println(result);
		}

		tuyav.tuyaUpdate();

		while (MP3player.isPlaying())
		{
			btnPlay.update();
			updateVolume();
			updateLowTone();
			tuyaYpdateDelay.start(2000);
			
			if (tuyaYpdateDelay.elapsed())
			{
				updateUpTime();
				tuyav.setAV9(String(currVolume) + "/" + String(currLowTone));

				tuyav.tuyaUpdate();
				Serial.println("Tuya update..");
				Serial.print("SensorInit:"); Serial.println(isSensorInit);
				Serial.print("TuyaInit:"); Serial.println(isTuyaInit);
				Serial.print("Current Position:"); Serial.println(MP3player.currentPosition());
				tuyaYpdateDelay.start(2000);
			}

			if (isSensorInit && btnPlay.isReleased())
			{
				Serial.println("Human out stopping sound");
				MP3player.stopTrack();
				isPlaying = false;
				_state = E_STATE::Idle;
				break;
			}

			if (isTuyaInit && tuyav.DIGITAL_OUT[4] == LOW)
			{
				Serial.println("Tuya stop initiated. ");
				MP3player.stopTrack();
				isPlaying = false;
				_state = E_STATE::Idle;
				break;
			}
		}

	}break;
	case Idle: {

		Serial.println("State:Idle");
		isSensorInit = false;
		isTuyaInit = false;

		while (!isPlaying)
		{
			tuyaYpdateDelay.start(3000);
			btnPlay.update();
			if (tuyaYpdateDelay.elapsed())
			{
				//set arbitrary values (9 are available - read only in the app)
				tuyav.setUserValue(AV1, "Blackfish EGG");
				tuyav.setUserValue(AV2, "SW V20241029");
				tuyav.setUserValue(AV3, "Idle");

				String AV4msg = "Up time:";
				tuyav.setUserValue(AV4, AV4msg);
				updateUpTime();

				tuyav.setAV6("m");
				tuyav.setAV7("Volume/Bass");
				tuyav.setUserValue(AV8, "Idle");
				tuyav.setAV9(String(currVolume) + "/" + String(currLowTone));
				
				tuyav.tuyaUpdate();
				tuyaYpdateDelay.start(3000);
			}

			if (btnPlay.isPressed() && !isPlaying)
			{
				Serial.println("Person detected !, playing file ");
				tuyav.setUserValue(AV3, "Playing");
				tuyav.setUserValue(AV8, "Local Play");
				tuyav.tuyaUpdate();
				_state = E_STATE::Playing;
				isPlaying = true;
				isSensorInit = true;
			}
			if (tuyav.DIGITAL_OUT[4] == true)
			{

				Serial.println("Tuya start playing file!" && !isPlaying);
				tuyav.setUserValue(AV3, "Playing");
				tuyav.setUserValue(AV8, "Remote Play");
				tuyav.tuyaUpdate();
				_state = E_STATE::Playing;
				isPlaying = true;
				isTuyaInit = true;
			}

			updateVolume();
			updateLowTone();
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

void updateUpTime()
{
	float upTime = millis() / 60000.0;
	Serial.print("Uptime:"); Serial.println(upTime);
	tuyav.setUserValue(AV5, String(upTime));

	tuyav.tuyaUpdate();

}

void updateVolume()
{
	currVolume = constrain(tuyav.ANALOG_OUT[0], 0, 255);
	if (currVolume != prevVolume)
	{
		Serial.print("New Volume:"); Serial.println(currVolume);
		MP3player.setVolume(currVolume, currVolume);
		prevVolume = currVolume;
	}
}

void updateLowTone()
{
	currLowTone = constrain(tuyav.ANALOG_OUT[1], 0, 255);
	if (currLowTone != prevLowTone)
	{
		Serial.print("New Bas Amplitude:"); Serial.println(currLowTone);
		MP3player.setBassAmplitude(currLowTone);
		prevLowTone = currLowTone;
	}
}