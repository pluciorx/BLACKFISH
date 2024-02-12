#ifndef Logger_h
#define Logger_h
#include "Arduino.h" 
class Logger {
public:
	Logger();
	void Init(HardwareSerial& );
	void Logln(const __FlashStringHelper&);
private:
	int _pin;
	HardwareSerial& serial;
};
#endif