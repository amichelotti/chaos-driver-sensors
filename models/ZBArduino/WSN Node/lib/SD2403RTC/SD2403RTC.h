#ifndef SD2403RTC_h
#define SD2403RTC_h

#include "Arduino.h" // Necessario per avere accesso ai tipi standard e le costanti di Arduino
#include <Wire.h>
#define RTCAddress 0x32 // RTC Address

/*
Nella classe abbiamo una linea per ogni funzione e le variabili necessarie
*/

class SD2403RTC {

	public:
		SD2403RTC();
		String getTimeStamp();
		
	private:
		unsigned char date[7];

};

#endif
