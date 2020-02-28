//******************************************************************************
// Zumo32U4 control using ESP32 and Blynk IoT services
// Version 1.0 22/02/2020
// 2020, Benjaminas Visockis
//
// To use this code
// Disconnect the LCD screen from Zumo (if connected)
// Establish UART communication from ESP32 to Zumo32U4 by connecting:
// ESP32			----->	Zumo32U4
// TXD2 (GPIO17)	----->	RXD1 (PD2)
// Vin (5V)			----->	5V
// GND				----->	GND
//
// IMPORTANT!
// If you want to establish two way communication between ESP32 and Zumo32U4,
// use a logic level converter, or at least, a voltage divider between ESP32 (RX), which uses 3.3V logic,
// and 32U4 (TX), 5V logic.
// (NOT required in this case, as this is only one way communication)
//******************************************************************************

#include <Zumo32U4.h>

#define DEBUGGING false

Zumo32U4Motors motors;

int value_x,
	value_y,
	LMS,
	RMS,
	max_speed = 400;

struct data_package {
	byte x_val;
	byte y_val;
};
data_package DATA;



void setup() {
	Serial1.begin(115200);
	delay(500);
	
	// Serial debugging with PC.
	#if DEBUGGING
		Serial.begin(9600);
		delay(500);
	#endif
	
}

void loop() {
	if (Serial1.available() > 0) {
		Serial1.readBytes((byte*)&DATA, sizeof(data_package)) == sizeof(data_package);
	}
	if (DATA.y_val < 128) {
		LMS = map(DATA.y_val, 128, 0, 0, -max_speed);
		RMS = map(DATA.y_val, 128, 0, 0, -max_speed);
	} else if (DATA.y_val > 128) {
		LMS = map(DATA.y_val, 128, 255, 0, max_speed);
		RMS = map(DATA.y_val, 128, 255, 0, max_speed);
	} else {
		LMS = 0;
		RMS = 0;
	}
	if (DATA.x_val < 128) {
		int map_x = map(DATA.x_val, 128, 0, 0, max_speed);
		
		LMS -= map_x;
		RMS += map_x;

		if (LMS < -max_speed) {
			LMS = -max_speed;
		}
		if (RMS > max_speed) {
			RMS = max_speed;
		}
	}
	if (DATA.x_val > 128) {
		int map_x = map(DATA.x_val, 128, 255, 0, max_speed);
		
		LMS += map_x;
		RMS -= map_x;

		if (LMS > max_speed) {
			LMS = max_speed;
		}
		if (RMS < -max_speed) {
			RMS = -max_speed;
		}
	}
	motors.setSpeeds(LMS, RMS);

	#if DEBUGGING
		Serial.println(String(String(LMS) + String("   ") + String(RMS)));
	#endif
}
