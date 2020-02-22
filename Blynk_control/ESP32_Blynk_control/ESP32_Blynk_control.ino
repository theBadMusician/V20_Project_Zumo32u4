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

#define BLYNK_PRINT Serial
#define DEBUGGING false
#define NTNU_SERVER false	// Only works on a NTNU own network or VPN connected through NTNU server.

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>


char auth[] = "********************************"
char ssid[] = "YOUR_SSID";
char pass[] = "YOUR_PASSWORD";

int value_x,
	value_y,
	LMS,
	RMS,
	max_speed = 400;


BLYNK_WRITE(V0)
{
	value_x = param.asInt();
}


BLYNK_WRITE(V1)
{
	value_y = param.asInt();
}


struct data_package {
	byte x_val;
	byte y_val;
};
data_package DATA;


void setup() {
	Serial2.begin(115200);
	delay(500);
	
	// Serial for debugging with PC.
	if (DEBUGGING) {
		Serial.begin(9600);
		delay(500);
	}
	if (NTNU_SERVER) {
		Blynk.begin(auth, ssid, pass, IPAddress(129,241,2,116), 8080);
	} else {
		Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 80);
	}
}

void loop()
{
  Blynk.run();

	DATA.x_val = value_x;
	DATA.y_val = value_y;

	Blynk.virtualWrite(V2, DATA.x_val);
	Blynk.virtualWrite(V3, DATA.y_val);
	
	Serial2.write((byte*)&DATA, sizeof(data_package));
	
	if (DEBUGGING) {
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
		Serial.print(LMS);
		Serial.print("   ");
		Serial.println(RMS);
	}
}
