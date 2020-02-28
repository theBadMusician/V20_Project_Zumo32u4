//******************************************************************************
// Zumo32U4 line follower using proportional regulation in combination with cruise control.
// It works by reducing the drone speed the further away from the line it is,
// which allows for more time for the necessary adjustments and greater level of correction e.g. on sharp turns.
//
// This version has an implementation of derivative regulation as well as p-regulation,
// although it needs more testing/calibration of the p-term, d-term, and cruise control definition.
// D-regulation works fine with sample rate of 0ms, as there isn't a lot of noise/false readings from line sensors.
// Derivative was implemented by using simple average approximation between find the rate of change between them.
// For better sampling the program uses five points error measurements.
// This could increased/decreased for better definition/more noise reduction. 
//
// Version 2.1 28/02/2020
// 2020, Benjaminas Visockis
//******************************************************************************

#include <Wire.h>
#include <Zumo32U4.h>
#include <stdio.h>

Zumo32U4LCD 				lcd;
Zumo32U4ButtonA 			buttonA;
Zumo32U4LineSensors 		lineSensors;
Zumo32U4ProximitySensors 	proxSensors;
Zumo32U4Motors 				motor;

#define		NUM_SENSORS 5
uint16_t	lineSensorValues[NUM_SENSORS];
bool 		useEmitters = true;
uint8_t 	selectedSensorIndex = 0;

#define  	proportional_term  0.9
#define		derivative_term  9.0
#define		sample_rate 0
#define		cc_definition 600

double 		error[5] = {0, 0, 0, 0, 0};
int16_t 	num_of_errors = sizeof(error)/sizeof(int);

int16_t 	left_speed,
			right_speed,
			turn_control,
			cruise_control;

uint32_t	check_time_calibrate,
			check_time_errors[5],
			check_time_lcd;


void calibrate_sensors() {
	lcd.clear();

	delay(1000);
	check_time_calibrate = millis();
	while (millis() - check_time_calibrate < 4000) {;
		if (millis() - check_time_calibrate > 1500 && millis() - check_time_calibrate < 3500) {
			motor.setSpeeds(-200, 200);
		} else {
			motor.setSpeeds(200, -200);
		}
		lineSensors.calibrate();
	}
	motor.setSpeeds(0, 0);
}


void setup() {
	Serial.begin(9600);

	lcd.clear();
	lcd.print("Press A");
	lcd.gotoXY(0, 1);
	lcd.print("to calibrate");
	buttonA.waitForButton();
	
	lineSensors.initFiveSensors();
	calibrate_sensors();
	
	lcd.clear();
	delay(200);
	check_time_errors[5] = {millis()};
}


void loop() {
	// char buffer[100];
	// sprintf(buffer, "%d   %d   %d   %d   %d", error[0], error[1], error[2], error[3], error[4]);
	// Serial.println(buffer);

	int pos = lineSensors.readLine(lineSensorValues);

	for (uint8_t i = 4; i > 0; i--) {
		if(millis() - check_time_errors[i] > sample_rate) {
			error[i] = error[i - 1];
			check_time_errors[i] = {millis()};
		}
	}

	error[0] = map(pos, 0, 4000, 400, -400);

	uint16_t cruise_control = map(abs(error[0]), 0, 400, cc_definition, 0);
	// cruise_control = log(cruise_control + 1)/log(cc_definition) * 250;
	cruise_control = pow(cruise_control, 2)/pow(cc_definition, 2) * 250;

	int16_t turn_control = (error[0] * proportional_term) + (derivative_term * ((error[0] - error[5]) / num_of_errors));

	left_speed = 100 + cruise_control + turn_control;
	right_speed = 100 + cruise_control - turn_control;
	
	left_speed = constrain(left_speed, -400, 400);
	right_speed = constrain(right_speed, -400, 400);

	motor.setSpeeds(right_speed, left_speed);
	
	if (millis() - check_time_lcd > 100) {
		lcd.clear();
		lcd.gotoXY(0,0);
		lcd.print(pos);
		lcd.gotoXY(0,1);
		lcd.print(left_speed);
		lcd.gotoXY(4,1);
		lcd.print(right_speed);
		check_time_lcd = millis();
	}
}
