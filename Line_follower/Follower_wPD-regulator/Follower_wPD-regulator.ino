//******************************************************************************
// Zumo32U4 line follower using proportional regulation in combination with smart speed control.
// It works by reducing the drone speed the further away from the line it is,
// which allows for more time for the necessary adjustments and greater level of correction e.g. on sharp turns.
//
// This version has a primitive implementation of derivative regulation as well as p-regulation,
// although it needs proper testing/calibration and is not otherwise finished.
// The plan is to implement the use of more than one measurement for the derivative term in the future (atleast 5).
//
// Version 2.0 21/02/2020
// 2020, Benjaminas Visockis
//******************************************************************************

#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4Motors motor;

#define NUM_SENSORS 5
uint16_t lineSensorValues[NUM_SENSORS];

bool useEmitters = true;

uint8_t selectedSensorIndex = 0;

int leftSpeed,
	rightSpeed,
	speed_correct,
	speed_var,
	proportional_term = 3,
	derivative_term = 1;

int16_t lastError[5] = {0, 0, 0, 0, 0};

uint32_t current_time, check_time = 0;

void calibrateSensors() {
	lcd.clear();

	delay(1000);
	check_time = millis();
	while (millis() - check_time < 3000) {;
		if (millis() - check_time > 500 && millis() - check_time < 2000) {
			motor.setSpeeds(-200, 200);
		} else {
			motor.setSpeeds(200, -200);
		}
		lineSensors.calibrate();
	}
	motor.setSpeeds(0, 0);
}


void setup() {
	lcd.clear();
	lcd.print("Press A");
	lcd.gotoXY(0, 1);
	lcd.print("to calibrate");
	buttonA.waitForButton();
	lineSensors.initFiveSensors();
	calibrateSensors();
	lcd.clear();
	delay(200);
}


void loop() {
	lcd.clear();
	int pos = lineSensors.readLine(lineSensorValues);
  
	int error = map(pos, 0, 4000, 400, -400);

	int speed_var = map(abs(error), 0, 400, 100, 0);
	int output = log(speed_var+1)/log(100)*300;

	int speed_correct = (double)(error * proportional_term) + (double)(derivative_term * (error - lastError[0]));

	leftSpeed = 100 + output + speed_correct;
	rightSpeed = 100 + output - speed_correct;

	lastError[0] = error;

	leftSpeed = constrain(leftSpeed, -400, 400);
	rightSpeed = constrain(rightSpeed, -400, 400);

	motor.setSpeeds(rightSpeed,leftSpeed);
	
	lcd.gotoXY(0,0);
	lcd.print(pos);
	lcd.gotoXY(0,1);
	lcd.print(leftSpeed);
	lcd.gotoXY(6,1);
	lcd.print(rightSpeed);
}
