//******************************************************************************
// Zumo32U4 line follower using proportional regulation in combination with smart speed control.
// It works by reducing the drone speed the further away from the line it is,
// which allows for more time for the necessary adjustments and greater level of correction e.g. on sharp turns.
//
// This version also adds a primitive way of getting back onto the line, in case of the drone was pushed away.
// After a set amount of time of not getting any new sensor data the car starts to drive in circles of increasing radius,
// until a new sensor value is detected. In most cases it means the drone has gotten back to line, but it is prone to errors,
// as it also reacts to any uneven surfaces, markings or inperfections in the floor.
//
// Version 1.1 21/02/2020
// 2020, Benjaminas Visockis
//******************************************************************************


#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
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
	speed_var;


void calibrateSensors() {
	lcd.clear();

	delay(1000);
	for(uint16_t i = 0; i < 120; i++) {
		if (i > 30 && i <= 90) {
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
	int pos = lineSensors.readLine(lineSensorValues);
	
	speed_correct = map(pos, 0, 4000, 400, -400);
	speed_var = map(abs(speed_correct), 0, 400, 200, 0);
	
	leftSpeed = 100 + speed_var + speed_correct;
	rightSpeed = 100 + speed_var - speed_correct;
	
	leftSpeed = constrain(leftSpeed, -400, 400);
	rightSpeed = constrain(rightSpeed, -400, 400);

	if (leftSpeed != lastLeftSpeed && rightSpeed != lastRightSpeed && out_of_bounds == false) {
		lastLeftSpeed = leftSpeed;
		lastRightSpeed = rightSpeed;
		check_time = current_time;
	}
	if (delta_time >= 1000) {
		if (delta_time <= 1100) {
			out_of_bounds = true;
			motor.setSpeeds(0, 0);
			delay(200);
		}
	motor.setSpeeds(200 + (delta_time / 1000), 100 + (delta_time / 500));
	
	if (pos > 1800 && pos < 2200) {
		out_of_bounds = false;
	}
	} else {
		motor.setSpeeds(rightSpeed,leftSpeed);
	} 
	lcd.gotoXY(0,0);
	lcd.print(pos);
	lcd.gotoXY(0,1);
	lcd.print(leftSpeed);
	lcd.gotoXY(6,1);
	lcd.print(rightSpeed);
}
