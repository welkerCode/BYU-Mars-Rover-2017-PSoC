/* ========================================
 * BYU Mars Rover 2016
 * Authors: Sam Bury, Marshall Garey
 * ========================================
*/
#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <project.h>

#define WRIST_ROTATE_ID 0x01
#define WRIST_TILT_ID 0x02

void wristGoalPosition( uint8 servoID, uint16 position);
void setWristTorque( uint8 servoID, uint16 torque);
void wristSpeed( uint8 servoID, uint16 speed);

#endif
/* [] END OF FILE */


/*
    //Initialize the dynamixels <-- Will do this on the computer
    ServoSpeed(0xFE, 300); // also only do once
    SetServoTorque(0xFE, 0x03FF); // maximum
	
	after init just use servoGoalPosition
	
	rotate - 0-4095; 2048 is middle
	
	tilt - 1024 - 3072; 2048 is middle
	
	initialize to the middle
*/