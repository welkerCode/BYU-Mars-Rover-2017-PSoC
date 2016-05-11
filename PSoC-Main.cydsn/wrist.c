/* ========================================
 * BYU Mars Rover 2016
 * Authors: Sam Bury, Marshall Garey
 * ========================================
*/

#include <project.h>
#include "wrist.h"

void wristGoalPosition( uint8 servoID, uint16 position) {
    static uint8 array[9];
    
    array[0] = 0xFF;
    array[1] = 0xFF;
    array[2] = servoID;
    array[3] = 0x05;
    array[4] = 0x03; //write instruction
    array[5] = 0x1E;
    array[6] = position; // little endian
    array[7] = position >> 8;
    array[8] = ~(servoID + 0x05 + 0x1E + array[6] + array[7] + 0x03);
    
//    UART_Wrist_PutArray(array, 0x09);  
}

void setWristTorque( uint8 servoID, uint16 torque) {
    static uint8 array[9];
    
    array[0] = 0xFF;
    array[1] = 0xFF;
    array[2] = servoID; 
    array[3] = 0x05;
    array[4] = 0x03; //write instruction
    array[5] = 0x0E;
    array[6] = torque;
    array[7] = torque >> 8;
    array[8] = ~(servoID + 0x05 + 0x0E + array[6] + array[7] + 0x03);
    
//    UART_Wrist_PutArray(array, 0x09);
}    

void wristSpeed( uint8 servoID, uint16 speed) {
    static uint8 array[9];
    
    array[0] = 0xFF;
    array[1] = 0xFF;
    array[2] = servoID;
    array[3] = 0x05;
    array[4] = 0x03; //write instruction
    array[5] = 0x20;
    array[6] = speed;
    array[7] = speed >> 8;
    array[8] = ~(servoID + 0x05 + 0x20 + array[6] + array[7] + 0x03);
    
//    UART_Wrist_PutArray(array, 0x09);  
}
    
/* [] END OF FILE */
