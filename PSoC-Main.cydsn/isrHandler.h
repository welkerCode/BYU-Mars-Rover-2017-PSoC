/* ========================================
 * BYU Mars Rover 2016
 * Authors: Marshall Garey, Rodolfo Alberto
 * ========================================
*/
#ifndef __ISR_HANDLER_H__ 
#define __ISR_HANDLER_H__ 

#include <project.h>

#define DEBUG_MODE 0

int compRxEventHandler();
void scienceEventHandler();
void heartbeatEventHandler();
void updateTurretPos();
void updateShoulderPos();
void updateElbowPos();
void updateForearmPos();

// Selects camera feeds
void selectCameras(uint8_t byte);

// Controls the h-bridges
void control_chutes(uint8_t byte);

// Sends test packet to psoc slave - just for debugging and testing
void sendDummySlaveCmd(uint8_t hand, uint8_t cam1, uint8_t cam2,
    uint8_t chuteSelect);

// event variables
extern volatile uint32_t events;

#define SERVO_NEUTRAL 1500
#define SERVO_MAX 2000
#define SERVO_MIN 1000

// events
#define COMP_RX_EVENT 0x0001
#define HEARTBEAT_EVENT 0x0002

// science sensor feedback event
#define SCIENCE_EVENT 0x0004

// positional feedback events
#define TURRET_POS_EVENT 0x0010
#define SHOULDER_POS_EVENT 0x0020
#define ELBOW_POS_EVENT 0x0040
#define FOREARM_POS_EVENT 0x0080

// Hand
void driveHand(uint16_t pos);

// general macros
#define SUCCESS 0
#define UART_READ_ERROR 1
#define MESSAGE_ERROR 2

// PWM values for video mux out of 5 video selects
#define VIDEO1 1100
#define VIDEO2 1500
#define VIDEO3 1900

#endif
/* [] END OF FILE */
