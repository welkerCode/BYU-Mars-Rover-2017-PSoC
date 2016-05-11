/* ========================================
 * BYU Mars Rover 2016
 * Authors: Marshall Garey, Rodolfo Alberto
 * ========================================
*/
#include "isrHandler.h"
#include "pololuControl.h"
#include <stdio.h>

// Sends feedback to the on-board computer
static void feedbackToOnboardComputer();

// Displays feedback in a readable format to a terminal
// For debugging only
static void feedbackToTerminal();

// Generates fake science data and outputs to UART
static void generateScienceTestData();

// Send a command to the slave PSoC
static void psocSlaveCmd();

enum hand_e { open, close, halt };
// Update hand position in payload
static void handMsg(enum hand_e handPos);

// Container for all events. See isrHandler.h for macros that define the events.
volatile uint32_t events = 0;

// Arm payload struct
// These are the last positions we wrote to the motors
static struct Payload {
    uint16_t leftWheels;
    uint16_t rightWheels;
    uint16_t cameraPan;
    uint16_t cameraTilt;
    uint8_t cameraNum;
    uint8_t chutes;
    uint16_t turretDest;
    uint16_t shoulderDest;
    uint16_t elbowDest;
    uint16_t forearmDest;
    uint16_t wristTiltDest;
    uint16_t wristSpinDest;
    uint16_t handDest;
    uint16_t shovel;
} Payload;

static struct {
    uint8_t hand;
    uint8_t camSelect;
    uint8_t chuteSelect;
} PSoC_Slave_Payload;

// Arm positions
// These are the most recent positions received as feedback from the motors
// We really don't need feedback from the hand since it's just open/close.
volatile uint16_t turretPos;
volatile uint16_t shoulderPos;
volatile uint16_t elbowPos;
volatile uint16_t forearmPos;

// Science sensor data
volatile int16_t temperature = 0;
volatile int16_t humidity = 0;

#define POSITION_PAYLOAD_SIZE (13) // 1 start byte, 2 bytes per joint, 6 joints
// The positions are stored in little endian format - low byte first, then
// high byte, in order from joints closest to the rover outward
// [turretlo, turrethi, shoulderlo, shoulderhi, elbowlo, elbowhi,
// forearmlo, forearmhi, wristspinlo, wristspinhi, wristtiltlo, wristtilthi]
static uint8_t feedbackArray[POSITION_PAYLOAD_SIZE];

// State machine states to receive commands from computer
// The state machine is defined in the function compRxEventHandler
#define PREAMBLE0 0xEA
static enum compRxStates_e { pre0, leftlo, lefthi, rightlo, righthi, campanlo,
    campanhi, camtiltlo, camtilthi, camSelect, turretlo, turrethi, 
    shoulderlo, shoulderhi, elbowlo, elbowhi, forearmlo, forearmhi,
    wristtiltlo, wristtilthi, wristspinlo, wristspinhi, 
    handlo, handhi, chutes, shovello, shovelhi } compRxState;

// Receive a message from the computer
int compRxEventHandler() {
    // get next element in uart rx buffer
    static uint16_t data;
    static uint8_t byte;
    static uint16_t oldHandDest;
    
    // Keep reading the rx buffer until empty.
    // GetRxBufferSize gets the number of bytes in the software buffer,
    // but not the hardware FIFO (which is 4 bytes in size), so we also want
    // to call ReadRxStatus and check if the RX_STS_FIFO_NOTEMPTY bit was set.
    while (UART_Computer_GetRxBufferSize() || 
          (UART_Computer_ReadRxStatus() & UART_Computer_RX_STS_FIFO_NOTEMPTY))
    {
        // MSB contains status, LSB contains data; if status is nonzero, an 
        // error has occurred
        data = UART_Computer_GetByte();
        
        // check status
        // TODO: maybe make an error LED and use this in the main function
        if (data & 0xff00) {
            return UART_READ_ERROR;
        }
        
        // mask the data to a single byte
        byte = data & 0xff;
        
        // state machine
        switch(compRxState) {
        case pre0:
            if (byte == PREAMBLE0) {
                compRxState = leftlo; // change state
            }
            break;
        case leftlo:
            Payload.leftWheels = byte;
            compRxState = lefthi; // change state
            break;
        case lefthi:
            Payload.leftWheels |= byte << 8;
            PWM_Drive_WriteCompare1(Payload.leftWheels);
            compRxState = rightlo; // change state
            break;
        case rightlo:
            Payload.rightWheels = byte;
            compRxState = righthi; // change state
            break;
        case righthi:
            Payload.rightWheels |= byte << 8;
            PWM_Drive_WriteCompare2(Payload.rightWheels);
            compRxState = campanlo; // change state
            break;
        case campanlo:
            Payload.cameraPan = byte;
            compRxState = campanhi; // change state
            break;
        case campanhi:
            Payload.cameraPan |= byte << 8;
            compRxState = camtiltlo;
            PWM_Gimbal_WriteCompare1(Payload.cameraPan);
            compRxState = camtiltlo; // change state
            break;
        case camtiltlo:
            Payload.cameraTilt = byte;
            compRxState = camtilthi; // change state
            break;
        case camtilthi:
            Payload.cameraTilt |= byte << 8;
            PWM_Gimbal_WriteCompare2(Payload.cameraTilt);
            compRxState = camSelect; // change state
            break;
        case camSelect:
            // update camera number in packet to send to slave PSoC
            PSoC_Slave_Payload.camSelect = byte;
            compRxState = turretlo; // change state
            break;
        case turretlo:
            Payload.turretDest = byte;
            compRxState = turrethi; // change state
            break;
        case turrethi:
            Payload.turretDest |= byte << 8;
            pololuControl_driveMotor(Payload.turretDest,
                POLOLUCONTROL_TURRET);
            compRxState = shoulderlo; // change state
            break;
        case shoulderlo:
            Payload.shoulderDest = byte;
            compRxState = shoulderhi; // change state
            break;
        case shoulderhi:
            Payload.shoulderDest |= byte << 8;
            pololuControl_driveMotor(Payload.shoulderDest, 
                POLOLUCONTROL_SHOULDER);
            compRxState = elbowlo; // change state
            break;
        case elbowlo:
            Payload.elbowDest = byte;
            compRxState = elbowhi; // change state
            break;
        case elbowhi:
            Payload.elbowDest |= byte << 8;
            pololuControl_driveMotor(Payload.elbowDest,
                POLOLUCONTROL_ELBOW);
            compRxState = forearmlo; // change state
            break;
        case forearmlo:
            Payload.forearmDest = byte;
            compRxState = forearmhi; // change state
            break;
        case forearmhi:
            Payload.forearmDest |= byte << 8;
            pololuControl_driveMotor(Payload.forearmDest,
                POLOLUCONTROL_FOREARM);
            compRxState = wristtiltlo; // change state
            break;
        case wristtiltlo:
            Payload.wristTiltDest = byte;
            compRxState = wristtilthi; // change state
            break;
        case wristtilthi:
            Payload.wristTiltDest |= byte << 8;
            // TODO: call dynamixel command
            //wristGoalPosition(WRIST_TILT_ID, Payload.wristTiltDest);
            compRxState = wristspinlo; // change state
            break;
        case wristspinlo:
            Payload.wristSpinDest = byte;
            compRxState = wristspinhi; // change state
            break;
        case wristspinhi:
            Payload.wristSpinDest |= byte << 8;
            // TODO: call dynamixel command
            //wristGoalPosition(WRIST_ROTATE_ID, Payload.wristTiltDest);
            compRxState = handlo; // change state
            break;
        case handlo:
            oldHandDest = Payload.handDest; // store old hand value
            Payload.handDest = byte; // get new hand value
            compRxState = handhi; // change state
            break;
        case handhi:
            Payload.handDest |= byte << 8; // finish getting new hand value
            driveHand(Payload.handDest);
            // send hand state to slave PSoC
            uint16_t tmpHand = Payload.handDest;
            if (tmpHand > oldHandDest) {
                handMsg(close); // close hand
            }
            else if (tmpHand < oldHandDest) {
                handMsg(open); // open hand
            }
            else {
                handMsg(halt); // don't move hand
            }
            compRxState = chutes; // change state
            break;
        case chutes:
            // the chute values are packed into the first 6 bits of the byte
            PSoC_Slave_Payload.chuteSelect = byte & 0x3f;
            
            // box lid open/close is 7th bit
            if ((byte >> 6) & 0x01) {
                PWM_BoxLid_WriteCompare(SERVO_MIN); // open box
            }
            else {
                PWM_BoxLid_WriteCompare(SERVO_MAX); // close box
            }
            compRxState = shovello; // change state
            break;
        case shovello:
            Payload.shovel = byte;
            compRxState = shovelhi; // change state
            break;
        case shovelhi:
            Payload.shovel |= (byte << 8);
            PWM_Excavator_WriteCompare(Payload.shovel);
            compRxState = pre0; // change state 
            break;
        default:
            // shouldn't get here!!!
            break;
        }
    }
    
    // Check if any data came in that we didn't get. If so, then queue up
    // this event again in the events variable.
    if (UART_Computer_GetRxBufferSize() || 
        UART_Computer_ReadRxStatus() & UART_Computer_RX_STS_FIFO_NOTEMPTY) 
    {
        events |= COMP_RX_EVENT;
    }
    return SUCCESS; // success
}

// Drives hand to correct position: open, close, or middle
void driveHand(uint16_t pos) {
    if (pos < SERVO_MIN) {
        pos = SERVO_MIN;
    }
    else if (pos > SERVO_MAX) {
        pos = SERVO_MAX;
    }
    PWM_Hand_WriteCompare(pos);
}

void scienceEventHandler() {
    // Get feedback from science sensors: temperature and humidity
    enum states_e { pre0, pre1, templo, temphi, humlo, humhi };
    static enum states_e state = templo;
    
    // Read until finished getting all bytes in buffer
    while (UART_ScienceMCU_GetRxBufferSize() || 
          (UART_ScienceMCU_ReadRxStatus() & UART_ScienceMCU_RX_STS_FIFO_NOTEMPTY))
    {
        // Get next byte from UART
        int16_t byte;
        byte = UART_ScienceMCU_GetByte();
        if (byte & 0xff00) {
            return; // Error - ignore byte
        }
        static int16_t temp = 0; // temporary data storage
        
        switch (state) {
        // Preamble:
        case pre0:
            if (byte == 0xff) {
                state = pre1;
            }
            break;
        // Preamble 2nd byte:
        case pre1:
            if (byte == 0x9e) {
                state = templo;
            }
            else {
                state = pre0;
            }
            break;
        // Temperature low byte
        case templo:
            temp = byte;
            state = temphi;
            break;
        // Temperature high byte
        case temphi:
            temp |= 0xff00 & (byte << 8);
            temperature = temp; // now assign temperature to this value
            state = humlo;
            break;
        // Humidity low byte
        case humlo:
            temp = byte;
            state = humhi;
            break;
        // Humidity high byte
        case humhi:
            temp |= 0xff00 & (byte << 8);
            humidity = temp;
            state = templo;
            break;
        // Shouldn't ever get here
        default:
            break;
        }
    }
}

// Report current positions and ask the pololus for updated positions
void heartbeatEventHandler() {
    
    #if DEBUG_MODE
    //generateScienceTestData(); // use this to generate fake science data
    feedbackToTerminal(); // use this to see output on a terminal
    #else
    feedbackToOnboardComputer(); // use this to send to on-board computer
    #endif

    // Ask Arduino for science sensor data
    UART_ScienceMCU_PutChar(0xae);
    UART_ScienceMCU_PutChar(1);
    
    // Get Arm feedback:
    // Turret
    pololuControl_readVariable(POLOLUCONTROL_READ_FEEDBACK_COMMAND,
		POLOLUCONTROL_TURRET);
    
    // Shoulder
    pololuControl_readVariable(POLOLUCONTROL_READ_FEEDBACK_COMMAND,
		POLOLUCONTROL_SHOULDER);
    
    // Elbow
    pololuControl_readVariable(POLOLUCONTROL_READ_FEEDBACK_COMMAND,
		POLOLUCONTROL_ELBOW);
    
    // Forearm
    pololuControl_readVariable(POLOLUCONTROL_READ_FEEDBACK_COMMAND,
		POLOLUCONTROL_FOREARM);
    
    // Send command to PSoC slave
    psocSlaveCmd();
}

// Update turret position
void updateTurretPos() {
    static enum states_e {low, high} state = low;
    
    static uint16_t temp = 0;
    while(UART_Turret_ReadRxStatus() & UART_Turret_RX_STS_FIFO_NOTEMPTY) {
        switch(state) {
            case low:
                temp = UART_Turret_GetByte() & 0xff;
                state = high;
            break;
            case high:
                temp |= (UART_Turret_GetByte() << 8) & 0xff00;
                turretPos = temp;
                state = low;
            break;
            default:
                state = low;
            break;
        }
    }
}

// Update shoulder position
void updateShoulderPos() {
    static enum states_e {low, high} state = low;
    
    static uint16_t temp;
    while(UART_Shoulder_ReadRxStatus() & UART_Shoulder_RX_STS_FIFO_NOTEMPTY) {
        switch(state) {
            case low:
                temp = UART_Shoulder_GetByte() & 0xff;
                state = high;
            break;
            case high:
                temp |= (UART_Shoulder_GetByte() << 8) & 0xff00;
                shoulderPos = temp;
                state = low;
            break;
            default:
                state = low;
            break;
        }
    }
}

// Update elbow position
void updateElbowPos() {
	static enum states_e {low, high} state = low;
    
    static uint16_t temp;
    while(UART_Elbow_ReadRxStatus() & UART_Elbow_RX_STS_FIFO_NOTEMPTY) {
        switch(state) {
            case low:
                temp = UART_Elbow_GetByte() & 0xff;
                state = high;
            break;
            case high:
                temp |= (UART_Elbow_GetByte() << 8) & 0xff00;
                elbowPos = temp;
                state = low;
            break;
            default:
                state = low;
            break;
        }
    }
}

// Update forearm position
void updateForearmPos() {
	static enum states_e {low, high} state = low;
    
    static uint16_t temp;
    while(UART_Forearm_ReadRxStatus() & UART_Forearm_RX_STS_FIFO_NOTEMPTY) {
        switch(state) {
            case low:
                temp = UART_Forearm_GetByte() & 0xff;
                state = high;
            break;
            case high:
                temp |= (UART_Forearm_GetByte() << 8) & 0xff00;
                forearmPos = temp;
                state = low;
            break;
            default:
                state = low;
            break;
        }
    }
}

// ===========================================================================
// Helper and debug function definitions
// ===========================================================================

// Send feedback to computer
static void feedbackToOnboardComputer() {
    feedbackArray[0] = 0xE3; // start byte;
    feedbackArray[1] =  (turretPos & 0xff);
    feedbackArray[2] = ((turretPos >> 8) & 0xff);
    feedbackArray[3] =  (shoulderPos & 0xff);
    feedbackArray[4] = ((shoulderPos >> 8) & 0xff);
    feedbackArray[5] =  (elbowPos & 0xff);
    feedbackArray[6] = ((elbowPos  >> 8) & 0xff);
    feedbackArray[7] =  (forearmPos & 0xff);
    feedbackArray[8] = ((forearmPos >> 8) & 0xff);
	feedbackArray[9] = ((temperature & 0xff));
	feedbackArray[10] = ((temperature >> 8) & 0xff);
	feedbackArray[11] =((humidity & 0xff));
	feedbackArray[12] =((humidity >> 8) & 0xff);
	UART_Computer_PutArray(feedbackArray, POSITION_PAYLOAD_SIZE);
}

// A debugging function to see output on a terminal
static void feedbackToTerminal() {
    //static int i = 0;
    //i++;
    //turretPos += i;
    //shoulderPos += 2*i;
    //elbowPos += 3*i;
    //forearmPos += 4*i;
    //temperature = 5*i;
    //humidity = 6*i;
    
    char pos[34];
    sprintf(pos, "\n\r\n\rpositions:%4d,%4d,%4d,%4d", 
        turretPos, shoulderPos, elbowPos, forearmPos);
    pos[33] = 0; // null terminate
    char tem[20];
    sprintf(tem, "%d", temperature);
    tem[19] = 0; // null terminate
    char hum[20];
    sprintf(hum, "%d", humidity);
    hum[19] = 0; // null terminate
    UART_Computer_PutString(pos);
    UART_Computer_PutString("\n\rtemp:");
    UART_Computer_PutString(tem);
    UART_Computer_PutString("\n\rhumid:");
    UART_Computer_PutString(hum);
}
// Sends pretend data out on science uart
static void generateScienceTestData() {
    static uint16_t hum = 0;
    static uint16_t temp = 0;
    hum++;
    temp--;
    static uint8_t array[6];
    array[0] = 0xff;
    array[1] = 0xe9;
    array[2] = (uint8_t)(temp & 0xff);
    array[3] = (uint8_t)(temp >> 8) & 0xff;
    array[4] = (uint8_t)(hum & 0xff);
    array[5] = (uint8_t)(hum >> 8) & 0xff;
    UART_ScienceMCU_PutArray(array, 6);
}

static void psocSlaveCmd() {
    static uint8_t cmd[5];
    cmd[0] = 0xd8; // preamble first byte
    cmd[1] = 0xc4; // preamble second byte
    cmd[2] = PSoC_Slave_Payload.hand;
    cmd[3] = PSoC_Slave_Payload.camSelect;
    cmd[4] = PSoC_Slave_Payload.chuteSelect;
    UART_PSoC_Slave_PutArray(cmd, 5);
}

// Update hand position in payload
// enum hand_e { open, close, halt };
// hand byte: xxxx x|en|a|b
// x: unused
// en: enable the h-bridge
// a: close
// b: open
static void handMsg(enum hand_e handPos) {
    switch(handPos) {
    case open:
        PSoC_Slave_Payload.hand = 0x05;
        break;
    case close:
        PSoC_Slave_Payload.hand = 0x06;
        break;
    case halt:
        PSoC_Slave_Payload.hand = 0x00;
        break;
    default:
        PSoC_Slave_Payload.hand = 0x00;
        break;
    }
}

/* [] END OF FILE */
