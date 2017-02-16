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

// Container for all events. See isrHandler.h for macros that define the events.
volatile uint32_t events = 0;

// Arm payload struct
// These are the last positions we wrote to the motors
static struct Payload {
    uint16_t leftWheels;
    //uint16_t leftWheelsDir;
    uint16_t rightWheels;
    //uint16_t rightWheelsDir;
    uint16_t cameraPan;		//Remove
    uint16_t cameraTilt;  	//Remove
    uint8_t cameraNum;		//Remove
    uint8_t chutes;
    uint16_t turretDest;
    uint16_t shoulderDest;
    uint16_t elbowDest;
    uint16_t forearmDest;
    uint16_t wristTiltDest;	//Remove???
    uint16_t wristSpinDest;	//Remove???
    uint8_t handDest;
    uint8_t miscSwitches; // byte: xxxx x laser | electromagnet | dynamixels on/off //Remove???
    uint16_t shovel;		//Remove???
} Payload;

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

#define POSITION_PAYLOAD_SIZE (13) // 1 start byte, 2 bytes per joint, 6 joints //PROBABLY CHANGE TO 9, remove all wrist data
// The positions are stored in little endian format - low byte first, then
// high byte, in order from joints closest to the rover outward
// [turretlo, turrethi, shoulderlo, shoulderhi, elbowlo, elbowhi,
// forearmlo, forearmhi, wristspinlo, wristspinhi, wristtiltlo, wristtilthi]
static uint8_t feedbackArray[POSITION_PAYLOAD_SIZE];

// State machine states to receive commands from computer
// The state machine is defined in the function compRxEventHandler
#define PREAMBLE0 0xEA
static enum compRxStates_e { pre0, leftlo, lefthi, rightlo, righthi, campanlo, //Remove the following states: campanlo/hi camtiltlo/hi, camSelect
    campanhi, camtiltlo, camtilthi, camSelect, turretlo, turrethi, 		//Wristtilt/spin/lo/hi, miscSwitch???, shovello/hi
    shoulderlo, shoulderhi, elbowlo, elbowhi, forearmlo, forearmhi,		//Add leftDirlo/hi, rightDirlo/hi
    wristtiltlo, wristtilthi, wristspinlo, wristspinhi, 
    handlo, miscSwitch, chutes, shovello, shovelhi } compRxState;

// Receive a message from the computer
int compRxEventHandler() {
    // get next element in uart rx buffer
    static uint16_t data;
    static uint8_t byte;
    
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
            
            // Check preamble byte
        case pre0:
            if (byte == PREAMBLE0) {
                compRxState = leftlo; // change state
            }
            break;
            
            // Actuate wheels - left and right
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
            
            // Actuate pan/tilt camera
        case campanlo:
            Payload.cameraPan = byte;
            compRxState = campanhi; // change state
            break;
        case campanhi:
            Payload.cameraPan |= byte << 8;
            compRxState = camtiltlo;
            PWM_PanTilt_WriteCompare1(Payload.cameraPan);
            compRxState = camtiltlo; // change state
            break;
        case camtiltlo:
            Payload.cameraTilt = byte;
            compRxState = camtilthi; // change state
            break;
        case camtilthi:
            Payload.cameraTilt |= byte << 8;
            PWM_PanTilt_WriteCompare2(Payload.cameraTilt);
            compRxState = camSelect; // change state
            break;
            
            // update camera feed
        case camSelect:
            selectCameras(byte);
            compRxState = turretlo; // change state
            break;
            
            // Actuate first 4 arm joints: turret, shoulder, elbow, forearm
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
            
            // Do nothing with wrist data
        case wristtiltlo:
            compRxState = wristtilthi; // change state
            break;
        case wristtilthi:
            compRxState = wristspinlo; // change state
            break;
        case wristspinlo:
            compRxState = wristspinhi; // change state
            break;
        case wristspinhi:
            compRxState = handlo; // change state
            break;
            
            // actuate hand
        case handlo:
            Payload.handDest = byte; // get new hand value
            driveHand(Payload.handDest); // update hand position
            compRxState = miscSwitch; // change state
            break;
            
            // byte: xxxx x laser | electromagnet | dynamixels
        case miscSwitch:
            Payload.miscSwitches = byte;
            electromagnet_Write((byte & 2) >> 1);
            compRxState = chutes; // change state
            break;
            
            // actuate chutes
        case chutes:
            // byte: box open/close | chute_en | c6 | c5 | c4 | c3 | c2 | c1
            if (byte & 0x40) {
                //chute_en_Write(1);
                control_chutes(byte);
            }
            else {
                //chute_en_Write(0);
            }
            
            // box lid open/close is 8th bit
            if (byte & 0x80) {
                PWM_BoxLid_WriteCompare(SERVO_MIN); // open box
            }
            else {
                PWM_BoxLid_WriteCompare(SERVO_MAX); // close box
            }
            compRxState = shovello; // change state
            break;
            
            // actuate shovel
        case shovello:
            Payload.shovel = byte;
            compRxState = shovelhi; // change state
            break;
        case shovelhi:
            Payload.shovel |= (byte << 8);
            PWM_Excavator_WriteCompare(Payload.shovel);
            compRxState = pre0; // change state - return to beginning
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

void scienceEventHandler() {
    // Get feedback from science sensors: temperature and humidity
    enum states_e { pre0, pre1, templo, temphi, humlo, humhi };
    static enum states_e state = templo;
    static uint16_t temp = 0; // temporary data storage
    
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
            if (temp <= 100) {
                temperature = temp; // now assign temperature to this value
            }
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
            if (humidity <= 1023) {
                humidity = temp; // now assign humidity to this value
            }
            state = pre0;
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
    UART_ScienceMCU_PutChar(0xae); // preamble
    UART_ScienceMCU_PutChar(1); // get feedback
    if (Payload.miscSwitches & 1) { // if this is set, turn dynamixels off
        UART_ScienceMCU_PutChar(5); // command to turn off dynamixels
    }
    else if (Payload.miscSwitches & 4) { // fire laser?
        UART_ScienceMCU_PutChar(2);
    }
    else {
        UART_ScienceMCU_PutChar(10); // do nothing
    }
    
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
}

// ===========================================================================
// Helper and debug function definitions
// ===========================================================================

// Control hand
void driveHand(uint16_t pos) {
    if (pos == 1) { // open (retract linear actuators)
        hand_a_Write(1);
        hand_b_Write(0);
    }
    else if (pos == 2) { // close (extend linear actuators)
        hand_b_Write(1);
        hand_a_Write(0);
    }
    else { // don't move
        hand_a_Write(0);
        hand_b_Write(0);
    }
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
                if (temp <= 4095) {
                    turretPos = temp;
                }
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
                if (temp <= 4095) {
                    temp |= (UART_Shoulder_GetByte() << 8) & 0xff00;
                }
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
                if (temp <= 4095) {
                    temp |= (UART_Elbow_GetByte() << 8) & 0xff00;
                }
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
                if (temp <= 4095) {
                    temp |= (UART_Forearm_GetByte() << 8) & 0xff00;
                }
                forearmPos = temp;
                state = low;
            break;
            default:
                state = low;
            break;
        }
    }
}

//Remove!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void selectCameras(uint8_t byte) {
    // byte: rc_en | x | v2[1:0] | x | x | v1[1:0]
    // rc_en is rc camera enable
    // v2 is second video feed
    // v1 is first video feed
    rc_cam_en_Write((byte & 0x80) >> 7);
    
    uint8_t v1 = byte & 0x0f;
    uint8_t v2 = (byte & 0xf0) >> 4;
    switch(v1) {
        case 0:
            PWM_Video_WriteCompare1(VIDEO1);
            break;
        case 1:
            PWM_Video_WriteCompare1(VIDEO2);
            break;
        case 2:
            PWM_Video_WriteCompare1(VIDEO3);
            break;
        default:
            PWM_Video_WriteCompare1(VIDEO1);
            break;
    }
    switch(v2) {
        case 0:
            PWM_Video_WriteCompare2(VIDEO1);
            break;
        case 1:
            PWM_Video_WriteCompare2(VIDEO2);
            break;
        case 2:
            PWM_Video_WriteCompare2(VIDEO3);
            break;
        default:
            PWM_Video_WriteCompare2(VIDEO1);
            break;
    }
}

void control_chutes(uint8_t byte) {
    // chutes 1-6 are bits 0-5;
    
    // chute 1
    if (byte & 0x1) // close
    {
        chute1b_Write(0);
        chute1a_Write(1);
    }
    else // open
    {
        chute1a_Write(0);
        chute1b_Write(1);
    }
    // chute 2
    if (byte & 0x2) // close
    {
        chute2b_Write(0);
        chute2a_Write(1);
    }
    else // open
    {
        chute2a_Write(0);
        chute2b_Write(1);
    }
    // chute 3
    if (byte & 0x4) // close
    {
        chute3b_Write(0);
        chute3a_Write(1);
    }
    else // open
    {
        chute3a_Write(0);
        chute3b_Write(1);
    }
    // chute 4
    if (byte & 0x8) // close
    {
        chute4b_Write(0);
        chute4a_Write(1);
    }
    else // open
    {
        chute4a_Write(0);
        chute4b_Write(1);
    }
	
	//Remove!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // chute 5
    if (byte & 0x10) // close
    {
        chute5b_Write(0);
        chute5a_Write(1);
    }
    else // open
    {
        chute5a_Write(0);
        chute5b_Write(1);
    }
	//Remove!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // chute 6
    if (byte & 0x20) // close
    {
        chute6b_Write(0);
        chute6a_Write(1);
    }
    else // open
    {
        chute6a_Write(0);
        chute6b_Write(1);
    }
}

// Send feedback to computer
static void feedbackToOnboardComputer() {
    feedbackArray[0] = 0xE3; // start byte;
    feedbackArray[1] = (turretPos & 0xff);
    feedbackArray[2] = ((turretPos >> 8) & 0xff);
    feedbackArray[3] = (shoulderPos & 0xff);
    feedbackArray[4] = ((shoulderPos >> 8) & 0xff);
    feedbackArray[5] = (elbowPos & 0xff);
    feedbackArray[6] = ((elbowPos  >> 8) & 0xff);
    feedbackArray[7] = (forearmPos & 0xff);
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

/* [] END OF FILE */
