/* ========================================
 * BYU Mars Rover 2016
 * Authors: Marshall Garey, Rodolfo Alberto
 * ========================================
*/

/* ===========================================================================
* Arm controller architecture
* Event driven
* Required functionality:
*   Receive messages from the computer
*   Transmit messages to the computer
*   Command each arm motor
*   Receive feedback from each arm motor
* 
* Computer messages:
*   Receive from computer:
*     2-byte preface
*     Positions for each of the 7 joints (2 bytes per position) - turret,
*       shoulder, elbow, forearm, wristtilt, wristspin, hand (close/open)
*     
*   Feedback (transmit to computer):
*     1-byte preface
*     Current positions of each of the 6 arm motors (not the hand)
*
* Events:
*   Receive message from the computer. This comes less often than the heartbeat
*   event.
*     Actuates the motors as we get the values
*   Heartbeat - 5 - 10 times per second.
*     Queue getting current feedback positions from motors
*     Lock the computer receive message event.
*   Position
*     Queued when all feedback positions have been received
*     Sends feedback positions to onboard computer
*     Unlock the computer receive message event.
*
* Delta Sigma ADC:
* Differential - if Vref = 1.024 V, and Vn is 2.048 V, the input range is
*     -Vn +- Vref = 2.048 +- -1.024 or (1.024 - 3.096 V)
*     If I connect Vn to the same as Vref, the input range is 0 - 2*Vref
* Or I can tie Vn to ground and the input range would be -Vref - +Vref
* I can switch between multiple configurations of this ADC during runtime.
* This will be useful because I can have one ADC for both humidity and
* temperature measurements.
* I'd have to use a hardware mux to select which input to use.
* I can also set the input gain to 1, 2, 4, or 8.
* This ADC has filtering, so I may not need to filter separately.
* When ADC is in the differential mode, the full input range will always be 
* symmetrical around zero. When in single ended mode, the input range will be
* from just below Vssa to the full scale value.
*
* If I set the ADC to 16 bits, get the 32-bit result. If I use 8-bit ADC, get
* the 16-bit version. (This is to prevent under/overflow.)
*
* API:
* StartConvert(), StopConvert(), IRQ_Enable(), IRQ_Disable(), GetResult8/16/32,
* SelectConfiguration(uint8 config, uint8 restart)
*     config - 1, 2, 3, or 4
*     restart - 1 means start the ADC and restart the conversion.
*               0 means do not start the ADC and conversion.
* int16 CountsTo_mVolts - For example, if the ADC measured 0.534 V,
*     the return value would be 534 mV. The calculation of voltage depends
*     on the value of the voltage reference
* int32 CountsTo_uVolts
* float CountsTo_Volts
*
* ========================================================================= */
#include <project.h>
#include "isr.h"
#include "isrHandler.h"
#include "pololuControl.h"

#define TOGGLE_LED0 LED0_Write(!LED0_Read())
#define FALSE 0
#define TRUE 1

// Set to false if the board is in normal operation.
// Set to true when a timeout occurs and the motors are reset.
int resetMode = FALSE;

// the main event loop
void eventLoop();

// an automated test to control multiple pololu joints with just the PSoC
void multiJointTest();

// an automated test to open and close the hand.
void handTest();
void shovelTest();

// an automated test to actuate the solenoids to open/close the chute doors
void chuteTest();

// an automated test to move the gimbal (main camera pan and tilt)
void gimbalTest();

// a function that resets everything
void resetAll();

void init();

int main() {

    init();
    
    // loop - the while(1) here is just to make the compiler happy
    while(1) {
        //multiJointTest();
        //handTest();
        //shovelTest();
        //chuteTest();
        //gimbalTest();
        eventLoop();
    }
}

void eventLoop() {
    #define TIMEOUT 6
    static int timeout = 0;
    // Main loop
    while(1) {
        if (events) {
            // Receive message from computer
            if (events & COMP_RX_EVENT) {
                events &= ~COMP_RX_EVENT;
                TOGGLE_LED0;
                timeout = 0; // reset timeout counter
                resetMode = FALSE; // not in reset mode.
                compRxEventHandler();
            }
            
            // Update turret position
            else if (events & TURRET_POS_EVENT) {
                events &= ~TURRET_POS_EVENT;
                updateTurretPos();
            }
            
            // Update shoulder position
            else if (events & SHOULDER_POS_EVENT) {
                events &= ~SHOULDER_POS_EVENT;
                updateShoulderPos();
            }
            
            // Update elbow position
            else if (events & ELBOW_POS_EVENT) {
                events &= ~ELBOW_POS_EVENT;
                updateElbowPos();
            }
            
            // Update forearm position
            else if (events & FOREARM_POS_EVENT) {
                events &= ~FOREARM_POS_EVENT;
                updateForearmPos();
            }
            
            // Get science sensor data
            else if (events & SCIENCE_EVENT){
                events &= ~SCIENCE_EVENT;
                scienceEventHandler();
            }
            
            // Heartbeat event - occurs every 100 ms
            else if (events & HEARTBEAT_EVENT) {
                events &= ~HEARTBEAT_EVENT;
                
                // If timeout reaches TIMEOUT, stop all motors with resetAll()
                timeout++;
                if (timeout >= TIMEOUT && resetMode == FALSE) {
                    resetMode = TRUE;
                    resetAll();
                }
                
                // Process heartbeat event
                heartbeatEventHandler();
            }
            
            // Invalid event
            else {
                // TODO: manage invalid event
            }
        }
    }
}

// turns off arm motors and resets servos to neutral position.
void resetAll() {
    
    PWM_Gimbal_WriteCompare1(SERVO_NEUTRAL);
    PWM_Gimbal_WriteCompare2(SERVO_NEUTRAL);
    PWM_Drive_WriteCompare1(SERVO_NEUTRAL);
    PWM_Drive_WriteCompare2(SERVO_NEUTRAL);
    
    pololuControl_turnMotorOff(POLOLUCONTROL_TURRET);
    pololuControl_turnMotorOff(POLOLUCONTROL_SHOULDER);
    pololuControl_turnMotorOff(POLOLUCONTROL_FOREARM);
    pololuControl_turnMotorOff(POLOLUCONTROL_ELBOW);
    
    PWM_Hand_WriteCompare(SERVO_NEUTRAL);
    PWM_Excavator_WriteCompare(SERVO_NEUTRAL);
    
    LED0_Write(1);
}

// automated test that moves 4 arm joints (all controlled with
// pololu PID boards)
void multiJointTest() {
    int i;
    uint16_t shoulder = 2048;
    uint16_t turret = 2048;
    uint16_t elbow = 2048;
    uint16_t forearm = 2048;
    uint16_t target = 2048;

    while (1) {
        // back to neutral
        TOGGLE_LED0;
        target = 2048;
        turret = shoulder = elbow = forearm = target;
        pololuControl_driveMotor(turret, POLOLUCONTROL_TURRET);
        pololuControl_driveMotor(shoulder, POLOLUCONTROL_SHOULDER);
        pololuControl_driveMotor(elbow, POLOLUCONTROL_ELBOW);
        pololuControl_driveMotor(forearm, POLOLUCONTROL_FOREARM);
        CyDelay(5000);
        
        // slowly move forward
        for (i = 0; i < 5; i++) {
            TOGGLE_LED0;
            target += 200;
            turret = shoulder = elbow = forearm = target;
            pololuControl_driveMotor(turret, POLOLUCONTROL_TURRET);
            pololuControl_driveMotor(shoulder, POLOLUCONTROL_SHOULDER);
            pololuControl_driveMotor(elbow, POLOLUCONTROL_ELBOW);
            pololuControl_driveMotor(forearm, POLOLUCONTROL_FOREARM);
            CyDelay(5000);
        }
        
        // back to neutral
        TOGGLE_LED0;
        target = 2048;
        turret = shoulder = elbow = forearm = target;
        pololuControl_driveMotor(turret, POLOLUCONTROL_TURRET);
        pololuControl_driveMotor(shoulder, POLOLUCONTROL_SHOULDER);
        pololuControl_driveMotor(elbow, POLOLUCONTROL_ELBOW);
        pololuControl_driveMotor(forearm, POLOLUCONTROL_FOREARM);
        CyDelay(5000);    
        
        // slowly move backward
        for (i = 0; i < 5; i++) {
            TOGGLE_LED0;
            target -= 200;
            turret = shoulder = elbow = forearm = target;
            pololuControl_driveMotor(turret, POLOLUCONTROL_TURRET);
            pololuControl_driveMotor(shoulder, POLOLUCONTROL_SHOULDER);
            pololuControl_driveMotor(elbow, POLOLUCONTROL_ELBOW);
            pololuControl_driveMotor(forearm, POLOLUCONTROL_FOREARM);
            CyDelay(5000);           
        }
    } 
}

// automated test that opens and closes the hand
void handTest() {
    
    while(1) {
        //PWM_Hand_WriteCompare1(SERVO_MAX);
        PWM_Hand_WriteCompare(SERVO_NEUTRAL);
        TOGGLE_LED0;
        CyDelay(4000);
        PWM_Hand_WriteCompare(SERVO_MIN);
        TOGGLE_LED0;
        CyDelay(4000);
        PWM_Hand_WriteCompare(1950);
        TOGGLE_LED0;
        CyDelay(30000);
    }
}

// automated test that opens and closes the hand
void shovelTest() {
    
    while(1) {
        PWM_Excavator_WriteCompare(SERVO_MAX);
        TOGGLE_LED0;         
        CyDelay(4000);       
        PWM_Excavator_WriteCompare(SERVO_NEUTRAL);
        TOGGLE_LED0;         
        CyDelay(4000);       
        PWM_Excavator_WriteCompare(SERVO_MIN);
        TOGGLE_LED0;         
        CyDelay(4000);       
        PWM_Excavator_WriteCompare(SERVO_NEUTRAL);
        TOGGLE_LED0;
        CyDelay(4000);
    }
}

// an automated test to move the gimbal (main camera pan and tilt)
void gimbalTest() {
    while (1) {
        PWM_Gimbal_WriteCompare1(SERVO_NEUTRAL);
        PWM_Gimbal_WriteCompare2(SERVO_NEUTRAL);
        TOGGLE_LED0;
        CyDelay(4000);
        PWM_Gimbal_WriteCompare1(SERVO_MIN);
        PWM_Gimbal_WriteCompare2(SERVO_MIN);
        TOGGLE_LED0;
        CyDelay(4000);
        PWM_Gimbal_WriteCompare1(SERVO_MAX);
        PWM_Gimbal_WriteCompare2(SERVO_MAX);
        TOGGLE_LED0;
        CyDelay(4000);
    }
}

void init() {
        // 5 second delay before we start everything up
    CyDelay(5000);
    
    // Initialize variables
    events = 0; // no pending events initially
    LED0_Write(0); // LED off, turns on when we're done initializing
    resetMode = FALSE;
    
    // Enable global interrupts
    CyGlobalIntEnable;
    
    // Initialize and start hardware components:
    
    // computer uart
    UART_Computer_Start();
    Comp_RX_ISR_StartEx(CompRxISR);
    
    // drive
    Clock_PWM_Start();
    PWM_Drive_Start();
    PWM_Drive_WriteCompare1(SERVO_NEUTRAL);
    PWM_Drive_WriteCompare2(SERVO_NEUTRAL);
    
    // gimbal (main camera pan/tilt)
    PWM_Gimbal_Start();
    PWM_Gimbal_WriteCompare1(SERVO_NEUTRAL);
    PWM_Gimbal_WriteCompare2(SERVO_NEUTRAL);
    
    // clock for all other uart modules
    UARTClk_Start();
    
    // turret
    UART_Turret_Start();
    TurretRxIsr_StartEx(TurretRxISR);
    pololuControl_turnMotorOff(POLOLUCONTROL_TURRET);
    
    // shoulder uart
    UART_Shoulder_Start();
    ShoulderRxIsr_StartEx(ShoulderRxISR);
    pololuControl_turnMotorOff(POLOLUCONTROL_SHOULDER);
    
    // elbow uart
    UART_Elbow_Start();
    ElbowRxIsr_StartEx(ElbowRxISR);
    pololuControl_turnMotorOff(POLOLUCONTROL_ELBOW);
    
    // forearm uart
    UART_Forearm_Start();
    ForearmRxIsr_StartEx(ForearmRxISR);
    pololuControl_turnMotorOff(POLOLUCONTROL_FOREARM);
    
    // science uart
    UART_ScienceMCU_Start();
    ScienceRxIsr_StartEx(ScienceRxISR);
    
    //Initialize the dynamixels
//    wristSpeed(0xFE, 300); // also only do once
//    setWristTorque(0xFE, 0x03FF); // maximum
//    wristGoalPosition(WRIST_TILT_ID, 2048);
//    wristGoalPosition(WRIST_ROTATE_ID, 2048);
    
    // hand pwm (also the heartbeat timer).
    PWM_Hand_Start();
    PWM_Hand_WriteCompare(SERVO_NEUTRAL); // hand/temperature probe
    PWM_Excavator_WriteCompare(SERVO_NEUTRAL); // excavator
    heartbeatIsr_StartEx(HeartbeatISR);
    
    // sample box pwm
    PWM_BoxLid_Start();
    PWM_BoxLid_WriteCompare(SERVO_NEUTRAL);
    
    LED0_Write(1); // done initializing
}

/* [] END OF FILE */
