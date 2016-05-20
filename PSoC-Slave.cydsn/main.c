/* ========================================
 * Author: Marshall Garey
 * Date Created: May 10, 2016
 * ========================================
*/
#include <project.h>

// ===========================================================================
// Macros
// ===========================================================================
#define LED1_ON LED1_Write(1)
#define LED1_OFF LED1_Write(0)
#define LED1_TOGGLE LED1_Write(!LED1_Read())

// ===========================================================================
// Constant defines
// ===========================================================================
#define RX_EVENT 0x01
#define HEARTBEAT_EVENT 0x08

#define VIDEO1 1100 // 0.9 - 1.35 ms
#define VIDEO2 1500 // 1.35 - 1.75 ms
#define VIDEO3 1900 // 1.75 - 2.1 ms

// ===========================================================================
// Global variables
// ===========================================================================
static uint16_t sys_event = 0;
static enum { pre1, pre2, hand, cam_sel, chutes } state = pre1;

// ===========================================================================
// Function prototypes
// ===========================================================================

void event_loop();
void rx_event_handler();
void init();
void update_hand(uint8_t byte);
void select_camera(uint8_t byte);
void control_chutes(uint8_t byte);
void pwm_cam_test();
CY_ISR_PROTO(uart_rx_isr);
CY_ISR_PROTO(heartbeat_isr);

// ===========================================================================
// Function definitions
// ===========================================================================
int main()
{
    init();
    // event loop
    while(1)
    {
        event_loop();
        //pwm_cam_test();
        //UART1_PutChar('d');
        //UART1_PutChar('\n');
        //LED1_TOGGLE;
        //CyDelay(1000);
    }
}

void event_loop()
{
    while(1)
    {
        if (sys_event)
        {
            if (sys_event & RX_EVENT)
            {
                sys_event &= ~RX_EVENT; // clear event
                rx_event_handler(); // call event handler
                //LED1_TOGGLE;
            }
            else if (sys_event & HEARTBEAT_EVENT)
            {
                sys_event &= ~HEARTBEAT_EVENT;
                //LED1_ON;
            }
        }
    }
}

void init()
{
    LED1_OFF;
    CyDelay(5000);
    // init variables
    sys_event = 0; // no pending events
    state = pre1;
    
    // init camera muxes
    CamMux1_Start();
    CamMux1_Select(0);
    CamMux2_Start();
    CamMux2_Select(0);
    
    // start pwm
    Clock_2_Start();
    PWM_Video_Start();
    
    // start uart
    UART1_Start();
    rxisr_StartEx(uart_rx_isr);
    
    // start heartbeat timer
    Heartbeat_Clock_Start();
    Heartbeat_Counter_Start();
    isr_1_StartEx(heartbeat_isr);
    
    CyGlobalIntEnable; /* Enable global interrupts. */
    LED1_ON;
}

/* This is the packet we're receiving:
static void psocSlaveCmd() {
    static uint8_t cmd[5];
    cmd[0] = 0xd8; // preamble first byte
    cmd[1] = 0xc4; // preamble second byte
    cmd[2] = PSoC_Slave_Payload.hand;
    cmd[3] = PSoC_Slave_Payload.camSelect;
    cmd[4] = PSoC_Slave_Payload.chuteSelect;
    UART_PSoC_Slave_PutArray(cmd, 5);
}
*/
void rx_event_handler()
{
    // Continue in this loop while bytes are available
    while(UART1_GetRxBufferSize() ||
        (UART1_ReadRxStatus() & UART1_RX_STS_FIFO_NOTEMPTY))
    {
		uint16_t data = UART1_GetByte();
        // DEBUG: echo character
        UART1_PutChar((uint8_t)(data & 0xff));
		// Error in upper byte
		if (data & 0xff00)
		{
			return; // error
		}
        uint8_t byte = data & 0xff;
        
        switch(state)
        {
        case pre1:
            if (byte == 0xd8)
                state = pre2;
            break;
        case pre2:
            if (byte == 0xc4)
                state = hand;
            else
                state = pre1;
            break;
        case hand:
            update_hand(byte);
            state = cam_sel;
            break;
        case cam_sel:
            select_camera(byte);
            state = chutes;
            break;
        case chutes:
            control_chutes(byte);
            state = pre1;
            break;
        default:
            break;
        }
    }
}

void update_hand(uint8_t byte)
{
    // hand byte: xxxx x|en|a|b
    // x: unused
    // en: enable the h-bridge
    // a: close
    // b: open
}

void select_camera(uint8_t byte)
{
    // low nybble: camera 1; high nybble: camera 2
    CamMux1_Select(byte & 0x0f);
    CamMux2_Select((byte & 0xf0) >> 4);
    
    uint8_t v1 = byte & 0x0f;
    uint8_t v2 = (byte & 0xf0) >> 4;
    switch(v1)
    {
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
    switch(v2)
    {
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

void control_chutes(uint8_t byte)
{

}

// Received a character from main PSoC
CY_ISR(uart_rx_isr)
{
    //UART1_ReadRxStatus(); // clear interrupt
    UART1_GetRxInterruptSource();
    rxisr_ClearPending();
    LED1_TOGGLE;
    
    sys_event |= RX_EVENT; // queue event
}

CY_ISR(heartbeat_isr)
{
    Heartbeat_Counter_ReadStatusRegister(); // clear interrupt
    isr_1_ClearPending();
    
    sys_event |= HEARTBEAT_EVENT; // queue event
}

void pwm_cam_test()
{
    PWM_Video_WriteCompare1(VIDEO1);
    PWM_Video_WriteCompare2(VIDEO1);
    LED1_TOGGLE;
    CyDelay(8000);
    
    PWM_Video_WriteCompare1(VIDEO2);
    PWM_Video_WriteCompare2(VIDEO2);
    LED1_TOGGLE;
    CyDelay(8000);
    
    PWM_Video_WriteCompare1(VIDEO3);
    PWM_Video_WriteCompare2(VIDEO3);
    LED1_TOGGLE;
    CyDelay(8000);
}

/* [] END OF FILE */
