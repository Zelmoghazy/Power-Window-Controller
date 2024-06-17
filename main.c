#include "FreeRTOS.h"
#include "TM4C123GH6PM.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include "math.h"
#include "stdint.h"
#include <stdint.h>
#include <stdio.h>

#define RED   0x02
#define BLUE  0x04
#define GREEN 0x08



#define FIFO_TX_FULL()                  ((UART0->FR & (1 << 5)) == 1)		// if bit 5 is 1 -> tx fifo is full
#define FIFO_RX_EMPTY()                 ((UART0->FR & (1 << 4)) == 1)		// if bit 4 is 1 -> rx fifo is empty

#define GPIO_READ_PIN(PORT,PIN)         (((GPIO##PORT->DATA)>>PIN)&1U)
#define GPIO_SET_PIN(PORT,PIN)          ((GPIO##PORT->DATA)|=1<<PIN)
#define GPIO_CLEAR_PIN(PORT,PIN)        ((GPIO##PORT->DATA)&=~(1<<PIN))
#define GPIO_TOGGLE_PIN(PORT,PIN)       ((GPIO##PORT->DATA)|=1<<PIN)

#define GPIO_READ_PORT(PORT)            ((GPIO##PORT->DATA))
#define GPIO_WRITE_PORT(PORT,VALUE)     ((GPIO##PORT->DATA)=VALUE)

#define READ_BIT(DATA,BIT)              (((DATA)>>BIT)&1U)

#define PASS_UP_PIN                     0
#define PASS_DOWN_PIN                   1
#define DRIVER_UP_PIN                   2
#define DRIVER_DOWN_PIN                 3
#define LIMIT_UP_PIN                    4
#define LIMIT_DOWN_PIN                  5
#define JAM_PIN                         6
#define LOCK_ON_PIN                     7


#define PASS_DOWN_PRESSED              !GPIO_READ_PIN(B, PASS_DOWN_PIN) 
#define PASS_UP_PRESSED                !GPIO_READ_PIN(B, PASS_UP_PIN)
#define DRIVER_UP_PRESSED              !GPIO_READ_PIN(B, DRIVER_UP_PIN)
#define DRIVER_DOWN_PRESSED            !GPIO_READ_PIN(B, DRIVER_DOWN_PIN)
#define JAM_PRESSED                    !GPIO_READ_PIN(B, JAM_PIN)
#define LOCK_ON_PRESSED                (!GPIO_READ_PIN(B, LOCK_ON_PIN))

#define NO_INPUT                       (GPIO_READ_PORT(B) == 0xFF)
#define PASS_DOWN_ONLY                 (GPIO_READ_PORT(B) == 0xFE)
#define PASS_UP_ONLY                   (GPIO_READ_PORT(B) == 0xFD)

#define DOWN_TIMER                      0
#define UP_TIMER                        1

#define mainONE_SHOT_TIMER_PERIOD       pdMS_TO_TICKS( 3333 )

static QueueHandle_t event_queue;
TimerHandle_t xOneShotTimer;

char buffer[256];

uint8_t debounce(uint8_t current);

void gpio_init(void);
void gpiof_handler(void);
void interrupt_config(void);

void uart_init(uint32_t baudrate);
void print_char(const char ch);
void print_string(const char *str);
void vPrintString(const char *pcString);
char read_char(void);

void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);
void vApplicationTickHook(void);

void Input_Handler_Task(void *pvParameters);
void Event_Handler_Task(void *pvParameters);

void Motor_Down(void);
void Motor_Up(void);

typedef enum {
    PASS_DOWN_BUTTON,
    PASS_UP_BUTTON,
    DRIVER_UP_BUTTON,
    DRIVER_DOWN_BUTTON,
    LIMIT_UP_BUTTON,
    LIMIT_DOWN_BUTTON,
    LOCK_ON,
    LOCK_OFF,
    JAM
} event_t;

typedef enum {
    NEUTRAL,
    PASS_DOWN,
    PASS_UP,
    DRIVER_DOWN,
    DRIVER_UP,
    EMERGENCY
} state_t;

uint8_t debounce(uint8_t current)
{
    static uint8_t asserted = 0x00;
    static uint8_t previous = 0x00;
    /*
    * Compare the current and previous states of each input.
    */
    asserted |= (previous & current); // Assert newly "on" inputs.
    asserted &= (previous | current); // Clear newly "off" inputs.
    /*
    * Update the history.
    */
    previous = current;
    /*
    * Return the debounced inputs to the caller.
    */
    return (asserted);
}

void uart_init(uint32_t baudrate) 
{
    /* Enable the UART module using the RCGCUART register */
    SYSCTL->RCGCUART |= (1 << 0);   // Enable and provide a clock to UART module 0 in Run mode

    /* Enable the clock to the appropriate GPIO module via the RCGCGPIO register */
    SYSCTL->RCGCGPIO |= (1 << 0);   // Enable and provide a clock to GPIO Port A in Run mode.
    while ((SYSCTL->PRGPIO & 0X1) == 0) ;

    /* Set the GPIO AFSEL bits for the appropriate pins */
    GPIOA->AFSEL = (1 << 1) | (1 << 0);   // Enable alternate function for Pin A0 and A1

    /* Configure the GPIO current level and/or slew rate as specified for the mode selected */

    /*
            - Configure the PMCn fields in the GPIOPCTL register to assign the
              UART signals to the appropriate pins

            - When a bit is set in the GPIOAFSEL register, the corresponding
              GPIO signal is controlled by an associated peripheral. The
              GPIOPCTL register selects one out of a set of peripheral functions for
              each GPIO, providing additional flexibility in signal definition
    */

    GPIOA->PCTL = (1 << 0) | (1 << 4);   // PMC0 and PMC1 -> enabling uart0 on PA0 and PA1

    GPIOA->DEN = (1 << 0) | (1 << 1);   // Enable digital functions for PA0 and PA1

    /*
            BRD = 50,000,000 / (16 * 115,200) = 27.126736
            UARTFBRD[DIVFRAC] = integer(0.126736 * 64 + 0.5) = 8
    */

    /* Disable the UART by clearing the UARTEN bit in the UARTCTL register. */
    UART0->CTL &= ~(1 << 0);   // UART Enable bit

    /* Write the integer portion of the BRD to the UARTIBRD register.*/

    SystemCoreClockUpdate();   // Update System Clock value

    /* 
        double modf(double value, double *iptr);
        - Splits value into integer and fractional parts; stores the integer part in the object pointed to by iptr.
        - Returns Fractional part of value
     */
    double integral;
    double fractional = modf((double) SystemCoreClock / (16 * baudrate), &integral);

    UART0->IBRD = (uint32_t) integral;

    /* Write the fractional portion of the BRD to the UARTFBRD register. */
    UART0->FBRD = (uint32_t) (fractional * 64 + 0.5);

    /* Write the desired serial parameters to the UARTLCRH register */
    UART0->LCRH = (0x3 << 5) | (1 << 4);   // 8-bit, no parity, 1-stop bit

    /* Configure the UART clock source by writing to the UARTCC register. */
    UART0->CC = 0x0;   // use system clock

    /* Enable the UART by setting the UARTEN bit in the UARTCTL register. */
    UART0->CTL = (1 << 0) | (1 << 8) | (1 << 9);   // enable uart, tx, rx
}

void print_char(const char ch) 
{
    while (FIFO_TX_FULL());   // Previous transmissions ended and FIFO is not full.
    UART0->DR = ch;
}

void print_string(const char *str) 
{
    while (*str) {
        print_char(*(str++));
    }
    print_char('\n');
}

char read_char(void) 
{
    char ch;
    while(FIFO_RX_EMPTY());   // Receiver fifo is not empty
    ch = UART0->DR;
    return ch;
}

void vPrintString(const char *pcString) 
{
    /* Print the string, suspending the scheduler as method of mutual exclusion. */
    taskENTER_CRITICAL();
    {
        print_string(pcString); 
    }
    taskEXIT_CRITICAL();
}

void gpio_init(void)   // Port F pin 4
{
    SYSCTL->RCGCGPIO |= 0X20;                // Enable clock to PORTF
    while ((SYSCTL->PRGPIO & 0X20) == 0) ;   // Wait until it is enabled

    GPIOF->LOCK = 0X4C4F434B;       // Unlock Commit register
    GPIOF->CR   = 0X01;             // Allow configuring PORTF0
    GPIOF->LOCK = 0;                // Lock it again

    GPIOF->DIR &= ~0x11U;           // PORTF4,0 input for switch
    GPIOF->DIR |= 0XEU;             // PORTF3, 2, 1 output for LEDs
    GPIOF->DEN |= 0X1FU;            // PORTF4-0 digital pins
    GPIOF->PUR |= 0x11U;            // enable pull up for PORTF4, 0

    GPIOF->IS  &= ~0x11U;           // make bit 4, 0 edge sensitive
    GPIOF->IBE &= ~0x11U;           // use IEV (not both edges)
    GPIOF->IEV &= ~0x11U;           // falling edge trigger
    GPIOF->ICR |= 0x11U;            // clear any prior interrupt
    GPIOF->IM  |= 0x11U;            // unmask interrupt
    
    SYSCTL->RCGCGPIO |= 0x02;
    while ((SYSCTL->PRGPIO & 0X02) == 0); 

    GPIOB->LOCK = 0x4C4F434B;
    GPIOB->CR  |= 0xFF;
    GPIOB->DIR &= ~0xFFU;                   // All input.
    GPIOB->DEN |= 0XFFU;                    // digital pins
    GPIOB->PUR |= 0xFFU;                    // enable pull up


    SYSCTL->RCGCGPIO |= 0x04;
    while ((SYSCTL->PRGPIO & 0X04) == 0);

    GPIOC->LOCK = 0x4C4F434B;
    GPIOC->CR  |= 0xFF;
    GPIOC->DIR |= 0xFFU;                    // All output.
    GPIOC->DEN |= 0XFFU;                    // digital pins 
}

void gpiof_handler(void) 
{
    volatile uint32_t readback;
    while (GPIOF->MIS != 0) {
        if (GPIOF->MIS & 0x01)          // masked interrupt status (detects if masking occurs) sw2
        {
            GPIOF->DATA ^= BLUE;        // toggle blue
            GPIOF->ICR |= 0x01;         // acknowledge flag
            readback = GPIOF->ICR;
        } else if (GPIOF->MIS & 0x10)   // masked interrupt status (detects if masking occurs) sw1
        {
            GPIOF->DATA ^= GREEN;       // toggle green
            GPIOF->ICR |= 0x10;         // acknowledge the flag
            readback = GPIOF->ICR;
        } else {
            GPIOF->ICR = GPIOF->MIS;
            readback = GPIOF->ICR;
        }
    }
}

void interrupt_config(void) 
{
    NVIC->IPR[30] |= 3 << 5;   // priority 3 to port F
    
    // enable IRQ from Port F (IRQ 30 > EN0 bit 30)
    NVIC->ISER[0] |= 1 << 30;   // enable interrupts for port f

    // Enable interrupts
    __enable_irq();
}

void Motor_Up(void) 
{
    vPrintString("Motor up ... \r\n");
    vTaskDelay(1000);
    GPIOF->DATA = BLUE;
}

void Motor_Down(void)
{
    vPrintString("Motor Down ... \r\n");
    vTaskDelay(1000);
    GPIOF->DATA = RED;
}

void Input_Handler_Task(void *pvParameters) 
{
    event_t event;
    uint8_t input;
    
    for (;;) 
    {
//        static count = 0;
        static struct ButtonsDebouncing {
            uint32_t depressed;
            uint32_t previous;
        } buttons = { 0U, 0U };

        uint32_t current;
        uint32_t tmp;

        current = ~GPIO_READ_PORT(B);

        tmp = buttons.depressed;                            /* save the debounced depressed buttons */

        buttons.depressed |= (buttons.previous & current);  /* set depressed */
        buttons.depressed &= (buttons.previous | current);  /* clear released */
        buttons.previous   = current;                       /* update the history */
        tmp ^= buttons.depressed;                           /* changed debounced depressed */

        if(READ_BIT(tmp, PASS_UP_PIN)){
            if((READ_BIT(buttons.depressed, PASS_UP_PIN))){
//                snprintf(buffer, 256, "count = %d \r\n",count++);
//                vPrintString(buffer);
                event = PASS_UP_BUTTON;
                if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
                    vPrintString("error\n");
            }
        }else if(READ_BIT(tmp , PASS_DOWN_PIN)){
            if((READ_BIT(buttons.depressed , PASS_DOWN_PIN))){
                event = PASS_DOWN_BUTTON;
                if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
                    vPrintString("error\n");
            }

        }else if(READ_BIT(tmp, DRIVER_UP_PIN)){
            if((READ_BIT(buttons.depressed, DRIVER_UP_PIN))){
                event = DRIVER_UP_BUTTON;
                if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
                    vPrintString("error\n");
            }

        }else if(READ_BIT(tmp, DRIVER_DOWN_PIN)){
            if((READ_BIT(buttons.depressed, DRIVER_DOWN_PIN))){
                event = DRIVER_DOWN_BUTTON;
                if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
                    vPrintString("error\n");
            }
            
        }else if(READ_BIT(tmp, LIMIT_UP_PIN)){
            if((READ_BIT(buttons.depressed, LIMIT_UP_PIN))){
                event = LIMIT_UP_BUTTON;
                if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
                    vPrintString("error\n");
            }
            
        }else if(READ_BIT(tmp, LIMIT_DOWN_PIN)){
            if((READ_BIT(buttons.depressed, LIMIT_DOWN_PIN))){
                event = LIMIT_UP_BUTTON;
                if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
                    vPrintString("error\n");
            }
            
        }else if(READ_BIT(tmp, JAM_PIN)){
            if((READ_BIT(buttons.depressed, LIMIT_DOWN_PIN))){
                event = JAM;
                if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
                    vPrintString("error\n");
            }
        }else if(READ_BIT(tmp, LOCK_ON_PIN)){
            if((READ_BIT(buttons.depressed, LOCK_ON_PIN))){
                event = LOCK_ON;
                if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
                    vPrintString("error\n");
            }
        }else{

        }
        vTaskDelay(5);
    }
}

void Event_Handler_Task(void *pvParameters) 
{
    event_t event;
    state_t state = NEUTRAL;

    for (;;) 
    {
        if (xQueueReceive(event_queue, &event, portMAX_DELAY) == pdPASS) 
        {
            switch (state) 
            {
                case NEUTRAL: {
                    // vPrintString("State : NEUTRAL\r\n");
                    switch (event) {
                        case PASS_DOWN_BUTTON:
                            if(!LOCK_ON_PRESSED) {// Lock not pressed
                                // vTimerSetTimerID(xOneShotTimer, (void *) DOWN_TIMER);
                                // xTimerStart(xOneShotTimer, 0);
                                state = PASS_DOWN;
                                event = PASS_DOWN_BUTTON;
                            }
//                                if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
//                                    vPrintString("error\n");
                            break;
                        case PASS_UP_BUTTON:
                            if(!LOCK_ON_PRESSED) {// Lock not pressed
                                state = PASS_UP;
                                event = PASS_UP_BUTTON;
                            }
//                                if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
//                                    vPrintString("error\n");
                            break;
                        case DRIVER_UP_BUTTON:
                            state = DRIVER_UP;      // even if lock is on
                            event = DRIVER_UP_BUTTON;
//                            if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
//                                vPrintString("error\n");
                            break;
                        case DRIVER_DOWN_BUTTON:
                            state = DRIVER_DOWN;    // even if lock is on
                            event = DRIVER_DOWN_BUTTON;
//                            if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
//                                vPrintString("error\n");
                            break;
                        case JAM:
                            state = EMERGENCY;
                            // may open semaphore
                            break;
                        default:
                            break;
                    }
                    break;
                }
                case PASS_DOWN: {
                    // vPrintString("State : PASS_DOWN\r\n");
                    switch (event) {
                        case PASS_DOWN_BUTTON:
                            vTaskDelay(100);    // Delay for a little bit 

                            if (!PASS_DOWN_PRESSED){ // passenger down not clicked anymore
                                // Auto mode
                                while (NO_INPUT) {
                                    Motor_Down();
                                }
                            } else {
                                // Manual mode
                                while(PASS_DOWN_ONLY){
                                    Motor_Down();
                                }
                            }
                            state = NEUTRAL;
                            break;
                        case PASS_UP_BUTTON:
                            state = PASS_UP;
                            event = PASS_UP_BUTTON;
//                            if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
//                                vPrintString("error\n");
                            
                            break;
                        case DRIVER_UP_BUTTON:
                            state = DRIVER_UP;
                            event = DRIVER_UP_BUTTON;
//                            if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
//                                vPrintString("error\n");
                            
                            break;
                        case DRIVER_DOWN_BUTTON:
                            state = DRIVER_DOWN;
                            event = DRIVER_DOWN_BUTTON;
//                            if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
//                                vPrintString("error\n");
                            
                            break;
                        case LOCK_ON:
                            state = NEUTRAL;
                            break;
                        case JAM:
                            state = EMERGENCY;
                            break;
                        default:
                            break;
                    }
                    break;
                }
                case PASS_UP: {
                    switch (event) {
                        case PASS_DOWN_BUTTON:
                            state = PASS_DOWN;
                            break;
                        case PASS_UP_BUTTON:
                            vTaskDelay(100);    // Delay for a little bit 

                            if (!PASS_UP_PRESSED){ // passenger down not clicked anymore
                                // Auto mode
                                while (NO_INPUT) {
                                    Motor_Up();
                                }
                            } else {
                                // Manual mode
                                while(PASS_UP_ONLY){
                                    Motor_Up();
                                }
                            }
                            state = NEUTRAL;
                            break;
                        case DRIVER_UP_BUTTON:
                            state = DRIVER_UP;
                            event = DRIVER_UP_BUTTON;
//                            if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
//                                vPrintString("error\n");
                            
                            break;
                        case DRIVER_DOWN_BUTTON:
                            state = DRIVER_DOWN;
                            event = DRIVER_DOWN_BUTTON;
//                            if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
//                                vPrintString("error\n");                            
                            break;
                        case LOCK_ON:
                            state = NEUTRAL;
                            break;
                        case JAM:
                            state = EMERGENCY;
                            break;
                        default:
                            break;
                    }
                    break;
                }
                case DRIVER_DOWN: {
                    switch (event) {
                        case DRIVER_DOWN_BUTTON:
                            vTaskDelay(100);    // Delay for a little bit 

                            if (!DRIVER_DOWN_PRESSED){ // passenger down not clicked anymore
                                // Auto mode
                                while (NO_INPUT) {
                                    Motor_Down();
                                }
                            } else {
                                // Manual mode
                                while(DRIVER_DOWN_PRESSED && !JAM_PRESSED){
                                    Motor_Down();
                                }
                            }
                            state = NEUTRAL;
                            break;
                        case DRIVER_UP_BUTTON:
                            state = DRIVER_UP;
                            event = DRIVER_UP_BUTTON;
//                            if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
//                                vPrintString("error\n");
                            break;
                        case JAM:
                            state = EMERGENCY;
                            break;
                        default:
                            break;
                    }
                    break;
                }
                case DRIVER_UP: {
                    switch (event) {
                        case DRIVER_DOWN_BUTTON:
                            state = DRIVER_DOWN;
                            break;
                        case DRIVER_UP_BUTTON:
                            vTaskDelay(100);    // Delay for a little bit 

                            if (!DRIVER_UP_PRESSED){ // passenger down not clicked anymore
                                // Auto mode
                                while (NO_INPUT) {
                                    Motor_Down();
                                }
                            } else {
                                // Manual mode
                                while(DRIVER_UP_PRESSED && !JAM_PRESSED){
                                    Motor_Down();
                                }
                            }
                            state = NEUTRAL;
                            break;
                        case JAM:
                            state = EMERGENCY;
                            break;
                        default:
                            break;
                    }
                    break;
                }
                case EMERGENCY: {
                    Motor_Down();
                    Motor_Down();
                    state = NEUTRAL;
                    break;
                }
            }
        }
    }
}


static void prvOneShotTimerCallback( TimerHandle_t xTimer )
{
    uint32_t TimerID;
    TimerID = (uint32_t) pvTimerGetTimerID(xTimer);
    switch (TimerID) {
        case DOWN_TIMER:
//            if(PASS_DOWN_PRESSED) // still pressed manual mode
//            else 
                // automatic
            break;
        case UP_TIMER:
            break;
        default:
            break;
    }
}

int main(void) 
                {
    gpio_init();
    uart_init(115200);
    interrupt_config();

    event_queue = xQueueCreate(10, sizeof(event_t));

    /* Create the one shot timer, storing the handle to the created timer in xOneShotTimer. */
    xOneShotTimer = xTimerCreate("OneShot",
                                 mainONE_SHOT_TIMER_PERIOD,
                                 pdFALSE,                   // one-shot software timer.
                                 0,                         // no timer id.
                                 prvOneShotTimerCallback);  // Callback function to used by timer.



    xTaskCreate(Input_Handler_Task, "Input Handler", 200, NULL, 1, NULL);
    xTaskCreate(Event_Handler_Task, "Event Handler", 200, NULL, 2, NULL);

    vTaskStartScheduler();

    for (;;);
}

void vApplicationMallocFailedHook(void) 
{
    for (;;);
}

void vApplicationIdleHook(void) {}

void vApplicationTickHook(void) {}
