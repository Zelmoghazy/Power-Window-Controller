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

#define RED                             0x02
#define BLUE                            0x04
#define GREEN                           0x08

#define LED_ON(COLOR)                   GPIOF->DATA = (COLOR)

#define CONCAT_AUX(a, b)                a##b
#define CONCAT(a, b)                    CONCAT_AUX(a, b)

/* UART */
#define FIFO_TX_FULL()                  ((UART0->FR & (1 << 5)) == 1)		// if bit 5 is 1 -> tx fifo is full
#define FIFO_RX_EMPTY()                 ((UART0->FR & (1 << 4)) == 1)		// if bit 4 is 1 -> rx fifo is empty

/* GPIO */
#define INPUT_PORT                      B
#define OUTPUT_PORT                     C

#define GPIO_READ_PIN(PORT,PIN)         (((CONCAT(GPIO,PORT->DATA))>>PIN)&1U)
#define GPIO_SET_PIN(PORT,PIN)          ((CONCAT(GPIO,PORT->DATA))|=1<<PIN)
#define GPIO_CLEAR_PIN(PORT,PIN)        ((CONCAT(GPIO,PORT->DATA))&=~(1<<PIN))
#define GPIO_TOGGLE_PIN(PORT,PIN)       ((CONCAT(GPIO,PORT->DATA))|=1<<PIN)

#define GPIO_READ_PORT(PORT)            ((CONCAT(GPIO,PORT->DATA)))
#define GPIO_WRITE_PORT(PORT,VALUE)     ((CONCAT(GPIO,PORT->DATA))=VALUE)

#define READ_BIT(DATA,BIT)              (((DATA)>>BIT)&1U)

/* MOTOR */
#define MOTOR_CLOCKWISE_PIN             4
#define MOTOR_COUNTER_CLOCKWISE_PIN     5

#define MOTOR_CLOCKWISE()               GPIO_SET_PIN(OUTPUT_PORT,MOTOR_CLOCKWISE_PIN);GPIO_CLEAR_PIN(OUTPUT_PORT,MOTOR_COUNTER_CLOCKWISE_PIN)
#define MOTOR_COUNTER_CLOCKWISE()       GPIO_SET_PIN(OUTPUT_PORT,MOTOR_COUNTER_CLOCKWISE_PIN);GPIO_CLEAR_PIN(OUTPUT_PORT,MOTOR_CLOCKWISE_PIN)
#define MOTOR_STOP()                    GPIO_CLEAR_PIN(OUTPUT_PORT,MOTOR_COUNTER_CLOCKWISE_PIN);GPIO_CLEAR_PIN(OUTPUT_PORT,MOTOR_CLOCKWISE_PIN)

/* INPUTS */
#define PASS_UP_PIN                     0
#define PASS_DOWN_PIN                   1
#define DRIVER_UP_PIN                   2
#define DRIVER_DOWN_PIN                 3
#define LIMIT_UP_PIN                    4
#define LIMIT_DOWN_PIN                  5
#define JAM_PIN                         6
#define LOCK_ON_PIN                     7

#define PASS_DOWN_PRESSED               (!GPIO_READ_PIN(INPUT_PORT, PASS_DOWN_PIN)) 
#define PASS_UP_PRESSED                 (!GPIO_READ_PIN(INPUT_PORT, PASS_UP_PIN))
#define DRIVER_UP_PRESSED               (!GPIO_READ_PIN(INPUT_PORT, DRIVER_UP_PIN))
#define DRIVER_DOWN_PRESSED             (!GPIO_READ_PIN(INPUT_PORT, DRIVER_DOWN_PIN))
#define LIMIT_UP_PRESSED                (!GPIO_READ_PIN(INPUT_PORT, LIMIT_UP_PIN))
#define LIMIT_DOWN_PRESSED              (!GPIO_READ_PIN(INPUT_PORT, LIMIT_DOWN_PIN))
#define JAM_PRESSED                     (!GPIO_READ_PIN(INPUT_PORT, JAM_PIN))
#define LOCK_ON_PRESSED                 (!GPIO_READ_PIN(INPUT_PORT, LOCK_ON_PIN))

#define PASS_DOWN_MANUAL_COND           (PASS_DOWN_PRESSED && !(DRIVER_UP_PRESSED || DRIVER_DOWN_PRESSED || LIMIT_DOWN_PRESSED || LOCK_ON_PRESSED) && !emergency)
#define PASS_DOWN_AUTO_COND             (!(DRIVER_UP_PRESSED || DRIVER_DOWN_PRESSED || PASS_UP_PRESSED || LIMIT_DOWN_PRESSED || LOCK_ON_PRESSED) && !emergency)
#define PASS_UP_MANUAL_COND             (PASS_UP_PRESSED && !(DRIVER_UP_PRESSED || DRIVER_DOWN_PRESSED || LIMIT_UP_PRESSED || LOCK_ON_PRESSED) && !emergency)
#define PASS_UP_AUTO_COND               (!(DRIVER_UP_PRESSED || DRIVER_DOWN_PRESSED || PASS_DOWN_PRESSED || LIMIT_UP_PRESSED || LOCK_ON_PRESSED) && !emergency)
#define DRIVER_DOWN_MANUAL_COND         (DRIVER_DOWN_PRESSED && !(LIMIT_DOWN_PRESSED) && !emergency)
#define DRIVER_DOWN_AUTO_COND           (!(LIMIT_DOWN_PRESSED || DRIVER_UP_PRESSED) && !emergency)
#define DRIVER_UP_MANUAL_COND           (DRIVER_UP_PRESSED && !(LIMIT_UP_PRESSED) && !emergency)
#define DRIVER_UP_AUTO_COND             (!(LIMIT_UP_PRESSED || DRIVER_DOWN_PRESSED) && !emergency)

#define STATE_EXIT()                    emergency = 0;MOTOR_STOP();LED_ON(GREEN);state = NEUTRAL

/* TIMER IDS */
#define PASS_DOWN_TIMER                 0
#define PASS_UP_TIMER                   1
#define DRIVER_UP_TIMER                 2
#define DRIVER_DOWN_TIMER               3

#define mainONE_SHOT_TIMER_PERIOD       200
#define MAX_BUFFER_SIZE                 256

char emergency = 0;
char buffer[MAX_BUFFER_SIZE];           // To use if you want to print formatted string.

static QueueHandle_t event_queue;
TimerHandle_t xOneShotTimer;
xSemaphoreHandle Emergency_Semaphore;

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
} signal_t;

typedef enum {
    NO_FLAG,
    AUTOMATIC_FLAG = 2,
    MANUAL_FLAG   
} flag_t;

typedef struct{
    signal_t    sig;
    flag_t      flag;
}event_t;

typedef enum {
    NEUTRAL,
    PASS_DOWN,
    PASS_UP,
    DRIVER_DOWN,
    DRIVER_UP,
    EMERGENCY
} state_t;

void gpio_init(void);
void GPIOB_Handler(void); 
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
void Emergency_Task(void *pvParameters);

void Motor_Clockwise(void);
void Motor_Counter_Clockwise(void);

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

    GPIOF->LOCK = 0X4C4F434B;                // Unlock Commit register
    GPIOF->CR   = 0X01;                      // Allow configuring PORTF0
    GPIOF->LOCK = 0;                         // Lock it again

    GPIOF->DIR &= ~0x11U;                    // PORTF4,0 input for switch
    GPIOF->DIR |= 0XEU;                      // PORTF3, 2, 1 output for LEDs
    GPIOF->DEN |= 0X1FU;                     // PORTF4-0 digital pins
    GPIOF->PUR |= 0x11U;                     // enable pull up for PORTF4, 0

    
    SYSCTL->RCGCGPIO |= 0x02;
    while ((SYSCTL->PRGPIO & 0X02) == 0); 

    GPIOB->LOCK = 0x4C4F434B;
    GPIOB->CR  |= 0xFF;
    GPIOB->DIR &= ~0xFFU;                     // All input.
    GPIOB->DEN |= 0XFFU;                      // digital pins
    GPIOB->PUR |= 0xFFU;                      // enable pull up
    
    GPIOB->IS  &= ~0x40U;                     // make bit 6 edge sensitive
    GPIOB->IBE &= ~0x40U;                     // use IEV (not both edges)
    GPIOB->IEV &= ~0x40U;                     // falling edge trigger
    GPIOB->ICR |= 0x40U;                      // clear any prior interrupt
    GPIOB->IM  |= 0x40U;                      // unmask interrupt


    SYSCTL->RCGCGPIO |= 0x04;
    while ((SYSCTL->PRGPIO & 0X04) == 0);

    GPIOC->LOCK = 0x4C4F434B;
    GPIOC->CR  |= 0xFF;
    GPIOC->DIR |= 0xFFU;                    // All output.
    GPIOC->DEN |= 0XFFU;                    // digital pins 
}

void interrupt_config(void) 
{
    NVIC->IPR[1] |= 3 << 5;   // priority 3 to port B
    
    // enable IRQ from Port B (IRQ 1 > EN0 bit 1)
    NVIC->ISER[0] |= 1 << 1;   // enable interrupts for port B

    // Enable interrupts
    __enable_irq();
}

void GPIOB_Handler(void) 
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    volatile uint32_t readback;
       
    GPIOB->ICR |= 0x40;                   // acknowledge flag
    readback = GPIOB->ICR;
    
    vPrintString("Emergency Interrupt!");
    emergency = 1;

    xSemaphoreGiveFromISR(Emergency_Semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Motor_Counter_Clockwise(void) 
{
    static uint32_t counter = 0;
    if (!(counter%10000)){
        MOTOR_COUNTER_CLOCKWISE();
        if(emergency){
            LED_ON(BLUE|RED|GREEN); 
        }else{
            LED_ON(BLUE);
        }
        counter = 0;
    }
    counter++;
}

void Motor_Clockwise(void)
{
    static uint32_t counter = 0;
    if (!(counter%10000))
    {
        MOTOR_CLOCKWISE();
        if(emergency){
            LED_ON(BLUE|RED|GREEN); 
        }else{
            LED_ON(RED);
        }
        counter = 0;
    }
    counter++;
}

void Input_Handler_Task(void *pvParameters) 
{
    event_t event = {0};
    
    for (;;) 
    {
        static struct ButtonsDebouncing {
            uint32_t depressed;
            uint32_t previous;
        } buttons = { 0U, 0U };

        uint32_t current;
        uint32_t tmp;

        current = ~GPIO_READ_PORT(INPUT_PORT);

        tmp = buttons.depressed;                            

        buttons.depressed |= (buttons.previous & current);  
        buttons.depressed &= (buttons.previous | current); 
        buttons.previous   = current;                       
        tmp ^= buttons.depressed;                           

        if(READ_BIT(tmp, PASS_UP_PIN)){
            if((READ_BIT(buttons.depressed, PASS_UP_PIN))){
                event.sig = PASS_UP_BUTTON;
                if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
                    vPrintString("error\n");
            }
        }else if(READ_BIT(tmp , PASS_DOWN_PIN)){
            if((READ_BIT(buttons.depressed , PASS_DOWN_PIN))){
                event.sig = PASS_DOWN_BUTTON;
                if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
                    vPrintString("error\n");
            }

        }else if(READ_BIT(tmp, DRIVER_UP_PIN)){
            if((READ_BIT(buttons.depressed, DRIVER_UP_PIN))){
                event.sig = DRIVER_UP_BUTTON;
                if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
                    vPrintString("error\n");
            }

        }else if(READ_BIT(tmp, DRIVER_DOWN_PIN)){
            if((READ_BIT(buttons.depressed, DRIVER_DOWN_PIN))){
                event.sig = DRIVER_DOWN_BUTTON;
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
    event_t event = {0};
    state_t state = NEUTRAL;

    for (;;) 
    {
        if (xQueueReceive(event_queue, &event, portMAX_DELAY) == pdPASS) 
        {
            switch (state) 
            {
                case NEUTRAL: {
                    switch (event.sig){
                        case PASS_DOWN_BUTTON:
                            if(!LOCK_ON_PRESSED) {  
                                state = PASS_DOWN;
                                vTimerSetTimerID(xOneShotTimer, (void *) PASS_DOWN_TIMER);
                                xTimerStart(xOneShotTimer, 0);
                            }
                            break;

                        case PASS_UP_BUTTON:
                            if(!LOCK_ON_PRESSED) {  
                                state = PASS_UP;
                                vTimerSetTimerID(xOneShotTimer, (void *) PASS_UP_TIMER);
                                xTimerStart(xOneShotTimer, 0);
                            }
                            break;

                        case DRIVER_UP_BUTTON:
                                state = DRIVER_UP;
                                vTimerSetTimerID(xOneShotTimer, (void *) DRIVER_UP_TIMER);
                                xTimerStart(xOneShotTimer, 0);
                            break;

                        case DRIVER_DOWN_BUTTON:
                                state = DRIVER_DOWN;
                                vTimerSetTimerID(xOneShotTimer, (void *) DRIVER_DOWN_TIMER);
                                xTimerStart(xOneShotTimer, 0);
                            break;
                        default:
                            break;
                    }
                    LED_ON(GREEN);
                    break;
                }
                case PASS_DOWN:{
                    switch (event.sig) {
                        case PASS_DOWN_BUTTON:
                            if (event.flag == AUTOMATIC_FLAG){ 
                                while(PASS_DOWN_AUTO_COND){
                                    Motor_Clockwise();
                                }
                            } else {
                                while(PASS_DOWN_MANUAL_COND){
                                    Motor_Clockwise();
                                }
                            }
                            STATE_EXIT();
                            break;
                        default:
                            break;
                    }
                    break;
                }
                case PASS_UP:{
                    switch (event.sig) {
                        case PASS_UP_BUTTON:
                            if (event.flag == AUTOMATIC_FLAG){ 
                                while(PASS_UP_AUTO_COND){
                                    Motor_Counter_Clockwise();
                                }
                            } else {
                                while(PASS_UP_MANUAL_COND){
                                    Motor_Counter_Clockwise();
                                }
                            }
                            STATE_EXIT();
                            break;
                        default:
                            break;
                    }
                    break;
                }
                case DRIVER_DOWN:{
                    switch (event.sig) {
                        case DRIVER_DOWN_BUTTON:
                            if (event.flag == AUTOMATIC_FLAG){ 
                                while(DRIVER_DOWN_AUTO_COND){
                                    Motor_Clockwise();
                                }
                            } else {
                                while(DRIVER_DOWN_MANUAL_COND){
                                    Motor_Clockwise();
                                }
                            }
                            STATE_EXIT();
                            break;
                        default:
                            break;
                    }
                    break;
                }

                case DRIVER_UP:{
                    switch (event.sig) {
                        case DRIVER_UP_BUTTON:
                            if (event.flag == AUTOMATIC_FLAG){ 
                                while(DRIVER_UP_AUTO_COND){
                                    Motor_Counter_Clockwise();
                                }
                            } else {
                                while(DRIVER_UP_MANUAL_COND){
                                    Motor_Counter_Clockwise();
                                }
                            }
                            STATE_EXIT();
                            break;
                        default:
                            break;
                    }
                    break;
                }
                default:
                    break;
            }
        }
    }
}

static void prvOneShotTimerCallback(TimerHandle_t xTimer)
{
    event_t event = {0};
    uint32_t TimerID;
    TimerID = (uint32_t) pvTimerGetTimerID(xTimer);
    
    switch (TimerID) 
    {
        case PASS_DOWN_TIMER:
            if (!PASS_DOWN_PRESSED){ 
                event.sig = PASS_DOWN_BUTTON;
                event.flag = AUTOMATIC_FLAG;
            } else {
                event.sig = PASS_DOWN_BUTTON;
                event.flag = MANUAL_FLAG;
            }
            if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
                vPrintString("error\n");

            break;

        case PASS_UP_TIMER:
            if (!PASS_UP_PRESSED){
                event.sig = PASS_UP_BUTTON;
                event.flag = AUTOMATIC_FLAG;
            } else {
                event.sig = PASS_UP_BUTTON;
                event.flag = MANUAL_FLAG;
            }
            if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
                vPrintString("error\n");

            break;

        case DRIVER_DOWN_TIMER:
            if (!DRIVER_DOWN_PRESSED){ 
                event.sig = DRIVER_DOWN_BUTTON;
                event.flag = AUTOMATIC_FLAG;
            } else {
                event.sig = DRIVER_DOWN_BUTTON;
                event.flag = MANUAL_FLAG;
            }
            if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
                vPrintString("error\n");

            break;

        case DRIVER_UP_TIMER:
            if (!DRIVER_UP_PRESSED){
                event.sig = DRIVER_UP_BUTTON;
                event.flag = AUTOMATIC_FLAG;
            } else {
                event.sig = DRIVER_UP_BUTTON;
                event.flag = MANUAL_FLAG;
            }
            if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
                vPrintString("error\n");

            break;

        default:
            break;
    }
}

void Emergency_Task(void *pvParameters)
{
	xSemaphoreTake(Emergency_Semaphore, 0);
    int counter = 0;
    
	for( ;; )
	{
        
		xSemaphoreTake(Emergency_Semaphore, portMAX_DELAY);
        
        __disable_irq();

        vPrintString("Jam Detected !!");
        while(counter < 1000000){
            Motor_Clockwise();
            counter ++;
        }
        MOTOR_STOP();
        counter = 0;
        
        __enable_irq();
	}
}

int main(void) 
{
    gpio_init();
    uart_init(115200);
    interrupt_config();

    event_queue         = xQueueCreate(3, sizeof(event_t));
    Emergency_Semaphore = xSemaphoreCreateBinary();

    xOneShotTimer = xTimerCreate("OneShot",
                                 mainONE_SHOT_TIMER_PERIOD,
                                 pdFALSE,                   // one-shot software timer.
                                 0,                         // no initial timer id.
                                 prvOneShotTimerCallback); 


    xTaskCreate(Input_Handler_Task, "Input Handler", 128, NULL, 1, NULL);
    xTaskCreate(Event_Handler_Task, "Event Handler", 128, NULL, 2, NULL);
    xTaskCreate(Emergency_Task, "Emergency Handler", 128, NULL, 3, NULL);

    vTaskStartScheduler();

    for (;;);
}

void vApplicationMallocFailedHook(void) 
{
    for (;;);
}

void vApplicationIdleHook(void) {}

void vApplicationTickHook(void) {}
