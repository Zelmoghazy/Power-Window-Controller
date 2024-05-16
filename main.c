#include "FreeRTOS.h"
#include "TM4C123GH6PM.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "math.h"
#include "stdint.h"

#define RED   0x02
#define BLUE  0x04
#define GREEN 0x08

void GPIO_Init(void);
void GPIOF_Handler(void);
void Interrupt_Config(void);

void UART_Init(uint32_t baudrate);
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

static QueueHandle_t event_queue;

typedef enum {
    PASS_DOWN_BUTTON,
    PASS_UP_BUTTON,
    LIMIT_BUTTON,
    DRIVER_UP_BUTTON,
    DRIVER_DOWN_BUTTON,
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

void UART_Init(uint32_t baudrate) 
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
    while ((UART0->FR & (1 << 5)) != 0);   // Previous transmissions ended and FIFO is not full.
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
    while ((UART0->FR & (1 << 4)) != 0);   // receiver fifo is not empty
    ch = UART0->DR;
    return ch;
}

void vPrintString(const char *pcString) 
{
    /* Print the string, suspending the scheduler as method of mutual exclusion. */
    vTaskSuspendAll();
    {
        print_string(pcString); 
    }
    xTaskResumeAll();
}

void GPIO_Init(void)   // Port F pin 4
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
}

void GPIOF_Handler(void) 
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

void Interrupt_Config(void) 
{
    NVIC->IPR[30] |= 3 << 5;   // priority 3 to port F
    
    // enable IRQ from Port F (IRQ 30 > EN0 bit 30)
    NVIC->ISER[0] |= 1 << 30;   // enable interrupts for port f

    // Enable interrupts
    __enable_irq();
}

void Motor_Up(void) 
{
    vPrintString("Motor up ... \n\r");
    GPIOF->DATA = BLUE;
}

void Motor_Down(void)
{
    vPrintString("Motor Down ... \n\r");
    GPIOF->DATA = RED;
}

void Input_Handler_Task(void *pvParameters) 
{
    /* As per most tasks, this task is implemented in an infinite loop. */
    for (;;) 
    {
        char ch = read_char();
        print_char(ch);
        event_t event;

        switch (ch) 
        {
            case 'a':
                event = PASS_DOWN_BUTTON;
                if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
                    vPrintString("error\n");
                break;
            case 'b':
                event = PASS_UP_BUTTON;
                if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
                    vPrintString("error\n");
                break;
            case 'c':
                event = LIMIT_BUTTON;
                if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
                    vPrintString("error\n");
                break;
            case 'd':
                event = DRIVER_UP_BUTTON;
                if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
                    vPrintString("error\n");
                break;
            case 'e':
                event = DRIVER_DOWN_BUTTON;
                if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
                    vPrintString("error\n");
                break;
            case 'f':
                event = LOCK_ON;
                if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
                    vPrintString("error\n");
                break;
            default:
                event = LOCK_OFF;
                if (xQueueSend(event_queue, (void *) &event, 0) != pdPASS)
                    vPrintString("error\n");
                break;
        }
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
                    switch (event) {
                        case PASS_DOWN_BUTTON:
                            Motor_Down();
                            state = PASS_DOWN;
                            break;
                        case PASS_UP_BUTTON:
                            Motor_Up();
                            state = PASS_UP;
                            break;
                        case DRIVER_UP_BUTTON:
                            Motor_Up();
                            state = DRIVER_UP;
                            break;
                        case DRIVER_DOWN_BUTTON:
                            Motor_Down();
                            state = DRIVER_DOWN;
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
                    switch (event) {
                        case PASS_DOWN_BUTTON:
                            vTaskDelay(100);
                            if ((UART0->FR & (1 << 4)) != 0) {
                                while ((UART0->FR & (1 << 4)) != 0) {
                                    Motor_Down();
                                }
                            } else {
                                while (UART0->DR == 'a') {
                                    Motor_Down();
                                }
                            }
                            break;
                        case PASS_UP_BUTTON:
                            Motor_Up();
                            state = PASS_UP;
                            break;
                        case DRIVER_UP_BUTTON:
                            Motor_Up();
                            state = DRIVER_UP;
                            break;
                        case DRIVER_DOWN_BUTTON:
                            Motor_Down();
                            state = DRIVER_DOWN;
                            break;
                        case LOCK_ON:
                            state = NEUTRAL;
                            break;
                        case JAM:
                            state = EMERGENCY;
                        default:
                            break;
                    }
                    break;
                }
                case PASS_UP: {
                    switch (event) {
                        case PASS_DOWN_BUTTON:
                            Motor_Down();
                            state = PASS_DOWN;
                            break;
                        case PASS_UP_BUTTON:
                            vTaskDelay(100);
                            if ((UART0->FR & (1 << 4)) != 0) {
                                while ((UART0->FR & (1 << 4)) != 0) {
                                    Motor_Up();
                                }
                            } else {
                                while (UART0->DR == 'a') {
                                    Motor_Up();
                                }
                            }
                            break;
                        case DRIVER_UP_BUTTON:
                            Motor_Up();
                            state = DRIVER_UP;
                            break;
                        case DRIVER_DOWN_BUTTON:
                            Motor_Down();
                            state = DRIVER_DOWN;
                            break;
                        case LOCK_ON:
                            state = NEUTRAL;
                            break;
                        case JAM:
                            state = EMERGENCY;
                        default:
                            break;
                    }
                    break;
                }
                case DRIVER_DOWN: {
                    switch (event) {
                        case DRIVER_UP_BUTTON:
                            Motor_Up();
                            state = DRIVER_UP;
                            break;
                        case DRIVER_DOWN_BUTTON:
                            vTaskDelay(100);
                            if ((UART0->FR & (1 << 4)) != 0) {
                                while ((UART0->FR & (1 << 4)) != 0) {
                                    Motor_Down();
                                }
                            } else {
                                while (UART0->DR == 'a') {
                                    Motor_Down();
                                }
                            }
                            break;
                        case JAM:
                            state = EMERGENCY;
                        default:
                            break;
                    }
                    break;
                }
                case DRIVER_UP: {
                    switch (event) {
                        case DRIVER_UP_BUTTON:
                            vTaskDelay(100);
                            if ((UART0->FR & (1 << 4)) != 0) {
                                while ((UART0->FR & (1 << 4)) != 0) {
                                    Motor_Up();
                                }
                            } else {
                                while (UART0->DR == 'a') {
                                    Motor_Up();
                                }
                            }
                            break;
                        case DRIVER_DOWN_BUTTON:
                            Motor_Down();
                            state = DRIVER_DOWN;
                            break;
                        case JAM:
                            state = EMERGENCY;
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

int main(void) 
{
    GPIO_Init();
    UART_Init(115200);
    Interrupt_Config();

    event_queue = xQueueCreate(10, sizeof(event_t));

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
