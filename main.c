#include "TM4C123GH6PM.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "stdint.h"
#include "math.h"

#define RED   0x02
#define BLUE  0x04
#define GREEN 0x08

void GPIO_Init(void);
void GPIOF_Handler(void);
void Interrupt_Config(void);

void UART_Init(uint32_t baudrate);
void print_char(const char ch);
void print_string(const char *str);
void vPrintString(const char *pcString );
char read_char(void);

void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationTickHook( void );

void Input_Handler_Task( void *pvParameters );
void Event_Handler_Task( void *pvParameters );

typedef enum {
		event_1,
		event_2,
		event_3,
		event_4,
		event_5,
		event_6,
		event_7
}event_t;

static QueueHandle_t event_queue;

void UART_Init(uint32_t baudrate)
{
	/* Enable the UART module using the RCGCUART register */
	SYSCTL->RCGCUART |= (1<<0);  // Enable and provide a clock to UART module 0 in Run mode
	
	/* Enable the clock to the appropriate GPIO module via the RCGCGPIO register */
	SYSCTL->RCGCGPIO |= (1<<0);   // Enable and provide a clock to GPIO Port A in Run mode.
	while((SYSCTL->PRGPIO&0X1)==0);  
	
	/* Set the GPIO AFSEL bits for the appropriate pins */
	GPIOA->AFSEL = (1<<1)|(1<<0); // Enable alternate function for Pin A0 and A1
	
	/* Configure the GPIO current level and/or slew rate as specified for the mode selected */
	
	/* 
		- Configure the PMCn fields in the GPIOPCTL register to assign the UART signals to the appropriate pins 
	
		- When a bit is set in the GPIOAFSEL register, the corresponding GPIO signal is controlled by an associated peripheral. 
			The GPIOPCTL register selects one out of a set of peripheral functions for each GPIO, 
			providing additional flexibility in signal definition
	*/
	
	 GPIOA->PCTL = (1<<0)|(1<<4);   // PMC0 and PMC1 -> enabling uart0 on PA0 and PA1
	
	 GPIOA->DEN = (1<<0)|(1<<1);   // Enable digital functions for PA0 and PA1

	/* 
		BRD = 50,000,000 / (16 * 115,200) = 27.126736
		UARTFBRD[DIVFRAC] = integer(0.126736 * 64 + 0.5) = 8
	*/
	
	
	/* Disable the UART by clearing the UARTEN bit in the UARTCTL register. */
	UART0->CTL &= ~(1<<0);  // UART Enable bit
	
	/* Write the integer portion of the BRD to the UARTIBRD register.*/
	
	SystemCoreClockUpdate();

	double integral;
	double fractional = modf((double)SystemCoreClock / (16 * baudrate),&integral);
	
	
	UART0->IBRD = (uint32_t)integral;    
	
	/* Write the fractional portion of the BRD to the UARTFBRD register. */
	//UART0->FBRD = 44;
	UART0->FBRD = (uint32_t) (fractional * 64 + 0.5); 
	
	/* Write the desired serial parameters to the UARTLCRH register */
	
	UART0->LCRH = (0x3<<5)|(1<<4);   // 8-bit, no parity, 1-stop bit
	
	/* Configure the UART clock source by writing to the UARTCC register. */
	UART0->CC = 0x0;   // use system clock
	
	/* Enable the UART by setting the UARTEN bit in the UARTCTL register. */
	UART0->CTL = (1<<0)|(1<<8)|(1<<9); // enable uart, tx, rx
}

void print_char(const char ch)
{
	while((UART0->FR & (1<<5)) != 0); // Previous transmissions ended and FIFO is not full.
	UART0->DR = ch;
}

void print_string(const char *str)
{
	while(*str){
		print_char(*(str++));
	}
}

char read_char(void)
{
	char ch;
	while((UART0->FR & (1<<4)) != 0); // receiver fifo is not empty
	ch = UART0->DR;
	return ch;
}

void vPrintString(const char *pcString )
{
    /* Print the string, suspending the scheduler as method of mutual
    exclusion. */
    vTaskSuspendAll();
    {
        print_string(pcString);
    }
    xTaskResumeAll();
}

void GPIO_Init(void) // Port F pin 4
{
	SYSCTL->RCGCGPIO |= 0X20;						// Enable clock to PORTF
	while((SYSCTL->PRGPIO&0X20)==0);    // Wait until it is enabled
	
	GPIOF->LOCK = 0X4C4F434B;						// Unlock Commit register
	GPIOF->CR   = 0X01;								  // Allow configuring PORTF0
	GPIOF->LOCK = 0;                    // Lock it again 
	
	GPIOF->DIR &= ~0x11U;								// PORTF4,0 input for switch 
	GPIOF->DIR |=  0XEU;								// PORTF3, 2, 1 output for LEDs 
	GPIOF->DEN |=  0X1FU;								// PORTF4-0 digital pins
	GPIOF->PUR |=  0x11U;								// enable pull up for PORTF4, 0
	
	GPIOF->IS  &= ~0x11U; 							// make bit 4, 0 edge sensitive
	GPIOF->IBE &= ~0x11U; 			  			// use IEV (not both edges)
	GPIOF->IEV &= ~0x11U; 			 				// falling edge trigger
	GPIOF->ICR |=  0x11U;       			  // clear any prior interrupt 
	GPIOF->IM  |=  0x11U;         			// unmask interrupt
}

void GPIOF_Handler(void)
{
	volatile uint32_t readback;
	while (GPIOF->MIS != 0)
	{
		if (GPIOF->MIS & 0x01) // masked interrupt status (detects if masking occurs) sw2
		{
			GPIOF->DATA ^= BLUE; // toggle blue
			GPIOF->ICR |= 0x01;  //acknowledge flag
			readback = GPIOF->ICR;
		}
		else if (GPIOF->MIS & 0x10) // masked interrupt status (detects if masking occurs) sw1
		{
			GPIOF->DATA ^= GREEN; // toggle green
			GPIOF->ICR  |= 0x10;  // acknowledge the flag
			readback = GPIOF->ICR;
		}
		else
		{
			GPIOF->ICR = GPIOF->MIS;
			readback = GPIOF->ICR;
		}
	}
}

void Interrupt_Config(void)
{
	NVIC->IPR[30] |= 3<<5; // priority 3 to port F
	// enable IRQ from Port F (IRQ 30 > EN0 bit 30)
	NVIC->ISER[0] |= 1<<30; // enable interrupts for port f
	
	// Enable interrupts
	__enable_irq();
}


/* Used as a loop counter to create a very crude delay. */
#define mainDELAY_LOOP_COUNT		( 0xfffff )

/* The task functions. */
void Input_Handler_Task( void *pvParameters );
void Event_Handler_Task( void *pvParameters );


void Input_Handler_Task( void *pvParameters )
{
	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		char ch = read_char();
		print_char(ch);
		event_t event;
		
		switch(ch){
			case 'a':
				event = event_1;
				if(xQueueSend(event_queue,(void *) &event,0) != pdPASS) vPrintString("error\n");
				break;
			case 'b':
				event = event_2;
				if(xQueueSend(event_queue,(void *) &event,0) != pdPASS) vPrintString("error\n");
				break;
			case 'c':
				event = event_3;
				if(xQueueSend(event_queue,(void *) &event,0) != pdPASS) vPrintString("error\n");
				break;
			case 'd':
				event = event_4;
				if(xQueueSend(event_queue,(void *) &event,0) != pdPASS) vPrintString("error\n");
				break;
			case 'e':
				event = event_5;
				if(xQueueSend(event_queue,(void *) &event,0) != pdPASS) vPrintString("error\n");
				break;
			case 'f':
				event = event_6;
				if(xQueueSend(event_queue,(void *) &event,0) != pdPASS) vPrintString("error\n");
				break;
			default:
				event = event_7;
				if(xQueueSend(event_queue,(void *) &event,0) != pdPASS) vPrintString("error\n");
				break;					
		}

		// vPrintString("Blue \n\r");
		// GPIOF->DATA ^= BLUE;
	}
}
/*-----------------------------------------------------------*/

void Event_Handler_Task( void *pvParameters )
{
	event_t event;

	for( ;; )
	{
		if(xQueueReceive(event_queue,&event,portMAX_DELAY) == pdPASS)
		{
			switch(event)
			{
				case event_1:
					GPIOF->DATA = RED;
					break;
				case event_2:
					GPIOF->DATA = GREEN;
					break;
				case event_3:
					GPIOF->DATA = BLUE;
					break;
				case event_4:
					GPIOF->DATA = RED|GREEN;
					break;
				case event_5:
					GPIOF->DATA = GREEN|BLUE;
					break;
				case event_6:
					GPIOF->DATA = RED|BLUE;
					break;
				default:
					GPIOF->DATA  = RED|GREEN|BLUE;
				 break;
			}
		}
		//vPrintString("Red \n\r");
		//GPIOF->DATA ^= RED;
	}
}
/*-----------------------------------------------------------*/


int main(void)
{
	GPIO_Init();
	UART_Init(115200);
	Interrupt_Config();

	
	event_queue = xQueueCreate(10,sizeof(event_t));

	xTaskCreate(Input_Handler_Task, "Input Handler", 200, NULL, 1,	NULL );	

	xTaskCreate(Event_Handler_Task, "Event Handler", 200, NULL, 2, NULL );

	vTaskStartScheduler();
	
	for( ;; );

}

void vApplicationMallocFailedHook( void )
{

	for( ;; );
}


void vApplicationIdleHook( void )
{

}

void vApplicationTickHook( void )
{

}
