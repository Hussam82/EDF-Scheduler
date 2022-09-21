/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
 */


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

#define NULL_PTR                             (void*)0



/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/


/* Periodicities */
#define BUTTON_1_MONITOR_PERIOD         50
#define BUTTON_2_MONITOR_PERIOD         50
#define PERIODIC_TRANSMITTER_PERIOD     100
#define UART_RECEIVER_PERIOD            20
#define LOAD_1_SIMULATION_PERIOD        10
#define LOAD_2_SIMULATION_PERIOD        100
#define MAX_MESSAGE_SIZE								16


/* Handlers */
TaskHandle_t Button_1_Monitor_TaskHandler = NULL;
TaskHandle_t Button_2_Monitor_TaskHandler = NULL;
TaskHandle_t Periodic_Transmitter_TaskHandler = NULL;
TaskHandle_t Uart_Receiver_TaskHandler = NULL;
TaskHandle_t Load_1_Simulation_TaskHandler = NULL;
TaskHandle_t Load_2_Simulation_TaskHandler = NULL;
QueueHandle_t xQueue = NULL;;


/* Global Variables to help in Tracing and Analysis */
int Button_1_Monitor_in_time = 0, Button_1_Monitor_out_time = 0, Button_1_Monitor_total_time = 0;
int Button_2_Monitor_in_time = 0, Button_2_Monitor_out_time = 0, Button_2_Monitor_total_time = 0;
int Periodic_Transmitter_Monitor_in_time = 0, Periodic_TransmitterMonitor_out_time = 0, Periodic_Transmitter_Monitor_total_time = 0;
int Uart_Receiver_in_time = 0, Uart_Receiver_out_time = 0, Uart_Receiver_Monitor_total_time = 0;
int Load_1_Simulation_in_time = 0, Load_1_Simulation_out_time = 0, Load_1_Simulation_total_time = 0;
int Load_2_Simulation_in_time = 0, Load_2_Simulation_out_time = 0, Load_2_Simulation_total_time = 0;
int System_Time = 0;
int CPU_Load = 0;


/* Task to be created. */
void Button_1_Monitor( void * pvParameters )
{
		TickType_t  xLastWakeTime = xTaskGetTickCount();
		pinState_t  prevButton_State = PIN_IS_LOW;
		pinState_t  currButton_State;
		char *sentMsg = NULL_PTR;
		vTaskSetApplicationTaskTag(NULL,(void *) 1);
    for( ;; )
    {
			/* Read the current button */
			currButton_State = GPIO_read(PORT_0, PIN0);
			/* Check if change happened from Rising to Falling */
			if(currButton_State != prevButton_State && currButton_State == PIN_IS_HIGH)
			{
				/* Send a message indicating rising edge to the Queue */
				if( xQueue != 0 && uxQueueSpacesAvailable(xQueue) != 0 )
				{
					sentMsg = "B1 RISING EDGE\n";
					xQueueSend(xQueue, ( void * )&sentMsg, ( TickType_t )0 );
				}				
			}
			/* Check if change happened from Falling to Rising */
			else if(currButton_State != prevButton_State && currButton_State == PIN_IS_LOW)
			{
				/* Send a message indicating falling edge to the Queue */
				if( xQueue != 0 && uxQueueSpacesAvailable(xQueue) != 0 )
				{
					sentMsg = "B1 FALLING EDGE\n";
					xQueueSend(xQueue, ( void * )&sentMsg, ( TickType_t )0 );
				}				
			}
			prevButton_State = currButton_State;
			vTaskDelayUntil(&xLastWakeTime, BUTTON_1_MONITOR_PERIOD);
			/* Idle Task Tracing */
			GPIO_write (PORT_0, PIN9, PIN_IS_LOW);
		}
		
}


void Button_2_Monitor( void * pvParameters )
{
		TickType_t  xLastWakeTime = xTaskGetTickCount();
		pinState_t  prevButton_State = PIN_IS_LOW;
		pinState_t  currButton_State;
		char *sentMsg = NULL_PTR;
		vTaskSetApplicationTaskTag(NULL,(void *) 2);
    for( ;; )
    {
			/* Read the current button state */
			currButton_State = GPIO_read(PORT_0, PIN1);
			/* Check if change happened from Rising to Falling */
			if(currButton_State != prevButton_State && currButton_State == PIN_IS_HIGH)
			{
				/* Send a message indicating rising edge to the Queue */
				if( xQueue != 0 && uxQueueSpacesAvailable(xQueue) != 0 )
				{
					sentMsg = "B2 RISING EDGE\n";
					xQueueSend(xQueue, ( void * )&sentMsg, ( TickType_t )0 );
				}				
			}
			/* Check if change happened from Falling to Rising */
			else if(currButton_State != prevButton_State && currButton_State == PIN_IS_LOW)
			{
				/* Send a message indicating falling edge to the Queue */
				if( xQueue != 0 && uxQueueSpacesAvailable(xQueue) != 0 )
				{
					sentMsg = "B2 FALLING EDGE\n";
					xQueueSend(xQueue, ( void * )&sentMsg, ( TickType_t )0 );
				}				
			}
			prevButton_State = currButton_State;
			vTaskDelayUntil(&xLastWakeTime, BUTTON_2_MONITOR_PERIOD);
			/* Idle Task Tracing */
			GPIO_write (PORT_0, PIN9, PIN_IS_LOW);
		}
		
}

void Periodic_Transmitter( void * pvParameters )
{
		TickType_t  xLastWakeTime = xTaskGetTickCount();
		char *Sent_message = NULL_PTR;
		
		vTaskSetApplicationTaskTag(NULL,(void *) 3);
	
    for( ;; )
    {
			/* Check if the queue is not full and Send message to the queue */
			if( xQueue != 0 && uxQueueSpacesAvailable(xQueue) != 0 )
			{
				Sent_message = "TRANSMITTING..\n";
				xQueueSend(xQueue, ( void * )&Sent_message, ( TickType_t )0 );
			}
			/* Delay the task for PERIODIC_TRANSMITTER_PERIOD */
			vTaskDelayUntil(&xLastWakeTime, PERIODIC_TRANSMITTER_PERIOD);
			/* Idle Task Tracing */
			GPIO_write (PORT_0, PIN9, PIN_IS_LOW);
		}
}

void Uart_Receiver( void * pvParameters )
{
		TickType_t  xLastWakeTime = xTaskGetTickCount();
		char *Received_message = NULL_PTR;
		vTaskSetApplicationTaskTag(NULL,(void *) 4);
	
    for( ;; )
    {
			/* Check if queue is not empty and receive a message */
			if( xQueueReceive( xQueue, &( Received_message ), ( TickType_t ) 0 ) == pdPASS )
			{
				/* Send the received messages from the queue through the UART serial */
				vSerialPutString((const signed char*)Received_message, MAX_MESSAGE_SIZE);
			}
			vTaskDelayUntil(&xLastWakeTime, UART_RECEIVER_PERIOD);
			/* Idle Task Tracing */
			GPIO_write (PORT_0, PIN9, PIN_IS_LOW);
		}
}

void Load_1_Simulation( void * pvParameters )
{
		int i = 0;
		TickType_t  xLastWakeTime = xTaskGetTickCount();
	
		vTaskSetApplicationTaskTag(NULL,(void *) 5);
    for( ;; )
    {
			/* Task code goes here. */
			/* Task execution time is 5ms */
			for(i=0;i<37000;i++)
			{
				/*Simulate heavy load*/
			}
			vTaskDelayUntil(&xLastWakeTime, LOAD_1_SIMULATION_PERIOD);
			/* Idle Task Tracing */
			GPIO_write (PORT_0, PIN9, PIN_IS_LOW);
		}
}

void Load_2_Simulation( void * pvParameters )
{
		int i = 0;
		TickType_t  xLastWakeTime = xTaskGetTickCount();
	
		vTaskSetApplicationTaskTag(NULL,(void *) 6);
    for( ;; )
    {
			/* Task code goes here. */
			/* Task execution time is 12ms */
			for(i=0;i<90000;i++)
			{
				/*Simulate heavy load*/
			}
			vTaskDelayUntil(&xLastWakeTime, LOAD_2_SIMULATION_PERIOD);
			/* Idle Task Tracing */
			GPIO_write (PORT_0, PIN9, PIN_IS_LOW);
		}
}




/* tick hook callback function */
void vApplicationTickHook( void )
{
	GPIO_write (PORT_0, PIN8, PIN_IS_HIGH);
	GPIO_write (PORT_0, PIN8, PIN_IS_LOW);
}

/* idle task callback function */
void vApplicationIdleHook( void )
{
	GPIO_write (PORT_0, PIN9, PIN_IS_HIGH);
}

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
								
	xTaskCreatePeriodic(
										Button_1_Monitor,      								/* Function that implements the task. */
                    "Button 1 Monitor",        						/* Task Name. */
                    100,      														/* Stack size in words, not bytes. */
                    ( void * ) 0,    											/* Parameter passed into the task. */
                    1,																		/* Priority at which the task is created. */
                    &Button_1_Monitor_TaskHandler,				/* Task Handler. */
										BUTTON_1_MONITOR_PERIOD); 						/* Used to pass out the created task's handle. */

	xTaskCreatePeriodic(
										Button_2_Monitor,      								/* Function that implements the task. */
                    "Button 2 Monitor",        						/* Task Name. */
                    100,      														/* Stack size in words, not bytes. */
                    ( void * ) 0,    											/* Parameter passed into the task. */
                    1,																		/* Priority at which the task is created. */
                    &Button_2_Monitor_TaskHandler,				/* Task Handler */
										BUTTON_2_MONITOR_PERIOD); 						/* Used to pass out the created task's handle. */
										
	xTaskCreatePeriodic(
										Periodic_Transmitter,      						/* Function that implements the task. */
                    "Periodic Transmitter",        				/* Task Name. */
                    100,      														/* Stack size in words, not bytes. */
                    ( void * ) 0,    											/* Parameter passed into the task. */
                    1,																		/* Priority at which the task is created. */
                    &Periodic_Transmitter_TaskHandler,		/* Task Handler. */
										PERIODIC_TRANSMITTER_PERIOD); 				/* Used to pass out the created task's handle. */
										
	xTaskCreatePeriodic(
										Uart_Receiver,      									/* Function that implements the task. */
                    "Uart Receiver",        							/* Task Name. */
                    100,      														/* Stack size in words, not bytes. */
                    ( void * ) 0,    											/* Parameter passed into the task. */
                    1,																		/* Priority at which the task is created. */
                    &Uart_Receiver_TaskHandler,						/* Task Handler. */
										UART_RECEIVER_PERIOD); 								/* Used to pass out the created task's handle. */
	
	xTaskCreatePeriodic(
										Load_1_Simulation,      							/* Function that implements the task. */
                    "Load 1 Simulation",        					/* Task Name. */
                    100,      														/* Stack size in words, not bytes. */
                    ( void * ) 0,    											/* Parameter passed into the task. */
                    1,																		/* Priority at which the task is created. */
                    &Load_1_Simulation_TaskHandler,				/* Task Handler. */
										LOAD_1_SIMULATION_PERIOD); 						/* Used to pass out the created task's handle. */
										
	xTaskCreatePeriodic(
										Load_2_Simulation,      							/* Function that implements the task. */
                    "Load 2 Simulation",        					/* Task Name. */
                    100,      														/* Stack size in words, not bytes. */
                    ( void * ) 0,    											/* Parameter passed into the task. */
                    1,																		/* Priority at which the task is created. */
                    &Load_2_Simulation_TaskHandler,				/* Task Handler. */
										LOAD_2_SIMULATION_PERIOD); 						/* Used to pass out the created task's handle. */		

	xQueue = xQueueCreate(3, sizeof(unsigned char[MAX_MESSAGE_SIZE]));
										

	/* Now all the tasks have been started - start the scheduler.
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

static void ConfigTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();

	/* Configure trace timer 1 and read TITC to get current tick */
	ConfigTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/
