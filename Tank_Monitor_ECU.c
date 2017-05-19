//*****************************************************************************
//
// freertos_demo.c - Simple FreeRTOS example.
//
// Copyright (c) 2012-2016 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.3.156 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/can.h"
#include "utils/uartstdio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "cancom.h"
#include "lcdmr.h"
#include "rtc.h"
#include "btnmr.h"
#include "LCD20x04.h"
#include "apptypes.h"
#include "buttonspoll_task.h"
#include "display_task.h"

//*****************************************************************************
//
// A counter that keeps track of the number of times the TX interrupt has
// occurred, which should match the number of TX messages that were sent.
//
//*****************************************************************************

volatile uint32_t g_ui32MsgCount = 0;

//*****************************************************************************
//
// A flag to indicate that some transmission error occurred.
//
//*****************************************************************************

volatile bool g_bErrFlag = 0;
volatile bool g_bRXFlag = 0;

/*****************************************************************************/

void vApplicationTickHook( void );	
volatile uint32_t ui32TickCounter;	

AppMode_t g_eAppMode = NORMAL;

/*****************************************************************************/
//
// Queue to hold the data received from the CAN Network
// 

QueueHandle_t xQueueData;

extern QueueHandle_t xQueuebtns;

QueueHandle_t xQueueCalendar;

TaskHandle_t Rx_Task_Handle = NULL;
TaskHandle_t Tx_Task_Handle = NULL;
TaskHandle_t SetupScreen_Task_Handle = NULL;


/*****************************************************************************/
//*****************************************************************************
//
// The mutex that protects concurrent access of UART from multiple tasks.
//
//*****************************************************************************

xSemaphoreHandle g_pLCDSemaphore;
xSemaphoreHandle g_pRTCSemaphore;

TickType_t g_xSensorTimeOut = pdMS_TO_TICKS(5000);

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}

#endif

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void
vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}


//*****************************************************************************

void itoa_new(uint16_t val, char * str)
{
		uint32_t temp_val;
		temp_val = val;
		str[0] = (temp_val / 10000) + 48;
		temp_val = (temp_val % 10000);
		str[1] = (temp_val / 1000) + 48;
		temp_val = (temp_val % 1000);
		str[2] = (temp_val / 100) + 48;
		temp_val = (temp_val % 100);
		str[3] =  (temp_val / 10) + 48;
		temp_val = (temp_val % 10);
		str[4] = temp_val + 48;
		str[5] = '\0';
}

//*****************************************************************************
//
// Configure the UART and its pins. This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
		
}

//*****************************************************************************
//
// This function is the interrupt handler for the CAN peripheral.  It checks
// for the cause of the interrupt, and maintains a count of all messages that
// have been transmitted.
//
//*****************************************************************************
extern void
CANIntHandler(void)
{
    uint32_t ui32Status;
	
		BaseType_t xHigherPriorityTaskWoken;
	
	/* xHigherPriorityTaskWoken must be initialised to pdFALSE.  If calling
    vTaskNotifyGiveFromISR() unblocks the handling task, and the priority of
    the handling task is higher than the priority of the currently running task,
    then xHigherPriorityTaskWoken will automatically get set to pdTRUE. */
	
		xHigherPriorityTaskWoken = pdFALSE;
	
	
    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ui32Status == CAN_INT_INTID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.  If the
        // CAN peripheral is not connected to a CAN bus with other CAN devices
        // present, then errors will occur and will be indicated in the
        // controller status.
        //
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        //
        // Set a flag to indicate some errors may have occurred.
        //
        g_bErrFlag = 1;
			
				//UARTprintf(" CAN status %04X\n", ui32Status);
				
    }

    //
    // Check if the cause is message object 1, which what we are using for
    // sending messages.
    // the number indicated in ui32Status is for the message object id in case of msg RX or TX
		
    else if(ui32Status == 1)
    {
        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 1);

			  //
        // Set flag to indicate received message is pending.
        //
        g_bRXFlag = 1;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlag = 0;
			
				vTaskNotifyGiveFromISR( Rx_Task_Handle, &xHigherPriorityTaskWoken );
			
				/* Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE.
				The macro used to do this is dependent on the port and may be called 
				*/
				portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
				
    }
		//
		// message object 2 is used for transmission, when the cause of interrupt is msg obj 2 it means a msg was transmitted
		//
		else if(ui32Status == 2)
		{
			CANIntClear(CAN0_BASE, 2);
		}

    //
    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }
}

//*********************************************************************************
//
// Sends sync signal to all slaves on the CAN network								
//
//*********************************************************************************

static void Tx_Task(void *pvParameters)
{
	TickType_t xLastWakeTime;	

  // Initialize the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
	
	while(1)
	{

		// Transmit SYNC signal to sensors every Heartbeat = 2s
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( HEARTBEAT ) );	
		UARTprintf("Sending Sync Msg...\n");
		TransmitSync();		
	
	}
	
}

//*********************************************************************************
//
// Receives data from sensors
//
//*********************************************************************************

static void Rx_Task(void *pvParameters)
{
	const TickType_t xBlockTime = 4000;	// block time set to 4 sec
	
	Node_t node;
	CANCom_Master_Init();
	
	while(1)
	{
		/* As before, block to wait for a notification from the ISR. This
		time however the first parameter is set to pdTRUE, clearing the task's
		notification value to 0, meaning each outstanding deferred
		interrupt event must be processed before ulTaskNotifyTake() is called
		again. */
	
		
		ulTaskNotifyTake( pdTRUE, xBlockTime );
		// Wait for receiving the response from sensor

		if(g_bRXFlag)
		{
			
				ReceiveData( &node );
			
				g_bRXFlag = 0;

				// Then send the msg received to sensor queue
				// Do not block if queue is full
				xQueueSend( xQueueData, &node, ( TickType_t ) 0);
				
		}
		else if(g_bErrFlag == 1)
		{
			UARTprintf(" error\n");
			g_bErrFlag = 0;

		}
			
	}
}


//*****************************************************************************
//
// Initialize FreeRTOS and start the initial set of tasks.
//
//*****************************************************************************
int
main(void)
{
    //
    // Set the clocking to run at 80 MHz from the PLL.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    //
    // Initialize the UART and configure it for 115,200, 8-N-1 operation.
    //
    ConfigureUART();
	
		Rtc_Init();
		LcdMr_Init();
		
	
    //
    // Print demo introduction.
    //
    UARTprintf("\n\nWelcome to the EK-TM4C123GXL Underground Tank Monitor!\n");


		
/************************************************************************/

    //
    // Create a mutex:
    //
    g_pLCDSemaphore = xSemaphoreCreateMutex();
		g_pRTCSemaphore = xSemaphoreCreateMutex();

		// Create a queue capable of containing 1 uint16_t values.
		// this queue will be used to receive the sensor data received from the CAN Network
		xQueueData = xQueueCreate( 1, sizeof( Node_t ) );

		xTaskCreate( Tx_Task, "Tx_Task", 100, NULL, 11, &Tx_Task_Handle ); 
		
		// Rx task has to be of highest priority to force the task to run after an interrupt occures
		// this will be as if the interrupt did the processing of the task
		xTaskCreate( Rx_Task, "Rx_Task", 100, NULL, 12, &Rx_Task_Handle );
		


		// create ButtonsPoll_Task
		if(ButtonsPoll_Task_Init() != TRUE)
		{
			while(1)
			{
				// Loop here if task was not created
			}
		}
		
		// create Display_Task
		if(Display_Task_Init() != TRUE)
		{
			while(1)
			{
				// Loop here if task was not created
			}
		}
		
    //
    // Start the scheduler.  This should not return.
    //
    vTaskStartScheduler();

    //
    // In case the scheduler returns for some reason, print an error and loop
    // forever.
    //
    while(1)
    {
    }
		
}


void vApplicationTickHook( void )
{
	ui32TickCounter++;
}


