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

#define TRUE 		1
#define FALSE 	0

typedef struct
{
	int radius;
	int length;
}TankDim_t;

typedef struct
{
	TankDim_t Tank;
	struct tm calendar;
	bool Unlocked;
	cursor_t cursor;
}SetupScreen_t;


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

void RollOverAdjust( SetupScreen_t *);

/*****************************************************************************/
//
// Queue to hold the data received from the CAN Network
// 
QueueHandle_t xQueueData1;
QueueHandle_t xQueueData2;
QueueHandle_t xQueueData3;
QueueHandle_t xQueueData4;
QueueHandle_t xQueueData5;
QueueHandle_t xQueueData6;
QueueHandle_t xQueuebtns;
QueueHandle_t xQueueCalendar;

TaskHandle_t Rx_Task_Handle = NULL;
TaskHandle_t Tx_Task_Handle = NULL;
TaskHandle_t Display_Task_Handle = NULL;
TaskHandle_t DispCal_Task_Handle = NULL;
TaskHandle_t DisplayT1_Task_Handle = NULL;
TaskHandle_t DisplayT2_Task_Handle = NULL;
TaskHandle_t DisplayT3_Task_Handle = NULL;
TaskHandle_t DisplayT4_Task_Handle = NULL;
TaskHandle_t DisplayT5_Task_Handle = NULL;
TaskHandle_t DisplayT6_Task_Handle = NULL;
TaskHandle_t SetupScreen_Task_Handle = NULL;
TaskHandle_t ButtonsPoll_Task_Handle = NULL;

/*****************************************************************************/
//*****************************************************************************
//
// The mutex that protects concurrent access of UART from multiple tasks.
//
//*****************************************************************************

xSemaphoreHandle g_pLCDSemaphore;
xSemaphoreHandle g_pRTCSemaphore;
xSemaphoreHandle g_pSetupSemaphore;
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
// Configure the UART and its pins.  This must be called before UARTprintf().
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
// Sends the sync signal to all slaves on the CAN network								
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
// Receives data from sensor
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
			
				taskENTER_CRITICAL();
				ReceiveData( &node );
				taskEXIT_CRITICAL();
			
				g_bRXFlag = 0;
			
				//ulNotifiedValue = 0;
			
				// Then send the msg received to the queue corresponding to the correct sensor
				// Do not block if queue is full
				switch(node.node_nr)
				{
					case 1: xQueueSend( xQueueData1, &node, ( TickType_t ) 0); break;
					case 2: xQueueSend( xQueueData2, &node, ( TickType_t ) 0); break;
					case 3: xQueueSend( xQueueData3, &node, ( TickType_t ) 0); break;
					case 4: xQueueSend( xQueueData4, &node, ( TickType_t ) 0); break;
					case 5: xQueueSend( xQueueData5, &node, ( TickType_t ) 0); break;
					case 6: xQueueSend( xQueueData6, &node, ( TickType_t ) 0); break;
				}
			
		
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
//	Polls buttons every 1ms to get their debounced status
//	Fills a queue with the state of the buttons
//
//*****************************************************************************

static void ButtonsPoll_Task(void *pvparameters)
{
	
	TickType_t xLastWakeTime;
	// Initialize the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
	
	SetupScreen_t setup1;
	setup1.Unlocked = FALSE;
	setup1.Tank.length = 23000;
	setup1.Tank.radius = 23000;
	setup1.cursor = HOUR;
	
	BtnMr_Init();
	
	while(1)
	{
		
		// If buttons are locked keep updating the calendar setup parameters
		if( setup1.Unlocked == FALSE )
		{
			// check if button is unlocked
			BtnMr( NULL, NULL, &setup1.Unlocked );
		
			xSemaphoreTake( g_pRTCSemaphore, portMAX_DELAY );
			Rtc_GetDate( &setup1.calendar );
			xSemaphoreGive( g_pRTCSemaphore );
		
		}

		// If buttons are unlocked then the setup parameters can be modified
		else
		{	
			setup1.calendar.tm_sec = 0;

			switch(setup1.cursor)
			{
				case HOUR: 		BtnMr( &setup1.cursor, &setup1.calendar.tm_hour, &setup1.Unlocked ); break;
				case MIN:			BtnMr( &setup1.cursor, &setup1.calendar.tm_min, &setup1.Unlocked ); break;
				
				case DAY:			BtnMr( &setup1.cursor, &setup1.calendar.tm_mday, &setup1.Unlocked ); break;
				case MONTH:		BtnMr( &setup1.cursor, &setup1.calendar.tm_mon, &setup1.Unlocked ); break;
				case YEAR:		BtnMr( &setup1.cursor, &setup1.calendar.tm_year, &setup1.Unlocked ); break;
				
				case RADIUS:	BtnMr( &setup1.cursor, &setup1.Tank.radius, &setup1.Unlocked ); break;
				case LENGTH:	BtnMr( &setup1.cursor, &setup1.Tank.length, &setup1.Unlocked ); break; 
			}

			// adjust rollover
			RollOverAdjust( &setup1 );
		
		}
		

		//
		// Send the updated calendar values into the queue to be received by the setup display task
		// which runs at a lower priority
		//
		xQueueSend( xQueuebtns, &setup1, (TickType_t) 0 );

		//
		// Block the task for 1ms for Three reasons:
		// 1: Poll the buttons periodically every 1ms
		// 2: Allow other tasks to run during this 1ms
		// 3: For debounce routine to run properly there has to be atleast a 1ms delay between each call
		//    for the the function
		//
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 ) );
		
		
	}
	
}


//*****************************************************************************
//
// Setup Screen Display Task
//
//*****************************************************************************

static void SetupScreen_Task( void *pvparameters )
{
	
	SetupScreen_t DispSetupParams1;

	static bool first_time_flag = TRUE;
	char cHr[2];
	char cMin[2];
	char cSec[2];
	char cDay[2];
	char cMonth[2];
	char cYear[2];

	char cRadius_buffer[6];
	char cLength_buffer[6];
	
	while(1)
	{
		//
		// Block for 1 second until a button is pressed and the setup display parameters are updated
		// During this 1 second block, other tasks can run
		//
		xQueueReceive( xQueuebtns, &DispSetupParams1, (TickType_t) 1000 );

		// if the setup screen is unlocked then display the setup parameters and setup screen
		if(DispSetupParams1.Unlocked == TRUE )
		{
			
			// convert the edited values into ascii
			strftime( cHr, 3, "%H", &DispSetupParams1.calendar );
			strftime( cMin, 3, "%M", &DispSetupParams1.calendar );
			strftime( cSec, 3, "%S", &DispSetupParams1.calendar );
			strftime( cDay, 3, "%d", &DispSetupParams1.calendar );
			strftime( cMonth, 3, "%m", &DispSetupParams1.calendar );
			strftime( cYear, 3, "%y", &DispSetupParams1.calendar );
				
			// Convert radius and length into ascii
			itoa_new( DispSetupParams1.Tank.length, cLength_buffer );
			itoa_new( DispSetupParams1.Tank.radius, cRadius_buffer );
					
			// If this is the first time to run this code display all the values on the screen for the first time
			if( first_time_flag == TRUE )
			{
				// In setup mode suspend all tasks below except for buttons poll and setup display tasks
				
				vTaskSuspend( Rx_Task_Handle );
				vTaskSuspend( Tx_Task_Handle );
				vTaskSuspend( DispCal_Task_Handle );
				vTaskSuspend( DisplayT1_Task_Handle );
				vTaskSuspend( DisplayT2_Task_Handle );
				vTaskSuspend( DisplayT3_Task_Handle );
				vTaskSuspend( DisplayT4_Task_Handle );
				vTaskSuspend( DisplayT5_Task_Handle );
				vTaskSuspend( DisplayT6_Task_Handle );
				
				LCD_Clear();
				LcdSetupScreenBase();
				LcdHr(cHr);
				LcdMin(cMin);
				LcdSec(cSec);
				LcdDay(cDay);
				LcdMonth(cMonth);
				LcdYr(cYear);
			
				LcdRadius(cRadius_buffer);
				LcdLength(cLength_buffer);
				LCD_DispCursor();
				first_time_flag = FALSE;
				
			}
			
			// if this is not the first time to run this code then update only the value that we are changing using buttons
			else
			{
				
				switch(DispSetupParams1.cursor)
				{
					case HOUR: LcdHr(cHr); break;
					case MIN: LcdMin(cMin); break;
					case DAY: LcdDay(cDay); break;
					case MONTH: LcdMonth(cMonth); break;
					case YEAR: LcdYr(cYear); break;
					case RADIUS: LcdRadius(cRadius_buffer); break;
					case LENGTH: LcdLength(cLength_buffer); break;
				}
				
			}

		}
		
		//
		// Else if setup mode was terminated by locking back the screen, the condition below must be true
		//
		else if( (DispSetupParams1.Unlocked == FALSE) && ( first_time_flag == FALSE) ) 
		{
			//**************EXIT SETUP MODE PROCEDURES************//
			
			//
			// Apply changes we made in the setup mode to RTC Hardware
			//
			Rtc_SetDate( &DispSetupParams1.calendar );
			
			//
			// ON EXIT from the setup mode reset the flag back to TRUE
			//
			first_time_flag = TRUE;
		
			//
			// Hide Cursor
			//
			LCD_HideCursor();

			//
			// Clear LCD
			//
			LCD_Clear();
			
			//
		  // Resume normal operation and run all tasks back
			//
			vTaskResume( Rx_Task_Handle );
			vTaskResume( Tx_Task_Handle );
			vTaskResume( DispCal_Task_Handle );
			vTaskResume( DisplayT1_Task_Handle );
			vTaskResume( DisplayT2_Task_Handle );
			vTaskResume( DisplayT3_Task_Handle );
			vTaskResume( DisplayT4_Task_Handle );
			vTaskResume( DisplayT5_Task_Handle );
			vTaskResume( DisplayT6_Task_Handle );
			
		}
		else{};
		
	}

}

//******************************************************************************

static void DispCal_Task( void *pvparameters )
{
	
	struct tm calendar;

	char cHr[2];
	char cMin[2];
	char cSec[2];
	char cDay[2];
	char cMonth[2];
	char cYear[2];
	
	TickType_t xLastWakeTime;
	
	// Initialize the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
		
	while(1)
	{				
		
		//xSemaphoreTake( g_pSetupSemaphore, portMAX_DELAY );
		
		LcdNormalScreenBase();
		
		// Get date from register
		taskENTER_CRITICAL();
		Rtc_GetDate( &calendar );		
		taskEXIT_CRITICAL();
		
		// convert the edited values into ascii
		strftime( cHr, 3, "%H", &calendar );
		strftime( cMin, 3, "%M", &calendar );
		strftime( cSec, 3, "%S", &calendar );
		strftime( cDay, 3, "%d", &calendar );
		strftime( cMonth, 3, "%m", &calendar );
		strftime( cYear, 3, "%y", &calendar );
	
		// protect the data displayed
		xSemaphoreTake( g_pLCDSemaphore, portMAX_DELAY );
		
		LcdHr(cHr);
		LcdMin(cMin);
		LcdSec(cSec);
		LcdMonth(cMonth);
		LcdDay(cDay);
		LcdYr(cYear);
		
		xSemaphoreGive( g_pLCDSemaphore );
		
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(400) );
		
	}
	
}

//******************************************************************************

static void DisplayT1_Task( void *pvparameters )
{
	
	BaseType_t xMsg1Received = pdFALSE;
	Node_t node1_disp;	
	uint16_t ui16T1SensorData;
	
	char t1[6];
	
	
	while(1)
	{
		
		//xSemaphoreTake( g_pSetupSemaphore, portMAX_DELAY );
		
		// Wait for the queue to be full
		xMsg1Received = xQueueReceive( xQueueData1, &node1_disp, g_xSensorTimeOut );		
		
		if( xMsg1Received == pdTRUE )
		{
			ui16T1SensorData = *((uint16_t *)node1_disp.node_data_ptr);
		
			itoa_new( ui16T1SensorData, t1);
		}
		else
		{
			strncpy(t1, "NA   ", 6);
		}
			
		// protect the data displayed
		xSemaphoreTake( g_pLCDSemaphore, portMAX_DELAY );
		LcdT1( t1 );
		xSemaphoreGive( g_pLCDSemaphore );
		
	}
	
}

//******************************************************************************

static void DisplayT2_Task( void *pvparameters )
{
	
	BaseType_t xMsg2Received = pdFALSE;
	Node_t node2_disp;	
	uint16_t ui16T2SensorData;
	
	char t2[6];
	
	while(1)
	{

		//xSemaphoreTake( g_pSetupSemaphore, portMAX_DELAY );
		
		// Wait for the queue to be full
		xMsg2Received = xQueueReceive( xQueueData2, &node2_disp, g_xSensorTimeOut );		
		
		if( xMsg2Received == pdTRUE )
		{
			ui16T2SensorData = *((uint16_t *)node2_disp.node_data_ptr);
		
			itoa_new( ui16T2SensorData, t2);
		}
		else
		{
			strncpy(t2, "NA   ", 6);
		}
			
		// protect the data displayed
		xSemaphoreTake( g_pLCDSemaphore, portMAX_DELAY );
		LcdT2( t2 );
		xSemaphoreGive( g_pLCDSemaphore );
		
	}
		
}

//******************************************************************************

static void DisplayT3_Task( void *pvparameters )
{
	
	BaseType_t xMsg3Received = pdFALSE;
	Node_t node3_disp;	
	uint16_t ui16T3SensorData;
	
	char t3[6];
	
	
	while(1)
	{	

		//xSemaphoreTake( g_pSetupSemaphore, portMAX_DELAY );
		
		// Wait for the queue to be full
		xMsg3Received = xQueueReceive( xQueueData3, &node3_disp, g_xSensorTimeOut );		
		
		if( xMsg3Received == pdTRUE )
		{
			ui16T3SensorData = *((uint16_t *)node3_disp.node_data_ptr);
		
			itoa_new( ui16T3SensorData, t3);
		}
		else
		{
			strncpy(t3, "NA   ", 6);
		}
			
		// protect the data displayed
		xSemaphoreTake( g_pLCDSemaphore, portMAX_DELAY );		
		LcdT3( t3 );
		xSemaphoreGive( g_pLCDSemaphore );
		
	}
	
}

//******************************************************************************

static void DisplayT4_Task( void *pvparameters )
{
	
	BaseType_t xMsg4Received = pdFALSE;
	Node_t node4_disp;	
	uint16_t ui16T4SensorData;
	
	char t4[6];
	
	
	while(1)
	{
		
		//xSemaphoreTake( g_pSetupSemaphore, portMAX_DELAY );
		
		// Wait for the queue to be full
		xMsg4Received = xQueueReceive( xQueueData4, &node4_disp, g_xSensorTimeOut );		
		
		if( xMsg4Received == pdTRUE )
		{
			ui16T4SensorData = *((uint16_t *)node4_disp.node_data_ptr);
		
			itoa_new( ui16T4SensorData, t4);
		}
		else
		{
			strncpy(t4, "NA   ", 6);
		}
			
		// protect the data displayed
		xSemaphoreTake( g_pLCDSemaphore, portMAX_DELAY );
		LcdT4( t4 );
		xSemaphoreGive( g_pLCDSemaphore );
		
	}
	
}


//******************************************************************************

static void DisplayT5_Task( void *pvparameters )
{
	
	BaseType_t xMsg5Received = pdFALSE;
	Node_t node5_disp;	
	uint16_t ui16T5SensorData;
	
	char t5[6];
	
	
	while(1)
	{
		
		//xSemaphoreTake( g_pSetupSemaphore, portMAX_DELAY );
		
		// Wait for the queue to be full
		xMsg5Received = xQueueReceive( xQueueData5, &node5_disp, g_xSensorTimeOut );		
		
		if( xMsg5Received == pdTRUE )
		{
			ui16T5SensorData = *((uint16_t *)node5_disp.node_data_ptr);
		
			itoa_new( ui16T5SensorData, t5);
		}
		else
		{
			strncpy(t5, "NA   ", 6);
		}
			
		// protect the data displayed
		xSemaphoreTake( g_pLCDSemaphore, portMAX_DELAY );		
		LcdT5( t5 );
		xSemaphoreGive( g_pLCDSemaphore );
		
	}
	
}

//******************************************************************************
static void DisplayT6_Task( void *pvparameters )
{

	BaseType_t xMsg6Received = pdFALSE;
	Node_t node6_disp;	
	uint16_t ui16T6SensorData;
		
	char t6[6];
	
	
	while(1)
	{
		
		//xSemaphoreTake( g_pSetupSemaphore, portMAX_DELAY );
		
		// Wait for the queue to be full
		xMsg6Received = xQueueReceive( xQueueData6, &node6_disp, g_xSensorTimeOut );		
		
		if( xMsg6Received == pdTRUE )
		{
			ui16T6SensorData = *((uint16_t *)node6_disp.node_data_ptr);
		
			itoa_new( ui16T6SensorData, t6);
		}
		else
		{
			strncpy(t6, "NA   ", 6);
		}
			
		// protect the data displayed
		xSemaphoreTake( g_pLCDSemaphore, portMAX_DELAY );		
		LcdT6( t6 );		
		xSemaphoreGive( g_pLCDSemaphore );
		
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
		LcdMr_Init();
		Rtc_Init();
	
    //
    // Print demo introduction.
    //
    UARTprintf("\n\nWelcome to the EK-TM4C123GXL Underground Tank Monitor!\n");


		
/************************************************************************/

    //
    // Create a mutex to guard the LCD.
    //
    g_pLCDSemaphore = xSemaphoreCreateMutex();
		g_pRTCSemaphore = xSemaphoreCreateMutex();
		g_pSetupSemaphore = xSemaphoreCreateMutex();

		// Create a queue capable of containing 1 uint16_t values.
		// this queue will be used to receive the sensor data received from the CAN Network
		xQueueData1 = xQueueCreate( 1, sizeof( Node_t ) );
		xQueueData2 = xQueueCreate( 1, sizeof( Node_t ) );
		xQueueData3 = xQueueCreate( 1, sizeof( Node_t ) );
		xQueueData4 = xQueueCreate( 1, sizeof( Node_t ) );
		xQueueData5 = xQueueCreate( 1, sizeof( Node_t ) );
		xQueueData6 = xQueueCreate( 1, sizeof( Node_t ) );

		xQueuebtns = xQueueCreate( 1, sizeof( SetupScreen_t ) );


		xTaskCreate( Tx_Task, "Tx_Task", 100, NULL, 11, &Tx_Task_Handle ); 
		
		// Rx task has to be of highest priority to force the task to run after an interrupt occures
		// this will be as if the interrupt did the processing of the task
		xTaskCreate( Rx_Task, "Rx_Task", 100, NULL, 12, &Rx_Task_Handle );
		
		xTaskCreate( DispCal_Task, "DispCal_Task", 100, NULL, 7, &DispCal_Task_Handle );
		xTaskCreate( DisplayT1_Task, "DisplayT1_Task", 100, NULL, 6, &DisplayT1_Task_Handle );
		xTaskCreate( DisplayT2_Task, "DisplayT2_Task", 100, NULL, 5, &DisplayT2_Task_Handle );
		xTaskCreate( DisplayT3_Task, "DisplayT3_Task", 100, NULL, 4, &DisplayT3_Task_Handle );
		xTaskCreate( DisplayT4_Task, "DisplayT4_Task", 100, NULL, 3, &DisplayT4_Task_Handle );
		xTaskCreate( DisplayT5_Task, "DisplayT5_Task", 100, NULL, 2, &DisplayT5_Task_Handle );
		xTaskCreate( DisplayT6_Task, "DisplayT6_Task", 100, NULL, 1, &DisplayT6_Task_Handle );
		
		xTaskCreate( SetupScreen_Task, "SetupScreen_Task", 100, NULL, 9, &SetupScreen_Task_Handle );

		xTaskCreate( ButtonsPoll_Task, "ButtonsPoll_Task", 100, NULL, 10, &ButtonsPoll_Task_Handle );
		
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


void RollOverAdjust( SetupScreen_t * setup )
{
	
		// adjust calendar rollover upper boundry
		if( setup->calendar.tm_min > 59)
		{
			setup->calendar.tm_min = 0;
		}
		if( setup->calendar.tm_hour > 23)
		{
			setup->calendar.tm_hour = 0;
		}
		if( setup->calendar.tm_mday > 31)
		{
			setup->calendar.tm_mday = 1;
		}
		if( setup->calendar.tm_mon > 11 )
		{
			setup->calendar.tm_mon = 0;
		}
		
		// adjust cursor rollover
		if( setup->cursor > LENGTH )
		{
			setup->cursor = HOUR;
		}
		
		// adjust calendar rollover lower boundry
		if( setup->calendar.tm_min < 0)
		{
			setup->calendar.tm_min = 59;
		}
		if( setup->calendar.tm_hour < 0 )
		{
			setup->calendar.tm_hour = 23;
		}
		if( setup->calendar.tm_mday < 1 )
		{
			setup->calendar.tm_mday = 31;
		}
		if( setup->calendar.tm_mon < 0 )
		{
			setup->calendar.tm_mon = 11;
		}
		
		// adjust cursor rollover
		if( setup->cursor < HOUR )
		{
			setup->cursor = LENGTH;
		}
		
}
