
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
#include "display_task.h"

TaskHandle_t Display_Task_Handle = NULL;

extern AppMode_t g_eAppMode;
extern xSemaphoreHandle g_pLCDSemaphore;
extern QueueHandle_t xQueueData;
extern uint32_t ui32TickCounter;
extern void itoa_new(uint16_t , char * );
extern QueueHandle_t xQueuebtns;

void Display_Calendar( void );
void Display_Sensors( void );
void Display_Setup_Data( void );

//*********************************************************************
//
// Display_Task is responsible for handling the LCD screen for any display
// requirments by the application
// currently the application requirements are to display a normal mode screen
// and a setup mode screen. in normal mode the sensor data along with the
// calendar are displayed. In setup mode the calendar along with the tank 
// dimensions are displayed.
//
//*********************************************************************

static void Display_Task( void * pvparameters )
{

	TickType_t xLastWakeTime;
	
	// Initialize the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
	
	while(1)
	{
		
		switch(g_eAppMode)
		{
			// normal application mode display
			case NORMAL:
			{
				// Display normal screen template
				LcdNormalScreenBase();
				
				// Display Calendar
				Display_Calendar();
				
				// Display Sensors
				Display_Sensors();

				// done
				break;
				
			}
		
			// Setup application mode display	
			case SETUP:
			{
				// Display Setup screen NOTE: modification of parameters are done by ButtonsPoll_Task() 
				Display_Setup_Data();
				
				break;
				
			}
			
		}
		
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(100) );
		
	}
	
}

//***********************************************************************************
//
// function to initialize and create Display_Task
//
//***********************************************************************************
uint32_t Display_Task_Init( void )
{
	
	LcdMr_Init();
	
	if( xTaskCreate( Display_Task, "Display_Task", 100, NULL, DISPLAY_PRIORITY, &Display_Task_Handle ) == pdTRUE)
	{
	  //
	  // success
	  //
		return(1);
	}
	
	//
	// fail
	//
	else
	{ 
		return(0);
	}

}


//*****************************************************************************************
//
//	Function to display the calendar on the first line of the LCD screen
//
//*****************************************************************************************
static void Display_Calendar( void )
{
		struct tm calendar;

		char cHr[2];
		char cMin[2];
		char cSec[2];
		char cDay[2];
		char cMonth[2];
		char cYear[2];
	
		// Get date from rtc hardware
		Rtc_GetDate( &calendar );		
		
		CONVERT_LCD_CALENDAR_TO_ASCII(calendar);
	
		// protect the data displayed
		xSemaphoreTake( g_pLCDSemaphore, portMAX_DELAY );
		
		DISP_LCD_CALENDAR_NORMAL();
		
		xSemaphoreGive( g_pLCDSemaphore );
		
}

//*****************************************************************************************
//
//	Function to display Sensor Data on LCD
//
//*****************************************************************************************

static void Display_Sensors( void )
{
	
	BaseType_t xMsgReceived = pdFALSE;
	Node_t node_disp;	
	uint16_t ui16T1SensorData;
	char asciiSensorReading[6];
	
	// this is an array to hold the time to calculate sensor timeouts
	static uint32_t SensorTimeout[6] = {0};
	
	// wait for data to be received from RX_Task, do not block if data was not recieved
	xMsgReceived = xQueueReceive( xQueueData, &node_disp, 0 );
	
	// If data was received, find out which sensor sent the data
	if( xMsgReceived == pdTRUE )
	{
		// Store sensor data
		ui16T1SensorData = *((uint16_t *)node_disp.node_data_ptr);		
		
		// Convert sensor data to ascii
		itoa_new( ui16T1SensorData, asciiSensorReading );
		
		// Send data recieved to the correct position on the LCD
		// Reset timeout data for sensor that sent the data
		switch(node_disp.node_nr)
		{
			case 1: LcdT1( asciiSensorReading ); SensorTimeout[0] = ui32TickCounter; break;
			case 2: LcdT2( asciiSensorReading ); SensorTimeout[1] = ui32TickCounter; break;
			case 3: LcdT3( asciiSensorReading ); SensorTimeout[2] = ui32TickCounter; break;
			case 4: LcdT4( asciiSensorReading ); SensorTimeout[3] = ui32TickCounter; break;
			case 5: LcdT5( asciiSensorReading ); SensorTimeout[4] = ui32TickCounter; break;
			case 6: LcdT6( asciiSensorReading ); SensorTimeout[5] = ui32TickCounter; break;
		}
		
	}
	
	// find out if sensors corresponding to other gasoline tanks have timed out
	int i;
	for( i=0; i<6; i++ )
	{
		// if sensor times out display NA
		if( ( ui32TickCounter - SensorTimeout[i] ) >= SENSOR_TIMEOUT_PERIOD )
		{
			switch(i+1)
			{
				case 1: LcdT1( "NA   " ); break;
				case 2: LcdT2( "NA   " ); break;
				case 3: LcdT3( "NA   " ); break;
				case 4: LcdT4( "NA   " ); break;
				case 5: LcdT5( "NA   " ); break;
				case 6: LcdT6( "NA   " ); break;
			}
			
		}

	}
		
}
			
		
//*****************************************************************************************
//
//	Function to display Setup Data on LCD
//
//*****************************************************************************************	

static void Display_Setup_Data()
{
	static SetupScreen_t DispSetupParams;
	static bool first_time_flag = TRUE;
	
	char cHr[2];
	char cMin[2];
	char cSec[2];
	char cDay[2];
	char cMonth[2];
	char cYear[2];
	char cRadius_buffer[6];
	char cLength_buffer[6];

	// recieve the updated display parameters from the ButtonsPoll_Task()
	xQueueReceive( xQueuebtns, &DispSetupParams, (TickType_t) 0 );
	
	// convert the edited values into ascii
	CONVERT_LCD_PARAM_TO_ASCII( DispSetupParams );
	
	// If this is the first time to enter setup, display all setup values on the screen for the first time only
	if( first_time_flag == TRUE )
	{
		// Display the current setup parameters values stored in memory
		LCD_Clear();
		
		DISP_LCD_INITIAL_VALUES_SETUP();	
		
		LCD_DispCursor();
		
		first_time_flag = FALSE;
		
	}
	
	// if this is not the first time to run this code then update only the value that we are changing using buttons
	// which are indicated by the cursor
	else
	{
		//
		// update the display of the parameter currently being modified
		//
		switch(DispSetupParams.cursor)
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
	
	//
	// if setup mode was terminated by locking back the screen, the condition below must be true
	//
	if( (DispSetupParams.Unlocked == FALSE) && ( first_time_flag == FALSE) ) 
	{
		//**************EXIT SETUP MODE PROCEDURES************//
		
		//
		// Apply changes we made in the setup mode to RTC Hardware
		//
		Rtc_SetDate( &DispSetupParams.calendar );
		
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
		// change application mode back to NORMAL state
		//
		g_eAppMode = NORMAL;

		
	}
	else{};
		
}
