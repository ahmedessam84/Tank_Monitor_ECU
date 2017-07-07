
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "btnmr.h"
#include "apptypes.h"
#include "rtc.h"

void RollOverAdjust( SetupScreen_t *);
extern AppMode_t g_eAppMode;
QueueHandle_t xQueuebtns;
TaskHandle_t ButtonsPoll_Task_Handle = NULL;



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
	
	SetupScreen_t setup;
	setup.Unlocked = FALSE;
	setup.Tank.length = 23000;
	setup.Tank.radius = 23000;
	setup.cursor = HOUR;
	
	while(1)
	{
		
		// If buttons are locked keep updating the calendar setup parameters
		if( (setup.Unlocked == FALSE) && (g_eAppMode == NORMAL) )
		{
			// check if buttons are unlocked
			BtnMr( NULL, NULL, &setup.Unlocked );
		
			Rtc_GetDate( &setup.calendar );
			
		}

		// If buttons are unlocked then the setup parameters can be modified
		else
		{
			// change app mode to setup
			g_eAppMode = SETUP;
	
			// reset seconds to zero whenever we enter setup
			setup.calendar.tm_sec = 0;

			// change the parameter according to the position of the cursor
			switch(setup.cursor)
			{
				case HOUR: 		BtnMr( &setup.cursor, &setup.calendar.tm_hour, &setup.Unlocked ); break;
				case MIN:			BtnMr( &setup.cursor, &setup.calendar.tm_min, &setup.Unlocked ); break;
				
				case DAY:			BtnMr( &setup.cursor, &setup.calendar.tm_mday, &setup.Unlocked ); break;
				case MONTH:		BtnMr( &setup.cursor, &setup.calendar.tm_mon, &setup.Unlocked ); break;
				case YEAR:		BtnMr( &setup.cursor, &setup.calendar.tm_year, &setup.Unlocked ); break;
				
				case RADIUS:	BtnMr( &setup.cursor, &setup.Tank.radius, &setup.Unlocked ); break;
				case LENGTH:	BtnMr( &setup.cursor, &setup.Tank.length, &setup.Unlocked ); break; 
			}

			// adjust rollover
			RollOverAdjust( &setup );
		
		}
		

		//
		// Send the updated calendar values into the queue to be received by the setup display task
		// which runs at a lower priority
		//
		xQueueSend( xQueuebtns, &setup, (TickType_t) 0 );

		//
		// Block the task for 1ms for Three reasons:
		// 1: Poll the buttons periodically every 1ms
		// 2: Allow other tasks to run during this 1ms
		// 3: For debounce routine to run properly there has to be atleast a 1ms delay between each call
		//    for the the function
		//
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 20 ) );
		
		
	}
	
}

uint32_t ButtonsPoll_Task_Init( void )
{
	
	BtnMr_Init();
	xQueuebtns = xQueueCreate( 1, sizeof( SetupScreen_t ) );


	if( xTaskCreate( ButtonsPoll_Task, "ButtonsPoll_Task", 100, NULL, BUTTONS_PRIORITY, &ButtonsPoll_Task_Handle ) == pdTRUE)
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


static void RollOverAdjust( SetupScreen_t * setup )
{
	
	// under test implementation to subsitute the code below	
		setup->calendar.tm_min %= 60;
		setup->calendar.tm_hour %= 24;
		setup->calendar.tm_mday %= 32;
		setup->calendar.tm_mon %= 12;
		setup->cursor = (cursor_t)((setup->cursor % 7) + 1);
	
  	/*
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
		*/
		
}
