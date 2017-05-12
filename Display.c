
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

extern SemaphoreHandle_t g_pLCDSemaphore;
extern void itoa_new(uint16_t , char *);

extern QueueHandle_t xQueueData1;
extern QueueHandle_t xQueueData2;
extern QueueHandle_t xQueueData3;
extern QueueHandle_t xQueueData4;
extern QueueHandle_t xQueueData5;
extern QueueHandle_t xQueueData6;


extern TickType_t g_xSensorTimeOut;



void DisplayCal( void )
{
	
	struct tm calendar;

	char cHr[2];
	char cMin[2];
	char cSec[2];
	char cDay[2];
	char cMonth[2];
	char cYear[2];
	
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

}
		
/*
void DisplaySensorData( void )
{
	
	BaseType_t xMsgReceived = pdFALSE;
	Node_t node1_disp;	
	uint16_t ui16T1SensorData;
	char t1[6];

	// Wait for the queue to be full
	xMsgReceived = xQueueReceive( xQueueData1, &node1_disp, g_xSensorTimeOut );		
	
	if( xMsgReceived == pdTRUE )
	{
		ui16T1SensorData = *((uint16_t *)node1_disp.node_data_ptr);	
		itoa_new( ui16T1SensorData, t1);
	}	
	else
	{
		strncpy(t1, "NA   ", 6);
	}
	
	// check which node sent the message
	// update the screen accordingly
	switch( node1_disp.node_nr )
	{
		case 1: LcdT1( t1 ); break;
		case 2: LcdT2( t1 ); break;
		case 3: LcdT3( t1 ); break;
		case 4: LcdT4( t1 ); break;
		case 5: LcdT5( t1 ); break;
		case 6: LcdT6( t1 ); break;
	}
	
	
	// protect the data displayed
	xSemaphoreTake( g_pLCDSemaphore, portMAX_DELAY );
	LcdT1( t1 );
	xSemaphoreGive( g_pLCDSemaphore );
	
	
}
*/
