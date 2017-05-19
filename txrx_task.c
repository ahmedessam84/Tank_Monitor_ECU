
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/can.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "cancom.h"
#include "utils/uartstdio.h"
#include "apptypes.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


//*****************************************************************************
//
// A flag to indicate that some transmission error occurred.
//
//*****************************************************************************

extern volatile bool g_bErrFlag;
extern volatile bool g_bRXFlag;

/*****************************************************************************/
//
// Queue to hold the data received from the CAN Network
// 
QueueHandle_t xQueueData;

TaskHandle_t Rx_Task_Handle = NULL;
TaskHandle_t Tx_Task_Handle = NULL;

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

uint32_t Rx_Task_Init( void )
{

		// Create a queue capable of containing 1 Node_t data type.
		// this queue will be used to send the sensor data received from the CAN Network to other tasks that need the info
		xQueueData = xQueueCreate( 1, sizeof( Node_t ) );

	
		// Rx task has to be of highest priority to force the task to run after an interrupt occures
		// this will be as if the interrupt did the processing of the task
		if( xTaskCreate( Rx_Task, "Rx_Task", 100, NULL, RX_PRIORITY, &Rx_Task_Handle ) == pdTRUE )
		{
			return (1);
		}
		else return (0);
	
	
}	

uint32_t Tx_Task_Init( void )
{
	
	
	
		
		if( xTaskCreate( Tx_Task, "Tx_Task", 100, NULL, TX_PRIORITY, &Tx_Task_Handle ) == pdTRUE )
		{
				return (1);
		}
		else return (0);
	
}
