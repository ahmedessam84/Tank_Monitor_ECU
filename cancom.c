
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

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

//***************************MyCANProtocol**********************************
// CAN protocol - Master transmits a SYNC msg all slave nodes receive the msg
// each node replies back with their data to the master
//
// Each node's ID consists of 11 bits the first Most significant three bits
// Indicate the type of sensor. The remaining 8 bits indicate the node number

//                         |----| |-------------|
// CAN ID 11-bits numbers: 10 9 8 7 6 5 4 3 2 1 0
//                         |type| |----number---|

//**************************************************************************
// NOTE: the node with the lowest identifier transmits more zeros at the start of the frame, 
// and that is the node that wins the arbitration or has the highest priority.
//
// the default SYNC time is set to 1ms
// each node replies back with (Node number)x(100us) from the time of reception of the SYNC
// Msg from the Master node.


	//
	// Master Msg to be sent
	//
	static tCANMsgObject txCANMessage;

	/*****************************************************************************/
	//
	// Data structure to be received
	//
	static tCANMsgObject rxCANMessage;

extern TaskHandle_t Rx_Task_Handle;


//*****************************************************************************
//
// A flag to indicate that some transmission error occurred.
//
//*****************************************************************************
volatile bool g_bErrFlag = 0;
volatile bool g_bRXFlag = 0;

//*****************************************************************************
//
// This function is the interrupt handler for the CAN peripheral.  It checks
// for the cause of the interrupt, and maintains a count of all messages that
// have been transmitted.
//
//*****************************************************************************
void
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



// Intializes the CAN module for Master node operation which requires:
// Master receives any msg on network
// Master sends a sync msg every 2s
//  
void CANCom_Master_Init(void)
{
	
	/*****************************************************************/		
	// CAN Hardware configuration
	/*****************************************************************/	
	  
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	GPIOPinConfigure(GPIO_PB4_CAN0RX);
	GPIOPinConfigure(GPIO_PB5_CAN0TX);

	GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

	//
	// The GPIO port and pins have been set up for CAN.  The CAN peripheral
	// must be enabled.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

	//
	// Initialize the CAN controller
	//
	CANInit(CAN0_BASE);
	
	CANBitRateSet( CAN0_BASE, SysCtlClockGet(), 125000 );
	
	//UARTprintf(" Baudrate set = %d\n", CANBitRateSet( CAN0_BASE, SysCtlClockGet(), 125000 ) );
	
	// Register interrupt in the vector table using dynamic addressing
	CANIntRegister(CAN0_BASE, CANIntHandler);
	
	CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
	
	IntEnable(INT_CAN0);

	CANEnable(CAN0_BASE);
	
	//****************************************************************************
	
	//	
	// Initialize the sync msg transmitted by this master
  // All sensors will receive this msg on the network and reply
	// accordingly
	//
	// sync msg: CAN ID + DATA
	// CAN ID = (NODE_TYPE<<8) | (NODE_NR)
	// DATA = ( HEARTBEAT )
	//
	
	static uint16_t sync_period = HEARTBEAT; 
	
	txCANMessage.ui32MsgID = ((NODE_TYPE<<8) | (NODE_NR));
	txCANMessage.ui32MsgIDMask = 0; 
	txCANMessage.pui8MsgData = (uint8_t *)&sync_period;
	txCANMessage.ui32MsgLen = 2;
  txCANMessage.ui32Flags = 0; //MSG_OBJ_TX_INT_ENABLE;

	
	// Initialize Msg received to zero
	static uint16_t dataRx = 0;
	
	// Receive any valid msg on the network by setting the ID and MASK to zeros
	rxCANMessage.pui8MsgData = (uint8_t *)&dataRx;
	rxCANMessage.ui32MsgID = 0;
	rxCANMessage.ui32MsgIDMask = 0;
	rxCANMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;

	// Receive response in rxCANMessage struct
	CANMessageSet(CAN0_BASE, 1, &rxCANMessage, MSG_OBJ_TYPE_RX);
	
}
//*******************************************************************************



// broadcast a sync msg to be received by all nodes on the network
void TransmitSync(void)
{
	CANMessageSet(CAN0_BASE, 2, &txCANMessage, MSG_OBJ_TYPE_TX);
}



//*******************************************************************************
// will receive the data then extract the node_nr, node_type and data info from the
// msg and store it in the passed parameters to be used by the application
void ReceiveData( Node_t * node )
{
		taskENTER_CRITICAL();
		CANMessageGet(CAN0_BASE, 1, &rxCANMessage, 0);
		node->node_nr = (rxCANMessage.ui32MsgID & 0xff); 					// extact the first 8 bits which correspond to the node id
		node->node_type = ((rxCANMessage.ui32MsgID) & (0x700))>>8; // extract the node type from the msg id
		node->node_data_size = rxCANMessage.ui32MsgLen;
		node->node_data_ptr = rxCANMessage.pui8MsgData;
		taskEXIT_CRITICAL();
}
//********************************************************************************
