
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

//***************************MyCANProtocol**********************************
// CAN protocol - Master transmits a SYNC msg all slave nodes receive the msg
// each node replies back with their data to the master
//
// Each node's ID consists of 11 bits the first Most significant three bits
// Indicate the type of sensor. The remaining 8 bits indicate the node number
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
	
	void CANIntHandler(void);
	
// Intializes the CAN module for Master node operation which requires:
// Master receives any msg on network
// Master sends a sync msg every 20 ms
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
		CANMessageGet(CAN0_BASE, 1, &rxCANMessage, 0);
		node->node_nr = (rxCANMessage.ui32MsgID & 0xff); 					// extract the first 8 bits which correspond to the node id
		node->node_type = ((rxCANMessage.ui32MsgID) & (0x700))>>8; // extract the node type from the msg id
		node->node_data_size = rxCANMessage.ui32MsgLen;
		node->node_data_ptr = rxCANMessage.pui8MsgData;
}
//********************************************************************************
