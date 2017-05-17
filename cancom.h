// Header: 
// File Name: 
// Author: 
// Date:

#ifndef _CANCOM_H_
#define _CANCOM_H_

// 
// C Binding for C++ Compilers
// 
#ifdef __cplusplus
extern "C"
{
#endif


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
	
	
#define HEARTBEAT		2000		// 2000 ms = 2s

//                         |----| |-------------|
// CAN ID 11-bits numbers: 10 9 8 7 6 5 4 3 2 1 0
//                         |type| |----number---|
#define NODE_NR			0x00		// master is node nr 0x00
#define NODE_TYPE		0x01		// for master = 0x01
														// for liquid level sensor  = 0x02
														// for pressure sensor = 0x03

typedef struct
{
	uint8_t node_nr;							// node number
	uint8_t node_type;						// type of sensor: liquid level, pressure...etc 
	uint8_t node_data_size;				
	uint8_t * node_data_ptr;
}Node_t;

	
void CANCom_Master_Init(void);
void TransmitSync(void);
void ReceiveData( Node_t * node );
	
// 
// End of C Binding
// 
#ifdef __cplusplus
}
#endif

#endif 
