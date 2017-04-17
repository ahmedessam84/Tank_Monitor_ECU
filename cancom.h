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

#define HEARTBEAT		0x7d0		// 2000 ms
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
