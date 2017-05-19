// Header: 
// File Name: 
// Author: 
// Date:

#ifndef _APPTYPES_H_
#define _APPTYPES_H_

// 
// C Binding for C++ Compilers
// 
#ifdef __cplusplus
extern "C"
{
#endif


#include <stdbool.h>
#include <stdint.h>	
#include <time.h>
#include "btnmr.h"
	
#define TRUE 		1
#define FALSE 	0

#define RX_PRIORITY						12	
#define TX_PRIORITY						11
#define BUTTONS_PRIORITY			10
#define DISPLAY_PRIORITY			5
	
	
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

typedef enum
{
	NORMAL,
	SETUP
}AppMode_t;

	
// 
// End of C Binding
// 
#ifdef __cplusplus
}
#endif

#endif 
