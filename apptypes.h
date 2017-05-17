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
	
#include <time.h>

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



	
// 
// End of C Binding
// 
#ifdef __cplusplus
}
#endif

#endif 
