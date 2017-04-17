// Header:
// File Name: 
// Author:
// Date:

#ifndef __LCDMR_H__
#define __LCDMR_H__

// 
// C Binding for C++ Compilers
// 
#ifdef __cplusplus
extern "C"
{
#endif
/*
typedef struct
	{
		uint8_t number_of_tanks;
		char * Radius_buffer[6];
		char * Length_buffer[6];
	}Tank_Dimensions_buffer_t;
*/
	
void LcdMr_Init(void);
void LcdMr(char *Cal_buffer, char *T1_buffer, char *T2_buffer, char *T3_buffer, char *T4_buffer, char *T5_buffer, char *T6_buffer );
void LcdMr_Setup_screen(char * Cal_buffer, char * Radius_buffer, char * Length_buffer );

void LcdCal( char * Cal_buffer );
void LcdT1( char * T1_buffer );
void LcdT3( char * T3_buffer );
void LcdT5( char * T5_buffer );
void LcdT2( char * T2_buffer );
void LcdT4( char * T4_buffer );
void LcdT6( char * T6_buffer );

void LcdHr( char * hr );
void LcdMin( char * min );
void LcdDay( char * day );
void LcdMonth( char * month );
void LcdYr( char * yr );
void LcdRadius( char * radius );
void LcdLength( char * length );
void LcdSetupScreenBase( void );
void LcdSec( char * );
void LcdNormalScreenBase( void );

	
// 
// End of C Binding
// 
#ifdef __cplusplus
}
#endif

#endif
