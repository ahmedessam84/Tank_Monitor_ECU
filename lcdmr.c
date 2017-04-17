// Header:
// File Name: 
// Author:
// Date:

//*****************************************************************************
//
// Screen module
//
// input: ascii Calendar buffer, ascii T1 volume buffer, ascii T2 volume buffer
//				ascii T3 volume buffer, ascii T4 volume buffer
//
// output: display input LCD
//
// return: nothing
//
//*****************************************************************************


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "LCD20x04.h"
#include "lcdmr.h"
#include "string.h"



void LcdMr_Init(void)
{
	LCD_Init();
}

//*************************
//**Display Screen Sample**
//*************************
//
//	   01234567890123456789
//	L1  01/01/01 00:00:00
//	L2 T1: 12345 T2: 12345
//	L3 T3:       T4:
//	L4 T5:       T6:
//
//*************************  

void LcdMr(char *Cal_buffer, char *T1_buffer, char *T2_buffer, char *T3_buffer, char *T4_buffer, char *T5_buffer, char *T6_buffer )
{
	// Display Calendar on line 1
	LCD_GoToXy(0,0);
	LCD_DispString(" ");
	
	if( Cal_buffer )
	{
		LCD_DispString( Cal_buffer );
	}
	else
	{
		// not available
		LCD_DispString("NA   ");
	}
	
	
	// display T1: on line 2
	LCD_GoToXy(0,1);
	LCD_DispString("T1: ");
	// if buffer is defined i.e sensor connected, display its data
	if(T1_buffer)
	{
		
		LCD_DispString( T1_buffer );
	}
	else
	{
		// not available
		LCD_DispString("NA   ");
	}
	
	LCD_DispString(" T2: ");
	if(T2_buffer)
	{
		LCD_DispString( T2_buffer );
	}
	else
	{
		// not available
		LCD_DispString("NA   ");
	}
	
	
	
	// display T3: on line 3
	LCD_GoToXy(0,2);
	LCD_DispString("T3: ");
	if(T3_buffer)
	{
		LCD_DispString( T3_buffer );
	}
	else
	{
		// not available
		LCD_DispString("NA   ");
	}		
	
	LCD_DispString(" T4: ");
	if(T4_buffer)
	{
		LCD_DispString( T4_buffer );
	}
	else
	{
		// not available
		LCD_DispString("NA   ");
	}
	
	
	
	// display T5: on line 4
	LCD_GoToXy(0,3);
	LCD_DispString("T5: ");
	if(T5_buffer)
	{
		LCD_DispString( T5_buffer );
	}
	else
	{
		// not available
		LCD_DispString("NA   ");
	}
	
	
	LCD_DispString(" T6: ");
	if(T6_buffer)
	{
		LCD_DispString( T6_buffer );
	}
	else
	{
		// not available
		LCD_DispString("NA   ");
	}
	
}

//*************************************************************************************

void LcdMr_Setup_screen(char * Cal_buffer, char * Radius_buffer, char * Length_buffer )
{
	// Display Calendar on line 1
	LCD_GoToXy(0,0);
	LCD_DispString(" ");
	LCD_DispString( Cal_buffer );

	// Display line 2 " Enter Tank Dimensions:
	LCD_GoToXy(0,1);
	LCD_DispString("Tank 1 Dimensions:");
	
	//Line 3
	LCD_GoToXy(0,2);
	LCD_DispString("Radius mm = ");
	
	if(Radius_buffer)
	{
		LCD_DispString(Radius_buffer);
	}
	else{
		LCD_DispString("NA");
	}
	
	//Line 4
	LCD_GoToXy(0,3);
	LCD_DispString("Length mm = ");
	
	if(Radius_buffer)
	{
		LCD_DispString(Length_buffer);
	}
	else{
		LCD_DispString("NA");
	}
	
}

//*************************************************************************














//*************************************************************************

void LcdCal( char * Cal_buffer )
{
	// Display Calendar on line 1
	LCD_GoToXy(0,0);
	LCD_DispString(" ");
	
	if( Cal_buffer )
	{
		LCD_DispString( Cal_buffer );
	}
	else
	{
		// not available
		LCD_DispString("NA   ");
	}
	
	LCD_GoToXy(0,1);
	LCD_DispString("T1: ");
	LCD_GoToXy(10,1);
	LCD_DispString("T2: ");
	
	LCD_GoToXy(0,2);
	LCD_DispString("T3: ");
	LCD_GoToXy(10,2);
	LCD_DispString("T4: ");
	
	LCD_GoToXy(0,3);
	LCD_DispString("T5: ");
	LCD_GoToXy(10,3);
	LCD_DispString("T6: ");
	
}

//****************************************************************************

void LcdT1( char * T1_buffer )
{
	// display T1: on line 2
	LCD_GoToXy(4,1);
	// if buffer is defined i.e sensor connected, display its data
	if(T1_buffer)
	{
		
		LCD_DispString( T1_buffer );
	}
	else
	{
		// not available
		LCD_DispString("NA   ");
	}	
}

//*********************************************************************************

void LcdT3( char * T3_buffer )
{
	// display T1: on line 2
	LCD_GoToXy(4,2);
	// if buffer is defined i.e sensor connected, display its data
	if(T3_buffer)
	{
		
		LCD_DispString( T3_buffer );
	}
	else
	{
		// not available
		LCD_DispString("NA   ");
	}	
}

//*****************************************************************************

void LcdT5( char * T5_buffer )
{
	// display T1: on line 2
	LCD_GoToXy(4,3);
	// if buffer is defined i.e sensor connected, display its data
	if(T5_buffer)
	{
		
		LCD_DispString( T5_buffer );
	}
	else
	{
		// not available
		LCD_DispString("NA   ");
	}	
}

//******************************************************************************

void LcdT2( char * T2_buffer )
{
	LCD_GoToXy( 14, 1 );
	if(T2_buffer)
	{
		LCD_DispString( T2_buffer );
	}
	else
	{
		// not available
		LCD_DispString("NA   ");
	}
}

//******************************************************************************

void LcdT4( char * T4_buffer )
{
	LCD_GoToXy( 14, 2 );
	if(T4_buffer)
	{
		LCD_DispString( T4_buffer );
	}
	else
	{
		// not available
		LCD_DispString("NA   ");
	}
}

//********************************************************************************

void LcdT6( char * T6_buffer )
{
	LCD_GoToXy( 14, 3 );
	if(T6_buffer)
	{
		LCD_DispString( T6_buffer );
	}
	else
	{
		// not available
		LCD_DispString("NA   ");
	}
}

//********************************************************************************

void LcdHr( char * hr )
{
	LCD_GoToXy( 1 , 0 );
	LCD_DispString( hr );
}

void LcdMin( char * min )
{
	LCD_GoToXy( 4 , 0 );
	LCD_DispString( min );
}

void LcdSec( char * sec )
{
	LCD_GoToXy( 7 , 0 );
	LCD_DispString( sec );
}

void LcdDay( char * day )
{
	LCD_GoToXy( 11 , 0 );
	LCD_DispString( day );
}

void LcdMonth( char * month )
{
	LCD_GoToXy( 14 , 0 );
	LCD_DispString( month );
}

void LcdYr( char * yr )
{
	LCD_GoToXy( 17 , 0 );
	LCD_DispString( yr );
}

void LcdRadius( char * radius )
{
	LCD_GoToXy( 12 , 2 );
	LCD_DispString( radius );
}

void LcdLength( char * length )
{
	LCD_GoToXy( 12 , 3 );
	LCD_DispString( length );
}

void LcdSetupScreenBase( void )
{
	
	LCD_GoToXy(3,0);
	LCD_DispChar(':');
	
	LCD_GoToXy(6,0);
	LCD_DispChar(':');
	
	LCD_GoToXy(13,0);
	LCD_DispChar('/');
	
	LCD_GoToXy(16,0);
	LCD_DispChar('/');	
	
	// Display line 2 " Enter Tank Dimensions:
	LCD_GoToXy(0,1);
	LCD_DispString("Tank 1 Dimensions:");
	
	//Line 3
	LCD_GoToXy(0,2);
	LCD_DispString("Radius mm = ");
	
	//Line 4
	LCD_GoToXy(0,3);
	LCD_DispString("Length mm = ");
}

void LcdNormalScreenBase( void )
{
	
	LCD_GoToXy(3,0);
	LCD_DispChar(':');
	
	LCD_GoToXy(6,0);
	LCD_DispChar(':');
	
	LCD_GoToXy(13,0);
	LCD_DispChar('/');
	
	LCD_GoToXy(16,0);
	LCD_DispChar('/');	
	
	LCD_GoToXy(0,1);
	LCD_DispString("T1: ");
	LCD_GoToXy(10,1);
	LCD_DispString("T2: ");
	
	LCD_GoToXy(0,2);
	LCD_DispString("T3: ");
	LCD_GoToXy(10,2);
	LCD_DispString("T4: ");
	
	LCD_GoToXy(0,3);
	LCD_DispString("T5: ");
	LCD_GoToXy(10,3);
	LCD_DispString("T6: ");

}



