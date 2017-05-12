//*************************************************************
// Button Manager for use with LCD
//*************************************************************
// This application takes a counter and a cursor and increments 
// or decrements their value according to the button pressed 
//*************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
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
#include "delay.h"
#include "LCD20x04.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "btn.h"
#include "btnmr.h"

static Debouncer port1;

extern uint32_t ui32TickCounter;


//*****************************************************************************

void BtnMr_Init(void)
{
	ButtonDebounceInit(&port1, ALL_BUTTONS);
}

//*****************************************************************************
// this is the main thread it has to run periodically. here is it set to run every clock tick
// it updates the position of the cursor and the counter and the edit flag periodically
// the edit flag enables and disables the operation of the other buttons
//
//

void BtnMr( cursor_t * cursor_ptr, int * counter_ptr, bool * EditFlag_ptr)
{
	
	static uint32_t up_scroll_speed = 1;
	static uint32_t down_scroll_speed = 1;
	
	ButtonProcess(&port1, Port1ReadBits());
	switch((ButtonCurrent(&port1, ALL_BUTTONS)))
	{
		
		case LEFT_BUTTON:	
		{
			
				if(*EditFlag_ptr)
				{
					if((ui32TickCounter % APP_BUTTON_POLL_DIVIDER_R_L) == 0)
					{
						if(cursor_ptr != NULL)
						{
							(*cursor_ptr)--;
						}											
					}
				}
			
			break;
		}
		
		case RIGHT_BUTTON:	
		{
			
				if(*EditFlag_ptr)
				{
					if((ui32TickCounter % APP_BUTTON_POLL_DIVIDER_R_L) == 0)
					{
						if(cursor_ptr != NULL )
						{
							(*cursor_ptr)++;
						}
					}
				}
			
			break;
		}
		
		case UP_BUTTON:	
		{
			
				if(*EditFlag_ptr)
				{
					if((ui32TickCounter % (APP_BUTTON_POLL_DIVIDER_UP_DOWN - up_scroll_speed) ) == 0)
					{
						if(counter_ptr != NULL )
						{
							up_scroll_speed = up_scroll_speed + 2;
							(*counter_ptr)++;
						}
					}
				}
			
			break;
		}
		
		case DOWN_BUTTON:	
		{
			
				if(*EditFlag_ptr)
				{
					if((ui32TickCounter % (APP_BUTTON_POLL_DIVIDER_UP_DOWN - down_scroll_speed)) == 0)
					{
						if(counter_ptr != NULL )
						{
							down_scroll_speed = down_scroll_speed + 2;
							(*counter_ptr)--;
						}
					}
				}
			
			break;
		}
		
		case SELECT_BUTTON:	
		{
			// hold the select button for a while to enter the editing mode
			// the hold time is in multiples of APP_BUTTON_POLL_DIVIDER_MODE_SELECT
			// increase the APP_BUTTON_POLL_DIVIDER_MODE_SELECT to increase the hold time
			if((ui32TickCounter % APP_BUTTON_POLL_DIVIDER_MODE_SELECT) == 0)
			{
				if( EditFlag_ptr != NULL )
				{
				 (*EditFlag_ptr) ^= 1;
				}
			}
			break;
		}
	}
	
	// if buttons up or down are released reset the scroll speed multiplier
	if(ButtonReleased(&port1, UP_BUTTON) == UP_BUTTON)
	{
		up_scroll_speed = 1;
	}
	
	// if buttons up or down are released reset the scroll speed multiplier
	if(ButtonReleased(&port1, DOWN_BUTTON) == DOWN_BUTTON)
	{
		down_scroll_speed = 1;
	}
	
}		
	
















