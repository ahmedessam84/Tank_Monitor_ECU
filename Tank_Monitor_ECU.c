//*****************************************************************************
//
// freertos_demo.c - Simple FreeRTOS example.
//
// Copyright (c) 2012-2016 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.3.156 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/can.h"
#include "utils/uartstdio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "cancom.h"
#include "rtc.h"
#include "apptypes.h"

#include "buttonspoll_task.h"
#include "display_task.h"
#include "txrx_task.h"

//*****************************************************************************
//
// A counter that keeps track of the number of times the TX interrupt has
// occurred, which should match the number of TX messages that were sent.
//
//*****************************************************************************

volatile uint32_t g_ui32MsgCount = 0;


/*****************************************************************************/

void vApplicationTickHook( void );
volatile uint32_t ui32TickCounter;	

AppMode_t g_eAppMode = NORMAL;

/*****************************************************************************/
//*****************************************************************************
//
// The mutex that protects concurrent access of UART from multiple tasks.
//
//*****************************************************************************

xSemaphoreHandle g_pLCDSemaphore;
xSemaphoreHandle g_pRTCSemaphore;


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}

#endif

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void
vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}


//*****************************************************************************

void itoa_new(uint16_t val, char * str)
{
		uint32_t temp_val;
		temp_val = val;
		str[0] = (temp_val / 10000) + 48;
		temp_val = (temp_val % 10000);
		str[1] = (temp_val / 1000) + 48;
		temp_val = (temp_val % 1000);
		str[2] = (temp_val / 100) + 48;
		temp_val = (temp_val % 100);
		str[3] =  (temp_val / 10) + 48;
		temp_val = (temp_val % 10);
		str[4] = temp_val + 48;
		str[5] = '\0';
}

//*****************************************************************************
//
// Configure the UART and its pins. This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
		
}


//*****************************************************************************
//
// Initialize FreeRTOS and start the initial set of tasks.
//
//*****************************************************************************
int
main(void)
{
    //
    // Set the clocking to run at 80 MHz from the PLL.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    //
    // Initialize the UART and configure it for 115,200, 8-N-1 operation.
    //
    ConfigureUART();
	
		Rtc_Init();
		CANCom_Master_Init();
	
    //
    // Print demo introduction.
    //
    UARTprintf("\n\nWelcome to the EK-TM4C123GXL Underground Tank Monitor!\n");


		
/************************************************************************/

    //
    // Create a mutex:
    //
    g_pLCDSemaphore = xSemaphoreCreateMutex();
		g_pRTCSemaphore = xSemaphoreCreateMutex();


		// create Rx_Task
		if(Rx_Task_Init() != TRUE )
		{
			while(1)
			{
				// Loop here if task was not created
			}
		}

		// create Tx_Task
		if(Tx_Task_Init() != TRUE )
		{
			while(1)
			{
				// Loop here if task was not created
			}
		}

		// create ButtonsPoll_Task
		if(ButtonsPoll_Task_Init() != TRUE)
		{
			while(1)
			{
				// Loop here if task was not created
			}
		}
		
		// create Display_Task
		if(Display_Task_Init() != TRUE)
		{
			while(1)
			{
				// Loop here if task was not created
			}
		}
		
    //
    // Start the scheduler.  This should not return.
    //
    vTaskStartScheduler();

    //
    // In case the scheduler returns for some reason, print an error and loop
    // forever.
    //
    while(1)
    {
    }
		
}


void vApplicationTickHook( void )
{
	ui32TickCounter++;
}


