#include "stdint.h"
#include "delay.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "LCD20x04.h"
#include "driverlib/rom_map.h"

// LCD connected to PC4-PC7 for DB4-DB7 respectively
// LCD RS, R/W connected to PA2, PA3 respectively
// EN pin is PA5

#define F										0<<2		//5x8 dots display
#define N										1<<3		// 2 lines display
#define LCD_DATA_PINS				0XF0
#define RS									1u<<2
//#define RW								1u<<3
#define EN									1u<<3
#define DCB									0x04		// display on ,cursor on, blinking on
#define ID									0x02		// increment right
#define S										0x00		// shift off

static void SendCommand( uint8_t command_id )
{
	// send higher nibble
	MAP_GPIOPinWrite( GPIO_PORTA_BASE, RS , 0x00 );
	MAP_GPIOPinWrite( GPIO_PORTA_BASE, EN, 0xff );
	MAP_GPIOPinWrite( GPIO_PORTC_BASE, LCD_DATA_PINS, command_id );
	delay_us(2);
	MAP_GPIOPinWrite( GPIO_PORTA_BASE, EN, 0);
	delay_us(2);
	
	// send lower nibble
	MAP_GPIOPinWrite( GPIO_PORTA_BASE, RS , 0x00 );
	MAP_GPIOPinWrite( GPIO_PORTA_BASE, EN, 0xff );
	MAP_GPIOPinWrite( GPIO_PORTC_BASE, LCD_DATA_PINS, (command_id<<4) );	
	delay_us(2);
	MAP_GPIOPinWrite( GPIO_PORTA_BASE, EN, 0);
	
	delay_us(100);
	
}

static void SendData( uint8_t data)
{
		// send highier nibble
	MAP_GPIOPinWrite( GPIO_PORTA_BASE, RS, 0xff );
	MAP_GPIOPinWrite( GPIO_PORTA_BASE, EN, 0xff );
	MAP_GPIOPinWrite( GPIO_PORTC_BASE, LCD_DATA_PINS, data );
	delay_us(2);
	MAP_GPIOPinWrite( GPIO_PORTA_BASE, EN, 0);
	delay_us(2);
	
	// send lower nibble
	MAP_GPIOPinWrite( GPIO_PORTA_BASE, RS, 0xff );
	MAP_GPIOPinWrite( GPIO_PORTA_BASE, EN, 0xff );
	MAP_GPIOPinWrite( GPIO_PORTC_BASE, LCD_DATA_PINS, (data<<4) );	
	delay_us(2);
	MAP_GPIOPinWrite( GPIO_PORTA_BASE, EN, 0);
	
	delay_us(100);
}

void LCD_Init(void)
{
	Timer1_Init();
	//initialize gpio PC4-PC7 for output
	MAP_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOC );
	MAP_GPIOPinTypeGPIOOutput( GPIO_PORTC_BASE, LCD_DATA_PINS );
	
	//initialize gpio PA2, PA3 and PA5 for output
	MAP_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOA );
	MAP_GPIOPinTypeGPIOOutput( GPIO_PORTA_BASE, RS | EN );
	MAP_GPIOPinWrite( GPIO_PORTA_BASE, RS | EN, 0x00 );
	//Start initialization start 
	
	//delay 40ms after power up
	delay_ms(100);
	
	//function set
	SendCommand(0x03);

	//function set
	SendCommand( 0x20 | N | F );
	
	//function set
	SendCommand( 0x20 | N | F );

	// Display, Cursor, Blinking
	SendCommand( 1<<3 | DCB );

	//clear display
	SendCommand( 0x01 );
	delay_ms(2);

	//entry mode
	SendCommand( 1<<2 | ID | S );
	
	LCD_GoToXy(0,0);
	
}

void LCD_Clear(void)
{
	SendCommand( 0x01 );
	delay_ms(2);
}	
	
void LCD_GoToXy(uint8_t x, uint8_t y)
{		
	//move cursor according to x & y
	// if line one is chosen addresses span from 00h to 0Fh

	switch(y)
	{
		case 0: {SendCommand( 0x80 | x ); break;}
		
		case 1: {SendCommand( 0x80 | ( 0x40 + x ) ); break;}
		
		case 2: {SendCommand( 0x80 | ( 0x14 + x ) ); break;}
		
		case 3:  SendCommand( 0x80 | ( 0x54 + x ) );
	}
	
}	

void LCD_DispChar(char letter)
{
	SendData( letter );
}

void LCD_DispString(char *StrPtr)
{
	while( *StrPtr != '\0' )
	{
			SendData( *StrPtr );
			StrPtr ++;
	}			
}

void LCD_DispCursor(void)
{
	// Display, Cursor, Blinking
	SendCommand( 1<<3 | 0x07 );
}

void LCD_HideCursor(void)
{
	// Display, Cursor, Blinking
	SendCommand( 1<<3 | 0x04 );
}
