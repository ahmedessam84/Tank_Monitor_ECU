#ifndef __lcd1602_h
#define __lcd1602_h

// LCD connected to PC4-PC7 for DB4-DB7 respectively
// LCD RS, R/W connected to PA2, PA3 respectively


void LCD_Init(void);
void LCD_Clear(void);
void LCD_GoToXy(uint8_t, uint8_t);
void LCD_DispChar(char);
void LCD_DispString(char *);
void LCD_HideCursor(void);
void LCD_DispCursor(void);


#endif
