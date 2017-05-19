// Header: 
// File Name: 
// Author: 
// Date:

#ifndef _DISPLAY_TASK_H_
#define _DISPLAY_TASK_H_

// 
// C Binding for C++ Compilers
// 
#ifdef __cplusplus
extern "C"
{
#endif
	
#define SENSOR_TIMEOUT_PERIOD									5000
	
	
#define CONVERT_LCD_PARAM_TO_ASCII(X)					strftime( cHr, 3, "%H", &X.calendar );			\
																							strftime( cMin, 3, "%M", &X.calendar );			\
																							strftime( cSec, 3, "%S", &X.calendar );			\
																							strftime( cDay, 3, "%d", &X.calendar );			\
																							strftime( cMonth, 3, "%m", &X.calendar );	  \
																							strftime( cYear, 3, "%y", &X.calendar );	  \
																							itoa_new( X.Tank.length, cLength_buffer );	\
																							itoa_new( X.Tank.radius, cRadius_buffer )

#define CONVERT_LCD_CALENDAR_TO_ASCII(X)			strftime( cHr, 3, "%H", &X );					\
																							strftime( cMin, 3, "%M", &X );				\
																							strftime( cSec, 3, "%S", &X );				\
																							strftime( cDay, 3, "%d", &X );				\
																							strftime( cMonth, 3, "%m", &X );			\
																							strftime( cYear, 3, "%y", &X )
	
#define DISP_LCD_CALENDAR_NORMAL()						LcdHr(cHr);					\
																							LcdMin(cMin);				\
																							LcdSec(cSec);				\
																							LcdMonth(cMonth);		\
																							LcdDay(cDay);				\
																							LcdYr(cYear)

	
#define	DISP_LCD_INITIAL_VALUES_SETUP()				LcdSetupScreenBase(); 		 \
																							LcdHr(cHr); 							 \
																							LcdMin(cMin); 						 \
																							LcdSec(cSec); 						 \
																							LcdDay(cDay); 						 \
																							LcdMonth(cMonth); 				 \
																							LcdYr(cYear); 						 \
																							LcdRadius(cRadius_buffer); \
																							LcdLength(cLength_buffer)



uint32_t Display_Task_Init(void);
	
// 
// End of C Binding
// 
#ifdef __cplusplus
}
#endif

#endif 
