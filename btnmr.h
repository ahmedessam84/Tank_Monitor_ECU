// 
// Header Guard
// 
#ifndef BTNMR_H
#define BTNMR_H

// 
// C Binding for C++ Compilers
// 
#ifdef __cplusplus
extern "C"
{
#endif

//*********************************************************************************
// Macros and Globals
//*********************************************************************************
#define APP_SYSTICKS_PER_SEC									1000
#define APP_BUTTON_POLL_DIVIDER_MODE_SELECT 	5000
#define APP_BUTTON_POLL_DIVIDER_UP_DOWN				100
#define APP_BUTTON_POLL_DIVIDER_R_L						400

		typedef enum 
	{
		NONE,
		UP,
		DOWN,
		RIGHT,
		LEFT
	}btn_t;
	
	typedef struct
	{
		btn_t btnpressed;
		bool EditFlag;
	}btnstatus_t;	

typedef enum {HOUR=1, MIN, DAY, MONTH, YEAR, RADIUS, LENGTH}cursor_t;

//*********************************************************************************
// Prototypes
//*********************************************************************************

void BtnMr_Init(void);
void BtnMr( cursor_t * cursor_ptr, int * counter_ptr, bool * EditFlag_ptr);
	
	
	
	
	
	
	
// 
// End of C Binding
// 
#ifdef __cplusplus
}
#endif

#endif // BTNMR_H
