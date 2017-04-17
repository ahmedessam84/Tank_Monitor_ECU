

#ifndef __rtc_h__
#define __rtc_h__



extern void Rtc_Init(void);
extern void Rtc_GetDate( struct tm *tm_ptr );
extern void Rtc_SetDate( struct tm *tm_ptr );

#endif
