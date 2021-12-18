#ifndef TIMERS_H_
#define TIMERS_H_

#include "stdint.h"


#define MAX_TIMERS 			1

#define USE_GLOBAL_TIMERS

#ifdef USE_GLOBAL_TIMERS
	#define MAX_GTIMERS 				9

	#define TIMER_PUMP_OFF				0
	#define TIMER_CONV_U_OFF			1
	#define TIMER_CONV_IGN_OFF			2
	#define TIMER_FRIDGE_U_OFF			3
	#define TIMER_FRIDGE_IGN_OFF		4
	#define TIMER_SHUTDOWN_DELAY		5
	#define TIMER_PRX_SENS_FEEDBACK		6
	#define TIMER_TEMP_SENS_UPDATE  	7
	#define TIMER_LCD_LIGHT_OFF			8


#endif

#define SEC 			1000
#define MIN 			60 * SEC
#define HOUR 			60 * MIN



void ProcessTimers(uint32_t * tick);
void InitTimers(void);
uint32_t  GetTimer(uint8_t Timer);
void ResetTimer(uint8_t Timer);

#ifdef USE_GLOBAL_TIMERS
	void InitGTimers(void);
	uint32_t  GetGTimer(uint8_t Timer);
	void StopGTimer(uint8_t Timer);
	void StartGTimer(uint8_t Timer);
	void PauseGTimer(uint8_t Timer);
	void ContinueGTimer(uint8_t Timer);
#endif

#endif /* TIMERS_H_ */
