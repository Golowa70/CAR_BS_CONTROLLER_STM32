#include "TIMERS.h"


volatile uint32_t Timers[MAX_TIMERS];

#ifdef USE_GLOBAL_TIMERS
	#define TIMER_STOPPED		0
	#define TIMER_RUNNING		1
	#define TIMER_PAUSED		2

	volatile uint32_t GTimers[MAX_GTIMERS];
	volatile uint8_t GTStates[MAX_GTIMERS];
#endif

void ProcessTimers(uint32_t * tick) {
	uint8_t i = 0;
	uint32_t x = *tick;

	if (x > 0) {
		for (i=0; i<MAX_TIMERS; i++) {
			Timers[i] += x;
#ifdef USE_GLOBAL_TIMERS
			if (GTStates[i] == TIMER_RUNNING) {
				GTimers[i] += x;
			}
#endif
		}
		*tick = 0;
	}
}

void InitTimers(void) {
	uint8_t i;
	for (i=0; i<MAX_TIMERS; i++) {
		Timers[i] = 0;
	}
}

uint32_t GetTimer(uint8_t Timer) {
	return Timers[Timer];
}

void ResetTimer(uint8_t Timer) {
	Timers[Timer] = 0;
}

#ifdef USE_GLOBAL_TIMERS

void InitGTimers(void) {
	uint8_t i;
	for (i=0; i<MAX_TIMERS; i++) {
		GTimers[i] = TIMER_STOPPED;
	}
}

uint32_t  GetGTimer(uint8_t Timer){
	return GTimers[Timer];
}

void StopGTimer(uint8_t Timer){
	GTStates[Timer] = TIMER_STOPPED;
	GTimers[Timer] = 0;//added
}

void StartGTimer(uint8_t Timer){
		GTimers[Timer] = 0;
		GTStates[Timer] = TIMER_RUNNING;
}

void PauseGTimer(uint8_t Timer) {
	if (GTStates[Timer] == TIMER_RUNNING) {
		GTStates[Timer] = TIMER_PAUSED;
	}
}

void ContinueGTimer(uint8_t Timer) {
	if (GTStates[Timer] == TIMER_PAUSED) {
		GTStates[Timer] = TIMER_RUNNING;
	}
}

#endif
