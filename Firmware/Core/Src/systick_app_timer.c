/*
 * systick_app_timer.c
 *
 *  Created on: Feb 22, 2024
 *      Author: brett
 */


#include "systick_app_timer.h"
#include "stdlib.h"
#include "string.h"

static systick_app_timer_module_t appTimer;

/*
 * Use this if you want the systick to incrment timers and process alarms/callbacks.
 * Do not use with long callback context
 * This is useful if at least one of the timers needs interrupt priority
 */
void systick_app_timer_tickAndProcess() {

	for (uint8_t i = 0; i < APP_TIMER_MAX_CHANNELS; i++) {
			// quick null check, for sanity
			if (appTimer.channel[i].timer != 0) {
				if (appTimer.channel[i].active) {
					appTimer.channel[i].timer->value++;
					// if the timer alarm has been set above 0, then there is something to process
					if (appTimer.channel[i].timer->alarm > 0) {
						// if value meets or exceeds alarm value, process the timer
						if (appTimer.channel[i].timer->value >= appTimer.channel[i].timer->alarm) {
							// if timer has a callback, call it and then reset the value
							if (appTimer.channel[i].timer->timerAlarmCallback) {
								appTimer.channel[i].timer->timerAlarmCallback();
								appTimer.channel[i].timer->value = 0;
							}
							// if single shot, make not active after expiration
							if (appTimer.channel[i].timer->mode == APP_TIMER_MODE_SINGLE) {
								appTimer.channel[i].active = false;
							}
						}
					}
				}
			}
		}
}

/*
 * Cleanest form. Systick interrupt only increments the counters.
 */
void systick_app_timer_tick() {
	uint8_t i;

	for (i = 0; i < APP_TIMER_MAX_CHANNELS; i++) {
		if (appTimer.channel[i].timer != 0) {
			if (appTimer.channel[i].active) {
				appTimer.channel[i].timer->value++;
			}
		}
	}
}

/*
 * Process should take place in the superloop
 */
uint32_t systick_app_timer_process() {
	uint8_t i;
	uint8_t active = 0;
	for (i = 0; i < APP_TIMER_MAX_CHANNELS; i++) {
		// quick null check, for sanity
		if (appTimer.channel[i].timer != 0) {
			if (appTimer.channel[i].active) {
				active = 1;
				// if the timer alarm has been set above 0, then there is something to process
				if (appTimer.channel[i].timer->alarm > 0) {
					// if value meets or exceeds alarm value, process the timer
					if (appTimer.channel[i].timer->value >= appTimer.channel[i].timer->alarm) {
						// if timer has a callback, call it and then reset the value
						if (appTimer.channel[i].timer->timerAlarmCallback) {
							appTimer.channel[i].timer->timerAlarmCallback();
							appTimer.channel[i].timer->value = 0;
						}
						// if single shot, make not active after expiration
						if (appTimer.channel[i].timer->mode == APP_TIMER_MODE_SINGLE) {
							appTimer.channel[i].active = false;
						}
					}
				}
			}
		}
	}
	return active;
}

/*
 * Delete a channel from the module.
 * Static timer config is held by the dependent process. It is responsible
 * for freeing memory if created. Pointer in the module is just set to null.
 */
uint32_t systick_app_timer_channel_delete(uint8_t channel) {
	if (channel >= APP_TIMER_MAX_CHANNELS) {
		return 2; // channel number out of bounds
	}

	appTimer.channel[channel].active = false;
	appTimer.channel[channel].timer = 0; // reset pointer. Memory is held by dependent, not app timer

	if (appTimer.count) {
		appTimer.count--;
	}

	return 0;
}

uint32_t systick_app_timer_channel_stop(uint8_t channel){

	if (channel >= APP_TIMER_MAX_CHANNELS) {
		return 2; // channel number out of bounds
	}

	if (appTimer.channel[channel].timer) {
		appTimer.channel[channel].active = false;
		return 0;
	}

	return 1; // channel not found
}

uint32_t systick_app_timer_channel_start(uint8_t channel){
	if (channel >= APP_TIMER_MAX_CHANNELS) {
		return 2; // channel number out of bounds
	}

	if (appTimer.channel[channel].timer) {
		appTimer.channel[channel].timer->value = 0; // reset when starting
		appTimer.channel[channel].active = true;
		return 0;
	}

	return 1; // channel not found
}

/*
 * Register a channel. The channel pointer should be declared
 * in memory by the requesting module. It is only pointed at by
 * the app timer module.
 */
uint32_t systick_app_timer_channel_create(systick_app_timer_t * timer){
	if (appTimer.count >= APP_TIMER_MAX_CHANNELS) {
		return 2; // timer module is full
	}

	uint8_t i;
	for (i = 0; i < APP_TIMER_MAX_CHANNELS; i++) {
		// take the first available slot
		if (appTimer.channel[i].timer == 0) {
			appTimer.channel[i].timer = timer;
			appTimer.channel[i].timer->channel = i;
			break;
		}
	}

	appTimer.count++;

	return 0;
}

void systick_app_timer_module_init() {

	memset(&appTimer, 0, sizeof(appTimer));

}
