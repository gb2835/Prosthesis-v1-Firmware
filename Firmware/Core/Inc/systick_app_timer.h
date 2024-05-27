/*
 * systick_app_timer.h
 *
 *  Created on: Feb 22, 2024
 *      Author: brett
 */

#ifndef INC_SYSTICK_APP_TIMER_H_
#define INC_SYSTICK_APP_TIMER_H_

#include "stdint.h"

#define APP_TIMER_MAX_CHANNELS 10

#ifndef false
#define false 0
#endif

#ifndef true
#define true 1
#endif

typedef enum {
	APP_TIMER_MODE_CONTINUOUS = 0,
	APP_TIMER_MODE_SINGLE
} systick_app_timer_mode_t;

/*
 * Structure is maintained by the dependent module.
 * To make changes, first disable the channel. Then changes can be made directly
 * to the structure. Once changes are complete, enable the channel.
 */
typedef struct {
	uint8_t 					channel;					// channel identifier. used for accessing.
	uint32_t 					value;						// counter variable
	systick_app_timer_mode_t 	mode;						// mode type. continuous will free run, reset on alarm. single will go inactive after an alarm.
	uint32_t 					alarm;						// (optional) count up alarm value. If set, counter resets to 0 each alarm.
	void 						(*timerAlarmCallback)(void);// (optional) callback on timer alarm equal
} systick_app_timer_t;

typedef struct {
	uint8_t active;						// flag the channel is active
	systick_app_timer_t	* timer;		// pointer to the dependent timer structure
} systick_app_timer_channel_t;

typedef struct {
	systick_app_timer_channel_t channel[APP_TIMER_MAX_CHANNELS];
	uint8_t count;
} systick_app_timer_module_t;

void systick_app_timer_tickAndProcess();
void systick_app_timer_tick();
uint32_t systick_app_timer_process();
uint32_t systick_app_timer_channel_delete(uint8_t channel);
uint32_t systick_app_timer_channel_stop(uint8_t channel);
uint32_t systick_app_timer_channel_start(uint8_t channel);
uint32_t systick_app_timer_channel_create(systick_app_timer_t * timer);
void systick_app_timer_module_init();
#endif /* INC_SYSTICK_APP_TIMER_H_ */


