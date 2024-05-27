/*
 * MPU9255.h
 *
 *  Created on: Feb 22, 2024
 *      Author: brett
 */

#ifndef DRIVERS_MPU9255_MPU9255_H_
#define DRIVERS_MPU9255_MPU9255_H_

#include "stdint.h"

typedef struct TUPLE_DATA {
	union {
		short array[3];
		struct {
			short x;
			short y;
			short z;
		} data;
	};
} dmp_tuple_t;

typedef struct QUATERNARION_DATA {
	union {
		long array[4];
		struct {
			long w;
			long x;
			long y;
			long z;
		} data;
	};
} dmp_quat_t;

typedef struct DMP_DATA {
	dmp_tuple_t	acceleration;
	dmp_tuple_t	gyro;
	dmp_quat_t	quaternarion;
	unsigned long timestamp;
	short sensors;
} dmp_data_t;


int mpu9255_write(unsigned char slave_addr, unsigned char reg_addr,
     unsigned char length, unsigned char const *data);
int mpu9255_read(unsigned char slave_addr, unsigned char reg_addr,
      unsigned char length, unsigned char *data);
void mpu9255_delay_ms(unsigned long num_ms);
void mpu9255_get_ms(unsigned long *count);
dmp_data_t * mpu9255_getLast();
void mpu9255_process();
void mpu9255_init();

void readTimer_event_handler(); // This wasn't initially here. Added to be used in main.c without declaration warning??

/*
 * readPeriod is how often to report the data for the DMP.
 * After init, the driver will read the fifo on this period.
 */
void mpu9255_init(uint32_t readPeriod);
#endif /* DRIVERS_MPU9255_MPU9255_H_ */
