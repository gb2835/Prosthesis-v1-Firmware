/*
 * mpu9255.c
 *
 *  Created on: Feb 22, 2024
 *      Author: brett
 */

#include "mpu9255.h"
#include "stm32l4xx_ll_spi.h"
#include "stm32l476xx.h"
//#include "spi1.h" ??
#include "spi.h" // ??
#include "systick_app_timer.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "string.h"

/*
 * i2c_write(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char const *data)
 * i2c_read(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char *data)
 * delay_ms(unsigned long num_ms)
 * get_ms(unsigned long *count)
*/

#define DEBUG_DMP 1

static systick_app_timer_t delayTimer;
static systick_app_timer_t readTimer;
static volatile uint32_t g_ul_ms_ticks = 0;
static volatile uint32_t ms_delay = 0;
static uint32_t updateRequired = false;

#if DEBUG_DMP
dmp_data_t dmpData[256];
dmp_data_t lastDmpData;
static uint8_t readIndex = 0;
static uint8_t writeIndex = 0;
#else
dmp_data_t dmpData;
#endif
// CS is active low. This disables the chip
__STATIC_INLINE void clearChipSelect() {
	LL_GPIO_SetOutputPin(IMU_CS_GPIO_Port, IMU_CS_Pin);
}

// CS is active low. This enables the chip.
__STATIC_INLINE void setChipSelect() {
	LL_GPIO_ResetOutputPin(IMU_CS_GPIO_Port, IMU_CS_Pin);
}

int mpu9255_write(unsigned char slave_addr, unsigned char reg_addr,
     unsigned char length, unsigned char const *data) {

	(void)slave_addr; // NOT NEEDED, I2C variable. Suppress unused variable warning

	setChipSelect();

	// 4 byte fifo. Respect that and write when free
	LL_SPI_TransmitData8(SPI1, reg_addr);
	for (uint8_t i = 0; i < length; i++) {
		while (!(SPI1->SR & SPI_SR_TXE)) {
		};
		LL_SPI_TransmitData8(SPI1, data[i]);
	}

	// wait for completion
	while ((SPI1->SR & SPI_SR_FTLVL)) {
	}; 	//transmit fifo empty?
	while ((SPI1->SR & SPI_SR_BSY)) {
	}; 	// no longer busy
	clearChipSelect();

	// clear receive fifo. Overrun data will be lost, but we are only sending.
	while ((SPI1->SR & SPI_SR_FRLVL)) {
		uint8_t dummy = SPI1->DR; 			// clear rx fifo from the receives.
		(void) dummy;						// suppress unused variable warning
	};
	return 0;
}

/*
 * Read assumes that calling function is providing a data location big enough
 * to handle the size of length. Be aware that this can overrun the calling
 * memory location if not sized appropriately!
 */
int mpu9255_read(unsigned char slave_addr, unsigned char reg_addr,
      unsigned char length, unsigned char *data) {

	(void) slave_addr; // not used, suppress unused variable warning.

	setChipSelect();

	// 4 byte fifo not used. Reads need to be captured.
	LL_SPI_TransmitData8(SPI1, (reg_addr | 0x80)); // MSb set signals read.
	while ((SPI1->SR & SPI_SR_FRLVL) == 0) {}; 	//wait for SR buffer to have data
	uint8_t dummy = SPI1->DR; 				// first byte is from cmd transfer. Ignore
	(void)dummy; // suppress unused variable warning

	// read the data sequentially
	for (uint8_t i = 0; i < length; i++) {
		LL_SPI_TransmitData8(SPI1, 0xFF);	// send clocking byte, not used.
		while ((SPI1->SR & SPI_SR_FRLVL) == 0) {}; // wait for SR buffer to have data
		data[i] = SPI1->DR;
	}

	while ((SPI1->SR & SPI_SR_BSY)) {
		}; 	// no longer busy
	clearChipSelect();

	return 0;
}

void mpu9255_delay_ms(unsigned long num_ms) {
	ms_delay = num_ms;
	while (ms_delay > 0) {
		// do nothing while we wait.
	}
}

void mpu9255_get_ms(unsigned long *count) {
	*count = g_ul_ms_ticks;
}

void delaytimer_event_handler() {
	g_ul_ms_ticks++;
	if (ms_delay) {
		ms_delay--;
	}
}

/*
 * Process reading the fifo from the mpu9255
 */
void readTimer_event_handler() {
	updateRequired = true;
}

void android_orient_cb(unsigned char orientation) {
	// do nothing, not used.
}
void tap_cb(unsigned char direction, unsigned char count) {
	// do nothing, not used.
}

static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}


static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

dmp_data_t * mpu9255_getLast() {
#if DEBUG_DMP
	if (readIndex != writeIndex) {
		memcpy(&lastDmpData, &dmpData[readIndex], sizeof(dmp_data_t));
		return &dmpData[readIndex++];
	} else {
		// no new data, keep returning last one
		return &lastDmpData;
	}
#else
	return &dmpData;
#endif
}

void mpu9255_process() {


	if (updateRequired) {

		unsigned char more;


#if DEBUG_DMP
		if (dmp_read_fifo(dmpData[writeIndex].gyro.array,
				dmpData[writeIndex].acceleration.array,
				dmpData[writeIndex].quaternarion.array,
				&dmpData[writeIndex].timestamp, &dmpData[writeIndex].sensors,
				&more) == 0) {

			if (dmpData[writeIndex].sensors == 0x178) {
				writeIndex++;
			}
#else
		dmp_data_t d;	// temporary variable for the read.

		dmp_read_fifo(d.gyro.array, d.acceleration.array, d.quaternarion.array, &d.timestamp, &d.sensors, &more);

		// if new data is valid, copy over last saved data
		if (d.sensors != 0) {
			memcpy(&dmpData, &d, sizeof(dmp_data_t));
		}
#endif
			if (!more) {
				updateRequired = false;
			}
		}

	}
}

void mpu9255_init(uint32_t readPeriod) {
//	spi1_init(); ??
	clearChipSelect();
	uint8_t whoami;
	mpu9255_read(0, 0x75, 1, &whoami);
	delayTimer.mode = APP_TIMER_MODE_CONTINUOUS;
	delayTimer.alarm = 1; // 1ms
	delayTimer.timerAlarmCallback = delaytimer_event_handler;
	systick_app_timer_channel_create(&delayTimer);
	systick_app_timer_channel_start(delayTimer.channel);

	readTimer.mode = APP_TIMER_MODE_CONTINUOUS;
	readTimer.alarm = readPeriod; // 1ms
	readTimer.timerAlarmCallback = readTimer_event_handler;
	systick_app_timer_channel_create(&readTimer);
	// don't start this one until after the dmp is initialized

	// reset internal data
	memset(&dmpData, 0, sizeof(dmpData));

	// First init the MPU chip
	struct int_param_s int_param;
	mpu_init(&int_param);
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

	// load dmp and turn on
	dmp_load_motion_driver_firmware();
	const signed char orientation[9] = {1, 0, 0,
							0, 0, -1,
							0, 1, 0};

	dmp_set_orientation( inv_orientation_matrix_to_scalar(orientation));
	dmp_register_tap_cb(tap_cb);
	dmp_register_android_orient_cb(android_orient_cb);

	/*
	 *  * There is a known issue in which if you do not enable DMP_FEATURE_TAP
     * then the interrupts will be at 200Hz even if fifo rate
     * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
	 */
	dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO);

	dmp_set_fifo_rate(100);
	mpu_set_dmp_state(1);
	mpu_reset_fifo();
	systick_app_timer_channel_start(readTimer.channel);
}
