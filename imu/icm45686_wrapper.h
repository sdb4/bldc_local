/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */
 
#ifndef ICM456xx_H
#define ICM456xx_H

#define ICM45686

#include "imu/inv_imu_driver_advanced.h"
#include "imu/inv_imu_edmp.h"
#if defined(ICM45686S) || defined(ICM45605S)
#include "imu/inv_imu_edmp_gaf.h"
#endif

#include "ch.h"
#include "hal.h"

#include <stdint.h>
#include <stdbool.h>

#include "i2c_bb.h"
#include "spi_bb.h"

typedef struct {
	inv_imu_device_t icm_driver;
	i2c_bb_state *i2cs;
	spi_bb_state *spis;
	uint8_t i2c_address;
	void(*read_callback)(float *accel, float *gyro, float *mag);
	volatile bool is_running;
	volatile bool should_stop;
	int rate_hz;
} ICM45686_STATE;

void icm45686_init(ICM45686_STATE *s, i2c_bb_state *i2c_state, spi_bb_state *spi_state, int ad0_val,
		stkalign_t *work_area, size_t work_area_size);
void icm45686_set_read_callback(ICM45686_STATE *s, void(*func)(float *accel, float *gyro, float *mag));
void icm45686_stop(ICM45686_STATE *s);

#endif // ICM456xx_H
