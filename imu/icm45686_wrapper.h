/*
	Copyright 2026 Benjamin Vedder	benjamin@vedder.se
	Written in 2026 by Samuel de Boer  sdb@pendrum.com

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
