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

#ifndef ENC_MA600A_H_
#define ENC_MA600A_H_

#include <stdint.h>
#include <stdbool.h>
#include "ch.h"
#include "hal.h"
#include "spi_bb.h"
#include "ma600a_driver/ma600a.h"

typedef struct {
	float last_enc_angle;
	float spi_error_rate;
	uint32_t spi_error_cnt;
	uint32_t last_update_time;
} MA600A_enc_state_t;

typedef struct {
	// Hardware SPI (used when spi_dev != NULL)
	SPIDriver *spi_dev;
	SPIConfig hw_spi_cfg;
	uint8_t spi_af;
	stm32_gpio_t *nss_gpio;
	int nss_pin;
	stm32_gpio_t *sck_gpio;
	int sck_pin;
	stm32_gpio_t *mosi_gpio;  // May be NULL (MA600A is read-only)
	int mosi_pin;
	stm32_gpio_t *miso_gpio;
	int miso_pin;
	// Software bit-bang SPI (used when spi_dev == NULL)
	spi_bb_state sw_spi;
	// Shared
	MA600A_Device device;
	MA600A_enc_state_t state;
} MA600A_config_t;

bool enc_ma600a_init(MA600A_config_t *cfg);
void enc_ma600a_deinit(MA600A_config_t *cfg);
void enc_ma600a_routine(MA600A_config_t *cfg);

#define MA600A_LAST_ANGLE(cfg) ((cfg)->state.last_enc_angle)

#endif /* ENC_MA600A_H_ */
