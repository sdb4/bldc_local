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
 
#include "icm45686_wrapper.h"
#include "commands.h"
#include "terminal.h"
#include "utils_math.h"

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#if defined(ICM45686S) || defined(ICM45605S)
#include "imu/inv_imu_edmp_gaf.h"
#endif

static int i2c_write(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
static int i2c_read(uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
static int spi_write(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
static int spi_read(uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
static void sleep_us(uint32_t us);

#include "hw.h"

#ifndef ICM45686_ACCEL_FSR
#define ICM45686_ACCEL_FSR (16)
#endif // ICM45686_ACCEL_FSR

#ifndef ICM45686_GYRO_FSR
#define ICM45686_GYRO_FSR (2000)
#endif // ICM45686_GYRO_FSR

// Threads
static THD_FUNCTION(icm_thread, arg);

// Private functions
static bool reset_init_icm(ICM45686_STATE *s);

// Private variables
static ICM45686_STATE *m_terminal_state = 0;

// This is used by the driver callbacks (not object aware), declared static
static ICM45686_STATE *icm_driver_ptr = NULL;

static int i2c_write(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen) {
	if ((wlen+1) > 16) {
		return INV_IMU_ERROR;
	}

	uint8_t txb2[16];
	txb2[0] = reg;
	for (size_t i=0; i<wlen; i++) {
		txb2[i+1]=wbuffer[i];
	}
	bool res = i2c_bb_tx_rx(icm_driver_ptr->i2cs, icm_driver_ptr->i2c_address, txb2, 1+wlen, 0, 0);

	return (res)? 0: 1;
}

static int i2c_read(uint8_t reg, uint8_t * rbuffer, uint32_t rlen) {
	uint8_t txb[1];
	txb[0] = reg;
	bool res = i2c_bb_tx_rx(icm_driver_ptr->i2cs, icm_driver_ptr->i2c_address, txb, 1, 0, 0);
	res &= i2c_bb_tx_rx(icm_driver_ptr->i2cs, icm_driver_ptr->i2c_address, 0, 0, rbuffer, rlen);
	return (res)? 0: 1;
}

#define SPI_BEGIN(cfg)		spi_bb_delay(); palClearPad(cfg->nss_gpio, cfg->nss_pin); spi_bb_delay();
#define SPI_END(cfg)		spi_bb_delay(); palSetPad(cfg->nss_gpio, cfg->nss_pin); spi_bb_delay();

#define SPI_READ                   UINT8_C(0x80)
#define SPI_WRITE                  UINT8_C(0x7F)

static int spi_write(uint8_t reg_addr, const uint8_t * wbuffer, uint32_t wlen) {
	spi_bb_state *m_spi_bb = icm_driver_ptr->spis;
	int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

	reg_addr = (reg_addr & SPI_WRITE);

	chMtxLock(&m_spi_bb->mutex);

	spi_bb_begin(m_spi_bb);
	spi_bb_exchange_8(m_spi_bb, reg_addr);
	spi_bb_delay();

	for (uint32_t i = 0; i < wlen; i++) {
		spi_bb_exchange_8(m_spi_bb, wbuffer[i]);
	}

	spi_bb_end(m_spi_bb);

	chMtxUnlock(&m_spi_bb->mutex);

	return rslt;
}

static int spi_read(uint8_t reg_addr, uint8_t * rbuffer, uint32_t rlen) {
	spi_bb_state *m_spi_bb = icm_driver_ptr->spis;

	int8_t rslt = 0; // Return 0 for Success, non-zero for failure

	reg_addr = (reg_addr | SPI_READ);

	chMtxLock(&m_spi_bb->mutex);
	spi_bb_begin(m_spi_bb);
	spi_bb_exchange_8(m_spi_bb, reg_addr);
	spi_bb_delay();

	for (uint32_t i = 0; i < rlen; i++) {
		rbuffer[i] = spi_bb_exchange_8(m_spi_bb, 0);
	}

	spi_bb_end(m_spi_bb);
	chMtxUnlock(&m_spi_bb->mutex);
	return rslt;
}

static void sleep_us(uint32_t us)
{
	chThdSleepMicroseconds(us);
}

static accel_config0_accel_ui_fs_sel_t accel_fsr_g_to_param(uint16_t accel_fsr_g) {
  accel_config0_accel_ui_fs_sel_t ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G;

  switch(accel_fsr_g) {
  case 2:  ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_2_G;  break;
  case 4:  ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G;  break;
  case 8:  ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_8_G;  break;
  case 16: ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G; break;
#if INV_IMU_HIGH_FSR_SUPPORTED
  case 32: ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_32_G; break;
#endif
  default:
    /* Unknown accel FSR. Set to default 16G */
    break;
  }
  return ret;
}

static gyro_config0_gyro_ui_fs_sel_t gyro_fsr_dps_to_param(uint16_t gyro_fsr_dps) {
  gyro_config0_gyro_ui_fs_sel_t ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS;

  switch(gyro_fsr_dps) {
  case 15:  ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_15_625_DPS;  break;
  case 31:  ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_31_25_DPS;  break;
  case 62:  ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_62_5_DPS;  break;
  case 125:  ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_125_DPS;  break;
  case 250:  ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_250_DPS;  break;
  case 500:  ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_500_DPS;  break;
  case 1000: ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS; break;
  case 2000: ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS; break;
#if INV_IMU_HIGH_FSR_SUPPORTED
  case 4000: ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_4000_DPS; break;
#endif
  default:
    /* Unknown gyro FSR. Set to default 2000dps" */
    break;
  }
  return ret;
}

static accel_config0_accel_odr_t accel_freq_to_param(uint16_t accel_freq_hz) {
  accel_config0_accel_odr_t ret = ACCEL_CONFIG0_ACCEL_ODR_100_HZ;

  switch(accel_freq_hz) {
  case 1:    ret = ACCEL_CONFIG0_ACCEL_ODR_1_5625_HZ;  break;
  case 3:    ret = ACCEL_CONFIG0_ACCEL_ODR_3_125_HZ;  break;
  case 6:    ret = ACCEL_CONFIG0_ACCEL_ODR_6_25_HZ;  break;
  case 12:   ret = ACCEL_CONFIG0_ACCEL_ODR_12_5_HZ;  break;
  case 25:   ret = ACCEL_CONFIG0_ACCEL_ODR_25_HZ;  break;
  case 50:   ret = ACCEL_CONFIG0_ACCEL_ODR_50_HZ;  break;
  case 100:  ret = ACCEL_CONFIG0_ACCEL_ODR_100_HZ; break;
  case 200:  ret = ACCEL_CONFIG0_ACCEL_ODR_200_HZ; break;
  case 400:  ret = ACCEL_CONFIG0_ACCEL_ODR_400_HZ; break;
  case 800:  ret = ACCEL_CONFIG0_ACCEL_ODR_800_HZ; break;
  case 1600: ret = ACCEL_CONFIG0_ACCEL_ODR_1600_HZ;  break;
  case 3200: ret = ACCEL_CONFIG0_ACCEL_ODR_3200_HZ;  break;
  case 6400: ret = ACCEL_CONFIG0_ACCEL_ODR_6400_HZ;  break;
  default:
    /* Unknown accel frequency. Set to default 100Hz */
    break;
  }
  return ret;
}

static gyro_config0_gyro_odr_t gyro_freq_to_param(uint16_t gyro_freq_hz) {
  gyro_config0_gyro_odr_t ret = GYRO_CONFIG0_GYRO_ODR_100_HZ;

  switch(gyro_freq_hz) {
  case 1:   ret = GYRO_CONFIG0_GYRO_ODR_1_5625_HZ;  break;
  case 3:   ret = GYRO_CONFIG0_GYRO_ODR_3_125_HZ;  break;
  case 6:   ret = GYRO_CONFIG0_GYRO_ODR_6_25_HZ;  break;
  case 12:   ret = GYRO_CONFIG0_GYRO_ODR_12_5_HZ;  break;
  case 25:   ret = GYRO_CONFIG0_GYRO_ODR_25_HZ;  break;
  case 50:   ret = GYRO_CONFIG0_GYRO_ODR_50_HZ;  break;
  case 100:  ret = GYRO_CONFIG0_GYRO_ODR_100_HZ; break;
  case 200:  ret = GYRO_CONFIG0_GYRO_ODR_200_HZ; break;
  case 400:  ret = GYRO_CONFIG0_GYRO_ODR_400_HZ; break;
  case 800:  ret = GYRO_CONFIG0_GYRO_ODR_800_HZ; break;
  case 1600: ret = GYRO_CONFIG0_GYRO_ODR_1600_HZ;  break;
  case 3200: ret = GYRO_CONFIG0_GYRO_ODR_3200_HZ;  break;
  case 6400: ret = GYRO_CONFIG0_GYRO_ODR_6400_HZ;  break;
  default:
    /* Unknown gyro ODR. Set to default 100Hz */
    break;
  }
  return ret;
}

static void opx_debug_imu_read(int argc, const char **argv) {
	(void)argv;
	if (argc == 1) {
		commands_printf("READING IMU");

		inv_imu_sensor_data_t imu_data;
		int res = inv_imu_get_register_data(&m_terminal_state->icm_driver, &imu_data);

		if (res == INV_IMU_OK) {
			float accel[3], gyro[3];

			accel[0] = imu_data.accel_data[0] * (float)ICM45686_ACCEL_FSR / 32768.0;
			accel[1] = imu_data.accel_data[1] * (float)ICM45686_ACCEL_FSR / 32768.0;
			accel[2] = imu_data.accel_data[2] * (float)ICM45686_ACCEL_FSR / 32768.0;

			gyro[0] = imu_data.gyro_data[0] * (float)ICM45686_GYRO_FSR / 32768.0;
			gyro[1] = imu_data.gyro_data[1] * (float)ICM45686_GYRO_FSR / 32768.0;
			gyro[2] = imu_data.gyro_data[2] * (float)ICM45686_GYRO_FSR / 32768.0;

			commands_printf("%f %f %f %f %f %f", (double)accel[0], (double)accel[1], (double)accel[2], (double)gyro[0], (double)gyro[1], (double)gyro[2]);
		} else {
			commands_printf("BAD DATA");
		}
	}
}

static void opx_debug_imu_whoami(int argc, const char **argv) {
	(void)argv;
	if (argc == 1) {
		ICM45686_STATE *s = m_terminal_state;
		int rc = 0;

		/* Read and check whoami */
		uint8_t whoami;
		rc |= inv_imu_get_who_am_i(&s->icm_driver, &whoami);
		commands_printf("WHOAMI %ul", whoami);
	}
}

static void opx_debug_imu_read_reg(int argc, const char **argv) {
	if (argc == 2) {
		ICM45686_STATE *s = m_terminal_state;
		int rc = 0;

		unsigned int reg = 0;
		sscanf(argv[1], "%ul", &reg);

		uint8_t res;
		rc = inv_imu_read_reg(s, reg, 1, &res);
		commands_printf("REG: %u, STAT: %u, RES: %u", reg, rc, res);

		char bl[9];
		//write_single_reg(m_terminal_state, ICM45686_BANK_SEL, 0 << 4);
		utils_byte_to_binary(res & 0xFF, bl);
		commands_printf("Reg 0x%02x: %s (0x%02x)\n", reg, bl, res);
	}
}
static void opx_debug_imu_write_reg(int argc, const char **argv) {
	if (argc == 3) {
		ICM45686_STATE *s = m_terminal_state;
		int rc = 0;

		unsigned int reg = 0;
		unsigned int val = 0;
		sscanf(argv[1], "%ul", &reg);
		sscanf(argv[2], "%ul", &val);

		rc = inv_imu_write_reg(s, reg, 1, (uint8_t *)&val);
		commands_printf("REG: %u, STAT: %u, VAL: %u", reg, rc, val);
	}
}

void icm45686_init(ICM45686_STATE *s, i2c_bb_state *i2c_state, spi_bb_state *spi_state, int ad0_val,
		stkalign_t *work_area, size_t work_area_size) {
	s->i2cs = i2c_state;
	s->spis = spi_state;
	s->i2c_address = ad0_val ? 0x69 : 0x68;
	s->read_callback = NULL;
	s->is_running = false;
	s->should_stop = true;

	//Init everything
	if (i2c_state != NULL) {
		s->icm_driver.transport.serif_type = UI_I2C;
		s->icm_driver.transport.read_reg  = i2c_read;
		s->icm_driver.transport.write_reg = i2c_write;
	} else {
		s->icm_driver.transport.serif_type = UI_SPI4;
		s->icm_driver.transport.read_reg  = spi_read;
		s->icm_driver.transport.write_reg = spi_write;
	}
	s->icm_driver.transport.sleep_us = sleep_us;

	for (size_t i=0; i<6; i++)
	{
		s->icm_driver.adv_var[0] = 0u;
	}
	s->icm_driver.endianness_data = 0u;
	icm_driver_ptr = s;

	if (reset_init_icm(s)) {
		s->should_stop = false;
		chThdCreateStatic(work_area, work_area_size, NORMALPRIO+1, icm_thread, s);
	} else {
		commands_printf("INIT FAIL");
		return;
	}

	// Only register terminal command for the first instance of this driver.
	if (m_terminal_state == 0) {
		m_terminal_state = s;

		terminal_register_command_callback(
				"opx_debug_imu_read",
				"Print internal IMU type",
				0,
				opx_debug_imu_read);

		terminal_register_command_callback(
				"opx_debug_imu_whoami",
				"Print internal IMU type",
				0,
				opx_debug_imu_whoami);

		terminal_register_command_callback(
				"opx_debug_imu_read_reg",
				"Print internal IMU type",
				0,
				opx_debug_imu_read_reg);
		terminal_register_command_callback(
				"opx_debug_imu_write_reg",
				"Print internal IMU type",
				0,
				opx_debug_imu_write_reg);
	}
}

void icm45686_set_read_callback(ICM45686_STATE *s, void(*func)(float *accel, float *gyro, float *mag)) {
	s->read_callback = func;
}

void icm45686_stop(ICM45686_STATE *s) {
	s->should_stop = true;
	while(s->is_running) {
		chThdSleep(1);
	}

	if (s == m_terminal_state) {
//		terminal_unregister_callback(terminal_read_reg);
		m_terminal_state = 0;
	}
}

static bool reset_init_icm(ICM45686_STATE *s) {
	if (s->i2cs != NULL) {
		i2c_bb_restore_bus(s->i2cs);
	}

	chThdSleepMilliseconds(1);

	int rc = inv_imu_adv_init(&s->icm_driver);
	if (rc != INV_IMU_OK) return false;

	sleep_us(3000);

	//STARTACCEL
	rc |= inv_imu_set_accel_fsr(&s->icm_driver, accel_fsr_g_to_param(ICM45686_ACCEL_FSR));
	rc |= inv_imu_set_accel_frequency(&s->icm_driver, accel_freq_to_param(s->rate_hz));
	rc |= inv_imu_set_accel_mode(&s->icm_driver, PWR_MGMT0_ACCEL_MODE_LN);

	//STARTGYRO
	rc |= inv_imu_set_gyro_fsr(&s->icm_driver, gyro_fsr_dps_to_param(ICM45686_GYRO_FSR));
	rc |= inv_imu_set_gyro_frequency(&s->icm_driver, gyro_freq_to_param(s->rate_hz));
	rc |= inv_imu_set_gyro_mode(&s->icm_driver, PWR_MGMT0_GYRO_MODE_LN);

	// Wait for PLL (MCLK) to be ready before accessing MREG registers.
	// IPREG_SYS1/SYS2 require MCLK; accessing them while the PLL is still
	// starting up stalls the ICM I2C bus, triggering clock_stretch_timeout
	// and breaking all subsequent reads.  PLL is typically ready ~10 ms after
	// LN mode is enabled on a cold start.
	{
		int1_status1_t st1 = {0};
		for (int i = 0; i < 100; i++) {
			sleep_us(1000);
			inv_imu_read_reg(&s->icm_driver, INT1_STATUS1, 1, (uint8_t *)&st1);
			if (st1.int1_status_pll_rdy) break;
		}
	}

	// Enable FIR filter + interpolator for accel and gyro (low-noise signal chain)
	ipreg_sys2_reg_123_t ipreg_sys2_reg_123;
	rc |= inv_imu_read_reg(&s->icm_driver, IPREG_SYS2_REG_123, 1, (uint8_t *)&ipreg_sys2_reg_123);
	ipreg_sys2_reg_123.accel_src_ctrl = IPREG_SYS2_REG_123_ACCEL_SRC_CTRL_INTERPOLATOR_ON_FIR_ON;
	rc |= inv_imu_write_reg(&s->icm_driver, IPREG_SYS2_REG_123, 1, (uint8_t *)&ipreg_sys2_reg_123);

	ipreg_sys1_reg_166_t ipreg_sys1_reg_166;
	rc |= inv_imu_read_reg(&s->icm_driver, IPREG_SYS1_REG_166, 1, (uint8_t *)&ipreg_sys1_reg_166);
	ipreg_sys1_reg_166.gyro_src_ctrl = IPREG_SYS1_REG_166_GYRO_SRC_CTRL_INTERPOLATOR_ON_FIR_ON;
	rc |= inv_imu_write_reg(&s->icm_driver, IPREG_SYS1_REG_166, 1, (uint8_t *)&ipreg_sys1_reg_166);

	return (rc == INV_IMU_OK);
}

static THD_FUNCTION(icm_thread, arg) {
	ICM45686_STATE *s = (ICM45686_STATE*)arg;

	chRegSetThreadName("ICM Sampling");

	s->is_running = true;

	for(;;) {
		//getDataFromRegisters
		inv_imu_sensor_data_t imu_data;
		int res = inv_imu_get_register_data(&s->icm_driver, &imu_data);

		if (res == INV_IMU_OK) {
			float accel[3], gyro[3], mag[3];

			accel[0] = imu_data.accel_data[0] * (float)ICM45686_ACCEL_FSR / 32768.0;
			accel[1] = imu_data.accel_data[1] * (float)ICM45686_ACCEL_FSR / 32768.0;
			accel[2] = imu_data.accel_data[2] * (float)ICM45686_ACCEL_FSR / 32768.0;

			gyro[0] = imu_data.gyro_data[0] * (float)ICM45686_GYRO_FSR / 32768.0;
			gyro[1] = imu_data.gyro_data[1] * (float)ICM45686_GYRO_FSR / 32768.0;
			gyro[2] = imu_data.gyro_data[2] * (float)ICM45686_GYRO_FSR / 32768.0;

			// TODO: Read magnetometer as well
			memset(mag, 0, sizeof(mag));

			if (s->read_callback) {
				s->read_callback(accel, gyro, mag);
			}
		} else {
			reset_init_icm(s);
			chThdSleepMilliseconds(10);
		}

		if (s->should_stop) {
			s->is_running = false;
			return;
		}

		chThdSleepMicroseconds(1000000 / s->rate_hz);
	}
}
