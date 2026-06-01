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

#include "enc_ma600a.h"
#include "spi_bb.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "hw.h"
#include "timer.h"
#include "terminal.h"
#include "commands.h"
#include "utils_math.h"

#include <stdlib.h>
#include <string.h>

// Module-level pointer to the active config for use by SPI callbacks.
// Only one MA600A encoder instance is active at a time.
static MA600A_config_t *s_cfg = NULL;

/*====================================================================================*/
/* Hardware SPI callbacks                                                             */
/*====================================================================================*/

static uint16_t ma600a_spi_transfer16(uint16_t tx) {
    uint8_t txbuf[2] = { (uint8_t)(tx >> 8), (uint8_t)(tx & 0xFF) };
    uint8_t rxbuf[2] = { 0, 0 };
    spiExchange(s_cfg->spi_dev, 2, txbuf, rxbuf);
    return ((uint16_t)rxbuf[0] << 8) | rxbuf[1];
}

// MA600A convention: level=1 → assert (CS low), level=0 → deassert (CS high)
static void ma600a_cs_set_level(uint8_t csPin, uint8_t level) {
    (void)csPin;
    if (level) {
        palClearPad(s_cfg->nss_gpio, s_cfg->nss_pin);
    } else {
        palSetPad(s_cfg->nss_gpio, s_cfg->nss_pin);
    }
}

/*====================================================================================*/
/* Software bit-bang SPI callbacks                                                    */
/*====================================================================================*/

static uint16_t ma600a_sw_spi_transfer16(uint16_t tx) {
    uint16_t rx = 0;
    spi_bb_transfer_16(&s_cfg->sw_spi, &rx, &tx, 1);
    return rx;
}

// CS for SW SPI: mirror spi_bb_begin/end behaviour (delay included)
static void ma600a_sw_cs_set_level(uint8_t csPin, uint8_t level) {
    (void)csPin;
    if (level) {
        palClearPad(s_cfg->sw_spi.nss_gpio, s_cfg->sw_spi.nss_pin);
        spi_bb_delay();
    } else {
        spi_bb_delay();
        palSetPad(s_cfg->sw_spi.nss_gpio, s_cfg->sw_spi.nss_pin);
    }
}

/*====================================================================================*/
/* Shared callbacks                                                                   */
/*====================================================================================*/

static void ma600a_pin_mode_output(uint8_t pin) {
    (void)pin; // All GPIO directions are configured during enc_ma600a_init
}

static void ma600a_delay_us(uint32_t us) {
    chSysPolledDelayX(US2RTC(STM32_SYSCLK, us));
}

static void ma600a_delay_ms(uint32_t ms) {
    chThdSleepMilliseconds(ms);
}

/*====================================================================================*/
/* Terminal commands                                                                  */
/*====================================================================================*/

static void terminal_ma600a_read_raw(int argc, const char **argv) {
    int count = 8;
    if (argc >= 2) {
        count = atoi(argv[1]);
        if (count < 1 || count > 64) {
            commands_printf("count must be 1-64");
            return;
        }
    }

    MA600A_config_t *cfg = s_cfg;
    bool hw = (cfg->spi_dev != NULL);

    commands_printf("MA600A raw reads (%s):", hw ? "HW SPI" : "SW SPI");

    for (int i = 0; i < count; i++) {
        uint16_t raw;
        if (hw) {
            spiAcquireBus(cfg->spi_dev);
            raw = MA600A_readAngleRaw16(&cfg->device);
            spiReleaseBus(cfg->spi_dev);
        } else {
            raw = MA600A_readAngleRaw16(&cfg->device);
        }
        commands_printf("  [%2d] 0x%04X  %.2f deg", i, raw, (double)(raw * (360.0f / 65536.0f)));
        chThdSleepMilliseconds(2);
    }
    commands_printf(" ");
}

static void terminal_ma600a_info(int argc, const char **argv) {
    (void)argc; (void)argv;

    MA600A_config_t *cfg = s_cfg;
    bool hw = (cfg->spi_dev != NULL);

    if (hw) spiAcquireBus(cfg->spi_dev);

    uint8_t  sil_id  = MA600A_getSiliconId(&cfg->device);
    uint8_t  sil_rev = MA600A_getSiliconRevision(&cfg->device);
    uint8_t  reg_rev = MA600A_getRegisterMapRevision(&cfg->device);
    uint16_t zero    = MA600A_getZero(&cfg->device);
    uint16_t bct     = MA600A_getBct(&cfg->device);

    if (hw) spiReleaseBus(cfg->spi_dev);

    commands_printf("MA600A info (%s):", hw ? "HW SPI" : "SW SPI");
    commands_printf("  Silicon ID       : 0x%02X (%s)", sil_id,
            (sil_id == 0x00 || sil_id == 0xFF) ? "INVALID - check wiring/power" : "ok");
    commands_printf("  Silicon revision : 0x%02X", sil_rev);
    commands_printf("  Reg-map revision : 0x%02X", reg_rev);
    commands_printf("  Zero offset      : 0x%04X (%.2f deg)", zero,
            (double)(zero * (360.0f / 65536.0f)));
    commands_printf("  BCT              : 0x%04X", bct);

    commands_printf("  Registers 0-7:");
    if (hw) spiAcquireBus(cfg->spi_dev);
    for (int r = 0; r <= 7; r++) {
        uint8_t val = MA600A_readRegister(&cfg->device, (uint8_t)r);
        commands_printf("    reg[%d] = 0x%02X", r, val);
    }
    if (hw) spiReleaseBus(cfg->spi_dev);

    uint16_t raw;
    if (hw) {
        spiAcquireBus(cfg->spi_dev);
        raw = MA600A_readAngleRaw16(&cfg->device);
        spiReleaseBus(cfg->spi_dev);
    } else {
        raw = MA600A_readAngleRaw16(&cfg->device);
    }
    commands_printf("  Raw angle now    : 0x%04X  %.2f deg", raw,
            (double)(raw * (360.0f / 65536.0f)));
    commands_printf(" ");
}

/*====================================================================================*/
/* Public API                                                                         */
/*====================================================================================*/

bool enc_ma600a_init(MA600A_config_t *cfg) {
    memset(&cfg->state, 0, sizeof(MA600A_enc_state_t));
    s_cfg = cfg;

    MA600A_Interface iface = {
        .pinModeOutput = ma600a_pin_mode_output,
        .delayUs       = ma600a_delay_us,
        .delayMs       = ma600a_delay_ms,
        .userData      = NULL,
    };

    if (cfg->spi_dev != NULL) {
        // ---- Hardware SPI path ----
        palSetPadMode(cfg->sck_gpio,  cfg->sck_pin,
                PAL_MODE_ALTERNATE(cfg->spi_af) | PAL_STM32_OSPEED_HIGHEST);
        palSetPadMode(cfg->miso_gpio, cfg->miso_pin,
                PAL_MODE_ALTERNATE(cfg->spi_af) | PAL_STM32_OSPEED_HIGHEST);
        if (cfg->mosi_gpio != NULL) {
            palSetPadMode(cfg->mosi_gpio, cfg->mosi_pin,
                    PAL_MODE_ALTERNATE(cfg->spi_af) | PAL_STM32_OSPEED_HIGHEST);
        }
        palSetPadMode(cfg->nss_gpio, cfg->nss_pin,
                PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
        palSetPad(cfg->nss_gpio, cfg->nss_pin); // deassert CS

        spiStart(cfg->spi_dev, &cfg->hw_spi_cfg);

        iface.spiTransfer16 = ma600a_spi_transfer16;
        iface.csSetLevel    = ma600a_cs_set_level;
    } else if (cfg->sw_spi.sck_gpio != NULL) {
        // ---- Software bit-bang SPI path ----
        spi_bb_init(&cfg->sw_spi);

        iface.spiTransfer16 = ma600a_sw_spi_transfer16;
        iface.csSetLevel    = ma600a_sw_cs_set_level;
    } else {
        s_cfg = NULL;
        return false;
    }

    // csPin=0: the csPin argument is unused; CS is driven via the callbacks above
    MA600A_init(&cfg->device, 0, iface);

    terminal_register_command_callback(
            "ma600a_read_raw",
            "Read raw 16-bit angle words from the MA600A.",
            "[count]",
            terminal_ma600a_read_raw);

    terminal_register_command_callback(
            "ma600a_info",
            "Read MA600A silicon ID, revision, zero offset, BCT, and registers 0-7.",
            0,
            terminal_ma600a_info);

    return true;
}

void enc_ma600a_deinit(MA600A_config_t *cfg) {
    if (cfg->spi_dev == NULL && cfg->sw_spi.sck_gpio == NULL) {
        return;
    }

    terminal_unregister_callback(terminal_ma600a_read_raw);
    terminal_unregister_callback(terminal_ma600a_info);

    MA600A_deinit(&cfg->device);

    if (cfg->spi_dev != NULL) {
        spiStop(cfg->spi_dev);
        palSetPadMode(cfg->miso_gpio, cfg->miso_pin, PAL_MODE_INPUT_PULLUP);
        palSetPadMode(cfg->sck_gpio,  cfg->sck_pin,  PAL_MODE_INPUT_PULLUP);
        palSetPadMode(cfg->nss_gpio,  cfg->nss_pin,  PAL_MODE_INPUT_PULLUP);
        if (cfg->mosi_gpio != NULL) {
            palSetPadMode(cfg->mosi_gpio, cfg->mosi_pin, PAL_MODE_INPUT_PULLUP);
        }
    } else {
        spi_bb_deinit(&cfg->sw_spi);
    }

    cfg->state.last_enc_angle = 0.0f;
    cfg->state.spi_error_rate = 0.0f;
    s_cfg = NULL;
}

void enc_ma600a_routine(MA600A_config_t *cfg) {
    float timestep = timer_seconds_elapsed_since(cfg->state.last_update_time);
    if (timestep > 1.0f) {
        timestep = 1.0f;
    }
    cfg->state.last_update_time = timer_time_now();

    // Use readAngleRaw16 (one 2-byte transfer, CS asserted once).
    // MA600A_readAngleRaw requires the PRT parity-enable bit to be set in the
    // sensor's PRT register, which is 0 by default. Using it without first
    // enabling parity causes the extra SPI8 read to clock in undefined MISO
    // state and produce false parity errors on every read.
    uint16_t raw;
    if (cfg->spi_dev != NULL) {
        spiAcquireBus(cfg->spi_dev);
        raw = MA600A_readAngleRaw16(&cfg->device);
        spiReleaseBus(cfg->spi_dev);
    } else {
        // SW SPI: spi_bb_transfer_16 uses its own internal mutex
        raw = MA600A_readAngleRaw16(&cfg->device);
    }

    // MA600A output is 0x0000 when MISO is floating (e.g. device not powered).
    // Treat a zero read as an error to detect disconnection.
    if (raw != 0x0000) {
        cfg->state.last_enc_angle = (float)raw * (360.0f / 65536.0f);
        UTILS_LP_FAST(cfg->state.spi_error_rate, 0.0f, timestep);
    } else {
        ++cfg->state.spi_error_cnt;
        UTILS_LP_FAST(cfg->state.spi_error_rate, 1.0f, timestep);
    }
}
