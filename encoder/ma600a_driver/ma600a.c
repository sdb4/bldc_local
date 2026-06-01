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

#include "ma600a.h"

/*====================================================================================*/
/*=============================== Private ===========================================*/
/*====================================================================================*/

/* RMAPID register is at address 0x1F (31) per Table 10 of the datasheet.
   There is no separate register at 0x1E; REG_REGMAPID must equal REG_PID. */
#define REG_REGMAPID  31
#define REG_PID       31

/* Helper macros that expand dev->iface.* with a resolved dev pointer */
#define _SPI16(dev, d)   ((dev)->iface.spiTransfer16((d)))
#define _CS(dev, lvl)    ((dev)->iface.csSetLevel((dev)->csPin, (lvl)))
#define _PINMODE(dev, p) ((dev)->iface.pinModeOutput((p)))
#define _DELAY_US(dev, u)((dev)->iface.delayUs((u)))

/*====================================================================================*/
/*=========================== Public API ===========================================*/
/*====================================================================================*/

void MA600A_init(MA600A_Device* dev, uint8_t csPin, MA600A_Interface iface) {
    dev->csPin = csPin;
    dev->iface = iface;
    _PINMODE(dev, csPin);
    _CS(dev, 0); /* deassert */
}

void MA600A_deinit(MA600A_Device* dev) {
    (void)dev;
    /* Bus ownership is managed by the platform layer; nothing to free here */
}

uint16_t MA600A_readAngleRaw16(MA600A_Device* dev) {
    uint16_t angle;
    _CS(dev, 1);
    angle = _SPI16(dev, 0x0000);
    _CS(dev, 0);
    return angle;
}

uint8_t MA600A_readRegister(MA600A_Device* dev, uint8_t address) {
    uint8_t value;
    _CS(dev, 1);
    _SPI16(dev, (uint16_t)(0xD200U | address)); /* READ_REG_COMMAND | address */
    _CS(dev, 0);
    _DELAY_US(dev, 1);
    _CS(dev, 1);
    value = _SPI16(dev, 0x0000) & 0x00FF;
    _CS(dev, 0);
    _DELAY_US(dev, 1);
    return value;
}

uint16_t MA600A_getZero(MA600A_Device* dev) {
    return ((uint16_t)MA600A_readRegister(dev, 1) << 8) |
            MA600A_readRegister(dev, 0);
}

uint16_t MA600A_getBct(MA600A_Device* dev) {
    return MA600A_readRegister(dev, 2);
}

/* The MA600A has one identification register: RMAPID (0x1F) containing
   SUFFIXID[7:0] — an 8-bit register map configuration identifier.
   There is no separate silicon ID or silicon revision register.
   getSiliconId returns the lower nibble; getSiliconRevision returns the
   upper nibble, splitting the single SUFFIXID byte between the two calls. */
uint8_t MA600A_getSiliconId(MA600A_Device* dev) {
    return MA600A_readRegister(dev, REG_PID) & 0x0F;
}

uint8_t MA600A_getSiliconRevision(MA600A_Device* dev) {
    return (MA600A_readRegister(dev, REG_PID) >> 4) & 0x0F;
}

uint8_t MA600A_getRegisterMapRevision(MA600A_Device* dev) {
    /* RMAPID (0x1F) SUFFIXID is a full 8-bit field per Table 10. */
    return MA600A_readRegister(dev, REG_REGMAPID) & 0xFF;
}
