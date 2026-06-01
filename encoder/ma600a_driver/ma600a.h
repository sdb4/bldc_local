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

#ifndef MA600A_H
#define MA600A_H

#include <stdint.h>

//Inspired from https://github.com/monolithicpower/MagAlpha-Arduino-Library

/*====================================================================================*/
/*=============================== Interface =========================================*/
/*====================================================================================*/

/**
 * @brief Hardware abstraction layer for SPI and timing.
 *        Implement these to port the driver to any platform.
 */
typedef struct MA600A_Interface {
    /** @brief Transfer 16 bits over SPI, return received 16-bit word */
    uint16_t (*spiTransfer16)(uint16_t txData);

    /** @brief Assert or deassert chip-select (true = LOW, false = HIGH) */
    void (*csSetLevel)(uint8_t csPin, uint8_t level);

    /** @brief Set a pin as output */
    void (*pinModeOutput)(uint8_t pin);

    /** @brief Delay in microseconds */
    void (*delayUs)(uint32_t us);

    /** @brief Delay in milliseconds */
    void (*delayMs)(uint32_t ms);

    /** @brief Optional user data passed through to callbacks (may be NULL) */
    void* userData;
} MA600A_Interface;

/*====================================================================================*/
/*=============================== Device ===========================================*/
/*====================================================================================*/

/**
 * @brief Device handle — one per sensor instance.
 *        Allocate this in caller (stack or heap) and pass to MA600A_init().
 */
typedef struct MA600A_Device {
    uint8_t              csPin;
    MA600A_Interface     iface;
    /* internal state */
    uint8_t              _regCmdRead;
    uint8_t              _regCmdWrite;
} MA600A_Device;

/*====================================================================================*/
/*=============================== Public API =======================================*/
/*====================================================================================*/

/**
 * @brief Initialize a MA600A device instance
 * @param dev      Pointer to device structure (caller allocates)
 * @param csPin    Arduino pin connected to CS (chip select)
 * @param iface    Interface function table
 */
void MA600A_init(MA600A_Device* dev, uint8_t csPin, MA600A_Interface iface);

/**
 * @brief Deinitialize the device (stops SPI bus)
 */
void MA600A_deinit(MA600A_Device* dev);

/**
 * @brief Read raw 16-bit angle count
 */
uint16_t MA600A_readAngleRaw16(MA600A_Device* dev);

/*=============================== Registers ================================*/

/**
 * @brief Read a single register
 * @param address  Register address (0-159)
 */
uint8_t MA600A_readRegister(MA600A_Device* dev, uint8_t address);

/*=============================== Zero / BCT ================================*/

/**
 * @brief Get zero position offset
 */
uint16_t MA600A_getZero(MA600A_Device* dev);

/**
 * @brief Get BCT (bandwidth/configuration trim)
 */
uint16_t MA600A_getBct(MA600A_Device* dev);

/*=============================== Part Info ================================*/

/**
 * @brief Get silicon ID
 */
uint8_t MA600A_getSiliconId(MA600A_Device* dev);

/**
 * @brief Get silicon revision
 */
uint8_t MA600A_getSiliconRevision(MA600A_Device* dev);

/**
 * @brief Get register map revision
 */
uint8_t MA600A_getRegisterMapRevision(MA600A_Device* dev);

#endif /* MA600A_H */
