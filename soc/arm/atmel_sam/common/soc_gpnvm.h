/*
 * Copyright (c) 2021 RIC Electronics
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Atmel SAM MCU family General Purpose NVM Driver
 */

#ifndef _ATMEL_SAM_SOC_GPNVM_H_
#define _ATMEL_SAM_SOC_GPNVM_H_

#include <zephyr/types.h>
#include <soc.h>

uint32_t soc_gpnvm_read_bits(void);

void soc_gpnvm_erase_bit(uint8_t bit);

void soc_gpnvm_set_bit(uint8_t bit);

#endif
