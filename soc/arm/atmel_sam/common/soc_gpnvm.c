/*
 * Copyright (c) 2021 RIC Electronics
 * SPDX-License-Identifier: Apache-2.0
 */

#include "soc_gpnvm.h"

uint32_t soc_gpnvm_read_bits(void)
{
	EFC->EEFC_FCR = EEFC_FCR_FKEY_PASSWD
			| EEFC_FCR_FCMD_GGPB;
	__DSB();

	return EFC->EEFC_FRR;
}

void soc_gpnvm_erase_bit(uint8_t bit)
{
	EFC->EEFC_FCR = EEFC_FCR_FKEY_PASSWD
			| EEFC_FCR_FCMD_CGPB
			| EEFC_FCR_FARG(bit);
	__DSB();
	while (!(EFC->EEFC_FSR & EEFC_FSR_FRDY_Msk)) {

	}
}

void soc_gpnvm_set_bit(uint8_t bit)
{
	EFC->EEFC_FCR = EEFC_FCR_FKEY_PASSWD
			| EEFC_FCR_FCMD_SGPB
			| EEFC_FCR_FARG(bit);
	__DSB();
	while (!(EFC->EEFC_FSR & EEFC_FSR_FRDY_Msk)) {
		;
	}
}
