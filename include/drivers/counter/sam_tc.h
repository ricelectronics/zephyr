/*
 * Copyright (c) 2022 RIC Electronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_COUNTER_SAM_TC_H_
#define ZEPHYR_INCLUDE_DRIVERS_COUNTER_SAM_TC_H_

#include <device.h>
#include <drivers/counter.h>

int sam_counter_read_capture(const struct device *dev, uint32_t *period);


#endif /* ZEPHYR_INCLUDE_DRIVERS_COUNTER_SAM_TC_H_ */
