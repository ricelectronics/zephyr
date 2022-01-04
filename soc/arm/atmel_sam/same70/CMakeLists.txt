# SPDX-License-Identifier: Apache-2.0
zephyr_include_directories(${ZEPHYR_BASE}/drivers)

zephyr_sources(
  soc.c
  soc_config.c
  )
