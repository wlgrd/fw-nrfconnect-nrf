# Kconfig - nRF91 PCA20035 board configuration
#
# Copyright (c) 2019 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic

if(CONFIG_TRUSTED_EXECUTION_NONSECURE)

zephyr_library()
zephyr_library_sources(board_nonsecure.c)

endif()
