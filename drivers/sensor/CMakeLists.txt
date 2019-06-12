#
# Copyright (c) 2019 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

add_subdirectory_ifdef(CONFIG_BH1749 bh1749)
add_subdirectory_ifdef(CONFIG_SENSOR_SIM sensor_sim)
