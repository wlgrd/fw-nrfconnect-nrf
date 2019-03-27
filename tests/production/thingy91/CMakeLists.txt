#
# Copyright (c) 2018 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

include($ENV{ZEPHYR_BASE}/../nrf/cmake/boilerplate.cmake)

cmake_minimum_required(VERSION 3.8.2)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(thingy91)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
