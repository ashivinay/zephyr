cmake_minimum_required(VERSION 3.20.0)
# Copyright (c) 2018 Nordic Semiconductor ASA
# Copyright (c) 2019 Linaro Limited
#
# SPDX-License-Identifier: Apache-2.0
#

set(REMOTE_ZEPHYR_DIR ${CMAKE_CURRENT_BINARY_DIR}/../openamp_remote/zephyr)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(openamp)

enable_language(C ASM)

target_sources(app PRIVATE src/main.c)

# Now that targets are created, handle RT1170 image dependency
if("${BOARD}" STREQUAL "mimxrt1170_evk_cm7")
  target_include_directories(zephyr_interface INTERFACE ${REMOTE_ZEPHYR_DIR}/include/public)
endif()

# Now that targets are created, handle RT1170 image dependency
if("${BOARD}" STREQUAL "mimxrt1170_evk_cm7")
  target_include_directories(zephyr_interface
    INTERFACE ${REMOTE_ZEPHYR_DIR}/include/public)
endif()

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
