#-------------------------------------------------------------------------------
# Zephyr Example Application
#
# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(pickleball-sensor)

FILE(GLOB app_sources src/*.c ${ZEPHYR_BASE}/samples/bluetooth/central_gatt_write/src/gatt_write_common.c)

target_sources(app PRIVATE ${app_sources})
