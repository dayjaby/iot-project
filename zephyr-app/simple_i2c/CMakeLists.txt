# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cpp_synchronization)
set(mavlink_dir ${CMAKE_CURRENT_SOURCE_DIR}/generated/include/mavlink/v2.0/common)

include_directories(
        ${mavlink_dir}
	include
)

target_sources(app PRIVATE src/main.cpp)
