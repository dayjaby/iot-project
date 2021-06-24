/*
 * Copyright (c) 2015-2016 Wind River Systems, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file C++ Synchronization demo.  Uses basic C++ functionality.
 */

#define LOG_LEVEL LOG_LEVEL_DBG
#include <logging/log.h>
#include <stdio.h>
#include <zephyr.h>
#include <arch/cpu.h>
#include <sys/printk.h>

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <console/console.h>
#include <net/net_config.h>
#include <cmath>

#define MAVLINK_NO_CONVERSION_HELPERS
#include "mavlink.h"
#include "common.h"

#define STACKSIZE 1024
#define M_PI      3.14159265358979323846

struct k_thread coop_thread;

K_THREAD_STACK_DEFINE(coop_stack, STACKSIZE);

struct coordinates {
	void* fifo_reserved;
	uint8_t sys_id;
	float latitude;
	float longitude;
	float altitude;
};

K_FIFO_DEFINE(coords_fifo);
//K_MUTEX_DEFINE(own_coords_mutex);
//K_MUTEX_DEFINE(their_coords_mutex);


inline float rad(float degrees) {
	return degrees * M_PI / 180.0;
}

inline float deg(float radians) {
	return radians * 180.0 / M_PI;
}

void euler_to_quaternion(float roll, float pitch, float yaw, float* quaternion) {
	float& w = *quaternion;
	float& x = *(quaternion + 1);
	float& y = *(quaternion + 2);
	float& z = *(quaternion + 3);
}

float calculate_heading(const coordinates& p1, const coordinates& p2) {
	float y = sin(rad(p2.longitude - p1.longitude)) * cos(rad(p2.latitude));
	float x = cos(rad(p1.latitude)) * sin(rad(p2.latitude));
	x -= sin(rad(p1.latitude)) * cos(rad(p2.latitude)) * cos(rad(p2.longitude - p1.longitude));
	return deg(atan2(y, x));	
}

float calculate_distance(const coordinates& p1, const coordinates& p2) {
	float r = 6371000.0;
	float a = pow(sin(rad(p2.latitude - p1.latitude) / 2), 2);
	float dlon = rad(p2.longitude - p1.longitude) / 2;
	a += cos(rad(p1.latitude)) * cos(rad(p2.latitude)) * pow(sin(dlon), 2);
	return r * 2 * atan2(sqrt(a), sqrt(1-a));
}

void handle_gps_raw_int(uint8_t sys_id, mavlink_gps_raw_int_t& gps_raw_int);

void handle_message(mavlink_message_t& msg) {
	switch(msg.msgid) {
	case MAVLINK_MSG_ID_GPS_RAW_INT:
		mavlink_gps_raw_int_t gps_raw_int;
		mavlink_msg_gps_raw_int_decode(&msg, &gps_raw_int);
		handle_gps_raw_int(msg.sysid, gps_raw_int);
	}
}

void handle_gps_raw_int(uint8_t sys_id, mavlink_gps_raw_int_t& gps_raw_int) {
	size_t size = sizeof(struct coordinates);
        void* mem_ptr = k_malloc(size);
	struct coordinates* coords = reinterpret_cast<struct coordinates*>(mem_ptr);
	coords->sys_id = sys_id;
	coords->latitude = gps_raw_int.lat / 1e7;
	coords->longitude = gps_raw_int.lon / 1e7;
	coords->altitude = gps_raw_int.alt / 1e3;
	k_fifo_put(&coords_fifo, mem_ptr);
}

void coop_thread_entry(void)
{
	mavlink_status_t status;
	mavlink_message_t msg;
	int chan = 0;
	while (1) {
		uint8_t c = console_getchar();
		if (mavlink_parse_char(chan, c, &msg, &status)) {
			handle_message(msg);
			// printk("Received message with ID %d, sequence: %d from component %d of system %d", msg.msgid, msg.seq, msg.compid, msg.sysid);
		}
	}
}

void main(void)
{
	const uint8_t SYS_IDS = 2;
	const uint8_t OUR_ID = 1;
	const uint8_t THEIR_ID = 2;
	// Coordinates of the Elbphilharmonie in Hamburg
	struct coordinates coords[SYS_IDS+1];
	coords[OUR_ID].sys_id = 1;
	coords[THEIR_ID].sys_id = 2;
	coords[THEIR_ID].latitude = 53.541350;
	coords[THEIR_ID].longitude = 9.985102;
	coords[THEIR_ID].altitude = 10.0;
	console_init();

	struct k_timer timer;

	k_thread_create(&coop_thread, coop_stack, STACKSIZE,
			(k_thread_entry_t) coop_thread_entry,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
	k_timer_init(&timer, NULL, NULL);

	struct coordinates* new_coords;
	while (1) {
		while((new_coords = reinterpret_cast<struct coordinates*>(k_fifo_get(&coords_fifo, K_NO_WAIT)))) {
			memcpy(&coords[new_coords->sys_id], new_coords, sizeof(struct coordinates));
			k_free(new_coords);
		}
		printk("%f meters at %f degrees towards Elbphilharmonie\n", 
			calculate_distance(coords[OUR_ID], coords[THEIR_ID]),
			calculate_heading(coords[OUR_ID], coords[THEIR_ID])
		);
		// printk("%s: Hello World!\n", __FUNCTION__);
		k_timer_start(&timer, K_MSEC(500), K_NO_WAIT);
		// printk("%s: Hello World!\n", __FUNCTION__);
		k_timer_status_sync(&timer);
	}
}
