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
#include <drivers/uart.h>
#include <console/console.h>
#include <net/net_config.h>
#include <cmath>

#define MAVLINK_NO_CONVERSION_HELPERS
#include "mavlink.h"
#include "common.h"

#define STACKSIZE 4096
#define UART_BUF_MAXSIZE 279
#define M_PI      3.14159265358979323846

const int MAVLINK_MAIN_CHANNEL = 0;
const int UWB_COMPONENT_ID = 25;

/*
 * THREADS and FIFOS
 */

int64_t time_stamp;
struct k_thread mavlink_receiver_thread;
struct k_thread distance_calculator_thread;

K_THREAD_STACK_DEFINE(thread_stack, STACKSIZE);

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
	float& w = quaternion[0];
	float& x = quaternion[1];
	float& y = quaternion[2];
	float& z = quaternion[3];

	// taken from https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm
	float c1 = cos(rad(yaw/2));
	float c2 = cos(rad(pitch/2));
	float c3 = cos(rad(roll/2));
	float s1 = sin(rad(yaw/2));
	float s2 = sin(rad(pitch/2));
	float s3 = sin(rad(roll/2));

	w = c1 * c2 * c3 - s1 * s2 * s3;
	x = s1 * s2 * c3 + c1 * c2 * s3;
	y = s1 * c2 * s3 + c1 * s2 * c3;
	z = c1 * s2 * s3 - s1 * c2 * c3;
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

// #define MAVLINK_SEND_UART_BYTES mavlink_send_uart_bytes

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

mavlink_status_t mavlink_status;
mavlink_message_t mavlink_msg;

void mavlink_receiver_thread_entry(void)
{
	while (1) {
		uint8_t c = console_getchar();
		if (mavlink_parse_char(MAVLINK_MAIN_CHANNEL, c, &mavlink_msg, &mavlink_status)) {
			handle_message(mavlink_msg);
			// printk("Received message with ID %d, sequence: %d from component %d of system %d", msg.msgid, msg.seq, msg.compid, msg.sysid);
		}
	}
}

/*
 * UART communication
 */

static const struct device* uart_dev;

static void uart_isr(const struct device* dev, void* userdata) {
	static uint8_t rx_buf[UART_BUF_MAXSIZE];
	uint32_t partial_size = UART_BUF_MAXSIZE;
	uint32_t total_size = 0;
	uint8_t* dst = rx_buf;
	while (uart_irq_update(uart_dev) 
		// && uart_irq_rx_ready(uart_dev)
		&& uart_irq_is_pending(uart_dev)
	) {
		if (!uart_irq_rx_ready(uart_dev)) continue;
		int len = uart_fifo_read(dev, dst, partial_size);
		if (len <= 0) continue;
		dst += len;
		partial_size -= len;
		total_size += len;
	}
	for(int i=0; i<total_size; ++i) {
		if (mavlink_parse_char(MAVLINK_MAIN_CHANNEL, rx_buf[i], &mavlink_msg, &mavlink_status)) {
			handle_message(mavlink_msg);
		}
	}
}

static void uart_init(void)
{
	uart_dev = device_get_binding("UART_3");

	uart_irq_rx_disable(uart_dev);
	//uart_irq_tx_disable(uart_dev);
	uart_irq_callback_set(uart_dev, uart_isr);
	uart_irq_rx_enable(uart_dev);
}

void mavlink_send_uart_bytes(const uint8_t *ch, int length)
{
	while(length--) {
		uart_poll_out(uart_dev, *ch++);
	}
	// uart_tx(uart_dev, ch, length, SYS_FOREVER_MS);
}


void distance_calculator_thread_entry(void) {
	const uint8_t SYS_IDS = 2;
	const uint8_t OUR_ID = 1;
	const uint8_t THEIR_ID = 2;
	// Coordinates of the Elbphilharmonie in Hamburg
	struct coordinates coords[SYS_IDS+1];
	coords[OUR_ID].sys_id = 0;
	coords[THEIR_ID].sys_id = 2;
	coords[THEIR_ID].latitude = 53.541350;
	coords[THEIR_ID].longitude = 9.985102;
	coords[THEIR_ID].altitude = 10.0;
	struct k_timer timer;

	k_timer_init(&timer, NULL, NULL);

	struct coordinates* new_coords;
	k_timer_start(&timer, K_MSEC(500), K_NO_WAIT);
	while (1) {
		k_timer_status_sync(&timer);
		bool new_data = false;
		// we publish distances twice a second
		k_timer_start(&timer, K_MSEC(500), K_NO_WAIT);
		while((new_coords = reinterpret_cast<struct coordinates*>(k_fifo_get(&coords_fifo, K_NO_WAIT)))) {
			memcpy(&coords[new_coords->sys_id], new_coords, sizeof(struct coordinates));
			k_free(new_coords);
			new_data = true;
		}
		if (coords[OUR_ID].sys_id == 0) continue;
		uint8_t send_buf[51]; // DISTANCE_SENSOR has at most 51 bytes, where as MAVLink 2.0 messages can be as large as 279 bytes
		mavlink_message_t mav_msg;
		mavlink_distance_sensor_t msg = {};
		for (int i=1; i<=SYS_IDS; ++i) {
			if (i==OUR_ID) continue; // do not calculate distance to us
			msg.time_boot_ms = k_uptime_delta(&time_stamp);
			msg.id = i;
			msg.type = MAV_DISTANCE_SENSOR_ULTRASOUND; // TODO: suggest additional types in github.com/mavlink/mavlink
			msg.orientation = MAV_SENSOR_ROTATION_CUSTOM;
			msg.min_distance = 1; // in cm; to be determined
			msg.max_distance = 10000; // in cm; to be determined
			msg.current_distance = calculate_distance(coords[OUR_ID], coords[i]); // send for now in m TODO: * 100; // in cm
			float roll = 0.0f; // TODO: get roll angle from ATTITUDE from flight controller
			float pitch = 0.0f; // TODO: get pitch angle from ATTITUDE from flight controller
			float yaw = calculate_heading(coords[OUR_ID], coords[i]); // TODO: compensate yaw angle from ATTITUDE from flight controller
			float quaternion[4];
			euler_to_quaternion(roll, pitch, yaw, quaternion); 
			// need this indirect way, otherwise we get misaligned memory accesses
			memcpy(msg.quaternion, quaternion, sizeof(quaternion));

			msg.covariance = 0.0f;

			mavlink_msg_distance_sensor_encode(OUR_ID, UWB_COMPONENT_ID, &mav_msg, &msg);
			int n = mavlink_msg_to_send_buffer(send_buf, &mav_msg);
			mavlink_send_uart_bytes(send_buf, n);
		}
	}
}

void main(void)
{
	time_stamp = k_uptime_get();
	// console_init();
	uart_init();
	/*k_thread_create(&mavlink_receiver_thread, thread_stack, STACKSIZE,
			(k_thread_entry_t) mavlink_receiver_thread_entry,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
	*/

	k_thread_create(&distance_calculator_thread, thread_stack, STACKSIZE,
			(k_thread_entry_t) distance_calculator_thread_entry,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
	struct k_timer timer;

	k_timer_init(&timer, NULL, NULL);
	while (1) {
		// doing nothing here
		k_timer_start(&timer, K_MSEC(1000), K_NO_WAIT);
		k_timer_status_sync(&timer);
	}
}
