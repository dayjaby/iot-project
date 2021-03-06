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
#include <cmath>

// Device Tree
#include <device.h>
#include <devicetree.h>

// Drivers
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <drivers/uart.h>
#include <drivers/i2c.h>

// Storage
#include <storage/flash_map.h>

// Network
#include <net/ieee802154_radio.h>
#include <net/net_config.h>

// MAVLink
#define MAVLINK_NO_CONVERSION_HELPERS
#include "mavlink.h"
#include "common.h"

// #define MAVLINK_DEBUG_MODE
const int MAVLINK_MAIN_CHANNEL = 0;
const int UWB_COMPONENT_ID = 25;

/* ieee802.15.4 device */
static struct ieee802154_radio_api *radio_api;
static const struct device *ieee802154_dev;
uint8_t mac_addr[8];

/*
 * THREADS and FIFOS
 */

#define STACKSIZE 4096
int64_t time_stamp;
// struct k_thread mavlink_receiver_thread;
struct k_thread read_range_thread;
struct k_thread distance_calculator_thread;

K_THREAD_STACK_DEFINE(thread_stack, STACKSIZE);

struct coordinates {
	void* fifo_reserved;
	uint8_t sys_id;
	float latitude;
	float longitude;
	float altitude;
	float uwb_distance;
};

K_FIFO_DEFINE(coords_fifo);

void queue_coords(uint8_t sys_id, float latitude, float longitude, float altitude, float uwb_distance=0.0f) {
	size_t size = sizeof(struct coordinates);
        void* mem_ptr = k_malloc(size);
	struct coordinates* coords = reinterpret_cast<struct coordinates*>(mem_ptr);
	coords->sys_id = sys_id;
	coords->latitude = latitude;
	coords->longitude = longitude;
	coords->altitude = altitude;
	coords->uwb_distance = uwb_distance;
	k_fifo_put(&coords_fifo, mem_ptr);
}

const uint8_t named_value_name_length = 10;

struct named_values {
	void* fifo_reserved;
	char name[named_value_name_length];
	int value;
};

K_FIFO_DEFINE(named_values_fifo);

void queue_named_value(const char* name, int value) {
	int len = strlen(name);
	if (len > named_value_name_length) len = 10;
	size_t size = sizeof(struct named_values);
        void* mem_ptr = k_malloc(size);
	struct named_values* named_value = reinterpret_cast<struct named_values*>(mem_ptr);
	memcpy(named_value->name, name, len);
	if (len < named_value_name_length) named_value->name[len] = '\0';
	named_value->value = value;
	k_fifo_put(&named_values_fifo, mem_ptr);
}

/*
 * Math helper functions
 */
#define M_PI      3.14159265358979323846

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

/*
 * UART communication
 */

const uint8_t SYS_IDS = 2;
static uint8_t OUR_ID = 0;


/*
 * MAVLink handlers
 */
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
	if (OUR_ID == 0 && sys_id <= SYS_IDS) {
		// now we know who we are
		OUR_ID = sys_id;
		mac_addr[1] = sys_id;
	}
	queue_coords(sys_id,
		gps_raw_int.lat / 1e7,
		gps_raw_int.lon / 1e7,
		gps_raw_int.alt / 1e3
	);
}

mavlink_status_t mavlink_status;
mavlink_message_t mavlink_msg;


#define UART_BUF_MAXSIZE 279
static uint8_t uart_tx_buf[UART_BUF_MAXSIZE];
static uint8_t* uart_tx_ptr = uart_tx_buf;
static uint16_t uart_tx_ctr = 0;
static const struct device* uart_dev;

static void uart_isr(const struct device* dev, void* userdata) {
	uint8_t c;
	
	while (uart_irq_update(dev)
		&& uart_irq_is_pending(dev)
	) {
        	if (uart_irq_rx_ready(dev)) {
                        if (uart_fifo_read(dev, &c, 1) == 0) {
                                break;   
                        }
			if (mavlink_parse_char(MAVLINK_MAIN_CHANNEL, c, &mavlink_msg, &mavlink_status)) {
				handle_message(mavlink_msg);
			}
		} else if (uart_irq_tx_ready(dev)) {
			if (uart_tx_ctr > 0) {
				int n = uart_fifo_fill(dev, uart_tx_ptr, uart_tx_ctr);
				uart_tx_ptr += n;
				uart_tx_ctr -= n;
			} else {
				uart_tx_ptr = uart_tx_buf;
				uart_irq_tx_disable(uart_dev);
			}
                } else {
			break;
		}
        }
}

static void uart_init(void)
{
	uart_dev = device_get_binding("UART_3");
	struct uart_config cfg;
	uart_config_get(uart_dev, &cfg);
	cfg.baudrate = 115200;
	uart_configure(uart_dev, &cfg);

	uart_irq_rx_disable(uart_dev);
	uart_irq_tx_disable(uart_dev);
	uart_irq_callback_set(uart_dev, uart_isr);
	uart_irq_rx_enable(uart_dev);
}

void mavlink_send_uart_bytes(const uint8_t *ch, int length)
{
	while(uart_tx_ctr > 0) {
		// another message being processed
		// let's wait for 5 milliseconds as sending a DISTANCE_SENSOR message at 115200 baudrate
		// should take around 4.43 milliseconds
		// TODO: just append the new data as long as it fits into the buffer
		k_msleep(5);
	}
	memcpy(uart_tx_buf, ch, length);
	uart_tx_ptr = uart_tx_buf;
	uart_tx_ctr = length;
	uart_irq_tx_enable(uart_dev);
}

void send_named_value_int(const char* name, int32_t value) {
	uint8_t send_buf[30];
	mavlink_message_t mav_msg;
	mavlink_named_value_int_t msg = {};
	msg.time_boot_ms = k_uptime_delta(&time_stamp);
	int len = strlen(name);
	if (len > 10) len = 10;
	memcpy(msg.name, name, len);
	msg.value = value;
	mavlink_msg_named_value_int_encode(OUR_ID, UWB_COMPONENT_ID, &mav_msg, &msg);
	int n = mavlink_msg_to_send_buffer(send_buf, &mav_msg);
	mavlink_send_uart_bytes(send_buf, n);
}

void send_named_value_float(const char* name, float value) {
	uint8_t send_buf[30];
	mavlink_message_t mav_msg;
	mavlink_named_value_float_t msg = {};
	msg.time_boot_ms = k_uptime_delta(&time_stamp);
	int len = strlen(name);
	if (len > 10) len = 10;
	memcpy(msg.name, name, len);
	msg.value = value;
	mavlink_msg_named_value_float_encode(OUR_ID, UWB_COMPONENT_ID, &mav_msg, &msg);
	int n = mavlink_msg_to_send_buffer(send_buf, &mav_msg);
	mavlink_send_uart_bytes(send_buf, n);
}

/*
 * Pozyx device
 */

namespace PozyxConstants {
	enum class Status : uint8_t {
		Failure = 0x00,
		Success = 0x01,
	};

	const uint16_t I2C_ADDRESS = 0x4B;

	enum class RangeProtocol : uint8_t {
		Precision = 0x00,
		Fast = 0x01
	};
}

namespace PozyxRegisters {
	const uint8_t WHO_AM_I = 0x00;
	const uint8_t FIRMWARE_VERSION = 0x01;
	const uint8_t HARDWARE_VERSION = 0x02;
	const uint8_t NETWORK_ID = 0x1A;
	const uint8_t RANGE_PROTOCOL = 0x21;

	const uint8_t LED_CONTROL = 0xB1;
	const uint8_t TX_DATA = 0xB2;
	const uint8_t TX_SEND = 0xB3;
	const uint8_t RX_DATA = 0xB4;
	const uint8_t DO_RANGING = 0xB5;

	const uint8_t DEVICE_GETRANGEINFO = 0xC7;
}

namespace PozyxParams {
	struct RxData {
		uint8_t offset;
		uint8_t length;
	};
}

class PozyxDev {
public:
	PozyxDev(const struct device *i2c_dev) :
		dev(i2c_dev),
		i2c_addr(PozyxConstants::I2C_ADDRESS)
	{
		set_range_protocol(PozyxConstants::RangeProtocol::Fast);
	}

	uint8_t who_am_i() {
		return read_byte(PozyxRegisters::WHO_AM_I);
	}

	uint8_t firmware_version() {
		return read_byte(PozyxRegisters::FIRMWARE_VERSION);
	}
	
	uint8_t hardware_version() {
		return read_byte(PozyxRegisters::HARDWARE_VERSION);
	}

	uint16_t network_id() {
		uint16_t net_id;
		read_bytes(PozyxRegisters::NETWORK_ID, reinterpret_cast<uint8_t*>(&net_id), 2);
		return net_id;
	}

	void set_network_id(uint16_t net_id) {
		write_bytes(PozyxRegisters::NETWORK_ID, reinterpret_cast<uint8_t*>(&net_id), 2);
	}


	PozyxConstants::RangeProtocol range_protocol() {
		return PozyxConstants::RangeProtocol(read_byte(PozyxRegisters::RANGE_PROTOCOL));
	}

	void set_range_protocol(PozyxConstants::RangeProtocol protocol) {
		write_byte(PozyxRegisters::RANGE_PROTOCOL, static_cast<uint8_t>(protocol));
	}


	PozyxConstants::Status led_control() {
		uint8_t bitflag = 0xAF;
		uint8_t led_status;
		PozyxConstants::Status status = register_function(PozyxRegisters::LED_CONTROL,
			&bitflag,
			1,
			&led_status,
			1);
		return status;
	}

	PozyxConstants::Status rx_data(uint8_t* result, uint32_t result_bytes) {
		PozyxParams::RxData params = {0x00, static_cast<uint8_t>(result_bytes)};
		return register_function(PozyxRegisters::RX_DATA,
			reinterpret_cast<uint8_t*>(&params),
			sizeof(params),
			result,
			result_bytes);
	}

	PozyxConstants::Status range(uint16_t net_id, uint32_t& timestamp, uint32_t& range, int16_t& signal_strength) {
		uint8_t buf[10];
		PozyxConstants::Status status = register_function(PozyxRegisters::DEVICE_GETRANGEINFO,
			reinterpret_cast<uint8_t*>(&net_id),
			2,
			buf,
			10);
		timestamp = *reinterpret_cast<uint32_t*>(&buf[0]);
		range = *reinterpret_cast<uint32_t*>(&buf[4]);
		signal_strength = buf[8] << 8 + buf[9];
		return status;
	}

protected:
	const struct device *dev;
	uint16_t i2c_addr;

	int write_byte(uint8_t register_addr, uint8_t value) {
		return write_bytes(register_addr, &value, 1U);
	}

	int write_bytes(uint8_t register_addr, uint8_t *data, uint32_t num_bytes) {
		struct i2c_msg msgs[2];
		msgs[0].buf = &register_addr;
		msgs[0].len = 1U;
		msgs[0].flags = I2C_MSG_WRITE;

		msgs[1].buf = data;
		msgs[1].len = num_bytes;
		msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
  
        	return i2c_transfer(dev, msgs, 2, i2c_addr);
	}


	uint8_t read_byte(uint8_t register_addr) {
		uint8_t data;
		i2c_reg_read_byte(dev, i2c_addr, register_addr, &data);
		return data;
	}

	int read_bytes(uint8_t register_addr, uint8_t *data, uint32_t num_bytes) {
		struct i2c_msg msgs[2];

		msgs[0].buf = &register_addr;
		msgs[0].len = 1U;
		msgs[0].flags = I2C_MSG_WRITE;

		msgs[1].buf = data;
		msgs[1].len = num_bytes;
		msgs[1].flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP;

		return i2c_transfer(dev, msgs, 2, i2c_addr);
	}
	
	PozyxConstants::Status register_function(uint8_t register_addr, uint8_t *params, uint32_t param_bytes, uint8_t *result, uint32_t result_bytes) {
		uint8_t buf[100];
		uint8_t status = static_cast<uint8_t>(PozyxConstants::Status::Failure);
		struct i2c_msg msgs[3];

		msgs[0].buf = &register_addr;
		msgs[0].len = 1U;
		msgs[0].flags = I2C_MSG_WRITE;

		msgs[1].buf = params;
		msgs[1].len = param_bytes;
		msgs[1].flags = I2C_MSG_WRITE;
		//msgs[2].buf = &status;
		//msgs[2].len = 1U;
		//msgs[2].flags = I2C_MSG_READ;

		msgs[2].buf = buf;
		msgs[2].len = result_bytes + 1;
		msgs[2].flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP;

		i2c_transfer(dev, msgs, 3, i2c_addr);
		memcpy(result, buf+1, result_bytes);
		return PozyxConstants::Status(buf[0]);
	}
};

/* threads
 */
void read_range_thread_entry(void) {
	struct k_timer timer;
	const struct device *i2c_dev;
	i2c_dev = device_get_binding("I2C_1");
	if (!i2c_dev) {
		printk("I2C device driver not found\n");
		return;
	} else {
		printk("I2C device driver found\n");
	}

	PozyxDev pozyx(i2c_dev);

	k_timer_init(&timer, NULL, NULL);
	struct {
		uint32_t latitude;
		uint32_t longitude;
		uint32_t altitude;
	} remote_position;
	while (1) {
		k_timer_start(&timer, K_MSEC(500), K_NO_WAIT);

		uint32_t timestamp;
		uint32_t range;
		int16_t signal_strength;
		if (pozyx.range(0x6778, timestamp, range, signal_strength) == PozyxConstants::Status::Success) {
			float scaled_range = range;
			if (pozyx.rx_data(reinterpret_cast<uint8_t*>(&remote_position), sizeof(remote_position)) == PozyxConstants::Status::Success) {
#ifdef MAVLINK_DEBUG_MODE
				queue_named_value("range", range);
				queue_named_value("pos1", remote_position.latitude);
				queue_named_value("pos2", remote_position.longitude);
				queue_named_value("pos3", remote_position.altitude);
#endif
				if (OUR_ID != 0) {
					uint8_t sys_id = SYS_IDS+1 - OUR_ID;
					queue_coords(sys_id,
						remote_position.latitude / 1e7,
						remote_position.longitude / 1e7,
						remote_position.altitude / 1e3,
						range / 1e3
					);
				}
			}
		}
#ifdef MAVLINK_DEBUG_MODE
		queue_named_value("netid", pozyx.network_id());
		queue_named_value("rngproto", static_cast<uint8_t>(pozyx.range_protocol()));
		queue_named_value("whoami", pozyx.who_am_i());
		queue_named_value("firmware", pozyx.firmware_version());
		queue_named_value("hardware", pozyx.hardware_version());
#endif
		k_timer_status_sync(&timer);
	}
}

void distance_calculator_thread_entry(void) {
	const uint8_t THEIR_ID = 2;
	// Coordinates of the Elbphilharmonie in Hamburg
	struct coordinates coords[SYS_IDS+1];
	coords[THEIR_ID].sys_id = 2;
	coords[THEIR_ID].latitude = 53.541350;
	coords[THEIR_ID].longitude = 9.985102;
	coords[THEIR_ID].altitude = 10.0;
	struct k_timer timer;

	k_timer_init(&timer, NULL, NULL);

	struct coordinates* new_coords;
	struct named_values* new_named_value;
	while (1) {
		bool new_data = false;
		// we publish distances twice a second
		k_timer_start(&timer, K_MSEC(500), K_NO_WAIT);
		while((new_named_value = reinterpret_cast<struct named_values*>(k_fifo_get(&named_values_fifo, K_NO_WAIT)))) {
			send_named_value_int(new_named_value->name, new_named_value->value);
			k_free(new_named_value);
		}

		while((new_coords = reinterpret_cast<struct coordinates*>(k_fifo_get(&coords_fifo, K_NO_WAIT)))) {
			if(new_coords->sys_id <= SYS_IDS) {
				memcpy(&coords[new_coords->sys_id], new_coords, sizeof(struct coordinates));
				new_data = true;
			}
			k_free(new_coords);
		}
		if (
			!new_data // no new data, so nothing to calculate
			|| (OUR_ID == 0) // we do not know our identity yet, waiting to receive GPS_RAW_INT with a SYS_ID != 0
		) {
			continue;
		}
		uint8_t send_buf[51]; // DISTANCE_SENSOR has at most 51 bytes, where as MAVLink 2.0 messages can be as large as 279 bytes
		mavlink_message_t mav_msg;
		mavlink_distance_sensor_t msg = {};
		for (int i=1; i<=SYS_IDS; ++i) {
			if (i==OUR_ID) {
				// send_named_value_int("mac_l", reinterpret_cast<int32_t*>(mac_addr)[0]);
				// send_named_value_int("mac_h", reinterpret_cast<int32_t*>(mac_addr)[1]);
				continue; // do not calculate distance to us
			}
			// send_named_value_int("peer", i);
			msg.time_boot_ms = k_uptime_delta(&time_stamp);
			msg.id = i;
			msg.type = MAV_DISTANCE_SENSOR_ULTRASOUND; // TODO: suggest additional types in github.com/mavlink/mavlink
			msg.orientation = MAV_SENSOR_ROTATION_CUSTOM;
			msg.min_distance = 1; // in cm; to be determined
			msg.max_distance = calculate_distance(coords[OUR_ID], coords[i]) * 100; // in cm; using gps distance here
			msg.current_distance = coords[i].uwb_distance * 100;
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
		k_timer_status_sync(&timer);
	}
}

static uint8_t *get_mac(const struct device *dev)
{                                                        
        mac_addr[7] = 0x00;
        mac_addr[6] = 0x12;
        mac_addr[5] = 0x4b;
                                  
        mac_addr[4] = 0x00;                                    
	mac_addr[3] = 0x00;
	mac_addr[2] = 0x00;
	mac_addr[1] = 0x00;
	mac_addr[0] = 0x02;
                
        return mac_addr;
}

void* dw1000_dev_state;

#ifdef CONFIG_IEEE802154_RAW_MODE
int net_recv_data(struct net_if *iface, struct net_pkt *pkt)
{
}
#endif

static bool init_ieee802154(void)
{
        // ieee802154_dev = device_get_binding(CONFIG_NET_CONFIG_IEEE802154_DEV_NAME);
	// dw1000_dev_state = ieee802154_dev->state;
	// printk("ptr: %x\n", ieee802154_dev->state);
	// radio_api = (struct ieee802154_radio_api *)ieee802154_dev->api;
	// get_mac(ieee802154_dev);
}

void main(void)
{
	time_stamp = k_uptime_get();
	uart_init();
	// init_ieee802154();

	printk("Creating threads\n");
	/*
	k_thread_create(&read_range_thread, thread_stack, STACKSIZE,
			(k_thread_entry_t) read_range_thread_entry,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
	*/

	k_thread_create(&distance_calculator_thread, thread_stack, STACKSIZE,
			(k_thread_entry_t) distance_calculator_thread_entry,
			NULL, NULL, NULL, K_PRIO_COOP(8), 0, K_NO_WAIT);

	read_range_thread_entry();

	/*
	struct k_timer timer;

	k_timer_init(&timer, NULL, NULL);
	while (1) {
		// doing nothing here
		k_timer_start(&timer, K_MSEC(1000), K_NO_WAIT);
		k_timer_status_sync(&timer);
	}*/
}
