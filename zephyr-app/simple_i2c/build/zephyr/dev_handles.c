#include <device.h>
#include <toolchain.h>

/* 1 : /soc/rcc@40023800:
 * - (/soc)
 * - (/clocks/pll)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_rcc_40023800[] = { DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 2 : /soc/interrupt-controller@40013c00:
 * - (/soc)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_interrupt_controller_40013c00[] = { DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 3 : /soc/serial@40004800:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 * - /soc/rcc@40023800
 * - (/soc/pin-controller@40020000/usart3_rx_pc11)
 * - (/soc/pin-controller@40020000/usart3_tx_pc10)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_serial_40004800[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 4 : /soc/serial@40004400:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 * - /soc/rcc@40023800
 * - (/soc/pin-controller@40020000/usart2_rx_pa3)
 * - (/soc/pin-controller@40020000/usart2_tx_pa2)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_serial_40004400[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 5 : /soc/serial@40011000:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 * - /soc/rcc@40023800
 * - (/soc/pin-controller@40020000/usart1_rx_pa10)
 * - (/soc/pin-controller@40020000/usart1_tx_pa9)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_serial_40011000[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 6 : sysinit:
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_sys_init_sys_clock_driver_init0[] = { DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 7 : /soc/pin-controller@40020000/gpio@40021c00:
 * - (/soc/pin-controller@40020000)
 * - /soc/rcc@40023800
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_40020000_S_gpio_40021c00[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 8 : /soc/pin-controller@40020000/gpio@40021000:
 * - (/soc/pin-controller@40020000)
 * - /soc/rcc@40023800
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_40020000_S_gpio_40021000[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 9 : /soc/pin-controller@40020000/gpio@40020c00:
 * - (/soc/pin-controller@40020000)
 * - /soc/rcc@40023800
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_40020000_S_gpio_40020c00[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 10 : /soc/pin-controller@40020000/gpio@40020800:
 * - (/soc/pin-controller@40020000)
 * - /soc/rcc@40023800
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_40020000_S_gpio_40020800[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 11 : /soc/pin-controller@40020000/gpio@40020400:
 * - (/soc/pin-controller@40020000)
 * - /soc/rcc@40023800
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_40020000_S_gpio_40020400[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 12 : /soc/pin-controller@40020000/gpio@40020000:
 * - (/soc/pin-controller@40020000)
 * - /soc/rcc@40023800
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_pin_controller_40020000_S_gpio_40020000[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 13 : /soc/i2c@40005c00:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 * - /soc/rcc@40023800
 * - (/soc/pin-controller@40020000/i2c3_scl_pa8)
 * - (/soc/pin-controller@40020000/i2c3_sda_pb4)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_i2c_40005c00[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 14 : /soc/i2c@40005800:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 * - /soc/rcc@40023800
 * - (/soc/pin-controller@40020000/i2c2_scl_pb10)
 * - (/soc/pin-controller@40020000/i2c2_sda_pb3)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_i2c_40005800[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 15 : /soc/i2c@40005400:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 * - /soc/rcc@40023800
 * - (/soc/pin-controller@40020000/i2c1_scl_pb8)
 * - (/soc/pin-controller@40020000/i2c1_sda_pb9)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_i2c_40005400[] = { 1, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };

/* 16 : /soc/spi@40013000:
 * - (/soc)
 * - (/soc/interrupt-controller@e000e100)
 * - /soc/rcc@40023800
 * - /soc/pin-controller@40020000/gpio@40020400
 * - (/soc/pin-controller@40020000/spi1_miso_pa6)
 * - (/soc/pin-controller@40020000/spi1_mosi_pa7)
 * - (/soc/pin-controller@40020000/spi1_nss_pa4)
 * - (/soc/pin-controller@40020000/spi1_sck_pa5)
 */
const device_handle_t __aligned(2) __attribute__((__section__(".__device_handles_pass2")))
__devicehdl_DT_N_S_soc_S_spi_40013000[] = { 1, 11, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS, DEVICE_HANDLE_ENDS };
