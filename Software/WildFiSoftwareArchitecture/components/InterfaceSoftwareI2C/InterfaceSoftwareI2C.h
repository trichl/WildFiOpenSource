/*

Copyright (c) 2018-2019 Mika Tuupola

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/
#ifndef InterfaceSoftwareI2C_H
#define InterfaceSoftwareI2C_H

#define LOW   0x00
#define HIGH  0x01

#define ACK   0x00
#define NAK   0x01

#define CLOCK_STRETCH_TIMEOUT   1000
#define CLOCK_SPEED_DELAY       2      // in us, 10 = 50kHz (DEFAULT), 2 = 250kHz, 1 = 500kHz


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include "freertos/FreeRTOS.h" // for RTC_IRAM_ATTR
#include <driver/i2c.h> // for I2C_MASTER_WRITE
#include <rom/gpio.h> // WARNING: rom/gpio.h marked as deprecated, when using esp32/rom/gpio.h -> BROWNOUTS + HIGHER DEEP SLEEP CURRENT

extern bool last_ack;
extern uint16_t readQuantitySw;

/** Arduino equivalent */
RTC_IRAM_ATTR void beginSw();
RTC_IRAM_ATTR void beginTransmissionSw(uint8_t address);
RTC_IRAM_ATTR void writeSw(uint8_t data);
RTC_IRAM_ATTR uint8_t endTransmissionSw();
RTC_IRAM_ATTR void requestFromSw(uint8_t address, uint16_t quantity, uint8_t *data);
//RTC_IRAM_ATTR uint8_t readSw();

/** ROM located GPIO setter/getter functions */
RTC_IRAM_ATTR void gpio_set_level_ram(gpio_num_t gpio_num, uint32_t level);
RTC_IRAM_ATTR int gpio_get_level_ram(gpio_num_t gpio_num);

/** I2C bit-banging  */
RTC_IRAM_ATTR esp_err_t sw_i2c_init(uint8_t sda, uint8_t scl);
RTC_IRAM_ATTR esp_err_t sw_i2c_master_start();
RTC_IRAM_ATTR void sw_i2c_master_stop();

RTC_IRAM_ATTR bool sw_i2c_master_write_byte(uint8_t buffer);
RTC_IRAM_ATTR esp_err_t sw_i2c_master_write(uint8_t *buffer, uint8_t length);

RTC_IRAM_ATTR esp_err_t sw_i2c_master_read_byte(uint8_t *buffer, bool ack);
RTC_IRAM_ATTR esp_err_t sw_i2c_master_read(uint8_t *buffer, uint16_t length, bool ack);

#ifdef __cplusplus
}
#endif

#endif
