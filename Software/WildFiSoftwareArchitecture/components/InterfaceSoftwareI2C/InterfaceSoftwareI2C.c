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

#include "InterfaceSoftwareI2C.h"

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>

RTC_DATA_ATTR static bool g_i2c_started;
RTC_DATA_ATTR static uint8_t g_i2c_sda;
RTC_DATA_ATTR static uint8_t g_i2c_scl;
RTC_DATA_ATTR bool last_ack;
RTC_DATA_ATTR uint16_t readQuantitySw;
RTC_DATA_ATTR uint16_t bufferPointerSw;

/* https://esp-idf.readthedocs.io/en/latest/api-reference/peripherals/i2c.html#_CPPv211i2c_set_pin10i2c_port_tii13gpio_pullup_t13gpio_pullup_t10i2c_mode_t */

RTC_IRAM_ATTR void beginSw() {
    sw_i2c_init(21, 22);
}

RTC_IRAM_ATTR void beginTransmissionSw(uint8_t address) {
	sw_i2c_master_start();
    last_ack = sw_i2c_master_write_byte((address << 1) | I2C_MASTER_WRITE); // ack needs to be stored because is directly executed
}

RTC_IRAM_ATTR void writeSw(uint8_t data) {
	sw_i2c_master_write_byte(data); // send 1 byte
}

RTC_IRAM_ATTR uint8_t endTransmissionSw() {
	sw_i2c_master_stop(); // stop sequence
	return last_ack;
}

RTC_IRAM_ATTR void requestFromSw(uint8_t address, uint16_t quantity, uint8_t *data) {
	sw_i2c_master_start();
	last_ack = sw_i2c_master_write_byte((address << 1) | I2C_MASTER_READ); // send 1 byte with slave address
	readQuantitySw = quantity;

    if(readQuantitySw > 1) {
        sw_i2c_master_read(data, readQuantitySw - 1, ACK);
    }
    sw_i2c_master_read_byte(&data[readQuantitySw - 1], NAK);
	endTransmissionSw();
}

RTC_IRAM_ATTR void gpio_set_level_ram(gpio_num_t gpio_num, uint32_t level) {
	//gpio_set_level(gpio_num, level);
    if(level == HIGH) {
        GPIO_OUTPUT_SET(gpio_num, 1);
    }
    else {
        GPIO_OUTPUT_SET(gpio_num, 0);
    }
}

RTC_IRAM_ATTR int gpio_get_level_ram(gpio_num_t gpio_num) {
	//return gpio_get_level(gpio_num);
    return GPIO_INPUT_GET(gpio_num);
}

/* esp_err_t i2c_set_pin(i2c_port_t i2c_num, int sda_io_num, int scl_io_num, gpio_pullup_t sda_pullup_en, gpio_pullup_t scl_pullup_en, i2c_mode_t mode) */
RTC_IRAM_ATTR esp_err_t sw_i2c_init(uint8_t sda, uint8_t scl) {
    //gpio_set_direction(sda, GPIO_MODE_INPUT_OUTPUT_OD);
    //gpio_pulldown_dis(sda);
    //gpio_pullup_dis(sda);
    PIN_INPUT_ENABLE(GPIO_PIN_REG_21); // enable input
    GPIO.enable_w1ts = (0x1 << 21); // enable output
    GPIO.pin[21].pad_driver = 1; // open drain
    REG_CLR_BIT(GPIO_PIN_REG_21, FUN_PD); // pull down disable
    REG_CLR_BIT(GPIO_PIN_REG_21, FUN_PU); // pull up disable

    //gpio_set_direction(scl, GPIO_MODE_INPUT_OUTPUT_OD);
    //gpio_pulldown_dis(scl);
    //gpio_pullup_dis(scl);
    PIN_INPUT_ENABLE(GPIO_PIN_REG_22); // enable input
    GPIO.enable_w1ts = (0x1 << 22); // enable output
    GPIO.pin[22].pad_driver = 1; // open drain
    REG_CLR_BIT(GPIO_PIN_REG_22, FUN_PD); // pull down disable
    REG_CLR_BIT(GPIO_PIN_REG_22, FUN_PU); // pull up disable

    /* Save the pins in static global variables. */
    g_i2c_sda = sda;
    g_i2c_scl = scl;

    return ESP_OK;
}

/* esp_err_t i2c_master_start(i2c_cmd_handle_t cmd_handle) */
RTC_IRAM_ATTR esp_err_t sw_i2c_master_start() {
    uint32_t stretch = CLOCK_STRETCH_TIMEOUT;

     /* If already started, do a restart condition. */
    if(g_i2c_started) {
        gpio_set_level_ram(g_i2c_sda, HIGH);
        ets_delay_us(CLOCK_SPEED_DELAY);
        gpio_set_level_ram(g_i2c_scl, HIGH);
        while (gpio_get_level_ram(g_i2c_scl) == LOW && stretch--) {
            ets_delay_us(1);
        };
        ets_delay_us(CLOCK_SPEED_DELAY);
    }

    if(LOW == gpio_get_level_ram(g_i2c_sda)) {
        //ESP_LOGD(TAG, "Arbitration lost in sw_i2c_master_start()");
    }

    /* Start bit is indicated by a high-to-low transition of SDA with SCL high. */
    gpio_set_level_ram(g_i2c_sda, LOW);
    ets_delay_us(CLOCK_SPEED_DELAY);
    gpio_set_level_ram(g_i2c_scl, LOW);

    g_i2c_started = true;

    return ESP_OK;
}

/* esp_err_t i2c_master_stop(i2c_cmd_handle_t cmd_handle) */
RTC_IRAM_ATTR void sw_i2c_master_stop() {
    uint32_t stretch = CLOCK_STRETCH_TIMEOUT;

    /* The stop bit is indicated by a low-to-high transition of SDA with SCL high. */
    gpio_set_level_ram(g_i2c_sda, LOW);
    ets_delay_us(CLOCK_SPEED_DELAY);
    gpio_set_level_ram(g_i2c_scl, HIGH);

    while(gpio_get_level_ram(g_i2c_scl) == LOW && stretch--) {
        ets_delay_us(1);
    };

    ets_delay_us(CLOCK_SPEED_DELAY);
    gpio_set_level_ram(g_i2c_sda, HIGH);
    ets_delay_us(CLOCK_SPEED_DELAY);

    if(gpio_get_level_ram(g_i2c_sda) == LOW) {
        //ESP_LOGD(TAG, "Arbitration lost in sw_i2c_master_stop()");
    }

    g_i2c_started = false;
}

RTC_IRAM_ATTR static void sw_i2c_write_bit(bool bit) {
    uint32_t stretch = CLOCK_STRETCH_TIMEOUT;

    gpio_set_level_ram(g_i2c_sda, bit);
    ets_delay_us(CLOCK_SPEED_DELAY); /* SDA change propagation delay */
    gpio_set_level_ram(g_i2c_scl, HIGH); /* New valid SDA value is available. */

    while(gpio_get_level_ram(g_i2c_scl) == LOW && stretch--) {
        ets_delay_us(1);
    };

    ets_delay_us(CLOCK_SPEED_DELAY); /* Wait for SDA value to be read by slave. */

    if(bit && (LOW == gpio_get_level_ram(g_i2c_sda))) {
        //ESP_LOGD(TAG, "Arbitration lost in sw_i2c_write_bit()");
    }

    gpio_set_level_ram(g_i2c_scl, LOW); /* Prepare for next bit. */
}

RTC_IRAM_ATTR static bool sw_i2c_read_bit() {
    uint32_t stretch = CLOCK_STRETCH_TIMEOUT;
    bool bit;

    gpio_set_level_ram(g_i2c_sda, HIGH); /* Let the slave drive data. */
    ets_delay_us(CLOCK_SPEED_DELAY); /* Wait for slave to write. */
    gpio_set_level_ram(g_i2c_scl, HIGH); /* New valid SDA value is available. */

    while(gpio_get_level_ram(g_i2c_scl) == LOW && stretch--) {
        ets_delay_us(1);
    };

    ets_delay_us(CLOCK_SPEED_DELAY); /* Wait for slave to write. */
    bit = gpio_get_level_ram(g_i2c_sda); /* SCL is high, read a bit. */
    gpio_set_level_ram(g_i2c_scl, LOW); /* Prepare for next bit. */

    return bit;
}

RTC_IRAM_ATTR static uint8_t sw_i2c_read_byte(bool ack) {
    uint8_t byte = 0;
    uint8_t bit;

    for(bit = 0; bit < 8; ++bit) {
        byte = (byte << 1) | sw_i2c_read_bit();
    }
    sw_i2c_write_bit(ack);

    return byte;
}

RTC_IRAM_ATTR static bool sw_i2c_write_byte(uint8_t byte) {
    uint8_t bit;
    bool ack;

    for(bit = 0; bit < 8; ++bit) {
        sw_i2c_write_bit((byte & 0x80) != 0);
        byte <<= 1;
    }
    ack = sw_i2c_read_bit();
    return ack;
}

/* esp_err_t i2c_master_write_byte(i2c_cmd_handle_t cmd_handle, uint8_t data, bool ack_en) */
RTC_IRAM_ATTR bool sw_i2c_master_write_byte(uint8_t buffer) {
    return sw_i2c_write_byte(buffer);
    //return ESP_OK;
}

/* esp_err_t i2c_master_write(i2c_cmd_handle_t cmd_handle, uint8_t *data, size_t data_len, bool ack_en) */
RTC_IRAM_ATTR esp_err_t sw_i2c_master_write(uint8_t *buffer, uint8_t length) {
    while(length--) {
        sw_i2c_write_byte(*buffer++);
    }
    return ESP_OK;
}

/* esp_err_t i2c_master_read_byte(i2c_cmd_handle_t cmd_handle, uint8_t *data, i2c_ack_type_t ack) */
RTC_IRAM_ATTR esp_err_t sw_i2c_master_read_byte(uint8_t *buffer, bool ack) {
    *buffer = sw_i2c_read_byte(ack);
    return ESP_OK;
};

/* esp_err_t i2c_master_read(i2c_cmd_handle_t cmd_handle, uint8_t *data, size_t data_len, i2c_ack_type_t ack) */
RTC_IRAM_ATTR esp_err_t sw_i2c_master_read(uint8_t *buffer, uint16_t length, bool ack) {
    while(length) {
        *buffer = sw_i2c_read_byte(ack);
        buffer++;
        length--;
    }
    return ESP_OK;
}

