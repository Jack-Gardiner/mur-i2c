/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Sweep through all 7-bit I2C addresses, to see if any slaves are present on
// the I2C bus. Print out a table that looks like this:
//
// I2C Bus Scan
//    0 1 2 3 4 5 6 7 8 9 A B C D E F
// 00 . . . . . . . . . . . . . . . .
// 10 . . @ . . . . . . . . . . . . .
// 20 . . . . . . . . . . . . . . . .
// 30 . . . . @ . . . . . . . . . . .
// 40 . . . . . . . . . . . . . . . .
// 50 . . . . . . . . . . . . . . . .
// 60 . . . . . . . . . . . . . . . .
// 70 . . . . . . . . . . . . . . . .
// E.g. if addresses 0x12 and 0x34 were acknowledged.

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
// Pico W devices use a GPIO on the WIFI chip for the LED,
// so when building for Pico W, CYW43_WL_GPIO_LED_PIN will be defined
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#ifndef LED_DELAY_MS
#define LED_DELAY_MS 500
#endif

// Perform initialisation
int pico_led_init(void) {
    #if defined(PICO_DEFAULT_LED_PIN)
        // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
        // so we can use normal GPIO functionality to turn the led on and off
        gpio_init(PICO_DEFAULT_LED_PIN);
        gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
        return PICO_OK;
    #elif defined(CYW43_WL_GPIO_LED_PIN)
        // For Pico W devices we need to initialise the driver etc
        return cyw43_arch_init();
    #endif
}
// Turn the led on or off
void pico_set_led(bool led_on) {
    #if defined(PICO_DEFAULT_LED_PIN)
        // Just set the GPIO on or off
        gpio_put(PICO_DEFAULT_LED_PIN, led_on);
    #elif defined(CYW43_WL_GPIO_LED_PIN)
        // Ask the wifi "driver" to set the GPIO on or off
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
    #endif
 }    
// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
    return false;
    //return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}
int main() {
    // Set the GPIO pin mux to the UART - pin 0 is TX, 1 is RX; note use of UART_FUNCSEL_NUM for the general
    // case where the func sel used for UART depends on the pin number
    // Do this before calling uart_init to avoid losing data
    gpio_set_function(0, UART_FUNCSEL_NUM(uart0, 0));
    gpio_set_function(1, UART_FUNCSEL_NUM(uart0, 1));

    // Initialise UART 0
    uart_init(uart0, 115200);

    uart_puts(uart0, "\nHello world!\n");
    i2c_init(i2c_default, 100 * 1000);

    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
    int target_address = 64;
    int led_add = 0;
    int deb = 1;
    for (int addr = 0; addr < 256; ++addr) {
        //if (addr % 16 == 0) {
            //printf("%02x ", addr);
        //}

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.
        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        /*
        if (reserved_addr(addr)){
            ret = PICO_ERROR_GENERIC;
            deb = 2;
        }else{
            ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);
            if (ret >= -1){
                led_add = 10;
                deb = 4;
            }
        }
        */
        ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);
        sleep_ms(100);
        uart_puts(uart0,ret < 0 ? "NO DICE\n" : "DICE!!\n");
        //uart_puts(uart0,addr % 16 == 15 ? "\n" : "  ");
    }
    if (reserved_addr(5)){
        deb = 3;
    }
    // Enable UART so we can print status output
    //stdio_init_all();
    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);
    int i = 0;

    pico_set_led(false);
    sleep_ms(1000);
    while (i < deb) {
        pico_set_led(true);
        sleep_ms(LED_DELAY_MS);
        pico_set_led(false);
        sleep_ms(LED_DELAY_MS);
        i += 1;

        uart_puts(uart0, "\n PING");
    }
}
