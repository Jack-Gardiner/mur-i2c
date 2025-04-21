#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
//#include "hardware/uart.h"
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#ifndef LED_DELAY_MS
#define LED_DELAY_MS 500
#endif

// Perform initialisation
int pico_led_init(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
}
// Turn the led on or off
void pico_set_led(bool led_on) {
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
 }    

int main() {
    i2c_init(i2c_default, 100 * 1000); //baudrate of 100k hz
    //setup sda/scl pins
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
    int target_address = 32 + 2; //0100 010
    sleep_ms(10);
    // Write configuration, to make pins 0-7 input, and 10-17 output.

    const uint8_t toWrite[] = {4,0};
    i2c_write_blocking(i2c_default, target_address, (const uint8_t[]){6,255}, 2, false);
    i2c_write_blocking(i2c_default, target_address, (const uint8_t[]){7,0}, 2, false);
    i2c_write_blocking(i2c_default, target_address, (const uint8_t[]){3,1 + 2}, 2, false);
    int i = 0;
    while(true){
        sleep_ms(1000);
        i2c_write_blocking(i2c_default, target_address, (const uint8_t[]){3,i}, 2, false);
        i = (i+1)%8;
    }
    sleep_ms(100);
    // request read:
    const uint8_t reg_address = 0;
    int success_write = i2c_write_blocking(i2c_default, target_address, &reg_address, 1, false);
    
    sleep_ms(10);
    uint8_t out_data = 0;
    int ret1 = i2c_read_blocking(i2c_default, target_address, &out_data, 1, false);

    sleep_ms(10);

    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);
    // flash a little bit
    i = 0;
   
    while (i < out_data+2) {
        pico_set_led(true);
        sleep_ms(LED_DELAY_MS);
        pico_set_led(false);
        sleep_ms(LED_DELAY_MS);
        i += 1;
    }
}
