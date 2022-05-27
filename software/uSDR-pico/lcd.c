/*
 * lcd.c
 *
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "ss_oled.h"
#include "lcd.h"


// RPI Pico
#define I2C1_SDA	18
#define I2C1_SCL	19
#define PICO_I2C i2c1
#define I2C_SPEED 100 * 1000


#define OLED_WIDTH 128
#define OLED_HEIGHT 64
SSOLED oled ;
static uint8_t ucBuffer[1024];
//picoSSOLED myOled(OLED_128x64, 0x3c, 0, 0, PICO_I2C, I2C1_SDA, I2C1_SCL, I2C_SPEED);

void lcd_init(void) {
	oled.oled_type = OLED_128x64 ;
	oled.oled_addr = 0x3c ;
	oled.oled_flip = (int) 0 ;
    oled.bbi2c.picoI2C = i2c1;
    oled.bbi2c.iSDA = I2C1_SDA ;
    oled.bbi2c.iSCL = I2C1_SCL ;
    
    i2c_init(i2c1, 100*1000);
    sleep_ms(100);
	gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
	gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);
	gpio_pull_up(I2C1_SDA);
	gpio_pull_up(I2C1_SCL);
	sleep_ms(100);

    if(OLED_NOT_FOUND==__oledInit(&oled, (int) 0, (int32_t) I2C_SPEED)) {
    	while(1) {};
    }
    __oledSetBackBuffer(&oled, ucBuffer);
    __oledSetContrast(&oled, (unsigned char)127);
    __oledPower(&oled, 1);
    __oledFill(&oled, 0x00, (int) 0x01);
}

void lcd_clear(void) {
	__oledFill(&oled, 0x00, (int) 0x01);
}


/**
 * moves the cursor to a specific point and enables / disables it
 * TODO
 */
void lcd_curxy(uint8_t x, uint8_t y, bool on) {

}

/**
 * Puts a character at a specific location on the screen
 * 
 */
void lcd_putxy(uint8_t x, uint8_t y, uint8_t c) {

}

void lcd_writexy(uint8_t x, uint8_t y, uint8_t *s) {
	__oledWriteString(&oled, 0, x, y, s, FONT_8x8, (int)0, (int)1);
}

void lcd_test(void) {

}