/*
 * oled.h
 *
 *  Created on: Oct 5, 2026
 *      Author: Shantanu
 */

#ifndef INC_OLED_H_
#define INC_OLED_H_


#include "main.h"

#define OLED_WIDTH   128
#define OLED_HEIGHT  64

void oled_init(I2C_HandleTypeDef *hi2c);
void oled_clear(void);
void oled_flush(void);                                   /* send framebuffer to the display */
void oled_pixel(int x, int y, uint8_t c);                /* c = 1 on, 0 off */
void oled_rect(int x, int y, int w, int h, uint8_t c);   /* filled */
void oled_frame(int x, int y, int w, int h, uint8_t c);  /* outline */
void oled_char(int x, int y, char ch, uint8_t scale, uint8_t c);
void oled_text(int x, int y, const char *s, uint8_t scale, uint8_t c);
void oled_text_inv(int x, int y, const char *s, uint8_t scale);   /* text on a filled box */
int  oled_text_width(const char *s, uint8_t scale);


#endif /* INC_OLED_H_ */
