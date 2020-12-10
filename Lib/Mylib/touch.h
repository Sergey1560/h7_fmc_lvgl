#ifndef TOUCH_INIT_H
#define TOUCH_INIT_H
#include "common_defs.h"
#include "lvgl.h"
/* 
XPT2046
Подключение

SPI2
PB10 - SCK
PB14 - MISO
PB15 - MOSI
PB11 - CS

PB1 - Irq


*/
#define TOUCH_COUNT 4

#define	GET_X 	0x90       
#define	GET_Y 	0xD0       

#define ADC_VALID_OFFSET	10

#define XPT2046_X_MIN       201 
#define XPT2046_Y_MIN       164
#define XPT2046_X_MAX       3919
#define XPT2046_Y_MAX       3776

#define XPT2046_X_INV       1

#define XPT2046_HOR_RES     800
#define XPT2046_VER_RES     480

#define Touch_GPIO_SPEED  S_VH

#define Touch_CS_Hi() GPIOB->BSRR = GPIO_BSRR_BS_11
#define Touch_CS_Low() GPIOB->BSRR = GPIO_BSRR_BR_11

//Touch screen values

void TouchInit(void);

void spi2_init(void);
uint8_t spi_transfer(uint8_t data);

bool touch_update(lv_coord_t *x, lv_coord_t*y);

void xpt2046_corr(uint16_t * x, uint16_t * y);
uint8_t	xpt2046_data_valid(uint16_t raw_x, uint16_t raw_y);
uint16_t xpt2046_cmd_get(uint8_t cmd);
void xpt2046_read(uint16_t *x,uint16_t *y);
bool xpt2046_get_data(lv_coord_t *x, lv_coord_t*y);

#endif
