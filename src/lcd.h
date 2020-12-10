#ifndef LCD_H
#define LCD_H
#include "common_defs.h"
#include "ssd1963.h"
#include "touch.h"

#include <stdio.h>

#include "lvgl.h"
//#include "lv_examples.h"

#define LVGL_BUF_SIZE (LV_HOR_RES_MAX*300)
#define FB_OUT_DMA

struct touch_data{
    uint16_t x;
    uint16_t y;
    uint16_t z;
};

extern volatile uint8_t flag_to_update_lcd;

void lcd_init(void);
void lcd_disp_flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_p);
void timer2_init(void);
void timer3_init(void);
bool lcd_touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data);

void gauge_formatter_cb(lv_obj_t *gauge, char *buf, int bufsize, int32_t value);

#endif