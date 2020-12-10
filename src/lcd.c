#include "lcd.h"
#include "string.h"

static lv_obj_t * label;
static lv_obj_t * btn1;

static lv_obj_t * clock_gauge;
static const lv_color_t needle_colors[] = {LV_COLOR_BLACK,LV_COLOR_GREEN,LV_COLOR_RED};

volatile uint8_t flag_to_update_lcd=0;

static lv_color_t ALGN32 lvgl_buff[LVGL_BUF_SIZE];
static lv_disp_buf_t disp_buf;

volatile uint32_t seconds_counter=0;

static void btn1_event_handler(lv_obj_t * obj, lv_event_t event);

void lcd_init(void){
    lv_disp_drv_t disp_drv;    
    lv_indev_drv_t indev_drv;
	
    SSD1963_Init();
    TouchInit();
	lv_init();

	lv_disp_buf_init(&disp_buf, lvgl_buff, NULL, LVGL_BUF_SIZE);
    lv_disp_drv_init(&disp_drv);            /*Basic initialization*/
    disp_drv.buffer = &disp_buf;            /*Set an initialized buffer*/
    disp_drv.flush_cb = lcd_disp_flush;        /*Set a flush callback to draw to the display*/
	lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/

	lv_indev_drv_init(&indev_drv);             
    indev_drv.type = LV_INDEV_TYPE_POINTER;    
    indev_drv.read_cb = lcd_touchpad_read;     
    lv_indev_drv_register(&indev_drv);         

    /* Кнопка старт-стоп */
    btn1 = lv_btn_create(lv_scr_act(), NULL);
    lv_obj_set_event_cb(btn1, btn1_event_handler);
    lv_obj_align(btn1, NULL, LV_ALIGN_IN_TOP_LEFT, 20, 50);

    label = lv_label_create(btn1, NULL);
    lv_label_set_text(label, "Start");


    /* Циферблат */
    clock_gauge = lv_gauge_create(lv_scr_act(), NULL);
    lv_gauge_set_needle_count(clock_gauge, 3, needle_colors);
    lv_obj_set_size(clock_gauge, 340, 340);
    lv_obj_align(clock_gauge, NULL, LV_ALIGN_CENTER, -0, 0);
	lv_gauge_set_scale(clock_gauge, 360, 61, 13);
	lv_gauge_set_range(clock_gauge,0,60);
    lv_gauge_set_angle_offset(clock_gauge,180);

    lv_gauge_set_formatter_cb(clock_gauge,gauge_formatter_cb);

    timer2_init();
    timer3_init();
}

void lcd_disp_flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_p){
    
     SetWindows(area->x1,area->y1,area->x2+1,area->y2+1);
     LCD_WriteCmd ( 0x2C );   

 #ifndef FB_OUT_DMA
    uint32_t width = (area->x2 - area->x1) + 1;
    uint32_t height = (area->y2 - area->y1) + 1;

    for(uint32_t i=0; i<width*height; i++){
        LCD_WriteData( *(uint16_t*)color_p++);
    }
 #else
     //Цвет 16бит, количество байт x2
     uint32_t total_bytes = 2 *(((area->x2 - area->x1) + 1) * ((area->y2 - area->y1) + 1));
     fmc_dma_copy((uint32_t *)color_p,(uint32_t *)LCD_DATA,total_bytes,1);
 #endif

    flag_to_update_lcd = 0;
    lv_disp_flush_ready((lv_disp_drv_t *)disp);
}

void timer2_init(void){
	RCC->APB1LENR |= RCC_APB1LENR_TIM2EN;
	TIM2->PSC = (uint16_t)(TIM2_CLK/10000)-1;
	TIM2->ARR = 200 - 1; 	
	TIM2->DIER = 0;
	TIM2->EGR |= TIM_EGR_UG;
	__DSB();
	TIM2->SR &= ~(TIM_SR_UIF);
	TIM2->DIER = TIM_DIER_UIE;
	
	TIM2->CR1 |= TIM_CR1_CEN;

	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority (TIM2_IRQn, 5);
}


void timer3_init(void){
	RCC->APB1LENR |= RCC_APB1LENR_TIM3EN;
	__DSB();
    TIM3->PSC = (uint16_t)(TIM3_CLK/10000)-1;
	TIM3->ARR = 5000 - 1; 	
	TIM3->DIER = 0;
	TIM3->EGR |= TIM_EGR_UG;
	__DSB();
	TIM3->SR &= ~(TIM_SR_UIF);
	TIM3->DIER = TIM_DIER_UIE;

	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_SetPriority (TIM3_IRQn, 6);
}

void TIM3_IRQHandler(void){
    uint32_t seconds,minutes,hours;
	
    if(TIM3->SR & TIM_SR_UIF)	TIM3->SR &= ~TIM_SR_UIF; 

    seconds_counter++;
    
    if(seconds_counter > 60){
        seconds = seconds_counter % 60;
    }else{
        seconds = seconds_counter;
    }

    minutes = seconds_counter / 60;
    hours = (minutes / 60) * 5;
        
    if(minutes > 60){
        minutes = minutes % 60;
    }

    if(hours > 12){
        hours = hours % 12;
    }

    lv_gauge_set_value(clock_gauge, 0, hours);
    lv_gauge_set_value(clock_gauge, 1, minutes);
    lv_gauge_set_value(clock_gauge, 2, seconds);
}



void TIM2_IRQHandler(void){
	if(TIM2->SR & TIM_SR_UIF)	TIM2->SR &= ~TIM_SR_UIF; 
    flag_to_update_lcd=1;
}


bool lcd_touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data){
  static lv_coord_t last_x = 0;
  static lv_coord_t last_y = 0;

    /*Save the state and save the pressed coordinate*/
    data->state = touch_update(&last_x,&last_y) ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;

    /*Set the coordinates (if released use the last pressed coordinates)*/
    data->point.x = last_x;
    data->point.y = last_y;

    return false; /*Return `false` because we are not buffering and no more data to read*/
}

static void btn1_event_handler(lv_obj_t * obj, lv_event_t event){
    static uint8_t button_state = 0;
    
    if(event == LV_EVENT_CLICKED) {
        if(button_state){
            lv_label_set_text(label, "Start");
            DEBUG("Stop");
            TIM3->CR1 &= ~TIM_CR1_CEN;
            seconds_counter = 0;
            button_state=0;
        }else{
            lv_label_set_text(label, "Stop");
            DEBUG("Start");
            TIM3->CR1 |= TIM_CR1_CEN;
            seconds_counter = 0;
            button_state=1;
        }
    }
}

void gauge_formatter_cb(lv_obj_t *gauge, char *buf, int bufsize, int32_t value)
{
  snprintf(buf, bufsize, "%ld", value / 5);

}
