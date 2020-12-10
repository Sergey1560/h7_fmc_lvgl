#include "main.h"

int main(void){
	RCC_init();
	MPU_Config();
	dwt_init();
	Delay_Init();

	SysTick_Config(SystemCoreClock/1000);

	lcd_init();
		
    while(1){
		if(flag_to_update_lcd){
			lv_task_handler();
		}

	
	};
}


void SysTick_Handler(void){
	lv_tick_inc(1);
}