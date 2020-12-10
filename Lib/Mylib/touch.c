#include "touch.h"

volatile lv_coord_t touch_x;
volatile lv_coord_t touch_y;
volatile uint8_t    touch_updated=0;

void TouchInit(void){
	spi2_init();
};


void spi2_init(void){
/*
SPI2
PB10 - SCK
PB14 - MISO
PB15 - MOSI
PB11 - CS

PB1 - Irq
*/
	RCC->D2CCIP1R |= (SPI_CLK_SRC << RCC_D2CCIP1R_SPI123SEL_Pos);
    RCC->APB1LENR |= RCC_APB1LENR_SPI2EN;

	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;   
	
	//Включение AF5 на пинах 
	GPIOB->AFR[1] |= (5 << GPIO_AFRH_AFRH10_Pos)|\
					 (5 << GPIO_AFRH_AFRH14_Pos)|\
					 (5 << GPIO_AFRH_AFRH15_Pos);

	//Режим альтернативной функции
	GPIOB->MODER &= ~(GPIO_MODER_MODER10|GPIO_MODER_MODER14|GPIO_MODER_MODER15);
	GPIOB->MODER |= GPIO_MODER_MODER10_1|GPIO_MODER_MODER14_1|GPIO_MODER_MODER15_1;
	  	
	//Скорость High speed
	GPIOB->OSPEEDR |= (Touch_GPIO_SPEED << GPIO_OSPEEDER_OSPEEDR10_Pos)|\
					  (Touch_GPIO_SPEED << GPIO_OSPEEDER_OSPEEDR14_Pos)|\
					  (Touch_GPIO_SPEED << GPIO_OSPEEDER_OSPEEDR15_Pos);
	
	//Режим push-pull
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_10|GPIO_OTYPER_OT_14|GPIO_OTYPER_OT_15);
	
	//Отключение подтяжки
	GPIOB->PUPDR &= ~((GPIO_PUPDR_PUPDR10_0|GPIO_PUPDR_PUPDR10_1)|\
					  (GPIO_PUPDR_PUPDR14_0|GPIO_PUPDR_PUPDR14_1)|\
					  (GPIO_PUPDR_PUPDR15_0|GPIO_PUPDR_PUPDR15_1));
	
	//TFT CS PB11
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_11;
	GPIOB->MODER &= ~GPIO_MODER_MODER11;
	GPIOB->MODER |= GPIO_MODER_MODER11_0;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11_1;
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR11;

	Touch_CS_Hi();

	SPI2->CR1 = 0;
	SPI2->CFG1 = (5 << SPI_CFG1_MBR_Pos)|(7 << SPI_CFG1_DSIZE_Pos)|(7 << SPI_CFG1_CRCSIZE_Pos);
	SPI2->CFG2 = SPI_CFG2_SSOE|\
				SPI_CFG2_CPOL|\
				SPI_CFG2_CPHA|\
				SPI_CFG2_AFCNTR|\
				SPI_CFG2_MASTER;


	//Включение SPI
	SPI2->CR1 |= SPI_CR1_SPE;
	
	//TFT Irq

    /****** PB1 Int  */
    //MODE - In
	GPIOB->MODER &= ~(GPIO_MODER_MODER1);
	//Push-pull mode 0
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_1);
	//Very High speed 11
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR1);
    GPIOB->OSPEEDR |= (S_VH << GPIO_OSPEEDER_OSPEEDR1_Pos);
	//Pull UP
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR1;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR1_0;


    RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;  
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PB; 
	EXTI->FTSR1 	|= EXTI_FTSR1_TR1;

	NVIC_EnableIRQ (EXTI1_IRQn);
	NVIC_SetPriority (EXTI1_IRQn, 8);
	//Разрешаем прерывания в периферии
	EXTI_D1->IMR1 |= EXTI_IMR1_IM1;
}

void EXTI1_IRQHandler(void){
	if(EXTI_D1->PR1 & EXTI_PR1_PR1){
		EXTI_D1->PR1 = EXTI_PR1_PR1;
		EXTI_D1->IMR1 &= ~EXTI_IMR1_IM1;

		if(xpt2046_get_data((lv_coord_t *)&touch_x,(lv_coord_t *)&touch_y)){
			touch_updated=1;
		}

		EXTI_D1->IMR1 |= EXTI_IMR1_IM1;
	};
}


bool touch_update(lv_coord_t *x, lv_coord_t*y){

	if(touch_updated){
		*x = touch_x;
		*y = touch_y;
		touch_updated=0;
		return true;
	}else{
		return false;
	}


}


uint8_t spi_transfer(uint8_t data){
    uint8_t *SPI_TXDR;
    uint8_t *SPI_RXDR;
	uint8_t result;

    for(uint32_t i=0; i<0x100; i++){__NOP();};

    SPI_TXDR = (uint8_t *)((uint32_t)SPI2+0x020);
    SPI_RXDR = (uint8_t *)((uint32_t)SPI2+0x030);

   	SPI2->CR2 = 1;
    SPI2->CR1 |= SPI_CR1_CSTART;

	while((SPI2->SR & SPI_SR_TXP) == 0){__NOP();};
	*SPI_TXDR = data;
        
	while((SPI2->SR & SPI_SR_RXP) == 0){__NOP();};
	result=*SPI_RXDR;	

    while((SPI2->SR & SPI_SR_EOT) == 0){__NOP();};
    SPI2->IFCR = SPI_IFCR_EOTC;

	return result;
}


bool xpt2046_get_data(lv_coord_t *x, lv_coord_t*y){
	lv_coord_t x_data = 0;
    lv_coord_t y_data = 0;

	xpt2046_read((uint16_t *)&x_data, (uint16_t *)&y_data);

	if(xpt2046_data_valid(x_data, y_data)){
		xpt2046_corr((uint16_t *)&x_data, (uint16_t *)&y_data);
		*x = x_data;
		*y = y_data;
		DEBUG("Touch: %d %d",x_data,y_data);
		return true;
	}else{
		return false;
	}
};

void xpt2046_read(uint16_t *x,uint16_t *y){
	uint32_t total_x = 0;
	uint32_t total_y = 0;
	uint8_t good_x = 0;
	uint8_t good_y = 0;
	uint16_t val;

	for(uint32_t i=0; i < TOUCH_COUNT; i++){
		val = xpt2046_cmd_get(GET_X);
		if(val > 0){
			total_y += val;
			good_y++;
		}
	
		val = xpt2046_cmd_get(GET_Y);
		if(val > 0){
			total_x += val;
			good_x++;
		}
	}

	if(good_x > 0){
		*x = total_x / good_x;
	}else{
		*x = 0;
	}
	
	if(good_y > 0){
		*y = total_y / good_y;
	}else{
		*y = 0;
	}
}

uint16_t xpt2046_cmd_get(uint8_t cmd){
	uint16_t ret_val;

	Touch_CS_Low();
	spi_transfer(cmd);
	ret_val = spi_transfer(0xFF);
	ret_val = ret_val << 8;
	ret_val |= spi_transfer(0xFF);
	ret_val >>= 3;
	
	Touch_CS_Hi();
	return ret_val & 0x0FFF;
}

uint8_t	xpt2046_data_valid(uint16_t raw_x, uint16_t raw_y){
	if ((   raw_x <= ADC_VALID_OFFSET) 
	    || (raw_y <= ADC_VALID_OFFSET)
		|| (raw_x >= 4095 - ADC_VALID_OFFSET)
		|| (raw_y >= 4095 - ADC_VALID_OFFSET))
	{
		return 0;
	}
	else{
		return 1;
	}
}

void xpt2046_corr(uint16_t * x, uint16_t * y){
	if((*x) > XPT2046_X_MIN)
		(*x) -= XPT2046_X_MIN;    
	else
		(*x) = 0;    
	if((*y) > XPT2046_Y_MIN)
		(*y) -= XPT2046_Y_MIN;    
	else
		(*y) = 0;    
	(*x) = (uint32_t)((uint32_t)(*x) * XPT2046_HOR_RES)/(XPT2046_X_MAX - XPT2046_X_MIN);    
	(*y) = (uint32_t)((uint32_t)(*y) * XPT2046_VER_RES)/(XPT2046_Y_MAX - XPT2046_Y_MIN);
#if XPT2046_X_INV     
	(*x) =  XPT2046_HOR_RES - (*x);
#endif
#if XPT2046_Y_INV     
	(*y) =  XPT2046_VER_RES - (*y);
#endif
}
