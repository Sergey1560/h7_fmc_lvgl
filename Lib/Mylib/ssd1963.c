#include "ssd1963.h"

volatile uint16_t RAM_D1 ALGN4 fill_color;

void fmc_dma_copy(uint32_t *src, uint32_t *dst, uint32_t len, uint8_t incr){
  static uint8_t first_run=1;

  if(first_run > 0){
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    first_run=0; 
  }
  
  uint32_t len32=len/4; // Копирование по 32 бита

	DMA2_Stream1->CR = 0; //Stop DMA
    
    /*
    Копирование память-память. В адресе переферии источник, в адресе памяти назначение.
    Адрес переферии увеличивать, адрес памяти нет.
    Копирование по 32 бита
    */

    if(incr){
            DMA2_Stream1->CR = ((2 << DMA_SxCR_DIR_Pos)| \
                                    DMA_SxCR_PINC|\
                                    DMA_SxCR_MSIZE_1|\
                                    DMA_SxCR_PSIZE_1);
    }else{
            DMA2_Stream1->CR = ((2 << DMA_SxCR_DIR_Pos)| \
                                    DMA_SxCR_MSIZE_1|\
                                    DMA_SxCR_PSIZE_1);
    }

	while(len32){
		if(len32 > 0xFFFC){
			DMA2_Stream1->NDTR = (uint16_t)0xFFFC;
			len32 = len32 - 0xFFFC;
		}else{
			DMA2_Stream1->NDTR = (uint16_t)len32;
			len32=0;
		};
		DMA2_Stream1->PAR = (uint32_t)src;
		DMA2_Stream1->M0AR = (uint32_t)dst;
        #ifdef ENABLE_DCACHE 
		SCB_CleanDCache();
		#endif
		DMA2_Stream1->CR |=  DMA_SxCR_EN;
        #ifdef ENABLE_DCACHE 
		SCB_InvalidateDCache();
		#endif
		
    while(!(DMA2 -> LISR & (DMA_LISR_TCIF1))){asm volatile("nop");};
	if(incr){
        src+=0xFFFC;
    }
	DMA2->LIFCR = DMA_CLEAR;
	}

};


void LCD_fill_area(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color, uint8_t fill_type){
    uint32_t dx,dy;
    
    SetWindows(x1,y1,x2,y2);
    LCD_WriteCmd ( 0x2C );          // SSD1963_WRITE_MEMORY_START

    if(x2 > x1){
        dx=x2-x1;
    }else{
        dx=x1-x2;
    }

    if(y2 > y1){
        dy=y2-y1;
    }else{
        dy=y1-y2;
    }

    if(fill_type == FILL_BY_DMA){
        fill_color = color; //выровненная переменная, для DMA
        fmc_dma_copy((uint32_t *)&fill_color,(uint32_t *)LCD_DATA,dx*dy*2,0);
    }else{
        for(uint32_t i=0; i < dx*dy; i++){
            LCD_WriteData(color);
        } 
    }
};

void SetWindows(uint16_t StartX,uint16_t StartY,uint16_t EndX,uint16_t EndY)
{
	uint16_t hi, lo;

	// set_column_address
	LCD_WriteCmd ( 0x2A );							// SSD1963_SET_COLUMN_ADDRESS
	
	hi = StartX >> 8;
	lo = StartX & 0x00ff;
	LCD_WriteData ( hi );					// Hi
	LCD_WriteData ( lo );					// Low
	
	hi = (EndX-1) >> 8;
	lo = (EndX-1) & 0x00ff;
	LCD_WriteData ( hi );					// Hi
	LCD_WriteData ( lo );					// Low

	// set_page_addres
	LCD_WriteCmd ( 0x2B );							// SSD1963_SET_PAGE_ADDRESS
	hi = StartY >> 8;
	lo = StartY & 0x00ff;
	
	LCD_WriteData ( hi );					// Hi
	LCD_WriteData ( lo );					// Low
	
	hi = (EndY-0) >> 8;
	lo = (EndY-0) & 0x00ff;
	LCD_WriteData ( hi );					// Hi
	LCD_WriteData ( lo );					// Low
}

void SSD1963_Init (void){
	
	fmc_init();

    LCD_RST_ON;
    Delay_ms(150);
    LCD_RST_OFF;
    Delay_ms(150);

    // Soft reset
    LCD_WriteCmd ( 0x01 );     // software reset
    Delay_ms(100); 

    //LCD_WriteCmd ( 0x28 ); 
    //Set PLL MN
    LCD_WriteCmd ( 0xE2 ); 
    LCD_WriteData( 0x23 );
    LCD_WriteData( 0x02 );
    LCD_WriteData( 0x04 );
            
    //SET PLL
    LCD_WriteCmd ( 0xE0 ); 
    //Enable PLL
    LCD_WriteData( 0x01 );
    Delay_ms(200); 

    LCD_WriteCmd ( 0xE0 ); 
    //Lock PLL
    LCD_WriteData( 0x03 );
    Delay_ms(200); 

    LCD_WriteCmd ( 0x01 );   // software reset
    Delay_ms(200);

    //Set dsip off
    //LCD_WriteCmd ( 0x28 );   
    //Set lshift freq
    LCD_WriteCmd ( 0xE6 );     
    // PLL setting for PCLK, depends on resolution
    LCD_WriteData ( 0x04 );    
    LCD_WriteData ( 0xFF );    
    LCD_WriteData ( 0xFF );

    //Set LCD mode
    LCD_WriteCmd ( 0xB0 );                        
    //P1: A0:A5
    LCD_WriteData ( 0x27 ); 
    LCD_WriteData ( 0x00 ); 
    LCD_WriteData ( (HDP>>8) & 0xFF );  
    LCD_WriteData ( HDP & 0xFF );
    LCD_WriteData ( (VDP>>8) & 0xFF );  
    LCD_WriteData ( VDP & 0xFF );
    LCD_WriteData (0x00); 

    LCD_WriteCmd (0xB4);
    LCD_WriteData ( (HT>>8)& 0xFF );          // Set HT
    LCD_WriteData ( HT & 0xFF );
    LCD_WriteData ( (HPS >> 8) & 0XFF );      // Set HPS
    LCD_WriteData ( HPS & 0xFF );
    LCD_WriteData ( HPW );                    // Set HPW
    LCD_WriteData ( (LPS>>8) & 0XFF );        // Set HPS
    LCD_WriteData ( LPS & 0XFF );
    LCD_WriteData ( 0x00 );

    LCD_WriteCmd ( 0xB6 );                    // Set vertical period
    LCD_WriteData ( (VT>>8) & 0xFF );         // Set VT
    LCD_WriteData ( VT & 0xFF );
    LCD_WriteData ( (VPS>>8) & 0xFF );        // Set VPS
    LCD_WriteData ( VPS & 0xFF );
    LCD_WriteData ( VPW );                    // Set VPW
    LCD_WriteData ( (FPS>>8) & 0xFF );        // Set FPS
    LCD_WriteData ( FPS & 0xFF );

    LCD_WriteCmd ( 0xBA );
    LCD_WriteData( 0x0F );                         // GPIO[3:0] out 1

    LCD_WriteCmd ( 0xB8 );
    LCD_WriteData( 0x07 );                         // 0x07 GPIO3=input, GPIO[2:0]=output
    LCD_WriteData( 0x01 );                         // 0x01 GPIO0 normal

    LCD_WriteCmd ( 0x36 );                         // Set Address mode - rotation
    LCD_WriteData (FlipState|(uint8_t)SSD1963_Hor_DecrDecr);

    LCD_WriteCmd ( 0xBC );        
    LCD_WriteData( 0x40 );      
    LCD_WriteData( 0x90 );
    LCD_WriteData( 0x40 );       
    LCD_WriteData( 0x01 );       

    LCD_WriteCmd ( 0xF0 );                         // pixel data interface
    LCD_WriteData( 0x03 );                         // 03h - RGB565
    //Delay(100);
    LCD_WriteCmd ( 0x29 );                         // display on

    LCD_WriteCmd ( 0xd0 );  
    LCD_WriteData( 0x0d ); 

}
