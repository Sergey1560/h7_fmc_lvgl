#ifndef SSD1963_H
#define SSD1963_H
#include "common_defs.h"
#include "clocks.h"
#include "fmc.h"
#include "delay.h"

#define FILL_BY_DMA		1
#define FILL_BY_PIXEL	0

#define LCD_WriteCmd(cmd)   { *(volatile uint16_t *) (LCD_REG) = cmd; Delay_ms(1); }
#define LCD_WriteData(data)     { *(volatile uint16_t *) (LCD_DATA) = data; }

#define DMA_MEM_INCR (uint32_t) ((2 << DMA_SxCR_DIR_Pos)| \
                                  DMA_SxCR_PINC|\
                                  DMA_SxCR_MINC|\
                                  DMA_SxCR_MSIZE_1|\
                                  DMA_SxCR_PSIZE_1)

#define DMA_MEM_STAT (uint32_t) ((2 << DMA_SxCR_DIR_Pos)| \
                                  DMA_SxCR_MINC|\
                                  DMA_SxCR_MSIZE_1|\
                                  DMA_SxCR_PSIZE_1)

#define DMA_CLEAR  		        ((uint32_t) DMA_LIFCR_CDMEIF1| DMA_LIFCR_CFEIF1 | DMA_LIFCR_CHTIF1  |DMA_LIFCR_CTCIF1  |DMA_LIFCR_CTEIF1)


#define HDP		(DISP_WIDTH - 1)
#define HT 		928
#define HPS		46
#define LPS		15
#define HPW		48

#define VDP		(DISP_HEIGHT - 1)
#define VT		525
#define VPS		16
#define FPS		8
#define VPW		16

//0 or 3
#define FlipState 3

#define   BLACK        0x0000
#define   NAVY         0x000F
#define   DGREEN       0x03E0
#define   DCYAN        0x03EF
#define   MAROON       0x7800
#define   PURPLE       0x780F
#define   OLIVE        0x7BE0
#define   GREY         0xF7DE
#define   LGRAY        0xC618
#define   DGRAY        0x7BEF
#define   BLUE         0x001F
#define   GREEN        0x07E0
#define   CYAN         0x07FF
#define   RED          0xF800
#define   MAGENTA      0xF81F
#define   YELLOW       0xFFE0
#define   WHITE        0xFFFF

#define   COLOR1			 0x0F0F
#define   COLOR2			 0xC618
#define   COLOR3			 0xF054 //RGB F708A5


typedef enum
{
	SSD1963_Hor_DecrDecr = 0x0000,		// ID=00, AM=0 - 0000
	SSD1963_Hor_IncrDecr = 0x0020,		// ID=01, AM=0 - 0010
	SSD1963_Hor_DecrIncr = 0x0040,		// ID=10, AM=0 - 0100
	SSD1963_Hor_IncrIncr = 0x0070,		// ID=11, AM=0 - 0110
	SSD1963_Vert_DecrDecr = 0x0080,		// ID=00, AM=1 - 1000
	SSD1963_Vert_IncrDecr = 0x00A0,		// ID=01, AM=1 - 1010
	SSD1963_Vert_DecrIncr = 0x00C0,		// ID=10, AM=1 - 1100
	SSD1963_Vert_IncrIncr = 0x00E0 		// ID=11, AM=1 - 1110
} TSSD1963Rotate;

void SSD1963_Init (void);
void SSD1963_SetCursor (uint16_t Xpos, uint16_t Ypos);
void SetWindows(uint16_t StartX,uint16_t StartY,uint16_t EndX,uint16_t EndY);
void LCD_fill_area(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color, uint8_t fill_type);
void fmc_dma_copy(uint32_t *src, uint32_t *dst, uint32_t len, uint8_t incr);

#endif