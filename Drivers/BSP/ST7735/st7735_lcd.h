#ifndef BSP_ST7735_ST7735_LCD_H
#define BSP_ST7735_ST7735_LCD_H

#include "main.h"
#include "st7735.h"
#include <stdio.h>

#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	 0x001F  
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40
#define BRRED 			 0XFC07
#define GRAY  			 0X8430
#define DARKBLUE      	 0X01CF
#define LIGHTBLUE      	 0X7D7C
#define GRAYBLUE       	 0X5458

extern ST7735_Object_t st7735_pObj;
extern uint32_t st7735_id;

extern uint16_t ST7735_WRAPPER_POINT_COLOR;
extern uint16_t ST7735_WRAPPER_BACK_COLOR;
extern uint16_t ST7735_WRAPPER_BACK_BRIGHT;

void ST7735_WRAPPER_Test(void);
void ST7735_WRAPPER_SetBrightness(uint32_t Brightness);
uint32_t ST7735_WRAPPER_GetBrightness(void);
void ST7735_WRAPPER_SoftPWMEnable(uint8_t enable);
uint8_t ST7735_WRAPPER_SoftPWMIsEnable(void);
void ST7735_WRAPPER_SoftPWMCtrlInit(void);
void ST7735_WRAPPER_SoftPWMCtrlDeInit(void);
void ST7735_WRAPPER_SoftPWMCtrlRun(void);
void ST7735_WRAPPER_Light(uint32_t Brightness_Dis, uint32_t time);
void ST7735_WRAPPER_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint8_t size, uint8_t mode);
void ST7735_WRAPPER_ShowString(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, uint8_t *p);
void ST7735_WRAPPER_Clear(void);
void ST7735_WRAPPER_Printf(uint8_t x, uint8_t y, const char *text, ...);

#endif /* __ST7735_LCD_H */
