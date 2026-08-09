#ifndef BSP_ST7789_ST7789_LCD_H
#define BSP_ST7789_ST7789_LCD_H

#include "main.h"
#include "fonts.h"
#include "st7789.h"
#include <stdio.h>

#define WHITE            0xFFFF
#define BLACK            0x0000
#define BLUE             0x001F
#define BRED             0XF81F
#define GRED             0XFFE0
#define GBLUE            0X07FF
#define RED              0xF800
#define MAGENTA          0xF81F
#define GREEN            0x07E0
#define CYAN             0x7FFF
#define YELLOW           0xFFE0
#define BROWN            0XBC40
#define BRRED            0XFC07
#define GRAY             0X8430
#define DARKBLUE         0X01CF
#define LIGHTBLUE        0X7D7C
#define GRAYBLUE         0X5458

extern ST7789_Object_t st7789_pObj;

extern uint16_t ST7789_WRAPPER_POINT_COLOR;
extern uint16_t ST7789_WRAPPER_BACK_COLOR;
extern uint16_t ST7789_WRAPPER_BACK_BRIGHT;

void ST7789_WRAPPER_Test(void);
void ST7789_WRAPPER_SetBrightness(uint32_t Brightness);
uint32_t ST7789_WRAPPER_GetBrightness(void);
void ST7789_WRAPPER_SoftPWMEnable(uint8_t enable);
uint8_t ST7789_WRAPPER_SoftPWMIsEnable(void);
void ST7789_WRAPPER_SoftPWMCtrlInit(void);
void ST7789_WRAPPER_SoftPWMCtrlDeInit(void);
void ST7789_WRAPPER_SoftPWMCtrlRun(void);
void ST7789_WRAPPER_Light(uint32_t Brightness_Dis, uint32_t time);
void ST7789_WRAPPER_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint8_t size, uint8_t mode);
void ST7789_WRAPPER_ShowString(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, uint8_t *p);
void ST7789_WRAPPER_Clear(void);
void ST7789_WRAPPER_Printf(uint16_t x, uint16_t y, const char *text, ...);
#endif /* BSP_ST7789_ST7789_LCD_H */
