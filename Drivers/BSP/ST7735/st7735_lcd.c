#include "st7735.h"
#include "st7735_LCD.h"
#include "font.h"
#include "spi.h"
#include "lptim.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define TFT96

//SPI 통신 인터페이스
//LCD_RST
#define LCD_RST_SET     
#define LCD_RST_RESET  
//LCD_RS//dc  
#define LCD_RS_SET    HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin,GPIO_PIN_SET)
#define LCD_RS_RESET    HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin,GPIO_PIN_RESET)
//LCD_CS  
#define LCD_CS_SET      HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin,GPIO_PIN_SET)
#define LCD_CS_RESET    HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin,GPIO_PIN_RESET)
//SPI Driver
#define SPI spi3
#define SPI_Drv (&hspi3)
#define delay_ms HAL_Delay
#define get_tick HAL_GetTick
//LCD_Brightness timer
#define LCD_Brightness_timer &hlptim1
#define LCD_Brightness_channel LPTIM_CHANNEL_2

static int32_t LCD_init(void);
static int32_t LCD_gettick(void);
static int32_t LCD_writereg(uint8_t reg, uint8_t *pdata, uint32_t length);
static int32_t LCD_readreg(uint8_t reg, uint8_t *pdata);
static int32_t LCD_senddata(uint8_t *pdata, uint32_t length);
static int32_t LCD_recvdata(uint8_t *pdata, uint32_t length);

uint16_t ST7735_WRAPPER_BACK_BRIGHT = 600;

ST7735_IO_t st7735_pIO = { LCD_init, 0, 0, LCD_writereg, LCD_readreg,
		LCD_senddata, LCD_recvdata, LCD_gettick };

ST7735_Object_t st7735_pObj;
uint32_t st7735_id;

void ST7735_WRAPPER_Test(void) {
	uint8_t text[20];

#if defined(TFT96)
	ST7735Ctx.Orientation = ST7735_ORIENTATION_LANDSCAPE_ROT180;
	ST7735Ctx.Panel = HannStar_Panel;
	ST7735Ctx.Type = ST7735_0_9_inch_screen;
#elif defined(TFT18)
	ST7735Ctx.Orientation = ST7735_ORIENTATION_LANDSCAPE_ROT180;
	ST7735Ctx.Panel = HannStar_Panel;
	ST7735Ctx.Type = ST7735_1_8a_inch_screen;
#else
	#error "Unknown Screen"
#endif

	ST7735_RegisterBusIO(&st7735_pObj, &st7735_pIO);
	ST7735_LCD_Driver.Init(&st7735_pObj, ST7735_FORMAT_RBG565, &ST7735Ctx);
	ST7735_LCD_Driver.ReadID(&st7735_pObj, &st7735_id);

	ST7735_WRAPPER_SetBrightness(0);
	ST7735_WRAPPER_Clear();

	// 버튼 입력을 기다리던 기존 while 루프(테스트 로직) 삭제
	// 바로 화면을 검은색으로 지우고 종료하여 Main_Menu로 빠르게 넘어가게 함

	ST7735_LCD_Driver.FillRect(&st7735_pObj, 0, 0, ST7735Ctx.Width,
			ST7735Ctx.Height, BLACK);

	ST7735_WRAPPER_Light(ST7735_WRAPPER_BACK_BRIGHT, 300);
}

static uint32_t ST7735_WRAPPER_LightSet;
static uint8_t IsST7735_WRAPPER_SoftPWM = 0;

void ST7735_WRAPPER_SetBrightness(uint32_t Brightness) {
	ST7735_WRAPPER_LightSet = Brightness;
	if (!IsST7735_WRAPPER_SoftPWM) {
		__HAL_LPTIM_COMPARE_SET(LCD_Brightness_timer, LCD_Brightness_channel,
				Brightness);
	}
}

uint32_t ST7735_WRAPPER_GetBrightness(void) {
	if (IsST7735_WRAPPER_SoftPWM)
		return ST7735_WRAPPER_LightSet;
	else
		return (*LCD_Brightness_timer).Instance->CCR2;
}

void ST7735_WRAPPER_SoftPWMEnable(uint8_t enable) {
	IsST7735_WRAPPER_SoftPWM = enable;
	if (!enable)
		ST7735_WRAPPER_SetBrightness(ST7735_WRAPPER_LightSet);
}
//
//void ST7735_WRAPPER_SoftPWMCtrlInit(void) {
//	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
//
//	__HAL_RCC_GPIOE_CLK_ENABLE();
//	GPIO_InitStruct.Pin = GPIO_PIN_10;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
//
//	MX_TIM16_Init(); // Freq: 10K
//	HAL_TIM_Base_Start_IT(&htim16);
//
//	ST7735_WRAPPER_SoftPWMEnable(1);
//}
//
//void ST7735_WRAPPER_SoftPWMCtrlDeInit(void) {
//	HAL_TIM_Base_DeInit(&htim16);
//	HAL_GPIO_DeInit(GPIOE, GPIO_PIN_10);
//}
//
//void ST7735_WRAPPER_SoftPWMCtrlRun(void) {
//	static uint32_t timecount;
//
//	if (timecount > 1000)
//		timecount = 0;
//	else
//		timecount += 10;
//
//	if (timecount >= ST7735_WRAPPER_LightSet)
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
//	else
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
//}

/* * HAL_TIM_PeriodElapsedCallback 은 메인 인터럽트 파일(stm32h7xx_it.c 등)에
 * 전역으로 하나만 존재해야 하므로 이곳에서는 주석 처리합니다.
 */
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
// 	if (htim->Instance == TIM16) {
// 		ST7735_WRAPPER_SoftPWMCtrlRun();
// 	}
// }
void ST7735_WRAPPER_Light(uint32_t Brightness_Dis, uint32_t time) {
	uint32_t Brightness_Now;
	uint32_t time_now;
	float temp1, temp2;
	float k, set;

	Brightness_Now = ST7735_WRAPPER_GetBrightness();
	time_now = 0;
	if (Brightness_Now == Brightness_Dis)
		return;

	if (time == time_now)
		return;

	temp1 = Brightness_Now;
	temp1 = temp1 - Brightness_Dis;
	temp2 = time_now;
	temp2 = temp2 - time;

	k = -temp1 / temp2;

	uint32_t tick = get_tick();
	while (1) {
		delay_ms(1);
		time_now = get_tick() - tick;
		temp2 = time_now - 0;
		set = temp2 * k + Brightness_Now;
		ST7735_WRAPPER_SetBrightness((uint32_t) set);
		if (time_now >= time)
			break;
	}
}

uint16_t ST7735_WRAPPER_POINT_COLOR = 0xFFFF;
uint16_t ST7735_WRAPPER_BACK_COLOR = BLACK;

void ST7735_WRAPPER_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint8_t size,
		uint8_t mode) {
	uint8_t temp, t1, t;
	uint16_t y0 = y;
	uint16_t x0 = x;
	uint16_t colortemp = ST7735_WRAPPER_POINT_COLOR;
	uint32_t h, w;

	uint16_t write[size][size == 12 ? 6 : 8];
	uint16_t count;

	ST7735_GetXSize(&st7735_pObj, &w);
	ST7735_GetYSize(&st7735_pObj, &h);

	num = num - ' ';
	count = 0;

	if (!mode) {
		for (t = 0; t < size; t++) {
			if (size == 12)
				temp = asc2_1206[num][t];
			else
				temp = asc2_1608[num][t];

			for (t1 = 0; t1 < 8; t1++) {
				if (temp & 0x80)
					ST7735_WRAPPER_POINT_COLOR = (colortemp & 0xFF) << 8
							| colortemp >> 8;
				else
					ST7735_WRAPPER_POINT_COLOR = (ST7735_WRAPPER_BACK_COLOR
							& 0xFF) << 8 | ST7735_WRAPPER_BACK_COLOR >> 8;

				write[count][t / 2] = ST7735_WRAPPER_POINT_COLOR;
				count++;
				if (count >= size)
					count = 0;

				temp <<= 1;
				y++;
				if (y >= h) {
					ST7735_WRAPPER_POINT_COLOR = colortemp;
					return;
				}
				if ((y - y0) == size) {
					y = y0;
					x++;
					if (x >= w) {
						ST7735_WRAPPER_POINT_COLOR = colortemp;
						return;
					}
					break;
				}
			}
		}
	} else {
		for (t = 0; t < size; t++) {
			if (size == 12)
				temp = asc2_1206[num][t];
			else
				temp = asc2_1608[num][t];
			for (t1 = 0; t1 < 8; t1++) {
				if (temp & 0x80)
					write[count][t / 2] = (ST7735_WRAPPER_POINT_COLOR & 0xFF)
							<< 8 | ST7735_WRAPPER_POINT_COLOR >> 8;
				count++;
				if (count >= size)
					count = 0;

				temp <<= 1;
				y++;
				if (y >= h) {
					ST7735_WRAPPER_POINT_COLOR = colortemp;
					return;
				}
				if ((y - y0) == size) {
					y = y0;
					x++;
					if (x >= w) {
						ST7735_WRAPPER_POINT_COLOR = colortemp;
						return;
					}
					break;
				}
			}
		}
	}
	ST7735_FillRGBRect(&st7735_pObj, x0, y0, (uint8_t*) &write,
			size == 12 ? 6 : 8, size);
	ST7735_WRAPPER_POINT_COLOR = colortemp;
}

void ST7735_WRAPPER_ShowString(uint16_t x, uint16_t y, uint16_t width,
		uint16_t height, uint8_t size, uint8_t *p) {
	uint8_t x0 = x;
	width += x;
	height += y;
	while ((*p <= '~') && (*p >= ' ')) {
		if (x + size / 2 > width) {
			x = x0;
			y += size;
		}
		if (y >= height)
			break;
		ST7735_WRAPPER_ShowChar(x, y, *p, size, 0);
		x += size / 2;
		p++;
	}
}

static int32_t LCD_init(void) {
	int32_t result = ST7735_OK;
	HAL_LPTIM_PWM_Start(LCD_Brightness_timer, LCD_Brightness_channel);
	return result;
}

static int32_t LCD_gettick(void) {
	return HAL_GetTick();
}

static int32_t LCD_writereg(uint8_t reg, uint8_t *pdata, uint32_t length) {
	int32_t result;
	LCD_CS_RESET;
	LCD_RS_RESET;
	result = HAL_SPI_Transmit(SPI_Drv, &reg, 1, 100);
	LCD_RS_SET;
	if (length > 0)
		result += HAL_SPI_Transmit(SPI_Drv, pdata, length, 500);
	LCD_CS_SET;
	result = result > 0 ? -1 : 0;
	return result;
}

static int32_t LCD_readreg(uint8_t reg, uint8_t *pdata) {
	int32_t result;
	LCD_CS_RESET;
	LCD_RS_RESET;

	result = HAL_SPI_Transmit(SPI_Drv, &reg, 1, 100);
	LCD_RS_SET;
	result += HAL_SPI_Receive(SPI_Drv, pdata, 1, 500);
	LCD_CS_SET;
	result = result > 0 ? -1 : 0;
	return result;
}

static int32_t LCD_senddata(uint8_t *pdata, uint32_t length) {
	int32_t result;
	LCD_CS_RESET;
	result = HAL_SPI_Transmit(SPI_Drv, pdata, length, 100);
	LCD_CS_SET;
	result = result > 0 ? -1 : 0;
	return result;
}

static int32_t LCD_recvdata(uint8_t *pdata, uint32_t length) {
	int32_t result;
	LCD_CS_RESET;
	result = HAL_SPI_Receive(SPI_Drv, pdata, length, 500);
	LCD_CS_SET;
	result = result > 0 ? -1 : 0;
	return result;
}

void ST7735_WRAPPER_Printf(uint8_t x, uint8_t y, const char *text, ...) {
	char txt[512] = { 0 };
	va_list args;
	va_start(args, text);
	vsprintf(txt, text, args);
	va_end(args);

	uint16_t px = 6 * x + 1;
	uint16_t py = 14 * y + 3;

	ST7735_WRAPPER_ShowString(px, py, ST7735Ctx.Width - px - 1,
			ST7735Ctx.Height - py - 1, 12, (uint8_t*) txt);
}

void ST7735_WRAPPER_Clear() {
	ST7735_WRAPPER_Light(0, 250);

	ST7735_LCD_Driver.FillRect(&st7735_pObj, 0, 0, ST7735Ctx.Width,
			ST7735Ctx.Height, BLACK);

	ST7735_WRAPPER_Light(900, 250);
}
