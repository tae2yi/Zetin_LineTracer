#include "st7789.h"
#include "st7789_LCD.h"
#include "spi.h"
#include "lptim.h"
#include "fonts.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define TFT135x240

#define LCD_RS_SET      HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET)
#define LCD_RS_RESET    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET)
#define LCD_CS_SET      HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET)
#define LCD_CS_RESET    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET)

#define SPI_Drv (&hspi3)
#define delay_ms HAL_Delay
#define get_tick HAL_GetTick

#define LCD_Brightness_timer &hlptim1
#define LCD_Brightness_channel LPTIM_CHANNEL_2

static int32_t LCD_init(void);
static int32_t LCD_gettick(void);
static int32_t LCD_writereg(uint8_t reg, uint8_t *pdata, uint32_t length);
static int32_t LCD_readreg(uint8_t reg, uint8_t *pdata);
static int32_t LCD_senddata(uint8_t *pdata, uint32_t length);
static int32_t LCD_recvdata(uint8_t *pdata, uint32_t length);

uint16_t ST7789_WRAPPER_BACK_BRIGHT = 600;

ST7789_IO_t st7789_pIO = { LCD_init, 0, 0, LCD_writereg,
		LCD_readreg, LCD_senddata, LCD_recvdata, LCD_gettick };

ST7789_Object_t st7789_pObj;

void ST7789_WRAPPER_Test(void) {

#if defined(TFT135x240)
	ST7789Ctx.Orientation = ST7789_ORIENTATION_LANDSCAPE;
	ST7789Ctx.Type = ST7789_135x240_screen;
#elif defined(TFT240x240)
	ST7789Ctx.Orientation = ST7789_ORIENTATION_PORTRAIT;
	ST7789Ctx.Type = ST7789_240x240_screen;
#else
	#error "Unknown Screen"
#endif

	ST7789_RegisterBusIO(&st7789_pObj, &st7789_pIO);
	ST7789_LCD_Driver.Init(&st7789_pObj, ST7789_FORMAT_RBG565, &ST7789Ctx);

	ST7789_LCD_Driver.FillRect(&st7789_pObj, 0, 0, ST7789Ctx.Width, ST7789Ctx.Height, BLACK);
	ST7789_WRAPPER_Light(ST7789_WRAPPER_BACK_BRIGHT, 300);
}

static uint32_t ST7789_WRAPPER_LightSet;
static uint8_t IsST7789_WRAPPER_SoftPWM = 0;

void ST7789_WRAPPER_SetBrightness(uint32_t Brightness) {
	ST7789_WRAPPER_LightSet = Brightness;
	if (!IsST7789_WRAPPER_SoftPWM)
		__HAL_LPTIM_COMPARE_SET(LCD_Brightness_timer, LCD_Brightness_channel, Brightness);
}

uint32_t ST7789_WRAPPER_GetBrightness(void) {
	if (IsST7789_WRAPPER_SoftPWM)
		return ST7789_WRAPPER_LightSet;
	else
		return (*LCD_Brightness_timer).Instance->CCR2;
}

void ST7789_WRAPPER_SoftPWMEnable(uint8_t enable) {
	IsST7789_WRAPPER_SoftPWM = enable;
	if (!enable)
		ST7789_WRAPPER_SetBrightness(ST7789_WRAPPER_LightSet);
}

//uint8_t ST7789_WRAPPER_SoftPWMIsEnable(void) {
//	return IsST7789_WRAPPER_SoftPWM;
//}

//void ST7789_WRAPPER_SoftPWMCtrlInit(void) {
//	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
//	__HAL_RCC_GPIOE_CLK_ENABLE();
//	GPIO_InitStruct.Pin = GPIO_PIN_10;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
//
//	ST7789_WRAPPER_SoftPWMEnable(1);
//}
//
//void ST7789_WRAPPER_SoftPWMCtrlDeInit(void) {
//	HAL_GPIO_DeInit(GPIOE, GPIO_PIN_10);
//}
//
//void ST7789_WRAPPER_SoftPWMCtrlRun(void) {
//	static uint32_t timecount;
//	if (timecount > 1000)
//		timecount = 0;
//	else
//		timecount += 10;
//
//	if (timecount >= ST7789_WRAPPER_LightSet)
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
//	else
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
//}

void ST7789_WRAPPER_Light(uint32_t Brightness_Dis, uint32_t time) {
	uint32_t Brightness_Now;
	uint32_t time_now;
	float temp1, temp2;
	float k, set;

	Brightness_Now = ST7789_WRAPPER_GetBrightness();
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
		ST7789_WRAPPER_SetBrightness((uint32_t) set);
		if (time_now >= time)
			break;
	}
}

uint16_t ST7789_WRAPPER_POINT_COLOR = 0xFFFF;
uint16_t ST7789_WRAPPER_BACK_COLOR = BLACK;

void ST7789_WRAPPER_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint8_t size, uint8_t mode) {
	uint8_t temp, t1, t;
	uint16_t y0 = y;
	uint16_t x0 = x;
	uint16_t colortemp = ST7789_WRAPPER_POINT_COLOR;
	uint32_t h, w;

	uint16_t write[size][size == 12 ? 6 : 8];
	uint16_t count;

	ST7789_GetXSize(&st7789_pObj, &w);
	ST7789_GetYSize(&st7789_pObj, &h);

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
					ST7789_WRAPPER_POINT_COLOR = (colortemp & 0xFF) << 8 | colortemp >> 8;
				else
					ST7789_WRAPPER_POINT_COLOR = (ST7789_WRAPPER_BACK_COLOR & 0xFF) << 8 | ST7789_WRAPPER_BACK_COLOR >> 8;

				write[count][t / 2] = ST7789_WRAPPER_POINT_COLOR;
				count++;
				if (count >= size)
					count = 0;

				temp <<= 1;
				y++;
				if (y > h) {
					ST7789_WRAPPER_POINT_COLOR = colortemp;
					return;
				}
				if ((y - y0) == size) {
					y = y0;
					x++;
					if (x >= w) {
						ST7789_WRAPPER_POINT_COLOR = colortemp;
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
					write[count][t / 2] = (ST7789_WRAPPER_POINT_COLOR & 0xFF) << 8 | ST7789_WRAPPER_POINT_COLOR >> 8;
				count++;
				if (count >= size)
					count = 0;

				temp <<= 1;
				y++;
				if (y > h) {
					ST7789_WRAPPER_POINT_COLOR = colortemp;
					return;
				}
				if ((y - y0) == size) {
					y = y0;
					x++;
					if (x >= w) {
						ST7789_WRAPPER_POINT_COLOR = colortemp;
						return;
					}
					break;
				}
			}
		}
	}
	ST7789_FillRGBRect(&st7789_pObj, x0, y0, (uint8_t*) &write, size == 12 ? 6 : 8, size);
	ST7789_WRAPPER_POINT_COLOR = colortemp;
}

void ST7789_WRAPPER_ShowString(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, uint8_t *p) {
	uint8_t x0 = x;
	width += x;
	height += y;
	while ((*p <= '~') && (*p >= ' ')) {
		if (x + size / 2 > width) {
			x = x0;
			y += size;
		}
		if (y > height)
			break;
		ST7789_WRAPPER_ShowChar(x, y, *p, size, 0);
		x += size / 2;
		p++;
	}
}

void ST7789_WRAPPER_Printf(uint16_t x, uint16_t y, const char *text, ...) {
	char txt[512] = { 0 };
	va_list args;
	va_start(args, text);
	vsprintf(txt, text, args);
	va_end(args);

	static uint8_t fixed_size = 16;
	static uint8_t x_bias;
	static uint8_t y_bias;
	static uint8_t offset_x = 0;
	static uint8_t offset_y = 0;

	switch (fixed_size) {
	case 12:
		x_bias = 6;
		y_bias = 12;
		offset_x = 1;
		offset_y = 0;
		break;
	default:
		x_bias = 8;
		y_bias = 16;
		offset_x = 3;
		offset_y = 0;
	}

	ST7789_WRAPPER_ShowString(x_bias * x + offset_x, y_bias * y + offset_y,
			ST7789Ctx.Width - x, ST7789Ctx.Height - y, fixed_size,
			(uint8_t*) txt);
}

static int32_t LCD_init(void) {
	HAL_LPTIM_PWM_Start(LCD_Brightness_timer, LCD_Brightness_channel);
	return ST7789_OK;
}

static int32_t LCD_gettick(void) {
	return HAL_GetTick();
}

static int32_t LCD_writereg(uint8_t reg, uint8_t *pdata, uint32_t length) {
	int32_t result;
	LCD_CS_RESET;
	LCD_RS_RESET;

	result = HAL_SPI_Transmit(SPI_Drv, &reg, 1, HAL_MAX_DELAY);
	LCD_RS_SET;

	if (length > 0) {
		uint32_t remaining = length;
		uint8_t *ptr = pdata;

		while (remaining > 0) {
			uint16_t chunk = (remaining > 65535) ? 65535 : (uint16_t) remaining;
			HAL_SPI_Transmit(SPI_Drv, ptr, chunk, HAL_MAX_DELAY);
			ptr += chunk;
			remaining -= chunk;
		}
	}

	LCD_CS_SET;
	return result > 0 ? -1 : 0;
}

static int32_t LCD_readreg(uint8_t reg, uint8_t *pdata) {
	int32_t result;
	LCD_CS_RESET;
	LCD_RS_RESET;

	result = HAL_SPI_Transmit(SPI_Drv, &reg, 1, HAL_MAX_DELAY);
	LCD_RS_SET;

	result += HAL_SPI_Receive(SPI_Drv, pdata, 1, HAL_MAX_DELAY);

	LCD_CS_SET;
	return result > 0 ? -1 : 0;
}

static int32_t LCD_senddata(uint8_t *pdata, uint32_t length) {
	LCD_CS_RESET;
	uint32_t remaining = length;
	uint8_t *ptr = pdata;

	while (remaining > 0) {
		uint16_t chunk = (remaining > 65535) ? 65535 : (uint16_t) remaining;
		HAL_SPI_Transmit(SPI_Drv, ptr, chunk, HAL_MAX_DELAY);
		ptr += chunk;
		remaining -= chunk;
	}

	LCD_CS_SET;
	return 0;
}

static int32_t LCD_recvdata(uint8_t *pdata, uint32_t length) {
	LCD_CS_RESET;
	uint32_t remaining = length;
	uint8_t *ptr = pdata;

	while (remaining > 0) {
		uint16_t chunk = (remaining > 65535) ? 65535 : (uint16_t) remaining;
		HAL_SPI_Receive(SPI_Drv, ptr, chunk, HAL_MAX_DELAY);
		ptr += chunk;
		remaining -= chunk;
	}

	LCD_CS_SET;
	return 0;
}

void ST7789_WRAPPER_Clear() {
	ST7789_WRAPPER_Light(0, 250);
	ST7789_LCD_Driver.FillRect(&st7789_pObj, 0, 0, ST7789Ctx.Width,
			ST7789Ctx.Height, BLACK);
	ST7789_WRAPPER_Light(900, 250);
}

void LCD_Set_Color(uint16_t point, uint16_t back) {
	ST7789_WRAPPER_POINT_COLOR = point;
	ST7789_WRAPPER_BACK_COLOR = back;
}
