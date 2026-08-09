#ifndef INC_CUSTOM_LCD_H_
#define INC_CUSTOM_LCD_H_

#include <stdint.h>
#include <stdarg.h>

/**
 * @brief 지원하는 LCD 디스플레이 타입
 */
typedef enum {
    LCD_TYPE_ST7789,
    LCD_TYPE_ST7735
} Custom_LCD_Type_t;

/**
 * @brief OLED 및 LCD에서 공통으로 사용할 16비트 색상 (RGB565)
 */
enum Custom_LCD_Colors {
    LCD_COLOR_RED        = 0xF800,
    LCD_COLOR_GREEN      = 0x07E0,
    LCD_COLOR_BLUE       = 0x001F,
    LCD_COLOR_RED_DARK   = 0x7800,
    LCD_COLOR_GREEN_DARK = 0x03E0,
    LCD_COLOR_BLUE_DARK  = 0x000F,

    LCD_COLOR_YELLOW     = 0xF800 | 0x07E0,
    LCD_COLOR_CYAN       = 0x07E0 | 0x001F,
    LCD_COLOR_MAGENTA    = 0xF800 | 0x001F,

    LCD_COLOR_ORANGE     = 0xF800 | 0x03E0,
    LCD_COLOR_MINT       = 0x07E0 | 0x000F,
    LCD_COLOR_ROSE       = 0xF800 | 0x000F,

    LCD_COLOR_LIME       = 0x7800 | 0x07E0,
    LCD_COLOR_SEA        = 0x03E0 | 0x001F,
    LCD_COLOR_VIOLET     = 0x7800 | 0x001F,

    LCD_COLOR_WHITE      = 0xF800 | 0x07E0 | 0x001F,
    LCD_COLOR_GRAY       = 0x7800 | 0x03E0 | 0x000F,
    LCD_COLOR_BLACK      = 0,
};

/**
 * @brief 사용할 LCD 타입을 지정하고 초기화하는 함수.
 *        내부 변수 설정을 위해 호출하며, 실제 하드웨어 Init은 각각의 LCD_Test() 또는 Init()에서 진행.
 * 
 * @param type LCD_TYPE_ST7789 또는 LCD_TYPE_ST7735
 */
void Custom_LCD_Init(Custom_LCD_Type_t type);

/**
 * @brief 서식을 포함한 문자열을 LCD에 출력하는 함수.
 *        과거 Custom_OLED_Printf 와 동일한 파싱 방식 지원 (/0~9 행 이동, /r, /w 등 색상)
 * 
 * @param format 서식 문자열
 * @param ... 
 */
void Custom_LCD_Printf(const char *format, ...);

/**
 * @brief 화면을 검은색으로 지우고 커서 위치를 맨 앞으로 초기화하는 함수
 */
void Custom_LCD_Clear(void);

/**
 * @brief LCD 백라이트 밝기를 설정하는 함수
 * @param brightness 밝기 값 (0 ~ 1000)
 */
void Custom_LCD_SetBrightness(uint32_t brightness);

#endif /* INC_CUSTOM_LCD_H_ */
