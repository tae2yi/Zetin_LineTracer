#include "custom_lcd.h"
#include <stdio.h>
#include "st7789_lcd.h"
#include "st7735_lcd.h"

static Custom_LCD_Type_t current_lcd_type = LCD_TYPE_ST7789;

static uint16_t posX = 0;
static uint16_t posY = 0;
static uint16_t color = LCD_COLOR_WHITE;
static uint8_t size = 12; // 기본 사이즈 12 (6x12)

void Custom_LCD_Init(Custom_LCD_Type_t type) {
    current_lcd_type = type;
    if (current_lcd_type == LCD_TYPE_ST7789) {
        ST7789_WRAPPER_Test(); // 초기화 수행
    } else {
        ST7735_WRAPPER_Test(); // 초기화 수행
    }
}

void Custom_LCD_Clear(void) {
    if (current_lcd_type == LCD_TYPE_ST7789) {
        ST7789_WRAPPER_Clear();
    } else {
        ST7735_WRAPPER_Clear();
    }
    
    posX = 0;
    posY = 0;
    color = LCD_COLOR_WHITE;
    size = 12;
}

void Custom_LCD_SetBrightness(uint32_t brightness) {
    if (current_lcd_type == LCD_TYPE_ST7789) ST7789_WRAPPER_SetBrightness(brightness);
    else ST7735_WRAPPER_SetBrightness(brightness);
}

void Custom_LCD_Printf(const char *format, ...) {
    char buffer[128];

    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    int cursor = 0;
    while (buffer[cursor] != '\0') {
        if (buffer[cursor] == '/') {
            char nextChar = buffer[cursor + 1];

            if (nextChar == '/') {
                cursor += 1; // 슬래시 자체 출력 지원 안함
            } else {
                switch (nextChar) {
                    case '0': case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9':
                        posX = 0;
                        posY = 16 * (nextChar - '0');
                        break;
                    
                    case 'w': color = LCD_COLOR_WHITE; break;
                    case 'r': color = LCD_COLOR_RED; break;
                    case 'g': color = LCD_COLOR_GREEN; break;
                    case 'b': color = LCD_COLOR_BLUE; break;
                    case 'y': color = LCD_COLOR_YELLOW; break;
                    case 'c': color = LCD_COLOR_CYAN; break;
                    case 'm': color = LCD_COLOR_MAGENTA; break;
                    
                    case 'A': size = 16; break;
                    case 'a': size = 12; break;
                    case 'C': Custom_LCD_Clear(); break;
                    
                    case '#':
                        if (buffer[cursor+2] != '\0') {
                            uint32_t colordata = 0;
                            for(int j=0; j<6; j++){
                                char c = buffer[cursor + 2 + j];
                                colordata *= 16;
                                if(c >= 'A' && c <= 'F') colordata += c - 'A' + 10;
                                else if(c >= 'a' && c <= 'f') colordata += c - 'a' + 10;
                                else if(c >= '0' && c <= '9') colordata += c - '0';
                            }
                            color = ((colordata >> 8) & 0xF800) | ((colordata >> 5) & 0x07E0) | ((colordata >> 3) & 0x001F);
                            cursor += 6;
                        }
                        break;
                }
                cursor += 2;
                continue;
            }
        }

        // 실제 문자 출력
        if (current_lcd_type == LCD_TYPE_ST7789) {
            uint16_t old_color = ST7789_WRAPPER_POINT_COLOR;
            ST7789_WRAPPER_POINT_COLOR = color;
            ST7789_WRAPPER_ShowChar(posX, posY, buffer[cursor], size, 0); // 0: 덮어쓰기 (배경색 출력)
            ST7789_WRAPPER_POINT_COLOR = old_color;
        } else {
            uint16_t old_color = ST7735_WRAPPER_POINT_COLOR;
            ST7735_WRAPPER_POINT_COLOR = color;
            ST7735_WRAPPER_ShowChar(posX, posY, buffer[cursor], size, 0); // 0: 덮어쓰기 (배경색 출력)
            ST7735_WRAPPER_POINT_COLOR = old_color;
        }

        // 폰트 크기에 따라 X축 이동 (size 12 -> 폭 6, size 16 -> 폭 8)
        posX += (size / 2);
        cursor++;
    }
}
