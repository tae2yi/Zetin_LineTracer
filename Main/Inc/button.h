/*
 * button.h
 *
 *  Created on: 2026. 6. 25.
 *      Author: kth59
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

typedef enum {
    INPUT_CMD_NONE = 0,

    INPUT_CMD_L_SINGLE,
    INPUT_CMD_L_DOUBLE,
    INPUT_CMD_L_HOLD,

    INPUT_CMD_R_SINGLE,
    INPUT_CMD_R_DOUBLE,
    INPUT_CMD_R_HOLD,

    INPUT_CMD_K_SINGLE,
    INPUT_CMD_K_DOUBLE,
    INPUT_CMD_K_HOLD,
} UserInput_t;

typedef enum {
    BTN_EVENT_NONE = 0,
    BTN_EVENT_SINGLE_CLICK,
    BTN_EVENT_DOUBLE_CLICK,
    BTN_EVENT_LONG_PRESS_HOLD
} ButtonEvent_t;

typedef enum {
    BTN_STATE_IDLE,
    BTN_STATE_PRESSED,
    BTN_STATE_LONG_PRESS,
    BTN_STATE_WAIT_RELEASE
} ButtonState_t;

typedef struct {
    ButtonState_t state;
    uint32_t start_time;
    uint32_t last_repeat_time;

    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState active_state;
} ButtonHandle_t;

extern ButtonHandle_t btn_l;
extern ButtonHandle_t btn_r;
extern ButtonHandle_t btn_k;

UserInput_t Button_Get_Input(void);
bool Button_IsCenterPressed(void);
void Button_IgnoreCenterUntilRelease(void);
void Button_Wait_Release(ButtonHandle_t *btn);
void MX_Button_Init();

#endif /* INC_BUTTON_H_ */
