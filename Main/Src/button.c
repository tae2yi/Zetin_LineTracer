/*
 * button.c
 *
 *  Created on: 2026. 6. 25.
 *      Author: kth59
 */


#include "button.h"

#define BTN_DEBOUNCE_TIME       20
#define BTN_LONG_PRESS_TIME     500
#define BTN_DOUBLE_CLICK_GAP    250
#define BTN_LONG_PRESS_REPEAT   100

ButtonHandle_t btn_l;
ButtonHandle_t btn_r;
ButtonHandle_t btn_k;

void Button_Init_Internal(ButtonHandle_t *btn, GPIO_TypeDef *port, uint16_t pin, GPIO_PinState active_state) {
    btn->state = BTN_STATE_IDLE;
    btn->start_time = 0;
    btn->last_repeat_time = 0;
    btn->port = port;
    btn->pin = pin;
    btn->active_state = active_state;
}

static ButtonEvent_t Button_Get_Event(ButtonHandle_t *btn) {
    uint32_t now = HAL_GetTick();

    bool is_pressed = (HAL_GPIO_ReadPin(btn->port, btn->pin) == btn->active_state);

    ButtonEvent_t event = BTN_EVENT_NONE;

    switch (btn->state) {
        case BTN_STATE_IDLE:
            if (is_pressed) {
                btn->state = BTN_STATE_PRESSED;
                btn->start_time = now;
            }
            break;

        case BTN_STATE_PRESSED:
            if (!is_pressed) {
                if (now - btn->start_time >= BTN_DEBOUNCE_TIME) {
                    event = BTN_EVENT_SINGLE_CLICK; // 즉각적으로 싱글 클릭 반환!
                    btn->state = BTN_STATE_IDLE;
                } else {
                    btn->state = BTN_STATE_IDLE;
                }
            } else if (now - btn->start_time >= BTN_LONG_PRESS_TIME) {
                btn->state = BTN_STATE_LONG_PRESS;
                btn->last_repeat_time = now;
                event = BTN_EVENT_LONG_PRESS_HOLD;
            }
            break;

        case BTN_STATE_LONG_PRESS:
            if (!is_pressed) {
                btn->state = BTN_STATE_IDLE;
            } else {
                if (now - btn->last_repeat_time >= BTN_LONG_PRESS_REPEAT) {
                    event = BTN_EVENT_LONG_PRESS_HOLD;
                    btn->last_repeat_time = now;
                }
            }
            break;

        case BTN_STATE_WAIT_RELEASE:
            if (!is_pressed) {
                btn->state = BTN_STATE_IDLE;
            }
            break;
    }
    return event;
}

UserInput_t Button_Get_Input(void) {

    ButtonEvent_t evt_l = Button_Get_Event(&btn_l);
    ButtonEvent_t evt_r = Button_Get_Event(&btn_r);
    ButtonEvent_t evt_k = Button_Get_Event(&btn_k);

    if (evt_k == BTN_EVENT_SINGLE_CLICK)    return INPUT_CMD_K_SINGLE;
    if (evt_k == BTN_EVENT_DOUBLE_CLICK)    return INPUT_CMD_K_DOUBLE;
    if (evt_k == BTN_EVENT_LONG_PRESS_HOLD) return INPUT_CMD_K_HOLD;

    if (evt_l == BTN_EVENT_SINGLE_CLICK)    return INPUT_CMD_L_SINGLE;
    if (evt_l == BTN_EVENT_DOUBLE_CLICK)    return INPUT_CMD_L_DOUBLE;
    if (evt_l == BTN_EVENT_LONG_PRESS_HOLD) return INPUT_CMD_L_HOLD;

    if (evt_r == BTN_EVENT_SINGLE_CLICK)    return INPUT_CMD_R_SINGLE;
    if (evt_r == BTN_EVENT_DOUBLE_CLICK)    return INPUT_CMD_R_DOUBLE;
    if (evt_r == BTN_EVENT_LONG_PRESS_HOLD) return INPUT_CMD_R_HOLD;

    return INPUT_CMD_NONE;
}

bool Button_IsCenterPressed(void)
{
	return HAL_GPIO_ReadPin(btn_k.port, btn_k.pin) == btn_k.active_state;
}

void Button_IgnoreCenterUntilRelease(void)
{
	btn_k.state = BTN_STATE_WAIT_RELEASE;
}

void Button_Wait_Release(ButtonHandle_t *btn) {
	while (HAL_GPIO_ReadPin(btn->port, btn->pin) == btn->active_state);
}

void MX_Button_Init(){
    /*
     * Confirmed on the assembled board: PC6, PC13 and PC8 are active-high
     * inputs.  Their idle level is 0 and a press changes the level to 1.
     */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SWL_Pin | SWR_Pin | KEY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Active-high buttons use GPIO_PIN_SET as the pressed state. */
    Button_Init_Internal(&btn_k, KEY_GPIO_Port, KEY_Pin, GPIO_PIN_SET);
    Button_Init_Internal(&btn_l, SWL_GPIO_Port, SWL_Pin, GPIO_PIN_SET);
    Button_Init_Internal(&btn_r, SWR_GPIO_Port, SWR_Pin, GPIO_PIN_SET);
}
