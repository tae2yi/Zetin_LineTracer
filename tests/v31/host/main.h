#ifndef V31_HOST_MAIN_H
#define V31_HOST_MAIN_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	HAL_OK = 0,
	HAL_ERROR = 1
} HAL_StatusTypeDef;

typedef struct {
	uintptr_t id;
} GPIO_TypeDef;

typedef struct {
	uint32_t prescaler;
	uint32_t autoreload;
	uint32_t counter;
	bool running;
} TIM_HandleTypeDef;

typedef struct {
	bool running;
	uint16_t value;
} DAC_HandleTypeDef;

#define GPIO_PIN_RESET             0U
#define GPIO_PIN_SET               1U
#define TIM_EVENTSOURCE_UPDATE    1U
#define TIM_FLAG_UPDATE            1U
#define DAC_CHANNEL_2              2U
#define DAC_ALIGN_12B_R            0U

#define __STATIC_INLINE static inline
#define __HAL_TIM_DISABLE(timer)             ((timer)->running = false)
#define __HAL_TIM_SET_PRESCALER(timer, val)  ((timer)->prescaler = (val))
#define __HAL_TIM_SET_AUTORELOAD(timer, val) ((timer)->autoreload = (val))
#define __HAL_TIM_SET_COUNTER(timer, val)    ((timer)->counter = (val))
#define __HAL_TIM_CLEAR_FLAG(timer, flag)    ((void)(flag))

extern uint32_t mock_tick;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern DAC_HandleTypeDef hdac1;

uint32_t HAL_GetTick(void);
uint32_t HAL_RCC_GetPCLK2Freq(void);
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *timer,
		uint32_t event);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer);
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *timer);
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef *dac, uint32_t channel);
HAL_StatusTypeDef HAL_DAC_Stop(DAC_HandleTypeDef *dac, uint32_t channel);
HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef *dac, uint32_t channel,
		uint32_t alignment, uint32_t value);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, uint32_t state);

/* The planner/track host tests do not use HAL; keep the IRQ wrappers visible
 * so the same planner source is compiled without a target SDK. */
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)

#endif
