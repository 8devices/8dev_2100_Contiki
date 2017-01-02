/*
 * board.h
 *
 *  Created on: 2016 gruod. 15
 *      Author: Giedrius
 */

#ifndef BSP_8DEV_2100_0003_INC_BOARD_H_
#define BSP_8DEV_2100_0003_INC_BOARD_H_

#include "stm32l4xx.h"

typedef enum
{
  LED2 = 0
}Led_TypeDef;

typedef enum
{
  BUTTON_USER = 0,
  /* Alias */
  BUTTON_KEY  = BUTTON_USER
} Button_TypeDef;

typedef enum
{
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
}ButtonMode_TypeDef;


#define LEDn                                    1

#define LED2_PIN                                GPIO_PIN_5
#define LED2_GPIO_PORT                          GPIOB
#define LED2_GPIO_CLK_ENABLE()                  __GPIOB_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()                 __GPIOB_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)         LED2_GPIO_CLK_ENABLE()
#define LEDx_GPIO_CLK_DISABLE(__INDEX__)        LED2_GPIO_CLK_DISABLE()


#define BUTTONn                                 1

#define USER_BUTTON_PIN                         GPIO_PIN_3
#define USER_BUTTON_GPIO_PORT                   GPIOH
#define USER_BUTTON_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOH_CLK_ENABLE()
#define USER_BUTTON_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOH_CLK_DISABLE()
#define USER_BUTTON_EXTI_LINE                   GPIO_PIN_3
#define USER_BUTTON_EXTI_IRQn                   EXTI3_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)       USER_BUTTON_GPIO_CLK_ENABLE()
#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)      USER_BUTTON_GPIO_CLK_DISABLE()

#endif /* BSP_8DEV_2100_0003_INC_BOARD_H_ */
