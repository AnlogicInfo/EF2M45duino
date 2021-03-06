/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 * Copyright (c) 2011, 2012 LeafLabs, LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*****************************************************************************/

/**
 * @file libmaple/include/libmaple/gpio.h
 * @brief General Purpose I/O (GPIO) interace.
 */

#ifndef _LIBMAPLE_GPIO_H_
#define _LIBMAPLE_GPIO_H_

#ifdef __cplusplus
extern "C"{
#endif

/*
 * Note: Series header must define:
 * - enum gpio_pin_mode (TODO think harder about portability here)
 */

// roger clark. replaced with line below #include <series/gpio.h>
#include "stm32f1/include/series/gpio.h"
#include <libmaple/libmaple_types.h>
#include <libmaple/rcc.h>
#include <libmaple/exti.h>

/*
 * Device type
 */

/** GPIO device type */
typedef struct gpio_dev {
    gpio_reg_map *regs;         /**< Register map */
    rcc_clk_id    clk_id;       /**< RCC clock information */
    /**
     * @brief (Deprecated) External interrupt port.
     * Instead of dev->exti_port, use gpio_exti_port(dev).
     */
    exti_cfg      exti_port;
} gpio_dev;

/*
 * Portable routines
 */

void gpio_init(gpio_dev *dev);
void gpio_init_all(void);
/* TODO flags argument version? */
void gpio_set_mode(gpio_dev *dev, uint8 pin, gpio_pin_mode mode);
gpio_pin_mode gpio_get_mode(gpio_dev *dev, uint8 pin);
/**
 * @brief Get a GPIO port's corresponding EXTI port configuration.
 * @param dev GPIO port whose exti_cfg to return.
 */
static inline exti_cfg gpio_exti_port(gpio_dev *dev) {
    return (exti_cfg)(EXTI_PA + (dev->clk_id - RCC_GPIOA));
}

/**
 * Set or reset a GPIO pin.
 *
 * Pin must have previously been configured to output mode.
 *
 * @param dev GPIO device whose pin to set.
 * @param pin Pin on to set or reset
 * @param val If true, set the pin.  If false, reset the pin.
 */
static inline void gpio_write_bit(gpio_dev *dev, uint8 pin, uint8 val) {
    val = !val;          /* "set" bits are lower than "reset" bits  */
    dev->regs->BSRR = (1U << pin) << (16 * val);
}

/**
 * Determine whether or not a GPIO pin is set.
 *
 * Pin must have previously been configured to input mode.
 *
 * @param dev GPIO device whose pin to test.
 * @param pin Pin on dev to test.
 * @return True if the pin is set, false otherwise.
 */
static inline uint32 gpio_read_bit(gpio_dev *dev, uint8 pin) {
    return dev->regs->IDR & (1U << pin);
}

/**
 * Toggle a pin configured as output push-pull.
 * @param dev GPIO device.
 * @param pin Pin on dev to toggle.
 */
static inline void gpio_toggle_bit(gpio_dev *dev, uint8 pin) {
    dev->regs->ODR = dev->regs->ODR ^ (1U << pin);
}


typedef enum
{
    gpio0_0 = 0,
    gpio0_1,
    gpio0_2,
    gpio0_3,
    gpio0_4,
    gpio0_5,
    gpio0_6,
    gpio0_7,
    gpio0_8,
    gpio0_9,
    gpio0_10,
    gpio0_11,
    gpio0_12,
    gpio0_13,
    gpio0_14,
    gpio0_15,
    gpio1_0,
    gpio1_1,
    gpio1_2,
    gpio1_3,
    gpio1_4,
    gpio1_5,
    gpio1_6,
    gpio1_7,
    gpio1_8,
    gpio1_9,
    gpio1_10,
    gpio1_11,
    gpio1_12,
    gpio1_13,
    gpio1_14,
    gpio1_15,
    gpioMax = 32
}GPIO_Num;

/**
 * @brief enum of GPIO Direction.
 */
typedef enum
{
    gpio_Input = 0,
    gpio_Output
}GPIO_Dir;

/**
 * @brief enum of Trigger type
 */
typedef enum
{
	gpio_Low_Level = 0,
	gpio_High_Level,
	gpio_Falling_Edge,
	gpio_Rising_Edge
}GPIO_TrigType;

/**
 * @brief enum of gpio debounce. GPIO Debounce is only useful when gpio direction is input.
 */
typedef enum
{
    gpio_NoDeb = 0,
    gpio_Deb
}GPIO_Debounce;

/**
 * @brief enum of gpio value.
 */
typedef enum
{
    gpio_DefaultValue = 0,
    gpio_Low = 0,
    gpio_High
}GPIO_Value;

/**
 * @brief This is the struct to initialize the gpio module.
 */
typedef struct
{
    GPIO_Dir dir;    
    GPIO_Debounce debounce;
    GPIO_Value value;
    GPIO_TrigType trig_type;
}GPIO_InitTypeDef;

/**
 * @brief Set the GPIO trigger type.
 * @param  n: the GPIO_Num to define which GPIO to operate.
 * @param  type: the GPIO_TrigType, choosing from enum GPIO_TrigType
 * @return This function has no return.
 */
void HAL_GPIO_TrigType(GPIO_Num n, GPIO_TrigType type);
/**
 * @brief Set the GPIO to trigger a interrupt at both edge, rising and falling.
 * @param  n: the GPIO_Num to define which GPIO to operate.
 * @param  enable: enable or disable whether a gpio trigger an interrupt at both edge.
 * @return This function has no return.
 */
void HAL_GPIO_TrigBothEdge(GPIO_Num n, uint8_t enable);
/**
 * @brief GPIO Initialize.
 * @param  n: the GPIO_Num to define which GPIO to operate.
 * @param  config: a struct to initialize a GPIO.
 * @return This function has no return.
 */
void HAL_GPIO_Init(GPIO_Num n, GPIO_InitTypeDef config);
/**
 * @brief Set GPIO Direction, Input or Output
 * @param  n: the GPIO_Num to define which GPIO to operate.
 * @param  dir: choose from GPIO_Direction, gpio_Input or gpio_Output
 * @return This function has no return.
 */
void HAL_GPIO_SetDir(GPIO_Num n, GPIO_Dir dir);
/**
 * @brief Get GPIO Direction, Input or Output
 * @param  n: the GPIO_Num to define which GPIO to operate.
 * @return return the direction of the specific gpio
 */
GPIO_Dir HAL_GPIO_GetDir(GPIO_Num n);
/**
 * @brief Enable the interrupt of specific GPIO
 * @param  n: the GPIO_Num to define which GPIO to operate. n can only choose from gpio0_0 to gpio0_15
 * @return This function has no return value.
 */
void HAL_GPIO_IntEnable(GPIO_Num n);
/**
 * @brief Disable the interrupt of specific GPIO
 * @param  n: the GPIO_Num to define which GPIO to operate. n can only choose from gpio0_0 to gpio0_15
 * @return This function has no return value.
 */
void HAL_GPIO_IntDisable(GPIO_Num n);
/**
 * @brief When GPIO direction is output, write value to set gpio level.
 * @param  n: the GPIO_Num to define which GPIO to operate.
 * @param  value: value can be choose from GPIO_Value, gpio_Low or gpio_High
 * @return This function has no return value.
 */
void HAL_GPIO_WritePin(GPIO_Num n, GPIO_Value value);
/**
 * @brief When GPIO direction is input, read current gpio level.
 * @param  n: the GPIO_Num to define which GPIO to operate.
 * @return return the result of current gpio level.
 */
GPIO_Value HAL_GPIO_ReadPin(GPIO_Num n);
/**
 * @brief Toggle a gpio pin
 * @param  n: the GPIO_Num to define which GPIO to operate.
 * @return This function has no return value.
 */
void HAL_GPIO_TogglePin(GPIO_Num n);
/**
 * @brief Mask the interrupt of specific GPIO, when the interrupt is masked, no interrupt will trigger to CPU
 * @param  n: the GPIO_Num to define which GPIO to operate. n can only choose from gpio0_0 to gpio0_15
 * @return This function has no return value.
 */
void HAL_GPIO_MaskIrq(GPIO_Num n);
/**
 * @brief Unmask the interrupt of specific GPIO, when the interrupt is unmasked, the interrupt will trigger to CPU
 * @param  n: the GPIO_Num to define which GPIO to operate. n can only choose from gpio0_0 to gpio0_15
 * @return This function has no return value.
 */
void HAL_GPIO_UnmaskIrq(GPIO_Num n);
/**
 * @brief Get all the interrupt status of gpio
 * @return return all the interrupt status of gpio ranging from ggpio0_0 to gpio0_15
 */
uint32_t HAL_GPIO_IntStatus(void);
/**
 * @brief Clear the interrupt of specific gpio
 * @param  n: the GPIO_Num to define which GPIO to operate. n can only choose from gpio0_0 to gpio0_15
 * @return This function has no return value.
 */
void HAL_GPIO_ClrIrq(GPIO_Num n);



#ifdef __cplusplus
}
#endif

#endif
