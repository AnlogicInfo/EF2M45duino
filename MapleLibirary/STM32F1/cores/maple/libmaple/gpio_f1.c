/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
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
 * @file libmaple/stm32f1/gpio.c
 * @brief STM32F1 GPIO support.
 */

#include <libmaple/gpio.h>
#include <libmaple/rcc.h>

/*
 * GPIO devices
 */

gpio_dev gpioa = {
    .regs      = GPIOA_BASE,
    .clk_id    = RCC_GPIOA,
    .exti_port = EXTI_PA,
};
/** GPIO port A device. */
gpio_dev* const GPIOA = &gpioa;

gpio_dev gpiob = {
    .regs      = GPIOB_BASE,
    .clk_id    = RCC_GPIOB,
    .exti_port = EXTI_PB,
};
/** GPIO port B device. */
gpio_dev* const GPIOB = &gpiob;

gpio_dev gpioc = {
    .regs      = GPIOC_BASE,
    .clk_id    = RCC_GPIOC,
    .exti_port = EXTI_PC,
};
/** GPIO port C device. */
gpio_dev* const GPIOC = &gpioc;

gpio_dev gpiod = {
    .regs      = GPIOD_BASE,
    .clk_id    = RCC_GPIOD,
    .exti_port = EXTI_PD,
};
/** GPIO port D device. */
gpio_dev* const GPIOD = &gpiod;

#if STM32_NR_GPIO_PORTS > 4
gpio_dev gpioe = {
    .regs      = GPIOE_BASE,
    .clk_id    = RCC_GPIOE,
    .exti_port = EXTI_PE,
};
/** GPIO port E device. */
gpio_dev* const GPIOE = &gpioe;

gpio_dev gpiof = {
    .regs      = GPIOF_BASE,
    .clk_id    = RCC_GPIOF,
    .exti_port = EXTI_PF,
};
/** GPIO port F device. */
gpio_dev* const GPIOF = &gpiof;

gpio_dev gpiog = {
    .regs      = GPIOG_BASE,
    .clk_id    = RCC_GPIOG,
    .exti_port = EXTI_PG,
};
/** GPIO port G device. */
gpio_dev* const GPIOG = &gpiog;
#endif

/*
 * GPIO routines
 */

/**
 * Initialize and reset all available GPIO devices.
 */
void gpio_init_all(void) {
    gpio_init(GPIOA);
    gpio_init(GPIOB);
    gpio_init(GPIOC);
    gpio_init(GPIOD);
#if STM32_NR_GPIO_PORTS > 4
    gpio_init(GPIOE);
    gpio_init(GPIOF);
    gpio_init(GPIOG);
#endif
}

/**
 * Set the mode of a GPIO pin.
 *
 * @param dev GPIO device.
 * @param pin Pin on the device whose mode to set, 0--15.
 * @param mode General purpose or alternate function mode to set the pin to.
 * @see gpio_pin_mode
 */
void gpio_set_mode(gpio_dev *dev, uint8 pin, gpio_pin_mode mode) {
    gpio_reg_map *regs = dev->regs;
    __IO uint32 *cr = &regs->CRL + (pin >> 3);
    uint32 shift = (pin & 0x7) * 4;
    uint32 tmp = *cr;

    tmp &= ~(0xF << shift);
    tmp |= (mode == GPIO_INPUT_PU ? GPIO_INPUT_PD : mode) << shift;
    *cr = tmp;

    if (mode == GPIO_INPUT_PD) {
        regs->ODR &= ~(1U << pin);
    } else if (mode == GPIO_INPUT_PU) {
        regs->ODR |= (1U << pin);
    }
}

gpio_pin_mode gpio_get_mode(gpio_dev *dev, uint8 pin) {
    gpio_reg_map *regs = dev->regs;
    __IO uint32 *cr = &regs->CRL + (pin >> 3);
    uint32 shift = (pin & 0x7) * 4;

	uint32 crMode = (*cr>>shift) & 0x0F;
	
	// could be pull up or pull down. Nee to check the ODR
	if (crMode==GPIO_INPUT_PD && ((regs->ODR >> pin) & 0x01) !=0 )
	{
		crMode = GPIO_INPUT_PU;
	}
	
    return(crMode);
}

/*
 * AFIO
 */

/**
 * @brief Initialize the AFIO clock, and reset the AFIO registers.
 */
void afio_init(void) {
    rcc_clk_enable(RCC_AFIO);
    rcc_reset_dev(RCC_AFIO);
}

#define AFIO_EXTI_SEL_MASK 0xF

/**
 * @brief Perform an alternate function remap.
 * @param remapping Remapping to perform.
 */
void afio_remap(afio_remap_peripheral remapping) {
    if (remapping & AFIO_REMAP_USE_MAPR2) {
        remapping &= ~AFIO_REMAP_USE_MAPR2;
        AFIO_BASE->MAPR2 |= remapping;
    } else {
        AFIO_BASE->MAPR |= remapping;
    }
}



//#define GPIO_PWIDTH_A 16
//
//typedef enum
//{
//    gpio_Level = 0,
//    gpio_Edge
//}GPIO_IrqLvl;
//
//typedef enum
//{
//    gpio_Low_Falling = 0,
//    gpio_High_Rising
//}GPIO_Ply;
//
//
//
//static void HAL_GPIO_SetPly(GPIO_Num n, GPIO_Ply ply)
//{
//    uint32_t polarity = gpio_int_polarity_getf();
//
//    if(gpio_High_Rising == ply){
//        polarity |= (1 << n);
//    }else{
//        polarity &= ~(1 << n);
//    }
//
//    gpio_int_polarity_setf(polarity);
//}
//static void HAL_GPIO_setIrqLevel(GPIO_Num n, GPIO_IrqLvl irqLvl)
//{
//    uint32_t irqLevel = gpio_int_level_getf();
//
//    if(gpio_Edge== irqLvl){
//        irqLevel |= (1 << n);
//    }else{
//        irqLevel &= ~(1 << n);
//    }
//    gpio_int_level_setf(irqLevel);
//}
//
//
///**
// * @brief Set the GPIO trigger type.
// * @param  n: the GPIO_Num to define which GPIO to operate.
// * @param  type: the GPIO_TrigType, choosing from enum GPIO_TrigType
// * @return This function has no return value.
// */
//void HAL_GPIO_TrigType(GPIO_Num n, GPIO_TrigType type)
//{
//	switch(type)
//	{
//		case gpio_Low_Level:
//			HAL_GPIO_setIrqLevel(n, gpio_Level);
//			HAL_GPIO_SetPly(n, gpio_Low_Falling);
//			break;
//		case gpio_High_Level:
//			HAL_GPIO_setIrqLevel(n, gpio_Level);
//			HAL_GPIO_SetPly(n, gpio_High_Rising);
//			break;
//		case gpio_Falling_Edge:
//			HAL_GPIO_setIrqLevel(n, gpio_Edge);
//			HAL_GPIO_SetPly(n, gpio_Low_Falling);
//			break;
//		case gpio_Rising_Edge:
//			HAL_GPIO_setIrqLevel(n, gpio_Edge);
//			HAL_GPIO_SetPly(n, gpio_High_Rising);
//			break;
//	}
//}
//
///**
// * @brief Set the GPIO to trigger a interrupt at both edge, rising and falling.
// * @param  n: the GPIO_Num to define which GPIO to operate.
// * @param  enable: enable or disable whether a gpio trigger an interrupt at both edge.
// * @return This function has no return value.
// */
//void HAL_GPIO_TrigBothEdge(GPIO_Num n, uint8_t enable)
//{
//	uint32_t reg = gpio_int_both_edge_getf();
//	if(enable)
//		gpio_int_both_edge_setf(reg | (1<<n));
//	else
//		gpio_int_both_edge_setf(reg & (~(1<<n)));
//}
//
///**
// * @brief Controls whether an external signal that is the source of an interrupt needs to be debounced to remove any spurious glitches.
// * @param  n: the GPIO_Num to define which GPIO to operate.
// * @return This function has no return value.
// */
//void HAL_GPIO_SetDebounce(GPIO_Num n, GPIO_Debounce debounce)
//{
//    uint32_t debReg = gpio_debounceenable_getf();
//
//    if(debounce == gpio_Deb)
//    {
//        debReg |= (1 << n);
//    }
//    else
//    {
//        debReg &= ~(1 << n);
//    }
//    gpio_debounceenable_setf(debReg);
//}
//
///**
// * @brief GPIO Initialize.
// * @param  n: the GPIO_Num to define which GPIO to operate.
// * @param  config: a struct to initialize a GPIO.
// * @return This function has no return value.
// */
//void HAL_GPIO_Init(GPIO_Num n, GPIO_InitTypeDef config)
//{
//    HAL_GPIO_SetDir(n, config.dir);
//    if(gpio_Output == config.dir)
//	{
//        HAL_GPIO_WritePin(n, config.value);
//    }
//    else
//	{
//		HAL_GPIO_SetDebounce(n, config.debounce);
//		HAL_GPIO_TrigType(n,config.trig_type);
//	}
//}
//
///**
// * @brief Set GPIO Direction, Input or Output
// * @param  n: the GPIO_Num to define which GPIO to operate.
// * @param  dir: choose from GPIO_Direction, gpio_Input or gpio_Output
// * @return This function has no return value.
// */
//void HAL_GPIO_SetDir(GPIO_Num n, GPIO_Dir dir)
//{
//    uint32_t dirReg;
//    
//    if(n<GPIO_PWIDTH_A){
//        dirReg = gpio_portadatadirectionregister_getf();
//        if(gpio_Output == dir){
//            dirReg |= 1<<n;
//        }else{
//            dirReg &= ~(1<<n);
//        }
//        gpio_portadatadirectionregister_setf(dirReg);
//    }else{
//        dirReg = gpio_portbdatadirectionregister_getf();
//        if(gpio_Output == dir){
//            dirReg |= 1<<(n-GPIO_PWIDTH_A);
//        }else{
//            dirReg &= ~(1<<(n-GPIO_PWIDTH_A));
//        }
//        gpio_portbdatadirectionregister_setf(dirReg);
//    }
//}
//
///**
// * @brief Get GPIO Direction, Input or Output
// * @param  n: the GPIO_Num to define which GPIO to operate.
// * @return return the direction of the specific gpio
// */
//GPIO_Dir HAL_GPIO_GetDir(GPIO_Num n)
//{
//	uint32_t dir;
//	if(n<GPIO_PWIDTH_A){
//        dir = (gpio_portadatadirectionregister_getf()>>n)&0x1;
//    }else{
//        dir = (gpio_portbdatadirectionregister_getf()>>(n-GPIO_PWIDTH_A))&0x1;
//    }
//    return (GPIO_Dir)dir;
//}
//
///**
// * @brief Enable the interrupt of specific GPIO
// * @param  n: the GPIO_Num to define which GPIO to operate. n can only choose from gpio0_0 to gpio0_15
// * @return This function has no return value.
// */
//void HAL_GPIO_IntEnable(GPIO_Num n)
//{
//    uint32_t irqEnableReg = gpio_interruptenable_getf();
//
//    irqEnableReg |= (1 << n);
//    gpio_interruptenable_setf(irqEnableReg);
//}
//
///**
// * @brief Disable the interrupt of specific GPIO
// * @param  n: the GPIO_Num to define which GPIO to operate. n can only choose from gpio0_0 to gpio0_15
// * @return This function has no return value.
// */
//void HAL_GPIO_IntDisable(GPIO_Num n)
//{
//    uint32_t irqEnableReg = gpio_interruptenable_getf();
//
//    irqEnableReg &= ~(1 << n);
//    gpio_interruptenable_setf(irqEnableReg);    
//}
//
///**
// * @brief When GPIO direction is output, write value to set gpio level.
// * @param  n: the GPIO_Num to define which GPIO to operate.
// * @param  value: value can be choose from GPIO_Value, gpio_Low or gpio_High
// * @return This function has no return value.
// */
//void HAL_GPIO_WritePin(GPIO_Num n, GPIO_Value value)
//{
//    uint32_t valueReg;
//    
//    if(n<GPIO_PWIDTH_A){
//        valueReg = gpio_portadataregister_getf();
//        if(gpio_Low == value){
//            valueReg &= ~(1 << n);
//        }else{
//            valueReg |= (1 << n);
//        }
//        gpio_portadataregister_setf(valueReg);
//    }else{
//        valueReg = gpio_portbdataregister_getf();
//        if(gpio_Low == value){
//            valueReg &= ~(1 << (n-GPIO_PWIDTH_A));
//        }else{
//            valueReg |= (1 << (n-GPIO_PWIDTH_A));
//        }
//        gpio_portbdataregister_setf(valueReg);
//    }
//}
//
///**
// * @brief When GPIO direction is input, read current gpio level.
// * @param  n: the GPIO_Num to define which GPIO to operate.
// * @return return the result of current gpio level.
// */
//GPIO_Value HAL_GPIO_ReadPin(GPIO_Num n)
//{
//    uint32_t value;
//
//    if(n<GPIO_PWIDTH_A){
//        value = (gpio_ext_porta_getf()>>n)&0x1;
//    }else{
//        value = (gpio_ext_portb_getf()>>(n-GPIO_PWIDTH_A))&0x1;
//    }
//    return (GPIO_Value)value;
//}
//
///**
// * @brief Toggle a gpio pin
// * @param  n: the GPIO_Num to define which GPIO to operate.
// * @return This function has no return value.
// */
//void HAL_GPIO_TogglePin(GPIO_Num n)
//{
//    uint32_t value;
//    uint32_t valueReg;
//    if(n<GPIO_PWIDTH_A){
//        value = (gpio_ext_porta_getf()>>n)&0x1;
//
//        valueReg = (gpio_portadataregister_getf() & (~(1<<n))) | ((~value)<<n);
//		
//        gpio_portadataregister_setf(valueReg);
//    }else{
//        value = (gpio_ext_portb_getf()>>(n-GPIO_PWIDTH_A))&0x1;
//
//        valueReg = (gpio_portbdataregister_getf() & (~(1<<(n-GPIO_PWIDTH_A)))) | ((~value)<<(n-GPIO_PWIDTH_A));
//		
//        gpio_portbdataregister_setf(valueReg);
//    }
//}
//
///**
// * @brief Mask the interrupt of specific GPIO, when the interrupt is masked, no interrupt will trigger to CPU
// * @param  n: the GPIO_Num to define which GPIO to operate. n can only choose from gpio0_0 to gpio0_15
// * @return This function has no return value.
// */
//void HAL_GPIO_MaskIrq(GPIO_Num n)
//{
//    uint32_t mask = gpio_int_mask_getf();
//
//    mask |= (1 << n);
//
//    gpio_int_mask_setf(mask);
//}
//
///**
// * @brief Unmask the interrupt of specific GPIO, when the interrupt is unmasked, the interrupt will trigger to CPU
// * @param  n: the GPIO_Num to define which GPIO to operate. n can only choose from gpio0_0 to gpio0_15
// * @return This function has no return value.
// */
//void HAL_GPIO_UnmaskIrq(GPIO_Num n)
//{
//    uint32_t mask = gpio_int_mask_getf();
//
//    mask &= ~(1 << n);
//
//    gpio_int_mask_setf(mask);
//}
//
///**
// * @brief Get all the interrupt status of gpio
// * @return return all the interrupt status of gpio ranging from ggpio0_0 to gpio0_15
// */
//uint32_t HAL_GPIO_IntStatus(void)
//{
//	return gpio_int_status_getf();
//}
//
///**
// * @brief Clear the interrupt of specific gpio
// * @param  n: the GPIO_Num to define which GPIO to operate. n can only choose from gpio0_0 to gpio0_15
// * @return This function has no return value.
// */
//void HAL_GPIO_ClrIrq(GPIO_Num n)
//{
//    gpio_gpio_porta_eoi_set(1<<n);
//}