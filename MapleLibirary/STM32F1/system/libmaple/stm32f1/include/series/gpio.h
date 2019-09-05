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
 * @file libmaple/stm32f1/include/series/gpio.h
 * @brief STM32F1 GPIO and AFIO support.
 * General purpose I/O (GPIO) and Alternate Function I/O (AFIO).
 */


#ifndef _LIBMAPLE_STM32F1_GPIO_H_
#define _LIBMAPLE_STM32F1_GPIO_H_

#ifdef __cplusplus
extern "C"{
#endif

#include <libmaple/stm32.h>
#include <libmaple/libmaple_types.h>
#include <libmaple/exti.h>

/*
 * GPIO register maps and devices
 */

/** GPIO register map type */
typedef struct gpio_reg_map {
    __IO uint32 CRL;      /**< Port configuration register low */
    __IO uint32 CRH;      /**< Port configuration register high */
    __IO uint32 IDR;      /**< Port input data register */
    __IO uint32 ODR;      /**< Port output data register */
    __IO uint32 BSRR;     /**< Port bit set/reset register */
    __IO uint32 BRR;      /**< Port bit reset register */
    __IO uint32 LCKR;     /**< Port configuration lock register */
} gpio_reg_map;


struct gpio_dev;
extern struct gpio_dev gpioa;
extern struct gpio_dev* const GPIOA;
extern struct gpio_dev gpiob;
extern struct gpio_dev* const GPIOB;
extern struct gpio_dev gpioc;
extern struct gpio_dev* const GPIOC;
extern struct gpio_dev gpiod;
extern struct gpio_dev* const GPIOD;
#if STM32_NR_GPIO_PORTS > 4
extern struct gpio_dev gpioe;
extern struct gpio_dev* const GPIOE;
extern struct gpio_dev gpiof;
extern struct gpio_dev* const GPIOF;
extern struct gpio_dev gpiog;
extern struct gpio_dev* const GPIOG;
#endif

/** GPIO port A register map base pointer */
#define GPIOA_BASE                      ((struct gpio_reg_map*)0x40010800)
/** GPIO port B register map base pointer */
#define GPIOB_BASE                      ((struct gpio_reg_map*)0x40010C00)
/** GPIO port C register map base pointer */
#define GPIOC_BASE                      ((struct gpio_reg_map*)0x40011000)
/** GPIO port D register map base pointer */
#define GPIOD_BASE                      ((struct gpio_reg_map*)0x40011400)
/** GPIO port E register map base pointer */
#define GPIOE_BASE                      ((struct gpio_reg_map*)0x40011800)
/** GPIO port F register map base pointer */
#define GPIOF_BASE                      ((struct gpio_reg_map*)0x40011C00)
/** GPIO port G register map base pointer */
#define GPIOG_BASE                      ((struct gpio_reg_map*)0x40012000)

/*
 * GPIO register bit definitions
 */

/* Control registers, low and high */

#define GPIO_CR_CNF                     (0x3 << 2)
#define GPIO_CR_CNF_INPUT_ANALOG        (0x0 << 2)
#define GPIO_CR_CNF_INPUT_FLOATING      (0x1 << 2)
#define GPIO_CR_CNF_INPUT_PU_PD         (0x2 << 2)
#define GPIO_CR_CNF_OUTPUT_PP           (0x0 << 2)
#define GPIO_CR_CNF_OUTPUT_OD           (0x1 << 2)
#define GPIO_CR_CNF_AF_OUTPUT_PP        (0x2 << 2)
#define GPIO_CR_CNF_AF_OUTPUT_OD        (0x3 << 2)
#define GPIO_CR_MODE                    0x3
#define GPIO_CR_MODE_INPUT              0x0
#define GPIO_CR_MODE_OUTPUT_10MHZ       0x1
#define GPIO_CR_MODE_OUTPUT_2MHZ        0x2
#define GPIO_CR_MODE_OUTPUT_50MHZ       0x3

/**
 * @brief GPIO pin modes.
 *
 * These only allow for 50MHZ max output speeds; if you want slower,
 * use direct register access.
 */
typedef enum gpio_pin_mode {
    /** Output push-pull. */
    GPIO_OUTPUT_PP      = GPIO_CR_CNF_OUTPUT_PP | GPIO_CR_MODE_OUTPUT_50MHZ,
    /** Output open-drain. */
    GPIO_OUTPUT_OD      = GPIO_CR_CNF_OUTPUT_OD | GPIO_CR_MODE_OUTPUT_50MHZ,
    /** Alternate function output push-pull. */
    GPIO_AF_OUTPUT_PP   = GPIO_CR_CNF_AF_OUTPUT_PP | GPIO_CR_MODE_OUTPUT_50MHZ,
    /** Alternate function output open drain. */
    GPIO_AF_OUTPUT_OD   = GPIO_CR_CNF_AF_OUTPUT_OD | GPIO_CR_MODE_OUTPUT_50MHZ,
    /** Analog input. */
    GPIO_INPUT_ANALOG   = GPIO_CR_CNF_INPUT_ANALOG | GPIO_CR_MODE_INPUT,
    /** Input floating. */
    GPIO_INPUT_FLOATING = GPIO_CR_CNF_INPUT_FLOATING | GPIO_CR_MODE_INPUT,
    /** Input pull-down. */
    GPIO_INPUT_PD       = GPIO_CR_CNF_INPUT_PU_PD | GPIO_CR_MODE_INPUT,
    /** Input pull-up. */
    GPIO_INPUT_PU, /* (treated a special case, for ODR twiddling) */
} gpio_pin_mode;

/* Hacks for F2: */
#define GPIO_MODE_ANALOG GPIO_INPUT_ANALOG
#define GPIO_MODE_OUTPUT GPIO_OUTPUT_PP

/*
 * AFIO register map
 */

/** AFIO register map */
typedef struct afio_reg_map {
    __IO uint32 EVCR;    /**< Event control register.  */
    __IO uint32 MAPR;    /**< AF remap and debug I/O configuration register. */
    __IO uint32 EXTICR1; /**< External interrupt configuration register 1. */
    __IO uint32 EXTICR2; /**< External interrupt configuration register 2. */
    __IO uint32 EXTICR3; /**< External interrupt configuration register 3. */
    __IO uint32 EXTICR4; /**< External interrupt configuration register 4. */
    __IO uint32 MAPR2;   /**<
                          * AF remap and debug I/O configuration register 2. */
} afio_reg_map;

/** AFIO register map base pointer. */
#define AFIO_BASE                       ((struct afio_reg_map *)0x40010000)

/*
 * AFIO register bit definitions
 */

/* Event control register */

#define AFIO_EVCR_EVOE                  (0x1 << 7)
#define AFIO_EVCR_PORT_PA               (0x0 << 4)
#define AFIO_EVCR_PORT_PB               (0x1 << 4)
#define AFIO_EVCR_PORT_PC               (0x2 << 4)
#define AFIO_EVCR_PORT_PD               (0x3 << 4)
#define AFIO_EVCR_PORT_PE               (0x4 << 4)
#define AFIO_EVCR_PIN_0                 0x0
#define AFIO_EVCR_PIN_1                 0x1
#define AFIO_EVCR_PIN_2                 0x2
#define AFIO_EVCR_PIN_3                 0x3
#define AFIO_EVCR_PIN_4                 0x4
#define AFIO_EVCR_PIN_5                 0x5
#define AFIO_EVCR_PIN_6                 0x6
#define AFIO_EVCR_PIN_7                 0x7
#define AFIO_EVCR_PIN_8                 0x8
#define AFIO_EVCR_PIN_9                 0x9
#define AFIO_EVCR_PIN_10                0xA
#define AFIO_EVCR_PIN_11                0xB
#define AFIO_EVCR_PIN_12                0xC
#define AFIO_EVCR_PIN_13                0xD
#define AFIO_EVCR_PIN_14                0xE
#define AFIO_EVCR_PIN_15                0xF

/* AF remap and debug I/O configuration register */

#define AFIO_MAPR_SWJ_CFG                      (0x7 << 24)
#define AFIO_MAPR_SWJ_CFG_FULL_SWJ             (0x0 << 24)
#define AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_NJRST    (0x1 << 24)
#define AFIO_MAPR_SWJ_CFG_NO_JTAG_SW           (0x2 << 24)
#define AFIO_MAPR_SWJ_CFG_NO_JTAG_NO_SW        (0x4 << 24)
#define AFIO_MAPR_ADC2_ETRGREG_REMAP           (1U << 20)
#define AFIO_MAPR_ADC2_ETRGINJ_REMAP           (1U << 19)
#define AFIO_MAPR_ADC1_ETRGREG_REMAP           (1U << 18)
#define AFIO_MAPR_ADC1_ETRGINJ_REMAP           (1U << 17)
#define AFIO_MAPR_TIM5CH4_IREMAP               (1U << 16)
#define AFIO_MAPR_PD01_REMAP                   (1U << 15)
#define AFIO_MAPR_CAN_REMAP                    (0x3 << 13)
#define AFIO_MAPR_CAN_REMAP_NONE               (0x0 << 13)
#define AFIO_MAPR_CAN_REMAP_PB8_PB9            (0x2 << 13)
#define AFIO_MAPR_CAN_REMAP_PD0_PD1            (0x3 << 13)
#define AFIO_MAPR_TIM4_REMAP                   (1U << 12)
#define AFIO_MAPR_TIM3_REMAP                   (0x3 << 10)
#define AFIO_MAPR_TIM3_REMAP_NONE              (0x0 << 10)
#define AFIO_MAPR_TIM3_REMAP_PARTIAL           (0x2 << 10)
#define AFIO_MAPR_TIM3_REMAP_FULL              (0x3 << 10)
#define AFIO_MAPR_TIM2_REMAP                   (0x3 << 8)
#define AFIO_MAPR_TIM2_REMAP_NONE              (0x0 << 8)
#define AFIO_MAPR_TIM2_REMAP_PA15_PB3_PA2_PA3  (0x1 << 8)
#define AFIO_MAPR_TIM2_REMAP_PA0_PA1_PB10_PB11 (0x2 << 8)
#define AFIO_MAPR_TIM2_REMAP_FULL              (0x3 << 8)
#define AFIO_MAPR_TIM1_REMAP                   (0x3 << 6)
#define AFIO_MAPR_TIM1_REMAP_NONE              (0x0 << 6)
#define AFIO_MAPR_TIM1_REMAP_PARTIAL           (0x1 << 6)
#define AFIO_MAPR_TIM1_REMAP_FULL              (0x3 << 6)
#define AFIO_MAPR_USART3_REMAP                 (0x3 << 4)
#define AFIO_MAPR_USART3_REMAP_NONE            (0x0 << 4)
#define AFIO_MAPR_USART3_REMAP_PARTIAL         (0x1 << 4)
#define AFIO_MAPR_USART3_REMAP_FULL            (0x3 << 4)
#define AFIO_MAPR_USART2_REMAP                 (1U << 3)
#define AFIO_MAPR_USART1_REMAP                 (1U << 2)
#define AFIO_MAPR_I2C1_REMAP                   (1U << 1)
#define AFIO_MAPR_SPI1_REMAP                   (1U << 0)

/* External interrupt configuration register 1 */

#define AFIO_EXTICR1_EXTI3              (0xF << 12)
#define AFIO_EXTICR1_EXTI3_PA           (0x0 << 12)
#define AFIO_EXTICR1_EXTI3_PB           (0x1 << 12)
#define AFIO_EXTICR1_EXTI3_PC           (0x2 << 12)
#define AFIO_EXTICR1_EXTI3_PD           (0x3 << 12)
#define AFIO_EXTICR1_EXTI3_PE           (0x4 << 12)
#define AFIO_EXTICR1_EXTI3_PF           (0x5 << 12)
#define AFIO_EXTICR1_EXTI3_PG           (0x6 << 12)
#define AFIO_EXTICR1_EXTI2              (0xF << 8)
#define AFIO_EXTICR1_EXTI2_PA           (0x0 << 8)
#define AFIO_EXTICR1_EXTI2_PB           (0x1 << 8)
#define AFIO_EXTICR1_EXTI2_PC           (0x2 << 8)
#define AFIO_EXTICR1_EXTI2_PD           (0x3 << 8)
#define AFIO_EXTICR1_EXTI2_PE           (0x4 << 8)
#define AFIO_EXTICR1_EXTI2_PF           (0x5 << 8)
#define AFIO_EXTICR1_EXTI2_PG           (0x6 << 8)
#define AFIO_EXTICR1_EXTI1              (0xF << 4)
#define AFIO_EXTICR1_EXTI1_PA           (0x0 << 4)
#define AFIO_EXTICR1_EXTI1_PB           (0x1 << 4)
#define AFIO_EXTICR1_EXTI1_PC           (0x2 << 4)
#define AFIO_EXTICR1_EXTI1_PD           (0x3 << 4)
#define AFIO_EXTICR1_EXTI1_PE           (0x4 << 4)
#define AFIO_EXTICR1_EXTI1_PF           (0x5 << 4)
#define AFIO_EXTICR1_EXTI1_PG           (0x6 << 4)
#define AFIO_EXTICR1_EXTI0              0xF
#define AFIO_EXTICR1_EXTI0_PA           0x0
#define AFIO_EXTICR1_EXTI0_PB           0x1
#define AFIO_EXTICR1_EXTI0_PC           0x2
#define AFIO_EXTICR1_EXTI0_PD           0x3
#define AFIO_EXTICR1_EXTI0_PE           0x4
#define AFIO_EXTICR1_EXTI0_PF           0x5
#define AFIO_EXTICR1_EXTI0_PG           0x6

/* External interrupt configuration register 2 */

#define AFIO_EXTICR2_EXTI7              (0xF << 12)
#define AFIO_EXTICR2_EXTI7_PA           (0x0 << 12)
#define AFIO_EXTICR2_EXTI7_PB           (0x1 << 12)
#define AFIO_EXTICR2_EXTI7_PC           (0x2 << 12)
#define AFIO_EXTICR2_EXTI7_PD           (0x3 << 12)
#define AFIO_EXTICR2_EXTI7_PE           (0x4 << 12)
#define AFIO_EXTICR2_EXTI7_PF           (0x5 << 12)
#define AFIO_EXTICR2_EXTI7_PG           (0x6 << 12)
#define AFIO_EXTICR2_EXTI6              (0xF << 8)
#define AFIO_EXTICR2_EXTI6_PA           (0x0 << 8)
#define AFIO_EXTICR2_EXTI6_PB           (0x1 << 8)
#define AFIO_EXTICR2_EXTI6_PC           (0x2 << 8)
#define AFIO_EXTICR2_EXTI6_PD           (0x3 << 8)
#define AFIO_EXTICR2_EXTI6_PE           (0x4 << 8)
#define AFIO_EXTICR2_EXTI6_PF           (0x5 << 8)
#define AFIO_EXTICR2_EXTI6_PG           (0x6 << 8)
#define AFIO_EXTICR2_EXTI5              (0xF << 4)
#define AFIO_EXTICR2_EXTI5_PA           (0x0 << 4)
#define AFIO_EXTICR2_EXTI5_PB           (0x1 << 4)
#define AFIO_EXTICR2_EXTI5_PC           (0x2 << 4)
#define AFIO_EXTICR2_EXTI5_PD           (0x3 << 4)
#define AFIO_EXTICR2_EXTI5_PE           (0x4 << 4)
#define AFIO_EXTICR2_EXTI5_PF           (0x5 << 4)
#define AFIO_EXTICR2_EXTI5_PG           (0x6 << 4)
#define AFIO_EXTICR2_EXTI4              0xF
#define AFIO_EXTICR2_EXTI4_PA           0x0
#define AFIO_EXTICR2_EXTI4_PB           0x1
#define AFIO_EXTICR2_EXTI4_PC           0x2
#define AFIO_EXTICR2_EXTI4_PD           0x3
#define AFIO_EXTICR2_EXTI4_PE           0x4
#define AFIO_EXTICR2_EXTI4_PF           0x5
#define AFIO_EXTICR2_EXTI4_PG           0x6

/* AF remap and debug I/O configuration register 2 */

#define AFIO_MAPR2_FSMC_NADV            (1U << 10)
#define AFIO_MAPR2_TIM14_REMAP          (1U << 9)
#define AFIO_MAPR2_TIM13_REMAP          (1U << 8)
#define AFIO_MAPR2_TIM11_REMAP          (1U << 7)
#define AFIO_MAPR2_TIM10_REMAP          (1U << 6)
#define AFIO_MAPR2_TIM9_REMAP           (1U << 5)

/*
 * AFIO convenience routines
 */

void afio_init(void);

/* HACK: Use upper bit to denote MAPR2, Bit 31 is reserved and
 * not used in either MAPR or MAPR2 */
#define AFIO_REMAP_USE_MAPR2            (1U << 31)

/**
 * @brief Available peripheral remaps.
 * @see afio_remap()
 */
typedef enum afio_remap_peripheral {
     /** ADC 2 external trigger regular conversion remapping */
    AFIO_REMAP_ADC2_ETRGREG   = AFIO_MAPR_ADC2_ETRGREG_REMAP,
     /** ADC 2 external trigger injected conversion remapping */
    AFIO_REMAP_ADC2_ETRGINJ   = AFIO_MAPR_ADC2_ETRGINJ_REMAP,
    /** ADC 1 external trigger regular conversion remapping */
    AFIO_REMAP_ADC1_ETRGREG   = AFIO_MAPR_ADC1_ETRGREG_REMAP,
    /** ADC 1 external trigger injected conversion remapping */
    AFIO_REMAP_ADC1_ETRGINJ   = AFIO_MAPR_ADC1_ETRGINJ_REMAP,
    /** Timer 5 channel 4 internal remapping */
    AFIO_REMAP_TIM5CH4_I      = AFIO_MAPR_TIM5CH4_IREMAP,
    /** Port D0/Port D1 mapping on OSC_IN/OSC_OUT */
    AFIO_REMAP_PD01           = AFIO_MAPR_PD01_REMAP,
    /** CAN alternate function remapping 1 (RX on PB8, TX on PB9) */
    AFIO_REMAP_CAN_1          = AFIO_MAPR_CAN_REMAP_PB8_PB9,
    /** CAN alternate function remapping 2 (RX on PD0, TX on PD1) */
    AFIO_REMAP_CAN_2          = AFIO_MAPR_CAN_REMAP_PD0_PD1,
    /** Timer 4 remapping */
    AFIO_REMAP_TIM4           = AFIO_MAPR_TIM4_REMAP,
    /** Timer 3 partial remapping */
    AFIO_REMAP_TIM3_PARTIAL   = AFIO_MAPR_TIM3_REMAP_PARTIAL,
    /** Timer 3 full remapping */
    AFIO_REMAP_TIM3_FULL      = AFIO_MAPR_TIM3_REMAP_FULL,
    /**
     * Timer 2 partial remapping 1 (CH1 and ETR on PA15, CH2 on PB3,
     * CH3 on                                      PA2, CH4 on PA3) */
    AFIO_REMAP_TIM2_PARTIAL_1 = AFIO_MAPR_TIM2_REMAP_PA15_PB3_PA2_PA3,
    /**
     * Timer 2 partial remapping 2 (CH1 and ETR on PA0, CH2 on PA1,
     * CH3 on                                      PB10, CH4 on PB11) */
    AFIO_REMAP_TIM2_PARTIAL_2 = AFIO_MAPR_TIM2_REMAP_PA0_PA1_PB10_PB11,
    /** Timer 2 full remapping */
    AFIO_REMAP_TIM2_FULL      = AFIO_MAPR_TIM2_REMAP_FULL,
     /** USART 3 part remapping */
    AFIO_REMAP_USART3_PARTIAL = AFIO_MAPR_USART3_REMAP_PARTIAL,    
    /** USART 2 remapping */
    AFIO_REMAP_USART2         = AFIO_MAPR_USART2_REMAP,
    /** USART 1 remapping */
    AFIO_REMAP_USART1         = AFIO_MAPR_USART1_REMAP,
    /** I2C 1 remapping */
    AFIO_REMAP_I2C1           = AFIO_MAPR_I2C1_REMAP,
    /** SPI 1 remapping */
    AFIO_REMAP_SPI1           = AFIO_MAPR_SPI1_REMAP,
    /** NADV signal not connected */
    AFIO_REMAP_FSMC_NADV      = AFIO_MAPR2_FSMC_NADV | AFIO_REMAP_USE_MAPR2,
    /** Timer 14 remapping */
    AFIO_REMAP_TIM14          = AFIO_MAPR2_TIM14_REMAP | AFIO_REMAP_USE_MAPR2,
    /** Timer 13 remapping */
    AFIO_REMAP_TIM13          = AFIO_MAPR2_TIM13_REMAP | AFIO_REMAP_USE_MAPR2,
    /** Timer 11 remapping */
    AFIO_REMAP_TIM11          = AFIO_MAPR2_TIM11_REMAP | AFIO_REMAP_USE_MAPR2,
    /** Timer 10 remapping */
    AFIO_REMAP_TIM10          = AFIO_MAPR2_TIM10_REMAP | AFIO_REMAP_USE_MAPR2,
    /** Timer 9 remapping */
    AFIO_REMAP_TIM9           = AFIO_MAPR2_TIM9_REMAP | AFIO_REMAP_USE_MAPR2,
} afio_remap_peripheral;

void afio_remap(afio_remap_peripheral p);

/**
 * @brief Debug port configuration
 *
 * Used to configure the behavior of JTAG and Serial Wire (SW) debug
 * ports and their associated GPIO pins.
 *
 * @see afio_cfg_debug_ports()
 */
typedef enum afio_debug_cfg {
    /** Full Serial Wire and JTAG debug */
    AFIO_DEBUG_FULL_SWJ          = AFIO_MAPR_SWJ_CFG_FULL_SWJ,
    /** Full Serial Wire and JTAG, but no NJTRST. */
    AFIO_DEBUG_FULL_SWJ_NO_NJRST = AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_NJRST,
    /** Serial Wire debug only (JTAG-DP disabled, SW-DP enabled) */
    AFIO_DEBUG_SW_ONLY           = AFIO_MAPR_SWJ_CFG_NO_JTAG_SW,
    /** No debug; all JTAG and SW pins are free for use as GPIOs. */
    AFIO_DEBUG_NONE              = AFIO_MAPR_SWJ_CFG_NO_JTAG_NO_SW,
} afio_debug_cfg;

/**
 * @brief Enable or disable the JTAG and SW debug ports.
 * @param config Desired debug port configuration
 * @see afio_debug_cfg
 */
static inline void afio_cfg_debug_ports(afio_debug_cfg config) {
    __IO uint32 *mapr = &AFIO_BASE->MAPR;
    *mapr = (*mapr & ~AFIO_MAPR_SWJ_CFG) | config;
}

/*
 * Deprecated bits
 */

/**
 * @brief Deprecated. Use exti_cfg instead.
 *
 * In previous versions of libmaple, exti_attach_interrupt() took an
 * afio_exti_port argument; afio_exti_port was also a member of struct
 * gpio_dev. This isn't portable, so we now use exti_cfg
 * instead. This typedef (and the macros AFIO_EXTI_PA, ...,
 * AFIO_EXTI_PG) exist to preserve backwards compatibility.
 */
typedef exti_cfg afio_exti_port;

/** Deprecated. Use EXTI_PA instead. */
#define AFIO_EXTI_PA EXTI_PA
/** Deprecated. Use EXTI_PB instead. */
#define AFIO_EXTI_PB EXTI_PB
/** Deprecated. Use EXTI_PC instead. */
#define AFIO_EXTI_PC EXTI_PC
/** Deprecated. Use EXTI_PD instead. */
#define AFIO_EXTI_PD EXTI_PD
/** Deprecated. Use EXTI_PE instead. */
#define AFIO_EXTI_PE EXTI_PE
/** Deprecated. Use EXTI_PF instead. */
#define AFIO_EXTI_PF EXTI_PF
/** Deprecated. Use EXTI_PG instead. */
#define AFIO_EXTI_PG EXTI_PG

/**
 * @brief Deprecated. Use exti_num instead.
 *
 * In previous versions of libmaple, exti_attach_interrupt() took an
 * afio_exti_num argument. This isn't portable, so we use exti_num
 * instead. This typedef (and the macros AFIO_EXTI_0, ...,
 * AFIO_EXTI_15) exist to preserve backwards compatibility.
 */
typedef exti_num afio_exti_num;

/** Deprecated. Use EXTI0 instead. */
#define AFIO_EXTI_0 EXTI0
/** Deprecated. Use EXTI1 instead. */
#define AFIO_EXTI_1 EXTI1
/** Deprecated. Use EXTI2 instead. */
#define AFIO_EXTI_2 EXTI2
/** Deprecated. Use EXTI3 instead. */
#define AFIO_EXTI_3 EXTI3
/** Deprecated. Use EXTI4 instead. */
#define AFIO_EXTI_4 EXTI4
/** Deprecated. Use EXTI5 instead. */
#define AFIO_EXTI_5 EXTI5
/** Deprecated. Use EXTI6 instead. */
#define AFIO_EXTI_6 EXTI6
/** Deprecated. Use EXTI7 instead. */
#define AFIO_EXTI_7 EXTI7
/** Deprecated. Use EXTI8 instead. */
#define AFIO_EXTI_8 EXTI8
/** Deprecated. Use EXTI9 instead. */
#define AFIO_EXTI_9 EXTI9
/** Deprecated. Use EXTI10 instead. */
#define AFIO_EXTI_10 EXTI10
/** Deprecated. Use EXTI11 instead. */
#define AFIO_EXTI_11 EXTI11
/** Deprecated. Use EXTI12 instead. */
#define AFIO_EXTI_12 EXTI12
/** Deprecated. Use EXTI13 instead. */
#define AFIO_EXTI_13 EXTI13
/** Deprecated. Use EXTI14 instead. */
#define AFIO_EXTI_14 EXTI14
/** Deprecated. Use EXTI15 instead. */
#define AFIO_EXTI_15 EXTI15

/**
 * @brief Deprecated. Use exti_select(exti, port) instead.
 */
static inline __always_inline void afio_exti_select(exti_num exti, exti_cfg port) {
    exti_select(exti, port);
}


//
//#define    REG_GPIO_BASE                  0x40007000 
//
//typedef union
//{
//    uint32_t val;
//    struct
//    {
//        uint32_t              portadataregister : 16; /*15: 0, Values written to this register are output on the I/O signals for
//Port A if the corresponding data direction bits for Port A are set
//to Output mode and the corresponding control bit for Port A is
//set to Software mode. The value read back is equal to the last
//value written to this register.*/
//        uint32_t                     reserved_0 : 16; /*31:16,                       reserved*/
//    } bit_field;
//} T_GPIO_GPIO_SWPORTA_DR;
//
////gpio_swporta_ddr
//typedef union
//{
//    uint32_t val;
//    struct
//    {
//        uint32_t     portadatadirectionregister : 16; /*15: 0, Values written to this register independently control the
//direction of the corresponding data bit in Port A. The default
//direction can be configured as input or output after system
//reset through the GPIO_DFLT_DIR_A parameter.
//0 每 Input (default)
//1 每 Output*/
//        uint32_t                     reserved_0 : 16; /*31:16,                       reserved*/
//    } bit_field;
//} T_GPIO_GPIO_SWPORTA_DDR;
//
////gpio_swportb_dr
//typedef union
//{
//    uint32_t val;
//    struct
//    {
//        uint32_t              portbdataregister : 16; /*15: 0, Values written to this register are output on the I/O signals for
//Port B if the corresponding data direction bits for Port B are set
//to Output mode and the corresponding control bit for Port B is
//set to Software mode. The value read back is equal to the last
//value written to this register.*/
//        uint32_t                     reserved_0 : 16; /*31:16,                       reserved*/
//    } bit_field;
//} T_GPIO_GPIO_SWPORTB_DR;
//
////gpio_swportb_ddr
//typedef union
//{
//    uint32_t val;
//    struct
//    {
//        uint32_t     portbdatadirectionregister : 16; /*15: 0, Values written to this register independently control the
//direction of the corresponding data bit in Port B. The default
//direction can be configured as input or output after system
//reset through the GPIO_DFLT_DIR_A parameter.
//0 每 Input (default)
//1 每 Output*/
//        uint32_t                     reserved_0 : 16; /*31:16,                       reserved*/
//    } bit_field;
//} T_GPIO_GPIO_SWPORTB_DDR;
//
////gpio_inten
//typedef union
//{
//    uint32_t val;
//    struct
//    {
//        uint32_t                interruptenable : 16; /*15: 0, Allows each bit of Port A to be configured for interrupts. By default the generation of interrupts is disabled. Whenever a 1 is written to a bit of this register, it configures the corresponding bit on Port A to become an interrupt; otherwise, Port A operates as a normal GPIO signal. Interrupts are disabled on the corresponding bits of Port A if the corresponding data direction register is set to Output or if Port A mode is set to Hardware.
//0 每 Configure Port A bit as normal GPIO signal (default)
//1 每 Configure Port A bit as interrupt*/
//        uint32_t                     reserved_0 : 16; /*31:16,                       reserved*/
//    } bit_field;
//} T_GPIO_GPIO_INTEN;
//
////gpio_intmask
//typedef union
//{
//    uint32_t val;
//    struct
//    {
//        uint32_t                       int_mask : 16; /*15: 0, Controls whether an interrupt on Port A can create an interrupt for the interrupt controller by not masking it. By default, all interrupts bits are unmasked. Whenever a 1 is written to a bit in this register, it masks the interrupt generation capability for this signal; otherwise interrupts are allowed through. The unmasked status can be read as well as the resultant status after masking.
//0 每 Interrupt bits are unmasked (default)
//1 每 Mask interrupt*/
//        uint32_t                     reserved_0 : 16; /*31:16,                       reserved*/
//    } bit_field;
//} T_GPIO_GPIO_INTMASK;
//
////gpio_inttype_level
//typedef union
//{
//    uint32_t val;
//    struct
//    {
//        uint32_t                      int_level : 16; /*15: 0, Controls the type of interrupt that can occur on Port A. Whenever a 0 is written to a bit of this register, it configures the interrupt type to be level-sensitive; otherwise, it is edge-sensitive.
//0 每 Level-sensitive (default)
//1 每 Edge-sensitive*/
//        uint32_t                     reserved_0 : 16; /*31:16,                       reserved*/
//    } bit_field;
//} T_GPIO_GPIO_INTTYPE_LEVEL;
//
////gpio_int_polarity
//typedef union
//{
//    uint32_t val;
//    struct
//    {
//        uint32_t                   int_polarity : 16; /*15: 0, Controls the polarity of edge or level sensitivity that can occur on input of Port A. Whenever a 0 is written to a bit of this register, it configures the interrupt type to falling-edge or active-low sensitive; otherwise, it is rising-edge or active-high sensitive.
//0 每 Active-low (default)
//1 每 Active-high*/
//        uint32_t                     reserved_0 : 16; /*31:16,                       reserved*/
//    } bit_field;
//} T_GPIO_GPIO_INT_POLARITY;
//
////gpio_intstatus
//typedef union
//{
//    uint32_t val;
//    struct
//    {
//        uint32_t                     int_status : 16; /*15: 0,     Interrupt status of Port A*/
//        uint32_t                     reserved_0 : 16; /*31:16,                       reserved*/
//    } bit_field;
//} T_GPIO_GPIO_INTSTATUS;
//
////gpio_raw_intstatus
//typedef union
//{
//    uint32_t val;
//    struct
//    {
//        uint32_t                 raw_int_status : 16; /*15: 0, Raw interrupt of status of Port A (premasking bits)*/
//        uint32_t                     reserved_0 : 16; /*31:16,                       reserved*/
//    } bit_field;
//} T_GPIO_GPIO_RAW_INTSTATUS;
//
////gpio_debounce
//typedef union
//{
//    uint32_t val;
//    struct
//    {
//        uint32_t                 debounceenable : 16; /*15: 0, Controls whether an external signal that is the source of an interrupt needs to be debounced to remove any spurious glitches. Writing a 1 to a bit in this register enables the debouncing circuitry. A signal must be valid for two periods of an external clock before it is internally processed.
//0 每 No debounce (default)
//1 每 Enable debounce*/
//        uint32_t                     reserved_0 : 16; /*31:16,                       reserved*/
//    } bit_field;
//} T_GPIO_GPIO_DEBOUNCE;
//
////gpio_porta_eoi
//typedef union
//{
//    uint32_t val;
//    struct
//    {
//        uint32_t                      int_clear : 16; /*15: 0, Controls the clearing of edge type interrupts from Port A. When a 1 is written into a corresponding bit of this register, the interrupt is cleared. All interrupts are cleared when Port A is not configured for interrupts.
//0 每 No interrupt clear (default)
//1 每 Clear interrupt*/
//        uint32_t                     reserved_0 : 16; /*31:16,                       reserved*/
//    } bit_field;
//} T_GPIO_GPIO_PORTA_EOI;
//
////gpio_ext_porta
//typedef union
//{
//    uint32_t val;
//    struct
//    {
//        uint32_t                      ext_porta : 16; /*15: 0, This register always reflects the signals value on the External Port A.*/
//        uint32_t                     reserved_0 : 16; /*31:16,                       reserved*/
//    } bit_field;
//} T_GPIO_GPIO_EXT_PORTA;
//
////gpio_ext_portb
//typedef union
//{
//    uint32_t val;
//    struct
//    {
//        uint32_t                      ext_portb : 16; /*15: 0, This register always reflects the signals value on the External Port B.*/
//        uint32_t                     reserved_0 : 16; /*31:16,                       reserved*/
//    } bit_field;
//} T_GPIO_GPIO_EXT_PORTB;
//
////gpio_ls_sync
//typedef union
//{
//    uint32_t val;
//    struct
//    {
//        uint32_t                     sync_level :  1; /* 0: 0, Writing a 1 to this register results in all level-sensitive interrupts being synchronized to pclk_intr.
//0 每 No synchronization to pclk_intr (default)
//1 每 Synchronize to pclk_intr*/
//        uint32_t                     reserved_0 : 31; /*31: 1,                             NA*/
//    } bit_field;
//} T_GPIO_GPIO_LS_SYNC;
//
////gpio_int_bothedge
//typedef union
//{
//    uint32_t val;
//    struct
//    {
//        uint32_t                  int_both_edge : 16; /*15: 0, Controls the edge type of interrupt that can occur on Port A.
//← Whenever a particular bit is programmed to 1, it enables the generation of interrupts on both the rising edge and the falling edge of an external input signal corresponding to that bit on port A.
//The values programmed in the registers gpio_intype_level and gpio_int_polarity for this particular bit are not considered when the corresponding bit of this register is set to 1.
//← Whenever a particular bit is programmed to 0, the interrupt type depends on the value of the corresponding bits in the gpio_inttype_level and gpio_int_polarity registers.
//0 每 Active-low (default)
//1 每 Active-high*/
//        uint32_t                     reserved_0 : 16; /*31:16,                       reserved*/
//    } bit_field;
//} T_GPIO_GPIO_INT_BOTHEDGE;
//
////gpio_ver_id_code
//typedef union
//{
//    uint32_t val;
//    struct
//    {
//        uint32_t                    gpio_ver_id : 32; /*31: 0, ASCII value for each number in the version, followed by *. For example 32_30_31_2A represents the version 2.01*
//Reset Value: See the releases table in the Release Notes*/
//    } bit_field;
//} T_GPIO_GPIO_VER_ID_CODE;
//
////Registers Mapping to the same address
//
//typedef struct
//{
//    volatile          T_GPIO_GPIO_SWPORTA_DR                gpio_swporta_dr; /*  0x0,    RW, 0x00000000,           Port A Data Register*/
//    volatile         T_GPIO_GPIO_SWPORTA_DDR               gpio_swporta_ddr; /*  0x4,    RW, 0x00000000,  Port A Data Direction Registe*/
//    volatile                        uint32_t                     reserved_0;
//    volatile          T_GPIO_GPIO_SWPORTB_DR                gpio_swportb_dr; /*  0xc,    RW, 0x00000000,           Port B Data Register*/
//    volatile         T_GPIO_GPIO_SWPORTB_DDR               gpio_swportb_ddr; /* 0x10,    RW, 0x00000000,  Port B Data Direction Registe*/
//    volatile                        uint32_t                  reserved_1[7];
//    volatile               T_GPIO_GPIO_INTEN                     gpio_inten; /* 0x30,    RW, 0x00000000,               Interrupt enable*/
//    volatile             T_GPIO_GPIO_INTMASK                   gpio_intmask; /* 0x34,    RW, 0x00000000,                 Interrupt mask*/
//    volatile       T_GPIO_GPIO_INTTYPE_LEVEL             gpio_inttype_level; /* 0x38,    RW, 0x00000000,                Interrupt level*/
//    volatile        T_GPIO_GPIO_INT_POLARITY              gpio_int_polarity; /* 0x3c,    RW, 0x00000000,             Interrupt polarity*/
//    volatile           T_GPIO_GPIO_INTSTATUS                 gpio_intstatus; /* 0x40,    RO, 0x00000000,     Interrupt status of Port A*/
//    volatile       T_GPIO_GPIO_RAW_INTSTATUS             gpio_raw_intstatus; /* 0x44,    RO, 0x00000000, Raw interrupt status of Port A (premasking)*/
//    volatile            T_GPIO_GPIO_DEBOUNCE                  gpio_debounce; /* 0x48,    RW, 0x00000000,                Debounce enable*/
//    volatile           T_GPIO_GPIO_PORTA_EOI                 gpio_porta_eoi; /* 0x4c,    WO, 0x00000000,         Port A clear interrupt*/
//    volatile           T_GPIO_GPIO_EXT_PORTA                 gpio_ext_porta; /* 0x50,    RO, 0x00000000,           Port A external port*/
//    volatile           T_GPIO_GPIO_EXT_PORTB                 gpio_ext_portb; /* 0x54,    RO, 0x00000000,           Port B external port*/
//    volatile                        uint32_t                  reserved_2[2];
//    volatile             T_GPIO_GPIO_LS_SYNC                   gpio_ls_sync; /* 0x60,    RW, 0x00000000, Level-sensitive synchronization enable*/
//    volatile                        uint32_t                     reserved_3;
//    volatile        T_GPIO_GPIO_INT_BOTHEDGE              gpio_int_bothedge; /* 0x68,    RW, 0x00000000,       Interrupt both edge type*/
//    volatile         T_GPIO_GPIO_VER_ID_CODE               gpio_ver_id_code; /* 0x6c,    RO, 0x3231312A,              Component Version*/
//} T_HWP_GPIO_T;
//
//#define hwp_gpio ((T_HWP_GPIO_T*)REG_GPIO_BASE)
//
//
//static inline  uint32_t gpio_gpio_swporta_dr_get(void)
//{
//    return hwp_gpio->gpio_swporta_dr.val;
//}
//
//static inline  void gpio_gpio_swporta_dr_set(uint32_t value)
//{
//    hwp_gpio->gpio_swporta_dr.val = value;
//}
//
//static inline  void gpio_gpio_swporta_dr_pack(uint16_t portadataregister)
//{
//    hwp_gpio->gpio_swporta_dr.val = (((uint32_t)portadataregister << 0));
//}
//
//static inline  void gpio_gpio_swporta_dr_unpack(uint16_t* portadataregister)
//{
//    T_GPIO_GPIO_SWPORTA_DR localVal = hwp_gpio->gpio_swporta_dr;
//
//    *portadataregister = localVal.bit_field.portadataregister;
//}
//
//static inline  uint16_t gpio_portadataregister_getf(void)
//{
//    return hwp_gpio->gpio_swporta_dr.bit_field.portadataregister;
//}
//
//static inline  void gpio_portadataregister_setf(uint16_t portadataregister)
//{
//    hwp_gpio->gpio_swporta_dr.bit_field.portadataregister = portadataregister;
//}
//
//static inline  uint32_t gpio_gpio_swporta_ddr_get(void)
//{
//    return hwp_gpio->gpio_swporta_ddr.val;
//}
//
//static inline  void gpio_gpio_swporta_ddr_set(uint32_t value)
//{
//    hwp_gpio->gpio_swporta_ddr.val = value;
//}
//
//static inline  void gpio_gpio_swporta_ddr_pack(uint16_t portadatadirectionregister)
//{
//    hwp_gpio->gpio_swporta_ddr.val = (((uint32_t)portadatadirectionregister << 0));
//}
//
//static inline  void gpio_gpio_swporta_ddr_unpack(uint16_t* portadatadirectionregister)
//{
//    T_GPIO_GPIO_SWPORTA_DDR localVal = hwp_gpio->gpio_swporta_ddr;
//
//    *portadatadirectionregister = localVal.bit_field.portadatadirectionregister;
//}
//
//static inline  uint16_t gpio_portadatadirectionregister_getf(void)
//{
//    return hwp_gpio->gpio_swporta_ddr.bit_field.portadatadirectionregister;
//}
//
//static inline  void gpio_portadatadirectionregister_setf(uint16_t portadatadirectionregister)
//{
//    hwp_gpio->gpio_swporta_ddr.bit_field.portadatadirectionregister = portadatadirectionregister;
//}
//
//static inline  uint32_t gpio_gpio_swportb_dr_get(void)
//{
//    return hwp_gpio->gpio_swportb_dr.val;
//}
//
//static inline  void gpio_gpio_swportb_dr_set(uint32_t value)
//{
//    hwp_gpio->gpio_swportb_dr.val = value;
//}
//
//static inline  void gpio_gpio_swportb_dr_pack(uint16_t portbdataregister)
//{
//    hwp_gpio->gpio_swportb_dr.val = (((uint32_t)portbdataregister << 0));
//}
//
//static inline  void gpio_gpio_swportb_dr_unpack(uint16_t* portbdataregister)
//{
//    T_GPIO_GPIO_SWPORTB_DR localVal = hwp_gpio->gpio_swportb_dr;
//
//    *portbdataregister = localVal.bit_field.portbdataregister;
//}
//
//static inline  uint16_t gpio_portbdataregister_getf(void)
//{
//    return hwp_gpio->gpio_swportb_dr.bit_field.portbdataregister;
//}
//
//static inline  void gpio_portbdataregister_setf(uint16_t portbdataregister)
//{
//    hwp_gpio->gpio_swportb_dr.bit_field.portbdataregister = portbdataregister;
//}
//
//static inline  uint32_t gpio_gpio_swportb_ddr_get(void)
//{
//    return hwp_gpio->gpio_swportb_ddr.val;
//}
//
//static inline  void gpio_gpio_swportb_ddr_set(uint32_t value)
//{
//    hwp_gpio->gpio_swportb_ddr.val = value;
//}
//
//static inline  void gpio_gpio_swportb_ddr_pack(uint16_t portbdatadirectionregister)
//{
//    hwp_gpio->gpio_swportb_ddr.val = (((uint32_t)portbdatadirectionregister << 0));
//}
//
//static inline  void gpio_gpio_swportb_ddr_unpack(uint16_t* portbdatadirectionregister)
//{
//    T_GPIO_GPIO_SWPORTB_DDR localVal = hwp_gpio->gpio_swportb_ddr;
//
//    *portbdatadirectionregister = localVal.bit_field.portbdatadirectionregister;
//}
//
//static inline  uint16_t gpio_portbdatadirectionregister_getf(void)
//{
//    return hwp_gpio->gpio_swportb_ddr.bit_field.portbdatadirectionregister;
//}
//
//static inline  void gpio_portbdatadirectionregister_setf(uint16_t portbdatadirectionregister)
//{
//    hwp_gpio->gpio_swportb_ddr.bit_field.portbdatadirectionregister = portbdatadirectionregister;
//}
//
//static inline  uint32_t gpio_gpio_inten_get(void)
//{
//    return hwp_gpio->gpio_inten.val;
//}
//
//static inline  void gpio_gpio_inten_set(uint32_t value)
//{
//    hwp_gpio->gpio_inten.val = value;
//}
//
//static inline  void gpio_gpio_inten_pack(uint16_t interruptenable)
//{
//    hwp_gpio->gpio_inten.val = (((uint32_t)interruptenable << 0));
//}
//
//static inline  void gpio_gpio_inten_unpack(uint16_t* interruptenable)
//{
//    T_GPIO_GPIO_INTEN localVal = hwp_gpio->gpio_inten;
//
//    *interruptenable = localVal.bit_field.interruptenable;
//}
//
//static inline  uint16_t gpio_interruptenable_getf(void)
//{
//    return hwp_gpio->gpio_inten.bit_field.interruptenable;
//}
//
//static inline  void gpio_interruptenable_setf(uint16_t interruptenable)
//{
//    hwp_gpio->gpio_inten.bit_field.interruptenable = interruptenable;
//}
//
//static inline  uint32_t gpio_gpio_intmask_get(void)
//{
//    return hwp_gpio->gpio_intmask.val;
//}
//
//static inline  void gpio_gpio_intmask_set(uint32_t value)
//{
//    hwp_gpio->gpio_intmask.val = value;
//}
//
//static inline  void gpio_gpio_intmask_pack(uint16_t int_mask)
//{
//    hwp_gpio->gpio_intmask.val = (((uint32_t)int_mask << 0));
//}
//
//static inline  void gpio_gpio_intmask_unpack(uint16_t* int_mask)
//{
//    T_GPIO_GPIO_INTMASK localVal = hwp_gpio->gpio_intmask;
//
//    *int_mask = localVal.bit_field.int_mask;
//}
//
//static inline  uint16_t gpio_int_mask_getf(void)
//{
//    return hwp_gpio->gpio_intmask.bit_field.int_mask;
//}
//
//static inline  void gpio_int_mask_setf(uint16_t int_mask)
//{
//    hwp_gpio->gpio_intmask.bit_field.int_mask = int_mask;
//}
//
//static inline  uint32_t gpio_gpio_inttype_level_get(void)
//{
//    return hwp_gpio->gpio_inttype_level.val;
//}
//
//static inline  void gpio_gpio_inttype_level_set(uint32_t value)
//{
//    hwp_gpio->gpio_inttype_level.val = value;
//}
//
//static inline  void gpio_gpio_inttype_level_pack(uint16_t int_level)
//{
//    hwp_gpio->gpio_inttype_level.val = (((uint32_t)int_level << 0));
//}
//
//static inline  void gpio_gpio_inttype_level_unpack(uint16_t* int_level)
//{
//    T_GPIO_GPIO_INTTYPE_LEVEL localVal = hwp_gpio->gpio_inttype_level;
//
//    *int_level = localVal.bit_field.int_level;
//}
//
//static inline  uint16_t gpio_int_level_getf(void)
//{
//    return hwp_gpio->gpio_inttype_level.bit_field.int_level;
//}
//
//static inline  void gpio_int_level_setf(uint16_t int_level)
//{
//    hwp_gpio->gpio_inttype_level.bit_field.int_level = int_level;
//}
//
//static inline  uint32_t gpio_gpio_int_polarity_get(void)
//{
//    return hwp_gpio->gpio_int_polarity.val;
//}
//
//static inline  void gpio_gpio_int_polarity_set(uint32_t value)
//{
//    hwp_gpio->gpio_int_polarity.val = value;
//}
//
//static inline  void gpio_gpio_int_polarity_pack(uint16_t int_polarity)
//{
//    hwp_gpio->gpio_int_polarity.val = (((uint32_t)int_polarity << 0));
//}
//
//static inline  void gpio_gpio_int_polarity_unpack(uint16_t* int_polarity)
//{
//    T_GPIO_GPIO_INT_POLARITY localVal = hwp_gpio->gpio_int_polarity;
//
//    *int_polarity = localVal.bit_field.int_polarity;
//}
//
//static inline  uint16_t gpio_int_polarity_getf(void)
//{
//    return hwp_gpio->gpio_int_polarity.bit_field.int_polarity;
//}
//
//static inline  void gpio_int_polarity_setf(uint16_t int_polarity)
//{
//    hwp_gpio->gpio_int_polarity.bit_field.int_polarity = int_polarity;
//}
//
//static inline  uint32_t gpio_gpio_intstatus_get(void)
//{
//    return hwp_gpio->gpio_intstatus.val;
//}
//
//static inline  void gpio_gpio_intstatus_unpack(uint16_t* int_status)
//{
//    T_GPIO_GPIO_INTSTATUS localVal = hwp_gpio->gpio_intstatus;
//
//    *int_status = localVal.bit_field.int_status;
//}
//
//static inline  uint16_t gpio_int_status_getf(void)
//{
//    return hwp_gpio->gpio_intstatus.bit_field.int_status;
//}
//
//static inline  uint32_t gpio_gpio_raw_intstatus_get(void)
//{
//    return hwp_gpio->gpio_raw_intstatus.val;
//}
//
//static inline  void gpio_gpio_raw_intstatus_unpack(uint16_t* raw_int_status)
//{
//    T_GPIO_GPIO_RAW_INTSTATUS localVal = hwp_gpio->gpio_raw_intstatus;
//
//    *raw_int_status = localVal.bit_field.raw_int_status;
//}
//
//static inline  uint16_t gpio_raw_int_status_getf(void)
//{
//    return hwp_gpio->gpio_raw_intstatus.bit_field.raw_int_status;
//}
//
//static inline  uint32_t gpio_gpio_debounce_get(void)
//{
//    return hwp_gpio->gpio_debounce.val;
//}
//
//static inline  void gpio_gpio_debounce_set(uint32_t value)
//{
//    hwp_gpio->gpio_debounce.val = value;
//}
//
//static inline  void gpio_gpio_debounce_pack(uint16_t debounceenable)
//{
//    hwp_gpio->gpio_debounce.val = (((uint32_t)debounceenable << 0));
//}
//
//static inline  void gpio_gpio_debounce_unpack(uint16_t* debounceenable)
//{
//    T_GPIO_GPIO_DEBOUNCE localVal = hwp_gpio->gpio_debounce;
//
//    *debounceenable = localVal.bit_field.debounceenable;
//}
//
//static inline  uint16_t gpio_debounceenable_getf(void)
//{
//    return hwp_gpio->gpio_debounce.bit_field.debounceenable;
//}
//
//static inline  void gpio_debounceenable_setf(uint16_t debounceenable)
//{
//    hwp_gpio->gpio_debounce.bit_field.debounceenable = debounceenable;
//}
//
//static inline  void gpio_gpio_porta_eoi_set(uint32_t value)
//{
//    hwp_gpio->gpio_porta_eoi.val = value;
//}
//
//static inline  void gpio_gpio_porta_eoi_pack(uint16_t int_clear)
//{
//    hwp_gpio->gpio_porta_eoi.val = (((uint32_t)int_clear << 0));
//}
//
//static inline  uint32_t gpio_gpio_ext_porta_get(void)
//{
//    return hwp_gpio->gpio_ext_porta.val;
//}
//
//static inline  void gpio_gpio_ext_porta_unpack(uint16_t* ext_porta)
//{
//    T_GPIO_GPIO_EXT_PORTA localVal = hwp_gpio->gpio_ext_porta;
//
//    *ext_porta = localVal.bit_field.ext_porta;
//}
//
//static inline  uint16_t gpio_ext_porta_getf(void)
//{
//    return hwp_gpio->gpio_ext_porta.bit_field.ext_porta;
//}
//
//static inline  uint32_t gpio_gpio_ext_portb_get(void)
//{
//    return hwp_gpio->gpio_ext_portb.val;
//}
//
//static inline  void gpio_gpio_ext_portb_unpack(uint16_t* ext_portb)
//{
//    T_GPIO_GPIO_EXT_PORTB localVal = hwp_gpio->gpio_ext_portb;
//
//    *ext_portb = localVal.bit_field.ext_portb;
//}
//
//static inline  uint16_t gpio_ext_portb_getf(void)
//{
//    return hwp_gpio->gpio_ext_portb.bit_field.ext_portb;
//}
//
//static inline  uint32_t gpio_gpio_ls_sync_get(void)
//{
//    return hwp_gpio->gpio_ls_sync.val;
//}
//
//static inline  void gpio_gpio_ls_sync_set(uint32_t value)
//{
//    hwp_gpio->gpio_ls_sync.val = value;
//}
//
//static inline  void gpio_gpio_ls_sync_pack(uint8_t sync_level)
//{
//    hwp_gpio->gpio_ls_sync.val = (((uint32_t)sync_level << 0));
//}
//
//static inline  void gpio_gpio_ls_sync_unpack(uint8_t* sync_level)
//{
//    T_GPIO_GPIO_LS_SYNC localVal = hwp_gpio->gpio_ls_sync;
//
//    *sync_level = localVal.bit_field.sync_level;
//}
//
//static inline  uint8_t gpio_sync_level_getf(void)
//{
//    return hwp_gpio->gpio_ls_sync.bit_field.sync_level;
//}
//
//static inline  void gpio_sync_level_setf(uint8_t sync_level)
//{
//    hwp_gpio->gpio_ls_sync.bit_field.sync_level = sync_level;
//}
//
//static inline  uint32_t gpio_gpio_int_bothedge_get(void)
//{
//    return hwp_gpio->gpio_int_bothedge.val;
//}
//
//static inline  void gpio_gpio_int_bothedge_set(uint32_t value)
//{
//    hwp_gpio->gpio_int_bothedge.val = value;
//}
//
//static inline  void gpio_gpio_int_bothedge_pack(uint16_t int_both_edge)
//{
//    hwp_gpio->gpio_int_bothedge.val = (((uint32_t)int_both_edge << 0));
//}
//
//static inline  void gpio_gpio_int_bothedge_unpack(uint16_t* int_both_edge)
//{
//    T_GPIO_GPIO_INT_BOTHEDGE localVal = hwp_gpio->gpio_int_bothedge;
//
//    *int_both_edge = localVal.bit_field.int_both_edge;
//}
//
//static inline  uint16_t gpio_int_both_edge_getf(void)
//{
//    return hwp_gpio->gpio_int_bothedge.bit_field.int_both_edge;
//}
//
//static inline  void gpio_int_both_edge_setf(uint16_t int_both_edge)
//{
//    hwp_gpio->gpio_int_bothedge.bit_field.int_both_edge = int_both_edge;
//}
//
//static inline  uint32_t gpio_gpio_ver_id_code_get(void)
//{
//    return hwp_gpio->gpio_ver_id_code.val;
//}
//
//static inline  void gpio_gpio_ver_id_code_unpack(uint32_t* gpio_ver_id)
//{
//    T_GPIO_GPIO_VER_ID_CODE localVal = hwp_gpio->gpio_ver_id_code;
//
//    *gpio_ver_id = localVal.bit_field.gpio_ver_id;
//}
//
//static inline  uint32_t gpio_gpio_ver_id_getf(void)
//{
//    return hwp_gpio->gpio_ver_id_code.bit_field.gpio_ver_id;
//}
//
//
//
//
//
//














#ifdef __cplusplus
}
#endif

#endif
