/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/desig.h>
#include "stm32_loader.h"
#include "my_string.h"
#include "hwdefs.h"
#include "hwinit.h"

/**
* Start clocks of all needed peripherals
*/
void clock_setup(void)
{
   #if TARGET == 107
   #warning building for prototype board
   rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE25_72MHZ]);
   #elif TARGET == 103
   rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
   #endif // TARGET

   //The reset value for PRIGROUP (=0) is not actually a defined
   //value. Explicitly set 16 preemtion priorities
   SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_PRIGROUP_GROUP16_NOSUB;

   rcc_periph_clock_enable(RCC_GPIOA);
   rcc_periph_clock_enable(RCC_GPIOB);
   rcc_periph_clock_enable(RCC_GPIOC);
   rcc_periph_clock_enable(RCC_GPIOD);
   rcc_periph_clock_enable(RCC_USART3);
   rcc_periph_clock_enable(RCC_TIM1); //Main PWM
   rcc_periph_clock_enable(RCC_TIM2); //Scheduler
   rcc_periph_clock_enable(RCC_TIM3); //Rotor Encoder
   rcc_periph_clock_enable(RCC_TIM4); //Overcurrent / AUX PWM
   rcc_periph_clock_enable(RCC_DMA1);  //ADC, Encoder and UART receive
   rcc_periph_clock_enable(RCC_ADC1);
   rcc_periph_clock_enable(RCC_CRC);
   rcc_periph_clock_enable(RCC_AFIO); //CAN
   rcc_periph_clock_enable(RCC_CAN1); //CAN
}

void write_bootloader_pininit()
{
   uint32_t flashSize = desig_get_flash_size();
   uint32_t pindefAddr = FLASH_BASE + flashSize * 1024 - PINDEF_BLKNUM * PINDEF_BLKSIZE;
   const struct pincommands* flashCommands = (struct pincommands*)pindefAddr;
   struct pincommands commands;

   memset32((int*)&commands, 0, PINDEF_NUMWORDS);

   //Keep vacuum pump off in bootloader
   commands.pindef[0].port = GPIOB;
   commands.pindef[0].pin = GPIO1;
   commands.pindef[0].inout = PIN_OUT;
   commands.pindef[0].level = 1;

   crc_reset();
   uint32_t crc = crc_calculate_block(((uint32_t*)&commands), PINDEF_NUMWORDS);
   commands.crc = crc;

   if (commands.crc != flashCommands->crc)
   {
      flash_unlock();
      flash_erase_page(pindefAddr);

      //Write flash including crc, therefor <=
      for (uint32_t idx = 0; idx <= PINDEF_NUMWORDS; idx++)
      {
         uint32_t* pData = ((uint32_t*)&commands) + idx;
         flash_program_word(pindefAddr + idx * sizeof(uint32_t), *pData);
      }
      flash_lock();
   }
}

/**
* Enable Timer refresh and break interrupts
*/
void nvic_setup(void)
{
   nvic_enable_irq(NVIC_TIM2_IRQ); //Scheduler
   nvic_set_priority(NVIC_TIM2_IRQ, 0); //highest priority
}

void rtc_setup()
{
   //Base clock is HSE/128 = 8MHz/128 = 62.5kHz
   //62.5kHz / (624 + 1) = 100Hz
   #if TARGET == 107
   //25 MHz HSE = 25 MHz/128 = 195.3 kHz. /(1952+1) = 100 Hz
   rtc_auto_awake(RCC_HSE, 1952); //10ms tick
   #else
   rtc_auto_awake(RCC_HSE, 624); //10ms tick
   #endif // TARGET
   rtc_set_counter_val(0);
}

/**
* Setup main PWM timer and timer for generating over current
* reference values and external PWM
*/
void tim_setup()
{
   /*** Setup over/undercurrent and PWM output timer */
   timer_disable_counter(FUELGAUGE_TIMER);
   //edge aligned PWM
   timer_set_alignment(FUELGAUGE_TIMER, TIM_CR1_CMS_EDGE);
   timer_enable_preload(FUELGAUGE_TIMER);
   /* PWM mode 1 and preload enable */
   timer_set_oc_mode(FUELGAUGE_TIMER, TIM_OC2, TIM_OCM_PWM1);
   timer_set_oc_mode(FUELGAUGE_TIMER, TIM_OC3, TIM_OCM_PWM1);
   timer_set_oc_mode(FUELGAUGE_TIMER, TIM_OC4, TIM_OCM_PWM1);
   timer_enable_oc_preload(FUELGAUGE_TIMER, TIM_OC2);
   timer_enable_oc_preload(FUELGAUGE_TIMER, TIM_OC3);
   timer_enable_oc_preload(FUELGAUGE_TIMER, TIM_OC4);

   timer_set_oc_polarity_high(FUELGAUGE_TIMER, TIM_OC2);
   timer_set_oc_polarity_high(FUELGAUGE_TIMER, TIM_OC3);
   timer_set_oc_polarity_high(FUELGAUGE_TIMER, TIM_OC4);
   timer_enable_oc_output(FUELGAUGE_TIMER, TIM_OC2);
   timer_enable_oc_output(FUELGAUGE_TIMER, TIM_OC3);
   timer_enable_oc_output(FUELGAUGE_TIMER, TIM_OC4);
   timer_generate_event(FUELGAUGE_TIMER, TIM_EGR_UG);
   timer_set_prescaler(FUELGAUGE_TIMER, 0);
   /* PWM frequency */
   timer_set_period(FUELGAUGE_TIMER, GAUGEMAX);
   timer_enable_counter(FUELGAUGE_TIMER);

   /** setup gpio */
   gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7 | GPIO8 | GPIO9);
}

