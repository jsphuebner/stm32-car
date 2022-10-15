#ifndef HWDEFS_H_INCLUDED
#define HWDEFS_H_INCLUDED


//Common for any config

#define USART_BAUDRATE 115200
//Maximum PWM frequency is 36MHz/2^MIN_PWM_DIGITS
#define MIN_PWM_DIGITS 11
#define PERIPH_CLK      ((uint32_t)36000000)

#define RCC_CLOCK_SETUP rcc_clock_setup_in_hse_8mhz_out_72mhz

#define PWM_TIMER     TIM1
#define PWM_TIMRST    RST_TIM1
#define PWM_TIMER_IRQ NVIC_TIM1_UP_IRQ
#define pwm_timer_isr tim1_up_isr

#define FUELGAUGE_TIMER    TIM4
//Maximum value for over current limit timer
#define GAUGEMAX           65535

//Address of parameter block in flash
#define FLASH_PAGE_SIZE 1024
#define PARAM_BLKSIZE FLASH_PAGE_SIZE
#define PARAM_BLKNUM  1
#define CAN1_BLKNUM   2
#define CAN2_BLKNUM   4


#endif // HWDEFS_H_INCLUDED
