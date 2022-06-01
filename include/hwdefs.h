#ifndef HWDEFS_H_INCLUDED
#define HWDEFS_H_INCLUDED


//Common for any config

//Maximum value for over current limit timer
#define GAUGEMAX 1024
#define USART_BAUDRATE 115200
//Maximum PWM frequency is 36MHz/2^MIN_PWM_DIGITS
#define MIN_PWM_DIGITS 11
#define PERIPH_CLK      ((uint32_t)36000000)

#define RCC_CLOCK_SETUP rcc_clock_setup_in_hse_8mhz_out_72mhz

#define AVAS_TIMER         TIM1
#define REV_CNT_TIMER      TIM4
#define FUELGAUGE_TIMER    TIM3

#define TERM_USART         USART3
#define TERM_USART_TXPIN   GPIO_USART3_TX
#define TERM_USART_TXPORT  GPIOB
#define TERM_USART_DMARX   DMA_CHANNEL3
#define TERM_USART_DMATX   DMA_CHANNEL2 //this means we can not use it on rev1 hardware (TIM3_CH3)
#define TERM_USART_DR      USART3_DR
#define TERM_BUFSIZE       128
//Address of parameter block in flash
#define PARAM_BLKSIZE 1024
#define PARAM_BLKNUM  1
#define CAN_BLKSIZE   1024
#define CAN_BLKNUM    2

typedef enum
{
   HW_REV1, HW_REV2, HW_REV3, HW_TESLA
} HWREV;

extern HWREV hwRev;

#endif // HWDEFS_H_INCLUDED
