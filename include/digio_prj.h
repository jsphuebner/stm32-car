#ifndef DIGIO_PRJ_H_INCLUDED
#define DIGIO_PRJ_H_INCLUDED

#include "hwdefs.h"

#if TARGET == 107
#define LEDIO GPIO6
#else
#define LEDIO GPIO12
#endif // TARGET

#define DIG_IO_LIST \
    DIG_IO_ENTRY(charge_in,   GPIOC, GPIO7,  PinMode::INPUT_PD)   \
    DIG_IO_ENTRY(start_in,    GPIOB, GPIO6,  PinMode::INPUT_PD)   \
    DIG_IO_ENTRY(brake_in,    GPIOA, GPIO2,  PinMode::INPUT_PD)   \
    DIG_IO_ENTRY(mprot_in,    GPIOA, GPIO3,  PinMode::INPUT_PU)    \
    DIG_IO_ENTRY(fwd_in,      GPIOA, GPIO4,  PinMode::INPUT_PD)   \
    DIG_IO_ENTRY(rev_in,      GPIOC, GPIO6,  PinMode::INPUT_PD)   \
    DIG_IO_ENTRY(bms_in,      GPIOC, GPIO8,  PinMode::INPUT_PD)   \
    DIG_IO_ENTRY(dcsw_out,    GPIOC, GPIO13, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(dcdc_out,    GPIOC, GPIO11, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(vacuum_out,  GPIOB, GPIO1,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(led_out,     GPIOC, LEDIO,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(heater_out,  GPIOC, GPIO10, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(oil_out,     GPIOC, GPIO5,  PinMode::OUTPUT)      \


#endif // DIGIO_PRJ_H_INCLUDED
