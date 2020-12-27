#ifndef DIGIO_PRJ_H_INCLUDED
#define DIGIO_PRJ_H_INCLUDED

#include "hwdefs.h"

#define DIG_IO_LIST \
    DIG_IO_ENTRY(cruise_in,   GPIOB, GPIO5,  PinMode::INPUT_PD)   \
    DIG_IO_ENTRY(start_in,    GPIOB, GPIO6,  PinMode::INPUT_PD)   \
    DIG_IO_ENTRY(brake_in,    GPIOA, GPIO2,  PinMode::INPUT_PD)   \
    DIG_IO_ENTRY(mprot_in,    GPIOA, GPIO3,  PinMode::INPUT_PU)    \
    DIG_IO_ENTRY(fwd_in,      GPIOA, GPIO4,  PinMode::INPUT_PD)   \
    DIG_IO_ENTRY(rev_in,      GPIOC, GPIO6,  PinMode::INPUT_PD)   \
    DIG_IO_ENTRY(emcystop_in, GPIOC, GPIO7,  PinMode::INPUT_PD)   \
    DIG_IO_ENTRY(bms_in,      GPIOC, GPIO8,  PinMode::INPUT_PD)   \
    DIG_IO_ENTRY(dcsw_out,    GPIOC, GPIO13, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(prec_out,    GPIOC, GPIO11, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(vacuum_out,  GPIOB, GPIO1,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(led_out,     GPIOC, GPIO12, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(err_out,     GPIOC, GPIO10, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(speed_out,   GPIOB, GPIO9,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(oil_out,     GPIOC, GPIO5,  PinMode::OUTPUT)      \


#endif // DIGIO_PRJ_H_INCLUDED
