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
#include <stdint.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/iwdg.h>
#include "stm32_can.h"
#include "terminal.h"
#include "params.h"
#include "hwdefs.h"
#include "digio.h"
#include "hwinit.h"
#include "anain.h"
#include "param_save.h"
#include "my_math.h"
#include "errormessage.h"
#include "printf.h"
#include "stm32scheduler.h"
#include "leafbms.h"

#define RMS_SAMPLES 256
#define SQRT2OV1 0.707106781187
#define PRECHARGE_TIMEOUT 500 //5s
#define CAN_TIMEOUT       50  //500ms

HWREV hwRev; //Hardware variant of board we are running on

static Stm32Scheduler* scheduler;

static void Ms100Task(void)
{
   static int seqCtr = 0;
   static uint8_t ctr = 0;

   const uint8_t seq1[] = { 0x0f, 0x28, 0x7f, 0x28 };
   const uint8_t seq2[] = { 0x1e, 0x10, 0x00, 0x10 };
   const uint8_t seq3[] = { 0x70, 0x56, 0xf0, 0x56 };
   const uint8_t seq4[] = { 0x0c, 0x48, 0xa7, 0x48 };
   const uint8_t seq5[] = { 0x46, 0x90, 0x28, 0x90 };

   uint8_t canData[8] = { (uint8_t)(0x80 | ctr), 0, 0, seq1[seqCtr], seq2[seqCtr], seq3[seqCtr], seq4[seqCtr], seq5[seqCtr] };

   DigIo::led_out.Toggle();
   //ErrorMessage::PrintNewErrors();
   iwdg_reset();
   s32fp cpuLoad = FP_FROMINT(scheduler->GetCpuLoad());
   Param::SetFlt(Param::cpuload, cpuLoad / 10);
   Param::SetInt(Param::lasterr, ErrorMessage::GetLastError());

   Can::Send(0x580, (uint32_t*)canData);
   seqCtr = (seqCtr + 1) & 0x3;
   ctr = (ctr + 1) & 0xF;
   LeafBMS::RequestNextFrame();
   LeafBMS::Send100msMessages();

   if (Param::GetInt(Param::canperiod) == CAN_PERIOD_100MS)
      Can::SendAll();
}

static void GetDigInputs()
{
   int canio = 0;

   if (Param::GetBool(Param::din_cruise))
      canio |= CAN_IO_CRUISE;
   if (Param::GetBool(Param::din_start))
      canio |= CAN_IO_START;
   if (Param::GetBool(Param::din_brake))
      canio |= CAN_IO_BRAKE;
   if (Param::GetBool(Param::din_forward))
      canio |= CAN_IO_FWD;
   if (Param::GetBool(Param::din_reverse))
      canio |= CAN_IO_REV;
   if (Param::GetBool(Param::din_bms))
      canio |= CAN_IO_BMS;

   Param::SetInt(Param::canio, canio);
}

static void CalcAndOutputTemp()
{
   static int temphsFlt = 0;
   static int tempmFlt = 0;
   int pwmgain = Param::GetInt(Param::pwmgain);
   int pwmofs = Param::GetInt(Param::pwmofs);
   int pwmfunc = Param::GetInt(Param::pwmfunc);
   int tmpout = 0;
   s32fp tmphs = 0, tmpm = 0;


   temphsFlt = IIRFILTER(tmphs, temphsFlt, 15);
   tempmFlt = IIRFILTER(tmpm, tempmFlt, 18);

   switch (pwmfunc)
   {
      default:
      case PWM_FUNC_TMPM:
         tmpout = FP_TOINT(tmpm) * pwmgain + pwmofs;
         break;
      case PWM_FUNC_TMPHS:
         tmpout = FP_TOINT(tmphs) * pwmgain + pwmofs;
         break;
      case PWM_FUNC_SPEED:
         tmpout = Param::Get(Param::speed) * pwmgain + pwmofs;
         break;
      case PWM_FUNC_SPEEDFRQ:
         //Handled in 1ms task
         break;
   }

   tmpout = MIN(0xFFFF, MAX(0, tmpout));

   timer_set_oc_value(OVER_CUR_TIMER, TIM_OC4, tmpout);

   Param::SetFlt(Param::tmphs, tmphs);
   //Param::SetFlt(Param::tmpm, tmpm);
}


static void Ms10Task(void)
{
   //const uint8_t seq1[] = { 0x27, 0x46, 0x8F, 0xD7 };
   const uint8_t seq2[] = { 0x10, 0x68, 0x94, 0xC0 };
   static int seq1Ctr = 0;
   static uint16_t seq2Ctr = 0;
   int vacuumthresh = Param::GetInt(Param::vacuumthresh);
   int vacuumhyst = Param::GetInt(Param::vacuumhyst);
   int oilthresh = Param::GetInt(Param::oilthresh);
   int oilhyst = Param::GetInt(Param::oilhyst);
   int vacuum = AnaIn::Get(AnaIn::vacuum);
   int speed = Param::GetInt(Param::speed);
   int opmode = Param::GetInt(Param::opmode);
   int cruiselight = Param::GetInt(Param::cruiselight);
   int errlights = Param::GetInt(Param::errlights);
   s32fp udc = Param::Get(Param::udcinv);
   s32fp udcthresh = Param::Get(Param::udcthresh);
   s32fp udchyst = Param::Get(Param::udchyst);
   s32fp ucellthresh = Param::Get(Param::ucellthresh);
   s32fp ucellhyst = Param::Get(Param::ucellhyst);
   s32fp ucellmax = Param::Get(Param::batmax);
   s32fp idc = Param::Get(Param::idc);
   s32fp udcbms = Param::Get(Param::udcbms);
   s32fp power = FP_MUL(idc, udcbms) / 1000;
   s32fp powerpos = power > 0 ? 0 : -power;

   seq1Ctr = (seq1Ctr + 1) & 0x3;
   seq2Ctr += FP_TOINT(FP_MUL(powerpos, FP_FROMFLT(2.8)));

   Param::SetFlt(Param::power, power);

   uint32_t canData[2];

   //Byte1 seq 2, Byte ?, Byte 7 XOR(bytes[0..6])
   uint8_t check = seq2[seq1Ctr] ^ errlights ^ (seq2Ctr & 0xFF) ^ (seq2Ctr >> 8) ^ cruiselight ^ 0x1A;
   canData[0] = seq2[seq1Ctr] | errlights << 8 | seq2Ctr << 16;
   canData[1] = 0x1A | cruiselight << 16 | check << 24;

   Can::Send(0x480, canData);

   if (vacuum > vacuumthresh)
   {
      DigIo::vacuum_out.Clear();
   }
   else if (vacuum < vacuumhyst)
   {
      DigIo::vacuum_out.Set();
   }

   if (speed > oilthresh)
   {
      DigIo::oil_out.Set();
   }
   else if (speed < oilhyst)
   {
      DigIo::oil_out.Clear();
   }

   if (opmode == MOD_OFF)
   {
      DigIo::dcsw_out.Clear();
      Param::SetInt(Param::speedmod, 0);
   }
   else
   {
      DigIo::dcsw_out.Set();
      Param::SetFlt(Param::speedmod, MAX(FP_FROMINT(700), Param::Get(Param::speed)));
   }

   if (udc > udcthresh || ucellmax > ucellthresh)
   {
      Param::SetInt(Param::din_bms, 1);
   }
   else if (udc < udchyst && ucellmax < ucellhyst)
   {
      Param::SetInt(Param::din_bms, 0);
   }

   Param::SetInt(Param::vacuum, vacuum);
   Param::SetInt(Param::pot, AnaIn::Get(AnaIn::throttle1));
   Param::SetInt(Param::pot2, AnaIn::Get(AnaIn::throttle2));
   Param::SetFlt(Param::tmpmod, FP_FROMINT(48) + ((Param::Get(Param::tmpm) * 4) / 3));

   GetDigInputs();
   CalcAndOutputTemp();
   ErrorMessage::SetTime(rtc_get_counter_val());

   LeafBMS::Send10msMessages();
   if (Param::GetInt(Param::canperiod) == CAN_PERIOD_10MS)
      Can::SendAll();
}

/** This function is called when the user changes a parameter */
extern void parm_Change(Param::PARAM_NUM paramNum)
{
   if (Param::canspeed == paramNum)
      Can::SetBaudrate((enum Can::baudrates)Param::GetInt(Param::canspeed));
}

static void CanCallback(uint32_t id, uint32_t data[2])
{
   LeafBMS::DecodeCAN(id, data);
}

static void ConfigureVariantIO()
{
   AnaIn::AnaInfo analogInputs[] = ANA_IN_ARRAY(ANA_IN_LIST);

   hwRev = detect_hw();
   Param::SetInt(Param::hwver, hwRev);

   DIG_IO_CONFIGURE(DIG_IO_LIST);

   switch (hwRev)
   {
      case HW_REV1:
         analogInputs[AnaIn::il2].port = GPIOA;
         analogInputs[AnaIn::il2].pin = 6;
         break;
      case HW_REV2:
         break;
      case HW_REV3:
         break;
      case HW_TESLA:
         DigIo::temp1_out.Configure(GPIOC, GPIO8, PinMode::OUTPUT);
         break;
   }

   AnaIn::Init(analogInputs);
}

extern "C" void tim2_isr(void)
{
   scheduler->Run();
}

extern "C" int main(void)
{
   clock_setup();
   rtc_setup();
   ConfigureVariantIO();
   usart_setup();
   tim_setup();
   nvic_setup();
   term_Init();
   parm_load();
   parm_Change(Param::PARAM_LAST);
   Can::Init((Can::baudrates)Param::GetInt(Param::canspeed));
   Can::SetReceiveCallback(CanCallback);
   Can::RegisterUserMessage(0x7BB);
   Can::RegisterUserMessage(0x1DB);
   Can::RegisterUserMessage(0x1DC);
   Can::RegisterUserMessage(0x55B);
   Can::RegisterUserMessage(0x5BC);
   Can::RegisterUserMessage(0x5C0);

   Stm32Scheduler s(TIM2); //We never exit main so it's ok to put it on stack
   scheduler = &s;

   s.AddTask(Ms10Task, 10);
   s.AddTask(Ms100Task, 100);

   DigIo::vacuum_out.Set();
   Param::SetInt(Param::version, 4); //backward compatibility

   term_Run();

   return 0;
}

