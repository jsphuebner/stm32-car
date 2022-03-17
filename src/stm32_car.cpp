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
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/flash.h>
#include "stm32_can.h"
#include "linbus.h"
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
#include "chademo.h"

#define RMS_SAMPLES 256
#define SQRT2OV1 0.707106781187
#define PRECHARGE_TIMEOUT 500 //5s
#define CAN_TIMEOUT       50  //500ms

HWREV hwRev; //Hardware variant of board we are running on

static Stm32Scheduler* scheduler;
static bool chargeMode = false;
static Can* can;
static LinBus* lin;

static void Ms500Task(void)
{
   static modes modeLast = MOD_OFF;
   static int blinks = 0;
   static int regenLevelLast = 0;
   modes mode = (modes)Param::GetInt(Param::opmode);
   bool cruiseLight = Param::GetBool(Param::cruiselight);
   int regenLevel = Param::GetInt(Param::regenlevel);

   if (mode == MOD_RUN && modeLast == MOD_OFF)
   {
      blinks = 10;
   }
   if (blinks > 0)
   {
      blinks--;
      Param::SetInt(Param::cruiselight, !cruiseLight);
   }
   else if (Param::GetInt(Param::cruisespeed) > 0)
   {
      Param::SetInt(Param::cruiselight, 1);
   }
   else
   {
      Param::SetInt(Param::cruiselight, 0);
      //Signal regen level by number of blinks + 1
      if (mode == MOD_RUN && regenLevel != regenLevelLast)
      {

         blinks = 2 * (regenLevel + 1);
      }
   }

   regenLevelLast = Param::GetInt(Param::regenlevel);
   modeLast = mode;
}

static void SendLin()
{
   static bool read = true;

   if (lin->HasReceived(22, 8))
   {
      uint8_t* data = lin->GetReceivedBytes();

      Param::SetInt(Param::tmpheater, data[1] - 40);
      Param::SetInt(Param::udcheater, data[4] | ((data[5] & 3) << 8));
      Param::SetFlt(Param::powerheater, FP_FROMINT(((data[5] >> 2) | (data[6] << 8)) * 20) / 1000);
   }

   if (read)
   {
      lin->Request(22, 0, 0);
   }
   else
   {
      uint8_t lindata[4];
      lindata[0] = Param::GetInt(Param::heatpowmax) / 40;
      lindata[1] = Param::GetInt(Param::heatmax) + 40;
      lindata[2] = 0;
      lindata[3] = Param::GetInt(Param::heatpowmax) > 0 ? 8 : 0;

      lin->Request(21, lindata, sizeof(lindata));
   }

   read = !read;
}

static void ProcessCruiseControlButtons()
{
   static bool transition = false;
   static int cruiseTarget = 0;
   int cruisespeed = Param::GetInt(Param::cruisespeed);
   int cruisestt = Param::GetInt(Param::cruisestt);

   if (transition)
   {
      if ((cruisestt & (CRUISE_SETP | CRUISE_SETN)) == 0)
      {
         transition = false;
      }
      return;
   }
   else
   {
      if (cruisestt & (CRUISE_SETP | CRUISE_SETN))
      {
         transition = true;
      }
   }

   //When pressing cruise control buttons and brake pedal
   //Use them to adjust regen level
   if (Param::GetBool(Param::din_brake))
   {
      int regenLevel = Param::GetInt(Param::regenlevel);
      if (cruisestt & CRUISE_SETP)
      {
         regenLevel++;
         regenLevel = MIN(3, regenLevel);
      }
      else if (cruisestt & CRUISE_SETN)
      {
         regenLevel--;
         regenLevel = MAX(0, regenLevel);
      }

      Param::SetInt(Param::regenlevel, regenLevel);
   }

   if (cruisestt & CRUISE_ON && Param::GetInt(Param::opmode) == MOD_RUN)
   {
      if (cruisespeed <= 0)
      {
         if (cruisestt & CRUISE_SETN) //Start cruise control at current speed
         {
            cruiseTarget = Param::GetInt(Param::speed);
            cruisespeed = cruiseTarget;
         }
         else if (cruisestt & CRUISE_SETP) //resume via ramp
         {
            cruisespeed = Param::GetInt(Param::speed);
         }
      }
      else
      {
         if (cruisestt & CRUISE_DISABLE || Param::GetBool(Param::din_brake))
         {
            cruisespeed = 0;
         }
         else if (cruisestt & CRUISE_SETP)
         {
            cruiseTarget += Param::GetInt(Param::cruisestep);
         }
         else if (cruisestt & CRUISE_SETN)
         {
            cruiseTarget -= Param::GetInt(Param::cruisestep);
         }
      }
   }
   else
   {
      cruisespeed = 0;
      cruiseTarget = 0;
   }

   if (cruisespeed <= 0)
   {
      Param::SetInt(Param::cruisespeed, 0);
   }
   else if (cruisespeed < cruiseTarget)
   {
      Param::SetInt(Param::cruisespeed, RAMPUP(cruisespeed, cruiseTarget, Param::GetInt(Param::cruiserampup)));
   }
   else if (cruisespeed > cruiseTarget)
   {
      Param::SetInt(Param::cruisespeed, RAMPDOWN(cruisespeed, cruiseTarget, Param::GetInt(Param::cruiserampdn)));
   }
   else
   {
      Param::SetInt(Param::cruisespeed, cruisespeed);
   }
}

static void RunChaDeMo()
{
   static uint32_t connectorLockTime = 0;

   if (DigIo::charge_in.Get() && !chargeMode)
   {
      chargeMode = true;
      Param::SetInt(Param::opmode, MOD_CHARGESTART);
      timer_set_oc_value(FUELGAUGE_TIMER, TIM_OC2, 0);
      timer_set_oc_value(FUELGAUGE_TIMER, TIM_OC3, 0);
   }

   /* 1s after entering charge mode, enable charge permission */
   if (Param::GetInt(Param::opmode) == MOD_CHARGESTART && rtc_get_counter_val() > 200 && DigIo::dcsw_out.Get())
   {
      ChaDeMo::SetEnabled(true);
      //Use fuel gauge line to control charge enable signal
      timer_set_oc_value(FUELGAUGE_TIMER, TIM_OC2, GAUGEMAX);
      timer_set_oc_value(FUELGAUGE_TIMER, TIM_OC3, GAUGEMAX);
   }

   if (connectorLockTime == 0 && ChaDeMo::ConnectorLocked())
   {
      connectorLockTime = rtc_get_counter_val();
      Param::SetInt(Param::opmode, MOD_CHARGELOCK);
   }
   //10s after locking tell EVSE that we closed the contactor (in fact we have no control)
   if (Param::GetInt(Param::opmode) == MOD_CHARGELOCK && (rtc_get_counter_val() - connectorLockTime) > 1000 && DigIo::dcsw_out.Get())
   {
      ChaDeMo::SetContactor(true);
      Param::SetInt(Param::opmode, MOD_CHARGE);
   }

   if (Param::GetInt(Param::opmode) == MOD_CHARGE)
   {
      int chargeCur = Param::GetInt(Param::chgcurlim);
      int chargeLim = Param::GetInt(Param::chargelimit);
      chargeCur = MIN(MIN(255, chargeLim), chargeCur);
      ChaDeMo::SetChargeCurrent(chargeCur);
      ChaDeMo::CheckSensorDeviation(Param::GetInt(Param::udcbms));
   }

   if (Param::GetInt(Param::opmode) == MOD_CHARGEND)
   {
      ChaDeMo::SetChargeCurrent(0);
   }

   ChaDeMo::SetTargetBatteryVoltage(Param::GetInt(Param::udcthresh));
   ChaDeMo::SetSoC(Param::Get(Param::soc));
   Param::SetInt(Param::cdmcureq, ChaDeMo::GetRampedCurrentRequest());

   if (chargeMode)
   {
      if (Param::GetInt(Param::batfull) ||
          Param::Get(Param::soc) >= Param::Get(Param::soclimit) ||
          Param::GetInt(Param::chargelimit) == 0 ||
          !LeafBMS::Alive(rtc_get_counter_val()))
      {
         if (!LeafBMS::Alive(rtc_get_counter_val()))
         {
            ChaDeMo::SetGeneralFault();
         }
         ChaDeMo::SetEnabled(false);
         timer_set_oc_value(FUELGAUGE_TIMER, TIM_OC2, 0);
         timer_set_oc_value(FUELGAUGE_TIMER, TIM_OC3, 0);
         Param::SetInt(Param::opmode, MOD_CHARGEND);
      }

      Param::SetInt(Param::udccdm, ChaDeMo::GetChargerOutputVoltage());
      Param::SetInt(Param::idccdm, ChaDeMo::GetChargerOutputCurrent());
      ChaDeMo::SendMessages(can);
   }
   Param::SetInt(Param::cdmstatus, ChaDeMo::GetChargerStatus());
   if (!LeafBMS::Alive(rtc_get_counter_val()))
   {
      ErrorMessage::Post(ERR_BMSCOMM);
   }
}

static void SetFuelGauge()
{
   int dcoffset = Param::GetInt(Param::gaugeoffset);
   int tmpaux = Param::GetInt(Param::tmpaux);
   s32fp dcgain = Param::Get(Param::gaugegain);
   int soctest = Param::GetInt(Param::soctest);
   int soc = Param::GetInt(Param::soc);
   soc = soctest != 0 ? soctest : soc;
   soc -= Param::GetInt(Param::gaugebalance);
   //Temperature compensation 1 digit per degree
   dcoffset -= tmpaux;
   int dc1 = FP_TOINT(dcgain * soc) + dcoffset;
   int dc2 = FP_TOINT(-dcgain * soc) + dcoffset;

   timer_set_oc_value(FUELGAUGE_TIMER, TIM_OC2, dc1);
   timer_set_oc_value(FUELGAUGE_TIMER, TIM_OC3, dc2);
}

static void Ms100Task(void)
{
   DigIo::led_out.Toggle();
   iwdg_reset();
   s32fp cpuLoad = FP_FROMINT(scheduler->GetCpuLoad());
   Param::SetFlt(Param::cpuload, cpuLoad / 10);
   Param::SetInt(Param::lasterr, ErrorMessage::GetLastError());
   Param::SetInt(Param::tmpecu, AnaIn::tint.Get());

   LeafBMS::RequestNextFrame(can);
   LeafBMS::Send100msMessages(can);

   if (!LeafBMS::Alive(rtc_get_counter_val()))
   {
      Param::SetFlt(Param::chgcurlim, 0);
      Param::SetFlt(Param::chglim, 0);
   }

   ProcessCruiseControlButtons();
   RunChaDeMo();

   if (!chargeMode && rtc_get_counter_val() > 100)
   {
      if (Param::GetInt(Param::canperiod) == CAN_PERIOD_100MS)
         can->SendAll();
      //SendVAG100msMessage();
      SetFuelGauge();
   }
}

static void GetDigInputs()
{
   int canio = 0;

   if (Param::GetBool(Param::din_cruise))
      canio |= CAN_IO_CRUISE;
   //lock out start signal when charging
   if ((Param::GetBool(Param::din_start) || DigIo::start_in.Get()) && !DigIo::charge_in.Get())
      canio |= CAN_IO_START;
   if (Param::GetBool(Param::din_brake) || DigIo::brake_in.Get())
      canio |= CAN_IO_BRAKE;
   if (Param::GetBool(Param::din_forward))
      canio |= CAN_IO_FWD;
   if (Param::GetBool(Param::din_reverse))
      canio |= CAN_IO_REV;
   if (Param::GetBool(Param::din_bms))
      canio |= CAN_IO_BMS;

   Param::SetInt(Param::din_charge, DigIo::charge_in.Get());

   Param::SetInt(Param::canio, canio);
}

static void ProcessThrottle()
{
   int pot1 = AnaIn::throttle1.Get();
   int pot2 = AnaIn::throttle2.Get();
   int brakePressure = Param::GetInt(Param::brakepressure);

   brakePressure += Param::GetInt(Param::regenlevel) * 40;
   brakePressure = MIN(255, brakePressure);

   /* hard coded throttle redundance */
   /*if (pot2 > 50)
   {
      pot1 = MIN(pot1, pot2 * 2);
   }*/

   Param::SetInt(Param::pot, pot1);
   Param::SetInt(Param::pot2, pot2);
   Param::SetInt(Param::potbrake, brakePressure);
}

static void Ms10Task(void)
{
   static int seq1Ctr = 0;
   static uint16_t consumptionCounter = 0;
   static uint32_t accumulatedRegen = 0;
   static int speedTimeout;
   static int dcdcDelay = 100;
   int vacuumthresh = Param::GetInt(Param::vacuumthresh);
   int vacuumhyst = Param::GetInt(Param::vacuumhyst);
   int vacuum = AnaIn::vacuum.Get();
   int speed = Param::GetInt(Param::speed);
   int opmode = Param::GetInt(Param::opmode);
   s32fp tmpm = Param::Get(Param::tmpm);
   s32fp idc = Param::Get(Param::idc);
   s32fp udcbms = Param::Get(Param::udcbms);
   s32fp udccdm = Param::Get(Param::udccdm);
   s32fp power = FP_MUL(idc, udcbms) / 1000;
   s32fp dcdcVoltage = 0;
   int32_t consumptionIncrement = -FP_TOINT(FP_MUL(power, FP_FROMFLT(2.8)));

   seq1Ctr = (seq1Ctr + 1) & 0x3;

   //Obviously the petrol consumption counter cannot handle
   //negative values. So we accumulate regen energy and
   //subtract it from the consumption once we're out of regen
   if (consumptionIncrement >= 0)
   {
      if (accumulatedRegen > (uint32_t)consumptionIncrement)
      {
         accumulatedRegen -= consumptionIncrement;
         consumptionIncrement = 0;
      }
      else if (accumulatedRegen > 0) //greater 0 but less then current draw
      {
         consumptionIncrement -= accumulatedRegen;
         accumulatedRegen = 0;
      }
      consumptionCounter += consumptionIncrement;
   }
   else
   {
      accumulatedRegen += -consumptionIncrement;
   }

   Param::SetFlt(Param::power, power);

   if (opmode < MOD_CHARGESTART)
   {
      if (vacuum > vacuumthresh)
      {
         DigIo::vacuum_out.Set();
      }
      else if (vacuum < vacuumhyst)
      {
         DigIo::vacuum_out.Clear();
      }
   }

   if (speed == 0 && opmode == MOD_RUN)
   {
      if (speedTimeout > 0)
      {
         speedTimeout--;
      }
      else if (Param::Get(Param::tmpaux) < Param::Get(Param::heathresh))
      {
         s32fp heatmax = Param::Get(Param::heatmax);
         s32fp heatcur = (heatmax - tmpm) * 50;
         s32fp heatcurmax = Param::Get(Param::heatpowmax);

         heatcur = MIN(heatcurmax, heatcur);
         Param::SetFlt(Param::heatcur, heatcur);
      }
      else
      {
         Param::SetFlt(Param::heatcur, 0);
      }
   }
   else
   {
      speedTimeout = 100;
      Param::SetFlt(Param::heatcur, 0);
   }

   if (Param::GetBool(Param::dashlight))
   {
      DigIo::dash_out.Set();
   }
   else
   {
      DigIo::dash_out.Clear();
   }

   if (opmode == MOD_OFF)
   {
      //Do not drop DC contactor in off mode because pre-charge
      //is always on and we will burn the precharge resistor
      //because the DC/DC converter still draws power
      Param::SetInt(Param::speedmod, 0);
      dcdcDelay = 100;
      timer_set_oc_value(PWM_TIMER, TIM_OC3, 0);
   }
   else if (opmode == MOD_RUN)
   {
      DigIo::dcsw_out.Set();
      DigIo::dash_out.Set();
      Param::SetFlt(Param::speedmod, MAX(FP_FROMINT(700), Param::Get(Param::speed)));
      Param::SetInt(Param::din_forward, !DigIo::charge_in.Get()); //lock out drive

      if (dcdcDelay > 0)
      {
         dcdcDelay--;
      }
      else
      {
         dcdcVoltage = Param::Get(Param::udcdc);
      }

      if (speed < Param::GetInt(Param::avasstop))
      {
         timer_set_oc_value(PWM_TIMER, TIM_OC3, Param::GetInt(Param::avasdc));
         timer_set_period(PWM_TIMER, Param::GetInt(Param::avasfrq));
      }
      else
      {
         timer_set_oc_value(PWM_TIMER, TIM_OC3, 0);
      }
   } //In charge mode charger will report DC bus voltage
   else if (udcbms > 0 && udccdm >= (udcbms - FP_FROMFLT(10)))
   {
      DigIo::dcsw_out.Set();
   }

   s32fp cur = FP_DIV((1000 * Param::Get(Param::chglim)), Param::Get(Param::udcbms));
   cur = FP_MUL(cur, Param::Get(Param::powerslack));
   Param::SetInt(Param::vacuum, vacuum);
   Param::SetFlt(Param::tmpmod, FP_FROMINT(48) + ((Param::Get(Param::tmpm) * 4) / 3));
   Param::SetFlt(Param::chgcurlim, cur);
   cur = FP_DIV((1000 * Param::Get(Param::dislim)), Param::Get(Param::udcbms));
   cur = FP_MUL(cur, Param::Get(Param::powerslack));
   Param::SetFlt(Param::discurlim, cur);
   Param::SetInt(Param::dout_dcsw, DigIo::dcsw_out.Get());

   GetDigInputs();
   ProcessThrottle();
   SendLin();

   ErrorMessage::SetTime(rtc_get_counter_val());

   LeafBMS::Send10msMessages(can, dcdcVoltage);

   if (Param::GetInt(Param::canperiod) == CAN_PERIOD_10MS)
      can->SendAll();
}

void Ms1Task()
{
   int ctr = 0;

   if (ctr == 0)
   {
      DigIo::speed_out.Toggle();
      ctr = Param::GetInt(Param::revtest);
   }

   ctr--;
}

/** This function is called when the user changes a parameter */
extern void parm_Change(Param::PARAM_NUM paramNum)
{
   if (Param::canspeed == paramNum)
      can->SetBaudrate((Can::baudrates)Param::GetInt(Param::canspeed));
}

static void CanCallback(uint32_t id, uint32_t data[2])
{
   switch (id)
   {
   case 0x108:
      ChaDeMo::Process108Message(data);
      break;
   case 0x109:
      ChaDeMo::Process109Message(data);
      break;
   case 0x420:
      Param::SetFlt(Param::tmpaux, FP_FROMINT((((data[0] >> 8) & 0xFF) - 100)) / 2);
      break;
   default:
      LeafBMS::DecodeCAN(id, data, rtc_get_counter_val());
      break;
   }
}

static void ConfigureVariantIO()
{
   ANA_IN_CONFIGURE(ANA_IN_LIST);
   DIG_IO_CONFIGURE(DIG_IO_LIST);

   AnaIn::Start();
}

extern "C" void tim2_isr(void)
{
   scheduler->Run();
}

//C++ run time requires that when using interfaces and not optimizing for size
extern "C" void __cxa_pure_virtual() { while (1); }

extern "C" int main(void)
{
   extern const TERM_CMD termCmds[];

   clock_setup();
   rtc_setup();
   write_bootloader_pininit();
   ConfigureVariantIO();
   tim_setup();
   nvic_setup();
   parm_load();

   LinBus l(USART1, 9600);
   Can c(CAN1, (Can::baudrates)Param::GetInt(Param::canspeed));

   c.SetNodeId(2);
   c.SetReceiveCallback(CanCallback);
   c.RegisterUserMessage(0x7BB);
   c.RegisterUserMessage(0x1DB);
   c.RegisterUserMessage(0x1DC);
   c.RegisterUserMessage(0x55B);
   c.RegisterUserMessage(0x5BC);
   c.RegisterUserMessage(0x5C0);
   c.RegisterUserMessage(0x108);
   c.RegisterUserMessage(0x109);
   c.RegisterUserMessage(0x420);

   can = &c;
   lin = &l;

   Stm32Scheduler s(TIM2); //We never exit main so it's ok to put it on stack
   scheduler = &s;

   Terminal t(USART3, termCmds);

   //s.AddTask(Ms1Task, 1);
   s.AddTask(Ms10Task, 10);
   s.AddTask(Ms100Task, 100);
   s.AddTask(Ms500Task, 500);

   Param::SetInt(Param::version, 4); //backward compatibility
   Param::SetInt(Param::dislim, -1);
   parm_Change(Param::PARAM_LAST);

   //extern uint8_t lindata[15];

   while(1)
   {
      t.Run();
   }

   return 0;
}

