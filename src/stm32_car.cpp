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
#include "chademo.h"
#include "linbus.h"
#include "picontroller.h"
#include "my_string.h"

#define RMS_SAMPLES 256
#define SQRT2OV1 0.707106781187
#define PRECHARGE_TIMEOUT 500 //5s
#define CAN_TIMEOUT       50  //500ms

static Stm32Scheduler* scheduler;
static bool chargeMode = false;
static Can* can;
static LinBus* lin;
static PiController fuelGaugeController;

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

   if (cruisestt & CRUISE_ON && Param::GetInt(Param::opmode) == MOD_RUN)
   {
      if (cruisespeed <= 0)
      {
         int currentSpeed = Param::GetInt(Param::speed);

         if (cruisestt & CRUISE_SETN && currentSpeed > 500) //Start cruise control at current speed
         {
            cruiseTarget = currentSpeed;
            cruisespeed = cruiseTarget;
         }
         else if (cruisestt & CRUISE_SETP && cruiseTarget > 0) //resume via ramp
         {
            cruisespeed = currentSpeed;
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

   if (Param::GetInt(Param::invmode) == INVMOD_CHARGE)
   {
      chargeMode = true;
      Param::SetInt(Param::opmode, MOD_CHARGESTART);
   }

   /* 1s after entering charge mode, enable charge permission */
   if (Param::GetInt(Param::opmode) == MOD_CHARGESTART && rtc_get_counter_val() > 200)
   {
      ChaDeMo::SetEnabled(true);
      ChaDeMo::SetContactor(true);
      Param::SetInt(Param::opmode, MOD_CHARGE);
   }

   if (Param::GetInt(Param::opmode) == MOD_CHARGE)
   {
      int chargeCur = Param::GetInt(Param::chgcurlim);
      int chargeLim = Param::GetInt(Param::chargelimit);
      chargeCur = MIN(MIN(255, chargeLim), chargeCur);
      ChaDeMo::SetChargeCurrent(chargeCur);

      if (Param::GetBool(Param::cdmcheckena))
         ChaDeMo::CheckSensorDeviation(Param::GetInt(Param::udcbms));
   }

   if (Param::GetInt(Param::opmode) == MOD_CHARGEND)
   {
      ChaDeMo::SetChargeCurrent(0);
   }

   ChaDeMo::SetTargetBatteryVoltage(Param::GetInt(Param::udclimit));
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

static void SendVAG100msMessage()
{
   static int seqCtr = 0;
   static uint8_t ctr = 0;

   const uint8_t seq1[] = { 0x0f, 0x28, 0x7f, 0x28 };
   const uint8_t seq2[] = { 0x1e, 0x10, 0x00, 0x10 };
   const uint8_t seq3[] = { 0x70, 0x56, 0xf0, 0x56 };
   const uint8_t seq4[] = { 0x0c, 0x48, 0xa7, 0x48 };
   const uint8_t seq5[] = { 0x46, 0x90, 0x28, 0x90 };

   uint8_t canData[8] = { (uint8_t)(0x80 | ctr), 0, 0, seq1[seqCtr], seq2[seqCtr], seq3[seqCtr], seq4[seqCtr], seq5[seqCtr] };

   can->Send(0x580, (uint32_t*)canData);
   seqCtr = (seqCtr + 1) & 0x3;
   ctr = (ctr + 1) & 0xF;
}

static void SetFuelGauge()
{
   static int startupDelay = 500;
   int counts = Param::GetInt(Param::gaugefrq);
   int fuelPos = Param::GetInt(Param::fuelpos);
   int fuelMax = Param::GetInt(Param::gaugemax);
   int soctest = Param::GetInt(Param::soctest);
   int soc = soctest > 0 ? soctest : Param::GetInt(Param::soc);
   int targetPos = (fuelMax * soc) / 100;

   if (startupDelay > 0)
   {
      startupDelay--;
      int preload = Param::GetInt(Param::fueldcmin) + (Param::GetInt(Param::fueldcmax) - Param::GetInt(Param::fueldcmin)) / 2;
      fuelGaugeController.PreloadIntegrator(preload);
   }

   if (fuelPos > fuelMax)
   {
      fuelGaugeController.PreloadIntegrator(500);
   }
   else
   {
      fuelGaugeController.SetRef(FP_FROMINT(fuelPos));
   }

   int dc = fuelGaugeController.Run(FP_FROMINT(targetPos));
   dc *= counts;
   dc /= 1000;

   Param::SetInt(Param::tmphs, dc);

   timer_set_period(FUELGAUGE_TIMER, counts);
   timer_set_oc_value(FUELGAUGE_TIMER, TIM_OC2, dc);
   timer_set_oc_value(FUELGAUGE_TIMER, TIM_OC3, dc);
}

static void RunLin()
{
   static int state = 0;
   uint8_t data[8] = { 0xb3, 0x05, 0x00, 0x90, 0xff, 0x00, 0x00, 0x00 };

   if (lin->HasReceived(33, 8))
   {
      uint8_t* data = lin->GetReceivedBytes();

      Param::SetInt(Param::udcompressor, data[7] * 2);
   }

   switch (state)
   {
   case 0:
      lin->Request(17, 0, 0);
      break;
   case 1:
      lin->Request(33, 0, 0);
      break;
   case 2:
      lin->Request(35, 0, 0);
      break;
   case 3:
      lin->Request(38, 0, 0);
      break;
   case 4:
      memset32((int*)data, 0, 2);
      data[0] = 1;
      lin->Request(32, data, 8);
      break;
   case 5:
      lin->Request(59, data, 8);
      break;
   }

   state = (state + 1) % 6;
}


static void Ms100Task(void)
{
   DigIo::led_out.Toggle();
   iwdg_reset();
   float cpuLoad = scheduler->GetCpuLoad();
   Param::SetFloat(Param::cpuload, cpuLoad / 10);
   Param::SetInt(Param::lasterr, ErrorMessage::GetLastError());

   LeafBMS::RequestNextFrame(can);
   LeafBMS::Send100msMessages(can);

   if (!LeafBMS::Alive(rtc_get_counter_val()))
   {
      Param::SetInt(Param::chgcurlim, 0);
      Param::SetInt(Param::chglim, 0);
   }

   ProcessCruiseControlButtons();
   RunChaDeMo();
   RunLin();

   bool start = Param::GetInt(Param::invmode) == MOD_OFF;
   start &= Param::GetInt(Param::udcinv) >= (Param::GetInt(Param::udcbms) - Param::GetInt(Param::bmsinvdiff));

   Param::SetInt(Param::din_start, start);

   if (Param::GetInt(Param::canperiod) == CAN_PERIOD_100MS)
      can->SendAll();
   SendVAG100msMessage();
}

static void ReadDirectionButtons()
{
   int drivesel = AnaIn::drivesel.Get();

   //Forward button
   if (drivesel > 2500 && drivesel < 3500)
   {
      drivesel = DIR_FORWARD;
   }
   //Reverse button
   else if (drivesel > 1500 && drivesel < 2400)
   {
      drivesel = DIR_REVERSE;
   }
   //Neutral button
   else if (drivesel > 500 && drivesel < 1400)
   {
      drivesel = DIR_NEUTRAL;
   }
   else
   {
      drivesel = DIR_NONE;
   }

   Param::SetInt(Param::drivesel, drivesel);
}

static void GetDigInputs()
{
   static int lastDrivesel = DIR_NONE;
   int drivesel = Param::GetInt(Param::drivesel);
   int invdir = Param::GetInt(Param::invdir);
   int speed = Param::GetInt(Param::speed);
   int cruisemode = Param::GetInt(Param::cruisestt) & CRUISE_ON;
   int canio = 0;

   //Forward button
   if (drivesel == DIR_FORWARD)
   {
      //while driving use as cruise control/regen adjust
      if (invdir == DIR_FORWARD && speed > 500)
      {
         Param::SetInt(Param::cruisestt, cruisemode | CRUISE_SETP);
      }
      else if (Param::GetBool(Param::din_brake) || invdir == DIR_REVERSE)
      {
         Param::SetInt(Param::din_reverse, 0);
         Param::SetInt(Param::din_forward, 1);
      }
   }
   //Reverse button
   else if (drivesel == DIR_REVERSE)
   {
      //while driving use as cruise control/regen adjust
      if (invdir == DIR_FORWARD && speed > 500)
      {
         Param::SetInt(Param::cruisestt, cruisemode | CRUISE_SETN);
      }
      else if (Param::GetBool(Param::din_brake) || invdir == DIR_FORWARD)
      {
         Param::SetInt(Param::din_forward, 0);
         Param::SetInt(Param::din_reverse, 1);
      }
   }
   //Neutral button
   else if (drivesel == DIR_NEUTRAL)
   {
      if (speed == 0)
      {
         Param::SetInt(Param::din_forward, 1);
         Param::SetInt(Param::din_reverse, 1);
         Param::SetInt(Param::din_bms, 1); //prevents going into charge mode
      }
      /*else if (speed > 500 && lastDrivesel == DIR_NONE)
      {
         if (cruisemode)
            Param::SetInt(Param::cruisestt, 0);
         else
            Param::SetInt(Param::cruisestt, CRUISE_ON);
      }*/
   }
   //No button
   else
   {
      Param::SetInt(Param::din_forward, 0);
      Param::SetInt(Param::din_reverse, 0);
      Param::SetInt(Param::cruisestt, cruisemode);
      Param::SetInt(Param::din_bms, invdir == DIR_REVERSE); //limit torque in reverse
   }

   lastDrivesel = drivesel;

   if (Param::GetBool(Param::din_cruise))
      canio |= CAN_IO_CRUISE;
   if (Param::GetBool(Param::din_start) || DigIo::start_in.Get())
      canio |= CAN_IO_START;
   if (Param::GetBool(Param::din_brake) || DigIo::brake_in.Get())
      canio |= CAN_IO_BRAKE;
   if (Param::GetBool(Param::din_forward))
      canio |= CAN_IO_FWD;
   if (Param::GetBool(Param::din_reverse))
      canio |= CAN_IO_REV;
   if (Param::GetBool(Param::din_bms))
      canio |= CAN_IO_BMS;

   Param::SetInt(Param::canio, canio);
}

static void TractionControl(float& throtmin, float& throtmax)
{
   if (!Param::GetBool(Param::espoff))
   {
      float frontAxleSpeed = (Param::GetFloat(Param::wheelfl) + Param::GetFloat(Param::wheelfr)) / 2;
      float rearAxleSpeed = (Param::GetFloat(Param::wheelrl) + Param::GetFloat(Param::wheelrr)) / 2;
      float diff = frontAxleSpeed - rearAxleSpeed;
      float kp = Param::GetFloat(Param::tractionkp);

      //Here we assume front wheel drive
      if (diff < 0)
      {
         //Front axle turns slower than rear axle -> too much breaking force
         float speedErr = Param::GetFloat(Param::allowedlag) - diff;
         throtmin = -100 + kp * speedErr;
      }
      else
      {
         //Front axle turns faster than rear axle -> wheel spin
         float speedErr = Param::GetFloat(Param::allowedspin) - diff;
         throtmax = 100 + kp * speedErr;
      }
   }
}

static void ProcessThrottle()
{
   int pot1 = AnaIn::throttle1.Get();
   int pot2 = AnaIn::throttle2.Get();
   int brakePressure = Param::GetInt(Param::brakepressure);
   int offPedalRegen = Param::GetInt(Param::regenlevel) * 60;

   brakePressure = MAX(offPedalRegen, brakePressure);
   brakePressure = MIN(255, brakePressure);

   /* hard coded throttle redundancy */
   if (pot2 > 50)
   {
      pot1 = MIN(pot1, pot2 * 2);

      if (ABS(2 * pot2 - pot1) > 200 && pot1 < 4090)
         Param::SetInt(Param::errlights, 4);
   }

   Param::SetInt(Param::pot, pot1);
   Param::SetInt(Param::pot2, pot2);
   Param::SetInt(Param::potbrake, brakePressure);
}

static void LimitThrottle()
{
   float throtmin = -100, throtmax = 100;

   TractionControl(throtmin, throtmax);

   throtmin = MIN(0, throtmin);
   throtmin = MAX(-100, throtmin);
   throtmax = MIN(100, throtmax);
   throtmax = MAX(0, throtmax);

   Param::SetFloat(Param::calcthrotmax, throtmax);
   Param::SetFloat(Param::calcthrotmin, throtmin);
}

static void SimulateOilSensor()
{
   static int ctr = 0;
   static int state = 0;

   switch (state)
   {
   case 0:
      DigIo::oilevel_out.Set();
      ctr = 2;
      state++;
      break;
   case 1:
      if (ctr == 0)
      {
         DigIo::oilevel_out.Clear();
         ctr = 3;
         state++;
      }
      ctr--;
      break;
   case 2:
      if (ctr == 0)
      {
         DigIo::oilevel_out.Set();
         ctr = 30;
         state++;
      }
      ctr--;
      break;
   case 3:
      if (ctr == 0)
      {
         DigIo::oilevel_out.Clear();
         state = 0;
      }
      ctr--;
      break;
   }
}

static void SetRevCounter()
{
   int regenLevel = Param::GetInt(Param::regenlevel);

   if (Param::GetInt(Param::invmode) != MOD_RUN || Param::GetInt(Param::invdir) == DIR_NEUTRAL)
   {
      DigIo::oilpres_out.Clear();
      Param::SetInt(Param::speedmod, 0);
   }
   else
   {
      DigIo::oilpres_out.Set();
      Param::SetInt(Param::speedmod, 1000 + regenLevel * 1000 - Param::GetInt(Param::idc) * 10);
   }
}

static void Ms10Task(void)
{
   const uint8_t seq2[] = { 0x10, 0x68, 0x94, 0xC0 };
   static int seq1Ctr = 0;
   static uint16_t consumptionCounter = 0;
   static uint32_t accumulatedRegen = 0;
   int vacuumthresh = Param::GetInt(Param::vacuumthresh);
   int vacuumhyst = Param::GetInt(Param::vacuumhyst);
   int vacuum = AnaIn::vacuum.Get();
   int invmode = Param::GetInt(Param::invmode);
   int cruiselight = Param::GetInt(Param::cruiselight);
   int errlights = Param::GetInt(Param::errlights);
   float idc = Param::GetFloat(Param::idc);
   float udcbms = Param::GetFloat(Param::udcbms);
   float power = (idc * udcbms) / 1000.0f;
   float dcdcVoltage = 0;
   int32_t consumptionIncrement = -power * 2.8f;

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
      else if (accumulatedRegen > 0) //greater 0 but less than current draw
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

   Param::SetFloat(Param::power, power);

   SimulateOilSensor();
   SetRevCounter();

   if (Param::GetInt(Param::invdir) == DIR_REVERSE)
   {
      DigIo::rev_out.Set();
   }
   else
   {
      DigIo::rev_out.Clear();
   }

   if (Param::GetInt(Param::invdir) == DIR_FORWARD)
   {
      DigIo::fwd_out.Set();
   }
   else
   {
      DigIo::fwd_out.Clear();
   }

   if (invmode == MOD_RUN)
   {
      if (vacuum > vacuumthresh)
      {
         DigIo::vacuum_out.Set();
      }
      else if (vacuum < vacuumhyst)
      {
         DigIo::vacuum_out.Clear();
      }
      Param::SetInt(Param::opmode, MOD_RUN);
   }
   else
   {
      DigIo::vacuum_out.Clear();
   }

   float cur = 1000 * Param::GetFloat(Param::chglim) / udcbms;
   cur *= Param::GetFloat(Param::powerslack);
   Param::SetInt(Param::vacuum, vacuum);
   Param::SetFloat(Param::chgcurlim, cur);
   cur = 1000 * Param::GetFloat(Param::dislim) / udcbms;
   cur *= Param::GetFloat(Param::powerslack);
   Param::SetFloat(Param::discurlim, cur);

   ReadDirectionButtons();
   GetDigInputs();
   ProcessThrottle();
   LimitThrottle();

   ErrorMessage::SetTime(rtc_get_counter_val());

   LeafBMS::Send10msMessages(can, dcdcVoltage);
   SetFuelGauge();

   uint32_t canData[2];

   //Byte1 seq 2, Byte ?, Byte 7 XOR(bytes[0..6])
   uint8_t check = seq2[seq1Ctr] ^ errlights ^ (consumptionCounter & 0xFF) ^ (consumptionCounter >> 8) ^ cruiselight ^ 0x1A;
   canData[0] = seq2[seq1Ctr] | errlights << 8 | consumptionCounter << 16;
   canData[1] = 0x1A | cruiselight << 18 | check << 24;

   can->Send(0x480, canData);

   if (Param::GetInt(Param::canperiod) == CAN_PERIOD_10MS)
      can->SendAll();
}

/** This function is called when the user changes a parameter */
void Param::Change(Param::PARAM_NUM paramNum)
{
   if (Param::canspeed == paramNum)
      can->SetBaudrate((Can::baudrates)Param::GetInt(Param::canspeed));

   fuelGaugeController.SetMinMaxY(Param::GetInt(Param::fueldcmin), Param::GetInt(Param::fueldcmax));
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
      Param::SetFloat(Param::tmpaux, (((data[0] >> 8) & 0xFF) - 100) / 2.0f);
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

   LinBus l(USART1, 19200);
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

   fuelGaugeController.SetGains(5, 20);
   fuelGaugeController.SetCallingFrequency(100);
   fuelGaugeController.SetMinMaxY(Param::GetInt(Param::fueldcmin), Param::GetInt(Param::fueldcmax));

   s.AddTask(Ms10Task, 10);
   s.AddTask(Ms100Task, 100);

   Param::SetInt(Param::version, 4); //COM protocol version 4
   Param::SetInt(Param::tmpaux, 87); //sends n/a value to Leaf BMS
   Param::SetInt(Param::heatcmd, 0); //Make sure we don't load this from flash
   Param::SetInt(Param::soc, 100); //Preload SoC for proper fuel gauge display

   while(1)
      t.Run();

   return 0;
}

