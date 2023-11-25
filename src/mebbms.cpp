/*
 * This file is part of the stm32-... project.
 *
 * Copyright (C) 2021 Johannes Huebner <dev@johanneshuebner.com>
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
/* This code is based on Tom's VWBms implementation: https://github.com/Tom-evnut/VW-bms */
#include "mebbms.h"
#include "my_math.h"

#define FIRST_VTG_ID    0x1C0

const uint16_t socCurve[] =
{  /* 0%   5%   10%   15%   20%   25%   30%   35%   40%   45%   50%   55%   60%   65%   70%   75%   80%   85%   90%   95%  100%*/
   3380, 3426, 3547, 3622, 3671, 3694, 3703, 3719, 3734, 3757, 3777, 3808, 3848, 3890, 3931, 3977, 4033, 4082, 4120, 4168, 4200
};

MebBms::MebBms(CanHardware* c)
: canHardware(c), balancerRunning(false), balCounter(0), lastSettlingSampleTime(0)
{
   for (int i = 0; i < NumCells; i++)
      cellVoltages[i] = 0;

   for (int i = 0; i < (NumCells / 12); i++)
   {
      temps[i] = 0;
      lastReceived[i] = 0;
      balFlags[i] = 0;
   }

   canHardware->AddCallback(this);
   HandleClear();
}

void MebBms::HandleClear()
{
   //We just talk to the larger 48 cell CMUs and ignore the smaller 12 cell one at 1B0
   canHardware->RegisterUserMessage(0x1C0, 0x7F0);
   canHardware->RegisterUserMessage(0x1D0, 0x7F0);
   //These messages contain one temperature per CMU
   canHardware->RegisterUserMessage(0x1A5555F4);
   canHardware->RegisterUserMessage(0x1A5555F5);
   canHardware->RegisterUserMessage(0x1A5555F6);
   canHardware->RegisterUserMessage(0x1A5555F7);
   canHardware->RegisterUserMessage(0x1A5555F8);
   canHardware->RegisterUserMessage(0x1A5555F9);
   canHardware->RegisterUserMessage(0x1A5555FA);
   canHardware->RegisterUserMessage(0x1A5555FB);
}

bool MebBms::HandleRx(uint32_t canId, uint32_t data[2], uint8_t)
{
   if (canId >= FIRST_VTG_ID && canId <= 0x1DF)
   {
      if ((canId & 3) != 3 && !balancerRunning) //every 4th message contains data we don't need
      {
         //every message contains 4 voltages. Here we calculate which group we are in.
         //Every 4th message is not relevant to us and must be excluded from this calculation
         //We do so by multiplying with 3/4 and rounding up
         int group = ((canId - FIRST_VTG_ID + 1) * 3) / 4;
         SetCellVoltage(group * 4, ((data[0] >> 12) & 0xFFF) + 1000);
         SetCellVoltage(group * 4 + 1, ((data[0] >> 24) | ((data[1] & 0xF) << 8)) + 1000);
         SetCellVoltage(group * 4 + 2, ((data[1] >> 4) & 0xFFF) + 1000);
         SetCellVoltage(group * 4 + 3, ((data[1] >> 16) & 0xFFF) + 1000);

         if (canId == FIRST_VTG_ID)
            Accumulate();
      }
      return true;
   }
   else if (canId >= 0x1A5555F4 && canId <= 0x1A5555FB)
   {
      int cmu = (canId & 0xF) - 4;
      temps[cmu] = ((data[1] >> 4) & 0xFF) * 0.5f - 40;
      lastReceived[cmu] = canHardware->GetLastRxTimestamp();

      if (cmu == 0)
      {
         float min = 100.0f, max = -100.0f;

         for (int i = 0; i < 8; i++)
         {
            min = MIN(min, temps[i]);
            max = MAX(max, temps[i]);
         }
         lowTemp = min;
         highTemp = max;
      }
      return true;
   }
   return false;
}

float MebBms::GetMaximumChargeCurrent()
{
   const float lowTempDerate = LowTempDerating();
   const float highTempDerate = HighTempDerating();
   const float cc1Current = 275.0f * lowTempDerate;
   const uint16_t cv1Voltage = 3950;
   const float cc2Current = 170.0f * lowTempDerate;
   const uint16_t cv2Voltage = 4050;
   const float cc3Current = 114.0f * lowTempDerate;
   const uint16_t cv3Voltage = 4200;
   float result;

   /* Here we try to mimic VWs charge curve for a warm battery.
    *
    * We run 3 subsequent CC-CV curves
    * 1st starts at cc1Current and aims for cv1Voltage
    * 2nd starts at cc2Current and aims for cv2Voltage
    * 3rd starts at cc3Current and aims for cv3Voltage
    *
    * Low temperature derating is done by scaling down the CC values
    * High temp derating is done by generally capping charge current
    */

   float cv1Result = (cv1Voltage - maxCellVoltage) * 6; //P-controller gain factor 6 A/mV
   cv1Result = MIN(cv1Result, cc1Current);

   float cv2Result = (cv2Voltage - maxCellVoltage) * 2;
   cv2Result = MIN(cv2Result, cc2Current);

   float cv3Result = (cv3Voltage - maxCellVoltage) * 2;
   cv3Result = MIN(cv3Result, cc3Current);
   cv3Result = MAX(cv3Result, 0);

   result = MAX(cv1Result, MAX(cv2Result, cv3Result));
   result *= highTempDerate;

   return result;
}

float MebBms::GetMaximumDischargeCurrent()
{
   const float highTempDerate = HighTempDerating();
   const float maxDischargeCurrent = 500;
   const float cellVoltageCutoff = 3380;
   float result = (minCellVoltage - cellVoltageCutoff) * 5;
   result = MIN(maxDischargeCurrent, result);
   result = MAX(result, 0);
   result *= highTempDerate;

   return result;
}

float MebBms::EstimateSocFromVoltage()
{
   int n = sizeof(socCurve) / sizeof(socCurve[0]);

   for (int i = 0; i < n; i++)
   {
      if (minCellVoltage < socCurve[i])
      {
         if (i == 0) return 0;

         float soc = i * 10;
         float lutDiff = socCurve[i] - socCurve[i - 1];
         float valDiff = socCurve[i] - minCellVoltage;
         //interpolate
         soc -= (valDiff / lutDiff) * 5;
         return soc;
      }
   }
   return 100;
}

bool MebBms::CellVoltagesSettled(float current, uint32_t time)
{
   if (ABS(current) < 1 && lastSettlingSampleTime == 0)
   {
      lastSettlingSampleTime = time;
      lastUpperCellVoltage = maxCellVoltage;
      lastLowerCellVoltage = minCellVoltage;
   }
   else
   {
      lastSettlingSampleTime = 0;
      return false;
   }

   //Once every 30s we consider the cell voltages settled if there hasn't been more than 2mV movement
   if ((time - lastSettlingSampleTime) > 3000) //10ms ticks, 30s
   {
      int16_t diff1 = (int)lastUpperCellVoltage - (int)maxCellVoltage;
      diff1 = ABS(diff1);
      int16_t diff2 = (int)lastLowerCellVoltage - (int)minCellVoltage;
      diff2 = ABS(diff2);
      lastUpperCellVoltage = maxCellVoltage;
      lastLowerCellVoltage = minCellVoltage;
      lastSettlingSampleTime = time;

      return diff1 <= 2 && diff2 <= 2;
   }
   return false;
}

void MebBms::Balance(bool enable)
{
   const uint16_t balHyst = 4;
   const uint16_t balMin = 3800;
   uint8_t balCmds[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFE, 0xFE, 0xFE, 0xFE };
   bool balance = enable && balCounter < 11; //toggle balancing for accurate cell voltage measurement
   bool balancing = false;

   balCounter++;
   balCounter &= 0xF;

   for (int i = 0; i < NumCells; i++)
   {
      const int group = i / CellsPerCmu;
      const int cell = i % CellsPerCmu;
      const bool balFlag = (cellVoltages[i] > (minCellVoltage + balHyst)) && (cellVoltages[i] > balMin);

      balCmds[cell] = balFlag && balance ? 0x8 : 0x0;
      balancing |= balFlag && balance;

      if (cell == 0) balFlags[group] = 0;
      balFlags[group] |= balFlag << cell;

      if (cell == 7) //first 8 cells calculated -> send cmd
      {
         uint32_t canId = (group < 5 ? 0x1A555412 : 0x1A5554A1) + group * 2;
         canHardware->Send(canId, (uint32_t*)balCmds);
      }
      else if (cell == (CellsPerCmu - 1))
      {
         uint32_t canId = (group < 5 ? 0x1A555413 : 0x1A5554A2) + group * 2;
         canHardware->Send(canId, (uint32_t*)&balCmds[8]);
      }
   }

   balancerRunning = balancing;
}

bool MebBms::Alive(uint32_t time)
{
   uint32_t lastRecv = time;

   for (int i = 0; i < (NumCells / 12); i++)
   {
      lastRecv = MIN(lastReceived[i], lastRecv);
   }
   return (time - lastRecv) < 100;
}

void MebBms::Accumulate()
{
   int min = 4500, max = 0, sum = 0;

   for (int i = 0; i < NumCells; i++)
   {
      uint16_t voltage = GetCellVoltage(i);
      sum += voltage;
      min = MIN(min, voltage);
      max = MAX(max, voltage);
   }

   minCellVoltage = min;
   maxCellVoltage = max;
   totalVoltage = sum;
}

float MebBms::LowTempDerating()
{
   const float drt1Temp = 21.0f;
	const float drt2Temp = 0;
	const float drt3Temp = -20.0f;
	const float factorAtDrt2 = 0.3f;
	float factor;

	//We allow the ideal charge curve above 21°C
	if (lowTemp > drt1Temp)
	   factor = 1;
	else if (lowTemp > drt2Temp)
	   factor = factorAtDrt2 + (1 - factorAtDrt2) * (lowTemp - drt2Temp) / (drt1Temp - drt2Temp);
	else if (lowTemp > drt3Temp)
	   factor = factorAtDrt2 * (lowTemp - drt3Temp) / (drt2Temp - drt3Temp);
	else
	   factor = 0; //inhibit charging below -20°C

	return factor;
}

float MebBms::HighTempDerating()
{
   const float maxTemp = 45.0f;
	float factor = (maxTemp - highTemp) * 0.4f;
	factor = MIN(1, factor);
	factor = MAX(0, factor);

	return factor;
}

void MebBms::SetCellVoltage(int idx, float vtg)
{
   cellVoltages[idx] = IIRFILTERF((float)cellVoltages[idx], vtg, 2) + 0.5f;
}
