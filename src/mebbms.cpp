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

const uint16_t vtgToSoc[] =
{/*2.80V  2.85  2.90  2.95  3.00  3.05 3.10  3.15  3.20  3.25  3.30  3.35  3.40  3.45  3.50  3.55  3.60  3.65  3.70  3.75  3.80  3.85  3.90  3.95  4.00  4.05  4.10  4.15  4.20 */
   0,     12,   31,   55,   80,   116,	153,	202,	239,	325,	496,	701,	1231,	1794,	2307,	3368,	4388,	5368,	5949,	6326,	6742,	7232,	7708,	8104,	8575,	8996,	9446,	9753,	10000
};

const uint16_t socToSoe[] =
{/*0	5%	  10%    15%   20%   25%   30%   35%   40%  45%    50%   55%   60%   65%   70%   75%   80%   85%   90%   95%   100% */
   0,	415,	870,	1334,	1803,	2278,	2758,	3241,	3727,	4216,	4709,	5206,	5707,	6217,	6734,	7259,	7791,	8331,	8879,	9434,	10000
};

static const uint16_t tableMinVtg = 2800; //mV
static const uint16_t tableMaxVtg = 4200; //mV
static const uint8_t socCurveTableItems = sizeof(vtgToSoc) / sizeof(vtgToSoc[0]);
static const uint8_t socCurveGranularity = 50; //mV between entries
static const uint8_t energyCurveTableItems = sizeof(socToSoe) / sizeof(socToSoe[0]);
static const float energyCurveGranularity = 100.0f / (energyCurveTableItems - 1);

MebBms::MebBms(CanHardware* c)
: canHardware(c), maxCellVoltage(0), minCellVoltage(0), totalVoltage(0), maxAh(148),
  balancerRunning(false), balCounter(0)
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

float MebBms::GetMaximumChargeCurrent(float cellmax)
{
   const float lowTempDerate = LowTempDerating();
   const float highTempDerate = HighTempDerating();
   const float cc1Current = 275.0f * lowTempDerate;
   const uint16_t cv1Voltage = 3960;
   const float cc2Current = 170.0f * lowTempDerate;
   const uint16_t cv2Voltage = 4050;
   const float cc3Current = 114.0f * lowTempDerate;
   const uint16_t cv3Voltage = cellmax;
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

   float cv1Result = (cv1Voltage - maxCellVoltage) * 2; //P-controller gain factor 2 A/mV
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

float MebBms::GetMaximumDischargeCurrent(float cellVoltageCutoff)
{
   const float highTempDerate = HighTempDerating();
   const float maxDischargeCurrent = 500;
   float result = (minCellVoltage - cellVoltageCutoff) * 5;
   result = MIN(maxDischargeCurrent, result);
   result = MAX(result, 0);
   result *= highTempDerate;

   return result;
}

float MebBms::EstimateSocFromVoltage()
{
   int lookupVoltage = MIN(tableMaxVtg, minCellVoltage) - tableMinVtg;
   lookupVoltage = MAX(socCurveGranularity, lookupVoltage);
   int socIndex = lookupVoltage / socCurveGranularity;
   float socFraction = (lookupVoltage - (socIndex * socCurveGranularity)) / (float)socCurveGranularity; //where are we in the segment?
   float diff = vtgToSoc[socIndex + 1] - vtgToSoc[socIndex];
   float soc = vtgToSoc[socIndex] + diff * socFraction;

   return soc / 100;
}

/** \brief Calculates the theoretically remaining energy in Wh at the given SoC
 *
 * \param soc state of charge
 * \return float
 *
 */
float MebBms::GetRemainingEnergy(float soc)
{
   const float nominalVoltage = 3.67f;
   int socIndex = soc / energyCurveGranularity;
   float socFraction = (soc - (socIndex * energyCurveGranularity)) / (float)energyCurveGranularity; //where are we in the segment?
   socIndex = MIN(socIndex, socCurveTableItems);
   socIndex = MAX(socIndex, 0);
   float energyAtSoc;

   if (socIndex < (socCurveTableItems - 1)) //less than 100% SoC
   {
      float diff = socToSoe[socIndex + 1] - socToSoe[socIndex];
      energyAtSoc = (float)socToSoe[socIndex] + diff * socFraction;
   }
   else //100%
   {
      energyAtSoc = socToSoe[socCurveTableItems - 1];
   }
   //divide by 10000 because SoE curve unit is 100 * 100%
   return energyAtSoc * nominalVoltage * NumCells * GetMaximumAmpHours() / 10000;
}

void MebBms::Balance(bool enable)
{
   const uint16_t balHyst = 4;
   const uint16_t balMin = 3730;
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
   float min = 4500, max = 0, sum = 0;

   for (int i = 0; i < NumCells; i++)
   {
      float voltage = cellVoltages[i];
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
   const float drt1Temp = 25.0f;
   const float drt2Temp = 0;
   const float drt3Temp = -20.0f;
   const float factorAtDrt2 = 0.3f;
   float factor;

   //We allow the ideal charge curve above 25°C
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
   const float maxTemp = 50.0f;
   float factor = (maxTemp - highTemp) * 0.4f;
   factor = MIN(1, factor);
   factor = MAX(0, factor);

   return factor;
}

void MebBms::SetCellVoltage(int idx, int vtg)
{
   if (vtg < 1000 || vtg > 4400) return; //Ignore implausible values
   if (cellVoltages[idx] == 0)
      cellVoltages[idx] = vtg; //The first value initializes the filter to speed up settle time
   else
      cellVoltages[idx] = IIRFILTERF(cellVoltages[idx], (float)vtg, 2);
}
