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
#include "params.h"
#include "my_math.h"

#define FIRST_VTG_ID    0x1C0


MebBms::MebBms(CanHardware* c)
: canHardware(c), balancerRunning(false)
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
         cellVoltages[group * 4] = ((data[0] >> 12) & 0xFFF) + 1000;
         cellVoltages[group * 4 + 1] = ((data[0] >> 24) | ((data[1] & 0xF) << 8)) + 1000;
         cellVoltages[group * 4 + 2] = ((data[1] >> 4) & 0xFFF) + 1000;
         cellVoltages[group * 4 + 3] = ((data[1] >> 16) & 0xFFF) + 1000;

         if (canId == FIRST_VTG_ID)
            Accumulate();
      }
      return true;
   }
   else if (canId >= 0x1A5555F4 && canId <= 0x1A5555FB)
   {
      int cmu = (canId & 0xF) - 4;
      temps[cmu] = ((data[1] >> 4) & 0xFF) * 0.5f - 40;
      Param::SetFloat(Param::tmpbat1, temps[0]);
      Param::SetFloat(Param::tmpbat2, temps[1]);
      Param::SetFloat(Param::tmpbat3, temps[2]);
      Param::SetFloat(Param::tmpbat4, temps[3]);
      Param::SetFloat(Param::tmpbat5, temps[4]);
      Param::SetFloat(Param::tmpbat6, temps[5]);
      Param::SetFloat(Param::tmpbat7, temps[6]);
      Param::SetFloat(Param::tmpbat8, temps[7]);
      lastReceived[cmu] = canHardware->GetLastRxTimestamp();
      return true;
   }
   return false;
}

void MebBms::Balance()
{
   const uint16_t balHyst = 4;
   const uint16_t balMin = 3800;
   uint16_t minVtg = Param::GetInt(Param::batmin);
   uint8_t balCmds[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFE, 0xFE, 0xFE, 0xFE };
   bool balance = Param::GetBool(Param::balance) && !balancerRunning; //toggle balancing for accurate cell voltage measurement

   if (!balancerRunning) balancerRunning = Param::GetBool(Param::balance);

   for (int i = 0; i < NumCells; i++)
   {
      const int group = i / CellsPerCmu;
      const int cell = i % CellsPerCmu;
      const bool balFlag = (cellVoltages[i] > (minVtg + balHyst)) && (cellVoltages[i] > balMin);

      balCmds[cell] = balFlag && balance ? 0x8 : 0x0;

      if (cell == 0) balFlags[group] = 0;
      balFlags[group] |= balFlag << cell;

      if (cell == 7) //first 8 cells calculated -> send cmd
      {
         uint32_t canId = 0x1A55540A + group * 2;
         canHardware->Send(canId, (uint32_t*)balCmds);
      }
      else if (cell == (CellsPerCmu - 1))
      {
         uint32_t canId = 0x1A55540B + group * 2;
         canHardware->Send(canId, (uint32_t*)&balCmds[8]);
      }
   }

   balancerRunning = balance;
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

   Param::SetInt(Param::batmin, min);
   Param::SetInt(Param::batmax, max);
   Param::SetInt(Param::batavg, sum / NumCells);
   Param::SetFloat(Param::udcbms, sum / 1000.0f);
}

