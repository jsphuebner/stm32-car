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
#include "mebbms.h"
#include "params.h"
#include "my_math.h"

#define FIRST_VTG_ID    0x1C0


MebBms::MebBms(CanHardware* c)
: canHardware(c)
{
   for (int i = 0; i < NumCells; i++)
      cellVoltages[i] = 0;
   canHardware->AddCallback(this);
   HandleClear();
}

void MebBms::HandleClear()
{
   canHardware->RegisterUserMessage(0x1C0, 0x7F0);
   canHardware->RegisterUserMessage(0x1D0, 0x7F0);
   canHardware->RegisterUserMessage(0x1A5555F4);
   canHardware->RegisterUserMessage(0x1A5555F5);
   canHardware->RegisterUserMessage(0x1A5555F6);
}

bool MebBms::HandleRx(uint32_t canId, uint32_t data[2], uint8_t)
{
   if (canId >= FIRST_VTG_ID && canId <= 0x1DF)
   {
      if ((canId & 3) != 3)
      {
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
   }
   return false;
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

