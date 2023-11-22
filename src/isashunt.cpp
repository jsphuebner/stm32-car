/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
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
#include "isashunt.h"

static int channel;

IsaShunt::IsaShunt(CanHardware* hw)
   : can(hw), current(0), state(Init)
{
   can = hw;
   channel = 0;
   can->AddCallback(this);
   HandleClear();
}

void IsaShunt::HandleClear()
{
   can->RegisterUserMessage(CAN_ID_REPLY);
   can->RegisterUserMessage(CAN_ID_CURRENT);
   can->RegisterUserMessage(CAN_ID_VOLTAGE);
   can->RegisterUserMessage(CAN_ID_POWER);
}

void IsaShunt::ResetCounters()
{
   uint32_t configData[2] = { 0x0030, 0 };

   Stop();
   can->Send(0x411, configData);
   Start();
}

void IsaShunt::RequestCharge()
{
   uint32_t configData[2] = { 0x0243, 0 };

   can->Send(0x411, configData);
   state = RequestChargeOut;
}

void IsaShunt::Start()
{
   uint32_t configData[2] = { 0x0134, 0 };

   can->Send(0x411, configData);
}

void IsaShunt::Stop()
{
   uint32_t configData[2] = { 0x0034, 0 };

   can->Send(0x411, configData);
}

bool IsaShunt::HandleRx(uint32_t id, uint32_t data[], uint8_t)
{
   bool isIsa = false;

   switch (id)
   {
   case CAN_ID_REPLY:
      RunStateMachine(data);
      isIsa = true;
      break;
   case CAN_ID_CURRENT:
      current = (data[0] >> 16) + (data[1] << 16);
      isIsa = true;
      break;
   case CAN_ID_VOLTAGE:
      voltage = (data[0] >> 16) + (data[1] << 16);
      isIsa = true;
      break;
   case CAN_ID_POWER:
      power = (data[0] >> 16) + (data[1] << 16);
      isIsa = true;
      break;
   }

   if (state == Init && channel == 0 && isIsa)
   {
      Stop();
   }

   return isIsa;
}

void IsaShunt::RunStateMachine(uint32_t data[])
{
   if (state == Init)
   {
      //Data channels
      //00: I [mA]
      //01: U1 [mV]
      //05: P [W]
      //06: As [As]
      //07: Wh [Wh]

      if (channel < 8)
      {
         //config channel I (0x20), little endian/disabled (0x40), 100ms (0x64, 0x00)
         uint32_t configData[2] = { 0x64004020, 0 };

         configData[0] |= channel;
         //Only enable I, U1 and P
         if (channel == 0 || channel == 1 || channel == 5)
            configData[0] |= 0x200; //cyclic trigger
         can->Send(0x411, configData);
         channel++;
      }
      else
      {
         Start();
         state = Operate;
      }
   }
   else if (state == RequestChargeOut) //Read charge in and request charge out
   {
      uint32_t configData[2] = { 0x0343, 0 };
      uint8_t* u8data = (uint8_t*)data;

      chargeIn = u8data[7] + (u8data[6] << 8) + (u8data[5] << 16) + (u8data[4] << 24);

      can->Send(0x411, configData);
      state = ReadChargeOut;
   }
   else if (state == ReadChargeOut)
   {
      uint8_t* u8data = (uint8_t*)data;
      chargeOut = u8data[7] + (u8data[6] << 8) + (u8data[5] << 16) + (u8data[4] << 24);
      state = Operate;
   }
}
