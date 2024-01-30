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

enum canids
{
   CAN_ID_REPLY = 0x511,
   CAN_ID_CURRENT = 0x521,
   CAN_ID_VOLTAGE1 = 0x522,
   CAN_ID_VOLTAGE2 = 0x523,
   CAN_ID_VOLTAGE3 = 0x524,
   CAN_ID_TEMP = 0x525,
   CAN_ID_POWER = 0x526,
   CAN_ID_CURINT = 0x527,
   CAN_ID_POWINT = 0x528,
   CAN_ID_CONFIG = 0x411
};

static int channel;

IsaShunt::IsaShunt(CanHardware* hw, uint8_t enabledChannels)
   : can(hw), current(0), initialized(false), started(false), enableMask(enabledChannels)
{
   can = hw;
   channel = 0;
   can->AddCallback(this);
   HandleClear();
   Stop();
}

void IsaShunt::HandleClear()
{
   can->RegisterUserMessage(CAN_ID_REPLY);

   for (uint8_t chan = 0; chan < 8; chan++)
      if ((1 << chan) & enableMask)
         can->RegisterUserMessage(CAN_ID_CURRENT + chan);
}

void IsaShunt::ResetCounters()
{
   //Only reset if something was accumulated
   if (initialized && GetValue(AS) != 0)
   {
      uint32_t configData[2] = { 0x0030, 0 };

      Stop();
      can->Send(CAN_ID_CONFIG, configData);
      Start();
   }
}

void IsaShunt::Start()
{
   uint32_t configData[2] = { 0x0134, 0 };

   can->Send(CAN_ID_CONFIG, configData);
}

void IsaShunt::Stop()
{
   uint32_t configData[2] = { 0x0034, 0 };

   can->Send(CAN_ID_CONFIG, configData);
   started = false;
}

int32_t IsaShunt::GetValue(channels chan)
{
   int32_t result = 0;
   uint32_t dummy[2] = { 0, 0 };

   if (!initialized) Configure(dummy);
   if (initialized && !started) Start();

   switch (chan)
   {
   case CURRENT:
      result = current;
      break;
   case U1:
      result = voltages[0];
      break;
   case U2:
      result = voltages[1];
      break;
   case U3:
      result = voltages[2];
      break;
   case TEMP:
      result = temp;
      break;
   case POWER:
      result = power;
      break;
   case AS:
      result = currentIntegral;
      break;
   case WH:
      result = powerIntegral;
      break;
   }
   return result;
}

bool IsaShunt::HandleRx(uint32_t id, uint32_t data[], uint8_t)
{
   bool isIsa = true;

   switch (id)
   {
   case CAN_ID_REPLY:
      Configure(data);
      break;
   case CAN_ID_CURRENT:
      current = (data[0] >> 16) + (data[1] << 16);
      break;
   case CAN_ID_VOLTAGE1:
      voltages[0] = (data[0] >> 16) + (data[1] << 16);
      break;
   case CAN_ID_VOLTAGE2:
      voltages[1] = (data[0] >> 16) + (data[1] << 16);
      break;
   case CAN_ID_VOLTAGE3:
      voltages[2] = (data[0] >> 16) + (data[1] << 16);
      break;
   case CAN_ID_TEMP:
      temp = (data[0] >> 16) + (data[1] << 16);
      break;
   case CAN_ID_POWER:
      power = (data[0] >> 16) + (data[1] << 16);
      break;
   case CAN_ID_CURINT:
      currentIntegral = (data[0] >> 16) + (data[1] << 16);
      break;
   default:
      isIsa = false;
      break;
   }

   return isIsa;
}

void IsaShunt::Configure(uint32_t data[2])
{
   //Bootup message received, initialize shunt
   if ((data[0] & 0xFF) == 0xBF)
   {
      initialized = false;
      channel = 0;
      Stop();
      return;
   }

   if ((data[0] & 0xFFFF) == 0x01B4)
      started = true;

   if ((data[0] & 0xF0F0) == 0x40A0)
      channel = (data[0] & 0xF) + 1;

   if (!initialized)
   {
      //Data channels
      //00: I [mA]
      //01: U1 [mV]
      //02: U2 [mV]
      //03: U3 [mV]
      //04: T [°C]
      //05: P [W]
      //06: As [As]
      //07: Wh [Wh]

      if (channel < 8)
      {
         //config channel I (0x20), little endian/disabled (0x40), 100ms (0x64, 0x00)
         uint32_t configData[2] = { 0x64004020, 0 };

         configData[0] |= channel;
         //Only enable I and As
         if ((1 << channel) & enableMask)
            configData[0] |= 0x200; //cyclic trigger
         can->Send(CAN_ID_CONFIG, configData);
         //channel++;
      }
      else
      {
         Start();
         initialized = true;
      }
   }
}
