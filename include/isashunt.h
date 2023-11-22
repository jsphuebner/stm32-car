/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2018 Johannes Huebner <dev@johanneshuebner.com>
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
#ifndef ISASHUNT_H
#define ISASHUNT_H

#include "canhardware.h"

class IsaShunt : public CanCallback
{
   public:
      /** Default constructor */
      IsaShunt(CanHardware* hw);
      bool HandleRx(uint32_t canId, uint32_t data[2], uint8_t dlc);
      void HandleClear();
      void Initialize();
      void ResetCounters();
      void RequestCharge();
      void Stop();
      void Start();
      bool IsReady() { return state == Operate; }
      int32_t GetCurrent() { return current; }
      int32_t GetVoltage() { return voltage; }
      int32_t GetPower() { return power; }
      int32_t GetChargeIn() { return chargeIn; }
      int32_t GetChargeOut() { return chargeOut; }

      static const int CAN_ID_REPLY = 0x511;
      static const int CAN_ID_CURRENT = 0x521;
      static const int CAN_ID_VOLTAGE = 0x522;
      static const int CAN_ID_POWER = 0x526;

   protected:

   private:
      enum IsaStates
      {
         Init, Reset, RequestChargeOut, ReadChargeOut, Operate
      };
      void RunStateMachine(uint32_t data[]);
      CanHardware *can;
      int32_t current;
      int32_t voltage;
      int32_t power;
      uint32_t chargeIn;
      uint32_t chargeOut;
      IsaStates state;
};

#endif // ISASHUNT_H
