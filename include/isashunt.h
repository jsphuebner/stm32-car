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
      enum channels { CURRENT = 1, U1 = 2, U2 = 4, U3 = 8, TEMP = 16, POWER = 32, AS = 64, WH = 128 };

      /** Default constructor */
      IsaShunt(CanHardware* hw, uint8_t enabledChannels = 0x4F);
      bool HandleRx(uint32_t canId, uint32_t data[2], uint8_t dlc);
      void HandleClear();
      void ResetCounters();
      void Stop();
      void Start();
      void InitializeAndStartIfNeeded();
      bool IsReady() { return started; }
      int32_t GetValue(channels chan);

   private:
      void Configure(uint32_t data[2]);

      CanHardware *can;
      int32_t current;
      int32_t voltages[3];
      int32_t power;
      int32_t temp;
      int32_t currentIntegral;
      int32_t powerIntegral;
      bool initialized;
      bool started;
      uint8_t enableMask;
};

#endif // ISASHUNT_H
