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
      void ResetCounters();
      void Stop();
      void Start();
      bool IsReady() { return initialized; }
      int32_t GetCurrent() { return current; }
      int32_t GetVoltage(int i) { return voltages[i]; }
      int32_t GetIntegratedCurrent() { return currentIntegral; }

   private:
      void Configure(uint32_t data[2]);

      CanHardware *can;
      int32_t current;
      int32_t voltages[3];
      int32_t currentIntegral;
      bool initialized;
      uint8_t enableMask;
};

#endif // ISASHUNT_H
