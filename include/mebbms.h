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
#ifndef MEBBMS_H
#define MEBBMS_H

#include "canhardware.h"


class MebBms : public CanCallback
{
   public:
      /** Default constructor */
      MebBms(CanHardware* c);
      bool HandleRx(uint32_t canId, uint32_t data[2], uint8_t dlc);
      void HandleClear();
      uint16_t GetCellVoltage(int idx) { return cellVoltages[idx]; }
      bool Alive(uint32_t time) { return true; }
      static const int NumCells = 96;

   protected:

   private:
      void Accumulate();

      CanHardware* canHardware;
      uint16_t cellVoltages[NumCells];
      uint8_t temps[NumCells / 12];

};

#endif // MEBBMS_H
