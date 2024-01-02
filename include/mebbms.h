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
      uint16_t GetCellVoltage(int idx) const { return cellVoltages[idx]; }
      float GetModuleTemperature(int idx) const { return temps[idx]; }
      bool GetBalanceFlag(int idx) { return (balFlags[idx / CellsPerCmu] & 1 << (idx % CellsPerCmu)) > 0; }
      float GetMaximumChargeCurrent();
      float GetMaximumDischargeCurrent();
      uint16_t GetMinCellVoltage() { return minCellVoltage; }
      uint16_t GetMaxCellVoltage() { return maxCellVoltage; }
      float GetAvgCellVoltage() { return totalVoltage / NumCells; }
      float GetTotalVoltage() { return totalVoltage / 1000.0f; }
      float EstimateSocFromVoltage();
      bool CellVoltagesSettled(float current, uint32_t time);
      void Balance(bool enable);
      bool Alive(uint32_t time);
      static const int NumCells = 96;

   protected:

   private:
      void Accumulate();
      void SetCellVoltage(int idx, float vtg);
      float LowTempDerating();
      float HighTempDerating();

      CanHardware* canHardware;
      static const int CellsPerCmu = 12;
      uint16_t cellVoltages[NumCells];
      uint16_t balFlags[NumCells / CellsPerCmu];
      uint16_t maxCellVoltage;
      uint16_t minCellVoltage;
      float totalVoltage;
      float temps[NumCells / CellsPerCmu];
      float lowTemp;
      float highTemp;
      uint32_t lastReceived[NumCells / CellsPerCmu];
      bool balancerRunning;
      uint8_t balCounter;
      uint16_t lastUpperCellVoltage;
      uint16_t lastLowerCellVoltage;
      uint32_t lastSettlingSampleTime;
};

#endif // MEBBMS_H