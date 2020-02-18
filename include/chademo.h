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

#ifndef CHADEMO_H
#define CHADEMO_H
#include <stdint.h>
#include "my_math.h"

class ChaDeMo
{
   public:
      static void Process108Message(uint32_t data[2]);
      static void Process109Message(uint32_t data[2]);
      static void SendMessages();

      static void SetTargetBatteryVoltage(uint16_t vtg) { targetBatteryVoltage = vtg; }
      static void SetChargeCurrent(uint8_t cur) { chargeCurrentRequest = MIN(cur, chargerMaxCurrent); }
      static void SetEnabled(bool enabled);
      static void SetSoC(uint8_t soC) { soc = soC; }
      static int GetChargerOutputVoltage() { return chargerOutputVoltage; }
      static int GetChargerOutputCurrent() { return chargerOutputCurrent; }
      static int GetChargerMaxCurrent() { return chargerMaxCurrent; }
      static bool ConnectorLocked() { return connectorLock; }
      static bool ChargerStopRequest() { return chargerStopRequest; }

   protected:

   private:
      static bool chargeEnabled;
      static bool connectorLock;
      static bool chargerStopRequest;
      static uint8_t chargerMaxCurrent;
      static uint8_t chargeCurrentRequest;
      static uint32_t rampedCurReq;
      static uint16_t targetBatteryVoltage;
      static uint16_t chargerOutputVoltage;
      static uint8_t chargerOutputCurrent;
      static uint8_t soc;
};

#endif // CHADEMO_H
