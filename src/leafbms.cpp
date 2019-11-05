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

#include "leafbms.h"
#include "my_fp.h"
#include "my_math.h"
#include "stm32_can.h"
#include "params.h"

#define CRCKEY 0x185

int LeafBMS::bmsGrp = 2;
int LeafBMS::bmsGrpIndex = -1;
uint8_t LeafBMS::run10ms = 0;
uint8_t LeafBMS::run100ms = 0;
uint8_t LeafBMS::voltBytes[NUMCELLS * 2];

void LeafBMS::DecodeCAN(int id, uint32_t data[2])
{
   uint8_t* bytes = (uint8_t*)data;

   if (bmsGrp == 2 && id == 0x7BB)
   {
      if (bmsGrpIndex == 0)
      {
         for (int i = 4; i < 8; i++)
         {
            voltBytes[i - 4] = bytes[i];
         }
      }
      else if (bmsGrpIndex < 28)
      {
         for (int i = 1; i < 8; i++)
         {
            voltBytes[7 * bmsGrpIndex - 4 + i] = bytes[i];
         }
      }
   }
   else if (id == 0x1DB)
   {
      s32fp cur = (int16_t)(bytes[0] << 8) + (bytes[1] & 0xE0);
      s32fp udc = ((bytes[2] << 8) + (bytes[3] & 0xC0)) >> 1;

      Param::SetFlt(Param::idc, cur / 2);
      Param::SetFlt(Param::udcbms, udc / 2);
   }
   else if (id == 0x1DC)
   {
      s32fp dislimit = ((bytes[0] << 8) + (bytes[1] & 0xC0)) >> 1;
      s32fp chglimit = ((bytes[1] & 0x3F) << 9) + ((bytes[2] & 0xF0) << 1);

      Param::SetFlt(Param::dislimit, dislimit / 4);
      Param::SetFlt(Param::chglimit, chglimit / 4);
   }
}

void LeafBMS::RequestNextFrame()
{
   if (bmsGrp == 2)
   {
      uint32_t canData[2] = { 0, 0xffffffff };
      if (bmsGrpIndex == -1)
      {
         bmsGrpIndex++;
         canData[0] = 0x2 | 0x21 << 8 | 0x2 << 16 | 0xff << 24;
         Can::Send(0x79B, canData);
      }
      else if (bmsGrpIndex < 28)
      {
         bmsGrpIndex++;
         canData[0] = 0x30 | 0x1 << 8 | 0x0 << 16 | 0xff << 24;
         Can::Send(0x79B, canData);
      }
      else
      {
         bmsGrpIndex = -1;
         int min = 4500, max = 0, avg = 0;

         for (int i = 0; i < NUMCELLS; i++)
         {
            uint16_t voltage = GetCellVoltage(i);
            avg += voltage;
            min = MIN(min, voltage);
            max = MAX(max, voltage);
         }

         Param::SetInt(Param::batmin, min);
         Param::SetInt(Param::batmax, max);
         Param::SetInt(Param::batavg, avg / 96);
      }
   }
}

uint16_t LeafBMS::GetCellVoltage(int idx)
{
   if (idx < NUMCELLS)
   {
      return voltBytes[2 * idx] << 8 | voltBytes[2 * idx + 1];
   }
   return -1;
}

void LeafBMS::Send10msMessages()
{
   uint32_t canData[2] = { 0, 0 };
   uint8_t crc;

   canData[1] = run10ms << 6 | 1 << 2 | 1 << 14;
   crc = Crc8ForHCM(7, (uint8_t*)canData);
   canData[1] |= crc << 24;
   Can::Send(0x1D4, canData);

   canData[1] = run10ms << 16;
   Can::Send(0x1F2, canData);

   run10ms = (run10ms + 1) & 3;
}

void LeafBMS::Send100msMessages()
{
   uint32_t canData[2] = { 0, 0 };
   uint8_t crc;
   //TODO: charger power?
   Can::Send(0x390, canData);

   canData[0] = run100ms << 24;
   canData[1] = 0xB2;
   crc = Crc8ForHCM(5, (uint8_t*)canData);
   canData[1] |= crc << 8;
   Can::Send(0x50C, canData);

   run100ms = (run100ms + 1) & 3;
}

uint8_t LeafBMS::Crc8ForHCM(int n, uint8_t *msg)
{
   uint16_t tmp = 0;
   uint8_t crc = 0;
   uint8_t i,j;

   for(i=0; i<n; i++)
   {
      tmp ^= *(msg + i);
      for(j=0; j<8; j++ )
      {
         tmp <<=1;
         if( tmp & 0x0100 )
         {
            tmp ^= CRCKEY;
         }
      }
   }
   crc = (uint8_t)(tmp & 0xff);
   return crc;
}
