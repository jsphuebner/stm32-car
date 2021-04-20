/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
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

#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/usart.h>
#include "hwdefs.h"
#include "terminal.h"
#include "params.h"
#include "my_string.h"
#include "my_fp.h"
#include "printf.h"
#include "param_save.h"
#include "errormessage.h"
#include "stm32_can.h"
#include "leafbms.h"
#include "terminalcommands.h"

static void PrintVoltages(Terminal* term, char* arg);
static void LoadDefaults(Terminal* term, char *arg);
static void GetAll(Terminal* term, char *arg);
static void PrintSerial(Terminal* term, char *arg);
static void PrintErrors(Terminal* term, char *arg);
static void PrintLin(Terminal* term, char *arg);

char lindata[15];

extern "C" const TERM_CMD termCmds[] =
{
  { "set", TerminalCommands::ParamSet },
  { "get", TerminalCommands::ParamGet },
  { "flag", TerminalCommands::ParamFlag },
  { "stream", TerminalCommands::ParamStream },
  { "json", TerminalCommands::PrintParamsJson },
  { "can", TerminalCommands::MapCan },
  { "save", TerminalCommands::SaveParameters },
  { "load", TerminalCommands::LoadParameters },
  { "reset", TerminalCommands::Reset },
  { "voltages", PrintVoltages },
  { "defaults", LoadDefaults },
  { "all", GetAll },
  { "serial", PrintSerial },
  { "errors", PrintErrors },
  { "lin", PrintLin },
  { NULL, NULL }
};

static void PrintLin(Terminal* term, char *arg)
{
   printf("%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\r\n",
          lindata[0],
          lindata[1],
          lindata[2],
          lindata[3],
          lindata[4],
          lindata[5],
          lindata[6],
          lindata[7],
          lindata[8],
          lindata[9],
          lindata[10],
          lindata[11],
          lindata[12],
          lindata[13],
          lindata[14]
          );
}

static void PrintVoltages(Terminal* term, char* arg)
{
   term = term;
   arg = arg;

   for (int i = 0; i < LeafBMS::NUMCELLS; i++)
   {
      printf("%d: %d, Shunt Flag: %d\r\n", i, LeafBMS::GetCellVoltage(i), LeafBMS::GetCellStatus(i));
   }
}


static void LoadDefaults(Terminal* term, char *arg)
{
   term = term;
   arg = arg;
   Param::LoadDefaults();
   printf("Defaults loaded\r\n");
}

static void GetAll(Terminal* term, char *arg)
{
   const Param::Attributes *pAtr;

   term = term;
   arg = arg;

   for (uint32_t  idx = 0; idx < Param::PARAM_LAST; idx++)
   {
      pAtr = Param::GetAttrib((Param::PARAM_NUM)idx);
      printf("%s\t\t%f\r\n", pAtr->name, Param::Get((Param::PARAM_NUM)idx));
   }
}

static void PrintErrors(Terminal* term, char *arg)
{
   term = term;
   arg = arg;
   ErrorMessage::PrintAllErrors();
}

static void PrintSerial(Terminal* term, char *arg)
{
   term = term;
   arg = arg;
   printf("%X%X%X\r\n", DESIG_UNIQUE_ID2, DESIG_UNIQUE_ID1, DESIG_UNIQUE_ID0);
}
