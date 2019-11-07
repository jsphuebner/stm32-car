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
#define OPMODES      "0=Off, 1=Run, 2=ManualRun, 3=Boost, 4=Buck, 5=Sine, 6=AcHeat"
#define PWMFRQS      "0=17.6kHz, 1=8.8kHz, 2=4.4KHz, 3=2.2kHz, 4=1.1kHz"
#define PWMPOLS      "0=ACTHIGH, 1=ACTLOW"
#define DIRS         "-1=Reverse, 0=Neutral, 1=Forward"
#define TRIPMODES    "0=AllOff, 1=DcSwOn, 2=PrechargeOn"
#define SNS_HS       "0=JCurve, 1=Semikron, 2=MBB600"
#define SNS_M        "12=KTY83-110, 13=KTY84-130, 14=Leaf"
#define PWMFUNCS     "0=tmpm, 1=tmphs, 2=speed, 3=speedfrq"
#define BTNSWITCH    "0=Button, 1=Switch"
#define DIRMODES     "0=Button, 1=Switch, 2=ButtonReversed, 3=SwitchReversed"
#define IDLEMODS     "0=always, 1=nobrake, 2=cruise"
#define ONOFF        "0=Off, 1=On, 2=na"
#define OKERR        "0=Error, 1=Ok, 2=na"
#define CHARGEMODS   "0=Off, 3=Boost, 4=Buck"
#define ENCMODES     "0=Single, 1=AB, 2=ABZ, 3=SPI, 4=Resolver, 5=SinCos"
#define POTMODES     "0=SingleRegen, 1=DualChannel, 2=CAN"
#define CANSPEEDS    "0=250k, 1=500k, 2=800k, 3=1M"
#define CANIOS       "1=Cruise, 2=Start, 4=Brake, 8=Fwd, 16=Rev, 32=Bms"
#define CANPERIODS   "0=100ms, 1=10ms"
#define HWREVS       "0=Rev1, 1=Rev2, 2=Rev3, 3=Tesla"
#define CRUISELIGHT  "0=Off, 4=On"
#define ERRLIGHTS    "0=Off, 4=EPC, 8=engine"
#define CAT_MOTOR    "Motor"
#define CAT_INVERTER "Inverter"
#define CAT_THROTTLE "Throttle"
#define CAT_REGEN    "Regen"
#define CAT_AUTOM    "Automation"
#define CAT_DERATE   "Derating"
#define CAT_PWM      "Aux PWM"
#define CAT_CONTACT  "Contactor Control"
#define CAT_TEST     "Testing"
#define CAT_CHARGER  "Charger"
#define CAT_COMM     "Communication"

#define BUTTON 0

#define PWM_FUNC_TMPM       0
#define PWM_FUNC_TMPHS      1
#define PWM_FUNC_SPEED      2
#define PWM_FUNC_SPEEDFRQ   3

#define CAN_PERIOD_100MS    0
#define CAN_PERIOD_10MS     1

#define IDLE_MODE_ALWAYS    0
#define IDLE_MODE_NOBRAKE   1
#define IDLE_MODE_CRUISE    2

#define POTMODE_REGENADJ    0
#define POTMODE_DUALCHANNEL 1
#define POTMODE_CAN         2

#define VER 0.13.B
#define VERSTR STRINGIFY(4=VER)

enum _modes
{
   MOD_OFF = 0,
   MOD_RUN,
   MOD_MANUAL,
   MOD_BOOST,
   MOD_BUCK,
   MOD_SINE,
   MOD_ACHEAT,
   MOD_LAST
};

enum _tripmodes
{
   TRIP_ALLOFF = 0,
   TRIP_DCSWON,
   TRIP_PRECHARGEON
};

enum _dirmodes
{
   DIR_BUTTON = 0,
   DIR_SWITCH = 1,
   DIR_REVERSED = 2, //used as a flag
};

enum _canio
{
   CAN_IO_CRUISE = 1,
   CAN_IO_START = 2,
   CAN_IO_BRAKE = 4,
   CAN_IO_FWD = 8,
   CAN_IO_REV = 16,
   CAN_IO_BMS = 32
};

extern const char* errorListString;


/* Entries must be ordered as follows:
   1. Saveable parameters (id != 0)
   2. Temporary parameters (id = 0)
   3. Display values
 */
//Next param id (increase when adding new parameter!): 98
/*              category     name         unit       min     max     default id */
#define PARAM_LIST \
    PARAM_ENTRY(CAT_DERATE,  bmslimhigh,  "%",       0,      100,    50,     55  ) \
    PARAM_ENTRY(CAT_DERATE,  bmslimlow,   "%",       -100,   0,      -1,     56  ) \
    PARAM_ENTRY(CAT_THROTTLE,potmin,      "dig",     0,      4095,   0,      17  ) \
    PARAM_ENTRY(CAT_THROTTLE,potmax,      "dig",     0,      4095,   4095,   18  ) \
    PARAM_ENTRY(CAT_THROTTLE,pot2min,     "dig",     0,      4095,   4095,   63  ) \
    PARAM_ENTRY(CAT_THROTTLE,pot2max,     "dig",     0,      4095,   4095,   64  ) \
    PARAM_ENTRY(CAT_THROTTLE,potmode,     POTMODES,  0,      2,      0,      82  ) \
    PARAM_ENTRY(CAT_THROTTLE,throtramp,   "%/10ms",  1,      100,    100,    81  ) \
    PARAM_ENTRY(CAT_THROTTLE,throtramprpm,"rpm",     0,      20000,  20000,  85  ) \
    PARAM_ENTRY(CAT_THROTTLE,ampmin,      "%",       0,      100,    10,     4   ) \
    PARAM_ENTRY(CAT_THROTTLE,slipstart,   "%",       10,     100,    50,     90  ) \
    PARAM_ENTRY(CAT_REGEN,   brknompedal, "%",       -100,   0,      -50,    38  ) \
    PARAM_ENTRY(CAT_REGEN,   brkpedalramp,"%/10ms",  1,      100,    100,    68  ) \
    PARAM_ENTRY(CAT_REGEN,   brknom,      "%",       0,      100,    30,     19  ) \
    PARAM_ENTRY(CAT_REGEN,   brkmax,      "%",       -100,   0,      -30,    49  ) \
    PARAM_ENTRY(CAT_REGEN,   brkrampstr,  "Hz",      0,      400,    10,     39  ) \
    PARAM_ENTRY(CAT_REGEN,   brkout,      "%",       -100,   -1,     -50,    67  ) \
    PARAM_ENTRY(CAT_CONTACT, vacuumthresh,"dig",     0,      4095,   2700,   20  ) \
    PARAM_ENTRY(CAT_CONTACT, vacuumhyst,  "dig",     0,      4095,   2500,   80  ) \
    PARAM_ENTRY(CAT_CONTACT, oilthresh,   "dig",     0,      10000,  900,    90  ) \
    PARAM_ENTRY(CAT_CONTACT, oilhyst,     "dig",     0,      10000,  500,    91  ) \
    PARAM_ENTRY(CAT_CONTACT, udcthresh,   "V",       0,      500,    380,    92  ) \
    PARAM_ENTRY(CAT_CONTACT, udchyst,     "V",       0,      500,    360,    93  ) \
    PARAM_ENTRY(CAT_CONTACT, ucellthresh, "mV",      3000,   4200,   4000,   96  ) \
    PARAM_ENTRY(CAT_CONTACT, ucellhyst,   "mV",      3000,   4200,   3900,   97  ) \
    PARAM_ENTRY(CAT_CONTACT, tripmode,    TRIPMODES, 0,      2,      0,      86  ) \
    PARAM_ENTRY(CAT_CONTACT, cruiselight, CRUISELIGHT,0,     255,    0,      94  ) \
    PARAM_ENTRY(CAT_CONTACT, errlights,   ERRLIGHTS, 0,      255,    0,      95  ) \
    PARAM_ENTRY(CAT_PWM,     pwmfunc,     PWMFUNCS,  0,      3,      0,      58  ) \
    PARAM_ENTRY(CAT_PWM,     pwmgain,     "",        -100000,100000, 100,    40  ) \
    PARAM_ENTRY(CAT_PWM,     pwmofs,      "dig",     -65535, 65535,  0,      41  ) \
    PARAM_ENTRY(CAT_COMM,    canspeed,    CANSPEEDS, 0,      3,      0,      83  ) \
    PARAM_ENTRY(CAT_COMM,    canperiod,   CANPERIODS,0,      1,      0,      88  ) \
    VALUE_ENTRY(version,     VERSTR,  2039 ) \
    VALUE_ENTRY(hwver,       HWREVS,  2036 ) \
    VALUE_ENTRY(opmode,      OPMODES, 2000 ) \
    VALUE_ENTRY(lasterr,     errorListString,  2038 ) \
    VALUE_ENTRY(batmin,      "mV",    2044 ) \
    VALUE_ENTRY(batmax,      "mV",    2045 ) \
    VALUE_ENTRY(batavg,      "mV",    2046 ) \
    VALUE_ENTRY(udcinv,      "V",     2001 ) \
    VALUE_ENTRY(udcbms,      "V",     2048 ) \
    VALUE_ENTRY(chglimit,    "kW",    2049 ) \
    VALUE_ENTRY(dislimit,    "kW",    2050 ) \
    VALUE_ENTRY(idc,         "A",     2047 ) \
    VALUE_ENTRY(power,       "kW",    2051 ) \
    VALUE_ENTRY(soc,         "%",     2052 ) \
    VALUE_ENTRY(soh,         "%",     2053 ) \
    VALUE_ENTRY(speed,       "rpm",   2012 ) \
    VALUE_ENTRY(speedmod,    "rpm",   2013 ) \
    VALUE_ENTRY(turns,       "",      2037 ) \
    VALUE_ENTRY(pot,         "dig",   2015 ) \
    VALUE_ENTRY(pot2,        "dig",   2016 ) \
    VALUE_ENTRY(potnom,      "%",     2017 ) \
    VALUE_ENTRY(vacuum,      "dig",   2018 ) \
    VALUE_ENTRY(tmphs,       "°C",    2019 ) \
    VALUE_ENTRY(tmpm,        "°C",    2020 ) \
    VALUE_ENTRY(tmpmod,      "dig",   2040 ) \
    VALUE_ENTRY(uaux,        "V",     2021 ) \
    VALUE_ENTRY(canio,       CANIOS,  2022 ) \
    VALUE_ENTRY(din_cruise,  ONOFF,   2023 ) \
    VALUE_ENTRY(din_start,   ONOFF,   2024 ) \
    VALUE_ENTRY(din_brake,   ONOFF,   2025 ) \
    VALUE_ENTRY(din_forward, ONOFF,   2027 ) \
    VALUE_ENTRY(din_reverse, ONOFF,   2028 ) \
    VALUE_ENTRY(din_bms,     ONOFF,   2032 ) \
    VALUE_ENTRY(cpuload,     "%",     2035 ) \

//Next value Id: 2054
