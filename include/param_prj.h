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
#define VER 1.36.R-meb


/* Entries must be ordered as follows:
   1. Saveable parameters (id != 0)
   2. Temporary parameters (id = 0)
   3. Display values
 */
//Next param id (increase when adding new parameter!): 115
/*              category     name         unit       min     max     default id */
#define PARAM_LIST \
    PARAM_ENTRY(CAT_ESP,      allowedspin, "km/h",    0,      50,     10,     5   ) \
    PARAM_ENTRY(CAT_ESP,      allowedlag,  "km/h",    -50,    0,      -5,     6   ) \
    PARAM_ENTRY(CAT_ESP,      tractionkp,  "",        0,      1000,   5,      7   ) \
    PARAM_ENTRY(CAT_CRUISE,   cruisestep,  "rpm",     1,      1000,   200,    3   ) \
    PARAM_ENTRY(CAT_CRUISE,   cruiserampup,"rpm/100ms",1,     1000,   20,     9   ) \
    PARAM_ENTRY(CAT_CRUISE,   cruiserampdn,"rpm/100ms",1,     1000,   20,     11  ) \
    PARAM_ENTRY(CAT_CRUISE,   regenlevel,  "",        0,      3,      2,      101 ) \
    PARAM_ENTRY(CAT_POWER,    balance,     ONOFF,     0,      1,      0,      110 ) \
    PARAM_ENTRY(CAT_POWER,    chargelimit, "A",       0,      255,    255,    10  ) \
    PARAM_ENTRY(CAT_POWER,    obclimit,    "A",       0,      255,    12,     107 ) \
    PARAM_ENTRY(CAT_POWER,    obcsoclimit, "%",       0,      100,    100,    12  ) \
    PARAM_ENTRY(CAT_POWER,    fcsoclimit,  "%",       0,      100,    100,    111 ) \
    PARAM_ENTRY(CAT_POWER,    udclimit,    "V",       0,      600,    400,    92  ) \
    PARAM_ENTRY(CAT_POWER,    cdmcheckena, ONOFF,     0,      1,      1,      13  ) \
    PARAM_ENTRY(CAT_POWER,    cdmversion,  "",        0,      2,      1,      14  ) \
    PARAM_ENTRY(CAT_CONTACT,  vacuumthresh,"dig",     0,      4095,   2700,   20  ) \
    PARAM_ENTRY(CAT_CONTACT,  vacuumhyst,  "dig",     0,      4095,   2500,   80  ) \
    PARAM_ENTRY(CAT_CONTACT,  oilthresh,   "rpm",     0,      10000,  900,    90  ) \
    PARAM_ENTRY(CAT_CONTACT,  oilhyst,     "rpm",     0,      10000,  500,    91  ) \
    PARAM_ENTRY(CAT_CONTACT,  udcdc,       "V",       10,     15,     14,     102 ) \
    PARAM_ENTRY(CAT_CONTACT,  dcdcresume,  "V",       10,     15,     12.5,   108 ) \
    PARAM_ENTRY(CAT_CONTACT,  dcdcutoff,   "A",       0,     150,     15,     109 ) \
    TESTP_ENTRY(CAT_CONTACT,  cruiselight, ONOFF,     0,      1,      0,      112 ) \
    TESTP_ENTRY(CAT_CONTACT,  errlights,   ERRLIGHTS, 0,      255,    0,      113 ) \
    PARAM_ENTRY(CAT_CONTACT,  heathresh,   "°C",      -20,    255,    10,     98  ) \
    PARAM_ENTRY(CAT_CONTACT,  heatsoc,     "%",       0,      100,    30,     99  ) \
    TESTP_ENTRY(CAT_CONTACT,  heatcmd,     ONOFFFORCE,0,      2,      0,      105 ) \
    PARAM_ENTRY(CAT_GAUGE,    gaugeoffset, "dig",     0,      4096,   1000,   1   ) \
    PARAM_ENTRY(CAT_GAUGE,    gaugegain,   "dig/%",   0,      4096,   5,      2   ) \
    PARAM_ENTRY(CAT_GAUGE,    gaugebalance,"%",       0,      100,   50,      8   ) \
    PARAM_ENTRY(CAT_GAUGE,    intempofs,   "dig",     0,      4095,  1700,    104 ) \
    TESTP_ENTRY(CAT_GAUGE,    soctest,     "%",       0,      100,    0,      114 ) \
    VALUE_ENTRY(version,      VERSTR,  2039 ) \
    VALUE_ENTRY(opmode,       OPMODES, 2086 ) \
    VALUE_ENTRY(invmode,      OPMODES, 2000 ) \
    VALUE_ENTRY(inverr,       "",      2096 ) \
    VALUE_ENTRY(cdmstatus,    CDMSTAT, 2070 ) \
    VALUE_ENTRY(cdmcureq,     "A",     2076 ) \
    VALUE_ENTRY(lasterr,      errorListString,  2038 ) \
    VALUE_ENTRY(lbcdtc,       "",      2098 ) \
    VALUE_ENTRY(chgtime,      "min",   2079 ) \
    VALUE_ENTRY(batfull,      ONOFF,   2069 ) \
    VALUE_ENTRY(batmin,       "mV",    2044 ) \
    VALUE_ENTRY(batmax,       "mV",    2045 ) \
    VALUE_ENTRY(batavg,       "mV",    2046 ) \
    VALUE_ENTRY(udcinv,       "V",     2001 ) \
    VALUE_ENTRY(udcbms,       "V",     2048 ) \
    VALUE_ENTRY(udccdm,       "V",     2068 ) \
    VALUE_ENTRY(udcobc,       "V",     2091 ) \
    VALUE_ENTRY(uacobc,       "V",     2092 ) \
    VALUE_ENTRY(bmschglim,    "A",     2049 ) \
    VALUE_ENTRY(power,        "kW",    2051 ) \
    VALUE_ENTRY(chgcurlim,    "A",     2066 ) \
    VALUE_ENTRY(discurlim,    "A",     2082 ) \
    VALUE_ENTRY(limreason,    LIMRS,   2084 ) \
    VALUE_ENTRY(idc,          "A",     2047 ) \
    VALUE_ENTRY(idccdm,       "A",     2067 ) \
    VALUE_ENTRY(idcdc,        "A",     2081 ) \
    VALUE_ENTRY(iacobc,       "A",     2093 ) \
    VALUE_ENTRY(energy,       "Wh",    2110 ) \
    VALUE_ENTRY(soc,          "%",     2052 ) \
    VALUE_ENTRY(soh,          "%",     2053 ) \
    VALUE_ENTRY(speed,        "rpm",   2012 ) \
    VALUE_ENTRY(speedmod,     "rpm",   2013 ) \
    VALUE_ENTRY(pot,          "dig",   2015 ) \
    VALUE_ENTRY(pot2,         "dig",   2016 ) \
    VALUE_ENTRY(potbrake,     "dig",   2075 ) \
    VALUE_ENTRY(brakepressure,"dig",   2074 ) \
    VALUE_ENTRY(canctr,       "dig",   2097 ) \
    VALUE_ENTRY(vacuum,       "dig",   2018 ) \
    VALUE_ENTRY(tmpbat1,      "°C",    2078 ) \
    VALUE_ENTRY(tmpbat2,      "°C",    2088 ) \
    VALUE_ENTRY(tmpbat3,      "°C",    2089 ) \
    VALUE_ENTRY(tmpbat4,      "°C",    2105 ) \
    VALUE_ENTRY(tmpbat5,      "°C",    2106 ) \
    VALUE_ENTRY(tmpbat6,      "°C",    2107 ) \
    VALUE_ENTRY(tmpbat7,      "°C",    2108 ) \
    VALUE_ENTRY(tmpbat8,      "°C",    2109 ) \
    VALUE_ENTRY(tmphs,        "°C",    2019 ) \
    VALUE_ENTRY(tmpm,         "°C",    2020 ) \
    VALUE_ENTRY(tmpaux,       "°C",    2072 ) \
    VALUE_ENTRY(tmpdcdc,      "°C",    2080 ) \
    VALUE_ENTRY(tmpecu,       "°C",    2083 ) \
    VALUE_ENTRY(tmpmod,       "dig",   2040 ) \
    VALUE_ENTRY(uaux,         "V",     2021 ) \
    VALUE_ENTRY(canio,        CANIOS,  2022 ) \
    VALUE_ENTRY(cruisespeed,  "rpm",   2059 ) \
    VALUE_ENTRY(cruisestt,CRUISESTATES,2055 ) \
    VALUE_ENTRY(wheelfl,      "km/h",  2060 ) \
    VALUE_ENTRY(wheelfr,      "km/h",  2061 ) \
    VALUE_ENTRY(wheelrl,      "km/h",  2062 ) \
    VALUE_ENTRY(wheelrr,      "km/h",  2063 ) \
    VALUE_ENTRY(calcthrotmax, "%",     2064 ) \
    VALUE_ENTRY(calcthrotmin, "%",     2065 ) \
    VALUE_ENTRY(din_cruise,   ONOFF,   2023 ) \
    VALUE_ENTRY(din_start,    ONOFF,   2024 ) \
    VALUE_ENTRY(din_brake,    ONOFF,   2025 ) \
    VALUE_ENTRY(din_forward,  ONOFF,   2027 ) \
    VALUE_ENTRY(din_reverse,  ONOFF,   2028 ) \
    VALUE_ENTRY(din_bms,      ONOFF,   2032 ) \
    VALUE_ENTRY(din_bmslock,  ONOFF,   2054 ) \
    VALUE_ENTRY(din_charge,   ONOFF,   2087 ) \
    VALUE_ENTRY(dout_dcsw,    ONOFF,   2090 ) \
    VALUE_ENTRY(dout_dcdc,    ONOFF,   2094 ) \
    VALUE_ENTRY(dout_evse,    ONOFF,   2095 ) \
    VALUE_ENTRY(handbrk,      ONOFF,   2071 ) \
    VALUE_ENTRY(espoff,       ONOFF,   2077 ) \
    VALUE_ENTRY(cpuload,      "%",     2035 ) \
    VALUE_ENTRY(canrec,       "",      2100 ) \
    VALUE_ENTRY(cantec,       "",      2101 ) \
    VALUE_ENTRY(canlec,       LEC,      2102 ) \
    VALUE_ENTRY(canerr,       LEC,      2103 ) \
    VALUE_ENTRY(uptime,       "",      2104 ) \

//Next value Id: 2111

#define VERSTR STRINGIFY(4=VER)
#define LEC          "0=Ok, 1=Stuff, 2=Form, 3=Ack, 4=Recessive, 5=Dominant, 6=Crc, 7=Sw"
#define CERR         "0=None, 1=ErrorWarning, 2=ErrorPassive, 4=BusOff"
#define OPMODES      "0=Off, 1=Run, 2=ChargeStart, 3=ConnectorLock, 4=Charge, 5=ChargeStop"
#define DIRS         "-1=Reverse, 0=Neutral, 1=Forward"
#define ONOFF        "0=Off, 1=On, 2=na"
#define ONOFFFORCE   "0=Off, 1=On, 2=Force"
#define OKERR        "0=Error, 1=Ok, 2=na"
#define CANSPEEDS    "0=250k, 1=500k, 2=800k, 3=1M"
#define CANIOS       "1=Cruise, 2=Start, 4=Brake, 8=Fwd, 16=Rev, 32=Bms"
#define CANPERIODS   "0=100ms, 1=10ms"
#define HWREVS       "0=Rev1, 1=Rev2, 2=Rev3, 3=Tesla"
#define ERRLIGHTS    "0=Off, 4=EPC, 8=engine"
#define CRUISESTATES "0=None, 1=On, 2=Disable, 4=SetN, 8=SetP"
#define CDMSTAT      "1=Charging, 2=Malfunction, 4=ConnLock, 8=BatIncomp, 16=SystemMalfunction, 32=Stop"
#define LIMRS        "0=Normal, 1=CapDrop, 2=Malfunction, 3=HighTemp, 4=LowTemp"
#define CAT_THROTTLE "Throttle"
#define CAT_POWER    "Power Limit"
#define CAT_CONTACT  "Contactor Control"
#define CAT_TEST     "Testing"
#define CAT_COMM     "Communication"
#define CAT_GAUGE    "Fuel Gauge"
#define CAT_ESP      "ESP/ABS integration"
#define CAT_CRUISE   "Cruise Control"

#define CAN_PERIOD_100MS    0
#define CAN_PERIOD_10MS     1

enum modes
{
   MOD_OFF = 0,
   MOD_RUN,
   MOD_CHARGESTART,
   MOD_CHARGELOCK,
   MOD_CHARGE,
   MOD_CHARGEND,
   MOD_LAST
};

enum cruisestate
{
   CRUISE_ON = 1,
   CRUISE_DISABLE = 2,
   CRUISE_SETN = 4,
   CRUISE_SETP = 8
};

enum onoffforce
{
   CMD_OFF = 0,
   CMD_ON = 1,
   CMD_FORCE = 2
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

