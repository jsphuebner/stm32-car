# stm32-car
This firmware is a hacky mishmash of VW and Nissan CAN code. It talks to the Nissan BMS (aka LBC) to find out cell voltages and such. It also produces all messages needed for satisfying the various controllers in newer VW cars (my testbed is a 2004 Touran). That means all warning lights are off.

# Features
- Send all messages necessary to make the DSC ligth go off
- Fake the oil pressure sensor depending on motor rpm
- Map the electric motor speed onto the speed dial
- Map the electric motor temperature onto the temp dial
- Map the momentary electric power onto the fuel economy display
- Read key switch, brake switch (and to be done cruise control buttons) from CAN bus
- Read brake vacuum sensor and control vacuum pump
- Read throttle pedal and put it on the CAN bus
- Read individual cell voltages from Nissan Leaf BMS
- Read SoC, SoH etc. from the Nissan Leaf BMS
- Control inverter charge mode depending on battery state

# CAN configuration
Not all CAN messages are hard coded, some are configured via the generic interface. The parameter names are strange, as I just used some left over stuff from the inverter firmware.
Send these commands:
- can tx pot 1 0 16 1 //this is for the inverter, configure however you want
- can tx pot2 1 16 16 1 //this is for the inverter, configure however you want
- can tx canio 1 32 6 1 //this is for the inverter, configure however you want
- can tx speedmod 640 16 16 4 //speed dial with simulated idling engine so that power steering works
- can tx din_forward 640 0 8 1 //apparently needed this constant
- can tx tmpmod 648 8 8 1 //temperature dial
- can tx turns 644 0 8 32 //copy from message #80
- can tx turns 644 8 8 32 //copy from message #80
- can tx din_forward 896 8 8 101 //apparently needed this constant
- can tx din_forward 896 56 8 17 //apparently needed this constant
- can rx din_start 1394 3 1 32 //grabs key switch start position
- can rx din_brake 416 11 1 32 //grabs brake light switch
- can rx din_forward 416 14 1 32 //some bit that is always 1
- can rx opmode 2 0 3 32 //various inverter values
- can rx udcinv 2 16 16 8
- can rx tmpm 2 48 8 32
- can rx speed 2 32 16 32
- can rx turns 80 20 4 1 //grabs sequence from canbus

# Compiling
You will need the arm-none-eabi toolchain: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
The only external depedency is libopencm3 which I forked. You can download and build this dependency by typing

`make get-deps`

Now you can compile stm32-sine by typing

`make`

And upload it to your board using a JTAG/SWD adapter, the updater.py script or the esp8266 web interface
