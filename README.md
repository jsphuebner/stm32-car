# stm32-car
This firmware is a hacky mishmash of VW and Nissan CAN code. It talks to the Nissan BMS (aka LBC) to find out cell voltages and such. It also produces all messages needed for satisfying the various controllers in newer VW cars (my testbed is a 2004 Touran). That means all warning lights are off.

# Compiling
You will need the arm-none-eabi toolchain: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
The only external depedency is libopencm3 which I forked. You can download and build this dependency by typing

`make get-deps`

Now you can compile stm32-sine by typing

`make`

And upload it to your board using a JTAG/SWD adapter, the updater.py script or the esp8266 web interface
