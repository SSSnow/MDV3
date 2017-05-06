#!/bin/sh

#
# flashing method #1
#
/home/dev/bin/st-flash write ../firmware/OPENMV2/openmv.bin 0x08000000

#
# flashing method #2
#

# a). Pull up BOOT0;
# b). Power on the board, link the USB to Ubuntu;
# c). run make flash_bin



