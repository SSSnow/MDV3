#!/bin/sh

# make clean && make -j 6
make clean && make TARGET=OPENMV2 -j 6

