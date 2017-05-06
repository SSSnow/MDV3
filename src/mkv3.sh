#!/bin/sh

# make clean && make -j 6
make clean && make TARGET=OPENMV3 -j 6

