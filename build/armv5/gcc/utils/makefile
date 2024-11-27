#******************************************************************************
#
# makefile - Rules for building the driver library.
#
# Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
#  ALL RIGHTS RESERVED
#
#******************************************************************************

#
# Locating the root directory 
#
ROOT=../../../../

#
# Include makefile definitions. 
#
include ${ROOT}/build/armv5/gcc/makedefs

#
# Source files for given target
#

SOURCE=${UTILITY_SRC}/cmdline.c   \
       ${UTILITY_SRC}/delay.c     \
       ${UTILITY_SRC}/systick.c   \
       ${UTILITY_SRC}/uartStdio.c

#
# Rules for building the source files
#
all: debug release

debug: 
	mkdir -p Debug
	$(CC) $(CFLAGS) -g $(SOURCE)
	$(AR) $(ARFLAGS) Debug/libutils.a *.o*
	@mv *.o* Debug/
	@mkdir -p ${UTILITY_BIN}/Debug
	@cp Debug/libutils.a ${UTILITY_BIN}/Debug/libutils.a

release: 
	mkdir -p Release
	$(CC) $(CFLAGS) -g -O2 $(SOURCE)
	$(AR) $(ARFLAGS) Release/libutils.a *.o*
	@mv *.o* Release/
	@mkdir -p ${UTILITY_BIN}/Release
	@cp Release/libutils.a ${UTILITY_BIN}/Release/libutils.a

#
# Rules for cleaning 
#
clean:
	@rm -rf Debug Release ${UTILITY_BIN}/Debug ${UTILITY_BIN}/Release
