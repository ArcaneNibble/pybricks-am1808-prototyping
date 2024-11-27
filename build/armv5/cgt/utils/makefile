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
include ${ROOT}/build/armv5/cgt/makedefs

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
	$(AR) $(ARFLAGS) Debug/utils.lib *.o*
	@mv *.o* Debug/
	@mkdir -p ${UTILITY_BIN}/Debug
	@cp Debug/utils.lib ${UTILITY_BIN}/Debug/utils.lib

release: 
	mkdir -p Release
	$(CC) $(CFLAGS) -O2 $(SOURCE)
	$(AR) $(ARFLAGS) Release/utils.lib *.o*
	@mv *.o* Release/
	@mkdir -p ${UTILITY_BIN}/Release
	@cp Release/utils.lib ${UTILITY_BIN}/Release/utils.lib

#
# Rules for cleaning 
#
clean:
	@rm -rf Debug Release ${UTILITY_BIN}/Debug ${UTILITY_BIN}/Release
