#******************************************************************************
#
# makefile - Rules for building the NAND library.
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
SOURCE=$(ROOT)/nandlib/nandlib.c $(ROOT)/nandlib/nand_emifa2.c

#
# Rules for building the source files
#
all: debug release

debug: 
	mkdir -p Debug
	$(CC) $(CFLAGS) -g $(SOURCE)
	$(AR) $(ARFLAGS) Debug/libnand.a *.o*
	@mv *.o* Debug/
	@mkdir -p ${NANDLIB_BIN}/Debug
	@cp Debug/libnand.a ${NANDLIB_BIN}/Debug/libnand.a

release: 
	mkdir -p Release
	$(CC) $(CFLAGS) -g -O2 $(SOURCE)
	$(AR) $(ARFLAGS) Release/libnand.a *.o*
	@mv *.o* Release/
	@mkdir -p ${NANDLIB_BIN}/Release
	@cp Release/libnand.a ${NANDLIB_BIN}/Release/libnand.a

#
# Rules for cleaning 
#
clean:
	@rm -rf Debug Release ${NANDLIB_BIN}/Debug ${NANDLIB_BIN}/Release
