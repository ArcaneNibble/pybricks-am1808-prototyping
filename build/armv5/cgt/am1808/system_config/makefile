#******************************************************************************
#
# makefile - Rules for building the system config library.
#
# Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
#  ALL RIGHTS RESERVED
#
#******************************************************************************

#
# Locating the root directory 
#
ROOT=../../../../../

#
# Device definition (EVM not required)
#
DEVICE=am1808

#
# Include makefile definitions. Includes compilation, linker and archiving
# options.
#
include ${ROOT}/build/armv5/cgt/makedefs

#
# Source files for given target
#
SOURCE=${SYSCONFIG_SRC}/${COMPILER}/cpu.c     \
       ${SYSCONFIG_SRC}/${COMPILER}/cp15.c    \
       ${SYSCONFIG_SRC}/${COMPILER}/init.asm  \
       ${SYSCONFIG_SRC}/${DEVICE}/interrupt.c \
       ${SYSCONFIG_SRC}/${DEVICE}/startup.c   \
       ${SYSCONFIG_SRC}/${DEVICE}/${COMPILER}/exceptionhandler.asm

#
# Rules for building the source files
#

all: debug release

debug: 
	mkdir -p Debug
	$(CC) $(CFLAGS) -g $(SOURCE)
	$(AR) $(ARFLAGS) Debug/system_config.lib *.obj
	@mv *.obj Debug/
	@mkdir -p ${SYSCONFIG_BIN}/Debug
	@cp Debug/system_config.lib ${SYSCONFIG_BIN}/Debug/system_config.lib

release: 
	mkdir -p Release
	$(CC) $(CFLAGS) -O2 $(SOURCE)
	$(AR) $(ARFLAGS) Release/system_config.lib *.obj
	@mv *.obj Release/
	@mkdir -p ${SYSCONFIG_BIN}/Release
	@cp Release/system_config.lib ${SYSCONFIG_BIN}/Release/system_config.lib

#
# Rules for cleaning
#
clean:
	@rm -rf Debug Release ${SYSCONFIG_BIN}/Debug ${SYSCONFIG_BIN}/Release
