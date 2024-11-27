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
# Device definition (EVM not necessary)
#
DEVICE=am1808

#
# Include makefile definitions. Includes compilation, linker and archiving
# options.
#
include ${ROOT}/build/armv5/gcc/makedefs

#
# Source files for given target
#
SOURCE=${SYSCONFIG_SRC}/${COMPILER}/cpu.c     \
       ${SYSCONFIG_SRC}/${COMPILER}/cp15.c    \
       ${SYSCONFIG_SRC}/${COMPILER}/init.S    \
       ${SYSCONFIG_SRC}/${DEVICE}/interrupt.c \
       ${SYSCONFIG_SRC}/${DEVICE}/startup.c   \
       ${SYSCONFIG_SRC}/${DEVICE}/${COMPILER}/exceptionhandler.S

#
# Rules for building the source files
#

all: debug release

debug: 
	mkdir -p Debug
	$(CC) $(CFLAGS) -g $(SOURCE)
	$(AR) $(ARFLAGS) Debug/libsystem_config.a *.o*
	@mv *.o* Debug/
	@mkdir -p ${SYSCONFIG_BIN}/Debug
	@cp Debug/libsystem_config.a ${SYSCONFIG_BIN}/Debug/libsystem_config.a

release: 
	mkdir -p Release
	$(CC) $(CFLAGS) -g -O2 $(SOURCE)
	$(AR) $(ARFLAGS) Release/libsystem_config.a *.o*
	@mv *.o* Release/
	@mkdir -p ${SYSCONFIG_BIN}/Release
	@cp Release/libsystem_config.a ${SYSCONFIG_BIN}/Release/libsystem_config.a

#
# Rules for cleaning
#
clean:
	@rm -rf Debug Release ${SYSCONFIG_BIN}/Debug ${SYSCONFIG_BIN}/Release
