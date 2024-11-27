#******************************************************************************
#
# makefile - Rules for building the LIDD example
#
# Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
# ALL RIGHTS RESERVED
#
#
#******************************************************************************

#
# Locating the root directory 
#
ROOT=../../../../../../

#
# Device and EVM definitions
#
DEVICE=am1808
EVM=evmAM1808

#
# Include the makefile definitions. This contains compiler, linker and
# archiver definitions and options
#
include ${ROOT}/build/armv5/cgt/makedefs

#
# Target Directories that need to be built
#
DIRS=${DRIVERS_BLD} ${PLATFORM_BLD} ${SYSCONFIG_BLD} ${UTILITY_BLD}

#
# Application Location
#
APP=${ROOT}examples/${EVM}/lidd/
APP_BIN=${ROOT}/binary/${TARGET}/${COMPILER}/${DEVICE}/${EVM}/lidd

#
# Application source files
#
APP_SRC=$(APP)/*.c

#
# Required library files
#
APP_LIB=-ldrivers.lib  \
        -lutils.lib    \
        -lplatform.lib \
        -lsystem_config.lib

#
# Rules for building the application and library 
#
all: debug release

debug: 
	make TARGET_MODE=debug lib
	make TARGET_MODE=Debug bin

release:
	make TARGET_MODE=release lib
	make TARGET_MODE=Release bin

lib:
	@for i in ${DIRS};                                \
	do                                                \
		if [ -f $${i}/makefile ] ;                    \
		then                                          \
			make $(TARGET_MODE) -C $${i} || exit $$?; \
		fi;                                           \
	done;


bin:
	$(CC)  $(CFLAGS) $(APP_SRC)
	@mkdir -p $(TARGET_MODE)/
	@mv *.o* $(TARGET_MODE)/
	$(LD) ${LDFLAGS} ${LPATH} -o $(TARGET_MODE)/liddDisplay.out -m $(TARGET_MODE)/liddDisplay.map \
              $(TARGET_MODE)/*.o* -l"libc.a" "liddDisplay.cmd" $(APP_LIB)
	@mkdir -p $(APP_BIN)/$(TARGET_MODE)
	@cp $(TARGET_MODE)/liddDisplay.out $(APP_BIN)/$(TARGET_MODE)/liddDisplay.out

#
# Rules for cleaning
#
clean:
	@rm -rf Debug Release $(APP_BIN)/Debug $(APP_BIN)/Release

clean+: clean
	@make TARGET_MODE=clean lib
