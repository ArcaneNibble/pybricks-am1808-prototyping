#******************************************************************************
#
# makefile - Rules for building the I2C example
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
APP=${ROOT}examples/${EVM}/i2c/
APP_BIN=${ROOT}/binary/${TARGET}/${COMPILER}/${DEVICE}/${EVM}/i2c

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
	$(LD) ${LDFLAGS} ${LPATH} -o $(TARGET_MODE)/i2cLedBlink.out -m $(TARGET_MODE)/i2cLedBlink.map \
              $(TARGET_MODE)/*.o* -l"libc.a" "i2cLedBlink.cmd" $(APP_LIB)
	@mkdir -p $(APP_BIN)/$(TARGET_MODE)
	@cp $(TARGET_MODE)/i2cLedBlink.out $(APP_BIN)/$(TARGET_MODE)/i2cLedBlink.out

#
# Rules for cleaning
#
clean:
	@rm -rf Debug Release $(APP_BIN)/Debug $(APP_BIN)/Release

clean+: clean
	@make TARGET_MODE=clean lib
