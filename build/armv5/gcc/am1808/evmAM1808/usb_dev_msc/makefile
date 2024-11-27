#******************************************************************************
#
# makefile - Rules for building the USB MSC device example
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
include ${ROOT}/build/armv5/gcc/makedefs

#
# Use DMA_MODE definition in application build
#
CFLAGS+=-DDMA_MODE

#
# Target Directories that need to be built
#
DIRS=${DRIVERS_BLD} ${PLATFORM_BLD} ${SYSCONFIG_BLD} ${UTILITY_BLD} ${GRLIB_BLD} ${USBLIB_BLD}

#
# Application Location
#
APP=${ROOT}examples/${EVM}/usb_dev_msc/
APP_BIN=${ROOT}/binary/${TARGET}/${COMPILER}/${DEVICE}/${EVM}/usb_dev_msc

#
# Application source files
#
APP_SRC=$(APP)/*.c

#
# Required library files
#
APP_LIB=-ldrivers  \
        -lutils    \
        -lplatform \
        -lsystem_config \
        -lusblib \
        -lgrlib

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
	$(LD) ${LDFLAGS} ${LPATH} -o $(TARGET_MODE)/usb_dev_msc.out -Map $(TARGET_MODE)/usb_dev_msc.map \
              $(TARGET_MODE)/*.o* -T usb_dev_msc.lds $(APP_LIB) -lc -lgcc $(APP_LIB) -lc -lgcc
	@mkdir -p $(APP_BIN)/$(TARGET_MODE)
	@cp $(TARGET_MODE)/usb_dev_msc.out $(APP_BIN)/$(TARGET_MODE)/usb_dev_msc.out

#
# Rules for cleaning
#
clean:
	@rm -rf Debug Release $(APP_BIN)/Debug $(APP_BIN)/Release

clean+: clean
	@make TARGET_MODE=clean lib	