#******************************************************************************
#
# makefile - Rules for building the example
#
# Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
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
include $(ROOT)/build/armv5/cgt/makedefs

#
# Default boot mode peripheral
#
BOOT?=SPI

#
# Target Directories that need to be built
#
DIRS=$(DRIVERS_BLD) $(PLATFORM_BLD) $(UTILITY_BLD) $(SYSCONFIG_BLD)
ifeq ($(BOOT), NAND)
DIRS+=$(NANDLIB_BLD)
endif

#
# Application Location
#
APP=$(ROOT)bootloader/
APP_BIN=$(ROOT)/binary/$(TARGET)/$(COMPILER)/$(DEVICE)/$(EVM)/bootloader

CFLAGS+=-D$(BOOT) -DRPRC_IMAGE -I$(APP)include -I$(APP)/include/$(EVM)

#
# Application source files
#
APP_SRC=$(APP)/src/bl_main.c \
        $(APP)/src/bl_copy_rprc.c \
        $(APP)/src/$(EVM)/bl_platform.c

ifeq ($(BOOT), SPI)
APP_SRC+=$(APP)/src/bl_spi.c
endif

ifeq ($(BOOT), NAND)
APP_SRC+=$(APP)/src/bl_nand.c
endif

#
# Required library files
#
APP_LIB=-ldrivers.lib  \
        -lutils.lib    \
        -lplatform.lib \
        -lsystem_config.lib

ifeq ($(BOOT), NAND)
APP_LIB+=-lnand.lib
endif

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
	@for i in $(DIRS);                                \
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
	$(LD) $(LDFLAGS) $(LPATH) -o $(TARGET_MODE)/boot.out -m $(TARGET_MODE)/boot.map \
              $(TARGET_MODE)/*.o* -l"libc.a" "boot.cmd" $(APP_LIB)
	@mkdir -p $(APP_BIN)/$(TARGET_MODE)
	@cp $(TARGET_MODE)/boot.out $(APP_BIN)/$(TARGET_MODE)/boot.out

#
# Rules for cleaning
#
clean:
	@rm -rf Debug Release $(APP_BIN)/Debug $(APP_BIN)/Release

clean+: clean
	@make TARGET_MODE=clean lib
