#******************************************************************************
#
# makefile - Rules for building the demo application
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
DIRS=${DRIVERS_BLD} ${PLATFORM_BLD} ${SYSCONFIG_BLD} ${UTILITY_BLD} ${GRLIB_BLD} ${USBLIB_BLD}

#
# Application Location
#
APP=${ROOT}examples/${EVM}/demo/
APP_BIN=${ROOT}/binary/${TARGET}/${COMPILER}/${DEVICE}/${EVM}/demo

#
# lwIP Stack location
#
LWIP=${ROOT}third_party/lwip-1.3.2
LWIPAPP=${LWIP}/apps/httpserver_raw_io
LWIPPORT=${LWIP}/ports/am1808

#
# Options for lwIP build
#
LWIP_IPATHS=-I$(LWIP) \
            -I$(LWIP)/src/include \
            -I$(LWIP)/src/include/ipv4 \
            -I$(LWIP)/src/include/lwip \
            -I$(LWIPPORT)/include \
            -I$(LWIPAPP) \
            -I$(APP)
LWIPCFLAGS=$(CFLAGS) ${LWIP_IPATHS}

#
# Include lwIP headers in application build
#
CFLAGS+=${LWIP_IPATHS}

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
        -lsystem_config.lib \
        -lgrlib.lib \
        -lusblib.lib

#
# Rules for building the application and library 
#
all: debug release

debug: 
	make TARGET_MODE=debug lib
	make TARGET_MODE=Debug lwip
	make TARGET_MODE=Debug bin

release:
	make TARGET_MODE=release lib
	make TARGET_MODE=Release lwip
	make TARGET_MODE=Release bin

lib:
	@for i in ${DIRS};                                \
	do                                                \
		if [ -f $${i}/makefile ] ;                    \
		then                                          \
			make $(TARGET_MODE) -C $${i} || exit $$?; \
		fi;                                           \
	done;

lwip:
	$(CC) $(LWIPCFLAGS) $(LWIPAPP)/httpd_io.c
	$(CC) $(LWIPCFLAGS) $(LWIPPORT)/lwiplib.c

bin:
	$(CC)  $(CFLAGS) $(APP_SRC)
	@mkdir -p $(TARGET_MODE)/
	@mv *.o* $(TARGET_MODE)/
	$(LD) ${LDFLAGS} ${LPATH} -o $(TARGET_MODE)/demo.out -m $(TARGET_MODE)/demo.map \
              $(TARGET_MODE)/*.o* -l"libc.a" "demo.cmd" $(APP_LIB)
	@mkdir -p $(APP_BIN)/$(TARGET_MODE)
	@cp $(TARGET_MODE)/demo.out $(APP_BIN)/$(TARGET_MODE)/demo.out

#
# Rules for cleaning
#
clean:
	@rm -rf Debug Release $(APP_BIN)/Debug $(APP_BIN)/Release

clean+: clean
	@make TARGET_MODE=clean lib
