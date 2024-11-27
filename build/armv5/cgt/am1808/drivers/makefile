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
ROOT=../../../../../

#
# Device definition (EVM not required)
#
DEVICE=am1808

#
# Include makefile definitions. 
#
include ${ROOT}/build/armv5/cgt/makedefs

#
# Source files for given target
#

SOURCE=${DRIVERS_SRC}/uart.c       \
       ${DRIVERS_SRC}/usbphyGS60.c \
       ${DRIVERS_SRC}/usb.c        \
       ${DRIVERS_SRC}/timer.c      \
       ${DRIVERS_SRC}/i2c.c        \
       ${DRIVERS_SRC}/gpio.c       \
       ${DRIVERS_SRC}/psc.c        \
       ${DRIVERS_SRC}/spi.c        \
       ${DRIVERS_SRC}/mcasp.c      \
       ${DRIVERS_SRC}/emac.c       \
       ${DRIVERS_SRC}/lan8710a.c   \
       ${DRIVERS_SRC}/mdio.c       \
       ${DRIVERS_SRC}/rtc.c        \
       ${DRIVERS_SRC}/raster.c     \
       ${DRIVERS_SRC}/edma.c       \
       ${DRIVERS_SRC}/ehrpwm.c     \
       ${DRIVERS_SRC}/emifa.c      \
       ${DRIVERS_SRC}/lidd.c       \
       ${DRIVERS_SRC}/cppi41dma.c  \
       ${DRIVERS_SRC}/vpif.c

#
# Uncomment this line to disable high speed USB (i.e. use full speed instead)
#
#CFLAGS += -DUSB_MODE_HS_DISABLE

#
# Rules for building the source files
#
all: debug release

debug: 
	mkdir -p Debug
	$(CC) $(CFLAGS) -g $(SOURCE)
	$(AR) $(ARFLAGS) Debug/drivers.lib *.o*
	@mv *.o* Debug/
	@mkdir -p ${DRIVERS_BIN}/Debug
	@cp Debug/drivers.lib ${DRIVERS_BIN}/Debug/drivers.lib

release: 
	mkdir -p Release
	$(CC) $(CFLAGS) -O2 $(SOURCE)
	$(AR) $(ARFLAGS) Release/drivers.lib *.o*
	@mv *.o* Release/
	@mkdir -p ${DRIVERS_BIN}/Release
	@cp Release/drivers.lib ${DRIVERS_BIN}/Release/drivers.lib

#
# Rules for cleaning 
#
clean:
	@rm -rf Debug Release ${DRIVERS_BIN}/Debug ${DRIVERS_BIN}/Release
