#******************************************************************************
#
# makefile - Rules for building the utility library.
#
# Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
#  ALL RIGHTS RESERVED
#
#******************************************************************************
#
# Locating the root directory 
#
ROOT=../../../../../

#
# Device definition (required for interrupt API prototypes)
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
SOURCE=${USBLIB_SRC}/usbbuffer.c             \
       ${USBLIB_SRC}/usbdesc.c               \
       ${USBLIB_SRC}/usbringbuf.c            \
       ${USBLIB_SRC}/usbtick.c               \
       ${USBLIB_SRC}/usbkeyboardmap.c        \
       ${USBLIB_SRC}/device/usbdcdc.c        \
       ${USBLIB_SRC}/device/usbdhid.c        \
       ${USBLIB_SRC}/device/usbdhidmouse.c   \
       ${USBLIB_SRC}/device/usbdmsc.c        \
       ${USBLIB_SRC}/device/usbdbulk.c       \
       ${USBLIB_SRC}/device/usbdcdesc.c      \
       ${USBLIB_SRC}/device/usbdcomp.c       \
       ${USBLIB_SRC}/device/usbdconfig.c     \
       ${USBLIB_SRC}/device/usbdenum.c       \
       ${USBLIB_SRC}/device/usbdhandler.c    \
       ${USBLIB_SRC}/host/usbhostenum.c      \
       ${USBLIB_SRC}/host/usbhhid.c          \
       ${USBLIB_SRC}/host/usbhmsc.c          \
       ${USBLIB_SRC}/host/usbhscsi.c         \
       ${USBLIB_SRC}/host/usbhhidmouse.c     \
       ${USBLIB_SRC}/host/usbhhidkeyboard.c

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
	$(CC) $(CFLAGS) -g -D DMA_MODE $(SOURCE)
	$(AR) $(ARFLAGS) Debug/usblib.lib *.o*
	@mv *.o* Debug/
	@mkdir -p ${USBLIB_BIN}/Debug
	@cp Debug/usblib.lib ${USBLIB_BIN}/Debug/usblib.lib

release: 
	mkdir -p Release
	$(CC) $(CFLAGS) -O2 -D DMA_MODE $(SOURCE)
	$(AR) $(ARFLAGS) Release/usblib.lib *.o*
	@mv *.o* Release/
	@mkdir -p ${USBLIB_BIN}/Release
	@cp Release/usblib.lib ${USBLIB_BIN}/Release/usblib.lib

#
# Rules for cleaning 
#
clean:
	@rm -rf Debug Release ${USBLIB_BIN}/Debug ${USBLIB_BIN}/Release
