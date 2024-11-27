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
ROOT=../../../../

#
# Include makefile definitions. 
#
include ${ROOT}/build/armv5/gcc/makedefs

#
# Source files for given target
#
SOURCE=${GRLIB_SRC}/canvas.c                \
       ${GRLIB_SRC}/checkbox.c              \
       ${GRLIB_SRC}/circle.c                \
       ${GRLIB_SRC}/container.c             \
       ${GRLIB_SRC}/context.c               \
       ${GRLIB_SRC}/image.c                 \
       ${GRLIB_SRC}/imgbutton.c             \
       ${GRLIB_SRC}/line.c                  \
       ${GRLIB_SRC}/listbox.c               \
       ${GRLIB_SRC}/offscr1bpp.c            \
       ${GRLIB_SRC}/offscr4bpp.c            \
       ${GRLIB_SRC}/offscr8bpp.c            \
       ${GRLIB_SRC}/offscr16bpp.c           \
       ${GRLIB_SRC}/pushbutton.c            \
       ${GRLIB_SRC}/radiobutton.c           \
       ${GRLIB_SRC}/rectangle.c             \
       ${GRLIB_SRC}/slider.c                \
       ${GRLIB_SRC}/string.c                \
       ${GRLIB_SRC}/widget.c                \
       ${GRLIB_SRC}/fonts/fontfixed6x8.c    \
       ${GRLIB_SRC}/fonts/fontcmss18b.c     \
       ${GRLIB_SRC}/fonts/fontcm12.c        \
       ${GRLIB_SRC}/fonts/fontcm22i.c       \
       ${GRLIB_SRC}/fonts/fontcm26b.c       \
       ${GRLIB_SRC}/fonts/fontcm34b.c       \
       ${GRLIB_SRC}/fonts/fontcmss38b.c     \
       ${GRLIB_SRC}/fonts/fontcm14.c        \
       ${GRLIB_SRC}/fonts/fontcm18.c        \
       ${GRLIB_SRC}/fonts/fontcm22.c        \
       ${GRLIB_SRC}/fonts/fontcm24.c        \
       ${GRLIB_SRC}/fonts/fontcm20.c        \
       ${GRLIB_SRC}/fonts/fontcmss18.c      \
       ${GRLIB_SRC}/fonts/fontcmss20b.c     \
       ${GRLIB_SRC}/fonts/fontcmss22b.c

#
# Un-define gcc macro for this build
#
CFLAGS+=-Ugcc

#
# Rules for building the source files
#
all: debug release

debug: 
	mkdir -p Debug
	$(CC) $(CFLAGS) -g $(SOURCE)
	$(AR) $(ARFLAGS) Debug/libgrlib.a *.o*
	@mv *.o* Debug/
	@mkdir -p ${GRLIB_BIN}/Debug
	@cp Debug/libgrlib.a ${GRLIB_BIN}/Debug/libgrlib.a

release: 
	mkdir -p Release
	$(CC) $(CFLAGS) -g -O2 $(SOURCE)
	$(AR) $(ARFLAGS) Release/libgrlib.a *.o*
	@mv *.o* Release/
	@mkdir -p ${GRLIB_BIN}/Release
	@cp Release/libgrlib.a ${GRLIB_BIN}/Release/libgrlib.a

#
# Rules for cleaning 
#
clean:
	@rm -rf Debug Release ${GRLIB_BIN}/Debug ${GRLIB_BIN}/Release
