/****************************************************************************/
/* lidd.cmd -  v4.5.0 COMMAND FILE FOR LINKING TMS470 32BIS C/C++ PROGRAMS  */
/*                                                                          */
/*   Usage:  lnk470 <obj files...>    -o <out file> -m <map file> lnk32.cmd */
/*           cl470 <src files...> -z -o <out file> -m <map file> lnk32.cmd  */
/*                                                                          */
/*   Description: This file is a sample command file that can be used       */
/*                for linking programs built with the TMS470 C/C++          */
/*                Compiler.   Use it as a guideline; you may want to change */
/*                the allocation scheme according to the size of your       */
/*                program and the memory layout of your target system.      */
/*                                                                          */
/*   Notes: (1)   You must specify the directory in which run-time support  */
/*                library is located.  Either add a "-i<directory>" line to */
/*                this file, or use the system environment variable C_DIR   */
/*                to specify a search path for libraries.                   */
/*                                                                          */
/*          (2)   If the run-time support library you are using is not      */
/*                named below, be sure to use the correct name here.        */
/*                                                                          */
/****************************************************************************/
-stack  0x8000                             /* SOFTWARE STACK SIZE           */
-heap   0x2000                             /* HEAP AREA SIZE                */
-e Entry

/* SPECIFY THE SYSTEM MEMORY MAP */

MEMORY
{
		DDR_MEM    	: org = 0xC1080000   len = 0x2F7FFFF     /* RAM */
}

/* SPECIFY THE SECTIONS ALLOCATION INTO MEMORY */

SECTIONS
{
    .init 	 : { 
    			 system_config.lib<init.obj> (.text) 
    		   } load > 0xC1080000
  			
    .text    : load > DDR_MEM              /* CODE                              */
	.data    : load	> DDR_MEM
    .bss     : load > DDR_MEM              /* GLOBAL & STATIC VARS              */
    				RUN_START(bss_start),
					RUN_END(bss_end)
    .const   : load > DDR_MEM              /* SOFTWARE SYSTEM STACK             */
    .cinit   : load > DDR_MEM              /* SOFTWARE SYSTEM STACK             */
    .stack   : load > 0xC3FF7FFC           /* SOFTWARE SYSTEM STACK             */

}

