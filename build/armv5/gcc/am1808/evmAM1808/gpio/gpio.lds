/*
*  Copyright (C) 2010 Texas Instruments Incorporated. - http://www.ti.com/
*  All rights reserved.
*/

/*
 * ld script for SitaraWare
 */

/* The start is pointed to 0xC1080000 because the flash writer
 * tool and the bootloader are designed to locate the image here.
 * However, the start of DDR is still 0xC0000000 and 64MByte is its size.
 * The start of stack is placed at 0xC3FFFFFC, which is the end of 
 * DDR. The stack grows from here upwards (decrementing address).
 */

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)

SECTIONS
{

	. = 0xC1080000;
	. = ALIGN(4);
        .startcode     :
        {
               *init.o      (.text)
        }

	. = ALIGN(4);
	.text      :
	{
		
		*(.text)
	}

	. = ALIGN(4);

	.data : 
	{ 
		*(.data) 
	}
	
	. = ALIGN(4);

	_bss_start = .;
	.bss : 
	{ 
		
		*(.bss) 
	}
        . = ALIGN(4);

	_bss_end = .;	
	
	_stack = 0xC3FFFFFC;
}

