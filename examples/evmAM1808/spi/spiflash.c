/**
 * \file spi.c
 *
 * \brief  This is a sample application file which invokes some APIs
 *         from the SPI device abstraction layer to perform configuration,
 *         transmission and reception operations.
 *
 */

/*
* Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/ 
*
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <string.h>
#include "soc_AM1808.h"
#include "hw_psc_AM1808.h"
#include "hw_syscfg0_AM1808.h"
#include "hw_pllc_AM1808.h"
#include "evmAM1808.h"
#include "uart.h"
#include "edma.h"
#include "edma_event.h"
#include "gpio.h"
#include "spi.h"
#include "psc.h"
#include "interrupt.h"
#include "uartStdio.h"

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/
/* value to configure SMIO,SOMI,CLK and CS pin as functional pin */
#define SIMO_SOMI_CLK_CS        0x00000E01
#define CHAR_LENGTH             0x8

/* flash address where data will be written and read */
#define SPI_FLASH_ADDR_MSB1     0x0A
#define SPI_FLASH_ADDR_MSB0     0x00
#define SPI_FLASH_ADDR_LSB      0x00

/* sector erase command */
#define SPI_FLASH_SECTOR_ERASE  0xD8

/* page program command */
#define SPI_FLASH_PAGE_WRITE    0x02

/* status register read command */
#define SPI_FLASH_STATUS_RX     0x05

/* write enable command */
#define SPI_FLASH_WRITE_EN      0x06

/* flash data read command */
#define SPI_FLASH_READ          0x03

/* flash data read */
#define WRITE_IN_PROGRESS       0x01

/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void SPIConfigDataFmtReg(unsigned long dataFormat);
static void ReadFromFlash(void);
static void IsFlashBusy(void);
static void WritetoFlash(void);
static void WriteEnable(void);
static void SpiTransfer(void);
static void SectorErase(void);
static void VerifyData(void);
static void StatusGet(void);
static void SetUpInt(void);
static void SetUpSPI(void);
void SPIIsr(void);

/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
volatile unsigned int flag = 1;
volatile unsigned int len;
volatile unsigned int len2;
char vrf_data[260];
char tx_data[260];
char rx_data[260];
char *p_tx;
char *p_rx;

const uint8_t mustbezero;

static const char *hexlut = "0123456789abcdef";
static void puthex(unsigned char c) {
    UARTPutc(hexlut[(c >> 4) & 0xf]);
    UARTPutc(hexlut[c & 0xf]);
}

static void puthex32(uint32_t x) {
    puthex(x >> 24);
    puthex(x >> 16);
    puthex(x >> 8);
    puthex(x);
}

#define GPIO_CS     23
// #define GPIO_MOSI   134
// #define GPIO_MISO   135
// #define GPIO_CLK    25

// static uint8_t bitbang(uint8_t cout) {
//     uint8_t cin = 0;
//     for (int i = 0; i < 8; i++) {
//         GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_MOSI, !!(cout & (1 << (7 - i))));
//         for (int j = 0; j < 10; j++) __asm__ volatile("");
//         if (GPIOPinRead(SOC_GPIO_0_REGS, GPIO_MISO))
//             cin |= (1 << (7 - i));
//         GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_CLK, 1);
//         for (int j = 0; j < 10; j++) __asm__ volatile("");
//         GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_CLK, 0);
//     }
//     return cin;
// }

/******************************************************************************
**                      INTERNAL FUNCTION DEFINITIONS
*******************************************************************************/
int main(void)
{
    /* Waking up the SPI0 instance. */
    PSCModuleControl(SOC_PSC_0_REGS, HW_PSC_SPI0, PSC_POWERDOMAIN_ALWAYS_ON,
                     PSC_MDCTL_NEXT_ENABLE);

    /* Initializing the UART instance for serial communication. */
    UARTStdioInit();
    
    UARTPuts("StarterWare AM1808 SPI application.\r\n\r\n", -1);
    UARTPuts("Here the SPI controller on the SoC communicates with", -1);
    UARTPuts(" the SPI Flash present on the SoM.\r\n\r\n", -1);

    PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_GPIO, PSC_POWERDOMAIN_ALWAYS_ON,
		     PSC_MDCTL_NEXT_ENABLE);
    uint32_t x;
    // WP GP5[2]
    x = HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(12));
    HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(12)) = (x & 0xFF0FFFFF) | (8 << 20);
    GPIODirModeSet(SOC_GPIO_0_REGS, 83, GPIO_DIR_OUTPUT);
    GPIOPinWrite(SOC_GPIO_0_REGS, 83, GPIO_PIN_HIGH);
    // HOLD GP2[0]
    x = HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(6));
    HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(6)) = (x & 0x0FFFFFFF) | (8 << 28);
    GPIODirModeSet(SOC_GPIO_0_REGS, 33, GPIO_DIR_OUTPUT);
    GPIOPinWrite(SOC_GPIO_0_REGS, 33, GPIO_PIN_HIGH);

    // ADCCS GP8[2]
    x = HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(3));
    HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(3)) = (x & 0xF0FFFFFF) | (4 << 24);
    GPIODirModeSet(SOC_GPIO_0_REGS, 131, GPIO_DIR_OUTPUT);
    GPIOPinWrite(SOC_GPIO_0_REGS, 131, GPIO_PIN_HIGH);

    // enable edma
    /* Enabling the PSC for EDMA3CC_0).*/ 
    PSCModuleControl(SOC_PSC_0_REGS, HW_PSC_CC0, PSC_POWERDOMAIN_ALWAYS_ON,
		     PSC_MDCTL_NEXT_ENABLE);
    /* Enabling the PSC for EDMA3TC_0.*/
    PSCModuleControl(SOC_PSC_0_REGS, HW_PSC_TC0, PSC_POWERDOMAIN_ALWAYS_ON,
		     PSC_MDCTL_NEXT_ENABLE);
    /* Initialization of EDMA3 */    
    EDMA3Init(SOC_EDMA30CC_0_REGS, 0);
    
    // LEDs
    GPIODirModeSet(SOC_GPIO_0_REGS, 109, GPIO_DIR_OUTPUT);
    GPIOPinWrite(SOC_GPIO_0_REGS, 109, GPIO_PIN_LOW);
    GPIODirModeSet(SOC_GPIO_0_REGS, 110, GPIO_DIR_OUTPUT);
    GPIOPinWrite(SOC_GPIO_0_REGS, 110, GPIO_PIN_LOW);
    GPIODirModeSet(SOC_GPIO_0_REGS, 111, GPIO_DIR_OUTPUT);
    GPIOPinWrite(SOC_GPIO_0_REGS, 111, GPIO_PIN_HIGH);
    GPIODirModeSet(SOC_GPIO_0_REGS, 104, GPIO_DIR_OUTPUT);
    GPIOPinWrite(SOC_GPIO_0_REGS, 104, GPIO_PIN_HIGH);

    // // wtf???
    // x = HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(3));
    // HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(3)) = (x & 0xFFFF00F0) | (0x4404);
    // x = HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(4));
    // HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(4)) = (x & 0xFFFFFF0F) | (4 << 4);
    // GPIODirModeSet(SOC_GPIO_0_REGS, GPIO_CLK, GPIO_DIR_OUTPUT);
    // GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_CLK, GPIO_PIN_LOW);
    // GPIODirModeSet(SOC_GPIO_0_REGS, GPIO_MOSI, GPIO_DIR_OUTPUT);
    // GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_MOSI, GPIO_PIN_LOW);
    // GPIODirModeSet(SOC_GPIO_0_REGS, GPIO_MISO, GPIO_DIR_OUTPUT);
    // GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_MISO, GPIO_PIN_LOW);
    // GPIODirModeSet(SOC_GPIO_0_REGS, GPIO_CS, GPIO_DIR_OUTPUT);
    // GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_CS, GPIO_PIN_HIGH);

    // GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_CS, 0);
    // puthex(bitbang(0x9f));
    // puthex(bitbang(0x5a));
    // puthex(bitbang(0xa5));
    // puthex(bitbang(0x33));
    // GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_CS, 1);
    // UARTPuts("\r\n", -1);

    // while (1) {}

    /* Performing the Pin Multiplexing for SPI0. */
    SPIPinMuxSetup(0);

    /* 
    ** Using the Chip Select(CS) 0 pin of SPI0 to communicate with SPI Flash.
    ** AM1808 EVM mandates us to do so.
    */
    SPI0CSPinMuxSetup(0);

    UARTPuts("PINMUX:\r\n", -1);
    for (int i = 0; i < 20; i++) {
        puthex32(HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(i)));
        UARTPuts("\r\n", -1);
    }
    UARTPuts("\r\n", -1);
    
    UARTPuts("PLLC0:\r\n", -1);

    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_REVID));       UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_RSTYPE));      UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_PLLCTL));      UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_OCSEL));       UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_PLLM));        UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_PREDIV));      UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_PLLDIV1));     UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_PLLDIV2));     UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_PLLDIV3));     UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_OSCDIV));      UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_POSTDIV));     UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_PLLCMD));      UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_PLLSTAT));     UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_ALNCTL));      UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_DCHANGE));     UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_CKEN));        UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_CKSTAT));      UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_SYSTAT));      UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_PLLDIV4));     UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_PLLDIV5));     UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_PLLDIV6));     UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_PLLDIV7));     UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_EMUCNT0));     UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_0_REGS + PLLC_EMUCNT1));     UARTPuts("\r\n", -1);

    UARTPuts("\r\n", -1);
    
    UARTPuts("PLLC1:\r\n", -1);

    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_REVID));       UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_RSTYPE));      UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_PLLCTL));      UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_OCSEL));       UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_PLLM));        UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_PREDIV));      UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_PLLDIV1));     UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_PLLDIV2));     UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_PLLDIV3));     UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_OSCDIV));      UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_POSTDIV));     UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_PLLCMD));      UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_PLLSTAT));     UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_ALNCTL));      UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_DCHANGE));     UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_CKEN));        UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_CKSTAT));      UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_SYSTAT));      UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_PLLDIV4));     UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_PLLDIV5));     UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_PLLDIV6));     UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_PLLDIV7));     UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_EMUCNT0));     UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_PLLC_1_REGS + PLLC_EMUCNT1));     UARTPuts("\r\n", -1);

    UARTPuts("\r\n", -1);

    // // polling hax??
    // SPIReset(SOC_SPI_0_REGS);
    // SPIOutOfReset(SOC_SPI_0_REGS);
    // SPIModeConfigure(SOC_SPI_0_REGS, SPI_MASTER_MODE);
    // SPIClkConfigure(SOC_SPI_0_REGS, 150000000, 1000000, SPI_DATA_FORMAT0);
    // unsigned int blah = SIMO_SOMI_CLK_CS;
    // SPIPinControl(SOC_SPI_0_REGS, 0, 0, &blah);
    // SPIDefaultCSSet(SOC_SPI_0_REGS, 1);
    // SPIConfigDataFmtReg(SPI_DATA_FORMAT0);
    // SPIDat1Config(SOC_SPI_0_REGS, SPI_CSHOLD | SPI_DATA_FORMAT0, 1);
    // SPIEnable(SOC_SPI_0_REGS);

    // // HWREG(SOC_SPI_0_REGS + SPI_SPIGCR1) |= (1 << 16);

    // UARTPuts("hihi\r\n", -1);
    // // GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_CS, 0);
    // HWREG(SOC_SPI_0_REGS + SPI_SPIFLG) = SPI_SPIFLG_RXINTFLG;

    // // *(volatile unsigned char *)(SOC_SPI_0_REGS + SPI_SPIDAT1 + 3) = 0x10;
    // while (!(HWREG(SOC_SPI_0_REGS + SPI_SPIFLG) & SPI_SPIFLG_TXINTFLG)) {}
    // // *(volatile unsigned char *)(SOC_SPI_0_REGS + SPI_SPIDAT1) = 0x9f;
    // HWREG(SOC_SPI_0_REGS + SPI_SPIDAT1) = SPI_CSHOLD | 0x9f;
    // while (!(HWREG(SOC_SPI_0_REGS + SPI_SPIFLG) & SPI_SPIFLG_RXINTFLG)) {}
    // puthex32(HWREG(SOC_SPI_0_REGS + SPI_SPIBUF));
    // UARTPuts("\r\n", -1);
    // while (!(HWREG(SOC_SPI_0_REGS + SPI_SPIFLG) & SPI_SPIFLG_TXINTFLG)) {}
    // // *(volatile unsigned char *)(SOC_SPI_0_REGS + SPI_SPIDAT1) = 0x5a;
    // HWREG(SOC_SPI_0_REGS + SPI_SPIDAT1) = SPI_CSHOLD | 0x5a;
    // while (!(HWREG(SOC_SPI_0_REGS + SPI_SPIFLG) & SPI_SPIFLG_RXINTFLG)) {}
    // puthex32(HWREG(SOC_SPI_0_REGS + SPI_SPIBUF));
    // UARTPuts("\r\n", -1);
    // while (!(HWREG(SOC_SPI_0_REGS + SPI_SPIFLG) & SPI_SPIFLG_TXINTFLG)) {}
    // // *(volatile unsigned char *)(SOC_SPI_0_REGS + SPI_SPIDAT1) = 0xa5;
    // HWREG(SOC_SPI_0_REGS + SPI_SPIDAT1) = SPI_CSHOLD | 0xa5;
    // while (!(HWREG(SOC_SPI_0_REGS + SPI_SPIFLG) & SPI_SPIFLG_RXINTFLG)) {}
    // puthex32(HWREG(SOC_SPI_0_REGS + SPI_SPIBUF));
    // UARTPuts("\r\n", -1);
    // while (!(HWREG(SOC_SPI_0_REGS + SPI_SPIFLG) & SPI_SPIFLG_TXINTFLG)) {}
    // // *(volatile unsigned char *)(SOC_SPI_0_REGS + SPI_SPIDAT1 + 3) = 0;
    // // *(volatile unsigned char *)(SOC_SPI_0_REGS + SPI_SPIDAT1) = 0x33;
    // HWREG(SOC_SPI_0_REGS + SPI_SPIDAT1) = 0x33;
    // while (!(HWREG(SOC_SPI_0_REGS + SPI_SPIFLG) & SPI_SPIFLG_RXINTFLG)) {}
    // puthex32(HWREG(SOC_SPI_0_REGS + SPI_SPIBUF));
    // UARTPuts("\r\n", -1);
    // // GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_CS, 1);

    // for (int i = 0; i < 10; i++)
    //     __asm__ volatile("");

    // // GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_CS, 0);

    // // *(volatile unsigned char *)(SOC_SPI_0_REGS + SPI_SPIDAT1 + 3) = 0x10;
    // while (!(HWREG(SOC_SPI_0_REGS + SPI_SPIFLG) & SPI_SPIFLG_TXINTFLG)) {}
    // // *(volatile unsigned char *)(SOC_SPI_0_REGS + SPI_SPIDAT1) = 0x9f;
    // HWREG(SOC_SPI_0_REGS + SPI_SPIDAT1) = SPI_CSHOLD | 0x9f;
    // while (!(HWREG(SOC_SPI_0_REGS + SPI_SPIFLG) & SPI_SPIFLG_RXINTFLG)) {}
    // puthex32(HWREG(SOC_SPI_0_REGS + SPI_SPIBUF));
    // UARTPuts("\r\n", -1);
    // while (!(HWREG(SOC_SPI_0_REGS + SPI_SPIFLG) & SPI_SPIFLG_TXINTFLG)) {}
    // // *(volatile unsigned char *)(SOC_SPI_0_REGS + SPI_SPIDAT1) = 0x5a;
    // HWREG(SOC_SPI_0_REGS + SPI_SPIDAT1) = SPI_CSHOLD | 0x5a;
    // while (!(HWREG(SOC_SPI_0_REGS + SPI_SPIFLG) & SPI_SPIFLG_RXINTFLG)) {}
    // puthex32(HWREG(SOC_SPI_0_REGS + SPI_SPIBUF));
    // UARTPuts("\r\n", -1);
    // while (!(HWREG(SOC_SPI_0_REGS + SPI_SPIFLG) & SPI_SPIFLG_TXINTFLG)) {}
    // // *(volatile unsigned char *)(SOC_SPI_0_REGS + SPI_SPIDAT1) = 0xa5;
    // HWREG(SOC_SPI_0_REGS + SPI_SPIDAT1) = SPI_CSHOLD | 0xa5;
    // while (!(HWREG(SOC_SPI_0_REGS + SPI_SPIFLG) & SPI_SPIFLG_RXINTFLG)) {}
    // puthex32(HWREG(SOC_SPI_0_REGS + SPI_SPIBUF));
    // UARTPuts("\r\n", -1);
    // while (!(HWREG(SOC_SPI_0_REGS + SPI_SPIFLG) & SPI_SPIFLG_TXINTFLG)) {}
    // // *(volatile unsigned char *)(SOC_SPI_0_REGS + SPI_SPIDAT1 + 3) = 0;
    // // *(volatile unsigned char *)(SOC_SPI_0_REGS + SPI_SPIDAT1) = 0x33;
    // HWREG(SOC_SPI_0_REGS + SPI_SPIDAT1) = 0x33;
    // while (!(HWREG(SOC_SPI_0_REGS + SPI_SPIFLG) & SPI_SPIFLG_RXINTFLG)) {}
    // puthex32(HWREG(SOC_SPI_0_REGS + SPI_SPIBUF));
    // UARTPuts("\r\n", -1);
    // // GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_CS, 1);

    // while (1) {}

    /* Enable use of SPI0 interrupts. */
    SetUpInt();

    /* Configuring and enabling the SPI0 instance. */
    SetUpSPI();

    /* Request DMA Channel and TCC for SPI Transmit*/
    EDMA3RequestChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA, \
                        EDMA3_CHA_SPI0_TX, EDMA3_CHA_SPI0_TX, 0);

    // // wtf edma test
    // EDMA3CCPaRAMEntry paramSet;
    // paramSet.srcAddr = 0xc1080000;
    // paramSet.destAddr = (unsigned int)rx_data;
    // paramSet.aCnt = 1;
    // paramSet.bCnt = 256;
    // paramSet.cCnt = 1;
    // paramSet.srcBIdx = 1;
    // paramSet.destBIdx = 1;
    // paramSet.srcCIdx = 0;
    // paramSet.destCIdx = 0;
    // paramSet.linkAddr = 0xffff;
    // paramSet.bCntReload = 0;
    // paramSet.opt = EDMA3CC_OPT_TCINTEN | (EDMA3_CHA_SPI0_TX << EDMA3CC_OPT_TCC_SHIFT);
    // // EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, EDMA3_CHA_SPI0_TX, &paramSet);
    // for (int x = 0; x < 30; x++)
    //     *(volatile unsigned char *)(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + EDMA3_CHA_SPI0_TX*32 + x) = *((unsigned char *)&paramSet + x);
    // *(volatile unsigned char *)(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + EDMA3_CHA_SPI0_TX*32 + 30) = 0;
    // *(volatile unsigned char *)(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + EDMA3_CHA_SPI0_TX*32 + 31) = 0;

    // EDMA3EnableTransfer(SOC_EDMA30CC_0_REGS, EDMA3_CHA_SPI0_TX, EDMA3_TRIG_MODE_MANUAL);
    // for (int i = 0; i < 256; i++)
    //     EDMA3SetEvt(SOC_EDMA30CC_0_REGS, EDMA3_CHA_SPI0_TX);

    // for (int i = 0; i < 100; i++) __asm__ volatile("");

    // for (int i = 0; i < 256; i++) {
    //     puthex(rx_data[i]);
    //     UARTPutc(' ');
    //     if (i % 16 == 15)
    //         UARTPuts("\r\n", -1);
    // }

    // while (1) {}

    // HWREG(SOC_SPI_0_REGS + SPI_SPIGCR1) |= (1 << 16);
    HWREG(SOC_SPI_0_REGS + SPI_SPIFLG) = SPI_SPIFLG_RXINTFLG;

    // //// XXX chip select hack
    // x = HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(4));
    // HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(4)) = (x & 0xFFFFFF0F) | (4 << 4);
    // GPIODirModeSet(SOC_GPIO_0_REGS, GPIO_CS, GPIO_DIR_OUTPUT);
    // GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_CS, GPIO_PIN_HIGH);

    UARTPuts("hihi\r\n", -1);
    tx_data[0] = 0x9f;
    tx_data[1] = 0x5a;
    tx_data[2] = 0xa5;
    tx_data[3] = 0x33;
    len = len2 = 4;
    // GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_CS, 0);
    // *(volatile unsigned char *)(SOC_SPI_0_REGS + SPI_SPIDAT1 + 3) = 0x10;
    // SPIDat1Config(SOC_SPI_0_REGS, (SPI_CSHOLD | SPI_DATA_FORMAT0), 0x1);
    SpiTransfer();
    // GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_CS, 1);
    puthex(rx_data[0]);
    puthex(rx_data[1]);
    puthex(rx_data[2]);
    puthex(rx_data[3]);
    UARTPuts("\r\nhihi2\r\n", -1);
    
    len = len2 = 4;
    // GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_CS, 0);
    // SPIDat1Config(SOC_SPI_0_REGS, (SPI_CSHOLD | SPI_DATA_FORMAT0), 0x1);
    // *(volatile unsigned char *)(SOC_SPI_0_REGS + SPI_SPIDAT1 + 3) = 0x10;
    SpiTransfer();
    // GPIOPinWrite(SOC_GPIO_0_REGS, GPIO_CS, 1);
    puthex(rx_data[0]);
    puthex(rx_data[1]);
    puthex(rx_data[2]);
    puthex(rx_data[3]);
    UARTPuts("\r\n", -1);

    // while (1) {}

    /* Preparing the Flash for a Write. */
    WriteEnable();
    UARTPuts("WREN ok\r\n", -1);

    /* Erasing a Sector of the Flash. */
    SectorErase();

    UARTPuts("erased\r\n", -1);

    WriteEnable();
    UARTPuts("WREN ok 2\r\n", -1);

    /* Programming the necessary data to Flash. */
    WritetoFlash();

    UARTPuts("written\r\n", -1);

    /* Reading from the required location from Flash. */
    ReadFromFlash();

    UARTPuts("read\r\n", -1);

    /* Comparing the written and read data. */
    VerifyData();

    while(1);
}

/*
** Reads the status register of m25p80 flash 
**
*/
static void StatusGet(void)
{
    tx_data[0] = SPI_FLASH_STATUS_RX;
    tx_data[1] = 0x5a;
    len = len2 = 2;
    // SPIDat1Config(SOC_SPI_0_REGS, (SPI_CSHOLD | SPI_DATA_FORMAT0), 0x1);
    SpiTransfer();
}

/* 
** Enables write to m25p80 flash 
**
*/
static void WriteEnable(void)
{
    tx_data[0] = SPI_FLASH_WRITE_EN;
    len = len2 = 1;
    // SPIDat1Config(SOC_SPI_0_REGS, (SPI_CSHOLD | SPI_DATA_FORMAT0), 0x1);
    SpiTransfer();
} 

/* 
** It polls for write in progress bit(wip)in status register of m25p80 flash 
**
*/
static void IsFlashBusy(void)
{
    do{
         StatusGet();
        //  UARTPuts("status = ", -1);
        //  puthex(rx_data[1]);
        //  UARTPuts("\r\n", -1);

      }while(rx_data[1] & WRITE_IN_PROGRESS);
}

/* 
** Set the all bits to 1 in chosen sector 
**
*/
static void SectorErase(void)
{
    tx_data[0] =  SPI_FLASH_SECTOR_ERASE;
    tx_data[1] =  SPI_FLASH_ADDR_MSB1;
    tx_data[2] =  SPI_FLASH_ADDR_MSB0;
    tx_data[3] =  SPI_FLASH_ADDR_LSB;

    len = len2 = 4;
    // SPIDat1Config(SOC_SPI_0_REGS, (SPI_CSHOLD | SPI_DATA_FORMAT0), 0x1);
    SpiTransfer(); 
    
    IsFlashBusy();
}

/* 
** Writes 256 bytes of data to desired memory address 
**
*/
static void WritetoFlash(void)
{
    unsigned int index;

    tx_data[0] =  SPI_FLASH_PAGE_WRITE;
    tx_data[1] =  SPI_FLASH_ADDR_MSB1;
    tx_data[2] =  SPI_FLASH_ADDR_MSB0;
    tx_data[3] =  SPI_FLASH_ADDR_LSB;

    /* Populate the data to be written to the flash */
    for (index = 4; index < 260; index++)
    {
        tx_data[index] =  index;
    }
 
    for(index = 4; index < 260; index++)
    { 
         vrf_data[index] = index;
    }

    len = len2 = index;
    // SPIDat1Config(SOC_SPI_0_REGS, (SPI_CSHOLD | SPI_DATA_FORMAT0), 0x1);
    SpiTransfer();

    IsFlashBusy();
}

/* 
** Reads the data from addressed memory of flash 
**
*/
static void ReadFromFlash(void)
{
    unsigned int index;

    UARTPuts("read wtf??\r\n", -1);

    tx_data[0] =  SPI_FLASH_READ;
    tx_data[1] =  SPI_FLASH_ADDR_MSB1;
    tx_data[2] =  SPI_FLASH_ADDR_MSB0;
    tx_data[3] =  SPI_FLASH_ADDR_LSB;
    UARTPuts("read wtf3??\r\n", -1);

    /* Reset the data in the tx buffer */
    for (index = 4; index < 260; index++)
    {
        tx_data[index] =  1;
    }
    UARTPuts("read wtf4??\r\n", -1);

    len = len2 = index;

    UARTPuts("read wtf2??\r\n", -1);
    // SPIDat1Config(SOC_SPI_0_REGS, (SPI_CSHOLD | SPI_DATA_FORMAT0), 0x1);
    SpiTransfer();
}

/* 
** Verfies the data written to flash and read from the flash of same address 
**
*/
static void VerifyData(void)
{
    unsigned int index;

    for(index = 4; index < 260; index++)
    { 
        if(vrf_data[index] != rx_data[index])
        {
            UARTPuts("\r\n", -1);
            UARTPuts("VerifyData: Comparing the data written to and read", -1);
            UARTPuts(" from Flash.\r\nThe two data blocks are unequal.", -1);
            UARTPuts(" Mismatch found at index ", -1);
            UARTPutNum((int)index - 3);
            UARTPuts("\r\n", -1);

            for (int i = 0; i < 256; i++) {
                puthex(vrf_data[4 + i]);
                UARTPutc(' ');
                if (i % 16 == 15)
                    UARTPuts("\r\n", -1);
            }
            UARTPuts("\r\n", -1);
            for (int i = 0; i < 256; i++) {
                puthex(rx_data[4 + i]);
                UARTPutc(' ');
                if (i % 16 == 15)
                    UARTPuts("\r\n", -1);
            }
            break;
        }
    }

    if (index == 260)
    {
        UARTPuts("\r\nThe data in the Flash and the one written ", -1);
        UARTPuts("to it are equal.\r\n", -1);
    }

}


/*
** EDMA3 Error Interrupt Service Routine(ISR).
*/

static void Edma3CCErrHandlerIsr(void)
{
    volatile unsigned int pendingIrqs = 0;
    unsigned int Cnt = 0;
    unsigned int index = 1;
    unsigned int regionNum = 0;  
    unsigned int evtqueNum = 0;

    IntSystemStatusClear(SYS_INT_CCERRINT);

    UARTPutc('?');

    if((HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_EMR) != 0 )
        || (HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_QEMR) != 0)
        || (HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_CCERR) != 0))
    {
        /* Loop for EDMA3CC_ERR_HANDLER_RETRY_COUNT number of time, breaks 
           when no pending interrupt is found. */
        while ((Cnt < EDMA3CC_ERR_HANDLER_RETRY_COUNT) && (index != 0))
        {
            index = 0;
            pendingIrqs = HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_EMR);
            while (pendingIrqs)
            {
                /*Process all the pending interrupts.*/
                if((pendingIrqs & 1)==TRUE)
                {
                    /* Write to EMCR to clear the corresponding EMR bits.*/
                    HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_EMCR) = (1<<index);
                    /*Clear any SER*/
                    HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_S_SECR(regionNum)) = (1<<index);
                }
                ++index;
                pendingIrqs >>= 1;
            }
            index = 0;
            pendingIrqs = HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_QEMR);
            while (pendingIrqs)
            {
                /*Process all the pending interrupts.*/
                if((pendingIrqs & 1)==TRUE)
                {
                    /* Here write to QEMCR to clear the corresponding QEMR bits.*/
                    HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_QEMCR) = (1<<index);
                    /*Clear any QSER*/
                    HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_S_QSECR(0)) = (1<<index);
                }
                ++index;
                pendingIrqs >>= 1;
            }
            index = 0;
            pendingIrqs = HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_CCERR);
            if(pendingIrqs != 0)
            {
                /* Process all the pending CC error interrupts. */
                /* Queue threshold error for different event queues.*/
                for (evtqueNum = 0; evtqueNum < EDMA3_0_NUM_EVTQUE; evtqueNum++)
                {
                    if((pendingIrqs & (1 << evtqueNum)) != 0)
                    {
                        /* Clear the error interrupt. */
                        HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_CCERRCLR) = (1 << evtqueNum);
                    }
                }

                /* Transfer completion code error. */
                if ((pendingIrqs & (1 << EDMA3CC_CCERR_TCCERR_SHIFT)) != 0)
                {
                    HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_CCERRCLR) = \
                         (0x01 << EDMA3CC_CCERR_TCCERR_SHIFT);
                }
                ++index;
            }
            Cnt++;
        }
    }
}

static void Edma3ComplHandlerIsr(void)
{
    IntSystemStatusClear(SYS_INT_CCINT0);
    uint32_t ipr;
    do {
        ipr = EDMA3GetIntrStatus(SOC_EDMA30CC_0_REGS);
        UARTPutc('e');
        puthex32(ipr);
        for (int i = 0; i < 32; i++) {
            if (ipr & (1 << i)) {
                UARTPutc('E');
                puthex(i);
            }
        }
        HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_ICR) = ipr;
    } while (ipr);
}

/*
** Configures ARM interrupt controller to generate SPI interrupt
**
*/
static void SetUpInt(void)
{
   /* Initialize the ARM Interrupt Controller.*/
    IntAINTCInit();

   /* Register the ISR in the Interrupt Vector Table.*/
    IntRegister(SYS_INT_SPINT0, SPIIsr);

    /* Set the channnel number 2 of AINTC for system interrupt 20.
     * Channel 2 is mapped to IRQ interrupt of ARM9.
     */
    IntChannelSet(SYS_INT_SPINT0, 2);

    /* Enable the System Interrupts for AINTC.*/
    IntSystemEnable(SYS_INT_SPINT0);

    IntRegister(SYS_INT_CCINT0, Edma3ComplHandlerIsr);
    IntChannelSet(SYS_INT_CCINT0, 2);
    IntSystemEnable(SYS_INT_CCINT0);

    IntRegister(SYS_INT_CCERRINT, Edma3CCErrHandlerIsr);
    IntChannelSet(SYS_INT_CCERRINT, 2);
    IntSystemEnable(SYS_INT_CCERRINT);

    /* Enable IRQ in CPSR.*/
    IntMasterIRQEnable();

    /* Enable the interrupts in GER of AINTC.*/
    IntGlobalEnable();

    /* Enable the interrupts in HIER of AINTC.*/
    IntIRQEnable();
}

/*
** Configures SPI Controller
**
*/
static void SetUpSPI(void)
{
    unsigned char cs  = 0x01;
    unsigned char dcs = 0x01;
    unsigned int  val = SIMO_SOMI_CLK_CS;
    
    SPIReset(SOC_SPI_0_REGS);

    SPIOutOfReset(SOC_SPI_0_REGS);

    SPIModeConfigure(SOC_SPI_0_REGS, SPI_MASTER_MODE);

    SPIClkConfigure(SOC_SPI_0_REGS, 150000000, 1000000, SPI_DATA_FORMAT0);

    SPIPinControl(SOC_SPI_0_REGS, 0, 0, &val);

    SPIDefaultCSSet(SOC_SPI_0_REGS, dcs);

    /* Configures SPI Data Format Register */
    SPIConfigDataFmtReg(SPI_DATA_FORMAT0);
  
     /* Selects the SPI Data format register to used and Sets CSHOLD 
      * to assert CS pin(line)  
      */
    SPIDat1Config(SOC_SPI_0_REGS, (SPI_CSHOLD | SPI_DATA_FORMAT0), cs);

     /* map interrupts to interrupt line INT1 */
    SPIIntLevelSet(SOC_SPI_0_REGS, SPI_RECV_INTLVL | SPI_TRANSMIT_INTLVL);
    
    /* Enable SPI communication */
    SPIEnable(SOC_SPI_0_REGS);
}
/*
** Configures Data Format register of SPI
**
*/
static void SPIConfigDataFmtReg(unsigned long dataFormat)
{
    /* Configures the polarity and phase of SPI clock */
    SPIConfigClkFormat(SOC_SPI_0_REGS,
                       (SPI_CLK_POL_HIGH | SPI_CLK_INPHASE),
                       dataFormat);

    /* Configures SPI to transmit MSB bit First during data transfer */
    SPIShiftMsbFirst(SOC_SPI_0_REGS, dataFormat);

    /* Sets the Charcter length */
    SPICharLengthSet(SOC_SPI_0_REGS, CHAR_LENGTH, dataFormat);
}

/*
** Enables SPI Transmit and Receive interrupt.
** Deasserts Chip Select line.
*/
static void  SpiTransfer(void)
{
    p_tx = &tx_data[0];
    p_rx = &rx_data[0];

    EDMA3CCPaRAMEntry paramSet;
    if (len > 1) {
        *(volatile unsigned char *)(SOC_SPI_0_REGS + SPI_SPIDAT1 + 3) = 0x10;
        paramSet.srcAddr = (unsigned int)tx_data;
        paramSet.destAddr = SOC_SPI_0_REGS + SPI_SPIDAT1;
        paramSet.aCnt = 1;
        paramSet.bCnt = len - 1;
        paramSet.cCnt = 1;
        paramSet.srcBIdx = 1;
        paramSet.destBIdx = 0;
        paramSet.srcCIdx = 0;
        paramSet.destCIdx = 0;
        paramSet.linkAddr = 0xffff;
        paramSet.linkAddr = 126 * 32;
        paramSet.bCntReload = 0;
        paramSet.opt = EDMA3CC_OPT_TCINTEN | (EDMA3_CHA_SPI0_TX << EDMA3CC_OPT_TCC_SHIFT);
        // EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, EDMA3_CHA_SPI0_TX, &paramSet);
        for (int x = 0; x < 30; x++)
            *(volatile unsigned char *)(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + EDMA3_CHA_SPI0_TX*32 + x) = *((unsigned char *)&paramSet + x);
        *(volatile unsigned char *)(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + EDMA3_CHA_SPI0_TX*32 + 30) = 0;
        *(volatile unsigned char *)(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + EDMA3_CHA_SPI0_TX*32 + 31) = 0;

        paramSet.srcAddr = (unsigned int)&mustbezero;
        paramSet.destAddr = SOC_SPI_0_REGS + SPI_SPIDAT1 + 3;
        paramSet.bCnt = 1;
        paramSet.linkAddr = 127 * 32;
        // EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, 126, &paramSet);
        for (int x = 0; x < 30; x++)
            *(volatile unsigned char *)(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + 126*32 + x) = *((unsigned char *)&paramSet + x);
        *(volatile unsigned char *)(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + 126*32 + 30) = 0;
        *(volatile unsigned char *)(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + 126*32 + 31) = 0;

        paramSet.srcAddr = (unsigned int)&tx_data[len - 1];
        paramSet.destAddr = SOC_SPI_0_REGS + SPI_SPIDAT1;
        paramSet.linkAddr = 0xffff;
        // EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, 127, &paramSet);
        for (int x = 0; x < 30; x++)
            *(volatile unsigned char *)(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + 127*32 + x) = *((unsigned char *)&paramSet + x);
        *(volatile unsigned char *)(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + 127*32 + 30) = 0;
        *(volatile unsigned char *)(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + 127*32 + 31) = 0;
    } else {
        *(volatile unsigned char *)(SOC_SPI_0_REGS + SPI_SPIDAT1 + 3) = 0;
        paramSet.srcAddr = (unsigned int)tx_data;
        paramSet.destAddr = SOC_SPI_0_REGS + SPI_SPIDAT1;
        paramSet.aCnt = 1;
        paramSet.bCnt = len;
        paramSet.cCnt = 1;
        paramSet.srcBIdx = 1;
        paramSet.destBIdx = 0;
        paramSet.srcCIdx = 0;
        paramSet.destCIdx = 0;
        paramSet.linkAddr = 0xffff;
        paramSet.bCntReload = 0;
        paramSet.opt = EDMA3CC_OPT_TCINTEN | (EDMA3_CHA_SPI0_TX << EDMA3CC_OPT_TCC_SHIFT);
        // EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, EDMA3_CHA_SPI0_TX, &paramSet);
        for (int x = 0; x < 30; x++)
            *(volatile unsigned char *)(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + EDMA3_CHA_SPI0_TX*32 + x) = *((unsigned char *)&paramSet + x);
        *(volatile unsigned char *)(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + EDMA3_CHA_SPI0_TX*32 + 30) = 0;
        *(volatile unsigned char *)(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + EDMA3_CHA_SPI0_TX*32 + 31) = 0;
    }

    UARTPuts("xxx setup params\r\n", -1);
    puthex32(HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + EDMA3_CHA_SPI0_TX*32 + 0x00));
    UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + EDMA3_CHA_SPI0_TX*32 + 0x04));
    UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + EDMA3_CHA_SPI0_TX*32 + 0x08));
    UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + EDMA3_CHA_SPI0_TX*32 + 0x0c));
    UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + EDMA3_CHA_SPI0_TX*32 + 0x10));
    UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + EDMA3_CHA_SPI0_TX*32 + 0x14));
    UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + EDMA3_CHA_SPI0_TX*32 + 0x18));
    UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_PaRAM_BASE + EDMA3_CHA_SPI0_TX*32 + 0x1c));
    UARTPuts("\r\n", -1);
    UARTPuts("\r\n", -1);

    __asm__ volatile("":::"memory");
    
    EDMA3EnableTransfer(SOC_EDMA30CC_0_REGS, EDMA3_CHA_SPI0_TX, EDMA3_TRIG_MODE_EVENT);
    SPIIntEnable(SOC_SPI_0_REGS, (SPI_RECV_INT | SPI_DMA_REQUEST_ENA_INT));

    // EDMA3SetEvt(SOC_EDMA30CC_0_REGS, EDMA3_CHA_SPI0_TX);

    for (int i = 0; i < 1000; i++) __asm__ volatile("");
    puthex32(HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_ER));
    UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_EER));
    UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_IER));
    UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_EDMA30CC_0_REGS + EDMA3CC_IPR));
    UARTPuts("\r\n", -1);
    UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_SPI_0_REGS + SPI_SPIINT0));
    UARTPuts("\r\n", -1);
    puthex32(HWREG(SOC_SPI_0_REGS + SPI_SPIFLG));
    UARTPuts("\r\n", -1);

    while(flag);
    flag = 1;
    // /* Deasserts the CS pin(line) */
    // SPIDat1Config(SOC_SPI_0_REGS, SPI_DATA_FORMAT0, 0);
}

/*
** Data transmission and receiption SPIIsr
**
*/
void SPIIsr(void)
{
    unsigned long intCode = 0;

    IntSystemStatusClear(SYS_INT_SPINT0);

    intCode = SPIInterruptVectorGet(SOC_SPI_0_REGS);

    while (intCode)
    {
        if(intCode == SPI_TX_BUF_EMPTY)
        {
            UARTPutc('t');
            // len--;
            // if (!len)
            //     *(volatile unsigned char *)(SOC_SPI_0_REGS + SPI_SPIDAT1 + 3) = 0;
            // *(volatile unsigned char *)(SOC_SPI_0_REGS + SPI_SPIDAT1) = *p_tx;
            // // SPITransmitData1(SOC_SPI_0_REGS, *p_tx);
            // p_tx++;
            // if (!len)
            // {
            //     SPIIntDisable(SOC_SPI_0_REGS, SPI_TRANSMIT_INT);
            // }
        }

        if(intCode == SPI_RECV_FULL)
        {
            UARTPutc('r');
            len2--;
            *p_rx = (char)SPIDataReceive(SOC_SPI_0_REGS);
            p_rx++;
            if (!len2)
            {
                __asm__ volatile("":::"memory");
                flag = 0;
                SPIIntDisable(SOC_SPI_0_REGS, SPI_RECV_INT);
            }
        }

        intCode = SPIInterruptVectorGet(SOC_SPI_0_REGS);
    }
}

/******************************* End of file *********************************/
