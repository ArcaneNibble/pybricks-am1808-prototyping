/**
 *  \file     i2cLedBlink.c
 *
 *  \brief    Sample application for i2c.
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

#include "hw_psc_AM1808.h"
#include "soc_AM1808.h"
#include "interrupt.h"
#include "evmAM1808.h"
#include "uart.h"
#include "uartStdio.h"
#include "i2c.h"

static const char *hexlut = "0123456789abcdef";
static void puthex(unsigned char c) {
    UARTPutc(hexlut[(c >> 4) & 0xf]);
    UARTPutc(hexlut[c & 0xf]);
}

int main(void)
{
    UARTStdioInit();
    
    UARTPuts("EV3 bootrom dumper\r\n\r\n", -1);

    // while (1) {}

    // unsigned int num  = 0;

    I2CPinMuxSetup(0);
    
    /* Put i2c in reset/disabled state */
    I2CMasterDisable(SOC_I2C_0_REGS);

    /* Configure i2c bus speed to 100khz */
    I2CMasterInitExpClk(SOC_I2C_0_REGS, 24000000, 8000000, 100000);

    I2CMasterControl(SOC_I2C_0_REGS, I2C_CFG_MST_TX);
  
    /* Bring i2c out of reset */
    I2CMasterEnable(SOC_I2C_0_REGS);

    /* Generate start condition on i2c bus */
    I2CSetDataCount(SOC_I2C_0_REGS, 2);
    I2CMasterStart(SOC_I2C_0_REGS);
    while (!(I2CMasterIntStatus(SOC_I2C_0_REGS) & I2C_ICSTR_ICXRDY)) {}
    I2CMasterDataPut(SOC_I2C_0_REGS, 0x00);
    while (!(I2CMasterIntStatus(SOC_I2C_0_REGS) & I2C_ICSTR_ICXRDY)) {}
    I2CMasterDataPut(SOC_I2C_0_REGS, 0x00);

    HWREG(SOC_I2C_0_REGS + I2C_ICMDR) &= ~I2C_ICMDR_TRX;
    I2CSetDataCount(SOC_I2C_0_REGS, 16*1024);
    I2CMasterStart(SOC_I2C_0_REGS);

    for (int i = 0; i < 16*1024; i++) {
        while (!(I2CMasterIntStatus(SOC_I2C_0_REGS) & I2C_ICSTR_ICRRDY)) {}
        puthex(I2CMasterDataGet(SOC_I2C_0_REGS));
        UARTPutc(' ');
        if (i % 16 == 15)
            UARTPuts("\r\n", -1);
    }

    I2CMasterStop(SOC_I2C_0_REGS);
}
