// ---------------------------------------------------------------------------------------
// Copyright (c) 2024 john_tito All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// ---------------------------------------------------------------------------------------

/**
 * @file dac81404.h
 * @brief
 * @author
 */

#ifndef _DAC81404_H_
#define _DAC81404_H_

/******************************************************************************/
/************************ Include Files ***************************************/
/******************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include "dac81404_ctrl.h"

/******************************************************************************/
/************************ Marco Definitions ***********************************/
/******************************************************************************/

#define DAC81404_REG_WR(addr) (0x7FU & (addr))
#define DAC81404_REG_RD(addr) (0x80U | (addr))

// Registers
#define DAC81404_REG_NOP 0x00
#define DAC81404_REG_DEVICEID 0x01
#define DAC81404_REG_STATUS 0x02
#define DAC81404_REG_SPICONFIG 0x03
#define DAC81404_REG_GENCONFIG 0x04
#define DAC81404_REG_BRDCONFIG 0x05
#define DAC81404_REG_SYNCCONFIG 0x06
#define DAC81404_REG_DACPWDWN 0x09
#define DAC81404_REG_DACRANGE 0x0A
#define DAC81404_REG_TRIGGER 0x0E
#define DAC81404_REG_BRDCAST 0x0F
#define DAC81404_REG_DAC(n) ((n) + 0x10U)

/* DAC81404_REG_SPICONFIG */
#define DAC81404_TEMPALM_EN(x) ((x) << 11) // When set to 1, a thermal alarm triggers the FAULT pin.
#define DAC81404_DACBUSY_EN(x) ((x) << 10) // When set to 1, the FAULT pin is set between DAC output updates.
#define DAC81404_CRCALM_EN(x) ((x) << 9)   // Contrary to other alarm events, this alarm resets automatically.
#define DAC81404_DEV_PWDWN(x) ((x) << 5)   // When set to 1, the device is in power-down mode.
#define DAC81404_CRC_EN(x) ((x) << 4)      // When set to 1, frame error checking is enabled.
#define DAC81404_SDO_EN(x) ((x) << 2)      // When set to 1, the SDO pin is operational.
#define DAC81404_FSDO(x) ((x) << 1)        // When set to 1, the SDO updates on SCLK falling edges.

/* DAC81404_REG_GENCONFIG */
#define DAC81404_INTERNAL_VREF (0 << 14)
#define DAC81404_EXTERNAL_VREF (1 << 14)

/* DAC81404_REG_TRIGGER */
#define DAC81404_SOFT_CLR(x) ((x) << 9)  // Set this bit to 1 to clear all DAC outputs.
#define DAC81404_ALM_RESET(x) ((x) << 8) // Set this bit to 1 to clear an alarm event. Not applicable for a DACBUSY-alarm event.
#define DAC81404_SOFT_LDAC(x) ((x) << 4)
#define DAC81404_SOFT_RST 0b1010

#define DAC81404_ID 0x029CU
#define DAC81402_ID 0x0298U
#define DAC61404_ID 0x024CU
#define DAC61402_ID 0x0248U

#define DAC81404_MAX_CH_NUM 4U

#define DAC81404_MAX_WR_SPI_FREQ 50E6
#define DAC81404_MAX_RD_SPI_FREQ 8E6f

/******************************************************************************/
/************************ Types Definitions ***********************************/
/******************************************************************************/
enum DAC81404_REFSEL
{
    DAC81404_EXTREF = 0x4000U,
    DAC81404_INTREF = 0X0000U,
};

enum DAC81404_RANGE
{
    DAC81404_RANGE_0_5V = 0b0000,
    DAC81404_RANGE_0_6V = 0b1000,
    DAC81404_RANGE_0_10V = 0b0001,
    DAC81404_RANGE_0_12V = 0b1001,
    DAC81404_RANGE_0_20V = 0b0010,
    DAC81404_RANGE_0_24V = 0b1010,
    DAC81404_RANGE_0_40V = 0b0011,
    DAC81404_RANGE_5V = 0b0101,
    DAC81404_RANGE_6V = 0b1101,
    DAC81404_RANGE_10V = 0b0110,
    DAC81404_RANGE_12V = 0b1110,
    DAC81404_RANGE_20V = 0b0111,
};

enum DAC81404_SYNC_MODE
{
    SYNC_SS = 0,   // async
    SYNC_LDAC = 1, // sync to LDAC trigger
};

typedef union dac81404_range_t
{
    struct
    {
        uint16_t a : 4;
        uint16_t b : 4;
        uint16_t c : 4;
        uint16_t d : 4;
    };
    uint16_t all;
} dac81404_range_t;

typedef struct dac81404_config_t
{
    uint16_t vref_sel;
    dac81404_range_t range;
    uint16_t sync_en;
    uint16_t brdcast_en;
    uint16_t ch_en;
} dac81404_config_t;

typedef struct dac81404_dev_t
{
    dac81404_ctrl_t *spi_desc; /* SPI */
    dac81404_config_t config;  /* Device Settings */
    int is_opened;
} dac81404_dev_t;

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

extern int dac81404_open(dac81404_dev_t **dev_p, int id, int *ch_en, int *sync_en, int *brdcast_en, enum DAC81404_RANGE range, enum DAC81404_REFSEL verf);
extern int dac81404_close(dac81404_dev_t **dev_p);

/******************************************************************************/
/************************ Variable Declarations *******************************/
/******************************************************************************/
#endif // _DAC81404_H_
