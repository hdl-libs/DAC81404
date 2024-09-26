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

#include "dac81404_ctrl.h"
#include "xparameters.h"
#include <stdlib.h>

extern int reg_read32(uint32_t addr, uint32_t *value);
extern int reg_write32(uint32_t addr, const uint32_t *value);

uint32_t dac_baseaddr[1] = {
    XPAR_DA_H_DAC81404_WRAPPER_0_BASEADDR,
};

/***************************************************************************
 * @brief reset the dac81404 chip
 *
 * @param dev           - The device structure.
 *
 * @return 0 for success or negative error code.
 *******************************************************************************/
int dac81404_soft_rst(dac81404_ctrl_t *dev)
{
    // check if dev is valid
    if (dev == NULL)
        return -1;

    dev->ctrl.all = 0;
    dev->ctrl.soft_rst = 1;
    reg_write32(dev->base_addr + offsetof(dac81404_ctrl_t, ctrl), &dev->ctrl.all);

    do
    {
        reg_read32(dev->base_addr + offsetof(dac81404_ctrl_t, ctrl), &dev->ctrl.all);

        // !todo: time out check
        // if (timeout)
        //    return -5;

    } while (dev->ctrl.soft_rst);

    return 0;
}

int dac81404_set_automode(dac81404_ctrl_t *dev, bool en, bool sync, bool brdcast)
{
    // check if dev is valid
    if (dev == NULL)
        return -1;

    reg_read32(dev->base_addr + offsetof(dac81404_ctrl_t, ctrl), &dev->ctrl.all);
    dev->ctrl.cfg_auto_mode = en;
    dev->ctrl.cfg_sync_mode = sync;
    dev->ctrl.cfg_brdcast_mode = brdcast;
    reg_write32(dev->base_addr + offsetof(dac81404_ctrl_t, ctrl), &dev->ctrl.all);

    return 0;
}

int dac81404_set_clear(dac81404_ctrl_t *dev)
{
    // check if dev is valid
    if (dev == NULL)
        return -1;

    reg_read32(dev->base_addr + offsetof(dac81404_ctrl_t, ctrl), &dev->ctrl.all);
    dev->ctrl.dac_clr = 1;
    reg_write32(dev->base_addr + offsetof(dac81404_ctrl_t, ctrl), &dev->ctrl.all);

    do
    {
        reg_read32(dev->base_addr + offsetof(dac81404_ctrl_t, ctrl), &dev->ctrl.all);

        // !todo: time out check
        // if (timeout)
        //    return -4;

    } while (dev->ctrl.dac_clr);

    return 0;
}

int dac81404_set_load(dac81404_ctrl_t *dev)
{
    // check if dev is valid
    if (dev == NULL)
        return -1;

    reg_read32(dev->base_addr + offsetof(dac81404_ctrl_t, ctrl), &dev->ctrl.all);
    dev->ctrl.dac_ldac = 1;
    reg_write32(dev->base_addr + offsetof(dac81404_ctrl_t, ctrl), &dev->ctrl.all);

    do
    {
        reg_read32(dev->base_addr + offsetof(dac81404_ctrl_t, ctrl), &dev->ctrl.all);

        // !todo: time out check
        // if (timeout)
        //    return -4;

    } while (dev->ctrl.dac_ldac);

    return 0;
}

int dac81404_set_update_rate(dac81404_ctrl_t *dev, double update_rate)
{
    // check if dev is valid
    if (dev == NULL)
        return -1;

    // check if sample_rete is valid
    if (update_rate > FPGA_CLK_FREQ || update_rate < 0)
        return -2;

    // check if any channel is enabled
    reg_read32(dev->base_addr + offsetof(dac81404_ctrl_t, channel_en), &dev->channel_en);

    if ((update_rate != 0) && (dev->channel_en != 0))
        dev->scan_period = FPGA_CLK_FREQ / update_rate;
    else
        dev->scan_period = 0;

    // disbale auto scan
    reg_read32(dev->base_addr + offsetof(dac81404_ctrl_t, ctrl), &dev->ctrl.all);
    dev->ctrl.cfg_auto_mode = 0;
    reg_write32(dev->base_addr + offsetof(dac81404_ctrl_t, ctrl), &dev->ctrl.all);

    // set sample rate
    reg_write32(dev->base_addr + offsetof(dac81404_ctrl_t, scan_period), &dev->scan_period);

    // enable auto scan
    if (dev->scan_period > 0)
    {
        reg_read32(dev->base_addr + offsetof(dac81404_ctrl_t, ctrl), &dev->ctrl.all);
        dev->ctrl.cfg_auto_mode = 1;
        reg_write32(dev->base_addr + offsetof(dac81404_ctrl_t, ctrl), &dev->ctrl.all);
    }

    return 0;
}

/**
 * @brief dac81404_transfer   		DAC 传输数据
 * @param *dev                   	DAC 句柄
 * @param update_rate               刷新率
 * @param *data                   	数据源
 * @param length                   	数据长度
 * @return                      	0:成功
 */
int dac81404_transfer(dac81404_ctrl_t *dev, double update_rate, uint64_t *data, uint32_t length, bool sync, bool brdcast)
{
    if (dev == NULL || data == NULL || length <= 0)
        return -1;

    if (dac81404_set_automode(dev, false, sync, brdcast))
        return -2;
    if (dac81404_set_update_rate(dev, update_rate))
        return -3;
    if (dac81404_set_automode(dev, true, sync, brdcast))
        return -2;

    return 0;
}

int dac81404_transfer_check(dac81404_ctrl_t *dev)
{
    if (dev == NULL)
        return -1;

    reg_read32(dev->base_addr + offsetof(dac81404_ctrl_t, status), &dev->status.all);
    if (dev->status.dac_busy)
        return 1;
    else if (dev->status.dac_fault)
        return -2;
    else if (!dev->status.dac_busy && !dev->status.dac_fault)
    {
        uint32_t clear = 0xff;
        reg_write32(dev->base_addr + offsetof(dac81404_ctrl_t, status), &clear);
        return 0;
    }
    else
        return 2;
}

/***************************************************************************
 * @brief Writes data into a register.
 *
 * @param dev      - The device structure.
 * @param spi_addr - The address of the register to be written.
 * @param spi_data  - The value to be written into the register.
 *
 * @return Returns 0 in case of success or negative error code.
 *******************************************************************************/
int dac81404_spi_transfer(dac81404_ctrl_t *dev, uint8_t spi_addr, uint16_t spi_wdata, uint16_t *spi_rdata)
{
    // check if dev is valid
    if (dev == NULL)
        return -1;

    // !todo: check if spi_addr is valid
    // if (...)
    //    return -2;

    // check if spi is busy
    reg_read32(dev->base_addr + offsetof(dac81404_ctrl_t, status), &dev->status.all);
    if (dev->status.spi_busy)
        return -3;

    // set addr
    reg_write32(dev->base_addr + offsetof(dac81404_ctrl_t, addr), (uint32_t *)&spi_addr);

    // set write data
    reg_write32(dev->base_addr + offsetof(dac81404_ctrl_t, wr_data), (uint32_t *)&spi_wdata);

    // read back ctrl reg
    reg_read32(dev->base_addr + offsetof(dac81404_ctrl_t, ctrl), &dev->ctrl.all);

    // mark spi start bit as 1 then write ctrl reg to start spi
    dev->ctrl.cfg_spi_start = 1;
    reg_write32(dev->base_addr + offsetof(dac81404_ctrl_t, ctrl), &dev->ctrl.all);

    // wait until spi is done, this step may not be nessary for pc
    do
    {
        reg_read32(dev->base_addr + offsetof(dac81404_ctrl_t, status), &dev->status.all);

        // !todo: time out check
        // if (timeout)
        //    return -4;

    } while (dev->status.spi_busy);

    if (!dev->status.spi_done)
        return -5;

    // printf("[I]: addr:%2x, wdata:%2x\r\n", spi_addr, spi_wdata);
    // takeout spi read data
    if (spi_rdata)
    {
        reg_read32(dev->base_addr + offsetof(dac81404_ctrl_t, rd_data), &dev->rd_data);
        *spi_rdata = (uint16_t)(dev->rd_data);
        // printf("[I]: addr:%2x, rdata:%x\r\n", spi_addr, dev->rd_data);
    }

    return 0;
}

int dac81404_spi_write_read(dac81404_ctrl_t *dev, uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len)
{
    // check if dev is valid
    if (dev == NULL)
        return -1;

    uint8_t addr = tx_buf[0];
    uint16_t wdata = (uint16_t)((tx_buf[2] << 8) | tx_buf[1]);
    uint16_t rdata;

    dac81404_ctrl_t *dev_int = (dac81404_ctrl_t *)dev;

    if (tx_buf && rx_buf && (len == 3))
    {
        // read data
        if (dac81404_spi_transfer(dev_int, addr, wdata, &rdata))
            return -2;

        rx_buf[0] = addr;
        rx_buf[1] = (uint8_t)rdata;
        rx_buf[2] = (uint8_t)(rdata >> 8);
    }
    else if (tx_buf && !rx_buf && (len == 3))
    {
        // write data
        if (dac81404_spi_transfer(dev_int, addr, wdata, NULL))
            return -3;
    }
    else
    {
        return -4;
    }

    return 0;
}

int dac81404_set_spi_div(dac81404_ctrl_t *dev, int div)
{
    if (dev == NULL)
        return -1;

    // check if div is valid
    if (div > FPGA_CLK_FREQ || div <= 0)
        return -2;

    div = (div < 4) ? 4 : div;

    dev->baud_div = div;
    reg_write32(dev->base_addr + offsetof(dac81404_ctrl_t, baud_div), &dev->baud_div);

    reg_read32(dev->base_addr + offsetof(dac81404_ctrl_t, ctrl), &dev->ctrl.all);
    dev->ctrl.baud_load = 1;
    reg_write32(dev->base_addr + offsetof(dac81404_ctrl_t, ctrl), &dev->ctrl.all);

    return 0;
}

int dac81404_spi_init(dac81404_ctrl_t **desc, int id)
{
    if (desc == NULL)
        return -1;

    dac81404_ctrl_t *dac81404_ctrl = (dac81404_ctrl_t *)calloc(1, sizeof(dac81404_ctrl_t));
    dac81404_ctrl->base_addr = dac_baseaddr[id];

    // soft reset
    if (dac81404_soft_rst(dac81404_ctrl))
        return -3;

    *desc = dac81404_ctrl;

    return 0;
}
