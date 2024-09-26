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

#include "dac81404.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

extern int dac81404_set_update_rate(dac81404_ctrl_t *dev, double update_rate);
extern int dac81404_set_automode(dac81404_ctrl_t *dev, bool en, bool sync, bool brdcast);
extern int dac81404_soft_ld(dac81404_dev_t *dev);
extern void usleep(unsigned long useconds);
/**
 * @brief dac81404_write_reg   		DAC 寄存器写入
 * @param *dev                   	DAC 句柄
 * @param addr                   	写入地址
 * @param data                   	数据源
 * @return                      	0:成功
 */
int dac81404_write_reg(dac81404_dev_t *dev, uint8_t addr, uint16_t data)
{
    if (dev == NULL)
    {
        return -1;
    }

    uint8_t buf[3];
    buf[0] = DAC81404_REG_WR(addr);
    buf[1] = (uint8_t)data;
    buf[2] = (uint8_t)(data >> 8);
    dac81404_spi_write_read(dev->spi_desc, buf, NULL, 3);
    return 0;
}

/**
 * @brief dac81404_read_reg   		DAC 寄存器读取
 * @param *dev                   	DAC 句柄
 * @param addr                   	读取地址
 * @param data                   	数据空间
 * @return                      	0:成功
 */
int dac81404_read_reg(dac81404_dev_t *dev, uint8_t addr, uint16_t *data)
{
    if (dev == NULL || data == NULL)
    {
        return -1;
    }

    uint8_t buf[3];
    buf[0] = DAC81404_REG_RD(addr);
    buf[1] = 0;
    buf[2] = 0;

    dac81404_spi_write_read(dev->spi_desc, buf, buf, 3);
    *data = ((uint16_t)buf[1]);
    *data |= (((uint16_t)buf[2]) << 8);
    return 0;
}

/**
 * @brief dac81404_reset   			DAC 复位
 * @param *dev                   	DAC 句柄
 * @return                      	0:成功
 */
int dac81404_reset(dac81404_dev_t *dev)
{

    dac81404_write_reg(dev, DAC81404_REG_TRIGGER, DAC81404_SOFT_RST);

    usleep(1000);

    uint16_t def[2] = {DAC81404_TEMPALM_EN(1) |
                           DAC81404_DACBUSY_EN(1) |
                           DAC81404_CRCALM_EN(1) |
                           DAC81404_DEV_PWDWN(0) |
                           DAC81404_CRC_EN(0) |
                           DAC81404_SDO_EN(1) |
                           DAC81404_FSDO(0) |
                           (0x2 << 6),
                       0};

    dac81404_write_reg(dev, DAC81404_REG_SPICONFIG, def[0]);

    dac81404_read_reg(dev, DAC81404_REG_SPICONFIG, def + 1);

    return ((def[0] == def[1]) ? 0 : -1);
}

/**
 * @brief dac81404_set_pd   		设置 DAC 输出通道使能
 * @param *dev                   	DAC 句柄
 * @param ch                   		通道
 * @param pd                   		使能状态
 * @return                      	0:成功
 */
int dac81404_set_pd(dac81404_dev_t *dev, int ch, uint16_t pd)
{
    uint16_t old_state;
    dac81404_read_reg(dev, DAC81404_REG_DACPWDWN, &old_state);
    old_state = (ch == DAC81404_MAX_CH_NUM) ? pd
                                            : ((pd) ? (uint16_t)(old_state | (1 << ch))
                                                    : (uint16_t)(old_state & ~(1 << ch)));
    return dac81404_write_reg(dev, DAC81404_REG_DACPWDWN, old_state);
}

/**
 * @brief dac81404_get_pd   		获取 DAC 输出通道使能
 * @param *dev                   	DAC 句柄
 * @param ch                   		通道
 * @param *pd                   	使能状态
 * @return                      	0:成功
 */
int dac81404_get_pd(dac81404_dev_t *dev, int ch, uint16_t *pd)
{
    int ret = dac81404_read_reg(dev, DAC81404_REG_DACPWDWN, pd);
    *pd = (ch == DAC81404_MAX_CH_NUM) ? (*pd) : ((*pd >> ch) & 0x01U);
    return ret;
}

/**
 * @brief dac81404_set_sync   		设置 DAC 输出通道同步
 * @param *dev                   	DAC 句柄
 * @param ch                   		通道
 * @param pd                   		同步状态
 * @return                      	0:成功
 */
int dac81404_set_sync(dac81404_dev_t *dev, int ch, uint16_t sync)
{
    uint16_t old_state;
    dac81404_read_reg(dev, DAC81404_REG_SYNCCONFIG, &old_state);
    old_state = (ch == DAC81404_MAX_CH_NUM) ? sync
                                            : ((sync) ? (uint16_t)(old_state | (1 << ch))
                                                      : (uint16_t)(old_state & ~(1 << ch)));
    return dac81404_write_reg(dev, DAC81404_REG_SYNCCONFIG, old_state);
}

/**
 * @brief dac81404_get_sync   		获取 DAC 输出通道同步
 * @param *dev                   	DAC 句柄
 * @param ch                   		通道
 * @param *pd                 		同步状态
 * @return                      	0:成功
 */
int dac81404_get_sync(dac81404_dev_t *dev, int ch, uint16_t *sync)
{
    int ret = dac81404_read_reg(dev, DAC81404_REG_SYNCCONFIG, sync);
    *sync = (ch == DAC81404_MAX_CH_NUM) ? (*sync) : ((*sync >> ch) & 0x01U);
    return ret;
}

/**
 * @brief dac81404_set_brdcast   	获取 DAC 输出通道广播
 * @param *dev                   	DAC 句柄
 * @param ch                   		通道
 * @param pd                 		广播状态
 * @return                      	0:成功
 */
int dac81404_set_brdcast(dac81404_dev_t *dev, int ch, uint16_t brdcast)
{
    uint16_t old_state;
    dac81404_read_reg(dev, DAC81404_REG_BRDCONFIG, &old_state);
    old_state = (ch == DAC81404_MAX_CH_NUM) ? brdcast
                                            : ((brdcast) ? (uint16_t)(old_state | (1 << ch))
                                                         : (uint16_t)(old_state & ~(1 << ch)));
    return dac81404_write_reg(dev, DAC81404_REG_BRDCONFIG, old_state);
}

/**
 * @brief dac81404_get_brdcast   	获取 DAC 输出通道广播
 * @param *dev                   	DAC 句柄
 * @param ch                   		通道
 * @param *pd                 		广播状态
 * @return                      	0:成功
 */
int dac81404_get_brdcast(dac81404_dev_t *dev, int ch, uint16_t *brdcast)
{
    int ret = dac81404_read_reg(dev, DAC81404_REG_BRDCONFIG, brdcast);
    *brdcast = (ch == DAC81404_MAX_CH_NUM) ? (*brdcast) : ((*brdcast >> ch) & 0x01U);
    return ret;
}

/**
 * @brief dac81404_set_verf   		设置 DAC 参考值
 * @param *dev                   	DAC 句柄
 * @param src                 		参考值
 * @return                      	0:成功
 */
int dac81404_set_verf(dac81404_dev_t *dev, uint16_t src)
{
    return dac81404_write_reg(dev, DAC81404_REG_GENCONFIG, src); // Turn on
}

/**
 * @brief dac81404_get_verf   		获取 DAC 参考值
 * @param *dev                   	DAC 句柄
 * @param *src                 		参考值
 * @return                      	0:成功
 */
int dac81404_get_verf(dac81404_dev_t *dev, uint16_t *src)
{
    int ret = dac81404_read_reg(dev, DAC81404_REG_GENCONFIG, src);
    return ret;
}

/**
 * @brief dac81404_set_range   		设置 DAC 输出通道电压范围
 * @param *dev                   	DAC 句柄
 * @param ch                   		通道
 * @param range                 	电压范围
 * @return                      	0:成功
 */
int dac81404_set_range(dac81404_dev_t *dev, int ch, uint16_t range)
{
    uint16_t mask = (uint16_t)(0xf << (4 * ch));
    dev->config.range.all = (ch == DAC81404_MAX_CH_NUM) ? range
                                                        : ((dev->config.range.all & ~mask) | ((range << (4 * ch)) & mask));
    return dac81404_write_reg(dev, DAC81404_REG_DACRANGE, dev->config.range.all);
}

/**
 * @brief dac81404_get_range   		获取 DAC 输出通道电压范围
 * @param *dev                   	DAC 句柄
 * @param ch                   		通道
 * @param *range                 	电压范围
 * @return                      	0:成功
 */
int dac81404_get_range(dac81404_dev_t *dev, int ch, uint16_t *range)
{
    int ret = dac81404_read_reg(dev, DAC81404_REG_DACRANGE, range);
    *range = (ch == DAC81404_MAX_CH_NUM) ? (dev->config.range.all) : ((dev->config.range.all >> (4 * ch)) & 0xF);
    return ret;
}

/**
 * @brief dac81404_open   			DAC 打开
 * @param **dev_p                   DAC 句柄
 * @param id                   		DAC 序号
 * @param *ch_en                   	输出通道使能
 * @param *sync_en                  同步属性
 * @param *brdcast_en               广播属性
 * @param range               		范围
 * @param verf               		参考值
 * @return                      	0:成功
 */
int dac81404_open(dac81404_dev_t **dev_p, int id, int *ch_en, int *sync_en, int *brdcast_en, enum DAC81404_RANGE range, enum DAC81404_REFSEL verf)
{
    int ret = 0;

    dac81404_dev_t *dac_handel = (dac81404_dev_t *)calloc(1, sizeof(dac81404_dev_t));

    /* Initializes the SPI peripheral */
    ret = dac81404_spi_init(&dac_handel->spi_desc, id);
    if (ret)
        return ret;

    dac81404_set_spi_div(dac_handel->spi_desc, (int)(FPGA_CLK_FREQ / DAC81404_MAX_RD_SPI_FREQ));

    ret = dac81404_reset(dac_handel);
    if (ret)
        return ret;

    uint16_t sync_tmp = (uint16_t)(sync_en[0] | sync_en[1] << 1 | sync_en[2] << 2 | sync_en[3] << 3);
    dac81404_set_sync(dac_handel, DAC81404_MAX_CH_NUM, sync_tmp);
    dac81404_get_sync(dac_handel, DAC81404_MAX_CH_NUM, &dac_handel->config.sync_en);

    uint16_t brdcast_tmp = (uint16_t)(brdcast_en[0] | brdcast_en[1] << 1 | brdcast_en[2] << 2 | brdcast_en[3] << 3);
    dac81404_set_brdcast(dac_handel, DAC81404_MAX_CH_NUM, brdcast_tmp);
    dac81404_get_brdcast(dac_handel, DAC81404_MAX_CH_NUM, &dac_handel->config.brdcast_en);

    dac_handel->config.ch_en = ((uint16_t)(ch_en[0] | ch_en[1] << 1 | ch_en[2] << 2 | ch_en[3] << 3));
    dac81404_set_pd(dac_handel, DAC81404_MAX_CH_NUM, ~(dac_handel->config.ch_en));
    dac81404_get_pd(dac_handel, DAC81404_MAX_CH_NUM, &dac_handel->config.ch_en);
    dac_handel->config.ch_en = ~dac_handel->config.ch_en;

    dac_handel->config.range.a = (uint16_t)range;
    dac_handel->config.range.b = (uint16_t)range;
    dac_handel->config.range.c = (uint16_t)range;
    dac_handel->config.range.d = (uint16_t)range;
    dac81404_set_range(dac_handel, DAC81404_MAX_CH_NUM, dac_handel->config.range.all);
    dac81404_get_range(dac_handel, 0, &dac_handel->config.range.all);

    dac81404_set_verf(dac_handel, (uint16_t)verf);
    dac81404_get_verf(dac_handel, &dac_handel->config.vref_sel);

    uint16_t readbuf;
    ret = dac81404_read_reg(dac_handel, DAC81404_REG_DEVICEID, &readbuf);
    if (ret)
        return ret;

    // REFER 8.6.2 DEVICEID Register (address = 01h) [reset = 0A60h or 0920h]
    readbuf = readbuf >> 2;
    if (readbuf != DAC61404_ID && readbuf != DAC61402_ID && readbuf != DAC81404_ID && readbuf != DAC81402_ID)
        return -2;

    dac81404_set_spi_div(dac_handel->spi_desc, (int)(FPGA_CLK_FREQ / DAC81404_MAX_WR_SPI_FREQ));

    dac_handel->is_opened = 1;
    *dev_p = dac_handel;

    return 0;
}

/**
 * @brief dac81404_close   			DAC 关闭
 * @param *dev_p                   	DAC 句柄
 * @return                      	0:成功
 */
int dac81404_close(dac81404_dev_t **dev_p)
{
    if ((dev_p == NULL) || (*dev_p == NULL))
    {
        return -1;
    }

    if ((*dev_p)->is_opened)
    {
        dac81404_set_spi_div((*dev_p)->spi_desc, (int)(FPGA_CLK_FREQ / DAC81404_MAX_RD_SPI_FREQ));
        dac81404_set_pd(*dev_p, DAC81404_MAX_CH_NUM, 0xFFFF);
    }

    free(*dev_p);
    *dev_p = NULL;

    return 0;
}
