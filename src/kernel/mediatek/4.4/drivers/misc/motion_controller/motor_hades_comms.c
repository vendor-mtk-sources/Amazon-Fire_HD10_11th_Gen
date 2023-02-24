/******************************************************************************
 * Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 *         file: motor_hades_comms.c
 *  description: Motor hades device communication source file
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************
 */

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/jiffies.h>
#include "motor_hades_comms.h"
#include "hds_host.h"
#include "abc123_motion.h"

void InitCompletion(struct hds_completion *hds_completion)
{
	hds_completion->flag = 0;
}

void SignalCompletion(struct hds_completion *hds_completion)
{
	hds_completion->flag = 1;
}

CompletionStatus WaitForCompletionTimeout(struct hds_completion *hds_completion,
	int timeout_us)
{
	int ret = wait_event_interruptible_timeout(hds_completion->wait_queue,
		hds_completion->flag == 1, usecs_to_jiffies(timeout_us));
	if (ret == 0)
		return COMP_TIMEOUT;
	else
		return COMP_RECEIVED;
}

SPIStatus SPI_Send(struct device *dev, const uint8_t *data, uint16_t length)
{
	struct spi_transfer t;
	struct spi_message m;
	int ret;
	struct spi_device *spi = container_of(dev, struct spi_device, dev);

	memset(&t, 0, sizeof(t));
	spi_message_init(&m);

	t.tx_buf = data;
	t.rx_buf = NULL;
	t.len = length;

	spi_message_add_tail(&t, &m);

	ret = spi_sync(spi, &m);
	if (ret < 0) {
		dev_err(dev, "sending start seq failed %d", ret);
		return -SPI_ERROR;
	}

	return SPI_OK;
}

SPIStatus SPI_Receive(struct device *dev, uint8_t *data, uint16_t length)
{
	struct spi_transfer t;
	struct spi_message m;
	int ret;
	struct spi_device *spi = container_of(dev, struct spi_device, dev);

	memset(&t, 0, sizeof(t));
	spi_message_init(&m);

	t.rx_buf = data;
	t.tx_buf = NULL;
	t.len = length;

	spi_message_add_tail(&t, &m);

	ret = spi_sync(spi, &m);
	if (ret < 0) {
		dev_err(dev, "sending start seq failed %d", ret);
		return -SPI_ERROR;
	}

	return SPI_OK;
}

uint32_t motion_motor_comms_checksum(const uint8_t *buf, size_t len)
{
	size_t idx32 = 0;
	uint32_t chksum = 0;
	size_t i, idx;

	const uint32_t *buf32 = (const uint32_t *)buf;

	if (!buf || (len == 0))
	{
		return 0;
	}

	do
	{
		if (len >= sizeof(uint32_t))
		{
			/* Sum up buffer in 4B chuncks */
			chksum += buf32[idx32++];
			len -= sizeof(uint32_t);
		} else
		{
			/* Pad remaining bytes with 0 */
			for (i = 0; i < len; i++)
			{
				idx = (idx32 * sizeof(uint32_t)) + i;
				chksum += ((uint32_t)buf[idx]) << (8 * i);
			}
			len = 0;
		}
	} while (len);

	return ~chksum;
}


/* Hades Host Framework ------------------------------------------------------- */
void Motor_Hades_SOF(void *context)
{
    struct abc123_motion *controller;
    struct device *dev = (struct device *) context;

    controller = dev_get_drvdata(dev);

    gpio_set_value(controller->pins.cs_gpio, 0);

    /* Clear completion flag on each start-of-frame */
    InitCompletion(&controller->hds_data.cts);

    /* Slight delay */
    udelay(HADES_POST_SOF_DELAY_US);
}

void Motor_Hades_EOF(void *context)
{
    struct device *dev;
    struct abc123_motion *controller;

    dev = (struct device *) context;
    controller = dev_get_drvdata(dev);

    gpio_set_value(controller->pins.cs_gpio, 1);

    /* Slight delay */
    udelay(HADES_POST_EOF_DELAY_US);
}

hds_phy_status_t Motor_Hades_PHY_Write(void *context,
				       const uint8_t *buf,
				       size_t len)
{
    SPIStatus sts;
    struct device *dev;

    if (!context)
    {
	return HDS_PHY_STATUS_ERROR;
    }

    dev = (struct device *) context;

    sts = SPI_Send(dev, (uint8_t *)buf, len);
    if (sts != SPI_OK)
    {
	dev_err(dev, "PHY WRITE ERR %d\n", sts);
	return HDS_PHY_STATUS_ERROR;
    }

    return HDS_PHY_STATUS_OK;
}

hds_phy_status_t Motor_Hades_PHY_Read(void *context,
				      uint8_t *buf,
				      size_t   len)
{
    SPIStatus sts;
    struct device *dev;

    if (!context)
    {
	return HDS_PHY_STATUS_ERROR;
    }

    dev = (struct device *) context;

    sts = SPI_Receive(dev, buf, len);
    if (sts != SPI_OK)
    {
	dev_err(dev, "PHY READ ERR %d\n", sts);
	return HDS_PHY_STATUS_ERROR;
    }

    return HDS_PHY_STATUS_OK;
}

hds_phy_cts_t Motor_Hades_CTS_Read(void *context)
{
    CompletionStatus status;
    struct device *dev;
    struct abc123_motion *controller;

    if (!context)
    {
	return HDS_PHY_CTS_TIMEOUT;
    }

    dev = (struct device *) context;
    controller = dev_get_drvdata(dev);

#if 0
    /* debug only, use reset gpio to simulate CTS */
    gpio_set_value(controller->pins.reset_gpio, 0);
    mdelay(1);
    gpio_set_value(controller->pins.reset_gpio, 1);
    mdelay(1);
    gpio_set_value(controller->pins.reset_gpio, 0);
#endif

    status = WaitForCompletionTimeout(&controller->hds_data.cts,
	controller->hds_data.response_timeout_us);

    InitCompletion(&controller->hds_data.cts);

    if (status != COMP_RECEIVED)
    {
	return HDS_PHY_CTS_TIMEOUT;
    }

    return HDS_PHY_CTS_ENABLE;
}

hds_retcode_t Motor_Write(struct hds_data *hd,
			  uint16_t         cmd,
			  const uint8_t   *buf,
			  uint16_t         len,
			  hds_tl_status_t *tl_status)
{
    hds_retcode_t rc;
    *tl_status = HDS_TL_STATUS_OK;

    mutex_lock(&hd->transfer_lock);
    rc = hds_host_write(hd->hds, cmd, buf, len, tl_status);
    if (rc != HDS_RETCODE_SUCCESS)
    {
	hds_host_stats_t stats;

	/* Read stats */
	hds_host_get_stats(hd->hds, &stats);
    }
    mutex_unlock(&hd->transfer_lock);

    return rc;
}

hds_retcode_t Motor_Read(struct hds_data *hd,
			 uint16_t         cmd,
			 uint8_t         *buf,
			 uint16_t        *len,
			 uint16_t         max_len,
			 hds_tl_status_t *tl_status)
{
    hds_retcode_t rc;
    *tl_status = HDS_TL_STATUS_OK;

    mutex_lock(&hd->transfer_lock);
    rc = hds_host_read(hd->hds, cmd, buf, len, max_len, tl_status);
    if (rc != HDS_RETCODE_SUCCESS)
    {
	hds_host_stats_t stats;

	/* Read stats */
	hds_host_get_stats(hd->hds, &stats);
    }
    mutex_unlock(&hd->transfer_lock);
    return rc;
}
