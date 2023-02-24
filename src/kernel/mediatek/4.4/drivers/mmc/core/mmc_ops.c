/*
 *  linux/drivers/mmc/core/mmc_ops.h
 *
 *  Copyright 2006-2007 Pierre Ossman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <linux/slab.h>
#include <linux/export.h>
#include <linux/types.h>
#include <linux/scatterlist.h>

#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>

#include "core.h"
#include "host.h"
#include "mmc_ops.h"

#define MMC_OPS_TIMEOUT_MS	(10 * 60 * 1000) /* 10 minute timeout */

static const u8 tuning_blk_pattern_4bit[] = {
	0xff, 0x0f, 0xff, 0x00, 0xff, 0xcc, 0xc3, 0xcc,
	0xc3, 0x3c, 0xcc, 0xff, 0xfe, 0xff, 0xfe, 0xef,
	0xff, 0xdf, 0xff, 0xdd, 0xff, 0xfb, 0xff, 0xfb,
	0xbf, 0xff, 0x7f, 0xff, 0x77, 0xf7, 0xbd, 0xef,
	0xff, 0xf0, 0xff, 0xf0, 0x0f, 0xfc, 0xcc, 0x3c,
	0xcc, 0x33, 0xcc, 0xcf, 0xff, 0xef, 0xff, 0xee,
	0xff, 0xfd, 0xff, 0xfd, 0xdf, 0xff, 0xbf, 0xff,
	0xbb, 0xff, 0xf7, 0xff, 0xf7, 0x7f, 0x7b, 0xde,
};

static const u8 tuning_blk_pattern_8bit[] = {
	0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00,
	0xff, 0xff, 0xcc, 0xcc, 0xcc, 0x33, 0xcc, 0xcc,
	0xcc, 0x33, 0x33, 0xcc, 0xcc, 0xcc, 0xff, 0xff,
	0xff, 0xee, 0xff, 0xff, 0xff, 0xee, 0xee, 0xff,
	0xff, 0xff, 0xdd, 0xff, 0xff, 0xff, 0xdd, 0xdd,
	0xff, 0xff, 0xff, 0xbb, 0xff, 0xff, 0xff, 0xbb,
	0xbb, 0xff, 0xff, 0xff, 0x77, 0xff, 0xff, 0xff,
	0x77, 0x77, 0xff, 0x77, 0xbb, 0xdd, 0xee, 0xff,
	0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00,
	0x00, 0xff, 0xff, 0xcc, 0xcc, 0xcc, 0x33, 0xcc,
	0xcc, 0xcc, 0x33, 0x33, 0xcc, 0xcc, 0xcc, 0xff,
	0xff, 0xff, 0xee, 0xff, 0xff, 0xff, 0xee, 0xee,
	0xff, 0xff, 0xff, 0xdd, 0xff, 0xff, 0xff, 0xdd,
	0xdd, 0xff, 0xff, 0xff, 0xbb, 0xff, 0xff, 0xff,
	0xbb, 0xbb, 0xff, 0xff, 0xff, 0x77, 0xff, 0xff,
	0xff, 0x77, 0x77, 0xff, 0x77, 0xbb, 0xdd, 0xee,
};

static inline int __mmc_send_status(struct mmc_card *card, u32 *status,
				    bool ignore_crc)
{
	int err;
	struct mmc_command cmd = {0};

	BUG_ON(!card);
	BUG_ON(!card->host);

	cmd.opcode = MMC_SEND_STATUS;
	if (!mmc_host_is_spi(card->host))
		cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;
	if (ignore_crc)
		cmd.flags &= ~MMC_RSP_CRC;

	err = mmc_wait_for_cmd(card->host, &cmd, MMC_CMD_RETRIES);
	if (err)
		return err;

	/* NOTE: callers are required to understand the difference
	 * between "native" and SPI format status words!
	 */
	if (status)
		*status = cmd.resp[0];

	return 0;
}

int mmc_send_status(struct mmc_card *card, u32 *status)
{
	return __mmc_send_status(card, status, false);
}

static int _mmc_select_card(struct mmc_host *host, struct mmc_card *card)
{
	int err;
	struct mmc_command cmd = {0};

	BUG_ON(!host);

	cmd.opcode = MMC_SELECT_CARD;

	if (card) {
		cmd.arg = card->rca << 16;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
	} else {
		cmd.arg = 0;
		cmd.flags = MMC_RSP_NONE | MMC_CMD_AC;
	}

	err = mmc_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES);
	if (err)
		return err;

	return 0;
}

int mmc_select_card(struct mmc_card *card)
{
	BUG_ON(!card);

	return _mmc_select_card(card->host, card);
}

int mmc_deselect_cards(struct mmc_host *host)
{
	return _mmc_select_card(host, NULL);
}

/*
 * Write the value specified in the device tree or board code into the optional
 * 16 bit Driver Stage Register. This can be used to tune raise/fall times and
 * drive strength of the DAT and CMD outputs. The actual meaning of a given
 * value is hardware dependant.
 * The presence of the DSR register can be determined from the CSD register,
 * bit 76.
 */
int mmc_set_dsr(struct mmc_host *host)
{
	struct mmc_command cmd = {0};

	cmd.opcode = MMC_SET_DSR;

	cmd.arg = (host->dsr << 16) | 0xffff;
	cmd.flags = MMC_RSP_NONE | MMC_CMD_AC;

	return mmc_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES);
}

int mmc_go_idle(struct mmc_host *host)
{
	int err;
	struct mmc_command cmd = {0};

	/*
	 * Non-SPI hosts need to prevent chipselect going active during
	 * GO_IDLE; that would put chips into SPI mode.  Remind them of
	 * that in case of hardware that won't pull up DAT3/nCS otherwise.
	 *
	 * SPI hosts ignore ios.chip_select; it's managed according to
	 * rules that must accommodate non-MMC slaves which this layer
	 * won't even know about.
	 */
	if (!mmc_host_is_spi(host)) {
		mmc_set_chip_select(host, MMC_CS_HIGH);
		mmc_delay(1);
	}

	cmd.opcode = MMC_GO_IDLE_STATE;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_NONE | MMC_CMD_BC;

	err = mmc_wait_for_cmd(host, &cmd, 0);

	mmc_delay(1);

	if (!mmc_host_is_spi(host)) {
		mmc_set_chip_select(host, MMC_CS_DONTCARE);
		mmc_delay(1);
	}

	host->use_spi_crc = 0;

	return err;
}

int mmc_send_op_cond(struct mmc_host *host, u32 ocr, u32 *rocr)
{
	struct mmc_command cmd = {0};
	int i, err = 0;

	BUG_ON(!host);

	cmd.opcode = MMC_SEND_OP_COND;
	cmd.arg = mmc_host_is_spi(host) ? 0 : ocr;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R3 | MMC_CMD_BCR;

	for (i = 100; i; i--) {
		err = mmc_wait_for_cmd(host, &cmd, 0);
		if (err)
			break;

		/* if we're just probing, do a single pass */
		if (ocr == 0)
			break;

		/* otherwise wait until reset completes */
		if (mmc_host_is_spi(host)) {
			if (!(cmd.resp[0] & R1_SPI_IDLE))
				break;
		} else {
			if (cmd.resp[0] & MMC_CARD_BUSY)
				break;
		}

		err = -ETIMEDOUT;

		mmc_delay(10);
	}

	if (rocr && !mmc_host_is_spi(host))
		*rocr = cmd.resp[0];

	return err;
}

int mmc_all_send_cid(struct mmc_host *host, u32 *cid)
{
	int err;
	struct mmc_command cmd = {0};

	BUG_ON(!host);
	BUG_ON(!cid);

	cmd.opcode = MMC_ALL_SEND_CID;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R2 | MMC_CMD_BCR;

	err = mmc_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES);
	if (err)
		return err;

	memcpy(cid, cmd.resp, sizeof(u32) * 4);

	return 0;
}

int mmc_set_relative_addr(struct mmc_card *card)
{
	int err;
	struct mmc_command cmd = {0};

	BUG_ON(!card);
	BUG_ON(!card->host);

	cmd.opcode = MMC_SET_RELATIVE_ADDR;
	cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	err = mmc_wait_for_cmd(card->host, &cmd, MMC_CMD_RETRIES);
	if (err)
		return err;

	return 0;
}

static int
mmc_send_cxd_native(struct mmc_host *host, u32 arg, u32 *cxd, int opcode)
{
	int err;
	struct mmc_command cmd = {0};

	BUG_ON(!host);
	BUG_ON(!cxd);

	cmd.opcode = opcode;
	cmd.arg = arg;
	cmd.flags = MMC_RSP_R2 | MMC_CMD_AC;

	err = mmc_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES);
	if (err)
		return err;

	memcpy(cxd, cmd.resp, sizeof(u32) * 4);

	return 0;
}

/*
 * NOTE: void *buf, caller for the buf is required to use DMA-capable
 * buffer or on-stack buffer (with some overhead in callee).
 */
static int
mmc_send_cxd_data(struct mmc_card *card, struct mmc_host *host,
		u32 opcode, void *buf, unsigned len)
{
	struct mmc_request mrq = {NULL};
	struct mmc_command cmd = {0};
	struct mmc_data data = {0};
	struct scatterlist sg;

	mrq.cmd = &cmd;
	mrq.data = &data;

	cmd.opcode = opcode;
	cmd.arg = 0;

	/* NOTE HACK:  the MMC_RSP_SPI_R1 is always correct here, but we
	 * rely on callers to never use this with "native" calls for reading
	 * CSD or CID.  Native versions of those commands use the R2 type,
	 * not R1 plus a data block.
	 */
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = len;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;

	sg_init_one(&sg, buf, len);

	if (opcode == MMC_SEND_CSD || opcode == MMC_SEND_CID) {
		/*
		 * The spec states that CSR and CID accesses have a timeout
		 * of 64 clock cycles.
		 */
		data.timeout_ns = 0;
		data.timeout_clks = 64;
	} else
		mmc_set_data_timeout(&data, card);

	mmc_wait_for_req(host, &mrq);

	if (cmd.error)
		return cmd.error;
	if (data.error)
		return data.error;

	return 0;
}

int mmc_send_csd(struct mmc_card *card, u32 *csd)
{
	int ret, i;
	u32 *csd_tmp;

	if (!mmc_host_is_spi(card->host))
		return mmc_send_cxd_native(card->host, card->rca << 16,
				csd, MMC_SEND_CSD);

	csd_tmp = kzalloc(16, GFP_KERNEL);
	if (!csd_tmp)
		return -ENOMEM;

	ret = mmc_send_cxd_data(card, card->host, MMC_SEND_CSD, csd_tmp, 16);
	if (ret)
		goto err;

	for (i = 0;i < 4;i++)
		csd[i] = be32_to_cpu(csd_tmp[i]);

err:
	kfree(csd_tmp);
	return ret;
}

int mmc_send_cid(struct mmc_host *host, u32 *cid)
{
	int ret, i;
	u32 *cid_tmp;

	if (!mmc_host_is_spi(host)) {
		if (!host->card)
			return -EINVAL;
		return mmc_send_cxd_native(host, host->card->rca << 16,
				cid, MMC_SEND_CID);
	}

	cid_tmp = kzalloc(16, GFP_KERNEL);
	if (!cid_tmp)
		return -ENOMEM;

	ret = mmc_send_cxd_data(NULL, host, MMC_SEND_CID, cid_tmp, 16);
	if (ret)
		goto err;

	for (i = 0;i < 4;i++)
		cid[i] = be32_to_cpu(cid_tmp[i]);

err:
	kfree(cid_tmp);
	return ret;
}

int mmc_get_ext_csd(struct mmc_card *card, u8 **new_ext_csd)
{
	int err;
	u8 *ext_csd;

	if (!card || !new_ext_csd)
		return -EINVAL;

	if (!mmc_can_ext_csd(card))
		return -EOPNOTSUPP;

	/*
	 * As the ext_csd is so large and mostly unused, we don't store the
	 * raw block in mmc_card.
	 */
	ext_csd = kzalloc(512, GFP_KERNEL);
	if (!ext_csd)
		return -ENOMEM;

	err = mmc_send_cxd_data(card, card->host, MMC_SEND_EXT_CSD, ext_csd,
				512);
	if (err)
		kfree(ext_csd);
	else
		*new_ext_csd = ext_csd;

	return err;
}
EXPORT_SYMBOL_GPL(mmc_get_ext_csd);

int mmc_spi_read_ocr(struct mmc_host *host, int highcap, u32 *ocrp)
{
	struct mmc_command cmd = {0};
	int err;

	cmd.opcode = MMC_SPI_READ_OCR;
	cmd.arg = highcap ? (1 << 30) : 0;
	cmd.flags = MMC_RSP_SPI_R3;

	err = mmc_wait_for_cmd(host, &cmd, 0);

	*ocrp = cmd.resp[1];
	return err;
}

int mmc_spi_set_crc(struct mmc_host *host, int use_crc)
{
	struct mmc_command cmd = {0};
	int err;

	cmd.opcode = MMC_SPI_CRC_ON_OFF;
	cmd.flags = MMC_RSP_SPI_R1;
	cmd.arg = use_crc;

	err = mmc_wait_for_cmd(host, &cmd, 0);
	if (!err)
		host->use_spi_crc = use_crc;
	return err;
}

int mmc_switch_status_error(struct mmc_host *host, u32 status)
{
	if (mmc_host_is_spi(host)) {
		if (status & R1_SPI_ILLEGAL_COMMAND)
			return -EBADMSG;
	} else {
		if (status & 0xFDFFA000)
			pr_warn("%s: unexpected status %#x after switch\n",
				mmc_hostname(host), status);
		if (status & R1_SWITCH_ERROR)
			return -EBADMSG;
	}
	return 0;
}

/**
 *	__mmc_switch - modify EXT_CSD register
 *	@card: the MMC card associated with the data transfer
 *	@set: cmd set values
 *	@index: EXT_CSD register index
 *	@value: value to program into EXT_CSD register
 *	@timeout_ms: timeout (ms) for operation performed by register write,
 *                   timeout of zero implies maximum possible timeout
 *	@use_busy_signal: use the busy signal as response type
 *	@send_status: send status cmd to poll for busy
 *	@ignore_crc: ignore CRC errors when sending status cmd to poll for busy
 *
 *	Modifies the EXT_CSD register for selected card.
 */
int __mmc_switch(struct mmc_card *card, u8 set, u8 index, u8 value,
		unsigned int timeout_ms, bool use_busy_signal, bool send_status,
		bool ignore_crc)
{
	struct mmc_host *host = card->host;
	int err;
	struct mmc_command cmd = {0};
	unsigned long timeout;
	u32 status = 0;
	bool use_r1b_resp = use_busy_signal, busy = false;
	bool expired = false;

	mmc_retune_hold(host);

	/*
	 * If the cmd timeout and the max_busy_timeout of the host are both
	 * specified, let's validate them. A failure means we need to prevent
	 * the host from doing hw busy detection, which is done by converting
	 * to a R1 response instead of a R1B.
	 */
	if (timeout_ms && host->max_busy_timeout &&
		(timeout_ms > host->max_busy_timeout))
		use_r1b_resp = false;

	cmd.opcode = MMC_SWITCH;
	cmd.arg = (MMC_SWITCH_MODE_WRITE_BYTE << 24) |
		  (index << 16) |
		  (value << 8) |
		  set;
	cmd.flags = MMC_CMD_AC;
	if (use_r1b_resp) {
		cmd.flags |= MMC_RSP_SPI_R1B | MMC_RSP_R1B;
		/*
		 * A busy_timeout of zero means the host can decide to use
		 * whatever value it finds suitable.
		 */
		cmd.busy_timeout = timeout_ms;
	} else {
		cmd.flags |= MMC_RSP_SPI_R1 | MMC_RSP_R1;
	}

	if (index == EXT_CSD_SANITIZE_START)
		cmd.sanitize_busy = true;

	err = mmc_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES);
	if (err)
		goto out;

	/* No need to check card status in case of unblocking command */
	if (!use_busy_signal)
		goto out;

	/*
	 * CRC errors shall only be ignored in cases were CMD13 is used to poll
	 * to detect busy completion.
	 */
	if ((host->caps & MMC_CAP_WAIT_WHILE_BUSY) && use_r1b_resp)
		ignore_crc = false;

	/* We have an unspecified cmd timeout, use the fallback value. */
	if (!timeout_ms)
		timeout_ms = MMC_OPS_TIMEOUT_MS;

	/* Must check status to be sure of no errors. */
	timeout = jiffies + msecs_to_jiffies(timeout_ms);
	do {
		/*
		 * Due to the possibility of being preempted after
		 * sending the status command, check the expiration
		 * time first.
		 */
		expired = time_after(jiffies, timeout);
		if (send_status) {
			err = __mmc_send_status(card, &status, ignore_crc);
			if (err)
				goto out;
		}
		if ((host->caps & MMC_CAP_WAIT_WHILE_BUSY) && use_r1b_resp)
			break;
		if (mmc_host_is_spi(host))
			break;

		/*
		 * We are not allowed to issue a status command and the host
		 * does'nt support MMC_CAP_WAIT_WHILE_BUSY, then we can only
		 * rely on waiting for the stated timeout to be sufficient.
		 */
		if (!send_status) {
			if (use_r1b_resp && host->ops->card_busy) {
				if (!card->host->ops->card_busy(host)) {
					err = 0;
					goto out;
				} else {
					busy = true;
				}
			} else {
				mmc_delay(timeout_ms);
				goto out;
			}
		}

		/* Timeout if the device never leaves the program state. */
		if (expired && (R1_CURRENT_STATE(status) == R1_STATE_PRG || busy)) {
			pr_err("%s: Card stuck in programming state! %s\n",
				mmc_hostname(host), __func__);
			err = -ETIMEDOUT;
			goto out;
		}
	} while ((R1_CURRENT_STATE(status) == R1_STATE_PRG) || busy);

	err = mmc_switch_status_error(host, status);
out:
	mmc_retune_release(host);

	return err;
}

int mmc_switch(struct mmc_card *card, u8 set, u8 index, u8 value,
		unsigned int timeout_ms)
{
	return __mmc_switch(card, set, index, value, timeout_ms, true, true,
				false);
}
EXPORT_SYMBOL_GPL(mmc_switch);

int mmc_send_tuning(struct mmc_host *host, u32 opcode, int *cmd_error)
{
	struct mmc_request mrq = {NULL};
	struct mmc_command cmd = {0};
	struct mmc_data data = {0};
	struct scatterlist sg;
	struct mmc_ios *ios = &host->ios;
	const u8 *tuning_block_pattern;
	int size, err = 0;
	u8 *data_buf;

	if (ios->bus_width == MMC_BUS_WIDTH_8) {
		tuning_block_pattern = tuning_blk_pattern_8bit;
		size = sizeof(tuning_blk_pattern_8bit);
	} else if (ios->bus_width == MMC_BUS_WIDTH_4) {
		tuning_block_pattern = tuning_blk_pattern_4bit;
		size = sizeof(tuning_blk_pattern_4bit);
	} else
		return -EINVAL;

	data_buf = kzalloc(size, GFP_KERNEL);
	if (!data_buf)
		return -ENOMEM;

	mrq.cmd = &cmd;
	mrq.data = &data;

	cmd.opcode = opcode;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = size;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;

	/*
	 * According to the tuning specs, Tuning process
	 * is normally shorter 40 executions of CMD19,
	 * and timeout value should be shorter than 150 ms
	 */
	data.timeout_ns = 150 * NSEC_PER_MSEC;

	data.sg = &sg;
	data.sg_len = 1;
	sg_init_one(&sg, data_buf, size);

	mmc_wait_for_req(host, &mrq);

	if (cmd_error)
		*cmd_error = cmd.error;

	if (cmd.error) {
		err = cmd.error;
		goto out;
	}

	if (data.error) {
		err = data.error;
		goto out;
	}

	if (memcmp(data_buf, tuning_block_pattern, size))
		err = -EIO;

out:
	kfree(data_buf);
	return err;
}
EXPORT_SYMBOL_GPL(mmc_send_tuning);

static int
mmc_send_bus_test(struct mmc_card *card, struct mmc_host *host, u8 opcode,
		  u8 len)
{
	struct mmc_request mrq = {NULL};
	struct mmc_command cmd = {0};
	struct mmc_data data = {0};
	struct scatterlist sg;
	u8 *data_buf;
	u8 *test_buf;
	int i, err;
	static u8 testdata_8bit[8] = { 0x55, 0xaa, 0, 0, 0, 0, 0, 0 };
	static u8 testdata_4bit[4] = { 0x5a, 0, 0, 0 };

	/* dma onto stack is unsafe/nonportable, but callers to this
	 * routine normally provide temporary on-stack buffers ...
	 */
	data_buf = kmalloc(len, GFP_KERNEL);
	if (!data_buf)
		return -ENOMEM;

	if (len == 8)
		test_buf = testdata_8bit;
	else if (len == 4)
		test_buf = testdata_4bit;
	else {
		pr_err("%s: Invalid bus_width %d\n",
		       mmc_hostname(host), len);
		kfree(data_buf);
		return -EINVAL;
	}

	if (opcode == MMC_BUS_TEST_W)
		memcpy(data_buf, test_buf, len);

	mrq.cmd = &cmd;
	mrq.data = &data;
	cmd.opcode = opcode;
	cmd.arg = 0;

	/* NOTE HACK:  the MMC_RSP_SPI_R1 is always correct here, but we
	 * rely on callers to never use this with "native" calls for reading
	 * CSD or CID.  Native versions of those commands use the R2 type,
	 * not R1 plus a data block.
	 */
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = len;
	data.blocks = 1;
	if (opcode == MMC_BUS_TEST_R)
		data.flags = MMC_DATA_READ;
	else
		data.flags = MMC_DATA_WRITE;

	data.sg = &sg;
	data.sg_len = 1;
	mmc_set_data_timeout(&data, card);
	sg_init_one(&sg, data_buf, len);
	mmc_wait_for_req(host, &mrq);
	err = 0;
	if (opcode == MMC_BUS_TEST_R) {
		for (i = 0; i < len / 4; i++)
			if ((test_buf[i] ^ data_buf[i]) != 0xff) {
				err = -EIO;
				break;
			}
	}
	kfree(data_buf);

	if (cmd.error)
		return cmd.error;
	if (data.error)
		return data.error;

	return err;
}

int mmc_bus_test(struct mmc_card *card, u8 bus_width)
{
	int err, width;

	if (bus_width == MMC_BUS_WIDTH_8)
		width = 8;
	else if (bus_width == MMC_BUS_WIDTH_4)
		width = 4;
	else if (bus_width == MMC_BUS_WIDTH_1)
		return 0; /* no need for test */
	else
		return -EINVAL;

	/*
	 * Ignore errors from BUS_TEST_W.  BUS_TEST_R will fail if there
	 * is a problem.  This improves chances that the test will work.
	 */
	mmc_send_bus_test(card, card->host, MMC_BUS_TEST_W, width);
	err = mmc_send_bus_test(card, card->host, MMC_BUS_TEST_R, width);
	return err;
}

int mmc_send_hpi_cmd(struct mmc_card *card, u32 *status)
{
	struct mmc_command cmd = {0};
	unsigned int opcode;
	int err;

	if (!card->ext_csd.hpi) {
		pr_warn("%s: Card didn't support HPI command\n",
			mmc_hostname(card->host));
		return -EINVAL;
	}

	opcode = card->ext_csd.hpi_cmd;
	if (opcode == MMC_STOP_TRANSMISSION)
		cmd.flags = MMC_RSP_R1B | MMC_CMD_AC;
	else if (opcode == MMC_SEND_STATUS)
		cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	cmd.opcode = opcode;
	cmd.arg = card->rca << 16 | 1;

	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err) {
		pr_warn("%s: error %d interrupting operation. "
			"HPI command response %#x\n", mmc_hostname(card->host),
			err, cmd.resp[0]);
		return err;
	}
	if (status)
		*status = cmd.resp[0];

	return 0;
}

int mmc_can_ext_csd(struct mmc_card *card)
{
	return (card && card->csd.mmca_vsn > CSD_SPEC_VER_3);
}

#ifdef CONFIG_MMC_SAMSUNG_SMART
/*
 * Vendor-specific Samsung BACK-DOOR command
 */
static int mmc_movi_vendor_cmd(struct mmc_card *card, unsigned int arg)
{
	struct mmc_command cmd;
	int err;
	u32 status;

	/* CMD62 is vendor CMD, it's not defined in eMMC spec. */
	cmd.opcode = 62;
	cmd.flags = MMC_RSP_R1B | MMC_CMD_AC;
	cmd.arg = arg;
	cmd.error = 0;

	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err)
		return err;

	do {
		err = mmc_send_status(card, &status);
		if (err)
			return err;
		if (card->host->caps & MMC_CAP_WAIT_WHILE_BUSY)
			break;
		if (mmc_host_is_spi(card->host))
			break;
	} while (R1_CURRENT_STATE(status) == R1_STATE_PRG);

	return err;
}

static int mmc_movi_read_cmd(struct mmc_card *card, u8 *buffer)
{
	struct mmc_command wcmd;
	struct mmc_data wdata;
	struct mmc_request brq;
	struct scatterlist sg;

	brq.cmd = &wcmd;
	brq.data = &wdata;
	brq.stop = NULL;
	brq.sbc = NULL;

	wcmd.opcode = MMC_READ_SINGLE_BLOCK;
	wcmd.arg = 0;
	wcmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;
	wcmd.error = 0;
	wcmd.retries = 0;
	wdata.blksz = 512;
	wdata.error = 0;
	wdata.stop = NULL;
	wdata.blocks = 1;
	wdata.flags = MMC_DATA_READ;

	wdata.sg = &sg;
	wdata.sg_len = 1;

	sg_init_one(&sg, buffer, 512);

	mmc_set_data_timeout(&wdata, card);

	mmc_wait_for_req(card->host, &brq);
	if (wcmd.error)
		return wcmd.error;
	if (wdata.error)
		return wdata.error;
	return 0;
}

static int mmc_samsung_smart_read(struct mmc_card *card, u8 *rdblock)
{
	int err, errx;

	/* enter vendor Smart Report mode */
	err = mmc_movi_vendor_cmd(card, 0xEFAC62EC);
	if (err) {
		pr_err("Failed entering Smart Report mode(1, %d)\n", err);
		return err;
	}
	err = mmc_movi_vendor_cmd(card, 0x0000CCEE);
	if (err) {
		pr_err("Failed entering Smart Report mode(2, %d)\n", err);
		return err;
	}

	/* read Smart Report */
	err = mmc_movi_read_cmd(card, rdblock);
	if (err)
		pr_err("Failed reading Smart Report(%d)\n", err);
		/* Do NOT return yet; we must leave Smart Report mode.*/

	/* exit vendor Smart Report mode */
	errx = mmc_movi_vendor_cmd(card, 0xEFAC62EC);
	if (errx)
		pr_err("Failed exiting Smart Report mode(1, %d)\n", errx);
	else {
		errx = mmc_movi_vendor_cmd(card, 0x00DECCEE);
		if (errx)
			pr_err("Failed exiting Smart Report mode(2, %d)\n",
									errx);
	}
	if (err)
		return err;
	return errx;
}

ssize_t mmc_samsung_smart_parse(u32 *report, char *for_sysfs)
{
	unsigned size = PAGE_SIZE;
	unsigned wrote;
	unsigned i;
	u32 val;
	char *str;
	static const struct {
		char *fmt;
		unsigned val_index;
	} to_output[] = {
		{ "super block size              : %u\n", 1 },
		{ "super page size               : %u\n", 2 },
		{ "optimal write size            : %u\n", 3 },
		{ "read reclaim count            : %u\n", 20 },
		{ "optimal trim size             : %u\n", 21 },
		{ "number of banks               : %u\n", 4 },
		{ "initial bad blocks per bank   : %u",	  5 },
		{ ",%u",				  8 },
		{ ",%u",				  11 },
		{ ",%u\n",				  14 },
		{ "runtime bad blocks per bank   : %u",	  6 },
		{ ",%u",				  9 },
		{ ",%u",				  12 },
		{ ",%u\n",				  15 },
		{ "reserved blocks left per bank : %u",	  7 },
		{ ",%u",				  10 },
		{ ",%u",				  13 },
		{ ",%u\n",				  16 },
		{ "all erase counts (min,avg,max): %u",	  18 },
		{ ",%u",				  19 },
		{ ",%u\n",				  17 },
		{ "SLC erase counts (min,avg,max): %u",	  31 },
		{ ",%u",				  32 },
		{ ",%u\n",				  30 },
		{ "MLC erase counts (min,avg,max): %u",	  34 },
		{ ",%u",				  35 },
		{ ",%u\n",				  33 },
	};

	/* A version field just in case things change. */
	wrote = scnprintf(for_sysfs, size,
				"version                       : %u\n", 0);
	size -= wrote;
	for_sysfs += wrote;

	/* The error mode. */
	val = le32_to_cpu(report[0]);
	switch (val) {
	case 0xD2D2D2D2:
		str = "Normal";
		break;
	case 0x5C5C5C5C:
		str = "RuntimeFatalError";
		break;
	case 0xE1E1E1E1:
		str = "MetaBrokenError";
		break;
	case 0x37373737:
		str = "OpenFatalError";
		val = 0; /* Remaining data is unreliable. */
		break;
	default:
		str = "Invalid";
		val = 0; /* Remaining data is unreliable. */
		break;
	}
	wrote = scnprintf(for_sysfs, size,
				"error mode                    : %s\n", str);
	size -= wrote;
	for_sysfs += wrote;
	/* Exit if we can't rely on the remaining data. */
	if (!val)
		return PAGE_SIZE - size;

	for (i = 0; i < ARRAY_SIZE(to_output); i++) {
		wrote = scnprintf(for_sysfs, size, to_output[i].fmt,
				  le32_to_cpu(report[to_output[i].val_index]));
		size -= wrote;
		for_sysfs += wrote;
	}

	return PAGE_SIZE - size;
}

ssize_t mmc_samsung_smart_handle(struct mmc_card *card, char *buf)
{
	int err;
	u32 *buffer;
	ssize_t len;

	buffer = kmalloc(512, GFP_KERNEL);
	if (!buffer) {
		pr_err("Failed to alloc memory for Smart Report\n");
		return 0;
	}

	mmc_claim_host(card->host);
	err = mmc_samsung_smart_read(card, (u8 *)buffer);
	mmc_release_host(card->host);

	if (err)
		len = 0;
	else
		len = mmc_samsung_smart_parse(buffer, buf);

	kfree(buffer);
	return len;
}

/*
 * Toshiba Smart report generation command handler
 */
int mmc_gen_cmd(struct mmc_card *card, void *buf,
	u8 index, u8 arg1, u8 arg2, u8 mode)
{
	struct mmc_request mrq;
	struct mmc_command cmd;
	struct mmc_data data;
	struct scatterlist sg;
	struct tsb_cmd_format *tsb_cmd;
	void *data_buf;
	static u8 smartpwd_cmd[8] = { 0x07, 0x00, 0x00, 0x00, 0x26, 0xE9,
		0x01, 0xEB };

	data_buf = tsb_cmd = kmalloc(512, GFP_KERNEL);
	if (!data_buf)
		return -ENOMEM;

	memset(&mrq, 0, sizeof(mrq));
	memset(&cmd, 0, sizeof(cmd));
	memset(&data, 0, sizeof(data));

	if (!mode)
		memcpy(data_buf, smartpwd_cmd, 8);

	mode &= 1;
	index &= 0x7F;

	/* Prepare MMC Command */
	cmd.opcode = MMC_GEN_CMD;
	cmd.arg = (arg2 << 16) | (arg1 << 8) | (index << 1) | mode;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	/* Prepare MMC Data */
	data.blksz = 512;
	data.blocks = 1;
	data.flags = (!mode) ? MMC_DATA_WRITE : MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;
	data.timeout_ns = 300000000;
	data.timeout_clks = 0;

	/* Prepare MMC Request */
	mrq.cmd = &cmd;
	mrq.data = &data;

	sg_init_one(&sg, data_buf, 512);
	mmc_claim_host(card->host);
	mmc_wait_for_req(card->host, &mrq);
	mmc_release_host(card->host);

	memcpy(buf, data_buf, 512);

	kfree(data_buf);

	if (cmd.error) {
		pr_info("cmd error: %d\n", cmd.error);
		return cmd.error;
	}

	if (data.error) {
		pr_info("data error: %d\n", data.error);
		return data.error;
	}

	return 0;
}

#ifdef CONFIG_AMAZON_METRICS_LOG
int mmc_samsung_report(struct mmc_card *card, u8 *buf)
{
	int err;

	mmc_claim_host(card->host);
	err = mmc_samsung_smart_read(card, buf);
	mmc_release_host(card->host);

	if (err)
		return err;

	return 0;
}

int mmc_toshiba_report(struct mmc_card *card, u8 *buf)
{
	struct tsb_wear_info *hbblk = (struct tsb_wear_info *)buf;
	int err;

	mmc_set_blocklen(card, 512);

	err = mmc_gen_cmd(card, buf, 0, 0, 0, 0);
	if (err) {
		pr_info("tsb eMMC wear leveling error %d\n", err);
		return err;
	}

	mmc_gen_cmd(card, buf, 0, 0, 0, 1);

	pr_info(
		"eMMC Health CMD:0x%x sts:0x%x, mlc(max:%d avg:%d), "
		"slc(max:%d avg:%d)\n", hbblk->sub_cmd_no, hbblk->status,
		__swab32(hbblk->mlc_wr_max), __swab32(hbblk->mlc_wr_avg),
		__swab32(hbblk->slc_wr_max), __swab32(hbblk->slc_wr_avg));

	return 0;
}

int mmc_gen_micron_cmd(struct mmc_card *card, void *buf, u8 cmd_index, u8 arg1)
{
	struct mmc_request mrq;
	struct mmc_command cmd;
	struct mmc_data data;
	struct scatterlist sg;
	void *data_buf;

	data_buf = kmalloc(512, GFP_KERNEL);
	if (!data_buf)
		return -ENOMEM;

	memset(&mrq, 0, sizeof(mrq));
	memset(&cmd, 0, sizeof(cmd));
	memset(&data, 0, sizeof(data));

	/*  Prepare MMC Command */
	cmd.opcode = MMC_GEN_CMD;
	cmd.arg = MICRON_READ | (arg1 << 8) | (cmd_index << 1);
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	/*  Prepare MMC Data */
	data.blksz = 512;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;
	data.timeout_ns = 300000000;
	data.timeout_clks = 0;

	/*  Prepare MMC Request */
	mrq.cmd = &cmd;
	mrq.data = &data;

	sg_init_one(&sg, data_buf, 512);
	mmc_claim_host(card->host);
	mmc_wait_for_req(card->host, &mrq);
	mmc_release_host(card->host);

	memcpy(buf, data_buf, 512);

	kfree(data_buf);

	if (cmd.error) {
		pr_info("cmd error: %d\n", cmd.error);
		return cmd.error;
	}

	if (data.error) {
		pr_info("data error: %d\n", data.error);
		return data.error;
	}

	return 0;
}

int mmc_micron_smart_report(struct mmc_card *card, u8 *buf)
{
	int ret = 1;

	pr_info("reading micron erase count...\n");

	ret = mmc_gen_micron_cmd(card, buf, BLK_ERASE_CNT, 0);
	pr_info("reading complete ret:0x%x\n", ret);
	return ret;
}
#endif /* CONFIG_AMAZON_METRICS_LOG */
#endif /* CONFIG_MMC_SAMSUNG_SMART */

