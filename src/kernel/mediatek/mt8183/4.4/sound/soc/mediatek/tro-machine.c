/*
* Copyright (C) 2015 MediaTek Inc.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*/


/*******************************************************************************
 *
 * Filename:
 * ---------
 *   mt_soc_machine.c
 *
 * Project:
 * --------
 *   Audio soc machine driver
 *
 * Description:
 * ------------
 *   Audio machine driver
 *
 * Author:
 * -------
 * Chipeng Chang
 *
 *------------------------------------------------------------------------------
 *
 *******************************************************************************/


/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/
#include "mtk-auddrv-common.h"
#include "mtk-auddrv-def.h"
#include "mtk-auddrv-afe.h"
#include "mtk-auddrv-ana.h"
#include "mtk-auddrv-clk.h"
#include "mtk-auddrv-kernel.h"
#include "mtk-soc-afe-control.h"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/completion.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <linux/semaphore.h>
#include <linux/jiffies.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <asm/div64.h>
#include <stdarg.h>
#include <linux/module.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/jack.h>
#include <linux/debugfs.h>
#include "mtk-soc-codec-63xx.h"
#include "mtk-soc-speaker-amp.h"
#include "tro-machine.h"

#include "mtk-hw-component.h"
#if defined(CONFIG_SND_SOC_CS43130)
#include "mtk-cs43130-machine-ops.h"
#endif
#if defined(CONFIG_SND_SOC_CS35L35)
#include "mtk-cs35l35-machine-ops.h"
#endif
#ifdef CONFIG_AMAZON_DSP_FRAMEWORK
#include "../../codecs/rt551x.h"
static const struct of_device_id mt_audio_driver_dt_match[];

static int rt551x_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, RT551X_SCLK_S_MCLK,
				params_rate(params) * 256, SND_SOC_CLOCK_IN);

	if (ret)
		return ret;
	return 0;
}

static struct snd_soc_ops rt551x_ops = {
	.hw_params = rt551x_hw_params,
};

static struct snd_soc_dai_link_component rt551x_codecs[] = {
	{
		.dai_name = "rt551x-aif1",
	},
};
#endif

static int board_channel_type;
static struct dentry *mt_sco_audio_debugfs;
#define DEBUG_FS_NAME "mtksocaudio"
#define DEBUG_ANA_FS_NAME "mtksocanaaudio"

#ifdef CONFIG_KPD_VOLUME_KEY_SWAP
enum mt_volume_key_switch {
	VOLKEY_NORMAL = 0,
	VOLKEY_SWAP
};

static const char *const mt_volume_key_switch[] = {
	"VOLKEY_NORMAL", "VOLKEY_SWAP"
};
#endif

#ifdef CONFIG_SND_SOC_MT6358_DL_LR_SWAP
enum dl_lr_swap {
	LR_NORMAL = 0,
	LR_SWAP
};

static const char *const mt_dl_lr_swap[] = {
	"LR_NORMAL", "LR_SWAP"
};
static bool mt_dl_lr_swap_enable;
#endif

static const char *const mt_channel_cap[] = {
	"Stereo", "MonoLeft", "MonoRight"
};

#ifdef CONFIG_KPD_VOLUME_KEY_SWAP
static int mt_volkey_switch_get
		(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	if (get_kpd_swap_vol_key())
		ucontrol->value.integer.value[0] = VOLKEY_SWAP;
	else
		ucontrol->value.integer.value[0] = VOLKEY_NORMAL;
	return 0;
}

static int mt_volkey_switch_set
		(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	if (ucontrol->value.integer.value[0] == VOLKEY_NORMAL)
		set_kpd_swap_vol_key(false);
	else
		set_kpd_swap_vol_key(true);
	return 0;
}
#endif

#ifdef CONFIG_SND_SOC_MT6358_DL_LR_SWAP
static int mt_dl_lr_switch_get
		(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = mt_dl_lr_swap_enable;
	return 0;
}

static int mt_dl_lr_switch_set
		(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	mt_dl_lr_swap_enable = ucontrol->value.integer.value[0];
	Ana_Set_Reg(AFE_UL_DL_CON0, mt_dl_lr_swap_enable << 14, 0x1 << 14);
	return 0;
}
#endif

static int mt_get_board_channel_type(void)
{
	return board_channel_type;
}

static int mt_channel_cap_set(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int mt_channel_cap_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = mt_get_board_channel_type();
	return 0;
}

static int mt_soc_ana_debug_open(struct inode *inode, struct file *file)
{
	pr_debug("mt_soc_ana_debug_open\n");
	return 0;
}

static ssize_t mt_soc_ana_debug_read(struct file *file, char __user *buf,
				     size_t count, loff_t *pos)
{
	const int size = 8192;
	/* char buffer[size]; */
	char *buffer = NULL; /* for reduce kernel stack */
	int n = 0;
	int ret = 0;

	buffer = kmalloc(size, GFP_KERNEL);
	if (!buffer) {
		kfree(buffer);
		return -ENOMEM;
	}

	pr_debug("mt_soc_ana_debug_read count = %zu\n", count);
	AudDrv_Clk_On();
	audckbufEnable(true);

	n = Ana_Debug_Read(buffer, size);

	pr_debug("mt_soc_ana_debug_read len = %d\n", n);

	audckbufEnable(false);
	AudDrv_Clk_Off();

	ret = simple_read_from_buffer(buf, count, pos, buffer, n);
	kfree(buffer);
	return ret;
}


static int mt_soc_debug_open(struct inode *inode, struct file *file)
{
	pr_debug("mt_soc_debug_open\n");
	return 0;
}

static ssize_t mt_soc_debug_read(struct file *file, char __user *buf,
				size_t count, loff_t *pos)
{
	const int size = 12288;
	/* char buffer[size]; */
	char *buffer = NULL; /* for reduce kernel stack */
	int n = 0;
	int ret = 0;

	buffer = kmalloc(size, GFP_KERNEL);
	if (!buffer) {
		kfree(buffer);
		return -ENOMEM;
	}

	AudDrv_Clk_On();

	n = AudDrv_Reg_Dump(buffer, size);
	pr_debug("mt_soc_debug_read len = %d\n", n);

	AudDrv_Clk_Off();

	ret = simple_read_from_buffer(buf, count, pos, buffer, n);
	kfree(buffer);
	return ret;
}

static char const ParSetkeyAfe[] = "Setafereg";
static char const ParSetkeyAna[] = "Setanareg";
static char const PareGetkeyAfe[] = "Getafereg";
static char const PareGetkeyAna[] = "Getanareg";

static ssize_t mt_soc_debug_write(struct file *f, const char __user *buf,
				  size_t count, loff_t *offset)
{
#define MAX_DEBUG_WRITE_INPUT 256
	int ret = 0;
	char InputString[MAX_DEBUG_WRITE_INPUT];
	char *token1 = NULL;
	char *token2 = NULL;
	char *token3 = NULL;
	char *token4 = NULL;
	char *token5 = NULL;
	char *temp = NULL;
	char *str_begin = NULL;

	unsigned long regaddr = 0;
	unsigned long regvalue = 0;
	char delim[] = " ,";

	if (!count) {
		pr_debug("%s(), count is 0, return directly\n", __func__);
		goto exit;
	}

	if (count > MAX_DEBUG_WRITE_INPUT)
		count = MAX_DEBUG_WRITE_INPUT;

	memset_io((void *)InputString, 0, MAX_DEBUG_WRITE_INPUT);

	if (copy_from_user((InputString), buf, count))
		pr_warn("%s(), copy_from_user fail, mt_soc_debug_write count = %zu, temp = %s\n",
			__func__, count, InputString);

	str_begin = kstrndup(InputString, MAX_DEBUG_WRITE_INPUT - 1, GFP_KERNEL);
	if (!str_begin) {
		pr_warn("%s(), kstrdup fail\n", __func__);
		goto exit;
	}
	temp = str_begin;

	pr_debug("copy_from_user mt_soc_debug_write count = %zu, temp = %s, pointer = %p\n",
		count, InputString, InputString);
	token1 = strsep(&temp, delim);
	pr_debug("token1 = %s\n", token1);
	token2 = strsep(&temp, delim);
	pr_debug("token2 = %s\n", token2);
	token3 = strsep(&temp, delim);
	pr_debug("token3 = %s\n", token3);
	token4 = strsep(&temp, delim);
	pr_debug("token4 = %s\n", token4);
	token5 = strsep(&temp, delim);
	pr_debug("token5 = %s\n", token5);

	AudDrv_Clk_On();
	if (strcmp(token1, ParSetkeyAfe) == 0) {
		pr_debug("strcmp(token1, ParSetkeyAfe)\n");
		if ((token3 != NULL) && (token5 != NULL)) {
			ret = kstrtoul(token3, 16, &regaddr);
			ret = kstrtoul(token5, 16, &regvalue);
			pr_debug("%s, regaddr = 0x%x, regvalue = 0x%x\n", ParSetkeyAfe, (unsigned int)regaddr,
					(unsigned int)regvalue);
			Afe_Set_Reg(regaddr, regvalue, 0xffffffff);
			regvalue = Afe_Get_Reg(regaddr);
			pr_debug("%s, regaddr = 0x%x, regvalue = 0x%x\n", ParSetkeyAfe, (unsigned int)regaddr,
					(unsigned int)regvalue);
		} else {
			pr_debug("token3 or token5 is NULL!\n");
		}
	}

	if (strcmp(token1, ParSetkeyAna) == 0) {
		pr_debug("strcmp(token1, ParSetkeyAna)\n");
		if ((token3 != NULL) && (token5 != NULL)) {
			ret = kstrtoul(token3, 16, &regaddr);
			ret = kstrtoul(token5, 16, &regvalue);
			pr_debug("%s, regaddr = 0x%x, regvalue = 0x%x\n", ParSetkeyAna, (unsigned int)regaddr,
					(unsigned int)regvalue);
			audckbufEnable(true);
			Ana_Set_Reg(regaddr, regvalue, 0xffffffff);
			regvalue = Ana_Get_Reg(regaddr);
			audckbufEnable(false);
			pr_debug("%s, regaddr = 0x%x, regvalue = 0x%x\n", ParSetkeyAna, (unsigned int)regaddr,
					(unsigned int)regvalue);
		} else {
			pr_debug("token3 or token5 is NULL!\n");
		}
	}

	if (strcmp(token1, PareGetkeyAfe) == 0) {
		pr_debug("strcmp(token1, PareGetkeyAfe)\n");
		if (token3 != NULL) {
			ret = kstrtoul(token3, 16, &regaddr);
			regvalue = Afe_Get_Reg(regaddr);
			pr_debug("%s, regaddr = 0x%x, regvalue = 0x%x\n", PareGetkeyAfe, (unsigned int)regaddr,
					(unsigned int)regvalue);
		} else {
			pr_debug("token3 is NULL!\n");
		}
	}

	if (strcmp(token1, PareGetkeyAna) == 0) {
		pr_debug("strcmp(token1, PareGetkeyAna)\n");
		if (token3 != NULL) {
			ret = kstrtoul(token3, 16, &regaddr);
			regvalue = Ana_Get_Reg(regaddr);
			pr_debug("%s, regaddr = 0x%x, regvalue = 0x%x\n", PareGetkeyAna, (unsigned int)regaddr,
					(unsigned int)regvalue);
		} else {
			pr_debug("token3 is NULL!\n");
		}
	}
	AudDrv_Clk_Off();

	kfree(str_begin);
exit:
	return count;
}

static const struct file_operations mtaudio_debug_ops = {
	.open = mt_soc_debug_open,
	.read = mt_soc_debug_read,
	.write = mt_soc_debug_write,
};


static const struct file_operations mtaudio_ana_debug_ops = {
	.open = mt_soc_ana_debug_open,
	.read = mt_soc_ana_debug_read,
};

/* snd_soc_ops */
static int mt_machine_trigger(struct snd_pcm_substream *substream,
				     int cmd)
{
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		EnableAfe(true);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		EnableAfe(false);
		return 0;
	}
	return -EINVAL;
}

static struct snd_soc_ops mt_machine_audio_ops = {
	.trigger = mt_machine_trigger,
};

/* Digital audio interface glue - connects codec <---> CPU */
static struct snd_soc_dai_link mt_soc_dai_common[] = {
	/* FrontEnd DAI Links */
	{
		.name = "MultiMedia1",
		.stream_name = MT_SOC_DL1_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_DL1DAI_NAME,
		.platform_name  = MT_SOC_DL1_PCM,
		.codec_dai_name = MT_SOC_CODEC_TXDAI_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
	{
		.name = "MultiMedia2",
		.stream_name = MT_SOC_UL1_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_UL1DAI_NAME,
		.platform_name  = MT_SOC_UL1_PCM,
		.codec_dai_name = MT_SOC_CODEC_RXDAI_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
	{
		.name = "Voice_MD1",
		.stream_name = MT_SOC_VOICE_MD1_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_VOICE_MD1_NAME,
		.platform_name  = MT_SOC_VOICE_MD1,
		.codec_dai_name = MT_SOC_CODEC_VOICE_MD1DAI_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
#ifdef CONFIG_MTK_HDMI_TDM
	{
		.name = "HDMI_OUT",
		.stream_name = MT_SOC_HDMI_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_HDMI_NAME,
		.platform_name  = MT_SOC_HDMI_PCM,
		.codec_dai_name = MT_SOC_CODEC_HDMI_DUMMY_DAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
#endif
	{
		.name = "ULDLOOPBACK",
		.stream_name = MT_SOC_ULDLLOOPBACK_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_ULDLLOOPBACK_NAME,
		.platform_name  = MT_SOC_ULDLLOOPBACK_PCM,
		.codec_dai_name = MT_SOC_CODEC_ULDLLOOPBACK_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
	{
		.name = "I2S0OUTPUT",
		.stream_name = MT_SOC_I2S0_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_I2S0_NAME,
		.platform_name  = MT_SOC_I2S0_PCM,
		.codec_dai_name = MT_SOC_CODEC_I2S0_DUMMY_DAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "MRGRX",
		.stream_name = MT_SOC_MRGRX_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_MRGRX_NAME,
		.platform_name  = MT_SOC_MRGRX_PCM,
		.codec_dai_name = MT_SOC_CODEC_MRGRX_DAI_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
	{
		.name = "MRGRXCAPTURE",
		.stream_name = MT_SOC_MRGRX_CAPTURE_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_MRGRX_NAME,
		.platform_name  = MT_SOC_MRGRX_AWB_PCM,
		.codec_dai_name = MT_SOC_CODEC_MRGRX_DUMMY_DAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "I2S0DL1OUTPUT",
		.stream_name = MT_SOC_I2SDL1_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_I2S0DL1_NAME,
		.platform_name  = MT_SOC_I2S0DL1_PCM,
		.codec_dai_name = MT_SOC_CODEC_I2S0TXDAI_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
	{
		.name = "DEEP_BUFFER_DL_OUTPUT",
		.stream_name = MT_SOC_DEEP_BUFFER_DL_STREAM_NAME,
		.cpu_dai_name = "snd-soc-dummy-dai",
		.platform_name = MT_SOC_DEEP_BUFFER_DL_PCM,
		.codec_dai_name = MT_SOC_CODEC_DEEPBUFFER_TX_DAI_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
	{
		.name = "DL1AWBCAPTURE",
		.stream_name = MT_SOC_DL1_AWB_RECORD_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_DL1AWB_NAME,
		.platform_name  = MT_SOC_DL1_AWB_PCM,
		.codec_dai_name = MT_SOC_CODEC_DL1AWBDAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "Voice_MD1_BT",
		.stream_name = MT_SOC_VOICE_MD1_BT_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_VOICE_MD1_BT_NAME,
		.platform_name  = MT_SOC_VOICE_MD1_BT,
		.codec_dai_name = MT_SOC_CODEC_VOICE_MD1_BTDAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "VOIP_CALL_BT_PLAYBACK",
		.stream_name = MT_SOC_VOIP_BT_OUT_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_VOIP_CALL_BT_OUT_NAME,
		.platform_name  = MT_SOC_VOIP_BT_OUT,
		.codec_dai_name = MT_SOC_CODEC_VOIPCALLBTOUTDAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "VOIP_CALL_BT_CAPTURE",
		.stream_name = MT_SOC_VOIP_BT_IN_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_VOIP_CALL_BT_IN_NAME,
		.platform_name  = MT_SOC_VOIP_BT_IN,
		.codec_dai_name = MT_SOC_CODEC_VOIPCALLBTINDAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "TDM_Debug_CAPTURE",
		.stream_name = MT_SOC_TDM_CAPTURE_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_TDMRX_NAME,
		.platform_name  = MT_SOC_TDMRX_PCM,
		.codec_dai_name = MT_SOC_CODEC_TDMRX_DAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "FM_MRG_TX",
		.stream_name = MT_SOC_FM_MRGTX_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_FM_MRGTX_NAME,
		.platform_name  = MT_SOC_FM_MRGTX_PCM,
		.codec_dai_name = MT_SOC_CODEC_FMMRGTXDAI_DUMMY_DAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
#ifdef CONFIG_AMAZON_DSP_FRAMEWORK
	{
		.name = "MultiMedia3",
		.stream_name = MT_SOC_UL1DATA2_STREAM_NAME,
		.cpu_dai_name = MT_SOC_UL2DAI_NAME,
		.platform_name = MT_SOC_UL2_PCM,
		.codec_dai_name = NULL,
		.codec_name = NULL,
		.codecs = rt551x_codecs,
		.num_codecs = 1,
		.ops = &rt551x_ops,
	},
#else
	{
		.name = "MultiMedia3",
		.stream_name = MT_SOC_UL1DATA2_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_UL2DAI_NAME,
		.platform_name  = MT_SOC_UL2_PCM,
		.codec_dai_name = MT_SOC_CODEC_RXDAI2_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
#endif
	{
		.name = "I2S0_AWB_CAPTURE",
		.stream_name = MT_SOC_I2S0AWB_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_I2S0AWBDAI_NAME,
		.platform_name  = MT_SOC_I2S0_AWB_PCM,
		.codec_dai_name = MT_SOC_CODEC_I2S0AWB_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "Voice_MD2",
		.stream_name = MT_SOC_VOICE_MD2_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_VOICE_MD2_NAME,
		.platform_name  = MT_SOC_VOICE_MD2,
		.codec_dai_name = MT_SOC_CODEC_VOICE_MD2DAI_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
	{
		.name = "PLATOFRM_CONTROL",
		.stream_name = MT_SOC_ROUTING_STREAM_NAME,
		.cpu_dai_name   = "snd-soc-dummy-dai",
		.platform_name  = MT_SOC_ROUTING_PCM,
		.codec_dai_name = MT_SOC_CODEC_DUMMY_DAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "Voice_MD2_BT",
		.stream_name = MT_SOC_VOICE_MD2_BT_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_VOICE_MD2_BT_NAME,
		.platform_name  = MT_SOC_VOICE_MD2_BT,
		.codec_dai_name = MT_SOC_CODEC_VOICE_MD2_BTDAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "HP_IMPEDANCE",
		.stream_name = MT_SOC_HP_IMPEDANCE_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_HP_IMPEDANCE_NAME,
		.platform_name  = MT_SOC_HP_IMPEDANCE_PCM,
		.codec_dai_name = MT_SOC_CODEC_HP_IMPEDANCE_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
	{
		.name = "FM_I2S_RX_Playback",
		.stream_name = MT_SOC_FM_I2S_PLAYBACK_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_FM_I2S_NAME,
		.platform_name  = MT_SOC_FM_I2S_PCM,
		.codec_dai_name = MT_SOC_CODEC_FM_I2S_DAI_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
	{
		.name = "FM_I2S_RX_Capture",
		.stream_name = MT_SOC_FM_I2S_CAPTURE_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_FM_I2S_NAME,
		.platform_name  = MT_SOC_FM_I2S_AWB_PCM,
		.codec_dai_name = MT_SOC_CODEC_FM_I2S_DUMMY_DAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
	{
		.name = "MultiMedia_DL2",
		.stream_name = MT_SOC_DL2_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_DL2DAI_NAME,
		.platform_name  = MT_SOC_DL2_PCM,
		.codec_dai_name = MT_SOC_CODEC_TXDAI2_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
	{
		.name = "MultiMedia_DL3",
		.stream_name = MT_SOC_DL3_STREAM_NAME,
		.cpu_dai_name   = "snd-soc-dummy-dai",
		.platform_name  = "snd-soc-dummy",
		.codec_dai_name = MT_SOC_CODEC_OFFLOAD_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
#ifdef CONFIG_SND_SOC_MTK_BTCVSD
	{
		.name = "BTCVSD_RX",
		.stream_name = MT_SOC_BTCVSD_CAPTURE_STREAM_NAME,
		.cpu_dai_name   = "snd-soc-dummy-dai",
		.platform_name  = MT_SOC_BTCVSD_RX_PCM,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		},
	{
		.name = "BTCVSD_TX",
		.stream_name = MT_SOC_BTCVSD_PLAYBACK_STREAM_NAME,
		.cpu_dai_name   = "snd-soc-dummy-dai",
		.platform_name  = MT_SOC_BTCVSD_TX_PCM,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
#endif
#ifdef _NON_COMMON_FEATURE_READY
	{
		.name = "MOD_DAI_CAPTURE",
		.stream_name = MT_SOC_MODDAI_STREAM_NAME,
		.cpu_dai_name	= MT_SOC_MOD_DAI_NAME,
		.platform_name	= MT_SOC_MOD_DAI_PCM,
		.codec_dai_name = MT_SOC_CODEC_MOD_DAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
#endif
#ifdef CONFIG_MTK_AUDIO_TUNNELING_SUPPORT
	{
		.name = "OFFLOAD",
		.stream_name = MT_SOC_OFFLOAD_STREAM_NAME,
		.cpu_dai_name = MT_SOC_OFFLOAD_PLAYBACK_DAI_NAME,
		.platform_name = MT_SOC_PLAYBACK_OFFLOAD,
		.codec_dai_name = MT_SOC_CODEC_OFFLOAD_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
#endif
#ifdef _NON_COMMON_FEATURE_READY
	{
		.name = "PCM_ANC",
		.stream_name = MT_SOC_ANC_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_ANC_NAME,
		.platform_name  = MT_SOC_ANC_PCM,
		.codec_dai_name = MT_SOC_CODEC_ANC_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
#endif
	{
		.name = "ANC_RECORD",
		.stream_name = MT_SOC_ANC_RECORD_STREAM_NAME,
		.cpu_dai_name	= MT_SOC_ANC_RECORD_DAI_NAME,
		.platform_name	= MT_SOC_I2S2_ADC2_PCM,
		.codec_dai_name = MT_SOC_CODEC_DUMMY_DAI_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
		.ops = &mt_machine_audio_ops,
	},
#ifdef _NON_COMMON_FEATURE_READY
	{
		.name = "Voice_Ultrasound",
		.stream_name = MT_SOC_VOICE_ULTRA_STREAM_NAME,
		.cpu_dai_name	= "snd-soc-dummy-dai",
		.platform_name	= MT_SOC_VOICE_ULTRA,
		.codec_dai_name = MT_SOC_CODEC_VOICE_ULTRADAI_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
#endif
	{
		.name = "Voice_USB",
		.stream_name = MT_SOC_VOICE_USB_STREAM_NAME,
		.cpu_dai_name	= "snd-soc-dummy-dai",
		.platform_name	= MT_SOC_VOICE_USB,
		.codec_dai_name = MT_SOC_CODEC_VOICE_USBDAI_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
	{
		.name = "Voice_USB_ECHOREF",
		.stream_name = MT_SOC_VOICE_USB_ECHOREF_STREAM_NAME,
		.cpu_dai_name	= "snd-soc-dummy-dai",
		.platform_name	= MT_SOC_VOICE_USB_ECHOREF,
		.codec_dai_name = MT_SOC_CODEC_VOICE_USB_ECHOREF_DAI_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
		.playback_only = true,
	},
#ifdef CONFIG_MTK_AUDIO_SCP_SPKPROTECT_SUPPORT
	{
		.name = "DL1SCPSPKOUTPUT",
		.stream_name = MT_SOC_DL1SCPSPK_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_DL1SCPSPK_NAME,
		.platform_name  = MT_SOC_DL1SCPSPK_PCM,
		.codec_dai_name = MT_SOC_CODEC_SPKSCPTXDAI_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
	{
		.name = "VOICE_SCP",
		.stream_name = MT_SOC_SCPVOICE_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_SCPVOICE_NAME,
		.platform_name  = MT_SOC_SCP_VOICE_PCM,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
#endif
#ifdef CONFIG_MTK_VOW_BARGE_IN_SUPPORT
	{
		.name = "VOW_BARGE_IN",
		.stream_name = MT_SOC_VOW_BARGE_IN_STREAM_NAME,
		.cpu_dai_name = MT_SOC_VOW_BARGE_IN_NAME,
		.platform_name = MT_SOC_VOW_BARGE_IN_PCM,
		.codec_dai_name = MT_SOC_CODEC_VOW_BARGE_IN_NAME,
		.codec_name = MT_SOC_CODEC_DUMMY_NAME,
	},
#endif
#ifdef CONFIG_AMAZON_DSP_FRAMEWORK
	{
		.name = "RT551X DSP",
		.stream_name = "DSP_Capture",
		.cpu_dai_name = "spi32765.0",
		.platform_name = "spi32765.0",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
#endif
	{
		.name = "HW_MERGE",
		.stream_name = MT_SOC_HW_MERGR_STREAM_NAME,
		.cpu_dai_name	= MT_SOC_HW_MERGR_DAI_NAME,
		.platform_name	= MT_SOC_HW_MERGE_PCM,
		.codec_dai_name = MT_SOC_CODEC_RXDAI2_NAME,
		.codec_name = MT_SOC_CODEC_NAME,
	},
};

static struct snd_soc_dai_link mt_soc_exthp_dai[] = {
	{
		.name = "ext_Headphone_Multimedia",
		.stream_name = MT_SOC_HEADPHONE_STREAM_NAME,
		.cpu_dai_name   = "snd-soc-dummy-dai",
		.platform_name  = "snd-soc-dummy",
#ifdef CONFIG_SND_SOC_CS43130
		.codec_dai_name = "cs43130-hifi",
		.codec_name = "cs43130.2-0030",
		.ignore_suspend = 1,
		.ignore_pmdown_time = true,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_CBS_CFS |
			   SND_SOC_DAIFMT_NB_NF,
		.ops = &cs43130_ops,
#else
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
#endif
	},
};

static struct snd_soc_dai_link mt_soc_extspk_dai[] = {
	{
		.name = "ext_Speaker_Multimedia",
		.stream_name = MT_SOC_SPEAKER_STREAM_NAME,
		.cpu_dai_name   = "snd-soc-dummy-dai",
		.platform_name  = "snd-soc-dummy",
#ifdef CONFIG_SND_SOC_MAX98926
		.codec_dai_name = "max98926-aif1",
		.codec_name = "MAX98926_MT",
#elif defined(CONFIG_SND_SOC_CS35L35)
		.codec_dai_name = "cs35l35-pcm",
		.codec_name = "cs35l35.2-0040",
		.ignore_suspend = 1,
		.ignore_pmdown_time = true,
		.dai_fmt = SND_SOC_DAIFMT_I2S |
			   SND_SOC_DAIFMT_CBS_CFS |
			   SND_SOC_DAIFMT_NB_NF,
		.ops = &cs35l35_ops,
#else
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
#endif
	},
	{
		.name = "I2S1_AWB_CAPTURE",
		.stream_name = MT_SOC_I2S2ADC2_STREAM_NAME,
		.cpu_dai_name   = MT_SOC_I2S2ADC2DAI_NAME,
		.platform_name  = MT_SOC_I2S2_ADC2_PCM,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ops = &mt_machine_audio_ops,
	},
};

static struct snd_soc_dai_link mt_soc_dai_component[
	ARRAY_SIZE(mt_soc_dai_common) +
	ARRAY_SIZE(mt_soc_exthp_dai) +
	ARRAY_SIZE(mt_soc_extspk_dai)];

#ifdef CONFIG_KPD_VOLUME_KEY_SWAP
static const struct soc_enum mt_key_switch_control_enum[] = {
	SOC_ENUM_SINGLE_EXT(
		ARRAY_SIZE(mt_volume_key_switch), mt_volume_key_switch),
};
#endif

#ifdef CONFIG_SND_SOC_MT6358_DL_LR_SWAP
static const struct soc_enum mt_dl_lr_swap_control_enum[] = {
	SOC_ENUM_SINGLE_EXT(
		ARRAY_SIZE(mt_dl_lr_swap), mt_dl_lr_swap),
};
#endif

static const struct soc_enum mt_channel_control_enum[] = {
	SOC_ENUM_SINGLE_EXT(
		ARRAY_SIZE(mt_channel_cap), mt_channel_cap),
};

static const struct snd_kcontrol_new mt_machine_controls[] = {
#ifdef CONFIG_KPD_VOLUME_KEY_SWAP
	SOC_ENUM_EXT("VOLKEY_SWITCH", mt_key_switch_control_enum[0],
			mt_volkey_switch_get,
			mt_volkey_switch_set),
#endif
#ifdef CONFIG_SND_SOC_MT6358_DL_LR_SWAP
	SOC_ENUM_EXT("DL1_LR_SWITCH", mt_dl_lr_swap_control_enum[0],
			mt_dl_lr_switch_get,
			mt_dl_lr_switch_set),
#endif
	SOC_ENUM_EXT("Board Channel Config", mt_channel_control_enum[0],
			mt_channel_cap_get,
			mt_channel_cap_set),
};

static struct snd_soc_card mt_snd_soc_card_mt = {
	.name       = "mt-snd-card",
	.dai_link   = mt_soc_dai_common,
	.num_links  = ARRAY_SIZE(mt_soc_dai_common),
	.controls   = mt_machine_controls,
	.num_controls = ARRAY_SIZE(mt_machine_controls),
};

static void get_ext_dai_codec_name(void)
{
	get_extspk_dai_codec_name(mt_soc_extspk_dai);
	get_exthp_dai_codec_name(mt_soc_exthp_dai);
}

static int mt_soc_snd_init(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mt_snd_soc_card_mt;
	int ret;
	int daiLinkNum = 0;
#ifdef CONFIG_AMAZON_DSP_FRAMEWORK
	struct device_node *node = NULL;
#endif

	ret = mtk_spk_update_dai_link(mt_soc_extspk_dai, pdev);
	if (ret) {
		dev_err(&pdev->dev, "%s(), mtk_spk_update_dai_link error\n",
			__func__);
		return -EINVAL;
	}

#ifdef CONFIG_AMAZON_DSP_FRAMEWORK
	node = of_find_matching_node(node, mt_audio_driver_dt_match);
	if (node)
		rt551x_codecs[0].of_node = of_parse_phandle(node, "sound-dai", 0);
#endif
	get_ext_dai_codec_name();
	ret = of_property_read_u32(pdev->dev.of_node,
		BOARD_CHANNEL_TYPE_PROPERTY,
		&board_channel_type);
	if (ret) {
		pr_warn("%s read property %s fail in node %s\n", __func__,
			BOARD_CHANNEL_TYPE_PROPERTY,
				pdev->dev.of_node->full_name);
	}
	pr_debug("mt_soc_snd_init dai_link = %p\n", mt_snd_soc_card_mt.dai_link);

	/* DEAL WITH DAI LINK */
	memcpy(mt_soc_dai_component, mt_soc_dai_common, sizeof(mt_soc_dai_common));
	daiLinkNum += ARRAY_SIZE(mt_soc_dai_common);
	memcpy(mt_soc_dai_component + daiLinkNum,
	mt_soc_exthp_dai, sizeof(mt_soc_exthp_dai));
	daiLinkNum += ARRAY_SIZE(mt_soc_exthp_dai);

	memcpy(mt_soc_dai_component + daiLinkNum,
	mt_soc_extspk_dai, sizeof(mt_soc_extspk_dai));
	daiLinkNum += ARRAY_SIZE(mt_soc_extspk_dai);

	mt_snd_soc_card_mt.dai_link = mt_soc_dai_component;
	mt_snd_soc_card_mt.num_links = daiLinkNum;

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret)
		dev_err(&pdev->dev, "%s snd_soc_register_card fail %d\n",
			__func__, ret);

	/* create debug file */
	mt_sco_audio_debugfs = debugfs_create_file(DEBUG_FS_NAME,
	   S_IFREG | S_IRUGO, NULL, (void *) DEBUG_FS_NAME, &mtaudio_debug_ops);

	/* create analog debug file */
	mt_sco_audio_debugfs = debugfs_create_file(DEBUG_ANA_FS_NAME,
	   S_IFREG | S_IRUGO, NULL, (void *) DEBUG_ANA_FS_NAME, &mtaudio_ana_debug_ops);

	return ret;
}

static const struct of_device_id mt_audio_driver_dt_match[] = {
	{ .compatible = "mediatek,audio", },
	{ }
};

MODULE_DEVICE_TABLE(of, mt_audio_driver_dt_match);

static struct platform_driver mt_audio_driver = {
	.driver = {
		   .name = "mtk-audio",
		   .of_match_table = mt_audio_driver_dt_match,
	},
	.probe = mt_soc_snd_init,
};

module_platform_driver(mt_audio_driver);

/* Module information */
MODULE_AUTHOR("ChiPeng <chipeng.chang@mediatek.com>");
MODULE_DESCRIPTION("ALSA SoC driver ");

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mt-snd-card");
