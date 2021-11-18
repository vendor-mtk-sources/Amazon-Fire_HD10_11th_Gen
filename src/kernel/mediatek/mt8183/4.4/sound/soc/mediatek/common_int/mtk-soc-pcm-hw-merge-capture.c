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
 *   mtk-soc-pcm-hw-merge-capture.c
 *
 * Project:
 * --------
 *   Audio Driver Kernel Function
 *
 * Description:
 * ------------
 *   implementation 4ch HW merge, 2ch mic in and 2ch echo reference
 *
 * Author:
 * -------
 *
 *
 *------------------------------------------------------------------------------
 *
 *
 *****************************************************************************/


/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

#include <linux/dma-mapping.h>
#include <sound/pcm_params.h>

#include "mtk-auddrv-common.h"
#include "mtk-soc-pcm-common.h"
#include "mtk-auddrv-def.h"
#include "mtk-auddrv-afe.h"
#include "mtk-auddrv-ana.h"
#include "mtk-auddrv-clk.h"
#include "mtk-auddrv-kernel.h"
#include "mtk-soc-afe-control.h"
#include "mtk-soc-pcm-platform.h"

/* information about */
struct afe_mem_control_t  *Hardware_Merge_Control_context;
static struct snd_dma_buffer *Hardware_Merge_Capture_dma_buf;
static unsigned int mPlaybackDramState;
static bool mPrepareDone;

/*
 *    function implementation
 */
static int mtk_Hardware_merge_probe(struct platform_device *pdev);
static int mtk_hardware_merge_pcm_close(struct snd_pcm_substream *substream);
static int mtk_hardware_merge_data_probe(struct snd_soc_platform *platform);

static struct snd_pcm_hardware mtk_hardware_merge_hardware = {
	.info = (SNDRV_PCM_INFO_MMAP |
	SNDRV_PCM_INFO_INTERLEAVED |
	SNDRV_PCM_INFO_RESUME |
	SNDRV_PCM_INFO_MMAP_VALID),
	.formats =      SND_SOC_ADV_MT_FMTS,
	.rates =        SOC_HIGH_USE_RATE,
	.rate_min =     SOC_HIGH_USE_RATE_MIN,
	.rate_max =     SOC_HIGH_USE_RATE_MAX,
	.channels_min =     SOC_NORMAL_USE_CHANNELS_MIN,
	.channels_max =     SOC_NORMAL_USE_CHANNELS_MAX,
	.buffer_bytes_max = UL1_MAX_BUFFER_SIZE,
	.period_bytes_max = UL1_MAX_BUFFER_SIZE,
	.periods_min =      UL1_MIN_PERIOD_SIZE,
	.periods_max =      UL1_MAX_PERIOD_SIZE,
	.fifo_size =        0,
};

static int mtk_hardware_merge_pcm_prepare(struct snd_pcm_substream *substream)
{
	pr_warn("%s, format = %d, rate = %d\n", __func__,
			substream->runtime->format, substream->runtime->rate);

	if (mPrepareDone == false) {
		SetMemifSubStream(Soc_Aud_Digital_Block_MEM_VUL_DATA2,
			substream);

		if (substream->runtime->format == SNDRV_PCM_FORMAT_S32_LE ||
			substream->runtime->format == SNDRV_PCM_FORMAT_U32_LE) {
			SetConnectionFormat(OUTPUT_DATA_FORMAT_24BIT,
				Soc_Aud_AFE_IO_Block_MEM_VUL);
			SetMemIfFetchFormatPerSample(
				Soc_Aud_Digital_Block_MEM_VUL_DATA2,
				AFE_WLEN_32_BIT_ALIGN_8BIT_0_24BIT_DATA);
			SetConnectionFormat(OUTPUT_DATA_FORMAT_24BIT,
				Soc_Aud_AFE_IO_Block_MEM_VUL_DATA2);
		} else {
			SetConnectionFormat(OUTPUT_DATA_FORMAT_16BIT,
				Soc_Aud_AFE_IO_Block_MEM_VUL);
			SetMemIfFetchFormatPerSample(
				Soc_Aud_Digital_Block_MEM_VUL_DATA2,
				AFE_WLEN_16_BIT);
			SetConnectionFormat(OUTPUT_DATA_FORMAT_16BIT,
				Soc_Aud_AFE_IO_Block_MEM_VUL_DATA2);
		}

		if (GetMemoryPathEnable(Soc_Aud_Digital_Block_ADDA_UL)
					== false) {
			SetMemoryPathEnable(
				Soc_Aud_Digital_Block_ADDA_UL, true);
			set_adc_in(substream->runtime->rate);
			set_adc_enable(true);
		} else {
			SetMemoryPathEnable(
				Soc_Aud_Digital_Block_ADDA_UL, true);
		}

		/* 2-mic + 2ch echo ref setting*/
		SetIntfConnection(Soc_Aud_InterCon_Connection,
			Soc_Aud_AFE_IO_Block_ADDA_UL,
			Soc_Aud_AFE_IO_Block_MEM_VUL);
		SetIntfConnection(Soc_Aud_InterCon_Connection,
			Soc_Aud_AFE_IO_Block_MEM_DL1,
			Soc_Aud_AFE_IO_Block_MEM_VUL_DATA2);
		SetIntfConnection(Soc_Aud_InterCon_Connection,
			Soc_Aud_AFE_IO_Block_MEM_DL2,
			Soc_Aud_AFE_IO_Block_MEM_VUL_DATA2);

		Afe_Set_Reg(AFE_MEMIF_PBUF_SIZE, 0x1 << 17, 0x1 << 17);

		mPrepareDone = true;
	}

	return 0;
}

static int mtk_hardware_merge_alsa_stop(struct snd_pcm_substream *substream)
{
	pr_warn("mtk_hardware_merge_alsa_stop\n");

	irq_remove_user(substream, irq_request_number(
				Soc_Aud_Digital_Block_MEM_VUL_DATA2));

	SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_VUL_DATA2, false);

	ClearMemBlock(Soc_Aud_Digital_Block_MEM_VUL_DATA2);
	return 0;
}

static snd_pcm_uframes_t mtk_hardware_merge_pcm_pointer(
				struct snd_pcm_substream *substream)
{
	return get_mem_frame_index(substream,
		Hardware_Merge_Control_context,
		Soc_Aud_Digital_Block_MEM_VUL_DATA2);
}

static int mtk_hardware_merge_pcm_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *hw_params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret = 0;

	substream->runtime->dma_bytes = params_buffer_bytes(hw_params);
	if (AllocateAudioSram(&substream->runtime->dma_addr,
			      &substream->runtime->dma_area,
			      substream->runtime->dma_bytes, substream,
			      params_format(hw_params), false) == 0) {
		mPlaybackDramState = false;
		/* pr_warn("mtk_pcm_hw_params dma_bytes = %d\n", */
		/* substream->runtime->dma_bytes); */
	} else {
		substream->runtime->dma_area =
				Hardware_Merge_Capture_dma_buf->area;
		substream->runtime->dma_addr =
				Hardware_Merge_Capture_dma_buf->addr;
		mPlaybackDramState = true;
		AudDrv_Emi_Clk_On();
	}
	pr_warn("mtk_hardware_merge_pcm_hw_params dma_bytes = %zu dma_area = %p dma_addr = 0x%lx\n",
	       runtime->dma_bytes, runtime->dma_area, (long)runtime->dma_addr);

	set_mem_block(substream, hw_params, Hardware_Merge_Control_context,
				Soc_Aud_Digital_Block_MEM_VUL_DATA2);
	return ret;
}

static int mtk_hardware_merge_capture_pcm_hw_free(
				struct snd_pcm_substream *substream)
{
	pr_warn("mtk_hardware_merge_capture_pcm_hw_free\n");
	if (mPlaybackDramState == true) {
		AudDrv_Emi_Clk_Off();
		mPlaybackDramState = false;
	} else
		freeAudioSram((void *)substream);
	return 0;
}

static int mtk_hardware_merge_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret = 0;

	mPlaybackDramState = false;

	pr_warn("mtk_hardware_merge_pcm_open\n");
	Hardware_Merge_Control_context = Get_Mem_ControlT(
				Soc_Aud_Digital_Block_MEM_VUL_DATA2);
	runtime->hw = mtk_hardware_merge_hardware;
	memcpy((void *)(&(runtime->hw)), (void *)&mtk_hardware_merge_hardware,
				sizeof(struct snd_pcm_hardware));

	ret = snd_pcm_hw_constraint_integer(
				runtime, SNDRV_PCM_HW_PARAM_PERIODS);

	if (ret < 0)
		pr_err("snd_pcm_hw_constraint_integer failed\n");

	runtime->hw.info |= SNDRV_PCM_INFO_INTERLEAVED;
	runtime->hw.info |= SNDRV_PCM_INFO_NONINTERLEAVED;
	runtime->hw.info |= SNDRV_PCM_INFO_MMAP_VALID;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		pr_debug("SNDRV_PCM_STREAM_CAPTURE\n");

	if (ret < 0) {
		pr_err("mtk_hardware_merge_pcm_close\n");
		mtk_hardware_merge_pcm_close(substream);
		return ret;
	}
		AudDrv_Clk_On();
	pr_debug("mtk_hardware_merge_pcm_open return\n");
	return 0;
}

static int mtk_hardware_merge_pcm_close(struct snd_pcm_substream *substream)
{
	pr_warn("%s\n", __func__);
	if (mPrepareDone == true) {

		/* 2-mic + 2ch echo ref setting*/
		SetIntfConnection(Soc_Aud_InterCon_DisConnect,
			Soc_Aud_AFE_IO_Block_ADDA_UL,
			Soc_Aud_AFE_IO_Block_MEM_VUL);
		SetIntfConnection(Soc_Aud_InterCon_DisConnect,
			Soc_Aud_AFE_IO_Block_MEM_DL1,
			Soc_Aud_AFE_IO_Block_MEM_VUL_DATA2);
		SetIntfConnection(Soc_Aud_InterCon_DisConnect,
			Soc_Aud_AFE_IO_Block_MEM_DL2,
			Soc_Aud_AFE_IO_Block_MEM_VUL_DATA2);

		SetConnectionFormat(OUTPUT_DATA_FORMAT_24BIT,
				Soc_Aud_AFE_IO_Block_MEM_VUL);
		SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_VUL_DATA2, false);
		SetMemoryPathEnable(Soc_Aud_Digital_Block_ADDA_UL, false);
		if (GetMemoryPathEnable(Soc_Aud_Digital_Block_ADDA_UL) == false)
			set_adc_enable(false);

		RemoveMemifSubStream(Soc_Aud_Digital_Block_MEM_VUL_DATA2,
						substream);

		EnableAfe(false);
		mPrepareDone = false;
	}
	AudDrv_Clk_Off();
	return 0;
}

static int mtk_hardware_merge_alsa_start(struct snd_pcm_substream *substream)
{
	pr_warn("%s\n", __func__);

	/* here to set interrupt */
	irq_add_user(substream,
		     irq_request_number(Soc_Aud_Digital_Block_MEM_VUL_DATA2),
		     substream->runtime->rate,
		     substream->runtime->period_size);

	SetSampleRate(Soc_Aud_Digital_Block_MEM_VUL_DATA2,
				substream->runtime->rate);
	SetChannels(Soc_Aud_Digital_Block_MEM_VUL_DATA2,
				substream->runtime->channels);
	SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_VUL_DATA2, true);

	EnableAfe(true);
	return 0;
}

static int mtk_hardware_merge_pcm_trigger(
				struct snd_pcm_substream *substream, int cmd)
{
	pr_warn("mtk_hardware_merge_pcm_trigger cmd = %d\n", cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		return mtk_hardware_merge_alsa_start(substream);
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		return mtk_hardware_merge_alsa_stop(substream);
	}
	return -EINVAL;
}

static int mtk_hardware_merge_pcm_copy(struct snd_pcm_substream *substream,
				 int channel, snd_pcm_uframes_t pos,
				 void __user *dst, snd_pcm_uframes_t count)
{
	return mtk_memblk_copy(substream, channel, pos, dst, count,
		Hardware_Merge_Control_context,
		Soc_Aud_Digital_Block_MEM_VUL_DATA2);
}

static int mtk_hardware_merge_pcm_silence(struct snd_pcm_substream *substream,
				    int channel, snd_pcm_uframes_t pos,
				    snd_pcm_uframes_t count)
{
	pr_warn("dummy_pcm_silence\n");
	return 0; /* do nothing */
}


static void *dummy_page[2];

static struct page *mtk_hardware_merge_pcm_page(
		struct snd_pcm_substream *substream, unsigned long offset)
{
	pr_warn("%s\n", __func__);
	return virt_to_page(dummy_page[substream->stream]); /* the same page */
}


static struct snd_pcm_ops mtk_afe_hardware_merge_ops = {
	.open =     mtk_hardware_merge_pcm_open,
	.close =    mtk_hardware_merge_pcm_close,
	.ioctl =    snd_pcm_lib_ioctl,
	.hw_params =    mtk_hardware_merge_pcm_hw_params,
	.hw_free =  mtk_hardware_merge_capture_pcm_hw_free,
	.prepare =  mtk_hardware_merge_pcm_prepare,
	.trigger =  mtk_hardware_merge_pcm_trigger,
	.pointer =  mtk_hardware_merge_pcm_pointer,
	.copy =     mtk_hardware_merge_pcm_copy,
	.silence =  mtk_hardware_merge_pcm_silence,
	.page =     mtk_hardware_merge_pcm_page,
};

static struct snd_soc_platform_driver mtk_soc_platform = {
	.ops        = &mtk_afe_hardware_merge_ops,
	.probe      = mtk_hardware_merge_data_probe,
};

static int mtk_Hardware_merge_probe(struct platform_device *pdev)
{
	pr_warn("mtk_Hardware_merge_probe\n");

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(64);
	if (pdev->dev.dma_mask == NULL)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	if (pdev->dev.of_node)
		dev_set_name(&pdev->dev, "%s", MT_SOC_HW_MERGE_PCM);

	pr_warn("%s: dev name %s\n", __func__, dev_name(&pdev->dev));
	return snd_soc_register_platform(&pdev->dev,
					 &mtk_soc_platform);
}

static int mtk_hardware_merge_data_probe(struct snd_soc_platform *platform)
{
	pr_warn("mtk_hardware_merge_data_probe\n");
	AudDrv_Allocate_mem_Buffer(platform->dev,
				Soc_Aud_Digital_Block_MEM_VUL_DATA2,
				UL1_MAX_BUFFER_SIZE);
	Hardware_Merge_Capture_dma_buf =  Get_Mem_Buffer(
				Soc_Aud_Digital_Block_MEM_VUL_DATA2);
	return 0;
}


static int mtk_hardware_merge_remove(struct platform_device *pdev)
{
	pr_debug("%s\n", __func__);
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mt_soc_pcm_hardware_merge_of_ids[] = {
	{ .compatible = "mediatek,mt_soc_pcm_hw_merge_capture", },
	{}
};
#endif

static struct platform_driver mtk_afe_hardware_merge_driver = {
	.driver = {
		.name = MT_SOC_HW_MERGE_PCM,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = mt_soc_pcm_hardware_merge_of_ids,
#endif
	},
	.probe = mtk_Hardware_merge_probe,
	.remove = mtk_hardware_merge_remove,
};

#ifndef CONFIG_OF
static struct platform_device *soc_mtkafe_hardware_merge_dev;
#endif

static int __init mtk_soc_hardware_merge_platform_init(void)
{
	int ret = 0;

	pr_warn("%s\n", __func__);
#ifndef CONFIG_OF
	soc_mtkafe_hardware_merge_dev = platform_device_alloc(
				MT_SOC_HW_MERGE_PCM, -1);
	if (!soc_mtkafe_hardware_merge_dev)
		return -ENOMEM;

	ret = platform_device_add(soc_mtkafe_hardware_merge_dev);
	if (ret != 0) {
		platform_device_put(soc_mtkafe_hardware_merge_dev);
		return ret;
	}
#endif

	ret = platform_driver_register(&mtk_afe_hardware_merge_driver);
	return ret;
}
module_init(mtk_soc_hardware_merge_platform_init);

static void __exit mtk_soc_hardware_merge_platform_exit(void)
{

	pr_warn("%s\n", __func__);
	platform_driver_unregister(&mtk_afe_hardware_merge_driver);
}

module_exit(mtk_soc_hardware_merge_platform_exit);

MODULE_DESCRIPTION("AFE Hardware Merge module platform driver");
MODULE_LICENSE("GPL");
