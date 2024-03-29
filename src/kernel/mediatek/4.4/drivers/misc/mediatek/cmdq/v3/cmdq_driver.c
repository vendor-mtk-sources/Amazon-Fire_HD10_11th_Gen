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
 */

#include "cmdq_driver.h"
#include "cmdq_struct.h"
#include "cmdq_core.h"
#include "cmdq_virtual.h"
#include "cmdq_reg.h"
#include "cmdq_mdp_common.h"
#include "cmdq_device.h"
#include "cmdq_sec.h"
#include "mdp_ioctl_ex.h"
#include "mdp_def_ex.h"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/suspend.h>
#ifdef CMDQ_USE_LEGACY
#include <mach/mt_boot.h>
#endif

#include <linux/pm_runtime.h>
/**
 * @device tree porting note
 * alps/kernel-3.10/arch/arm64/boot/dts/{platform}.dts
 *  - use of_device_id to match driver and device
 *  - use io_map to map and get VA of HW's rgister
**/
static const struct of_device_id cmdq_of_ids[] = {
	{.compatible = "mediatek,gce",},
	{}
};

static dev_t gCmdqDevNo;
static struct cdev *gCmdqCDev;
static struct class *gCMDQClass;

void cmdq_driver_dump_readback(u32 *addrs, u32 count, u32 *values)
{}

static ssize_t cmdq_driver_dummy_write(struct device *dev,
				       struct device_attribute *attr, const char *buf, size_t size)
{
	return -EACCES;
}

static DEVICE_ATTR(error, S_IRUSR | S_IWUSR, cmdqCorePrintError, cmdq_driver_dummy_write);
static DEVICE_ATTR(log_level, S_IRUSR | S_IWUSR, cmdqCorePrintLogLevel, cmdqCoreWriteLogLevel);
static DEVICE_ATTR(profile_enable, S_IRUSR | S_IWUSR, cmdqCorePrintProfileEnable,
		   cmdqCoreWriteProfileEnable);


static int cmdq_proc_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, cmdqCorePrintStatusSeq, inode->i_private);
}

static int cmdq_proc_record_open(struct inode *inode, struct file *file)
{
	return single_open(file, cmdqCorePrintRecordSeq, inode->i_private);
}

static const struct file_operations cmdqDebugStatusOp = {
	.owner = THIS_MODULE,
	.open = cmdq_proc_status_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations cmdqDebugRecordOp = {
	.owner = THIS_MODULE,
	.open = cmdq_proc_record_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

#ifdef CMDQ_INSTRUCTION_COUNT
static DEVICE_ATTR(instruction_count_level, S_IRUSR | S_IWUSR, cmdqCorePrintInstructionCountLevel,
		   cmdqCoreWriteInstructionCountLevel);

static int cmdq_proc_instruction_count_open(struct inode *inode, struct file *file)
{
	return single_open(file, cmdqCorePrintInstructionCountSeq, inode->i_private);
}

static const struct file_operations cmdqDebugInstructionCountOp = {
	.owner = THIS_MODULE,
	.open = cmdq_proc_instruction_count_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

static int cmdq_open(struct inode *pInode, struct file *pFile)
{
	struct cmdqFileNodeStruct *pNode;

	CMDQ_VERBOSE("CMDQ driver open fd=%p begin\n", pFile);

	pFile->private_data = kzalloc(sizeof(struct cmdqFileNodeStruct), GFP_KERNEL);
	if (pFile->private_data == NULL) {
		CMDQ_ERR("Can't allocate memory for CMDQ file node\n");
		return -ENOMEM;
	}

	pNode = (struct cmdqFileNodeStruct *) pFile->private_data;
	pNode->userPID = current->pid;
	pNode->userTGID = current->tgid;

	INIT_LIST_HEAD(&(pNode->taskList));
	spin_lock_init(&pNode->nodeLock);

	CMDQ_VERBOSE("CMDQ driver open end\n");

	return 0;
}


static int cmdq_release(struct inode *pInode, struct file *pFile)
{
	struct cmdqFileNodeStruct *pNode;
	unsigned long flags;

	CMDQ_VERBOSE("CMDQ driver release fd=%p begin\n", pFile);

	pNode = (struct cmdqFileNodeStruct *) pFile->private_data;

	if (pNode == NULL) {
		CMDQ_ERR("CMDQ file node NULL\n");
		return -EFAULT;
	}

	spin_lock_irqsave(&pNode->nodeLock, flags);

	/* note that we did not release CMDQ tasks */
	/* issued by this file node, */
	/* since their HW operation may be pending. */

	spin_unlock_irqrestore(&pNode->nodeLock, flags);

	/* scan through tasks that created by this file node and release them */
	cmdq_core_release_task_by_file_node((void *)pNode);
	cmdqCoreFreeWriteAddressNode((void *)pNode);

	if (pFile->private_data != NULL) {
		kfree(pFile->private_data);
		pFile->private_data = NULL;
	}

	CMDQ_VERBOSE("CMDQ driver release end\n");

	return 0;
}

static int cmdq_driver_create_reg_address_buffer(struct cmdqCommandStruct *pCommand)
{
	int status = 0;
	uint32_t totalRegCount = 0;
	uint32_t *regAddrBuf = NULL;

	uint32_t *kernelRegAddr = NULL;
	uint32_t kernelRegCount = 0;

	const uint32_t userRegCount = pCommand->regRequest.count;


	if (pCommand->debugRegDump != 0) {
		/* get kernel dump request count */
		status =
		    cmdqCoreDebugRegDumpBegin(pCommand->debugRegDump, &kernelRegCount,
					      &kernelRegAddr);
		if (status != 0) {
			CMDQ_ERR
			    ("cmdqCoreDebugRegDumpBegin returns %d, ignore kernel reg dump request\n",
			     status);
			kernelRegCount = 0;
			kernelRegAddr = NULL;
		}
	}

	/* how many register to dump? */
	if (kernelRegCount > CMDQ_MAX_DUMP_REG_COUNT || userRegCount > CMDQ_MAX_DUMP_REG_COUNT)
		return -EINVAL;
	totalRegCount = kernelRegCount + userRegCount;

	if (totalRegCount == 0) {
		/* no need to dump register */
		pCommand->regRequest.count = 0;
		pCommand->regValue.count = 0;
		pCommand->regRequest.regAddresses = (cmdqU32Ptr_t) (unsigned long)NULL;
		pCommand->regValue.regValues = (cmdqU32Ptr_t) (unsigned long)NULL;
	} else {
		regAddrBuf = kcalloc(totalRegCount, sizeof(uint32_t), GFP_KERNEL);
		if (regAddrBuf == NULL)
			return -ENOMEM;

		/* collect user space dump request */
		if (userRegCount) {
			if (copy_from_user
			    (regAddrBuf, CMDQ_U32_PTR(pCommand->regRequest.regAddresses),
			     userRegCount * sizeof(uint32_t))) {
				kfree(regAddrBuf);
				return -EFAULT;
			}
		}

		/* collect kernel space dump request, concatnate after user space request */
		if (kernelRegCount) {
			memcpy(regAddrBuf + userRegCount, kernelRegAddr,
			       kernelRegCount * sizeof(uint32_t));
		}


		/* replace address buffer and value address buffer with kzalloc memory */
		pCommand->regRequest.regAddresses = (cmdqU32Ptr_t) (unsigned long)(regAddrBuf);
		pCommand->regRequest.count = totalRegCount;
	}

	return 0;
}

static void cmdq_driver_process_read_address_request(struct cmdqReadAddressStruct *req_user)
{
	/* create kernel-space buffer for working */
	uint32_t *addrs = NULL;
	uint32_t *values = NULL;

	CMDQ_MSG("[READ_PA] cmdq_driver_process_read_address_request()\n");

	do {
		if (req_user == NULL ||
		    req_user->count == 0 ||
		    req_user->count > CMDQ_MAX_DUMP_REG_COUNT ||
		    CMDQ_U32_PTR(req_user->values) == NULL ||
		    CMDQ_U32_PTR(req_user->dmaAddresses) == NULL) {
			CMDQ_ERR("[READ_PA] invalid req_user\n");
			break;
		}

		addrs = kcalloc(req_user->count, sizeof(uint32_t), GFP_KERNEL);
		if (addrs == NULL) {
			CMDQ_ERR("[READ_PA] fail to alloc addr buf\n");
			break;
		}

		values = kcalloc(req_user->count, sizeof(uint32_t), GFP_KERNEL);
		if (values == NULL) {
			CMDQ_ERR("[READ_PA] fail to alloc value buf\n");
			break;
		}

		/* copy from user */
		if (copy_from_user
		    (addrs, CMDQ_U32_PTR(req_user->dmaAddresses),
		     req_user->count * sizeof(uint32_t))) {
			CMDQ_ERR("[READ_PA] fail to copy user dmaAddresses\n");
			break;
		}

		/* actually read these PA write buffers */
		cmdqCoreReadWriteAddressBatch(addrs, req_user->count, values);

		/* copy value to user */
		if (copy_to_user
		    (CMDQ_U32_PTR(req_user->values), values, req_user->count * sizeof(uint32_t))) {
			CMDQ_ERR("[READ_PA] fail to copy to user value buf\n");
			break;
		}

	} while (0);


	kfree(addrs);
	addrs = NULL;

	kfree(values);
	values = NULL;

}

#define CMDQ_PTR_FREE_NULL(ptr) \
do { \
	vfree(CMDQ_U32_PTR((ptr))); \
	(ptr) = 0; \
} while (0)

static long cmdq_driver_destroy_secure_medadata(struct cmdqCommandStruct *pCommand)
{
	u32 i;

	kfree(CMDQ_U32_PTR(pCommand->secData.addrMetadatas));
	pCommand->secData.addrMetadatas = 0;

	for (i = 0; i < ARRAY_SIZE(pCommand->secData.ispMeta.ispBufs); i++)
		CMDQ_PTR_FREE_NULL(pCommand->secData.ispMeta.ispBufs[i].va);

	return 0;
}

#ifdef CMDQ_SECURE_PATH_SUPPORT
static s32 cmdq_driver_copy_meta(void *src, void **dest, size_t copy_size,
	size_t max_size, bool vm)
{
	void *meta_buf;

	if (!copy_size)
		return -EINVAL;

	if (copy_size > max_size) {
		CMDQ_ERR("source size exceed:%zu > %zu", copy_size, max_size);
		return -EFAULT;
	}

	if (vm)
		meta_buf = vzalloc(copy_size);
	else
		meta_buf = kzalloc(copy_size, GFP_KERNEL);
	if (!meta_buf) {
		CMDQ_ERR("allocate size fail:%zu\n", copy_size);
		return -ENOMEM;
	}
	*dest = meta_buf;

	if (copy_from_user(meta_buf, src, copy_size)) {
		CMDQ_ERR("fail to copy user data\n");
		return -EFAULT;
	}

	return 0;
}
#endif

static long cmdq_driver_create_secure_medadata(struct cmdqCommandStruct *pCommand)
{
#ifdef CMDQ_SECURE_PATH_SUPPORT
	u32 length, max_length;
	void *meta_buf;
	s32 ret;
	void *addr_meta = CMDQ_U32_PTR(pCommand->secData.addrMetadatas);
	void *isp_bufs[ARRAY_SIZE(pCommand->secData.ispMeta.ispBufs)] = {0};
	u32 i;

	max_length = CMDQ_IWC_MAX_ADDR_LIST_LENGTH * sizeof(struct cmdqSecAddrMetadataStruct);
	length = pCommand->secData.addrMetadataCount * sizeof(struct cmdqSecAddrMetadataStruct);

	/* always clear to prevent free unknown memory */
	pCommand->secData.addrMetadatas = 0;
	for (i = 0; i < ARRAY_SIZE(pCommand->secData.ispMeta.ispBufs); i++) {
		isp_bufs[i] = (void *)(uintptr_t)pCommand->secData.ispMeta.ispBufs[i].va;
		pCommand->secData.ispMeta.ispBufs[i].va = 0;
	}

	/* verify parameter */
	if (!pCommand->secData.is_secure && pCommand->secData.addrMetadataCount) {
		/* normal path with non-zero secure metadata */
		CMDQ_ERR
		    ("[secData]mismatch secData.is_secure(%d) and secData.addrMetadataCount(%d)\n",
		     pCommand->secData.is_secure, pCommand->secData.addrMetadataCount);
		return -EFAULT;
	}

	/* mdp max secure matadata count is 9. */
	if (pCommand->secData.addrMetadataCount >= 10) {
		CMDQ_ERR("invalid metadata count:%d\n",
			pCommand->secData.addrMetadataCount);
		return -EFAULT;
	}

	/* revise max count field */
	pCommand->secData.addrMetadataMaxCount = pCommand->secData.addrMetadataCount;

	/* bypass 0 metadata case */
	if (!pCommand->secData.addrMetadataCount)
		return 0;

	/* create kernel-space buffer for working */
	meta_buf = NULL;
	ret = cmdq_driver_copy_meta(addr_meta, &meta_buf, length, max_length,
		false);
	if (ret < 0) {
		CMDQ_ERR("[secData]copy meta fail count:%d alloacted size:%d ret:%d\n",
			 pCommand->secData.addrMetadataCount, length, ret);
		/* replace buffer first to ensure that
		 * meta_buf is valid kernel space buffer address when free it
		 * crazy casting to cast 64bit int to 32/64 bit pointer
		 */
		pCommand->secData.addrMetadatas = (cmdqU32Ptr_t)(unsigned long)meta_buf;
		/* free secure path metadata */
		cmdq_driver_destroy_secure_medadata(pCommand);
		return ret;
	}
	/* replace buffer with kernel buffer */
	pCommand->secData.addrMetadatas = (cmdqU32Ptr_t)(unsigned long)meta_buf;

	/* check isp data valid */

	for (i = 0; i < ARRAY_SIZE(pCommand->secData.ispMeta.ispBufs); i++) {
		if (!isp_bufs[i])
			continue;
		meta_buf = NULL;
		ret = cmdq_driver_copy_meta(isp_bufs[i], &meta_buf,
			pCommand->secData.ispMeta.ispBufs[i].size,
			isp_iwc_buf_size[i], true);
		pCommand->secData.ispMeta.ispBufs[i].va =
			(cmdqU32Ptr_t)(unsigned long)meta_buf;
		if (ret < 0) {
			CMDQ_ERR(
				"[secData]copy meta %u size:%llu va:0x%llx ret:%d\n",
				i, pCommand->secData.ispMeta.ispBufs[i].size,
				pCommand->secData.ispMeta.ispBufs[i].va,
				ret);
			pCommand->secData.ispMeta.ispBufs[i].size = 0;
		}
	}

#if 0
	cmdq_core_dump_secure_metadata(&(pCommand->secData));
#endif
#endif
	return 0;
}

long cmdq_driver_process_command_request(
	struct cmdqCommandStruct *pCommand, struct CmdqRecExtend *ext)
{
	int32_t status = 0;
	uint32_t *userRegValue = NULL;
	uint32_t userRegCount = 0;

	if (pCommand->regRequest.count != pCommand->regValue.count) {
		CMDQ_ERR("mismatch regRequest and regValue\n");
		return -EFAULT;
	}

	/* avoid copy large string */
	if (pCommand->userDebugStrLen > CMDQ_MAX_DBG_STR_LEN)
		pCommand->userDebugStrLen = CMDQ_MAX_DBG_STR_LEN;

	/* allocate secure medatata */
	status = cmdq_driver_create_secure_medadata(pCommand);
	if (status != 0) {
		CMDQ_ERR("create secure metadata failed:%d\n", status);
		return status;
	}

	/* backup since we are going to replace these */
	userRegValue = CMDQ_U32_PTR(pCommand->regValue.regValues);
	userRegCount = pCommand->regValue.count;

	/* create kernel-space address buffer */
	status = cmdq_driver_create_reg_address_buffer(pCommand);
	if (status != 0) {
		/* free secure path metadata */
		cmdq_driver_destroy_secure_medadata(pCommand);
		CMDQ_ERR("create reg addr buffer failed:%d\n", status);
		return status;
	}

	/* create kernel-space value buffer */
	pCommand->regValue.regValues = (cmdqU32Ptr_t) (unsigned long)
	    kzalloc(pCommand->regRequest.count * sizeof(uint32_t), GFP_KERNEL);
	pCommand->regValue.count = pCommand->regRequest.count;
	if (CMDQ_U32_PTR(pCommand->regValue.regValues) == NULL) {
		kfree(CMDQ_U32_PTR(pCommand->regRequest.regAddresses));
		CMDQ_ERR("create reg values buffer failed\n");
		return -ENOMEM;
	}

	/* scenario id fixup */
	cmdq_core_fix_command_scenario_for_user_space(pCommand);

	status = cmdqCoreSubmitTask(pCommand, ext);
	if (status < 0) {
		CMDQ_ERR("Submit user commands for execution failed = %d\n", status);
		cmdq_driver_destroy_secure_medadata(pCommand);

		kfree(CMDQ_U32_PTR(pCommand->regRequest.regAddresses));
		kfree(CMDQ_U32_PTR(pCommand->regValue.regValues));
		return -EFAULT;
	}

	/* notify kernel space dump callback */
	if (pCommand->debugRegDump != 0) {
		status = cmdqCoreDebugRegDumpEnd(pCommand->debugRegDump,
						 pCommand->regRequest.count - userRegCount,
						 CMDQ_U32_PTR(pCommand->regValue.regValues) +
						 userRegCount);
		if (status != 0) {
			/* Error status print */
			CMDQ_ERR("cmdqCoreDebugRegDumpEnd returns %d\n", status);
		}
	}

	/* copy back to user space buffer */
	if (userRegValue && userRegCount) {
		/* copy results back to user space */
		CMDQ_VERBOSE("regValue[0] is %d\n", CMDQ_U32_PTR(pCommand->regValue.regValues)[0]);
		if (copy_to_user
		    (userRegValue, CMDQ_U32_PTR(pCommand->regValue.regValues),
		     userRegCount * sizeof(uint32_t))) {
			CMDQ_ERR("Copy REGVALUE to user space failed\n");
		}
	}

	/* free allocated kernel buffers */
	kfree(CMDQ_U32_PTR(pCommand->regRequest.regAddresses));
	kfree(CMDQ_U32_PTR(pCommand->regValue.regValues));

	if (pCommand->readAddress.count > 0)
		cmdq_driver_process_read_address_request(&pCommand->readAddress);

	/* free allocated secure metadata */
	cmdq_driver_destroy_secure_medadata(pCommand);

	return 0;
}

bool cmdq_driver_support_wait_and_receive_event_in_same_tick(void)
{
#ifdef CMDQ_USE_LEGACY
	const unsigned int code = mt_get_chip_hw_code();
	CHIP_SW_VER ver = mt_get_chip_sw_ver();
	bool support = false;

	if (code == 0x6795) {
		support = true;
	} else if (ver >= CHIP_SW_VER_02) {
		/* SW V2 */
		support = true;
	} else if (ver >= CHIP_SW_VER_01) {
		support = false;
	}

	return support;
#else
	return true;
#endif
}

s32 cmdq_driver_copy_task_prop_from_user(void *from, u32 size, void **to)
{
	void *task_prop = NULL;

	/* considering backward compatible, we won't return error when argument not available */
	if (from && size && to) {
		task_prop = kzalloc(size, GFP_KERNEL);
		if (!task_prop) {
			CMDQ_ERR("allocate task_prop failed\n");
			return -ENOMEM;
		}

		if (copy_from_user(task_prop, from, size)) {
			CMDQ_ERR("cannot copy task property from user, size=%d\n", size);
			kfree(task_prop);
			return -EFAULT;
		}

		*to = task_prop;
	} else if (to) {
		CMDQ_LOG("Initialize prop_addr to NULL...\n");
		*to = NULL;
	}

	return 0;
}

void cmdq_release_task_property(void **prop_addr, u32 *prop_size)
{
	if (!prop_addr || !prop_size || !*prop_size) {
		CMDQ_LOG("Return w/o need of kfree(prop_addr)\n");
		return;
	}

	kfree(*prop_addr);
	*prop_addr = NULL;
	*prop_size = 0;
}


s32 cmdq_driver_ioctl_query_usage(struct file *pf, unsigned long param)
{
	int count[CMDQ_MAX_ENGINE_COUNT] = {0};

	if (cmdqCoreQueryUsage(count))
		return -EFAULT;

	if (copy_to_user((void *)param, count,
		sizeof(int32_t) * CMDQ_MAX_ENGINE_COUNT)) {
		CMDQ_ERR("CMDQ_IOCTL_QUERY_USAGE copy_to_user failed\n");
		return -EFAULT;
	}
	return 0;
}

s32 cmdq_driver_ioctl_query_cap_bits(unsigned long param)
{
	int capBits = 0;

	if (cmdq_driver_support_wait_and_receive_event_in_same_tick())
		capBits |= (1L << CMDQ_CAP_WFE);
	else
		capBits &= ~(1L << CMDQ_CAP_WFE);

	if (copy_to_user((void *)param, &capBits, sizeof(int))) {
		CMDQ_ERR("Copy capacity bits to user space failed\n");
		return -EFAULT;
	}
	return 0;
}

s32 cmdq_driver_ioctl_query_dts(unsigned long param)
{
	struct cmdqDTSDataStruct *pDtsData;

	pDtsData = cmdq_core_get_whole_DTS_Data();

	if (copy_to_user((void *)param, pDtsData,
		sizeof(struct cmdqDTSDataStruct))) {
		CMDQ_ERR("Copy device tree to user space failed\n");
		return -EFAULT;
	}
	return 0;
}

s32 cmdq_driver_ioctl_notify_engine(unsigned long param)
{
	uint64_t engineFlag;

	if (copy_from_user(&engineFlag, (void *)param, sizeof(uint64_t))) {
		CMDQ_ERR("CMDQ_IOCTL_NOTIFY_ENGINE copy_from_user failed\n");
		return -EFAULT;
	}
	cmdqCoreLockResource(engineFlag, true);
	return 0;
}

static long cmdq_ioctl(struct file *pFile, unsigned int code, unsigned long param)
{
	int32_t status = 0;

	switch (code) {
	case CMDQ_IOCTL_QUERY_USAGE:
		status = cmdq_driver_ioctl_query_usage(pFile, param);
		break;
	case CMDQ_IOCTL_QUERY_CAP_BITS:
		status = cmdq_driver_ioctl_query_cap_bits(param);
		break;
	case CMDQ_IOCTL_QUERY_DTS:
		status = cmdq_driver_ioctl_query_dts(param);
		break;
	case CMDQ_IOCTL_NOTIFY_ENGINE:
		status = cmdq_driver_ioctl_notify_engine(param);
		break;
	case CMDQ_IOCTL_ASYNC_EXEC:
		CMDQ_MSG("ioctl CMDQ_IOCTL_ASYNC_EXEC\n");
		status = mdp_ioctl_async_exec(pFile, param);
		break;
	case CMDQ_IOCTL_ASYNC_WAIT:
		CMDQ_MSG("ioctl CMDQ_IOCTL_ASYNC_WAIT\n");
		status = mdp_ioctl_async_wait(param);
		break;
	case CMDQ_IOCTL_ALLOC_READBACK_SLOTS:
		CMDQ_MSG("ioctl CMDQ_IOCTL_ALLOC_READBACK_SLOTS\n");
		status = mdp_ioctl_alloc_readback_slots(pFile, param);
		break;
	case CMDQ_IOCTL_FREE_READBACK_SLOTS:
		CMDQ_MSG("ioctl CMDQ_IOCTL_FREE_READBACK_SLOTS\n");
		status = mdp_ioctl_free_readback_slots(pFile, param);
		break;
	case CMDQ_IOCTL_READ_READBACK_SLOTS:
		CMDQ_MSG("ioctl CMDQ_IOCTL_READ_READBACK_SLOTS\n");
		status = mdp_ioctl_read_readback_slots(param);
		break;
	default:
		CMDQ_ERR("unrecognized ioctl 0x%08x\n", code);
		return -ENOIOCTLCMD;
	}

	return status;
}

#ifdef CONFIG_COMPAT
static long cmdq_ioctl_compat(struct file *pFile, unsigned int code, unsigned long param)
{
	switch (code) {
	case CMDQ_IOCTL_QUERY_USAGE:
	case CMDQ_IOCTL_QUERY_CAP_BITS:
	case CMDQ_IOCTL_QUERY_DTS:
	case CMDQ_IOCTL_NOTIFY_ENGINE:
	case CMDQ_IOCTL_ASYNC_EXEC:
	case CMDQ_IOCTL_ASYNC_WAIT:
	case CMDQ_IOCTL_ALLOC_READBACK_SLOTS:
	case CMDQ_IOCTL_FREE_READBACK_SLOTS:
	case CMDQ_IOCTL_READ_READBACK_SLOTS:
		/* All ioctl structures should be the same size in 32-bit and 64-bit linux. */
		return cmdq_ioctl(pFile, code, param);
	case CMDQ_IOCTL_LOCK_MUTEX:
	case CMDQ_IOCTL_UNLOCK_MUTEX:
		CMDQ_ERR("[COMPAT]deprecated ioctl 0x%08x\n", code);
		return -ENOIOCTLCMD;
	default:
		CMDQ_ERR("[COMPAT]unrecognized ioctl 0x%08x\n", code);
		return -ENOIOCTLCMD;
	}

	CMDQ_ERR("[COMPAT]unrecognized ioctl 0x%08x\n", code);
	return -ENOIOCTLCMD;
}
#endif


static const struct file_operations cmdqOP = {
	.owner = THIS_MODULE,
	.open = cmdq_open,
	.release = cmdq_release,
	.unlocked_ioctl = cmdq_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = cmdq_ioctl_compat,
#endif
};

static int cmdq_pm_notifier_cb(struct notifier_block *nb, unsigned long event, void *ptr)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:	/* Going to suspend the system */
		/* The next stage is freeze process. */
		/* We will queue all request in suspend callback, */
		/* so don't care this stage */
		return NOTIFY_DONE;	/* don't care this event */
	case PM_POST_SUSPEND:
		/* processes had resumed in previous stage (system resume callback) */
		/* resume CMDQ driver to execute. */
		cmdqCoreResumedNotifier();
		return NOTIFY_OK;	/* process done */
	default:
		return NOTIFY_DONE;
	}
	return NOTIFY_DONE;
}

/* Hibernation and suspend events */
static struct notifier_block cmdq_pm_notifier_block = {
	.notifier_call = cmdq_pm_notifier_cb,
	.priority = 5,
};

static irqreturn_t cmdq_irq_handler(int IRQ, void *pDevice)
{
	int index;
	uint32_t irqStatus;
	bool handled = false;	/* we share IRQ bit with CQ-DMA, */
	/* so it is possible that this handler */
	/* is called but GCE does not have IRQ flag. */
	const u32 max_thread_count = cmdq_dev_get_thread_count();

	do {
		if (cmdq_dev_get_irq_id() == IRQ) {
			if (!cmdq_core_is_clock_enabled()) {
				CMDQ_ERR("Got IRQ when clock is disabled\n");
				break;
			}
			irqStatus = CMDQ_REG_GET32(CMDQ_CURR_IRQ_STATUS) & 0x0FFFF;
			for (index = 0; (irqStatus != 0xFFFF) && index < max_thread_count;
			     index++) {
				/* STATUS bit set to 0 means IRQ asserted */
				if (irqStatus & (1 << index))
					continue;

				/* so we mark irqStatus to 1 to denote finished processing */
				/* and we can early-exit if no more threads being asserted */
				irqStatus |= (1 << index);

				cmdqCoreHandleIRQ(index);
				handled = true;
			}
		} else if (cmdq_dev_get_irq_secure_id() == IRQ) {
			CMDQ_ERR("receive secure IRQ %d in NWD\n", IRQ);
		}
	} while (0);

	if (handled) {
		cmdq_core_add_consume_task();
		return IRQ_HANDLED;
	}
	/* allow CQ-DMA to process this IRQ bit */
	return IRQ_NONE;
}

static int cmdq_create_debug_entries(void)
{
	struct proc_dir_entry *debugDirEntry = NULL;

	debugDirEntry = proc_mkdir(CMDQ_DRIVER_DEVICE_NAME "_debug", NULL);
	if (debugDirEntry) {
		struct proc_dir_entry *entry = NULL;

		entry = proc_create("status", 0440, debugDirEntry, &cmdqDebugStatusOp);
		entry = proc_create("record", 0440, debugDirEntry, &cmdqDebugRecordOp);
#ifdef CMDQ_INSTRUCTION_COUNT
		entry =
		    proc_create("instructionCount", 0440, debugDirEntry,
				&cmdqDebugInstructionCountOp);
#endif
	}

	return 0;
}

static int cmdq_probe(struct platform_device *pDevice)
{
	int status;
	struct device *cmdq_dev = NULL;

	CMDQ_MSG("CMDQ driver probe begin\n");

	/* Function link */
	cmdq_virtual_function_setting();

	/* init cmdq device related data */
	cmdq_dev_init(pDevice);

	/* init cmdq context */
	cmdqCoreInitialize();

	status = alloc_chrdev_region(&gCmdqDevNo, 0, 1, CMDQ_DRIVER_DEVICE_NAME);
	if (status != 0) {
		/* Cannot get CMDQ device major number */
		CMDQ_ERR("Get CMDQ device major number(%d) failed(%d)\n", gCmdqDevNo, status);
	} else {
		/* Get CMDQ device major number successfully */
		CMDQ_MSG("Get CMDQ device major number(%d) success(%d)\n", gCmdqDevNo, status);
	}

	/* ioctl access point (/dev/mtk_cmdq) */
	gCmdqCDev = cdev_alloc();
	gCmdqCDev->owner = THIS_MODULE;
	gCmdqCDev->ops = &cmdqOP;

	status = cdev_add(gCmdqCDev, gCmdqDevNo, 1);

	gCMDQClass = class_create(THIS_MODULE, CMDQ_DRIVER_DEVICE_NAME);
	cmdq_dev = device_create(gCMDQClass, NULL, gCmdqDevNo, NULL, CMDQ_DRIVER_DEVICE_NAME);

	status =
	    request_irq(cmdq_dev_get_irq_id(), cmdq_irq_handler,
			IRQF_TRIGGER_LOW | IRQF_SHARED, CMDQ_DRIVER_DEVICE_NAME, gCmdqCDev);
	if (status != 0) {
		CMDQ_ERR("Register cmdq driver irq handler(%d) failed(%d)\n", gCmdqDevNo, status);
		return -EFAULT;
	}

	/* although secusre CMDQ driver is responsible for handle secure IRQ, */
	/* MUST registet secure IRQ to GIC in normal world to ensure it will be initialize correctly */
	/* (that's because t-base does not support GIC init IRQ in secure world...) */
#ifdef CMDQ_SECURE_PATH_SUPPORT
	status =
	    request_irq(cmdq_dev_get_irq_secure_id(), cmdq_irq_handler, IRQF_TRIGGER_LOW,
			CMDQ_DRIVER_DEVICE_NAME, gCmdqCDev);
	CMDQ_MSG("register sec IRQ:%d\n", cmdq_dev_get_irq_secure_id());
	if (status != 0) {
		CMDQ_ERR("Register cmdq driver secure irq handler(%d) failed(%d)\n", gCmdqDevNo,
			 status);
		return -EFAULT;
	}
#endif

	/* proc debug access point */
	cmdq_create_debug_entries();

	/* device attributes for debugging */
	device_create_file(&pDevice->dev, &dev_attr_error);
	device_create_file(&pDevice->dev, &dev_attr_log_level);
	device_create_file(&pDevice->dev, &dev_attr_profile_enable);
#ifdef CMDQ_INSTRUCTION_COUNT
	device_create_file(&pDevice->dev, &dev_attr_instruction_count_level);
#endif

	mdp_limit_dev_create(pDevice);
	CMDQ_MSG("CMDQ driver probe end\n");
	cmdq_mdp_get_func()->mdp_probe();
	return 0;
}


static int cmdq_remove(struct platform_device *pDevice)
{
	disable_irq(cmdq_dev_get_irq_id());

	device_remove_file(&pDevice->dev, &dev_attr_error);
	device_remove_file(&pDevice->dev, &dev_attr_log_level);
	device_remove_file(&pDevice->dev, &dev_attr_profile_enable);
#ifdef CMDQ_INSTRUCTION_COUNT
	device_remove_file(&pDevice->dev, &dev_attr_instruction_count_level);
#endif
	return 0;
}


static int cmdq_suspend(struct device *pDevice)
{
	return cmdqCoreSuspend();
}


static int cmdq_resume(struct device *pDevice)
{
	return cmdqCoreResume();
}

static int cmdq_pm_restore_noirq(struct device *pDevice)
{
	return 0;
}

static const struct dev_pm_ops cmdq_pm_ops = {
	.suspend = cmdq_suspend,
	.resume = cmdq_resume,
	.freeze = NULL,
	.thaw = NULL,
	.poweroff = NULL,
	.restore = NULL,
	.restore_noirq = cmdq_pm_restore_noirq,
};


static struct platform_driver gCmdqDriver = {
	.probe = cmdq_probe,
	.remove = cmdq_remove,
	.driver = {
		   .name = CMDQ_DRIVER_DEVICE_NAME,
		   .owner = THIS_MODULE,
		   .pm = &cmdq_pm_ops,
		   .of_match_table = cmdq_of_ids,
		   }
};

static int __init cmdq_init(void)
{
	int status;

	CMDQ_MSG("CMDQ driver init begin\n");

	/* Initialize group callback */
	cmdqCoreInitGroupCB();

	/* MDP function link */
	cmdq_mdp_virtual_function_setting();
	cmdq_mdp_platform_function_setting();

	/* Register MDP callback */
	cmdqCoreRegisterCB(CMDQ_GROUP_MDP,
			   cmdq_mdp_get_func()->mdpClockOn, cmdq_mdp_get_func()->mdpDumpInfo,
			   cmdq_mdp_get_func()->mdpResetEng, cmdq_mdp_get_func()->mdpClockOff);

	cmdqCoreRegisterErrorResetCB(CMDQ_GROUP_MDP,
			   cmdq_mdp_get_func()->errorReset);

	/* Register module dispatch callback */
	cmdqCoreRegisterDispatchModCB(CMDQ_GROUP_MDP,
			   cmdq_mdp_get_func()->dispatchModule);

	/* Register restore task */
	cmdqCoreRegisterTrackTaskCB(CMDQ_GROUP_MDP,
			   cmdq_mdp_get_func()->trackTask);

	cmdqCoreRegisterTaskCycleCB(CMDQ_GROUP_MDP,
		cmdq_mdp_get_func()->beginTask,
		cmdq_mdp_get_func()->endTask);

	cmdqCoreRegisterTaskCycleCB(CMDQ_GROUP_ISP,
		cmdq_mdp_get_func()->beginISPTask,
		cmdq_mdp_get_func()->endISPTask);

	/* Register VENC callback */
	cmdqCoreRegisterCB(CMDQ_GROUP_VENC, NULL, cmdq_mdp_get_func()->vEncDumpInfo, NULL, NULL);

	cmdqCoreRegisterMonitorTaskCB(CMDQ_GROUP_MDP,
		cmdq_mdp_get_func()->startTask_atomic, cmdq_mdp_get_func()->finishTask_atomic);

	status = platform_driver_register(&gCmdqDriver);
	if (status != 0) {
		CMDQ_ERR("Failed to register the CMDQ driver(%d)\n", status);
		return -ENODEV;
	}

	/* register pm notifier */
	status = register_pm_notifier(&cmdq_pm_notifier_block);
	if (status != 0) {
		CMDQ_ERR("Failed to register_pm_notifier(%d)\n", status);
		return -ENODEV;
	}

	CMDQ_MSG("CMDQ driver init end\n");

	return 0;
}

static void __exit cmdq_exit(void)
{
	int32_t status;

	CMDQ_MSG("CMDQ driver exit begin\n");

	device_destroy(gCMDQClass, gCmdqDevNo);

	class_destroy(gCMDQClass);

	cdev_del(gCmdqCDev);

	gCmdqCDev = NULL;

	unregister_chrdev_region(gCmdqDevNo, 1);

	platform_driver_unregister(&gCmdqDriver);

	/* register pm notifier */
	status = unregister_pm_notifier(&cmdq_pm_notifier_block);
	if (status != 0) {
		/* Failed to unregister_pm_notifier */
		CMDQ_ERR("Failed to unregister_pm_notifier(%d)\n", status);
	}

	/* Unregister MDP callback */
	cmdqCoreRegisterCB(CMDQ_GROUP_MDP, NULL, NULL, NULL, NULL);

	/* Unregister VENC callback */
	cmdqCoreRegisterCB(CMDQ_GROUP_VENC, NULL, NULL, NULL, NULL);

	/* De-Initialize group callback */
	cmdqCoreDeinitGroupCB();

	/* De-Initialize cmdq core */
	cmdqCoreDeInitialize();

	/* De-Initialize cmdq dev related data */
	cmdq_dev_deinit();
	mdp_limit_dev_destroy();

	CMDQ_MSG("CMDQ driver exit end\n");
}

static int __init cmdq_late_init(void)
{
	int status;

	CMDQ_MSG("CMDQ driver late init begin\n");

	status = cmdqCoreLateInitialize();
	status = mdp_limit_late_init();

	CMDQ_MSG("CMDQ driver late init end\n");

	return 0;
}

late_initcall(cmdq_late_init);

subsys_initcall(cmdq_init);
module_exit(cmdq_exit);

MODULE_DESCRIPTION("MTK CMDQ driver");
MODULE_AUTHOR("Pablo<pablo.sun@mediatek.com>");
MODULE_LICENSE("GPL");
