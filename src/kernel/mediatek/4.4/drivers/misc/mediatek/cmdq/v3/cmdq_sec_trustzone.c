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

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/slab.h>

#include "cmdq_core.h"
#include "cmdq_sec_trustzone.h"
#include "tz_cross/trustzone.h"

void cmdq_sec_setup_tee_context(struct cmdq_sec_tee_context *tee)
{
	memset(tee, 0, sizeof(struct cmdq_sec_tee_context));
	/* "5c071864-505d-11e4-9e35-164230d1df67" */
	memset(tee->uuid, 0, sizeof(tee->uuid));
	snprintf(tee->uuid, sizeof(tee->uuid), "%s", "5c071864-505d-11e4-9e35-164230d1df67");
}

s32 cmdq_sec_init_context(struct cmdq_sec_tee_context *tee)
{
	return 0;
}

s32 cmdq_sec_deinit_context(struct cmdq_sec_tee_context *tee)
{
	return 0;
}

/* allocate share memory for communicate with trustzone */
s32 cmdq_sec_allocate_wsm(struct cmdq_sec_tee_context *tee,
	void **wsm_buffer, u32 size)
{
	s32 status = 0;
#if 0
	TZ_RESULT tzRes;
	struct KREE_SHAREDMEM_PARAM cmdq_shared_param;
	union MTEEC_PARAM cmdq_param[4];
	unsigned int paramTypes;
	KREE_SHAREDMEM_HANDLE cmdq_share_handle = 0;
#endif

	if (!wsm_buffer)
		return -EINVAL;

	/* because world shared mem(WSM) will ba managed by mobicore device,
	 * instead of linux kernel vmalloc/kmalloc, call mc_malloc_wasm to
	 * alloc WSM to prvent error such as "can not resolve tci physicall
	 * address" etc
	 */
	tee->share_memory = kmalloc(sizeof(struct iwcCmdqMessage_t), GFP_KERNEL);
	if (!tee->share_memory) {
		CMDQ_ERR("share memory kmalloc failed!\n");
		return -ENOMEM;
	}

	*wsm_buffer = tee->share_memory;

#if 0
	/* init share memory */
	cmdq_shared_param.buffer = gCmdqContext.hSecSharedMem->pVABase;
	cmdq_shared_param.size = gCmdqContext.hSecSharedMem->size;
	tzRes = KREE_RegisterSharedmem(tee->mem_session,
		&cmdq_share_handle,
		&cmdq_shared_param);

	/* save for unregister */
	gCmdqContext.hSecSharedMem->cmdq_share_cookie_handle = cmdq_share_handle;
	if (tzRes != TZ_RESULT_SUCCESS) {
		CMDQ_ERR
		    ("cmdq register share memory Error: %d, line:%d, cmdq_mem_session_handle(%x)\n",
		     tzRes, __LINE__, (unsigned int)(handle->memSessionHandle));
		return tzRes;
	}
	/* KREE_Tee service call */
	cmdq_param[0].memref.handle = (uint32_t) cmdq_share_handle;
	cmdq_param[0].memref.offset = 0;
	cmdq_param[0].memref.size = cmdq_shared_param.size;
	paramTypes = TZ_ParamTypes1(TZPT_MEMREF_INOUT);
	tzRes =
	    KREE_TeeServiceCall(handle->sessionHandle,
				CMD_CMDQ_TL_INIT_SHARED_MEMORY, paramTypes,
				cmdq_param);
	if (tzRes != TZ_RESULT_SUCCESS) {
		CMDQ_ERR("CMD_CMDQ_TL_INIT_SHARED_MEMORY fail, ret=0x%x\n", tzRes);
		return tzRes;
	}
	CMDQ_MSG("KREE_TeeServiceCall tzRes =0x%x\n", tzRes);
#endif
	return status;
}

/* free share memory  */
s32 cmdq_sec_free_wsm(struct cmdq_sec_tee_context *tee,
	void **wsm_buffer)
{
	if (wsm_buffer && *wsm_buffer) {
		kfree(*wsm_buffer);
		*wsm_buffer = NULL;
		tee->share_memory = NULL;
	}

	return 0;
}


/* create cmdq session & memory session */
s32 cmdq_sec_open_session(struct cmdq_sec_tee_context *tee,
	void *wsm_buffer)
{
#if defined(CMDQ_SECURE_PATH_SUPPORT)
	if (!tee->session) {
		TZ_RESULT ret;

		CMDQ_LOG("TZ_TA_CMDQ_UUID:%s\n", tee->uuid);
		ret = KREE_CreateSession(tee->uuid, &tee->session);
		if (ret != TZ_RESULT_SUCCESS) {
			CMDQ_ERR("%s failed to create cmdq session, ret=%d\n",
				__func__,
				ret);
			return 0;
		}
	}

	if (!tee->mem_session) {
		TZ_RESULT ret;

		ret = KREE_CreateSession(TZ_TA_MEM_UUID, &tee->mem_session);
		if (ret != TZ_RESULT_SUCCESS) {
			CMDQ_ERR("%s failed to create memory session, ret=%d\n",
				__func__,
				ret);
			return 0;
		}
	}

	tee->share_memory = wsm_buffer;

#else
		CMDQ_ERR("SVP feature is not on\n");
		return 0;
#endif
	return 0;
}

/* close cmdq & memory session */
s32 cmdq_sec_close_session(struct cmdq_sec_tee_context *tee)
{
	TZ_RESULT ret;

	ret = KREE_CloseSession(tee->session);
	if (ret != TZ_RESULT_SUCCESS) {
		CMDQ_ERR("DDP close ddp_session fail ret=%d\n", ret);
		return 0;
	}

	ret = KREE_CloseSession(tee->session);
	if (ret != TZ_RESULT_SUCCESS) {
		CMDQ_ERR("DDP close ddp_session fail ret=%d\n", ret);
		return 0;
	}
	return 0;
}

s32 cmdq_sec_execute_session(struct cmdq_sec_tee_context *tee,
	u32 cmd, s32 timeout_ms)
{
	TZ_RESULT tzRes;

	do {
		/* Register share memory */
		union MTEEC_PARAM cmdq_param[4];
		unsigned int paramTypes;
		KREE_SHAREDMEM_HANDLE cmdq_share_handle = 0;
		struct KREE_SHAREDMEM_PARAM cmdq_shared_param;

		cmdq_shared_param.buffer = tee->share_memory;
		cmdq_shared_param.size = (sizeof(struct iwcCmdqMessage_t));

#if 0				/* add for debug */
		CMDQ_ERR("dump secure task instructions in Normal world\n");
		cmdq_core_dump_instructions((uint64_t
						 *) (((iwcCmdqMessage_t *) (handle->
									iwcMessage))->command.
						 pVABase),
						((iwcCmdqMessage_t *) (handle->iwcMessage))->
						command.commandSize);
#endif

		tzRes =	KREE_RegisterSharedmem(tee->mem_session,
			&cmdq_share_handle,
			&cmdq_shared_param);
		if (tzRes != TZ_RESULT_SUCCESS) {
			CMDQ_ERR("register share memory Error: %d, mem_session(%x)\n",
				 tzRes, (unsigned int)(tee->mem_session));
			return tzRes;
		}

		/* KREE_Tee service call */
		cmdq_param[0].memref.handle = (uint32_t) cmdq_share_handle;
		cmdq_param[0].memref.offset = 0;
		cmdq_param[0].memref.size = cmdq_shared_param.size;
		paramTypes = TZ_ParamTypes1(TZPT_MEMREF_INPUT);

		CMDQ_MSG("start to enter Secure World\n");
		tzRes =	KREE_TeeServiceCall(tee->session,
					tee->share_memory->cmd,
					paramTypes,
					cmdq_param);
		if (tzRes != TZ_RESULT_SUCCESS) {
			CMDQ_ERR("leave secure world KREE_TeeServiceCall fail, ret=0x%x\n", tzRes);
			return tzRes;
		}
		CMDQ_MSG("leave secure world KREE_TeeServiceCall tzRes =0x%x\n", tzRes);


		/* Unregister share memory */
		KREE_UnregisterSharedmem(tee->mem_session, cmdq_share_handle);
	} while (0);

	CMDQ_PROF_END("CMDQ_SEC_EXE");

	/* return tee service call result */
	return tzRes;
}


