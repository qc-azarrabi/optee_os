/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __KERNEL_THREAD_RPMI_H
#define __KERNEL_THREAD_RPMI_H

#include <rpmi_reqfwd.h>
#include <kernel/rpmi_shm.h>

/*
 * Dispatch a decoded request and construct its complete service response data.
 * resp_len is the supplied capacity, replaced by the actual output length.
 * Firmware must authenticate the TEE_CALL sender before forwarding it.
 */
TEE_Result optee_rpmi_dispatch(uint32_t self_id,
			       const struct rpmi_reqfwd_request *request,
			       void *resp, size_t *resp_len);

#ifdef CFG_CORE_RPMI
/* Bind explicit per-hart channels before publishing the endpoint registry. */
void optee_rpmi_init_primary(void);
struct sbi_mpxy_rpmi_channel *optee_rpmi_channel(void);
struct rpmi_shm_context *optee_rpmi_shm_context(void);

/* Platform overrides may replace the provisional DT channel association. */
TEE_Result optee_rpmi_get_channel_ids(uint32_t hart_id, uint32_t *reqfwd_id,
				      uint32_t *tee_id);

/* Enter on the temporary stack; retrieval waits in firmware when idle. */
void optee_rpmi_loop(void) __noreturn;
#endif

#endif /* __KERNEL_THREAD_RPMI_H */
