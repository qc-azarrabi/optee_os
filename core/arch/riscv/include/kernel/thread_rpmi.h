/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __KERNEL_THREAD_RPMI_H
#define __KERNEL_THREAD_RPMI_H

#include <rpmi_reqfwd.h>
#include <kernel/rpmi_shm.h>
#include <optee_rpmi.h>

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
uint32_t optee_rpmi_caller(void);
void optee_rpmi_set_caller(uint32_t caller);
/* The initial backend serves one authenticated REE until secure OS restart. */
bool optee_rpmi_claim_caller(uint32_t caller);
#ifdef CFG_CORE_ASYNC_NOTIF
int32_t optee_rpmi_signal_bus_setup(void);
void optee_rpmi_signal_bus_teardown(void);
int32_t optee_rpmi_enable_async_notif(uint32_t peer, uint32_t signal);
void optee_rpmi_flush_async_notif(void);
#else
static inline int32_t optee_rpmi_signal_bus_setup(void)
{
	return RPMI_ERR_DENIED;
}

static inline void optee_rpmi_signal_bus_teardown(void)
{
}

static inline void optee_rpmi_flush_async_notif(void)
{
}
#endif
void optee_rpmi_set_call_response(int32_t status, uint32_t result,
				  uint64_t token);
void optee_rpmi_complete_and_loop(void) __noreturn;

TEE_Result thread_rpmi_handle_control(const void *data, size_t len,
				      void *response, size_t *response_len);
/* Called after the architecture has saved and suspended a secure thread. */
void thread_rpmi_suspend(uint32_t thread_id, uint32_t result) __noreturn;
/* Reset the temporary stack before entering completion and retrieval again. */
void thread_rpmi_return(void) __noreturn;

/* Platform overrides may replace the provisional DT channel association. */
TEE_Result optee_rpmi_get_channel_ids(uint32_t hart_id, uint32_t *reqfwd_id,
				      uint32_t *tee_id);

/* Enter on the temporary stack; retrieval waits in firmware when idle. */
void optee_rpmi_loop(void) __noreturn;
#endif

#endif /* __KERNEL_THREAD_RPMI_H */
