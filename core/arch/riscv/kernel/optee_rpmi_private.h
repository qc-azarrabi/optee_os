/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __KERNEL_OPTEE_RPMI_PRIVATE_H
#define __KERNEL_OPTEE_RPMI_PRIVATE_H

#include <drivers/mailbox.h>
#include <kernel/notif.h>
#include <mm/mobj.h>
#include <optee_rpmi.h>
#include <rpmi.h>
#include <rpmi_tee.h>
#include <stddef.h>
#include <sys/queue.h>
#include <tee_api_types.h>

/* Shared-memory parcel registry. */
struct rpmi_shm_entry;
SLIST_HEAD(rpmi_shm_head, rpmi_shm_entry);

/*
 * An endpoint's parcel registry, keyed by creator ID and opaque global ID.
 * lock protects entries, their state and the pending acceptance count.
 * No firmware call, mapping operation or mobj_put()
 * runs with this lock held. Each LIVE entry owns one mapped MOBJ reference;
 * borrowers acquire references under lock, so retirement cannot race lookup.
 * Initialize immutable identity and limits before publishing the registry.
 */
struct rpmi_shm_context {
	struct rpmi_shm_head entries;
	unsigned int lock;
	uint32_t self_id;
	uint32_t multisegment_max;
	size_t max_pages;
	size_t pending;
};

/* Validate fixed limits and initialize before publishing ctx to users. */
TEE_Result rpmi_shm_init(struct rpmi_shm_context *ctx, uint32_t self_id,
			 uint32_t multisegment_max, size_t max_pages);

/*
 * Get a mapped reference, accepting an unseen RW SHARE parcel if necessary.
 * The caller supplies an initialized ctx, a valid channel and non-NULL mobj.
 */
TEE_Result rpmi_shm_get(struct rpmi_shm_context *ctx,
			struct mbox_chan *channel,
			uint32_t creator_id, uint64_t global_id,
			struct mobj **mobj);

/*
 * Allocate a logical view of mapped SHM with a non-NULL parent. Success
 * transfers the caller's parent reference to the returned object; failure
 * leaves that reference unchanged.
 * An unchanged full-range view returns parent itself without allocating.
 */
struct mobj *rpmi_shm_view_alloc(struct mobj *parent, size_t offset,
				 size_t size);

/* Return a logical view's byte offset within its parent parcel. */
size_t rpmi_shm_view_offset(struct mobj *mobj);

/*
 * Retire an idle parcel. BUSY leaves an active mapping unchanged. Unmap before
 * releasing receiver interest; retain failed release records for retry and do
 * not permit a fresh acceptance to hide an outstanding cleanup failure.
 * The caller supplies an initialized ctx and a valid channel.
 */
TEE_Result rpmi_shm_unregister(struct rpmi_shm_context *ctx,
			       struct mbox_chan *channel,
			       uint32_t creator_id, uint64_t global_id);

/* Supported OP-TEE control responses, excluding the TEE_CALL header. */
union optee_rpmi_control_response {
	struct optee_rpmi_status_resp status;
	struct optee_rpmi_api_resp api;
	struct optee_rpmi_os_resp os;
	struct optee_rpmi_caps_resp caps;
	struct optee_rpmi_call_resp call;
};

/* Per-hart request and response buffers. */
struct rpmi_hart {
	uint8_t *request;
	uint8_t response[RPMI_TEE_CALL_RESP_HEADER_SIZE +
			 sizeof(union optee_rpmi_control_response)];
	size_t response_len;
};

/* Immutable after activation except for per-hart data and locked peer state. */
struct optee_rpmi_core_state {
	struct rpmi_hart harts[CFG_TEE_CORE_NB_CORE];
	struct mbox_client client;
	struct mbox_chan *reqfwd_channel;
	struct mbox_chan *tee_channel;
	bool probed;
	bool initialized;
	uint32_t signal_bus_feature;
	/* Protects peer_id and peer_bound. */
	unsigned int peer_lock;
	uint32_t peer_id;
	bool peer_bound;
};

/* Asynchronous notification state. */
#ifdef CFG_CORE_ASYNC_NOTIF
/* lock protects the bus, binding, pending doorbell and in-flight raises. */
struct optee_rpmi_notif_state {
	unsigned int lock;
	uint32_t peer_id;
	uint32_t signal_id;
	unsigned int in_flight;
	bool enabled;
	bool pending;
	unsigned int stopping;
	bool stopped;
	bool bus_active;
	struct notif_driver driver;
};
#endif

/* One domain-wide backend; each state group retains its own synchronization. */
struct optee_rpmi_context {
	struct optee_rpmi_core_state core;
	struct rpmi_shm_context shm;
#ifdef CFG_CORE_ASYNC_NOTIF
	struct optee_rpmi_notif_state notif;
#endif
};

/* Available during probing and activation, before runtime readiness checks. */
struct optee_rpmi_context *optee_rpmi_get_context(void);

struct mbox_chan *optee_rpmi_channel(void);
struct rpmi_shm_context *optee_rpmi_shm_context(void);
/* The initial backend serves one authenticated REE until secure OS restart. */
bool optee_rpmi_claim_caller(uint32_t caller);
bool optee_rpmi_check_caller(uint32_t caller);

/* Forwarded-request dispatch and completion. */
/*
 * Decode an RPMI packet and construct its complete service response data.
 * The caller supplies non-NULL data, resp and resp_len pointers.
 * Malformed packets receive RPMI_ERR_INVALID_PARAM for normal completion.
 * resp_len is the supplied buffer size, replaced by the actual output length.
 * Firmware must authenticate the TEE_CALL sender before forwarding it.
 */
TEE_Result optee_rpmi_dispatch(uint32_t self_id,
			       const void *data, size_t data_len,
			       void *resp, size_t *resp_len);

void optee_rpmi_set_call_response(TEE_Result status, uint32_t result,
				  uint32_t token);
void optee_rpmi_complete_and_loop(void) __noreturn;

/* Enter on the temporary stack; retrieval waits in firmware when idle. */
void optee_rpmi_loop(void) __noreturn;

/* Yielding-call control and architecture entry points. */
TEE_Result thread_rpmi_handle_control(uint32_t caller, uint32_t op,
				      const void *data, size_t len,
				      struct optee_rpmi_call_resp *resp,
				      size_t *response_len);
/* Called after the architecture has saved and suspended a secure thread. */
void thread_rpmi_suspend(uint32_t thread_id, uint32_t result) __noreturn;
/* Reset the temporary stack before entering completion and retrieval again. */
void thread_rpmi_return(void) __noreturn;

/* Asynchronous notification operations and disabled-feature stubs. */
#ifdef CFG_CORE_ASYNC_NOTIF
void notif_rpmi_init(void);
int32_t optee_rpmi_signal_bus_setup(void);
void optee_rpmi_signal_bus_teardown(void);
TEE_Result optee_rpmi_enable_async_notif(uint32_t peer, uint32_t signal);
void optee_rpmi_flush_async_notif(void);
#else
static inline void notif_rpmi_init(void)
{
}

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

#endif /* __KERNEL_OPTEE_RPMI_PRIVATE_H */
