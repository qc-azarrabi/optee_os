/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __KERNEL_OPTEE_RPMI_PRIVATE_H
#define __KERNEL_OPTEE_RPMI_PRIVATE_H

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
};

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

#endif /* __KERNEL_OPTEE_RPMI_PRIVATE_H */
