/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __KERNEL_RPMI_SHM_H
#define __KERNEL_RPMI_SHM_H

#include <mm/mobj.h>
#include <rpmi_tee.h>
#include <sys/queue.h>

struct rpmi_shm_entry;
SLIST_HEAD(rpmi_shm_head, rpmi_shm_entry);

/*
 * An endpoint's parcel registry. lock protects entries, their state and the
 * pending acceptance count. No firmware call, mapping operation or mobj_put()
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

/* Initialize before secondary harts or service operations can use ctx. */
void rpmi_shm_init(struct rpmi_shm_context *ctx, uint32_t self_id,
		   uint32_t multisegment_max, size_t max_pages);

/* Get a mapped reference, accepting an unseen RW SHARE parcel if necessary. */
TEE_Result rpmi_shm_get(struct rpmi_shm_context *ctx,
			struct sbi_mpxy_rpmi_channel *channel,
			uint32_t creator_id, uint32_t parcel_id, uint32_t nonce,
			struct mobj **mobj);

/*
 * Retire an idle parcel. BUSY leaves an active mapping unchanged. Unmap before
 * releasing receiver interest; retain failed release records for retry and do
 * not permit a fresh acceptance to hide an outstanding cleanup failure.
 */
TEE_Result rpmi_shm_unregister(struct rpmi_shm_context *ctx,
			       struct sbi_mpxy_rpmi_channel *channel,
			       uint32_t creator_id, uint32_t parcel_id,
			       uint32_t nonce);

#endif /* __KERNEL_RPMI_SHM_H */
