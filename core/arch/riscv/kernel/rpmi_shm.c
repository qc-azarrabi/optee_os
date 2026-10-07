// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <assert.h>
#include <kernel/rpmi_shm.h>
#include <kernel/spinlock.h>
#include <stdlib.h>

enum rpmi_shm_state {
	RPMI_SHM_ACCEPTING,
	RPMI_SHM_LIVE,
	RPMI_SHM_RELEASING,
	RPMI_SHM_FAILED,
};

struct rpmi_shm_entry {
	SLIST_ENTRY(rpmi_shm_entry) link;
	uint32_t creator_id;
	uint32_t parcel_id;
	uint32_t nonce;
	enum rpmi_shm_state state;
	struct mobj *mobj;
	bool slot_held;
};

void rpmi_shm_init(struct rpmi_shm_context *ctx, uint32_t self_id,
		   uint32_t multisegment_max, size_t max_pages)
{
	*ctx = (struct rpmi_shm_context){
		.self_id = self_id,
		.multisegment_max = multisegment_max,
		.max_pages = max_pages,
		.lock = SPINLOCK_UNLOCK,
	};
	SLIST_INIT(&ctx->entries);
}

/* Called under ctx->lock; a key includes its authenticated creator. */
static struct rpmi_shm_entry *find_entry(struct rpmi_shm_context *ctx,
					 uint32_t creator_id,
					 uint32_t parcel_id,
					 uint32_t nonce)
{
	struct rpmi_shm_entry *entry = NULL;

	SLIST_FOREACH(entry, &ctx->entries, link)
		if (entry->creator_id == creator_id &&
		    entry->parcel_id == parcel_id && entry->nonce == nonce)
			return entry;
	return NULL;
}

/* A successful final segment or release frees the firmware acceptance slot. */
static void release_slot(struct rpmi_shm_context *ctx,
			 struct rpmi_shm_entry *entry)
{
	if (entry->slot_held) {
		assert(ctx->pending);
		ctx->pending--;
		entry->slot_held = false;
	}
}

/* Preserve failed cleanup; this entry is exclusively owned by the caller. */
static TEE_Result release_entry(struct rpmi_shm_context *ctx,
				struct sbi_mpxy_rpmi_channel *channel,
				struct rpmi_shm_entry *entry)
{
	int32_t status = RPMI_ERR_FAILED;
	TEE_Result res = TEE_SUCCESS;
	uint32_t exceptions = 0;
	uint32_t parcel_id __maybe_unused = entry->parcel_id;

	mobj_put(entry->mobj);
	entry->mobj = NULL;
	res = rpmi_tee_memory_release(channel, ctx->self_id, entry->parcel_id,
				      &status);
	exceptions = cpu_spin_lock_xsave(&ctx->lock);
	if (res || status != RPMI_SUCCESS) {
		entry->state = RPMI_SHM_FAILED;
		cpu_spin_unlock_xrestore(&ctx->lock, exceptions);
		EMSG("Parcel %#"PRIx32" release failed: local=%#"PRIx32
		     " status=%"PRId32, parcel_id, res, status);
		return res ? res : TEE_ERROR_COMMUNICATION;
	}
	release_slot(ctx, entry);
	SLIST_REMOVE(&ctx->entries, entry, rpmi_shm_entry, link);
	cpu_spin_unlock_xrestore(&ctx->lock, exceptions);
	free(entry);
	return TEE_SUCCESS;
}

TEE_Result rpmi_shm_get(struct rpmi_shm_context *ctx,
			struct sbi_mpxy_rpmi_channel *channel,
			uint32_t creator_id, uint32_t parcel_id, uint32_t nonce,
			struct mobj **mobj)
{
	struct rpmi_shm_entry *entry = NULL;
	struct rpmi_shm_entry *candidate = NULL;
	paddr_t *pages = NULL;
	size_t count = 0;
	uint32_t exceptions = 0;
	int32_t status = RPMI_ERR_FAILED;
	TEE_Result res = TEE_SUCCESS;
	bool needs_release = false;

	if (!ctx || !channel || !mobj || !ctx->max_pages ||
	    ctx->max_pages > UINT32_MAX ||
	    ctx->max_pages > SIZE_MAX / SMALL_PAGE_SIZE ||
	    ctx->max_pages > SIZE_MAX / sizeof(*pages))
		return TEE_ERROR_BAD_PARAMETERS;
	*mobj = NULL;
	if (!parcel_id && !nonce)
		return TEE_ERROR_BAD_PARAMETERS;

	exceptions = cpu_spin_lock_xsave(&ctx->lock);
	entry = find_entry(ctx, creator_id, parcel_id, nonce);
	if (entry) {
		if (entry->state == RPMI_SHM_LIVE)
			*mobj = mobj_get(entry->mobj);
		cpu_spin_unlock_xrestore(&ctx->lock, exceptions);
		return *mobj ? TEE_SUCCESS : TEE_ERROR_BUSY;
	}
	cpu_spin_unlock_xrestore(&ctx->lock, exceptions);

	candidate = calloc(1, sizeof(*candidate));
	pages = calloc(ctx->max_pages, sizeof(*pages));
	if (!candidate || !pages) {
		res = TEE_ERROR_OUT_OF_MEMORY;
		goto out;
	}
	candidate->creator_id = creator_id;
	candidate->parcel_id = parcel_id;
	candidate->nonce = nonce;
	candidate->state = RPMI_SHM_ACCEPTING;

	exceptions = cpu_spin_lock_xsave(&ctx->lock);
	entry = find_entry(ctx, creator_id, parcel_id, nonce);
	if (entry) {
		if (entry->state == RPMI_SHM_LIVE)
			*mobj = mobj_get(entry->mobj);
		res = *mobj ? TEE_SUCCESS : TEE_ERROR_BUSY;
		cpu_spin_unlock_xrestore(&ctx->lock, exceptions);
		goto out;
	}
	/* Without segmented support, serialize possible acceptances. */
	if (ctx->pending >= MAX(ctx->multisegment_max, 1U)) {
		res = TEE_ERROR_BUSY;
		cpu_spin_unlock_xrestore(&ctx->lock, exceptions);
		goto out;
	}
	ctx->pending++;
	candidate->slot_held = true;
	SLIST_INSERT_HEAD(&ctx->entries, candidate, link);
	entry = candidate;
	candidate = NULL;
	cpu_spin_unlock_xrestore(&ctx->lock, exceptions);

	res = rpmi_tee_memory_accept(channel, ctx->self_id, creator_id,
				     parcel_id, nonce, pages, ctx->max_pages,
				     &count, &needs_release, &status);
	if (!res && status == RPMI_SUCCESS) {
		exceptions = cpu_spin_lock_xsave(&ctx->lock);
		release_slot(ctx, entry);
		cpu_spin_unlock_xrestore(&ctx->lock, exceptions);
		entry->mobj = mobj_mapped_shm_alloc(pages, count, 0,
						    ((uint64_t)nonce << 32) |
						    parcel_id);
		if (!entry->mobj)
			res = TEE_ERROR_OUT_OF_MEMORY;
	} else if (!res) {
		res = status == RPMI_ERR_BUSY ? TEE_ERROR_BUSY :
			TEE_ERROR_BAD_PARAMETERS;
	}
	if (!res) {
		exceptions = cpu_spin_lock_xsave(&ctx->lock);
		entry->state = RPMI_SHM_LIVE;
		*mobj = mobj_get(entry->mobj);
		cpu_spin_unlock_xrestore(&ctx->lock, exceptions);
	} else if (needs_release) {
		/* A failed release retains the entry and uncertain slot. */
		release_entry(ctx, channel, entry);
	} else {
		exceptions = cpu_spin_lock_xsave(&ctx->lock);
		release_slot(ctx, entry);
		SLIST_REMOVE(&ctx->entries, entry, rpmi_shm_entry, link);
		cpu_spin_unlock_xrestore(&ctx->lock, exceptions);
		free(entry);
	}
out:
	free(candidate);
	free(pages);
	return res;
}

TEE_Result rpmi_shm_unregister(struct rpmi_shm_context *ctx,
			       struct sbi_mpxy_rpmi_channel *channel,
			       uint32_t creator_id, uint32_t parcel_id,
			       uint32_t nonce)
{
	struct rpmi_shm_entry *entry = NULL;
	uint32_t exceptions = 0;

	if (!ctx || !channel || (!parcel_id && !nonce))
		return TEE_ERROR_BAD_PARAMETERS;
	exceptions = cpu_spin_lock_xsave(&ctx->lock);
	entry = find_entry(ctx, creator_id, parcel_id, nonce);
	if (!entry) {
		/* An unseen key has no lazily acquired receiver interest. */
		cpu_spin_unlock_xrestore(&ctx->lock, exceptions);
		return TEE_SUCCESS;
	}
	if (entry->state == RPMI_SHM_ACCEPTING ||
	    entry->state == RPMI_SHM_RELEASING ||
	    (entry->mobj && refcount_val(&entry->mobj->refc) != 1)) {
		cpu_spin_unlock_xrestore(&ctx->lock, exceptions);
		return TEE_ERROR_BUSY;
	}
	entry->state = RPMI_SHM_RELEASING;
	cpu_spin_unlock_xrestore(&ctx->lock, exceptions);
	return release_entry(ctx, channel, entry);
}
