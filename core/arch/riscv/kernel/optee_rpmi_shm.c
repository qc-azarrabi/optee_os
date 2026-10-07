// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <assert.h>
#include <kernel/spinlock.h>
#include <optee_rpmi.h>
#include <rpmi_tee.h>
#include <stdlib.h>

#include "optee_rpmi_private.h"

/*
 * Parcel registry lifetime
 *
 * ACCEPTING reserves the creator/global-ID pair and an acceptance slot before
 * calling firmware. A successful acceptance and mapping changes it to LIVE.
 * Each LIVE entry owns one mapped MOBJ reference; borrowers take additional
 * references under ctx->lock and release them with mobj_put().
 *
 * Unregistration returns BUSY while acceptance, release or borrowers are
 * active. Otherwise it marks the entry RELEASING, drops the registry's mapping
 * reference and releases receiver interest in firmware. Success removes the
 * entry. A failed release retains it as FAILED, without a mapping, so a later
 * unregistration can retry cleanup rather than accepting the parcel again.
 *
 * Failed acceptance with acquired or uncertain interest uses the same release
 * path. An explicit initial rejection removes the entry without releasing it.
 * The acceptance slot is freed after the final successful segment or release;
 * uncertain incomplete acceptances retain it until cleanup succeeds.
 *
 * ctx->lock protects registry state, reference acquisition and slot accounting.
 * Firmware calls, mapping operations and mobj_put() run outside the lock.
 */

enum rpmi_shm_state {
	RPMI_SHM_ACCEPTING,
	RPMI_SHM_LIVE,
	RPMI_SHM_RELEASING,
	RPMI_SHM_FAILED,
};

struct rpmi_shm_entry {
	SLIST_ENTRY(rpmi_shm_entry) link;
	uint32_t creator_id;
	uint64_t global_id;
	enum rpmi_shm_state state;
	struct mobj *mobj;
	bool slot_held;
};

/* A logical SHM view keeps the cached whole-parcel mapping alive. */
struct rpmi_shm_view {
	struct mobj mobj;
	struct mobj *parent;
	size_t offset;
};

static struct rpmi_shm_view *to_view(struct mobj *mobj)
{
	return container_of(mobj, struct rpmi_shm_view, mobj);
}

static void *view_get_va(struct mobj *mobj, size_t offset, size_t len)
{
	struct rpmi_shm_view *view = to_view(mobj);

	if (!mobj_check_offset_and_len(mobj, offset, len))
		return NULL;
	return mobj_get_va(view->parent, view->offset + offset, len);
}

static TEE_Result view_get_pa(struct mobj *mobj, size_t offset,
			      size_t granule, paddr_t *pa)
{
	struct rpmi_shm_view *view = to_view(mobj);

	if (offset >= mobj->size)
		return TEE_ERROR_BAD_PARAMETERS;
	return mobj_get_pa(view->parent, view->offset + offset, granule, pa);
}

static size_t view_get_phys_offs(struct mobj *mobj, size_t granule)
{
	struct rpmi_shm_view *view = to_view(mobj);

	assert(granule && IS_POWER_OF_TWO(granule));
	return (mobj_get_phys_offs(view->parent, granule) + view->offset) &
	       (granule - 1);
}

static TEE_Result view_get_mem_type(struct mobj *mobj, uint32_t *type)
{
	return mobj_get_mem_type(to_view(mobj)->parent, type);
}

static bool view_matches(struct mobj *mobj, enum buf_is_attr attr)
{
	return mobj_matches(to_view(mobj)->parent, attr);
}

static uint64_t view_get_cookie(struct mobj *mobj)
{
	return mobj_get_cookie(to_view(mobj)->parent);
}

static void view_free(struct mobj *mobj)
{
	struct rpmi_shm_view *view = to_view(mobj);

	mobj_put(view->parent);
	free(view);
}

static const struct mobj_ops view_ops = {
	.get_va = view_get_va,
	.get_pa = view_get_pa,
	.get_phys_offs = view_get_phys_offs,
	.get_mem_type = view_get_mem_type,
	.matches = view_matches,
	.get_cookie = view_get_cookie,
	.free = view_free,
};

size_t rpmi_shm_view_offset(struct mobj *mobj)
{
	return mobj->ops == &view_ops ? to_view(mobj)->offset : 0;
}

struct mobj *rpmi_shm_view_alloc(struct mobj *parent, size_t offset,
				 size_t size)
{
	struct rpmi_shm_view *view = NULL;

	if (offset > parent->size || size > parent->size - offset)
		return NULL;
	if (!offset && size == parent->size)
		return parent;
	view = calloc(1, sizeof(*view));
	if (!view)
		return NULL;
	view->mobj.ops = &view_ops;
	view->mobj.size = size;
	view->mobj.phys_granule = parent->phys_granule;
	refcount_set(&view->mobj.refc, 1);
	view->parent = parent;
	view->offset = offset;
	return &view->mobj;
}

TEE_Result rpmi_shm_init(struct rpmi_shm_context *ctx, uint32_t self_id,
			 uint32_t multisegment_max, size_t max_pages)
{
	if (!max_pages || max_pages > UINT32_MAX ||
	    max_pages > SIZE_MAX / SMALL_PAGE_SIZE)
		return TEE_ERROR_BAD_PARAMETERS;

	*ctx = (struct rpmi_shm_context){
		.self_id = self_id,
		.multisegment_max = multisegment_max,
		.max_pages = max_pages,
		.lock = SPINLOCK_UNLOCK,
	};
	SLIST_INIT(&ctx->entries);

	return TEE_SUCCESS;
}

/* Called under ctx->lock; a key includes its authenticated creator. */
static struct rpmi_shm_entry *find_entry(struct rpmi_shm_context *ctx,
					 uint32_t creator_id,
					 uint64_t global_id)
{
	struct rpmi_shm_entry *entry = NULL;

	SLIST_FOREACH(entry, &ctx->entries, link)
		if (entry->creator_id == creator_id &&
		    entry->global_id == global_id)
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
				struct mbox_chan *channel,
				struct rpmi_shm_entry *entry)
{
	TEE_Result res = TEE_SUCCESS;
	uint32_t exceptions = 0;
	uint32_t parcel_id = optee_rpmi_parcel_id(entry->global_id);

	mobj_put(entry->mobj);
	entry->mobj = NULL;
	res = rpmi_tee_memory_release(channel, ctx->self_id, parcel_id);
	exceptions = cpu_spin_lock_xsave(&ctx->lock);
	if (res) {
		entry->state = RPMI_SHM_FAILED;
		cpu_spin_unlock_xrestore(&ctx->lock, exceptions);
		EMSG("Parcel %#"PRIx32" release failed: %#"PRIx32,
		     parcel_id, res);
		return res;
	}
	release_slot(ctx, entry);
	SLIST_REMOVE(&ctx->entries, entry, rpmi_shm_entry, link);
	cpu_spin_unlock_xrestore(&ctx->lock, exceptions);
	free(entry);
	return TEE_SUCCESS;
}

TEE_Result rpmi_shm_get(struct rpmi_shm_context *ctx,
			struct mbox_chan *channel,
			uint32_t creator_id, uint64_t global_id,
			struct mobj **mobj)
{
	struct rpmi_shm_entry *entry = NULL;
	struct rpmi_shm_entry *candidate = NULL;
	struct rpmi_tee_memory_accept_args args = {
		.creator_id = creator_id,
		.parcel_id = optee_rpmi_parcel_id(global_id),
		.nonce = optee_rpmi_nonce(global_id),
		.access = RPMI_TEE_ACCESS_READ | RPMI_TEE_ACCESS_WRITE,
		.creator_access = RPMI_TEE_ACCESS_READ | RPMI_TEE_ACCESS_WRITE,
	};
	paddr_t *pages = NULL;
	uint32_t exceptions = 0;
	TEE_Result res = TEE_SUCCESS;

	*mobj = NULL;
	if (!optee_rpmi_nonce(global_id))
		return TEE_ERROR_BAD_PARAMETERS;

	exceptions = cpu_spin_lock_xsave(&ctx->lock);
	entry = find_entry(ctx, creator_id, global_id);
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
	candidate->global_id = global_id;
	candidate->state = RPMI_SHM_ACCEPTING;

	exceptions = cpu_spin_lock_xsave(&ctx->lock);
	entry = find_entry(ctx, creator_id, global_id);
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

	args.acceptor_id = ctx->self_id;
	args.pages = pages;
	args.page_count_max = ctx->max_pages;
	res = rpmi_tee_memory_accept(channel, &args);
	if (!res) {
		exceptions = cpu_spin_lock_xsave(&ctx->lock);
		release_slot(ctx, entry);
		cpu_spin_unlock_xrestore(&ctx->lock, exceptions);
		entry->mobj = mobj_mapped_shm_alloc(pages, args.page_count, 0,
						    global_id);
		if (!entry->mobj)
			res = TEE_ERROR_OUT_OF_MEMORY;
	}
	if (!res) {
		exceptions = cpu_spin_lock_xsave(&ctx->lock);
		entry->state = RPMI_SHM_LIVE;
		*mobj = mobj_get(entry->mobj);
		cpu_spin_unlock_xrestore(&ctx->lock, exceptions);
	} else if (args.needs_release) {
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
			       struct mbox_chan *channel,
			       uint32_t creator_id, uint64_t global_id)
{
	struct rpmi_shm_entry *entry = NULL;
	uint32_t exceptions = 0;

	if (!optee_rpmi_nonce(global_id))
		return TEE_ERROR_BAD_PARAMETERS;
	exceptions = cpu_spin_lock_xsave(&ctx->lock);
	entry = find_entry(ctx, creator_id, global_id);
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
