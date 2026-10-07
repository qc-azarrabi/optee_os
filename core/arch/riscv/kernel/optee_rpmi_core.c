// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <drivers/mailbox.h>
#include <initcall.h>
#include <kernel/dt.h>
#include <kernel/dt_driver.h>
#include <kernel/misc.h>
#include <kernel/panic.h>
#include <kernel/thread.h>
#include <rpmi_reqfwd.h>
#include <rpmi_tee.h>
#include <stdlib.h>

#include "optee_rpmi_private.h"

/*
 * OP-TEE RPMI backend state, forwarded-request processing and initialization.
 * This file is divided into the following sections:
 * 1. Backend state and caller tracking
 * 2. Forwarded-request processing
 * 3. DT probing and late activation
 */

/* 1. Backend state and caller tracking. */

static struct optee_rpmi_context context;

/* Each hart owns its request context and buffers through completion. */
static struct rpmi_hart *current_hart(void)
{
	size_t pos = get_core_pos();

	assert(pos < ARRAY_SIZE(context.core.harts));
	return context.core.harts + pos;
}

struct mbox_chan *optee_rpmi_channel(void)
{
	assert(context.core.initialized);
	return context.core.tee_channel;
}

struct rpmi_shm_context *optee_rpmi_shm_context(void)
{
	assert(context.core.initialized);
	return &context.shm;
}

TEE_Result mobj_rpmi_get_by_cookie(uint64_t global_id, struct mobj **mobj)
{
	return rpmi_shm_get(optee_rpmi_shm_context(), optee_rpmi_channel(),
			    thread_get_tsd()->rpmi_caller_id, global_id, mobj);
}

/* 2. Forwarded-request processing. */

static TEE_Result init_hart_buffer(struct rpmi_hart *hart)
{
	hart->request = malloc(CFG_RPMI_TEE_ABI_MAX_REQUEST_SIZE);
	if (!hart->request)
		return TEE_ERROR_OUT_OF_MEMORY;

	return TEE_SUCCESS;
}

static void complete_current(struct rpmi_hart *hart)
{
	uint32_t pending = 0;
	TEE_Result res = TEE_SUCCESS;

	res = rpmi_reqfwd_complete(context.core.reqfwd_channel, hart->response,
				   hart->response_len, &pending);
	/* An uncertain completion cannot be retried without duplicating it. */
	if (res)
		panic("Forwarded request completion failed");
}

/*
 * Retrieve and dispatch on the hart's temporary stack with no active secure
 * thread and all managed interrupts masked. Yielding calls switch to a secure
 * thread, where interrupts may be enabled. Native interrupts return to that
 * thread; RPC and foreign-interrupt yields complete the current forwarded
 * request on its owning hart before retrieval restarts. Resumption arrives
 * as a new forwarded request and may run the secure thread on another hart.
 */
void optee_rpmi_loop(void)
{
	struct optee_rpmi_core_state *core = &context.core;
	struct rpmi_hart *hart = current_hart();

	assert(core->initialized && thread_get_exceptions() == THREAD_EXCP_ALL);

	if (!hart->request && init_hart_buffer(hart))
		panic("Cannot allocate forwarded request buffer");

	while (true) {
		size_t request_len = 0;
		TEE_Result res = TEE_SUCCESS;

		hart->response_len = sizeof(hart->response);
		/* Firmware tracks the current request per calling hart. */
		res = rpmi_reqfwd_retrieve(core->reqfwd_channel, hart->request,
					   CFG_RPMI_TEE_ABI_MAX_REQUEST_SIZE,
					   &request_len);
		/* Idle retrieval must block in firmware, not return NO_DATA. */
		if (res)
			panic("Forwarded request retrieval failed");
		res = optee_rpmi_dispatch(context.shm.self_id, hart->request,
					  request_len, hart->response,
					  &hart->response_len);
		if (res)
			panic("Cannot encode forwarded response");
		complete_current(hart);
	}
}

/* 3. DT probing and late activation. */

static TEE_Result check_channels(struct mbox_chan *reqfwd,
				 struct mbox_chan *tee)
{
	uint32_t group = 0;
	uint32_t max_msg_data_size = 0;
	TEE_Result res = TEE_SUCCESS;

	res = rpmi_mbox_get_attribute(reqfwd, RPMI_MBOX_ATTR_SERVICEGROUP_ID,
				      &group);
	if (res)
		return res;
	if (group != RPMI_SRVGRP_REQUEST_FORWARD)
		return TEE_ERROR_BAD_PARAMETERS;
	res = rpmi_mbox_get_attribute(tee, RPMI_MBOX_ATTR_SERVICEGROUP_ID,
				      &group);
	if (res)
		return res;
	if (group != RPMI_TEE_SERVICEGROUP_ID)
		return TEE_ERROR_BAD_PARAMETERS;
	res = rpmi_mbox_get_attribute(reqfwd, RPMI_MBOX_ATTR_MAX_MSG_DATA_SIZE,
				      &max_msg_data_size);
	if (res)
		return res;
	if (max_msg_data_size < sizeof(context.core.harts[0].response))
		return TEE_ERROR_NOT_SUPPORTED;

	return TEE_SUCCESS;
}

/* Prepare the single backend during DT probing, before late activation. */
static TEE_Result optee_rpmi_probe_channels(struct mbox_chan *reqfwd,
					    struct mbox_chan *tee)
{
	struct optee_rpmi_core_state *core = &context.core;
	struct rpmi_hart *hart = current_hart();
	uint32_t domain_id = 0;
	uint32_t ep_id = 0;
	uint32_t share = 0;
	uint32_t multisegment = 0;
	TEE_Result res = TEE_SUCCESS;

	if (core->probed)
		return TEE_ERROR_BAD_STATE;
	res = check_channels(reqfwd, tee);
	if (res)
		return res;
	res = rpmi_tee_probe_system(tee, &domain_id, &ep_id);
	if (res)
		return res;
	res = rpmi_tee_probe_features(tee, RPMI_TEE_FEATURE_MEMORY_SHARE,
				      &share);
	if (res)
		return res;
	if (share != RPMI_TEE_MEMORY_FULLY_SUPPORTED)
		return TEE_ERROR_NOT_SUPPORTED;
	res = rpmi_tee_probe_features(tee, RPMI_TEE_FEATURE_MULTISEGMENT_OPS,
				      &multisegment);
	if (res)
		return res;

	res = rpmi_shm_init(&context.shm, ep_id, multisegment,
			    CFG_RPMI_TEE_ABI_MAX_PARCEL_PAGES);
	if (res)
		return res;
	res = init_hart_buffer(hart);
	if (res)
		return res;
	core->reqfwd_channel = reqfwd;
	core->tee_channel = tee;
	core->probed = true;
	return TEE_SUCCESS;
}

static TEE_Result optee_rpmi_probe(const void *fdt, int node,
				   const void *compat_data __unused)
{
	struct optee_rpmi_core_state *core = &context.core;
	struct mbox_chan *reqfwd_chan = NULL;
	struct mbox_chan *tee_chan = NULL;
	TEE_Result res = TEE_SUCCESS;

	if (!(fdt_get_status(fdt, node) & DT_STATUS_OK_SEC))
		return TEE_ERROR_NODE_DISABLED;

	res = mbox_request_channel_byname(fdt, node, "reqfwd", &core->client,
					  &reqfwd_chan);
	if (res)
		return res;
	res = mbox_request_channel_byname(fdt, node, "tee", &core->client,
					  &tee_chan);
	if (res)
		goto err_reqfwd;
	res = optee_rpmi_probe_channels(reqfwd_chan, tee_chan);
	if (!res)
		return TEE_SUCCESS;

	mbox_free_channel(tee_chan);
err_reqfwd:
	mbox_free_channel(reqfwd_chan);
	return res;
}

/* The consumer compatible is provisional pending a secure-side DT binding. */
static const struct dt_device_match optee_rpmi_match_table[] = {
	{ .compatible = "linaro,optee-rpmi" },
	{ }
};

DEFINE_DT_DRIVER(optee_rpmi_driver) = {
	.name = "optee-rpmi",
	.match_table = optee_rpmi_match_table,
	.probe = optee_rpmi_probe,
};

/* Activate only after DT dependencies and the common services are ready. */
static TEE_Result optee_rpmi_activate(void)
{
	struct optee_rpmi_core_state *core = &context.core;

	if (!core->probed)
		panic("OP-TEE RPMI backend was not probed");
	core->initialized = true;
	return TEE_SUCCESS;
}
driver_init_late(optee_rpmi_activate);
