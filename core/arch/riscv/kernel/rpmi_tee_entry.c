// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <io.h>
#include <kernel/misc.h>
#include <kernel/panic.h>
#include <kernel/thread.h>
#include <kernel/thread_rpmi.h>
#include <rpmi_tee.h>
#include <stdlib.h>

struct rpmi_hart {
	struct sbi_mpxy_rpmi_channel *reqfwd;
	struct sbi_mpxy_rpmi_channel *tee;
	uint8_t *request;
	uint8_t response[32];
};

static struct rpmi_hart harts[CFG_TEE_CORE_NB_CORE];
static struct rpmi_shm_context shm_context;
static bool initialized;

/* Each hart exclusively owns its queue and buffers through completion. */
static struct rpmi_hart *current_hart(void)
{
	size_t pos = get_core_pos();

	assert(pos < ARRAY_SIZE(harts));
	return harts + pos;
}

static void bind_hart(struct rpmi_hart *hart)
{
	uint32_t reqfwd_id = 0;
	uint32_t tee_id = 0;

	if (optee_rpmi_get_channel_ids(thread_get_hartid(), &reqfwd_id,
				       &tee_id))
		panic("Missing RPMI hart channel association");
	hart->reqfwd = sbi_mpxy_rpmi_get_channel(reqfwd_id,
						 RPMI_SRVGRP_REQUEST_FORWARD);
	hart->tee = sbi_mpxy_rpmi_get_channel(tee_id, RPMI_TEE_SERVICEGROUP_ID);
	if (!hart->reqfwd || !hart->tee)
		panic("Missing RPMI hart channels");
	if (!(hart->reqfwd->attrs.capability &
	      SBI_MPXY_CHAN_CAP_SEND_WITH_RESP) ||
	    hart->reqfwd->attrs.msg_max_len < sizeof(hart->response))
		panic("Unsupported RPMI hart channels");
	hart->request = malloc(CFG_CORE_RPMI_MAX_REQUEST_SIZE);
	if (!hart->request)
		panic("Cannot allocate forwarded request buffer");
}

void optee_rpmi_init_primary(void)
{
	struct rpmi_hart *hart = current_hart();
	uint32_t domain_id = 0;
	uint32_t ep_id = 0;
	uint32_t share = 0;
	uint32_t multisegment = 0;
	int32_t status = RPMI_ERR_FAILED;

	bind_hart(hart);
	if (rpmi_tee_probe_system(hart->tee, &domain_id, &ep_id, &status) ||
	    status != RPMI_SUCCESS)
		panic("Cannot query RPMI endpoint identity");
	if (rpmi_tee_probe_features(hart->tee, RPMI_TEE_FEATURE_MEMORY_SHARE,
				    &share, &status) ||
	    status != RPMI_SUCCESS ||
	    share != RPMI_TEE_MEMORY_FULLY_SUPPORTED)
		panic("RPMI SHARE is not available to normal world");
	if (rpmi_tee_probe_features(hart->tee,
				    RPMI_TEE_FEATURE_MULTISEGMENT_OPS,
				    &multisegment, &status) ||
	    status != RPMI_SUCCESS)
		panic("Cannot query RPMI segment limit");
	rpmi_shm_init(&shm_context, ep_id, multisegment,
		      CFG_CORE_RPMI_MAX_PARCEL_PAGES);
	initialized = true;
}

struct sbi_mpxy_rpmi_channel *optee_rpmi_channel(void)
{
	assert(initialized);
	return current_hart()->tee;
}

struct rpmi_shm_context *optee_rpmi_shm_context(void)
{
	assert(initialized);
	return &shm_context;
}

void optee_rpmi_loop(void)
{
	struct rpmi_hart *hart = current_hart();

	assert(initialized && thread_get_exceptions() == THREAD_EXCP_ALL);
	if (!hart->request)
		bind_hart(hart);
	while (true) {
		struct rpmi_reqfwd_request request = {};
		size_t request_len = 0;
		size_t response_len = sizeof(hart->response);
		uint32_t pending = 0;
		int32_t status = RPMI_ERR_FAILED;
		TEE_Result res = TEE_SUCCESS;

		res = rpmi_reqfwd_retrieve(hart->reqfwd, hart->request,
					   CFG_CORE_RPMI_MAX_REQUEST_SIZE,
					   &request_len, &status);
		/* Idle retrieval must block in firmware, not return NO_DATA. */
		if (res || status != RPMI_SUCCESS)
			panic("Forwarded request retrieval failed");
		res = rpmi_reqfwd_parse_request(hart->request, request_len,
						&request);
		if (res) {
			put_unaligned_le32(hart->response,
					   RPMI_ERR_INVALID_PARAM);
			response_len = sizeof(uint32_t);
		} else {
			res = optee_rpmi_dispatch(shm_context.self_id, &request,
						  hart->response,
						  &response_len);
			if (res)
				panic("Cannot encode forwarded response");
		}
		res = rpmi_reqfwd_complete(hart->reqfwd, hart->response,
					   response_len, &pending, &status);
		/* Do not retry an uncertain completion. */
		if (res || status != RPMI_SUCCESS)
			panic("Forwarded request completion failed");
	}
}
