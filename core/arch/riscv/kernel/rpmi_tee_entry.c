// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <io.h>
#include <kernel/misc.h>
#include <kernel/panic.h>
#include <kernel/spinlock.h>
#include <kernel/thread.h>
#include <kernel/thread_rpmi.h>
#include <rpmi_tee.h>
#include <stdlib.h>

struct rpmi_hart {
	struct sbi_mpxy_rpmi_channel *reqfwd;
	struct sbi_mpxy_rpmi_channel *tee;
	uint8_t *request;
	uint8_t response[32];
	uint32_t caller;
	size_t response_len;
};

struct rpmi_call_response {
	int32_t status;
	uint32_t len;
	struct optee_rpmi_call_resp call;
} __packed;

static struct rpmi_hart harts[CFG_TEE_CORE_NB_CORE];
static struct rpmi_shm_context shm_context;
static bool initialized;
static unsigned int peer_lock = SPINLOCK_UNLOCK;
static uint32_t peer_id;
static bool peer_bound;

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

uint32_t optee_rpmi_caller(void)
{
	return current_hart()->caller;
}

void optee_rpmi_set_caller(uint32_t caller)
{
	current_hart()->caller = caller;
}

bool optee_rpmi_claim_caller(uint32_t caller)
{
	uint32_t exceptions = cpu_spin_lock_xsave(&peer_lock);
	bool accepted = false;

	if (!peer_bound) {
		peer_id = caller;
		peer_bound = true;
	}
	accepted = peer_id == caller;
	cpu_spin_unlock_xrestore(&peer_lock, exceptions);
	return accepted;
}

void optee_rpmi_set_call_response(int32_t status, uint32_t result,
				  uint64_t token)
{
	struct rpmi_hart *hart = current_hart();
	struct rpmi_call_response *resp = (void *)hart->response;

	put_unaligned_le32(&resp->status, RPMI_SUCCESS);
	put_unaligned_le32(&resp->len, sizeof(resp->call));
	put_unaligned_le32(&resp->call.status, status);
	put_unaligned_le32(&resp->call.result, result);
	put_unaligned_le64(&resp->call.resume_token, token);
	hart->response_len = sizeof(*resp);
}

static void complete_current(struct rpmi_hart *hart)
{
	uint32_t pending = 0;
	int32_t status = RPMI_ERR_FAILED;
	TEE_Result res = TEE_SUCCESS;

	optee_rpmi_flush_async_notif();
	res = rpmi_reqfwd_complete(hart->reqfwd, hart->response,
				   hart->response_len, &pending, &status);
	/* An uncertain completion cannot be retried without duplicating it. */
	if (res || status != RPMI_SUCCESS)
		panic("Forwarded request completion failed");
}

void optee_rpmi_complete_and_loop(void)
{
	complete_current(current_hart());
	optee_rpmi_loop();
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
		int32_t status = RPMI_ERR_FAILED;
		TEE_Result res = TEE_SUCCESS;

		hart->response_len = sizeof(hart->response);
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
			hart->response_len = sizeof(uint32_t);
		} else {
			res = optee_rpmi_dispatch(shm_context.self_id, &request,
						  hart->response,
						  &hart->response_len);
			if (res)
				panic("Cannot encode forwarded response");
		}
		complete_current(hart);
	}
}
