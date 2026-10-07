// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <io.h>
#include <kernel/notif.h>
#include <kernel/thread.h>
#include <kernel/thread_rpmi.h>
#include <optee_rpmi.h>
#include <rpmi_tee.h>
#include <string.h>

#define RPMI_TEE_CALL	0x18

struct rpmi_tee_call_req {
	uint32_t sender_id;
	uint32_t target_id;
	uint8_t service[16];
	uint32_t data_len;
	uint8_t data[];
} __packed;

struct rpmi_tee_call_resp {
	int32_t status;
	uint32_t data_len;
	uint8_t data[];
} __packed;

/* RFC 4122 byte order, matching Linux's OPTEE_RPMI_SERVICE_UUID. */
static const uint8_t service_uuid[] = {
	0x48, 0x61, 0x78, 0xe0, 0xe7, 0xf8, 0x11, 0xe3,
	0xbc, 0x5e, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b,
};

/* Query-only operations run on the temporary stack without allocating a thread. */
static TEE_Result dispatch_control(const void *data, size_t len,
				   void *response, size_t *response_len)
{
	union {
		struct optee_rpmi_status_resp status;
		struct optee_rpmi_api_resp api;
		struct optee_rpmi_os_resp os;
		struct optee_rpmi_caps_resp caps;
	} resp = {};
	size_t size = sizeof(resp.status);
	int32_t status = RPMI_SUCCESS;
	uint32_t op = 0;

	if (len < sizeof(struct optee_rpmi_probe_req)) {
		status = RPMI_ERR_INVALID_PARAM;
		goto out;
	}
	op = get_unaligned_le32(data);
	if (op <= OPTEE_RPMI_EXCHANGE_CAPABILITIES &&
	    len != sizeof(struct optee_rpmi_probe_req)) {
		status = RPMI_ERR_INVALID_PARAM;
		goto out;
	}
	switch (op) {
	case OPTEE_RPMI_GET_API_VERSION:
		size = sizeof(resp.api);
		put_unaligned_le32(&resp.api.major, OPTEE_RPMI_VERSION_MAJOR);
		put_unaligned_le32(&resp.api.minor, OPTEE_RPMI_VERSION_MINOR);
		break;
	case OPTEE_RPMI_GET_OS_VERSION:
		size = sizeof(resp.os);
		put_unaligned_le32(&resp.os.major, CFG_OPTEE_REVISION_MAJOR);
		put_unaligned_le32(&resp.os.minor, CFG_OPTEE_REVISION_MINOR);
		break;
	case OPTEE_RPMI_EXCHANGE_CAPABILITIES:
		size = sizeof(resp.caps);
		put_unaligned_le32(&resp.caps.rpc_param_count,
				   THREAD_RPC_MAX_NUM_PARAMS);
		put_unaligned_le32(&resp.caps.notification_count,
				   NOTIF_VALUE_MAX + 1);
		break;
#ifdef CFG_CORE_RPMI
	case OPTEE_RPMI_YIELDING_CALL_WITH_ARG:
	case OPTEE_RPMI_YIELDING_CALL_RESUME:
		return thread_rpmi_handle_control(data, len, response,
						  response_len);
	case OPTEE_RPMI_UNREGISTER_SHM: {
		const struct optee_rpmi_unregister_req *req = data;
		TEE_Result res = TEE_SUCCESS;

		if (len != sizeof(*req)) {
			status = RPMI_ERR_INVALID_PARAM;
			break;
		}
		res = rpmi_shm_unregister(optee_rpmi_shm_context(),
					  optee_rpmi_channel(),
					  optee_rpmi_caller(),
					  get_unaligned_le32(&req->parcel_id),
					  get_unaligned_le32(&req->nonce));
		if (res == TEE_ERROR_BUSY)
			status = RPMI_ERR_BUSY;
		else if (res == TEE_ERROR_BAD_PARAMETERS)
			status = RPMI_ERR_INVALID_PARAM;
		else if (res)
			status = RPMI_ERR_FAILED;
		break;
	}
#endif
	default:
		status = RPMI_ERR_NOTSUPP;
		break;
	}
out:
	if (*response_len < size)
		return TEE_ERROR_SHORT_BUFFER;
	put_unaligned_le32(&resp.status.status, status);
	memcpy(response, &resp, size);
	*response_len = size;
	return TEE_SUCCESS;
}

TEE_Result optee_rpmi_dispatch(uint32_t self_id,
			       const struct rpmi_reqfwd_request *request,
			       void *response, size_t *response_len)
{
	const struct rpmi_tee_call_req *req = NULL;
	struct rpmi_tee_call_resp *resp = response;
	size_t len = 0;
	TEE_Result res = TEE_SUCCESS;

	if (!request || !response || !response_len)
		return TEE_ERROR_BAD_PARAMETERS;
	if (request->servicegroup_id != RPMI_TEE_SERVICEGROUP_ID ||
	    request->service_id != RPMI_TEE_CALL) {
		if (*response_len < sizeof(uint32_t))
			return TEE_ERROR_SHORT_BUFFER;
		put_unaligned_le32(response, RPMI_ERR_NOTSUPP);
		*response_len = sizeof(uint32_t);
		return TEE_SUCCESS;
	}
	if (*response_len < sizeof(*resp))
		return TEE_ERROR_SHORT_BUFFER;
	put_unaligned_le32(&resp->status, RPMI_ERR_INVALID_PARAM);
	put_unaligned_le32(&resp->data_len, 0);
	len = *response_len - sizeof(*resp);
	*response_len = sizeof(*resp);
	if (!request->data || request->len < sizeof(*req))
		return TEE_SUCCESS;
	req = request->data;
	if (get_unaligned_le32(&req->target_id) != self_id ||
	    memcmp(req->service, service_uuid, sizeof(service_uuid)) ||
	    get_unaligned_le32(&req->data_len) != request->len - sizeof(*req))
		return TEE_SUCCESS;

#ifdef CFG_CORE_RPMI
	optee_rpmi_set_caller(get_unaligned_le32(&req->sender_id));
#endif
	res = dispatch_control(req->data, request->len - sizeof(*req),
			       resp->data, &len);
	if (res)
		return res;
	put_unaligned_le32(&resp->status, RPMI_SUCCESS);
	put_unaligned_le32(&resp->data_len, len);
	*response_len += len;
	return TEE_SUCCESS;
}
