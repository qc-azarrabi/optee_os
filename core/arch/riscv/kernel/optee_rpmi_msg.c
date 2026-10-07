// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <io.h>
#include <kernel/notif.h>
#include <kernel/panic.h>
#include <kernel/thread.h>
#include <optee_rpmi.h>
#include <rpmi_tee.h>
#include <string.h>
#include <utee_defines.h>

#include "optee_rpmi_private.h"

/* Decoded view of a normal RPMI request; data borrows the input buffer. */
struct rpmi_request {
	uint16_t servicegroup_id;
	uint8_t service_id;
	const void *data;
	size_t len;
};

/* Validate the RPMI header and return a view of its request data. */
static TEE_Result rpmi_parse_request(const void *data, size_t len,
				     struct rpmi_request *request)
{
	const struct rpmi_message *msg = data;
	uint16_t datalen = 0;

	*request = (struct rpmi_request){};
	if (len < sizeof(msg->header))
		return TEE_ERROR_BAD_FORMAT;

	datalen = get_unaligned_le16(&msg->header.datalen);
	/* Bit 3 belongs to the transport; the remaining flags describe type. */
	if ((msg->header.flags & ~BIT(3)) != RPMI_MSG_NORMAL_REQUEST ||
	    datalen % sizeof(uint32_t) ||
	    datalen != len - sizeof(msg->header))
		return TEE_ERROR_BAD_FORMAT;

	request->servicegroup_id =
		get_unaligned_le16(&msg->header.servicegroup_id);
	request->service_id = msg->header.service_id;
	request->data = msg->data;
	request->len = datalen;
	return TEE_SUCCESS;
}

struct rpmi_tee_signal_bus_setup_req {
	uint32_t target_id;
	uint32_t bus_width;
	uint32_t sender_signals;
} __packed;

struct rpmi_tee_signal_bus_teardown_req {
	uint32_t target_id;
} __packed;

/* Complete bus control with its own status, not a TEE_CALL envelope. */
static TEE_Result dispatch_signal_bus(uint32_t self_id,
				      const struct rpmi_request *request,
				      void *response, size_t *response_len)
{
	int32_t status = RPMI_ERR_DENIED;

	if (request->service_id == RPMI_TEE_SIGNAL_BUS_SETUP) {
		const struct rpmi_tee_signal_bus_setup_req *req = request->data;

		/* Firmware checks widths, ranges and peer authorization. */
		if (request->len == sizeof(*req) &&
		    get_unaligned_le32(&req->target_id) == self_id)
			status = optee_rpmi_signal_bus_setup();
	} else {
		const struct rpmi_tee_signal_bus_teardown_req *req =
			request->data;

		/* Forwarded teardown permits only a success response. */
		if (request->len != sizeof(*req) ||
		    get_unaligned_le32(&req->target_id) != self_id)
			panic("Malformed forwarded signal-bus teardown");
		optee_rpmi_signal_bus_teardown();
		status = RPMI_SUCCESS;
	}
	put_unaligned_le32(response, status);
	*response_len = sizeof(status);
	return TEE_SUCCESS;
}

/* RFC 4122 byte order, matching Linux's OPTEE_RPMI_SERVICE_UUID. */
static const uint8_t service_uuid[] = {
	0x48, 0x61, 0x78, 0xe0, 0xe7, 0xf8, 0x11, 0xe3,
	0xbc, 0x5e, 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b,
};

/* Retire a parcel identified by an unaligned control request. */
static TEE_Result handle_unregister_shm(uint32_t caller, const void *data,
					size_t len)
{
	struct optee_rpmi_unregister_req req = {};
	uint64_t global_id = 0;

	if (len != sizeof(req))
		return TEE_ERROR_BAD_PARAMETERS;
	memcpy(&req, data, sizeof(req));
	if (TEE_U32_FROM_LITTLE_ENDIAN(req.reserved))
		return TEE_ERROR_BAD_PARAMETERS;
	global_id = TEE_U64_FROM_LITTLE_ENDIAN(req.global_id);

	return rpmi_shm_unregister(optee_rpmi_shm_context(),
				   optee_rpmi_channel(), caller,
				   global_id);
}

#ifdef CFG_CORE_ASYNC_NOTIF
/* Enable the doorbell described by an unaligned control request. */
static TEE_Result handle_enable_async_notif(uint32_t caller, const void *data,
					    size_t len)
{
	struct optee_rpmi_enable_notif_req req = {};
	uint32_t signal = 0;

	if (len != sizeof(req))
		return TEE_ERROR_BAD_PARAMETERS;
	memcpy(&req, data, sizeof(req));
	signal = TEE_U32_FROM_LITTLE_ENDIAN(req.signal_id);

	return optee_rpmi_enable_async_notif(caller, signal);
}
#endif

/* Dispatch controls on the temporary stack or enter a secure thread. */
static TEE_Result dispatch_control(uint32_t caller, const void *data,
				   size_t len,
				   union optee_rpmi_control_response *resp,
				   size_t *response_len)
{
	size_t size = sizeof(resp->status);
	TEE_Result status = TEE_SUCCESS;
	uint32_t op = 0;

	if (len < sizeof(struct optee_rpmi_probe_req)) {
		status = TEE_ERROR_BAD_PARAMETERS;
		goto out;
	}
	op = get_unaligned_le32(data);
	if (op <= OPTEE_RPMI_EXCHANGE_CAPABILITIES &&
	    len != sizeof(struct optee_rpmi_probe_req)) {
		status = TEE_ERROR_BAD_PARAMETERS;
		goto out;
	}
	switch (op) {
	case OPTEE_RPMI_GET_API_VERSION:
		size = sizeof(resp->api);
		resp->api.major =
			TEE_U32_TO_LITTLE_ENDIAN(OPTEE_RPMI_VERSION_MAJOR);
		resp->api.minor =
			TEE_U32_TO_LITTLE_ENDIAN(OPTEE_RPMI_VERSION_MINOR);
		break;
	case OPTEE_RPMI_GET_OS_VERSION:
		size = sizeof(resp->os);
		resp->os.major =
			TEE_U32_TO_LITTLE_ENDIAN(CFG_OPTEE_REVISION_MAJOR);
		resp->os.minor =
			TEE_U32_TO_LITTLE_ENDIAN(CFG_OPTEE_REVISION_MINOR);
		break;
	case OPTEE_RPMI_EXCHANGE_CAPABILITIES:
		size = sizeof(resp->caps);
		resp->caps.rpc_param_count =
			TEE_U32_TO_LITTLE_ENDIAN(THREAD_RPC_MAX_NUM_PARAMS);
		resp->caps.max_notification_key =
			TEE_U32_TO_LITTLE_ENDIAN(NOTIF_VALUE_MAX);
		break;
#ifdef CFG_CORE_ASYNC_NOTIF
	case OPTEE_RPMI_ENABLE_ASYNC_NOTIF:
		if (*response_len < size)
			return TEE_ERROR_SHORT_BUFFER;
		status = handle_enable_async_notif(caller, data, len);
		break;
#endif
	case OPTEE_RPMI_YIELDING_CALL_WITH_ARG:
	case OPTEE_RPMI_YIELDING_CALL_RESUME:
		return thread_rpmi_handle_control(caller, op, data, len,
						  &resp->call, response_len);
	case OPTEE_RPMI_UNREGISTER_SHM:
		if (*response_len < size)
			return TEE_ERROR_SHORT_BUFFER;
		status = handle_unregister_shm(caller, data, len);
		break;
	default:
		status = TEE_ERROR_NOT_SUPPORTED;
		break;
	}
out:
	if (*response_len < size)
		return TEE_ERROR_SHORT_BUFFER;
	resp->status.status = TEE_U32_TO_LITTLE_ENDIAN(status);
	*response_len = size;
	return TEE_SUCCESS;
}

TEE_Result optee_rpmi_dispatch(uint32_t self_id,
			       const void *data, size_t data_len,
			       void *response, size_t *response_len)
{
	struct rpmi_request request = {};
	union optee_rpmi_control_response control = {};
	const uint8_t *service = NULL;
	const void *service_data = NULL;
	size_t service_len = 0;
	size_t response_len_max = 0;
	size_t len = 0;
	uint32_t sender_id = 0;
	uint32_t target_id = 0;
	TEE_Result res = TEE_SUCCESS;

	if (*response_len < sizeof(uint32_t))
		return TEE_ERROR_SHORT_BUFFER;
	res = rpmi_parse_request(data, data_len, &request);
	if (res) {
		put_unaligned_le32(response, RPMI_ERR_INVALID_PARAM);
		*response_len = sizeof(uint32_t);
		return TEE_SUCCESS;
	}
	if (request.servicegroup_id == RPMI_TEE_SERVICEGROUP_ID &&
	    (request.service_id == RPMI_TEE_SIGNAL_BUS_SETUP ||
	     request.service_id == RPMI_TEE_SIGNAL_BUS_TEARDOWN))
		return dispatch_signal_bus(self_id, &request, response,
					   response_len);
	if (request.servicegroup_id != RPMI_TEE_SERVICEGROUP_ID ||
	    request.service_id != RPMI_TEE_CALL) {
		put_unaligned_le32(response, RPMI_ERR_NOTSUPP);
		*response_len = sizeof(uint32_t);
		return TEE_SUCCESS;
	}
	response_len_max = *response_len;
	res = rpmi_tee_call_set_response(RPMI_ERR_INVALID_PARAM, NULL, 0,
					 response, response_len);
	if (res)
		return res;
	/* Limit the operation's output to the caller's payload buffer size. */
	len = MIN(sizeof(control), response_len_max - *response_len);
	res = rpmi_tee_call_get_args(request.data, request.len, &sender_id,
				     &target_id, &service, &service_data,
				     &service_len);
	if (res || target_id != self_id ||
	    memcmp(service, service_uuid, sizeof(service_uuid)))
		return TEE_SUCCESS;

	res = dispatch_control(sender_id, service_data, service_len,
			       &control, &len);
	if (res)
		return res;
	*response_len = response_len_max;
	return rpmi_tee_call_set_response(RPMI_SUCCESS, &control, len,
					  response, response_len);
}
