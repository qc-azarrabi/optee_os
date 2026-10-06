// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <io.h>
#include <rpmi_reqfwd.h>
#include <stdlib.h>
#include <string.h>
#include <util.h>

struct rpmi_reqfwd_retrieve_req {
	uint32_t start_index;
};

struct rpmi_reqfwd_retrieve_resp {
	int32_t status;
	uint32_t remaining;
	uint32_t returned;
	uint8_t data[];
};

struct rpmi_reqfwd_complete_resp {
	int32_t status;
	uint32_t num_messages;
};

static TEE_Result reqfwd_send(struct sbi_mpxy_rpmi_channel *channel,
			      uint32_t service_id, void *req, size_t req_len,
			      void *resp, unsigned long *resp_len)
{
	struct sbi_mpxy_rpmi_message msg = {};
	int ret = 0;

	if (!channel || channel->rpmi_attrs.servicegroup_id !=
			RPMI_SRVGRP_REQUEST_FORWARD)
		return TEE_ERROR_BAD_PARAMETERS;

	if (!(channel->attrs.capability & SBI_MPXY_CHAN_CAP_SEND_WITH_RESP))
		return TEE_ERROR_NOT_SUPPORTED;

	if (req_len > channel->attrs.msg_max_len)
		return TEE_ERROR_EXCESS_DATA;

	sbi_mpxy_rpmi_init_send_with_response(&msg, service_id, req, req_len,
					      resp, *resp_len);
	ret = sbi_mpxy_rpmi_send_data(channel, &msg);
	if (ret)
		return TEE_ERROR_COMMUNICATION;
	if (msg.error)
		return TEE_ERROR_COMMUNICATION;

	*resp_len = msg.data.response_len;
	return TEE_SUCCESS;
}

TEE_Result rpmi_reqfwd_retrieve(struct sbi_mpxy_rpmi_channel *channel,
				void *data, size_t capacity, size_t *len,
				int32_t *status)
{
	struct rpmi_reqfwd_retrieve_req req = {};
	struct rpmi_reqfwd_retrieve_resp *resp = NULL;
	unsigned long shmem_size = 0;
	size_t response_capacity = 0;
	size_t offset = 0;
	size_t total = 0;
	TEE_Result res = TEE_SUCCESS;

	if (!channel || !data || !len || !status)
		return TEE_ERROR_BAD_PARAMETERS;

	*len = 0;
	*status = RPMI_ERR_FAILED;
	if (sbi_mpxy_get_shmem_size(&shmem_size))
		return TEE_ERROR_COMMUNICATION;

	response_capacity = MIN(shmem_size, channel->attrs.msg_max_len);
	if (response_capacity <= sizeof(*resp))
		return TEE_ERROR_NOT_SUPPORTED;

	resp = malloc(response_capacity);
	if (!resp)
		return TEE_ERROR_OUT_OF_MEMORY;

	while (true) {
		unsigned long response_len = response_capacity;
		uint32_t remaining = 0;
		uint32_t returned = 0;
		uint64_t end = 0;

		put_unaligned_le32(&req.start_index, offset);
		res = reqfwd_send(channel, RPMI_REQFWD_RETRIEVE_CURRENT_MESSAGE,
				  &req, sizeof(req), resp, &response_len);
		if (res)
			break;
		if (response_len < sizeof(resp->status)) {
			res = TEE_ERROR_BAD_FORMAT;
			break;
		}

		*status = (int32_t)get_unaligned_le32(&resp->status);
		if (*status != RPMI_SUCCESS)
			break;
		if (response_len < sizeof(*resp)) {
			res = TEE_ERROR_BAD_FORMAT;
			break;
		}

		remaining = get_unaligned_le32(&resp->remaining);
		returned = get_unaligned_le32(&resp->returned);
		end = (uint64_t)offset + returned + remaining;
		if (!returned || returned != response_len - sizeof(*resp) ||
		    end > UINT32_MAX || (offset && end != total)) {
			res = TEE_ERROR_BAD_FORMAT;
			break;
		}
		if (!offset)
			total = end;
		if (total > capacity) {
			*len = total;
			res = TEE_ERROR_SHORT_BUFFER;
			break;
		}

		memcpy((uint8_t *)data + offset, resp->data, returned);
		offset += returned;
		if (!remaining) {
			*len = offset;
			break;
		}
	}

	free(resp);
	return res;
}

TEE_Result rpmi_reqfwd_complete(struct sbi_mpxy_rpmi_channel *channel,
				void *data, size_t len, uint32_t *num_messages,
				int32_t *status)
{
	struct rpmi_reqfwd_complete_resp resp = {};
	unsigned long response_len = sizeof(resp);
	TEE_Result res = TEE_SUCCESS;

	if ((!data && len) || !num_messages || !status)
		return TEE_ERROR_BAD_PARAMETERS;

	*num_messages = 0;
	*status = RPMI_ERR_FAILED;
	res = reqfwd_send(channel, RPMI_REQFWD_COMPLETE_CURRENT_MESSAGE, data,
			  len, &resp, &response_len);
	if (res)
		return res;
	if (response_len < sizeof(resp.status))
		return TEE_ERROR_BAD_FORMAT;

	*status = (int32_t)get_unaligned_le32(&resp.status);
	if (*status != RPMI_SUCCESS)
		return TEE_SUCCESS;
	if (response_len != sizeof(resp))
		return TEE_ERROR_BAD_FORMAT;

	*num_messages = get_unaligned_le32(&resp.num_messages);
	return TEE_SUCCESS;
}
