// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <io.h>
#include <rpmi_tee.h>
#include <stdlib.h>
#include <util.h>

struct rpmi_tee_probe_features_req {
	uint32_t feature_id;
};

struct rpmi_tee_probe_features_resp {
	int32_t status;
	uint32_t value;
};

struct rpmi_tee_probe_system_resp {
	int32_t status;
	uint32_t caller_domain;
	uint32_t caller_endpoint;
	uint32_t domain_count;
	uint32_t domains[];
};

/* Separate local exchange errors from the firmware's RPMI service status. */
static TEE_Result tee_send(struct sbi_mpxy_rpmi_channel *channel,
			   uint32_t service_id, void *req, size_t req_len,
			   void *resp, unsigned long *resp_len, int32_t *status)
{
	struct sbi_mpxy_rpmi_message msg = {};

	if (channel->rpmi_attrs.servicegroup_id != RPMI_TEE_SERVICEGROUP_ID ||
	    !(channel->attrs.capability & SBI_MPXY_CHAN_CAP_SEND_WITH_RESP))
		return TEE_ERROR_NOT_SUPPORTED;
	if (req_len > channel->attrs.msg_max_len)
		return TEE_ERROR_SHORT_BUFFER;

	sbi_mpxy_rpmi_init_send_with_response(&msg, service_id, req, req_len,
					      resp, *resp_len);
	if (sbi_mpxy_rpmi_send_data(channel, &msg) || msg.error)
		return TEE_ERROR_COMMUNICATION;
	if (msg.data.response_len < sizeof(uint32_t))
		return TEE_ERROR_BAD_FORMAT;

	*resp_len = msg.data.response_len;
	*status = (int32_t)get_unaligned_le32(resp);
	return TEE_SUCCESS;
}

TEE_Result rpmi_tee_probe_features(struct sbi_mpxy_rpmi_channel *channel,
				   uint32_t feature_id, uint32_t *value,
				   int32_t *status)
{
	struct rpmi_tee_probe_features_req req = {};
	struct rpmi_tee_probe_features_resp resp = {};
	unsigned long resp_len = sizeof(resp);
	TEE_Result res = TEE_SUCCESS;

	if (!channel || !value || !status)
		return TEE_ERROR_BAD_PARAMETERS;
	*value = 0;
	*status = RPMI_ERR_FAILED;
	put_unaligned_le32(&req.feature_id, feature_id);
	res = tee_send(channel, RPMI_TEE_PROBE_FEATURES, &req, sizeof(req),
		       &resp, &resp_len, status);
	if (res || *status != RPMI_SUCCESS)
		return res;
	if (resp_len != sizeof(resp))
		return TEE_ERROR_BAD_FORMAT;

	*value = get_unaligned_le32(&resp.value);
	return TEE_SUCCESS;
}

TEE_Result rpmi_tee_probe_system(struct sbi_mpxy_rpmi_channel *channel,
				 uint32_t *domain_id, uint32_t *ep_id,
				 int32_t *status)
{
	struct rpmi_tee_probe_system_resp *resp = NULL;
	unsigned long shmem_size = 0;
	unsigned long resp_len = 0;
	uint32_t domain_count = 0;
	TEE_Result res = TEE_SUCCESS;

	if (!channel || !domain_id || !ep_id || !status)
		return TEE_ERROR_BAD_PARAMETERS;
	*domain_id = 0;
	*ep_id = 0;
	*status = RPMI_ERR_FAILED;
	if (sbi_mpxy_get_shmem_size(&shmem_size))
		return TEE_ERROR_COMMUNICATION;
	resp_len = MIN(shmem_size, channel->attrs.msg_max_len);
	if (resp_len < sizeof(*resp))
		return TEE_ERROR_NOT_SUPPORTED;

	resp = malloc(resp_len);
	if (!resp)
		return TEE_ERROR_OUT_OF_MEMORY;
	/* TEE_PROBE_SYSTEM has no request data in this revision. */
	res = tee_send(channel, RPMI_TEE_PROBE_SYSTEM, NULL, 0, resp, &resp_len,
		       status);
	if (res || *status != RPMI_SUCCESS)
		goto out;
	if (resp_len < sizeof(*resp)) {
		res = TEE_ERROR_BAD_FORMAT;
		goto out;
	}

	domain_count = get_unaligned_le32(&resp->domain_count);
	if ((resp_len - sizeof(*resp)) % sizeof(uint32_t) ||
	    domain_count != (resp_len - sizeof(*resp)) / sizeof(uint32_t)) {
		res = TEE_ERROR_BAD_FORMAT;
		goto out;
	}
	*domain_id = get_unaligned_le32(&resp->caller_domain);
	*ep_id = get_unaligned_le32(&resp->caller_endpoint);
out:
	free(resp);
	return res;
}
