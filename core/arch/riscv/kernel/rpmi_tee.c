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

/* Fixed RPMI memory granule, independent of the host's page size. */
#define RPMI_TEE_PAGE_SHIFT		12
#define RPMI_TEE_PAGE_SIZE		BIT(RPMI_TEE_PAGE_SHIFT)
#define RPMI_TEE_ACCESS_RW		(BIT(29) | BIT(30))
#define RPMI_TEE_MEMORY_MULTI_SEGMENT	BIT(31)
#define RPMI_TEE_MEMORY_LAST_SEGMENT	BIT(31)

struct rpmi_tee_memory_accept_req {
	uint32_t acceptor_id;
	uint32_t access;
	uint32_t parcel_id;
	uint32_t nonce;
	uint32_t creator_id;
	uint32_t creator_access;
	uint32_t flags;
	uint32_t address_high;
	uint32_t address_low;
	uint32_t max_pages;
	uint32_t other_count;
};

struct rpmi_tee_memory_accept_resp {
	int32_t status;
	uint32_t flags;
	uint32_t page_count;
	uint32_t block_count;
	uint32_t blocks[];
};

struct rpmi_tee_memory_segment_req {
	uint32_t acceptor_id;
	uint32_t parcel_id;
};

struct rpmi_tee_memory_segment_resp {
	int32_t status;
	uint32_t flags;
	uint32_t block_count;
	uint32_t blocks[];
};

struct rpmi_tee_memory_release_req {
	uint32_t parcel_id;
	uint32_t flags;
	uint32_t ep_count;
	uint32_t ep_id;
};

struct rpmi_tee_signal_raise_req {
	uint32_t target_id;
	uint32_t signal_count;
	uint32_t signal;
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

/* Expand split HIGH[]/LOW[] arrays into bounded, representable 4 KiB pages. */
static TEE_Result decode_blocks(const void *data, size_t len, uint32_t count,
				paddr_t *pages, size_t total, size_t *next)
{
	const uint32_t *blocks = data;
	uint32_t i = 0;

	if (!count || len % (2 * sizeof(uint32_t)) ||
	    count != len / (2 * sizeof(uint32_t)))
		return TEE_ERROR_BAD_FORMAT;
	for (i = 0; i < count; i++) {
		uint32_t high = get_unaligned_le32(blocks + i);
		uint32_t low = get_unaligned_le32(blocks + count + i);
		uint32_t n = (low & (RPMI_TEE_PAGE_SIZE - 1)) + 1;
		uint64_t address = ((uint64_t)high << 32) |
				   (low & ~(RPMI_TEE_PAGE_SIZE - 1));
		uint64_t last = 0;

		if (*next > total || n > total - *next ||
		    ADD_OVERFLOW(address, ((uint64_t)n << RPMI_TEE_PAGE_SHIFT) - 1,
				 &last) ||
		    (paddr_t)address != address || (paddr_t)last != last)
			return TEE_ERROR_BAD_FORMAT;
		while (n--) {
			pages[(*next)++] = address;
			address += RPMI_TEE_PAGE_SIZE;
		}
	}
	return TEE_SUCCESS;
}

TEE_Result rpmi_tee_memory_accept(struct sbi_mpxy_rpmi_channel *channel,
				  uint32_t acceptor_id, uint32_t creator_id,
				  uint32_t parcel_id, uint32_t nonce,
				  paddr_t *pages, size_t capacity,
				  size_t *page_count, bool *needs_release,
				  int32_t *status)
{
	struct rpmi_tee_memory_accept_req req = {};
	struct rpmi_tee_memory_accept_resp *resp = NULL;
	struct rpmi_tee_memory_segment_req segment = {};
	unsigned long shmem_size = 0;
	unsigned long max_resp_len = 0;
	unsigned long resp_len = 0;
	size_t total = 0;
	size_t next = 0;
	uint32_t flags = 0;
	TEE_Result res = TEE_SUCCESS;

	if (!channel || !pages || !capacity || capacity > UINT32_MAX ||
	    !page_count || !needs_release || !status)
		return TEE_ERROR_BAD_PARAMETERS;
	*page_count = 0;
	*needs_release = false;
	*status = RPMI_ERR_FAILED;
	if (sbi_mpxy_get_shmem_size(&shmem_size))
		return TEE_ERROR_COMMUNICATION;
	max_resp_len = MIN(shmem_size, channel->attrs.msg_max_len);
	if (max_resp_len < sizeof(*resp) + 2 * sizeof(uint32_t) ||
	    shmem_size < sizeof(req) ||
	    channel->attrs.msg_max_len < sizeof(req) ||
	    channel->rpmi_attrs.servicegroup_id != RPMI_TEE_SERVICEGROUP_ID ||
	    !(channel->attrs.capability & SBI_MPXY_CHAN_CAP_SEND_WITH_RESP))
		return TEE_ERROR_NOT_SUPPORTED;
	resp = malloc(max_resp_len);
	if (!resp)
		return TEE_ERROR_OUT_OF_MEMORY;

	put_unaligned_le32(&req.acceptor_id, acceptor_id);
	put_unaligned_le32(&req.access, RPMI_TEE_ACCESS_RW);
	put_unaligned_le32(&req.parcel_id, parcel_id);
	put_unaligned_le32(&req.nonce, nonce);
	put_unaligned_le32(&req.creator_id, creator_id);
	put_unaligned_le32(&req.creator_access, RPMI_TEE_ACCESS_RW);
	put_unaligned_le32(&req.max_pages, capacity);
	resp_len = max_resp_len;
	/* A failed exchange may still have acquired interest in firmware. */
	*needs_release = true;
	res = tee_send(channel, RPMI_TEE_MEMORY_PARCEL_ACCEPT, &req, sizeof(req),
		       resp, &resp_len, status);
	if (res)
		goto out;
	if (*status != RPMI_SUCCESS) {
		*needs_release = false;
		goto out;
	}
	if (resp_len < sizeof(*resp)) {
		res = TEE_ERROR_BAD_FORMAT;
		goto out;
	}
	flags = get_unaligned_le32(&resp->flags);
	total = get_unaligned_le32(&resp->page_count);
	if (flags & ~RPMI_TEE_MEMORY_MULTI_SEGMENT || !total || total > capacity) {
		res = TEE_ERROR_BAD_FORMAT;
		goto out;
	}
	res = decode_blocks(resp->blocks, resp_len - sizeof(*resp),
			    get_unaligned_le32(&resp->block_count), pages,
			    total, &next);
	if (res)
		goto out;
	put_unaligned_le32(&segment.acceptor_id, acceptor_id);
	put_unaligned_le32(&segment.parcel_id, parcel_id);
	while (flags & RPMI_TEE_MEMORY_MULTI_SEGMENT) {
		struct rpmi_tee_memory_segment_resp *part = (void *)resp;

		/* A non-final segment must leave room for at least one more page. */
		if (next == total) {
			res = TEE_ERROR_BAD_FORMAT;
			goto out;
		}
		resp_len = max_resp_len;
		res = tee_send(channel, RPMI_TEE_MEMORY_SEGMENT_RECEIVE, &segment,
			       sizeof(segment), part, &resp_len, status);
		if (res || *status != RPMI_SUCCESS)
			goto out;
		if (resp_len < sizeof(*part)) {
			res = TEE_ERROR_BAD_FORMAT;
			goto out;
		}
		flags = get_unaligned_le32(&part->flags);
		if (flags & ~RPMI_TEE_MEMORY_LAST_SEGMENT) {
			res = TEE_ERROR_BAD_FORMAT;
			goto out;
		}
		res = decode_blocks(part->blocks, resp_len - sizeof(*part),
				    get_unaligned_le32(&part->block_count),
				    pages, total, &next);
		if (res)
			goto out;
		flags = flags & RPMI_TEE_MEMORY_LAST_SEGMENT ? 0 :
			RPMI_TEE_MEMORY_MULTI_SEGMENT;
	}
	if (next != total)
		res = TEE_ERROR_BAD_FORMAT;
	else
		*page_count = total;
out:
	free(resp);
	return res;
}

TEE_Result rpmi_tee_memory_release(struct sbi_mpxy_rpmi_channel *channel,
				   uint32_t ep_id, uint32_t parcel_id,
				   int32_t *status)
{
	struct rpmi_tee_memory_release_req req = {};
	uint32_t resp = 0;
	unsigned long resp_len = sizeof(resp);
	TEE_Result res = TEE_SUCCESS;

	if (!channel || !status)
		return TEE_ERROR_BAD_PARAMETERS;
	*status = RPMI_ERR_FAILED;
	put_unaligned_le32(&req.parcel_id, parcel_id);
	put_unaligned_le32(&req.ep_count, 1);
	put_unaligned_le32(&req.ep_id, ep_id);
	res = tee_send(channel, RPMI_TEE_MEMORY_PARCEL_RELEASE, &req, sizeof(req),
		       &resp, &resp_len, status);
	if (!res && resp_len != sizeof(resp))
		return TEE_ERROR_BAD_FORMAT;
	return res;
}

TEE_Result rpmi_tee_signal_raise(struct sbi_mpxy_rpmi_channel *channel,
				 uint32_t target_id, uint32_t signal_id,
				 int32_t *status)
{
	struct rpmi_tee_signal_raise_req req = {};
	uint32_t resp = 0;
	unsigned long resp_len = sizeof(resp);
	TEE_Result res = TEE_SUCCESS;

	if (!channel || !status)
		return TEE_ERROR_BAD_PARAMETERS;
	*status = RPMI_ERR_FAILED;
	put_unaligned_le32(&req.target_id, target_id);
	put_unaligned_le32(&req.signal_count, 1);
	put_unaligned_le32(&req.signal, signal_id);
	res = tee_send(channel, RPMI_TEE_SIGNAL_RAISE, &req, sizeof(req),
		       &resp, &resp_len, status);
	if (!res && resp_len != sizeof(resp))
		return TEE_ERROR_BAD_FORMAT;
	return res;
}
