/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __RPMI_TEE_H
#define __RPMI_TEE_H

#include <sbi_mpxy_rpmi.h>
#include <stdint.h>
#include <tee_api_types.h>
#include <types_ext.h>

/* TEE service group, specification revision d19395961d1a. */
#define RPMI_TEE_SERVICEGROUP_ID		0x0010
#define RPMI_TEE_PROBE_FEATURES		0x02
#define RPMI_TEE_PROBE_SYSTEM		0x03
#define RPMI_TEE_MEMORY_PARCEL_ACCEPT	0x0f
#define RPMI_TEE_MEMORY_PARCEL_RELEASE	0x10
#define RPMI_TEE_MEMORY_SEGMENT_RECEIVE	0x13

/* Feature IDs in TEE_PROBE_FEATURES. */
#define RPMI_TEE_FEATURE_MEMORY_DONATE	0
#define RPMI_TEE_FEATURE_MEMORY_LEND	1
#define RPMI_TEE_FEATURE_MEMORY_SHARE	2
#define RPMI_TEE_FEATURE_SIGNAL_BUS	3
#define RPMI_TEE_FEATURE_MULTISEGMENT_OPS	4

/* Support levels returned for memory features. */
#define RPMI_TEE_MEMORY_UNSUPPORTED	0
#define RPMI_TEE_MEMORY_TEE_ONLY		1
#define RPMI_TEE_MEMORY_FULLY_SUPPORTED	2

/*
 * Query a framework feature on an explicitly selected TEE-group channel.
 * TEE_SUCCESS indicates a valid exchange, not successful feature discovery:
 * *status is the firmware's RPMI result and *value is valid only on success.
 * On local failure, *status must not be consumed and *value is zero.
 */
TEE_Result rpmi_tee_probe_features(struct sbi_mpxy_rpmi_channel *channel,
				   uint32_t feature_id, uint32_t *value,
				   int32_t *status);

/*
 * Discover the calling physical endpoint and domain with TEE_PROBE_SYSTEM.
 * Validate the domain-array extent but do not retain the system topology.
 * TEE_SUCCESS indicates a valid exchange; *status is the firmware's RPMI
 * result. Identity outputs are valid only for RPMI_SUCCESS and are zero on
 * failure. No fixed endpoint ID or old SYSINFO-format query is used.
 */
TEE_Result rpmi_tee_probe_system(struct sbi_mpxy_rpmi_channel *channel,
				 uint32_t *domain_id, uint32_t *ep_id,
				 int32_t *status);

/*
 * Accept an RW SHARE parcel from creator_id for acceptor_id. The caller owns
 * pages and supplies its capacity in 4 KiB pages. No mapping is exposed before
 * all segments and the total page count have been validated. *page_count is
 * valid only when both the local result and the remote status are successful.
 *
 * *needs_release records acquired or uncertain receiver interest. On failure,
 * the caller must release it and retain recovery state if release fails. It is
 * not safe to blindly retry acceptance. Serialize operations on the same parcel
 * and obey the endpoint's concurrent multi-segment-operation limit.
 */
TEE_Result rpmi_tee_memory_accept(struct sbi_mpxy_rpmi_channel *channel,
				  uint32_t acceptor_id, uint32_t creator_id,
				  uint32_t parcel_id, uint32_t nonce,
				  paddr_t *pages, size_t capacity,
				  size_t *page_count, bool *needs_release,
				  int32_t *status);

/* Release this endpoint's interest, including an incomplete acceptance. */
TEE_Result rpmi_tee_memory_release(struct sbi_mpxy_rpmi_channel *channel,
				   uint32_t ep_id, uint32_t parcel_id,
				   int32_t *status);

#endif /* __RPMI_TEE_H */
