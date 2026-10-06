/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __RPMI_TEE_H
#define __RPMI_TEE_H

#include <sbi_mpxy_rpmi.h>
#include <stdint.h>
#include <tee_api_types.h>

/* TEE service group, specification revision d19395961d1a. */
#define RPMI_TEE_SERVICEGROUP_ID		0x0010
#define RPMI_TEE_PROBE_FEATURES		0x02
#define RPMI_TEE_PROBE_SYSTEM		0x03

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

#endif /* __RPMI_TEE_H */
