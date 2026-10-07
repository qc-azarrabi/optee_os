/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __OPTEE_RPMI_H
#define __OPTEE_RPMI_H

#include <compiler.h>
#include <stdint.h>

/*
 * OP-TEE service over RPMI TEE_CALL. All control fields are little-endian.
 * Control status is a signed RPMI error, separate from the TEE_CALL status
 * and the command's GP result. Shared command arguments use optee_msg.h.
 */
#define OPTEE_RPMI_VERSION_MAJOR	1
#define OPTEE_RPMI_VERSION_MINOR	0

struct optee_rpmi_probe_req {
	uint32_t op;
} __packed;

struct optee_rpmi_status_resp {
	int32_t status;
} __packed;

/* Query the service ABI revision using optee_rpmi_probe_req. */
#define OPTEE_RPMI_GET_API_VERSION	0

struct optee_rpmi_api_resp {
	int32_t status;
	uint32_t major;
	uint32_t minor;
} __packed;

/* Query the trusted OS revision using optee_rpmi_probe_req. */
#define OPTEE_RPMI_GET_OS_VERSION		1

struct optee_rpmi_os_resp {
	int32_t status;
	uint32_t major;
	uint32_t minor;
	uint32_t reserved;	/* Zero in version 1. */
	uint64_t build_id;	/* Zero when unspecified. */
} __packed;

/* Query capabilities and argument capacities using optee_rpmi_probe_req. */
#define OPTEE_RPMI_EXCHANGE_CAPABILITIES	2

struct optee_rpmi_caps_resp {
	int32_t status;
	uint32_t secure_caps;		/* Reserved; zero in version 1. */
	uint32_t rpc_param_count;		/* Capacity of the supplied RPC buffer. */
	uint32_t notification_count;	/* Keys are [0, notification_count). */
} __packed;

/* Retire a parcel; success acknowledges unmapping and receiver release. */
#define OPTEE_RPMI_UNREGISTER_SHM		3

struct optee_rpmi_unregister_req {
	uint32_t op;
	uint32_t parcel_id;
	uint32_t nonce;
} __packed;

/* Enable bottom-half doorbells to an authenticated caller's allocated signal. */
#define OPTEE_RPMI_ENABLE_ASYNC_NOTIF	4

struct optee_rpmi_enable_notif_req {
	uint32_t op;
	uint32_t signal_id;
} __packed;

/* Start a yielding command using parcel-backed command and RPC ranges. */
#define OPTEE_RPMI_YIELDING_CALL_WITH_ARG	5

struct optee_rpmi_call_req {
	uint32_t op;
	uint32_t parcel_id;
	uint32_t nonce;
	uint32_t flags;		/* Zero in version 1. */
	uint64_t arg_offset;	/* Offset from the parcel's first byte. */
	uint64_t rpc_offset;	/* Disjoint from the command range. */
	uint32_t arg_size;
	uint32_t rpc_size;	/* Must fit the negotiated RPC capacity. */
} __packed;

#define OPTEE_RPMI_YIELDING_CALL_RETURN_DONE	0
#define OPTEE_RPMI_YIELDING_CALL_RETURN_RPC_CMD	1
#define OPTEE_RPMI_YIELDING_CALL_RETURN_INTERRUPT	2

struct optee_rpmi_call_resp {
	int32_t status;
	uint32_t result;		/* RETURN_* when status is RPMI_SUCCESS. */
	uint64_t resume_token;	/* Zero for DONE; opaque, nonzero otherwise. */
} __packed;

/* Resume an existing call; rejection must not be reported as RPMI_ERR_BUSY. */
#define OPTEE_RPMI_YIELDING_CALL_RESUME	6

struct optee_rpmi_resume_req {
	uint32_t op;
	uint32_t reserved;	/* Zero in version 1. */
	uint64_t resume_token;	/* Bound to the call, caller and service. */
} __packed;

#endif /* __OPTEE_RPMI_H */
