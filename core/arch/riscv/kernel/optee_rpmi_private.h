/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __KERNEL_OPTEE_RPMI_PRIVATE_H
#define __KERNEL_OPTEE_RPMI_PRIVATE_H

#include <optee_rpmi.h>
#include <rpmi.h>
#include <rpmi_tee.h>
#include <stddef.h>
#include <tee_api_types.h>

/* Supported OP-TEE control responses, excluding the TEE_CALL header. */
union optee_rpmi_control_response {
	struct optee_rpmi_status_resp status;
	struct optee_rpmi_api_resp api;
	struct optee_rpmi_os_resp os;
	struct optee_rpmi_caps_resp caps;
};

/* Forwarded-request dispatch and completion. */
/*
 * Decode an RPMI packet and construct its complete service response data.
 * The caller supplies non-NULL data, resp and resp_len pointers.
 * Malformed packets receive RPMI_ERR_INVALID_PARAM for normal completion.
 * resp_len is the supplied buffer size, replaced by the actual output length.
 * Firmware must authenticate the TEE_CALL sender before forwarding it.
 */
TEE_Result optee_rpmi_dispatch(uint32_t self_id,
			       const void *data, size_t data_len,
			       void *resp, size_t *resp_len);

#endif /* __KERNEL_OPTEE_RPMI_PRIVATE_H */
