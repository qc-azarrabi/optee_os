/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __RPMI_REQFWD_H
#define __RPMI_REQFWD_H

#include <sbi_mpxy_rpmi.h>
#include <stddef.h>
#include <stdint.h>
#include <tee_api_types.h>

/* REQUEST_FORWARD service IDs, RPMI specification section 4.13. */
#define RPMI_REQFWD_RETRIEVE_CURRENT_MESSAGE	0x02
#define RPMI_REQFWD_COMPLETE_CURRENT_MESSAGE	0x03

/*
 * Retrieve a complete forwarded request into caller-owned storage.
 *
 * The caller exclusively owns this queue from retrieval through completion;
 * no other consumer may retrieve or complete its current message. Select the
 * channel for the servicing hart explicitly and do not migrate between chunks.
 * MPXY protects scratch access with exception masking; no global lock is held
 * while firmware waits or switches domains.
 *
 * On TEE_SUCCESS, *status contains the RPMI result; *len contains the request
 * length only for RPMI_SUCCESS. NO_DATA is returned without polling. A failure
 * after retrieval starts may leave a current request in firmware: the caller
 * must stop normal dispatch rather than retry retrieval from index zero.
 * On TEE_ERROR_SHORT_BUFFER, *len contains the required request size.
 */
TEE_Result rpmi_reqfwd_retrieve(struct sbi_mpxy_rpmi_channel *channel,
				void *data, size_t capacity, size_t *len,
				int32_t *status);

/*
 * Complete the queue's current request with exactly len response-data bytes.
 * The caller must have retrieved that request successfully and must not submit
 * a second completion. TEE_SUCCESS reports a valid RPMI response; *status is
 * its service result. *num_messages is valid only for RPMI_SUCCESS. Completion
 * failure does not prove that firmware retained or delivered the response.
 */
TEE_Result rpmi_reqfwd_complete(struct sbi_mpxy_rpmi_channel *channel,
				void *data, size_t len, uint32_t *num_messages,
				int32_t *status);

#endif /* __RPMI_REQFWD_H */
