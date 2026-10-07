/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef __KERNEL_THREAD_RPMI_H
#define __KERNEL_THREAD_RPMI_H

#include <rpmi_reqfwd.h>

/*
 * Dispatch a decoded request and construct its complete service response data.
 * resp_len is the supplied capacity, replaced by the actual output length.
 * Firmware must authenticate the TEE_CALL sender before forwarding it.
 */
TEE_Result optee_rpmi_dispatch(uint32_t self_id,
			       const struct rpmi_reqfwd_request *request,
			       void *resp, size_t *resp_len);

#endif /* __KERNEL_THREAD_RPMI_H */
