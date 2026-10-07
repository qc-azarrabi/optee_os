// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <kernel/dt.h>
#include <kernel/thread_rpmi.h>
#include <libfdt.h>

/* This per-CPU sample convention is not a finalized firmware DT binding. */
TEE_Result __weak
optee_rpmi_get_channel_ids(uint32_t hart_id, uint32_t *reqfwd_id,
			   uint32_t *tee_id)
{
	const void *fdt = get_dt();
	const char *reqfwd_compat = "riscv,rpmi-mpxy-reqfwd";
	const char *tee_compat = "riscv,rpmi-mpxy-tee";
	const char *channel_prop = "riscv,sbi-mpxy-channel-id";
	bool found_cpu = false;
	bool found_reqfwd = false;
	bool found_tee = false;
	int cpus = 0;
	int cpu = 0;
	int node = 0;

	if (!fdt || !reqfwd_id || !tee_id)
		return TEE_ERROR_BAD_PARAMETERS;
	cpus = fdt_path_offset(fdt, "/cpus");
	if (cpus < 0)
		return TEE_ERROR_ITEM_NOT_FOUND;
	fdt_for_each_subnode(cpu, fdt, cpus) {
		const fdt32_t *reg = NULL;
		int len = 0;
		uint64_t id = 0;

		reg = fdt_getprop(fdt, cpu, "reg", &len);
		if (!reg || (len != 4 && len != 8))
			continue;
		id = fdt32_to_cpu(reg[0]);
		if (len == 8)
			id = (id << 32) | fdt32_to_cpu(reg[1]);
		if (id != hart_id)
			continue;
		if (found_cpu)
			return TEE_ERROR_BAD_FORMAT;
		found_cpu = true;
		fdt_for_each_subnode(node, fdt, cpu) {
			bool *found = NULL;
			uint32_t *channel_id = NULL;

			if (!fdt_node_check_compatible(fdt, node,
						       reqfwd_compat)) {
				found = &found_reqfwd;
				channel_id = reqfwd_id;
			} else if (!fdt_node_check_compatible(fdt, node,
							     tee_compat)) {
				found = &found_tee;
				channel_id = tee_id;
			} else {
				continue;
			}
			if (*found || fdt_read_uint32(fdt, node,
						      channel_prop, channel_id))
				return TEE_ERROR_BAD_FORMAT;
			*found = true;
		}
	}
	if (found_reqfwd && found_tee)
		return TEE_SUCCESS;
	return TEE_ERROR_ITEM_NOT_FOUND;
}
