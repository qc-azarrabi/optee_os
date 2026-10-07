// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <inttypes.h>
#include <kernel/notif.h>
#include <kernel/panic.h>
#include <kernel/spinlock.h>
#include <rpmi_tee.h>
#include <trace.h>

#include "optee_rpmi_private.h"

/* Firmware authenticates bus control for the initial single-peer system. */
int32_t optee_rpmi_signal_bus_setup(void)
{
	struct optee_rpmi_notif_state *state =
		&optee_rpmi_get_context()->notif;
	uint32_t exceptions = cpu_spin_lock_xsave(&state->lock);
	int32_t status = RPMI_ERR_DENIED;

	if (!state->bus_active && !state->stopping) {
		state->bus_active = true;
		status = RPMI_SUCCESS;
	}
	cpu_spin_unlock_xrestore(&state->lock, exceptions);
	return status;
}

TEE_Result optee_rpmi_enable_async_notif(uint32_t peer, uint32_t signal)
{
	struct optee_rpmi_notif_state *state =
		&optee_rpmi_get_context()->notif;
	uint32_t exceptions = 0;

	if (!optee_rpmi_claim_caller(peer))
		return TEE_ERROR_ACCESS_DENIED;
	exceptions = cpu_spin_lock_xsave(&state->lock);
	if (!state->bus_active) {
		cpu_spin_unlock_xrestore(&state->lock, exceptions);
		return TEE_ERROR_ACCESS_DENIED;
	}
	if (state->enabled || state->stopping) {
		cpu_spin_unlock_xrestore(&state->lock, exceptions);
		return TEE_ERROR_BUSY;
	}
	/* Linux owns signal allocation; do not probe it by raising a signal. */
	state->peer_id = peer;
	state->signal_id = signal;
	state->stopped = false;
	state->enabled = true;
	cpu_spin_unlock_xrestore(&state->lock, exceptions);
	notif_deliver_atomic_event(NOTIF_EVENT_STARTED, 0);
	return TEE_SUCCESS;
}

void notif_send_async(uint32_t value, uint16_t guest_id __unused)
{
	struct optee_rpmi_notif_state *state =
		&optee_rpmi_get_context()->notif;
	uint32_t exceptions = cpu_spin_lock_xsave(&state->lock);

	assert(value == NOTIF_VALUE_DO_BOTTOM_HALF);
	if (!state->stopped)
		state->pending = true;
	cpu_spin_unlock_xrestore(&state->lock, exceptions);
}

/* Flush outside common notification locks and before returning to firmware. */
void optee_rpmi_flush_async_notif(void)
{
	struct optee_rpmi_notif_state *state =
		&optee_rpmi_get_context()->notif;
	uint32_t exceptions = cpu_spin_lock_xsave(&state->lock);
	uint32_t peer = state->peer_id;
	uint32_t signal = state->signal_id;
	TEE_Result res = TEE_SUCCESS;

	if (!state->enabled || !state->pending) {
		cpu_spin_unlock_xrestore(&state->lock, exceptions);
		return;
	}
	state->pending = false;
	state->in_flight++;
	cpu_spin_unlock_xrestore(&state->lock, exceptions);

	res = rpmi_tee_signal_raise(optee_rpmi_channel(), peer, signal);
	if (res) {
		EMSG("RPMI signal raise failed: %#" PRIx32, res);
		panic();
	}
	exceptions = cpu_spin_lock_xsave(&state->lock);
	state->in_flight--;
	cpu_spin_unlock_xrestore(&state->lock, exceptions);
}

static void stop_async_notif(bool teardown)
{
	struct optee_rpmi_notif_state *state =
		&optee_rpmi_get_context()->notif;
	uint32_t exceptions = 0;
	bool busy = false;

	exceptions = cpu_spin_lock_xsave(&state->lock);
	if (teardown)
		state->bus_active = false;
	state->enabled = false;
	state->pending = false;
	state->stopping++;
	state->stopped = true;
	cpu_spin_unlock_xrestore(&state->lock, exceptions);

	/* Only another hart can have a synchronous raise still in progress. */
	do {
		exceptions = cpu_spin_lock_xsave(&state->lock);
		busy = state->in_flight != 0;
		cpu_spin_unlock_xrestore(&state->lock, exceptions);
	} while (busy);

	/* Allow a new binding only after the old producer has stopped. */
	exceptions = cpu_spin_lock_xsave(&state->lock);
	state->stopping--;
	cpu_spin_unlock_xrestore(&state->lock, exceptions);
}

/* Bus teardown is not a yielding service STOP or parcel/call retirement. */
void optee_rpmi_signal_bus_teardown(void)
{
	stop_async_notif(true);
}

static void notif_rpmi_event(struct notif_driver *driver __unused,
			     enum notif_event event)
{
	if (event == NOTIF_EVENT_STOPPED)
		stop_async_notif(false);
}

void notif_rpmi_init(void)
{
	struct optee_rpmi_notif_state *state =
		&optee_rpmi_get_context()->notif;
	state->driver.yielding_cb = notif_rpmi_event;
	notif_register_driver(&state->driver);
}
