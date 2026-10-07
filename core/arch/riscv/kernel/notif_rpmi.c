// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <initcall.h>
#include <inttypes.h>
#include <kernel/notif.h>
#include <kernel/panic.h>
#include <kernel/spinlock.h>
#include <kernel/thread_rpmi.h>
#include <trace.h>

/* Protect the single bus, binding, pending doorbell and raise count. */
static unsigned int notif_lock = SPINLOCK_UNLOCK;
static uint32_t peer_id;
static uint32_t signal_id;
static unsigned int in_flight;
static bool enabled;
static bool pending;
static unsigned int stopping;
static bool stopped;
static bool bus_active;

/* Firmware authenticates bus control for the initial single-peer system. */
int32_t optee_rpmi_signal_bus_setup(void)
{
	uint32_t exceptions = cpu_spin_lock_xsave(&notif_lock);
	int32_t status = RPMI_ERR_DENIED;

	if (!bus_active && !stopping) {
		bus_active = true;
		status = RPMI_SUCCESS;
	}
	cpu_spin_unlock_xrestore(&notif_lock, exceptions);
	return status;
}

int32_t optee_rpmi_enable_async_notif(uint32_t peer, uint32_t signal)
{
	uint32_t exceptions = 0;

	if (!optee_rpmi_claim_caller(peer))
		return RPMI_ERR_DENIED;
	exceptions = cpu_spin_lock_xsave(&notif_lock);
	if (!bus_active) {
		cpu_spin_unlock_xrestore(&notif_lock, exceptions);
		return RPMI_ERR_DENIED;
	}
	if (enabled || stopping) {
		cpu_spin_unlock_xrestore(&notif_lock, exceptions);
		return RPMI_ERR_BUSY;
	}
	/* Linux owns signal allocation; do not probe it by raising a signal. */
	peer_id = peer;
	signal_id = signal;
	stopped = false;
	enabled = true;
	cpu_spin_unlock_xrestore(&notif_lock, exceptions);
	notif_deliver_atomic_event(NOTIF_EVENT_STARTED, 0);
	return RPMI_SUCCESS;
}

void notif_send_async(uint32_t value, uint16_t guest_id __unused)
{
	uint32_t exceptions = cpu_spin_lock_xsave(&notif_lock);

	assert(value == NOTIF_VALUE_DO_BOTTOM_HALF);
	if (!stopped)
		pending = true;
	cpu_spin_unlock_xrestore(&notif_lock, exceptions);
}

/* Flush outside common notification locks and before returning to firmware. */
void optee_rpmi_flush_async_notif(void)
{
	uint32_t exceptions = cpu_spin_lock_xsave(&notif_lock);
	uint32_t peer = peer_id;
	uint32_t signal = signal_id;
	int32_t status = RPMI_ERR_FAILED;
	TEE_Result res = TEE_SUCCESS;

	if (!enabled || !pending) {
		cpu_spin_unlock_xrestore(&notif_lock, exceptions);
		return;
	}
	pending = false;
	in_flight++;
	cpu_spin_unlock_xrestore(&notif_lock, exceptions);

	res = rpmi_tee_signal_raise(optee_rpmi_channel(), peer, signal,
				    &status);
	if (res || status != RPMI_SUCCESS) {
		EMSG("RPMI signal raise failed: result %#" PRIx32
		     ", status %" PRId32,
		     res, status);
		panic();
	}
	exceptions = cpu_spin_lock_xsave(&notif_lock);
	in_flight--;
	cpu_spin_unlock_xrestore(&notif_lock, exceptions);
}

static void stop_async_notif(bool teardown)
{
	uint32_t exceptions = 0;
	bool busy = false;

	exceptions = cpu_spin_lock_xsave(&notif_lock);
	if (teardown)
		bus_active = false;
	enabled = false;
	pending = false;
	stopping++;
	stopped = true;
	cpu_spin_unlock_xrestore(&notif_lock, exceptions);

	/* Only another hart can have a synchronous raise still in progress. */
	do {
		exceptions = cpu_spin_lock_xsave(&notif_lock);
		busy = in_flight != 0;
		cpu_spin_unlock_xrestore(&notif_lock, exceptions);
	} while (busy);

	/* Allow a new binding only after the old producer has stopped. */
	exceptions = cpu_spin_lock_xsave(&notif_lock);
	stopping--;
	cpu_spin_unlock_xrestore(&notif_lock, exceptions);
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

static struct notif_driver notif_driver = {
	.yielding_cb = notif_rpmi_event,
};

static TEE_Result notif_rpmi_init(void)
{
	notif_register_driver(&notif_driver);
	return TEE_SUCCESS;
}
driver_init(notif_rpmi_init);
