// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <io.h>
#include <kernel/misc.h>
#include <kernel/spinlock.h>
#include <kernel/thread_private.h>
#include <kernel/thread_rpmi.h>
#include <optee_msg.h>
#include <optee_rpc_cmd.h>
#include <stdlib.h>
#include <string.h>
#include <tee/entry_std.h>
#include <tee/tee_cryp_utl.h>

enum call_state {
	CALL_FREE,
	CALL_RUNNING,
	CALL_SUSPENDED,
};

struct rpmi_call {
	struct optee_rpmi_call_req req;
	struct mobj *mobj;
	struct optee_msg_arg *arg;
	size_t num_params;
	uint32_t caller;
	uint64_t token;
	enum call_state state;
};

/* Temporary-stack callers cannot migrate; a running secure thread can. */
static struct optee_rpmi_call_req starts[CFG_TEE_CORE_NB_CORE];
static struct rpmi_call calls[CFG_NUM_THREADS];
/* Protect token allocation and resume claims, never across a firmware call. */
static unsigned int call_lock = SPINLOCK_UNLOCK;
static uint64_t next_token;
static unsigned int rpc_pnum;

/* A logical RPC allocation can begin partway through its backing parcel. */
struct rpmi_payload {
	struct mobj mobj;
	struct mobj *parent;
	size_t offset;
};

static struct rpmi_payload *to_payload(struct mobj *mobj)
{
	return container_of(mobj, struct rpmi_payload, mobj);
}

static void *payload_get_va(struct mobj *mobj, size_t offset, size_t len)
{
	struct rpmi_payload *p = to_payload(mobj);

	if (!mobj_check_offset_and_len(mobj, offset, len))
		return NULL;
	return mobj_get_va(p->parent, p->offset + offset, len);
}

static TEE_Result payload_get_pa(struct mobj *mobj, size_t offset,
				 size_t granule, paddr_t *pa)
{
	struct rpmi_payload *p = to_payload(mobj);

	if (offset >= mobj->size)
		return TEE_ERROR_BAD_PARAMETERS;
	return mobj_get_pa(p->parent, p->offset + offset, granule, pa);
}

static size_t payload_get_phys_offs(struct mobj *mobj, size_t granule)
{
	struct rpmi_payload *p = to_payload(mobj);

	assert(granule && IS_POWER_OF_TWO(granule));
	return p->offset & (granule - 1);
}

static TEE_Result payload_get_mem_type(struct mobj *mobj, uint32_t *type)
{
	return mobj_get_mem_type(to_payload(mobj)->parent, type);
}

static bool payload_matches(struct mobj *mobj, enum buf_is_attr attr)
{
	return mobj_matches(to_payload(mobj)->parent, attr);
}

static uint64_t payload_get_cookie(struct mobj *mobj)
{
	return mobj_get_cookie(to_payload(mobj)->parent);
}

static void payload_free(struct mobj *mobj)
{
	struct rpmi_payload *p = to_payload(mobj);

	mobj_put(p->parent);
	free(p);
}

static const struct mobj_ops payload_ops = {
	.get_va = payload_get_va,
	.get_pa = payload_get_pa,
	.get_phys_offs = payload_get_phys_offs,
	.get_mem_type = payload_get_mem_type,
	.matches = payload_matches,
	.get_cookie = payload_get_cookie,
	.free = payload_free,
};

static int32_t result_status(TEE_Result res)
{
	if (!res)
		return RPMI_SUCCESS;
	if (res == TEE_ERROR_BUSY)
		return RPMI_ERR_BUSY;
	if (res == TEE_ERROR_BAD_PARAMETERS || res == TEE_ERROR_SECURITY ||
	    res == TEE_ERROR_BAD_FORMAT)
		return RPMI_ERR_INVALID_PARAM;
	return RPMI_ERR_FAILED;
}

TEE_Result thread_rpmi_handle_control(const void *data, size_t len,
				      void *response, size_t *response_len)
{
	uint32_t op = get_unaligned_le32(data);
	int32_t status = RPMI_ERR_INVALID_PARAM;
	uint32_t exceptions = 0;
	size_t n = 0;

	if (*response_len < sizeof(struct optee_rpmi_call_resp))
		return TEE_ERROR_SHORT_BUFFER;
	if (op == OPTEE_RPMI_YIELDING_CALL_WITH_ARG) {
		if (len != sizeof(struct optee_rpmi_call_req))
			goto out;
		if (!optee_rpmi_claim_caller(optee_rpmi_caller())) {
			status = RPMI_ERR_NOTSUPP;
			goto out;
		}
		memcpy(starts + get_core_pos(), data, len);
		thread_alloc_and_run(0, 0, 0, 0, 0, 0);
		/* No thread was acquired, so the command has not started. */
		status = RPMI_ERR_BUSY;
	} else if (op == OPTEE_RPMI_YIELDING_CALL_RESUME) {
		const struct optee_rpmi_resume_req *req = data;
		uint64_t token = 0;

		if (len != sizeof(*req) || get_unaligned_le32(&req->reserved))
			goto out;
		token = get_unaligned_le64(&req->resume_token);
		if (!token)
			goto out;
		exceptions = cpu_spin_lock_xsave(&call_lock);
		for (n = 0; n < ARRAY_SIZE(calls); n++) {
			if (calls[n].state != CALL_SUSPENDED ||
			    calls[n].token != token ||
			    calls[n].caller != optee_rpmi_caller())
				continue;
			calls[n].state = CALL_RUNNING;
			break;
		}
		cpu_spin_unlock_xrestore(&call_lock, exceptions);
		if (n == ARRAY_SIZE(calls))
			goto out;
		thread_resume_from_rpc(n, 0, 0, 0, 0);
		/* A claimed token must identify a suspended thread. */
		panic("RPMI call/thread state mismatch");
	}
out:
	memset(response, 0, sizeof(struct optee_rpmi_call_resp));
	put_unaligned_le32(response, status);
	*response_len = sizeof(struct optee_rpmi_call_resp);
	return TEE_SUCCESS;
}

/* Check both parcel-relative ranges before exposing the shared arguments. */
static TEE_Result get_arguments(struct rpmi_call *call,
				struct optee_msg_arg **rpc_arg)
{
	const struct optee_rpmi_call_req *req = &call->req;
	uint64_t arg_offset = get_unaligned_le64(&req->arg_offset);
	uint64_t rpc_offset = get_unaligned_le64(&req->rpc_offset);
	size_t arg_size = get_unaligned_le32(&req->arg_size);
	size_t rpc_size = get_unaligned_le32(&req->rpc_size);
	TEE_Result res = TEE_SUCCESS;

	if (get_unaligned_le32(&req->flags) ||
	    arg_offset > SIZE_MAX || rpc_offset > SIZE_MAX ||
	    arg_size < sizeof(struct optee_msg_arg) ||
	    rpc_size < OPTEE_MSG_GET_ARG_SIZE(THREAD_RPC_MAX_NUM_PARAMS))
		return TEE_ERROR_BAD_PARAMETERS;
	res = rpmi_shm_get(optee_rpmi_shm_context(), optee_rpmi_channel(),
			   call->caller, get_unaligned_le32(&req->parcel_id),
			   get_unaligned_le32(&req->nonce), &call->mobj);
	if (res)
		return res;
	if (arg_offset > call->mobj->size ||
	    arg_size > call->mobj->size - arg_offset ||
	    rpc_offset > call->mobj->size ||
	    rpc_size > call->mobj->size - rpc_offset ||
	    (arg_offset < rpc_offset + rpc_size &&
	     rpc_offset < arg_offset + arg_size))
		return TEE_ERROR_BAD_PARAMETERS;
	call->arg = mobj_get_va(call->mobj, arg_offset, arg_size);
	*rpc_arg = mobj_get_va(call->mobj, rpc_offset, rpc_size);
	if (!call->arg || !*rpc_arg ||
	    !IS_ALIGNED_WITH_TYPE(call->arg, struct optee_msg_arg) ||
	    !IS_ALIGNED_WITH_TYPE(*rpc_arg, struct optee_msg_arg))
		return TEE_ERROR_BAD_PARAMETERS;
	call->num_params = READ_ONCE(call->arg->num_params);
	if (call->num_params > OPTEE_MSG_MAX_NUM_PARAMS ||
	    OPTEE_MSG_GET_ARG_SIZE(call->num_params) != arg_size)
		return TEE_ERROR_BAD_PARAMETERS;
	return TEE_SUCCESS;
}

uint32_t __thread_std_abi_entry(uint32_t a0 __unused, uint32_t a1 __unused,
				uint32_t a2 __unused, uint32_t a3 __unused,
				uint32_t a4 __unused, uint32_t a5 __unused)
{
	struct thread_ctx *thr = threads + thread_get_id();
	struct rpmi_call *call = calls + thread_get_id();
	struct optee_msg_arg *rpc_arg = NULL;
	uint32_t exceptions = 0;
	TEE_Result res = TEE_SUCCESS;

	call->req = starts[get_core_pos()];
	call->caller = optee_rpmi_caller();
	res = get_arguments(call, &rpc_arg);
	if (res)
		goto out;
	exceptions = cpu_spin_lock_xsave(&call_lock);
	/* Never recycle a token that a malicious peer may have retained. */
	if (next_token == UINT64_MAX)
		panic("RPMI resume token space exhausted");
	call->token = ++next_token;
	call->state = CALL_RUNNING;
	cpu_spin_unlock_xrestore(&call_lock, exceptions);
	thr->rpc_arg = rpc_arg;
	res = tee_entry_std(call->arg, call->num_params);
	/* Cache cleanup can itself issue RPC while the call is still live. */
	thread_rpc_shm_cache_clear(&thr->shm_cache);
	thr->rpc_arg = NULL;
out:
	thread_set_exceptions(THREAD_EXCP_ALL);
	mobj_put(call->mobj);
	exceptions = cpu_spin_lock_xsave(&call_lock);
	*call = (struct rpmi_call){};
	cpu_spin_unlock_xrestore(&call_lock, exceptions);
	return result_status(res);
}

void thread_rpmi_suspend(uint32_t thread_id, uint32_t result)
{
	struct rpmi_call *call = NULL;
	uint32_t exceptions = 0;

	assert(thread_id < ARRAY_SIZE(calls));
	call = calls + thread_id;
	exceptions = cpu_spin_lock_xsave(&call_lock);
	assert(call->state == CALL_RUNNING);
	optee_rpmi_set_call_response(RPMI_SUCCESS, result, call->token);
	call->state = CALL_SUSPENDED;
	cpu_spin_unlock_xrestore(&call_lock, exceptions);
	thread_rpmi_return();
}

bool thread_disable_prealloc_rpc_cache(uint64_t *cookie)
{
	*cookie = 0;
	return true;
}

bool thread_enable_prealloc_rpc_cache(void)
{
	return false;
}

/* Use the caller-supplied RPC range; no register-ABI RPC_ALLOC is needed. */
uint32_t thread_rpc_cmd(uint32_t cmd, size_t num_params,
			struct thread_param *params)
{
	struct optee_msg_arg *arg = threads[thread_get_id()].rpc_arg;
	uint32_t rv[THREAD_RPC_NUM_ARGS] = {};
	size_t n = 0;

	if (!arg || num_params > THREAD_RPC_MAX_NUM_PARAMS)
		return TEE_ERROR_BAD_PARAMETERS;
	plat_prng_add_jitter_entropy(CRYPTO_RNG_SRC_JITTER_RPC, &rpc_pnum);
	memset(arg, 0, OPTEE_MSG_GET_ARG_SIZE(num_params));
	arg->cmd = cmd;
	arg->num_params = num_params;
	arg->ret = TEE_ERROR_GENERIC;
	for (n = 0; n < num_params; n++) {
		struct thread_param *p = params + n;
		struct optee_msg_param *mp = arg->params + n;

		switch (p->attr) {
		case THREAD_PARAM_ATTR_NONE:
			break;
		case THREAD_PARAM_ATTR_VALUE_IN:
		case THREAD_PARAM_ATTR_VALUE_OUT:
		case THREAD_PARAM_ATTR_VALUE_INOUT:
			mp->attr = p->attr - THREAD_PARAM_ATTR_VALUE_IN +
				   OPTEE_MSG_ATTR_TYPE_VALUE_INPUT;
			mp->u.value.a = p->u.value.a;
			mp->u.value.b = p->u.value.b;
			mp->u.value.c = p->u.value.c;
			break;
		case THREAD_PARAM_ATTR_MEMREF_IN:
		case THREAD_PARAM_ATTR_MEMREF_OUT:
		case THREAD_PARAM_ATTR_MEMREF_INOUT:
			mp->attr = p->attr - THREAD_PARAM_ATTR_MEMREF_IN +
				   OPTEE_MSG_ATTR_TYPE_RMEM_INPUT;
			mp->u.rmem.offs = p->u.memref.offs;
			mp->u.rmem.size = p->u.memref.size;
			mp->u.rmem.shm_ref = mobj_get_cookie(p->u.memref.mobj);
			if (p->u.memref.mobj &&
			    (!mp->u.rmem.shm_ref ||
			     !mobj_is_nonsec(p->u.memref.mobj) ||
			     p->u.memref.offs > p->u.memref.mobj->size ||
			     p->u.memref.size > p->u.memref.mobj->size -
							  p->u.memref.offs))
				return TEE_ERROR_BAD_PARAMETERS;
			if (!p->u.memref.mobj)
				mp->u.rmem.offs = 0;
			else if (p->u.memref.mobj->ops == &payload_ops)
				mp->u.rmem.offs +=
					to_payload(p->u.memref.mobj)->offset;
			break;
		default:
			return TEE_ERROR_BAD_PARAMETERS;
		}
	}
	thread_rpc(rv);
	for (n = 0; n < num_params; n++) {
		switch (params[n].attr) {
		case THREAD_PARAM_ATTR_VALUE_OUT:
		case THREAD_PARAM_ATTR_VALUE_INOUT:
			params[n].u.value.a = arg->params[n].u.value.a;
			params[n].u.value.b = arg->params[n].u.value.b;
			params[n].u.value.c = arg->params[n].u.value.c;
			break;
		case THREAD_PARAM_ATTR_MEMREF_OUT:
		case THREAD_PARAM_ATTR_MEMREF_INOUT:
			params[n].u.memref.size = arg->params[n].u.rmem.size;
			break;
		default:
			break;
		}
	}
	return arg->ret;
}

static void release_payload(unsigned int type, uint64_t key)
{
	struct thread_param p = THREAD_PARAM_VALUE(IN, type, (uint32_t)key,
						 key >> 32);
	uint32_t caller = calls[thread_get_id()].caller;
	TEE_Result res = TEE_SUCCESS;

	/* Release secure receiver interest before Linux reclaims the parcel. */
	res = rpmi_shm_unregister(optee_rpmi_shm_context(),
				  optee_rpmi_channel(), caller, key, key >> 32);
	if (res) {
		EMSG("Cannot retire RPC parcel: %#"PRIx32, res);
		return;
	}
	thread_rpc_cmd(OPTEE_RPC_CMD_SHM_FREE, 1, &p);
}

static void free_payload(unsigned int type, struct mobj *mobj)
{
	uint64_t key = mobj_get_cookie(mobj);

	if (!mobj)
		return;
	mobj_put(mobj);
	release_payload(type, key);
}

static struct mobj *alloc_payload(size_t size, unsigned int type)
{
	struct thread_param p = THREAD_PARAM_VALUE(IN, type, size, 8);
	struct optee_msg_arg *arg = threads[thread_get_id()].rpc_arg;
	struct mobj *mobj = NULL;
	struct rpmi_payload *payload = NULL;
	uint64_t key = 0;
	uint64_t offset = 0;
	uint64_t length = 0;
	TEE_Result res = TEE_SUCCESS;

	res = thread_rpc_cmd(OPTEE_RPC_CMD_SHM_ALLOC, 1, &p);
	if (res || arg->num_params != 1 ||
	    arg->params[0].attr != OPTEE_MSG_ATTR_TYPE_RMEM_OUTPUT)
		return NULL;
	key = READ_ONCE(arg->params[0].u.rmem.shm_ref);
	offset = READ_ONCE(arg->params[0].u.rmem.offs);
	length = READ_ONCE(arg->params[0].u.rmem.size);
	if (!key || offset > SIZE_MAX || length > SIZE_MAX || length < size)
		goto err;
	res = rpmi_shm_get(optee_rpmi_shm_context(), optee_rpmi_channel(),
			   calls[thread_get_id()].caller, key, key >> 32,
			   &mobj);
	if (res)
		goto err;
	if (offset > mobj->size || length > mobj->size - offset)
		goto err;
	payload = calloc(1, sizeof(*payload));
	if (!payload)
		goto err;
	payload->mobj.ops = &payload_ops;
	payload->mobj.size = length;
	payload->mobj.phys_granule = SMALL_PAGE_SIZE;
	refcount_set(&payload->mobj.refc, 1);
	payload->parent = mobj;
	payload->offset = offset;
	return &payload->mobj;
err:
	mobj_put(mobj);
	if (key)
		release_payload(type, key);
	return NULL;
}

struct mobj *thread_rpc_alloc_payload(size_t size)
{
	return alloc_payload(size, OPTEE_RPC_SHM_TYPE_APPL);
}

struct mobj *thread_rpc_alloc_kernel_payload(size_t size)
{
	return alloc_payload(size, OPTEE_RPC_SHM_TYPE_KERNEL);
}

struct mobj *thread_rpc_alloc_global_payload(size_t size)
{
	return alloc_payload(size, OPTEE_RPC_SHM_TYPE_GLOBAL);
}

void thread_rpc_free_payload(struct mobj *mobj)
{
	free_payload(OPTEE_RPC_SHM_TYPE_APPL, mobj);
}

void thread_rpc_free_kernel_payload(struct mobj *mobj)
{
	free_payload(OPTEE_RPC_SHM_TYPE_KERNEL, mobj);
}

void thread_rpc_free_global_payload(struct mobj *mobj)
{
	free_payload(OPTEE_RPC_SHM_TYPE_GLOBAL, mobj);
}
