/*
 * Copyright (c) 2022 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/can.h>

struct can_prioq_elem {
	struct zcan_frame frame;
	can_tx_callback_t callback;
	void *user_data;
};

struct can_prioq {
	struct k_mem_slab *mem_slab;

};

#define CAN_PRIOQ_INITIALIZER(mem_slab) \
	{ \
	.mem_slab = mem_slab, \
	}

#define CAN_PRIOQ_DEFINE_STATIC(name, max_frames)			\
	K_MEM_SLAB_DEFINE_STATIC(can_prioq_mem_slab_##name,		\
				 sizeof(struct can_prioq_elem),		\
				 max_frames, 4);			\
	static struct can_prioq ##name =				\
		CAN_PRIOQ_INITIALIZER(&can_prioq_mem_slab_##name)

int can_prioq_add_frame(struct can_prioq *prioq, const struct zcan_frame *frame,
			k_timeout_t timeout, can_tx_callback_t callback,
			void *user_data);
