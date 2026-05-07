/* SPDX-License-Identifier: GPL-2.0 */
/*
 * camss-isp-bufq.h
 *
 * CAMSS ISP per-queue ready-buffer FIFO.
 *
 * Provides N spinlock-protected FIFO lists of ready vb2 buffers, one per
 * queue index.  Drivers call these helpers from their vb2 ops and job
 * completion paths.
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef CAMSS_ISP_BUFQ_H
#define CAMSS_ISP_BUFQ_H

#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <media/videobuf2-v4l2.h>

/**
 * struct camss_isp_buf - vb2 buffer wrapper
 *
 * Use as vb2_queue.buf_struct_size so buffers can be placed on the
 * ready lists managed by camss_isp_bufq.
 *
 * @vb:   The vb2 V4L2 buffer — must be first.
 * @list: Entry in the per-queue ready list.
 */
struct camss_isp_buf {
	struct vb2_v4l2_buffer	vb;	/* must be first */
	struct list_head	list;
};

/**
 * struct camss_isp_bufq_entry - per-queue ready-buffer state (opaque)
 */
struct camss_isp_bufq_entry {
	struct list_head	rdy_queue;
	spinlock_t		rdy_spinlock;
	u32			num_rdy;
};

/**
 * struct camss_isp_bufq - multi-queue ready-buffer state
 *
 * Allocate with camss_isp_bufq_init(), free with camss_isp_bufq_release().
 *
 * @num_queues: Number of entries in @entries.
 * @entries:    Per-queue state; flexible array.
 */
struct camss_isp_bufq {
	unsigned int			num_queues;
	struct camss_isp_bufq_entry	entries[] __counted_by(num_queues);
};

/**
 * camss_isp_bufq_init() - allocate a multi-queue ready-buffer state
 * @num_queues: number of per-queue FIFO lists to create
 *
 * Returns a pointer to the new bufq or ERR_PTR on allocation failure.
 */
struct camss_isp_bufq *camss_isp_bufq_init(unsigned int num_queues);

/**
 * camss_isp_bufq_release() - free a bufq allocated with camss_isp_bufq_init()
 * @bufq: bufq to free
 */
void camss_isp_bufq_release(struct camss_isp_bufq *bufq);

/**
 * camss_isp_bufq_queue() - append a buffer to the ready list for @queue_idx
 * @bufq:      target bufq
 * @queue_idx: queue index (must be < bufq->num_queues)
 * @vbuf:      buffer to enqueue; must be embedded in a &struct camss_isp_buf
 */
void camss_isp_bufq_queue(struct camss_isp_bufq *bufq, unsigned int queue_idx,
			   struct vb2_v4l2_buffer *vbuf);

/**
 * camss_isp_bufq_next() - peek at the head of the ready list without removing
 * @bufq:      target bufq
 * @queue_idx: queue index
 *
 * Returns the head buffer or NULL if the list is empty.
 */
struct vb2_v4l2_buffer *camss_isp_bufq_next(struct camss_isp_bufq *bufq,
					     unsigned int queue_idx);

/**
 * camss_isp_bufq_remove() - dequeue and return the head of the ready list
 * @bufq:      target bufq
 * @queue_idx: queue index
 *
 * Returns the dequeued buffer or NULL if the list is empty.
 */
struct vb2_v4l2_buffer *camss_isp_bufq_remove(struct camss_isp_bufq *bufq,
					       unsigned int queue_idx);

/**
 * camss_isp_bufq_drain() - return all ready buffers with the given state
 * @bufq:      target bufq
 * @queue_idx: queue index
 * @state:     vb2 state to pass to vb2_buffer_done() for each buffer
 */
void camss_isp_bufq_drain(struct camss_isp_bufq *bufq, unsigned int queue_idx,
			   enum vb2_buffer_state state);

static inline u32 camss_isp_bufq_num_ready(struct camss_isp_bufq *bufq,
					    unsigned int queue_idx)
{
	return bufq->entries[queue_idx].num_rdy;
}

static inline void camss_isp_buf_done(struct vb2_v4l2_buffer *vbuf,
				       enum vb2_buffer_state state)
{
	vb2_buffer_done(&vbuf->vb2_buf, state);
}

#endif /* CAMSS_ISP_BUFQ_H */
