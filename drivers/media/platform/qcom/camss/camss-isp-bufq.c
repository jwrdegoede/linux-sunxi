// SPDX-License-Identifier: GPL-2.0
/*
 * camss-isp-bufq.c
 *
 * CAMSS ISP per-queue ready-buffer FIFO.
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/module.h>
#include <linux/slab.h>

#include "camss-isp-bufq.h"

struct camss_isp_bufq *camss_isp_bufq_init(unsigned int num_queues)
{
	struct camss_isp_bufq *bufq;
	unsigned int i;

	bufq = kzalloc(struct_size(bufq, entries, num_queues), GFP_KERNEL);
	if (!bufq)
		return ERR_PTR(-ENOMEM);

	bufq->num_queues = num_queues;

	for (i = 0; i < num_queues; i++) {
		INIT_LIST_HEAD(&bufq->entries[i].rdy_queue);
		spin_lock_init(&bufq->entries[i].rdy_spinlock);
	}

	return bufq;
}
EXPORT_SYMBOL_GPL(camss_isp_bufq_init);

void camss_isp_bufq_release(struct camss_isp_bufq *bufq)
{
	kfree(bufq);
}
EXPORT_SYMBOL_GPL(camss_isp_bufq_release);

void camss_isp_bufq_queue(struct camss_isp_bufq *bufq, unsigned int queue_idx,
			  struct vb2_v4l2_buffer *vbuf)
{
	struct camss_isp_buf *buf =
		container_of(vbuf, struct camss_isp_buf, vb);
	struct camss_isp_bufq_entry *entry = &bufq->entries[queue_idx];
	unsigned long flags;

	spin_lock_irqsave(&entry->rdy_spinlock, flags);
	list_add_tail(&buf->list, &entry->rdy_queue);
	entry->num_rdy++;
	spin_unlock_irqrestore(&entry->rdy_spinlock, flags);
}
EXPORT_SYMBOL_GPL(camss_isp_bufq_queue);

struct vb2_v4l2_buffer *camss_isp_bufq_next(struct camss_isp_bufq *bufq, unsigned int queue_idx)
{
	struct camss_isp_bufq_entry *entry = &bufq->entries[queue_idx];
	struct camss_isp_buf *buf;
	unsigned long flags;

	spin_lock_irqsave(&entry->rdy_spinlock, flags);
	buf = list_first_entry_or_null(&entry->rdy_queue,
				       struct camss_isp_buf, list);
	spin_unlock_irqrestore(&entry->rdy_spinlock, flags);

	return buf ? &buf->vb : NULL;
}
EXPORT_SYMBOL_GPL(camss_isp_bufq_next);

struct vb2_v4l2_buffer *camss_isp_bufq_remove(struct camss_isp_bufq *bufq, unsigned int queue_idx)
{
	struct camss_isp_bufq_entry *entry = &bufq->entries[queue_idx];
	struct camss_isp_buf *buf;
	unsigned long flags;

	spin_lock_irqsave(&entry->rdy_spinlock, flags);
	buf = list_first_entry_or_null(&entry->rdy_queue,
				       struct camss_isp_buf, list);
	if (buf) {
		list_del(&buf->list);
		entry->num_rdy--;
	}
	spin_unlock_irqrestore(&entry->rdy_spinlock, flags);

	return buf ? &buf->vb : NULL;
}
EXPORT_SYMBOL_GPL(camss_isp_bufq_remove);

void camss_isp_bufq_drain(struct camss_isp_bufq *bufq, unsigned int queue_idx,
			  enum vb2_buffer_state state)
{
	struct vb2_v4l2_buffer *vbuf;

	while ((vbuf = camss_isp_bufq_remove(bufq, queue_idx)))
		camss_isp_buf_done(vbuf, state);
}
EXPORT_SYMBOL_GPL(camss_isp_bufq_drain);

MODULE_DESCRIPTION("CAMSS ISP per-queue ready-buffer FIFO");
MODULE_LICENSE("GPL");
