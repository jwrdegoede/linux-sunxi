/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * CAMSS ISP scheduler helper — ISP job scheduling
 *
 * Tracks which context is currently running on the hardware and
 * serialises job execution. This is a pure helper: it has no knowledge
 * of buffers, vb2 queues, or the uAPI.  Drivers call these functions
 * explicitly from their own code paths.
 *
 * Usage pattern:
 *   - Embed struct camss_isp_sched in the driver's device struct.
 *   - Call camss_isp_sched_init() at probe time.
 *   - Call camss_isp_job_init() with ready_fn/run_fn/abort_fn/priv.
 *   - Call camss_isp_sched_try_run() from buf_queue / streamon to start jobs.
 *   - Call camss_isp_sched_job_finish() from the IRQ handler when done.
 *   - Call camss_isp_sched_cancel() from streamoff / release.
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef _CAMSS_ISP_SCHED_H
#define _CAMSS_ISP_SCHED_H

#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

/**
 * struct camss_isp_job_ops - per-job operation callbacks
 *
 * @ready:  Optional; return %true if the job can be submitted to hardware.
 *          Called outside the scheduler spinlock.  May be NULL (always ready).
 * @run:    Start the hardware for this job.  Called from workqueue context.
 *          @ctx_changed is %true when this job differs from the previously
 *          run job (i.e. first run ever, or a different context took over).
 * @abort:  Optional; abort a running job (e.g. trigger a HW reset).
 *          Called from process context during camss_isp_sched_cancel().
 *          May be NULL.
 */
struct camss_isp_job_ops {
	bool	(*ready)(void *priv);
	void	(*run)(void *priv, bool ctx_changed);
	void	(*abort)(void *priv);
};

/**
 * struct camss_isp_job - per-context scheduler state
 *
 * Embed one of these in the driver's per-context struct.
 * Initialise with camss_isp_job_init().
 *
 * @queue:     Entry in the scheduler's pending-job list.
 * @flags:     Internal state flags (ISP_JOB_*).
 * @finished:  Wait queue signalled when the running job completes.
 * @ops:   Job operation callbacks (ready/run/abort).
 * @priv:  Opaque pointer passed to all callbacks.
 */
struct camss_isp_job {
	struct list_head	queue;
	unsigned long		flags;
	wait_queue_head_t	finished;
	const struct camss_isp_job_ops *ops;
	void			*priv;
};

/**
 * struct camss_isp_sched - ISP job scheduler
 *
 * Embed one of these in the driver's device struct.
 * Initialise with camss_isp_sched_init().
 *
 * @curr_job:    Job currently running on hardware (NULL if idle).
 * @prev_job:    Job that ran most recently (never dereferenced, pointer only).
 * @pending:     List of jobs waiting to run.
 * @lock:        Protects @curr_job, @pending, and @flags.
 * @work:        Work item used to run jobs from non-atomic context.
 * @flags:       Scheduler-level flags (ISP_SCHED_PAUSED).
 */
struct camss_isp_sched {
	struct camss_isp_job	*curr_job;
	struct camss_isp_job	*prev_job;
	struct list_head	pending;
	spinlock_t		lock;
	struct work_struct	work;
	unsigned long		flags;
};

/**
 * camss_isp_sched_init() - initialise a scheduler
 * @sched: scheduler to initialise
 */
void camss_isp_sched_init(struct camss_isp_sched *sched);

/**
 * camss_isp_sched_destroy() - destroy a scheduler (waits for any running job)
 * @sched: scheduler to destroy
 */
void camss_isp_sched_destroy(struct camss_isp_sched *sched);

/**
 * camss_isp_job_init() - initialise per-context job state
 * @job:  job to initialise
 * @ops:  operation callbacks (run is required; ready and abort may be NULL)
 * @priv: opaque pointer passed to all callbacks
 */
void camss_isp_job_init(struct camss_isp_job *job,
		       const struct camss_isp_job_ops *ops,
		       void *priv);

/**
 * camss_isp_sched_try_run() - enqueue a job and try to start it
 * @sched: scheduler
 * @job:   job to enqueue; callbacks and @priv are taken from the job.
 *
 * Calls @job->ready_fn (if set); returns immediately if it returns %false.
 * Otherwise enqueues the job and starts it if the hardware is idle.
 * Safe to call from atomic context.
 */
void camss_isp_sched_try_run(struct camss_isp_sched *sched,
			     struct camss_isp_job *job);

/**
 * camss_isp_sched_job_finish() - signal that the current job has completed
 * @sched: scheduler
 * @job:   job that just finished (must be the currently running job)
 * @requeue: if %true and the job's ready_fn passes, immediately re-enqueue
 *           the job so the next frame starts as soon as the workqueue runs.
 *
 * Clears the running state, wakes any waiter in camss_isp_sched_cancel(),
 * and schedules the next pending job via the work queue.
 * Safe to call from atomic/IRQ context.
 */
void camss_isp_sched_job_finish(struct camss_isp_sched *sched,
				struct camss_isp_job *job,
				bool requeue);

/**
 * camss_isp_sched_cancel() - cancel a pending or running job and wait
 * @sched: scheduler
 * @job:   job to cancel; @job->abort_fn is called if the job is running.
 *
 * If the job is queued but not yet running, it is simply removed.
 * If the job is running, @job->abort_fn is called (if set) and the
 * function blocks until camss_isp_sched_job_finish() is called.
 * Must be called from process context (may sleep).
 */
void camss_isp_sched_cancel(struct camss_isp_sched *sched,
			    struct camss_isp_job *job);

/**
 * camss_isp_sched_suspend() - pause the scheduler and wait for current job
 * @sched: scheduler
 *
 * No new jobs will be started until camss_isp_sched_resume() is called.
 * Blocks until any currently running job finishes.
 */
void camss_isp_sched_suspend(struct camss_isp_sched *sched);

/**
 * camss_isp_sched_resume() - resume the scheduler
 * @sched: scheduler
 */
void camss_isp_sched_resume(struct camss_isp_sched *sched);

/**
 * camss_isp_sched_is_running() - check if a job is currently running
 * @sched: scheduler
 * @job:   job to check
 */
bool camss_isp_sched_is_running(struct camss_isp_sched *sched,
				struct camss_isp_job *job);

#endif /* _CAMSS_ISP_SCHED_H */
