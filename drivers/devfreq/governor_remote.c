// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/devfreq.h>
#include <linux/devfreq-governor.h>

static int devfreq_remote_track_func(struct devfreq *devfreq, unsigned long *freq)
{
	if (!devfreq->profile->get_cur_freq)
		return -ENXIO;

	return devfreq->profile->get_cur_freq(devfreq->dev.parent, freq);
}

static int devfreq_remote_track_handler(struct devfreq *devfreq, unsigned int event, void *data)
{
	switch (event) {
	case DEVFREQ_GOV_START:
		devfreq_monitor_start(devfreq);
		break;

	case DEVFREQ_GOV_STOP:
		devfreq_monitor_stop(devfreq);
		break;

	case DEVFREQ_GOV_UPDATE_INTERVAL:
		devfreq_update_interval(devfreq, (unsigned int *)data);
		break;

	case DEVFREQ_GOV_SUSPEND:
		devfreq_monitor_suspend(devfreq);
		break;

	case DEVFREQ_GOV_RESUME:
		devfreq_monitor_resume(devfreq);
		break;
	}

	return 0;
}

static struct devfreq_governor devfreq_remote_track = {
	.name = DEVFREQ_GOV_REMOTE,
	.attrs = DEVFREQ_GOV_ATTR_POLLING_INTERVAL
		| DEVFREQ_GOV_ATTR_TIMER,
	.flags = DEVFREQ_GOV_FLAG_IMMUTABLE
		| DEVFREQ_GOV_FLAG_TRACK_REMOTE,
	.get_target_freq = devfreq_remote_track_func,
	.event_handler = devfreq_remote_track_handler,
};

static int __init devfreq_remote_track_init(void)
{
	return devfreq_add_governor(&devfreq_remote_track);
}
subsys_initcall(devfreq_remote_track_init);

static void __exit devfreq_remote_track_exit(void)
{
	int ret;

	ret = devfreq_remove_governor(&devfreq_remote_track);
	if (ret)
		pr_err("%s: failed to remove governor %d\n", __func__, ret);
}
module_exit(devfreq_remote_track_exit);

MODULE_DESCRIPTION("DEVFREQ Remote Tracking governor");
MODULE_LICENSE("GPL");
