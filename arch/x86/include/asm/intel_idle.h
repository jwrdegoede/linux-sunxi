#ifndef _ASM_X86_INTEL_IDLE_H
#define _ASM_X86_INTEL_IDLE_H

#include <linux/notifier.h>

#ifdef CONFIG_PM_DEBUG
void intel_idle_freeze_notifier_register(struct notifier_block *nb);
void intel_idle_freeze_notifier_unregister(struct notifier_block *nb);
#endif


#endif /* _ASM_X86_INTEL_IDLE_H */
