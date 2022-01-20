// SPDX-License-Identifier: GPL-2.0

#include <asm/msr.h>
#include <linux/i2c.h>
#include <linux/psp-sev.h>

#include "i2c-designware-core.h"

#define MSR_AMD_PSP_ADDR	0xc00110a2
#define PSP_MBOX_OFFSET		0x10570
#define PSP_CMD_TIMEOUT_MS	500

#define PSP_I2C_REQ_BUS_CMD		0x64
#define PSP_I2C_REQ_RETRY_CNT		10
#define PSP_I2C_REQ_RETRY_DELAY_MSEC	50
#define PSP_I2C_REQ_STS_OK		0x0
#define PSP_I2C_REQ_STS_BUS_BUSY	0x1
#define PSP_I2C_REQ_STS_INV_PARAM	0x3

union psp_req_buffer_hdr {
	struct {
		u32 total_size;
		u32 status;
	} __packed;
	u64 hdr_val;
};

enum psp_i2c_req_type {
	PSP_I2C_REQ_ACQUIRE,
	PSP_I2C_REQ_RELEASE,
	PSP_I2C_REQ_MAX,
};

struct psp_i2c_req {
	union psp_req_buffer_hdr hdr;
	enum psp_i2c_req_type type;
} __packed __aligned(32);

union psp_mbox_cmd_reg {
	struct psp_mbox_cmd_fields {
		u16 mbox_status;
		u8 mbox_cmd;
		u8 reserved:6;
		u8 recovery:1;
		u8 ready:1;
	} __packed fields;
	u32 val;
};

struct psp_mbox {
	union psp_mbox_cmd_reg fields;
	uintptr_t i2c_req_addr;
} __packed;

static DEFINE_MUTEX(psp_i2c_access_mutex);
static unsigned long psp_i2c_sem_acquired;
static void __iomem *mbox_iomem;
static u32 psp_i2c_access_count;
static bool psp_i2c_mbox_fail;
static struct device *psp_i2c_dev;

static int psp_get_mbox_addr(unsigned long *mbox_addr)
{
	unsigned long long psp_mmio;

	if (rdmsrl_safe(MSR_AMD_PSP_ADDR, &psp_mmio))
		return -EIO;

	*mbox_addr = (unsigned long)(psp_mmio + PSP_MBOX_OFFSET);

	return 0;
}

static int psp_mbox_probe(void)
{
	unsigned long mbox_addr;

	if (psp_get_mbox_addr(&mbox_addr))
		return -1;

	mbox_iomem = ioremap(mbox_addr, sizeof(struct psp_mbox));
	if (!mbox_iomem)
		return -ENOMEM;

	return 0;
}

/* Recovery field should be equal 0 to start sending commands */
static int psp_check_mbox_recovery(struct psp_mbox *mbox)
{
	union psp_mbox_cmd_reg tmp = {0};

	tmp.val = readl(&mbox->fields.val);
	return !!tmp.fields.recovery;
}

static int psp_wait_cmd(struct psp_mbox *mbox)
{
	union psp_mbox_cmd_reg expected = { .val = 0 };
	u32 tmp;

	/* Expect mbox_cmd to be cleared and ready bit to be set by PSP */
	expected.fields.ready = 1;

	return readl_poll_timeout(&mbox->fields.val, tmp, (tmp == expected.val),
				  0, 1000 * PSP_CMD_TIMEOUT_MS);
}

/* Status equal to 0 means that PSP succeed processing command */
static int psp_check_mbox_sts(struct psp_mbox *mbox)
{
	union psp_mbox_cmd_reg cmd_reg = {0};

	cmd_reg.val = readl(&mbox->fields.val);
	return cmd_reg.fields.mbox_status;
}

static int psp_send_cmd(struct psp_i2c_req *req)
{
	struct psp_mbox *mbox = (struct psp_mbox *)mbox_iomem;
	union psp_mbox_cmd_reg cmd_reg = {0};

	if (psp_check_mbox_recovery(mbox))
		return -EIO;

	if (psp_wait_cmd(mbox))
		return -EBUSY;

	/* Fill address of command-response buffer */
	writeq((uintptr_t)__psp_pa((void *)req), &mbox->i2c_req_addr);

	/* Write command register to trigger processing */
	cmd_reg.fields.mbox_cmd = PSP_I2C_REQ_BUS_CMD;
	writel(cmd_reg.val, &mbox->fields.val);

	if (psp_wait_cmd(mbox))
		return -ETIMEDOUT;

	if (psp_check_mbox_sts(mbox))
		return -EIO;

	return 0;
}

/* Helper to verify status returned by PSP */
static int check_i2c_req_sts(struct psp_i2c_req *req)
{
	int status;

	status = readl(&req->hdr.status);

	switch (status) {
	case PSP_I2C_REQ_STS_OK:
		return 0;
	case PSP_I2C_REQ_STS_BUS_BUSY:
		return -EBUSY;
	case PSP_I2C_REQ_STS_INV_PARAM:
	default:
		return -EIO;
	};
}

static int psp_send_i2c_req(enum psp_i2c_req_type i2c_req_type)
{
	int status, ret, retry_cnt = PSP_I2C_REQ_RETRY_CNT;
	struct psp_i2c_req *req;
	unsigned long start;

	/* Allocate command-response buffer */
	req = kzalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	req->hdr.total_size = sizeof(*req);
	req->type = i2c_req_type;

	start = jiffies;
	do {
		if (psp_send_cmd(req)) {
			ret = -EIO;
			goto cleanup;
		}

		status = check_i2c_req_sts(req);
		if (!status) {
			dev_dbg(psp_i2c_dev, "Request accepted by PSP after %ums\n",
				jiffies_to_msecs(jiffies - start));
			ret = 0;
			goto cleanup;
		} else if (status == -EBUSY) {
			retry_cnt--;
		} else {
			ret = -EIO;
			goto cleanup;
		};

		/* IF EBUSY, give PSP time to finish its i2c activities */
		mdelay(PSP_I2C_REQ_RETRY_DELAY_MSEC);
	} while (retry_cnt);


	ret = -ETIMEDOUT;

cleanup:
	kfree(req);
	return ret;
}

static int psp_acquire_i2c_bus(void)
{
	int status;

	mutex_lock(&psp_i2c_access_mutex);

	/* Return early if mailbox malfunctioned */
	if (psp_i2c_mbox_fail)
		goto cleanup;

	/*
	 * Simply increment usage counter and return if PSP semaphore was
	 * already taken by kernel
	 */
	if (psp_i2c_access_count > 0) {
		psp_i2c_access_count++;
		goto cleanup;
	};

	status = psp_send_i2c_req(PSP_I2C_REQ_ACQUIRE);
	if (!status) {
		psp_i2c_sem_acquired = jiffies;
		psp_i2c_access_count++;
		goto cleanup;
	} else if (status == -ETIMEDOUT) {
		dev_err(psp_i2c_dev, "Timed out waiting for PSP to release I2C bus\n");
	} else {
		dev_err(psp_i2c_dev, "PSP communication error\n");
	};

	dev_err(psp_i2c_dev, "Assume i2c bus is for exclusive host usage\n");
	psp_i2c_mbox_fail = true;

cleanup:
	mutex_unlock(&psp_i2c_access_mutex);
	return 0;
}

static void psp_release_i2c_bus(void)
{
	int status;

	mutex_lock(&psp_i2c_access_mutex);

	/* Return early if mailbox was malfunctional */
	if (psp_i2c_mbox_fail)
		goto cleanup;

	/*
	 * If we are last owner of PSP semaphore, need to release aribtration
	 * via mailbox
	 */
	psp_i2c_access_count--;
	if (psp_i2c_access_count > 0)
		goto cleanup;

	/* Send a release command to PSP */
	status = psp_send_i2c_req(PSP_I2C_REQ_RELEASE);
	if (!status) {
		dev_dbg(psp_i2c_dev, "PSP semaphore held for %ums\n",
			jiffies_to_msecs(jiffies - psp_i2c_sem_acquired));
		goto cleanup;
	} else if (status == -ETIMEDOUT) {
		dev_err(psp_i2c_dev, "Timed out waiting for PSP to acquire I2C bus\n");
	} else {
		dev_err(psp_i2c_dev, "PSP communication error\n");
	}

	dev_err(psp_i2c_dev, "Assume i2c bus is for exclusive host usage\n");
	psp_i2c_mbox_fail = true;

cleanup:
	mutex_unlock(&psp_i2c_access_mutex);
}

/*
 * Locking methods are based on the default implementation from
 * drivers/i2c/i2c-core.base.c, but with psp acquire and release operations
 * added. With this in place we can ensure that i2c clients on the bus shared
 * with psp are able to lock HW access to the bus for arbitrary number of
 * operations - that is e.g. write-wait-read.
 */
static void i2c_adapter_dw_psp_lock_bus(struct i2c_adapter *adapter,
					unsigned int flags)
{
	psp_acquire_i2c_bus();
	rt_mutex_lock_nested(&adapter->bus_lock, i2c_adapter_depth(adapter));
}

static int i2c_adapter_dw_psp_trylock_bus(struct i2c_adapter *adapter,
					  unsigned int flags)
{
	int ret;

	ret = rt_mutex_trylock(&adapter->bus_lock);
	if (!ret)
		psp_acquire_i2c_bus();

	return ret;
}

static void i2c_adapter_dw_psp_unlock_bus(struct i2c_adapter *adapter,
					  unsigned int flags)
{
	psp_release_i2c_bus();
	rt_mutex_unlock(&adapter->bus_lock);
}

static const struct i2c_lock_operations i2c_dw_psp_lock_ops = {
	.lock_bus = i2c_adapter_dw_psp_lock_bus,
	.trylock_bus = i2c_adapter_dw_psp_trylock_bus,
	.unlock_bus = i2c_adapter_dw_psp_unlock_bus,
};

int i2c_dw_amdpsp_probe_lock_support(struct dw_i2c_dev *dev)
{
	if (!dev || !dev->dev)
		return -ENODEV;

	if (!(dev->flags & ARBITRATION_SEMAPHORE))
		return -ENODEV;

	/* Allow to bind only one instance of a driver */
	if (!psp_i2c_dev)
		psp_i2c_dev = dev->dev;
	else
		return -EEXIST;

	if (psp_mbox_probe())
		return -EIO;

	dev_info(psp_i2c_dev, "I2C bus managed by AMD PSP\n");

	/*
	 * Install global locking callbacks for adapter as well as internal i2c
	 * controller locks
	 */
	dev->adapter.lock_ops = &i2c_dw_psp_lock_ops;
	dev->acquire_lock = psp_acquire_i2c_bus;
	dev->release_lock = psp_release_i2c_bus;

	return 0;
}

/* Unmap area used as a mailbox with PSP */
void i2c_dw_amdpsp_remove_lock_support(struct dw_i2c_dev *dev)
{
	iounmap(mbox_iomem);
}
