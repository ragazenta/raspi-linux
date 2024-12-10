/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (c) 2024 Jeremy Garff <jer @ jers.net>
 *
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/miscdevice.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/of_dma.h>
#include <linux/dma-mapping.h>

#include "pwm-ws281x-rp1.h"


#define PWM_WS281X_RP1_DRIVER_VERSION            0x00000100

//
// PWM Hardware Register Structures
// See https://datasheets.raspberrypi.com/rp1/rp1-peripherals.pdf
//
typedef struct {
    uint32_t ctrl;
#define PWM_RP1_REGS_CHAN_CTRL_MODE_0            ((0x0 & 0x7) << 0)
#define PWM_RP1_REGS_CHAN_CTRL_MODE_TEMS         ((0x1 & 0x7) << 0)
#define PWM_RP1_REGS_CHAN_CTRL_MODE_PCMS         ((0x2 & 0x7) << 0)
#define PWM_RP1_REGS_CHAN_CTRL_MODE_PDEO         ((0x3 & 0x7) << 0)
#define PWM_RP1_REGS_CHAN_CTRL_MODE_MSBS         ((0x4 & 0x7) << 0)
#define PWM_RP1_REGS_CHAN_CTRL_MODE_PPMO         ((0x5 & 0x7) << 0)
#define PWM_RP1_REGS_CHAN_CTRL_MODE_LEMS         ((0x6 & 0x7) << 0)
#define PWM_RP1_REGS_CHAN_CTRL_MODE_LSBO         ((0x7 & 0x7) << 0)
#define PWM_RP1_REGS_CHAN_CTRL_INVERT            (1 << 3)
#define PWM_RP1_REGS_CHAN_CTRL_BIND              (1 << 4)
#define PWM_RP1_REGS_CHAN_CTRL_USEFIFO           (1 << 5)
#define PWM_RP1_REGS_CHAN_CTRL_SDM               (1 << 6)
#define PWM_RP1_REGS_CHAN_CTRL_DITHER            (1 << 7)
#define PWM_RP1_REGS_CHAN_CTRL_FIFO_POP          (1 << 8)
#define PWM_RP1_REGS_CHAN_CTRL_SDM_BITWIDTH(val) (((val) & 0xf) << 12)
#define PWM_RP1_REGS_CHAN_CTRL_SDM_BIAS(val)     (((val) & 0xffff) << 16)
    uint32_t range;
    uint32_t phase;
    uint32_t duty;
} __attribute__((packed)) pwm_rp1_chan_t;

typedef struct {
    uint32_t global_ctrl;
#define PWM_RP1_REGS_GLOBAL_CTRL_CHAN_EN(chan)   (1 << chan)
#define PWM_RP1_REGS_GLOBAL_CTRL_SET_UPDATE      (1 << 31)
    uint32_t fifo_ctrl;
#define PWM_RP1_REGS_FIFO_CTRL_LEVEL(reg)        ((reg & 0x1f) >> 0)
#define PWM_RP1_REGS_FIFO_CTRL_FLUSH             (1 << 5)
#define PWM_RP1_REGS_FIFO_CTRL_FLUSH_DONE        (1 << 6)
#define PWM_RP1_REGS_FIFO_CTRL_THRESHOLD(val)    (((val) & 0x1f) << 11)
#define PWM_RP1_REGS_FIFO_CTRL_DWELL_TIME(val)   (((val) & 0x1f) << 16)
#define PWM_RP1_REGS_FIFO_CTRL_DREQ_EN           (1 << 31)
    uint32_t common_range;
    uint32_t common_duty;
    uint32_t duty_fifo;
    pwm_rp1_chan_t chan[4];
    uint32_t intr;                               // Raw interrupts
    uint32_t inte;                               // Interrupt Enable
    uint32_t intf;                               // Interrupt Force
    uint32_t ints;                               // Interrupt status after mask and forcing
} __attribute__((packed)) pwm_rp1_regs_t;


//
// WS281X_PWM Device Driver Structure
//
typedef struct pwm_rp1_device {
    volatile uint32_t flags;
#define PWM_WS281X_RP1_DEVICE_FLAGS_BUSY         (1 << 0)
    volatile uint32_t active;
#define PWM_WS281X_RP1_DEVICE_ACTIVE             (1 << 0)
    struct miscdevice mdev;
    pwm_rp1_regs_t __iomem *regs;
	struct clk *clk;
    struct platform_device *pdev;
    struct resource *res;
    struct dma_chan *chan;
    void *dmabuf;
    struct mutex *lock;
    wait_queue_head_t *wq;
    struct sg_table dma_table;
    int page_count;
    struct page *pages[SG_CHUNK_SIZE];
} pwm_ws281x_rp1_device_t;


//
// Global Variables
//
DECLARE_WAIT_QUEUE_HEAD(pwm_ws281x_rp1_wq);
DEFINE_MUTEX(pwm_ws281x_rp1_lock);
pwm_ws281x_rp1_device_t pwm_ws281x_rp1 = {
    .lock = &pwm_ws281x_rp1_lock,
    .wq = &pwm_ws281x_rp1_wq,
    .flags = 0,
    .active = 0,
};

static int pwm_channel = 2;

//
// PWM Channel Setup
//
void pwm_ws281x_rp1_chan(int channel, int invert) {
    uint32_t tmp;

    tmp = PWM_RP1_REGS_CHAN_CTRL_MODE_MSBS |
          PWM_RP1_REGS_CHAN_CTRL_USEFIFO |
          PWM_RP1_REGS_CHAN_CTRL_BIND |
          PWM_RP1_REGS_CHAN_CTRL_FIFO_POP;

    if (invert) {
        tmp |= PWM_RP1_REGS_CHAN_CTRL_INVERT;
    }
          
    iowrite32(tmp, &pwm_ws281x_rp1.regs->chan[channel].ctrl);
}

//
// PWM Controller Setup
//
void pwm_ws281x_rp1_init(int channel, int invert) {
    pwm_ws281x_rp1_chan(channel, invert);

    // Set the range to 32-bits since we're sending data through
    // the FIFO 32-bits per DMA cycle.
    iowrite32(31, &pwm_ws281x_rp1.regs->common_range);

    iowrite32(PWM_RP1_REGS_FIFO_CTRL_DWELL_TIME(2) |
              PWM_RP1_REGS_FIFO_CTRL_THRESHOLD(8) |
              PWM_RP1_REGS_FIFO_CTRL_DREQ_EN,
              &pwm_ws281x_rp1.regs->fifo_ctrl);

    iowrite32(PWM_RP1_REGS_GLOBAL_CTRL_CHAN_EN(channel) |
              PWM_RP1_REGS_GLOBAL_CTRL_SET_UPDATE,
              &pwm_ws281x_rp1.regs->global_ctrl);

    // Set the level to zero
    iowrite32(0, &pwm_ws281x_rp1.regs->duty_fifo);
}

void pwm_ws281x_rp1_cleanup(void) {
    // Set level back to zero
    iowrite32(0, &pwm_ws281x_rp1.regs->duty_fifo);

    iowrite32(PWM_RP1_REGS_GLOBAL_CTRL_SET_UPDATE,
              &pwm_ws281x_rp1.regs->global_ctrl);
}


//
// Character device file operations
//
int pwm_ws281x_rp1_open(struct inode *inode, struct file *file) {
    if (mutex_lock_interruptible(pwm_ws281x_rp1.lock)) {
        return -EINTR;
    }

    // Only one user at a time
    if (pwm_ws281x_rp1.flags & PWM_WS281X_RP1_DEVICE_FLAGS_BUSY) {
        mutex_unlock(pwm_ws281x_rp1.lock);
        return -EBUSY;
    }
    pwm_ws281x_rp1.flags |= PWM_WS281X_RP1_DEVICE_FLAGS_BUSY;

    file->private_data = &pwm_ws281x_rp1;

    mutex_unlock(pwm_ws281x_rp1.lock);

    pwm_ws281x_rp1_init(pwm_channel, 0);

    return 0;
}

int pwm_ws281x_rp1_release(struct inode *inode, struct file *file) {
    int retval;

    if (mutex_lock_interruptible(pwm_ws281x_rp1.lock)) {
        return -EINTR;
    }

    pwm_ws281x_rp1.flags &= ~PWM_WS281X_RP1_DEVICE_FLAGS_BUSY;

    retval = wait_event_interruptible(*pwm_ws281x_rp1.wq, !(pwm_ws281x_rp1.active & PWM_WS281X_RP1_DEVICE_ACTIVE));
    if (retval) {  // Ctrl-C / Kill
        dmaengine_terminate_sync(pwm_ws281x_rp1.chan);
        pwm_ws281x_rp1.flags &= ~PWM_WS281X_RP1_DEVICE_ACTIVE;
    }

    mutex_unlock(pwm_ws281x_rp1.lock);

    return 0;
}

long pwm_ws281x_rp1_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    uint32_t ver = PWM_WS281X_RP1_DRIVER_VERSION;
    pwm_ws281x_rp1_ioctl_reg_t reg;

    switch (cmd) {
        case PWM_WS281X_RP1_IOCTL_VERSION:
            if (copy_to_user((uint32_t *)arg, &ver, sizeof(ver))) {
                return -EACCES;
            }
            break;

        case PWM_WS281X_RP1_IOCTL_REG_READ:
            if (copy_from_user(&reg, (pwm_ws281x_rp1_ioctl_reg_t *)arg, sizeof(reg))) {
                return -EACCES;
            }

            // Bounds check the register space
            if (reg.reg_offset >= sizeof(pwm_rp1_regs_t)) {
                return -EINVAL;
            }

            reg.reg_value = ioread32((uint8_t *)pwm_ws281x_rp1.regs + reg.reg_offset);

            if (copy_to_user((pwm_ws281x_rp1_ioctl_reg_t *)arg, &reg, sizeof(reg))) {
                return -EACCES;
            }

            break;

        case PWM_WS281X_RP1_IOCTL_REG_WRITE:
            if (copy_from_user(&reg, (pwm_ws281x_rp1_ioctl_reg_t *)arg, sizeof(reg))) {
                return -EACCES;
            }

            // Bounds check the register space
            if (reg.reg_offset >= sizeof(pwm_rp1_regs_t)) {
                return -EINVAL;
            }

            iowrite32(reg.reg_value, (uint8_t *)pwm_ws281x_rp1.regs + reg.reg_offset);

            break;

        default:
            return -EINVAL;
    }

    return 0;
}

void rp1_ws281x_dma_callback(void *param) {
    pwm_ws281x_rp1.active &= ~PWM_WS281X_RP1_DEVICE_ACTIVE;
    wake_up(pwm_ws281x_rp1.wq);
}

ssize_t rp1_ws281x_dma(const char *buf, ssize_t len) {
    uint64_t first = (uint64_t)buf >> PAGE_SHIFT;
    uint64_t last = ((uint64_t)buf + (len - 1)) >> PAGE_SHIFT;
    int offset = (uint64_t)buf % PAGE_SIZE;
    int count = last - first + 1;
    struct dma_async_tx_descriptor *desc;
    int retval = 0;

    if (count > ARRAY_SIZE(pwm_ws281x_rp1.pages)) {
        count = ARRAY_SIZE(pwm_ws281x_rp1.pages);
        len = (PAGE_SIZE - offset) + ((count - 1) * PAGE_SIZE);
    }

    retval = pin_user_pages_fast((uint64_t)buf, count, 0, pwm_ws281x_rp1.pages);
    if (retval != count) {
        if (retval) {
            unpin_user_pages(pwm_ws281x_rp1.pages, retval);
        }

        dev_err(&pwm_ws281x_rp1.pdev->dev, "Failed to map user pages %d %d\n", retval, count);
        return -ENOBUFS;
    }

    retval = sg_alloc_table_from_pages(&pwm_ws281x_rp1.dma_table, pwm_ws281x_rp1.pages,
                                       count, offset, len, GFP_KERNEL);
    if (retval) {
        unpin_user_pages(pwm_ws281x_rp1.pages, count);

        return -ENOBUFS;
    }

    retval = dma_map_sgtable(&pwm_ws281x_rp1.pdev->dev, &pwm_ws281x_rp1.dma_table, DMA_TO_DEVICE, 0);
    if (retval) {
        sg_free_table(&pwm_ws281x_rp1.dma_table);
        unpin_user_pages(pwm_ws281x_rp1.pages, count);

        return -ENOBUFS;
    }

    pwm_ws281x_rp1.active |= PWM_WS281X_RP1_DEVICE_ACTIVE;

    desc = dmaengine_prep_slave_sg(pwm_ws281x_rp1.chan, pwm_ws281x_rp1.dma_table.sgl,
                                   pwm_ws281x_rp1.dma_table.nents, DMA_MEM_TO_DEV, 0);
    if (!desc) {
        len = -ENOBUFS;
        goto cleanup;
    }

    desc->callback = rp1_ws281x_dma_callback;
    desc->callback_param = NULL;

    retval = dmaengine_submit(desc);
    if (retval < 0) {
        len = -ENOBUFS;

        goto cleanup;
    }
    dma_async_issue_pending(pwm_ws281x_rp1.chan);

    // Wait for the DMA to complete, this ensures the user can't mess with the pinned memory,
    // at least in the writer thread context.
    retval = wait_event_interruptible(*pwm_ws281x_rp1.wq, !(pwm_ws281x_rp1.active & PWM_WS281X_RP1_DEVICE_ACTIVE));
    if (retval) {  // Ctrl-C / Kill
        dmaengine_terminate_sync(pwm_ws281x_rp1.chan);
        pwm_ws281x_rp1.flags &= ~PWM_WS281X_RP1_DEVICE_ACTIVE;
        len = -ERESTARTSYS;
        goto cleanup;
    }

cleanup:
    dma_unmap_sgtable(&pwm_ws281x_rp1.pdev->dev, &pwm_ws281x_rp1.dma_table, DMA_TO_DEVICE, 0);
    sg_free_table(&pwm_ws281x_rp1.dma_table);
    unpin_user_pages(pwm_ws281x_rp1.pages, count);

    return len;
}

ssize_t pwm_ws281x_rp1_write(struct file *file, const char *buf, size_t total, loff_t *loff) {
    ssize_t len = 0;

    if (mutex_lock_interruptible(pwm_ws281x_rp1.lock)) {
        return -EINTR;
    }

    while (len < total) {
        ssize_t retval = rp1_ws281x_dma(buf + len, total - len);
        if (retval <= 0) {
            mutex_unlock(pwm_ws281x_rp1.lock);
            return retval;
        }

        len += retval;
    }

    mutex_unlock(pwm_ws281x_rp1.lock);
 
    return len;
}


static struct file_operations pwm_ws281x_rp1_fops = {
    .owner = THIS_MODULE,
    .open = pwm_ws281x_rp1_open,
    .release = pwm_ws281x_rp1_release,
    .write = pwm_ws281x_rp1_write,
    .unlocked_ioctl = pwm_ws281x_rp1_ioctl,
};


/*
 * Driver Probe / Init
 */
int pwm_ws281x_rp1_probe(struct platform_device *pdev) {
    struct dma_slave_config dma_conf = {
        .dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
        .src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
        .direction = DMA_MEM_TO_DEV,
        .dst_maxburst = 8,
        .dst_port_window_size = 1,
        .device_fc = false,
    };
    int result;

    pwm_ws281x_rp1.pdev = pdev;

    pwm_ws281x_rp1.res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!pwm_ws281x_rp1.res) {
        dev_err(&pdev->dev, "%s: Failed to get platform memory region\n", DEVICE_NAME);
        return -EIO;
    }

    // Setup the target fifo register address for the DMA operations
    dma_conf.dst_addr = pwm_ws281x_rp1.res->start + offsetof(pwm_rp1_regs_t, duty_fifo);

    pwm_ws281x_rp1.regs = devm_ioremap_resource(&pdev->dev, pwm_ws281x_rp1.res);
    if (!pwm_ws281x_rp1.regs) {
        dev_err(&pdev->dev, "%s: Failed to request register space\n", DEVICE_NAME);
        return -EIO;
    }

    pwm_ws281x_rp1.clk = devm_clk_get(&pdev->dev, NULL);
    if (IS_ERR(pwm_ws281x_rp1.clk)) {
        dev_err(&pdev->dev, "Clock not provided\n");
        devm_iounmap(&pdev->dev, pwm_ws281x_rp1.regs);
        return -ENODEV;
    }

    result = clk_prepare_enable(pwm_ws281x_rp1.clk);
    if (result) {
        clk_disable_unprepare(pwm_ws281x_rp1.clk);
        devm_iounmap(&pdev->dev, pwm_ws281x_rp1.regs);
        dev_err(&pdev->dev, "Clock could not be enabled\n");
        return result;
    }

    pwm_ws281x_rp1.chan = dma_request_chan(&pdev->dev, "pwm0");
    if (IS_ERR(pwm_ws281x_rp1.chan)) {
        clk_disable_unprepare(pwm_ws281x_rp1.clk);
        devm_iounmap(&pdev->dev, pwm_ws281x_rp1.regs);
        dev_err(&pdev->dev, "Unable to allocate dma channel\n");
        return -ENODEV;
    }

    result = dmaengine_slave_config(pwm_ws281x_rp1.chan, &dma_conf);
    if (result) {
        dma_release_channel(pwm_ws281x_rp1.chan);
        clk_disable_unprepare(pwm_ws281x_rp1.clk);
        devm_iounmap(&pdev->dev, pwm_ws281x_rp1.regs);
        dev_err(&pdev->dev, "Unable to allocate dma channel\n");
        return -ENODEV;
    }

    result = dma_set_mask(&pdev->dev, DMA_BIT_MASK(64));
    if (result) {
        dma_release_channel(pwm_ws281x_rp1.chan);
        clk_disable_unprepare(pwm_ws281x_rp1.clk);
        devm_iounmap(&pdev->dev, pwm_ws281x_rp1.regs);
        dev_err(&pdev->dev, "Unable to allocate dma channel\n");
        return -ENODEV;
    }

    pwm_ws281x_rp1.mdev.minor = MISC_DYNAMIC_MINOR;
    pwm_ws281x_rp1.mdev.name = DEVICE_NAME;
    pwm_ws281x_rp1.mdev.fops = &pwm_ws281x_rp1_fops;

    if (misc_register(&pwm_ws281x_rp1.mdev)) {
        dma_release_channel(pwm_ws281x_rp1.chan);
        clk_disable_unprepare(pwm_ws281x_rp1.clk);
        devm_iounmap(&pdev->dev, pwm_ws281x_rp1.regs);
        return -ENODEV;
    }

    platform_set_drvdata(pdev, &pwm_ws281x_rp1);

    pwm_ws281x_rp1_init(pwm_channel, 0);

    return 0;
}

int pwm_ws281x_rp1_remove(struct platform_device *pdev) {
    pwm_ws281x_rp1_cleanup();

    misc_deregister(&pwm_ws281x_rp1.mdev);
    dmaengine_terminate_sync(pwm_ws281x_rp1.chan);
    dma_release_channel(pwm_ws281x_rp1.chan);
    clk_disable_unprepare(pwm_ws281x_rp1.clk);
    devm_iounmap(&pdev->dev, pwm_ws281x_rp1.regs);

    pwm_ws281x_rp1.pdev = NULL;

    return 0;
}

static const struct of_device_id pwm_ws281x_rp1_of_match[] = {
	{ .compatible = "pwm-ws281x-rp1" },
	{ }
};
MODULE_DEVICE_TABLE(of, pwm_ws281x_rp1_of_match);

static struct platform_driver pwm_ws281x_rp1_driver = {
	.driver = {
		.name = "pwm-ws281x-rp1",
		.of_match_table = pwm_ws281x_rp1_of_match,
	},
	.probe = pwm_ws281x_rp1_probe,
	.remove = pwm_ws281x_rp1_remove,
};
module_platform_driver(pwm_ws281x_rp1_driver);

module_param(pwm_channel, int, 0644);

MODULE_AUTHOR("Jeremy Garff <jer @ jers.net>");
MODULE_LICENSE("GPL");
