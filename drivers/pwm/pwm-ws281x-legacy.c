/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Based on pwm-ws281x-rp1
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

#include "pwm-ws281x-legacy.h"


#define PWM_WS281X_LEGACY_DRIVER_VERSION            0x00000100

//
// PWM Hardware Register Structure
// See the BCM2835 ARM Peripherals datasheet, chapter 9, and the BCM2711 addendum.
//
// This is the pre-RP1 ("legacy") PWM block found on BCM2835/6/7 and BCM2711.
//
typedef struct {
    uint32_t ctl;
#define PWM_LEGACY_REGS_CTL_MSEN2                   (1 << 15)
#define PWM_LEGACY_REGS_CTL_USEF2                   (1 << 13)
#define PWM_LEGACY_REGS_CTL_POLA2                   (1 << 12)
#define PWM_LEGACY_REGS_CTL_SBIT2                   (1 << 11)
#define PWM_LEGACY_REGS_CTL_RPTL2                   (1 << 10)
#define PWM_LEGACY_REGS_CTL_MODE2                   (1 << 9)
#define PWM_LEGACY_REGS_CTL_PWEN2                   (1 << 8)
#define PWM_LEGACY_REGS_CTL_MSEN1                   (1 << 7)
#define PWM_LEGACY_REGS_CTL_CLRF1                   (1 << 6)
#define PWM_LEGACY_REGS_CTL_USEF1                   (1 << 5)
#define PWM_LEGACY_REGS_CTL_POLA1                   (1 << 4)
#define PWM_LEGACY_REGS_CTL_SBIT1                   (1 << 3)
#define PWM_LEGACY_REGS_CTL_RPTL1                   (1 << 2)
#define PWM_LEGACY_REGS_CTL_MODE1                   (1 << 1)
#define PWM_LEGACY_REGS_CTL_PWEN1                   (1 << 0)
    uint32_t sta;
#define PWM_LEGACY_REGS_STA_STA4                    (1 << 12)
#define PWM_LEGACY_REGS_STA_STA3                    (1 << 11)
#define PWM_LEGACY_REGS_STA_STA2                    (1 << 10)
#define PWM_LEGACY_REGS_STA_STA1                    (1 << 9)
#define PWM_LEGACY_REGS_STA_BERR                    (1 << 8)
#define PWM_LEGACY_REGS_STA_GAPO4                   (1 << 7)
#define PWM_LEGACY_REGS_STA_GAPO3                   (1 << 6)
#define PWM_LEGACY_REGS_STA_GAPO2                   (1 << 5)
#define PWM_LEGACY_REGS_STA_GAPO1                   (1 << 4)
#define PWM_LEGACY_REGS_STA_RERR1                   (1 << 3)
#define PWM_LEGACY_REGS_STA_WERR1                   (1 << 2)
#define PWM_LEGACY_REGS_STA_EMPT1                   (1 << 1)
#define PWM_LEGACY_REGS_STA_FULL1                   (1 << 0)
#define PWM_LEGACY_REGS_STA_ERRS                    (PWM_LEGACY_REGS_STA_BERR  | \
                                                     PWM_LEGACY_REGS_STA_GAPO4 | \
                                                     PWM_LEGACY_REGS_STA_GAPO3 | \
                                                     PWM_LEGACY_REGS_STA_GAPO2 | \
                                                     PWM_LEGACY_REGS_STA_GAPO1 | \
                                                     PWM_LEGACY_REGS_STA_RERR1 | \
                                                     PWM_LEGACY_REGS_STA_WERR1)
    uint32_t dmac;
#define PWM_LEGACY_REGS_DMAC_ENAB                   (1 << 31)
#define PWM_LEGACY_REGS_DMAC_PANIC(val)             (((val) & 0xff) << 8)
#define PWM_LEGACY_REGS_DMAC_DREQ(val)              (((val) & 0xff) << 0)
    uint32_t resvd_0x0c;
    uint32_t rng1;
    uint32_t dat1;
    uint32_t fif1;
    uint32_t resvd_0x1c;
    uint32_t rng2;
    uint32_t dat2;
} __attribute__((packed)) pwm_legacy_regs_t;

#define PWM_LEGACY_SERIALIZER_BITS                  32
#define PWM_LEGACY_FIFO_DREQ_THRESHOLD              3
#define PWM_LEGACY_FIFO_PANIC_THRESHOLD             7
#define PWM_LEGACY_CHANNEL_DEFAULT                  1
#define PWM_LEGACY_CHANNEL_MIN                      1
#define PWM_LEGACY_CHANNEL_MAX                      2
#define PWM_LEGACY_REGS_CTL_CHAN_SHIFT(chan)        (((chan) - 1) * 8)


//
// WS281X_PWM Device Driver Structure
//
typedef struct pwm_legacy_device {
    volatile uint32_t flags;
#define PWM_WS281X_LEGACY_DEVICE_FLAGS_BUSY         (1 << 0)
    volatile uint32_t active;
#define PWM_WS281X_LEGACY_DEVICE_ACTIVE             (1 << 0)
    struct miscdevice mdev;
    uint32_t channel;
    pwm_legacy_regs_t __iomem *regs;
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
} pwm_ws281x_legacy_device_t;


//
// Global Variables
//
DECLARE_WAIT_QUEUE_HEAD(pwm_ws281x_legacy_wq);
DEFINE_MUTEX(pwm_ws281x_legacy_lock);
pwm_ws281x_legacy_device_t pwm_ws281x_legacy = {
    .lock = &pwm_ws281x_legacy_lock,
    .wq = &pwm_ws281x_legacy_wq,
    .flags = 0,
    .active = 0,
    .channel = PWM_LEGACY_CHANNEL_DEFAULT,
};

//
// PWM Controller Setup
//
void pwm_ws281x_legacy_init(int invert) {
    uint32_t shift = PWM_LEGACY_REGS_CTL_CHAN_SHIFT(pwm_ws281x_legacy.channel);
    uint32_t ctl;

    iowrite32(0, &pwm_ws281x_legacy.regs->ctl);

    iowrite32(PWM_LEGACY_REGS_STA_ERRS, &pwm_ws281x_legacy.regs->sta);

    // Set the range to 32-bits since we're sending data through
    // the FIFO 32-bits per DMA cycle.
    iowrite32(PWM_LEGACY_SERIALIZER_BITS,
              pwm_ws281x_legacy.channel == 1 ? &pwm_ws281x_legacy.regs->rng1
                                             : &pwm_ws281x_legacy.regs->rng2);

    iowrite32(PWM_LEGACY_REGS_CTL_CLRF1, &pwm_ws281x_legacy.regs->ctl);

    iowrite32(PWM_LEGACY_REGS_DMAC_ENAB |
              PWM_LEGACY_REGS_DMAC_PANIC(PWM_LEGACY_FIFO_PANIC_THRESHOLD) |
              PWM_LEGACY_REGS_DMAC_DREQ(PWM_LEGACY_FIFO_DREQ_THRESHOLD),
              &pwm_ws281x_legacy.regs->dmac);

    ctl = PWM_LEGACY_REGS_CTL_USEF1 | PWM_LEGACY_REGS_CTL_MODE1;

    if (invert) {
        ctl |= PWM_LEGACY_REGS_CTL_POLA1;
    }

    ctl <<= shift;

    iowrite32(ctl, &pwm_ws281x_legacy.regs->ctl);
    iowrite32(ctl | (PWM_LEGACY_REGS_CTL_PWEN1 << shift), &pwm_ws281x_legacy.regs->ctl);
}

void pwm_ws281x_legacy_cleanup(void) {
    iowrite32(0, &pwm_ws281x_legacy.regs->dmac);
    iowrite32(0, &pwm_ws281x_legacy.regs->ctl);
}


//
// Character device file operations
//
int pwm_ws281x_legacy_open(struct inode *inode, struct file *file) {
    if (mutex_lock_interruptible(pwm_ws281x_legacy.lock)) {
        return -EINTR;
    }

    // Only one user at a time
    if (pwm_ws281x_legacy.flags & PWM_WS281X_LEGACY_DEVICE_FLAGS_BUSY) {
        mutex_unlock(pwm_ws281x_legacy.lock);
        return -EBUSY;
    }
    pwm_ws281x_legacy.flags |= PWM_WS281X_LEGACY_DEVICE_FLAGS_BUSY;

    file->private_data = &pwm_ws281x_legacy;

    mutex_unlock(pwm_ws281x_legacy.lock);

    pwm_ws281x_legacy_init(0);

    return 0;
}

int pwm_ws281x_legacy_release(struct inode *inode, struct file *file) {
    int retval;

    if (mutex_lock_interruptible(pwm_ws281x_legacy.lock)) {
        return -EINTR;
    }

    pwm_ws281x_legacy.flags &= ~PWM_WS281X_LEGACY_DEVICE_FLAGS_BUSY;

    retval = wait_event_interruptible(*pwm_ws281x_legacy.wq,
                                      !(pwm_ws281x_legacy.active & PWM_WS281X_LEGACY_DEVICE_ACTIVE));
    if (retval) {  // Ctrl-C / Kill
        dmaengine_terminate_sync(pwm_ws281x_legacy.chan);
        pwm_ws281x_legacy.active &= ~PWM_WS281X_LEGACY_DEVICE_ACTIVE;
    }

    mutex_unlock(pwm_ws281x_legacy.lock);

    return 0;
}

long pwm_ws281x_legacy_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    uint32_t ver = PWM_WS281X_LEGACY_DRIVER_VERSION;
    pwm_ws281x_legacy_ioctl_reg_t reg;

    switch (cmd) {
        case PWM_WS281X_LEGACY_IOCTL_VERSION:
            if (copy_to_user((uint32_t *)arg, &ver, sizeof(ver))) {
                return -EACCES;
            }
            break;

        case PWM_WS281X_LEGACY_IOCTL_REG_READ:
            if (copy_from_user(&reg, (pwm_ws281x_legacy_ioctl_reg_t *)arg, sizeof(reg))) {
                return -EACCES;
            }

            // Bounds check the register space
            if ((reg.reg_offset & 0x3) || reg.reg_offset >= sizeof(pwm_legacy_regs_t)) {
                return -EINVAL;
            }

            reg.reg_value = ioread32((uint8_t __iomem *)pwm_ws281x_legacy.regs + reg.reg_offset);

            if (copy_to_user((pwm_ws281x_legacy_ioctl_reg_t *)arg, &reg, sizeof(reg))) {
                return -EACCES;
            }

            break;

        case PWM_WS281X_LEGACY_IOCTL_REG_WRITE:
            if (copy_from_user(&reg, (pwm_ws281x_legacy_ioctl_reg_t *)arg, sizeof(reg))) {
                return -EACCES;
            }

            // Bounds check the register space
            if ((reg.reg_offset & 0x3) || reg.reg_offset >= sizeof(pwm_legacy_regs_t)) {
                return -EINVAL;
            }

            iowrite32(reg.reg_value, (uint8_t __iomem *)pwm_ws281x_legacy.regs + reg.reg_offset);

            break;

        default:
            return -EINVAL;
    }

    return 0;
}

void legacy_ws281x_dma_callback(void *param) {
    pwm_ws281x_legacy.active &= ~PWM_WS281X_LEGACY_DEVICE_ACTIVE;
    wake_up(pwm_ws281x_legacy.wq);
}

ssize_t legacy_ws281x_dma(const char *buf, ssize_t len) {
    uint64_t first = (uint64_t)buf >> PAGE_SHIFT;
    uint64_t last = ((uint64_t)buf + (len - 1)) >> PAGE_SHIFT;
    int offset = (uint64_t)buf % PAGE_SIZE;
    int count = last - first + 1;
    struct dma_async_tx_descriptor *desc;
    int retval = 0;

    if (count > ARRAY_SIZE(pwm_ws281x_legacy.pages)) {
        count = ARRAY_SIZE(pwm_ws281x_legacy.pages);
        len = (PAGE_SIZE - offset) + ((count - 1) * PAGE_SIZE);
    }

    retval = pin_user_pages_fast((uint64_t)buf, count, 0, pwm_ws281x_legacy.pages);
    if (retval != count) {
        if (retval > 0) {
            unpin_user_pages(pwm_ws281x_legacy.pages, retval);
        }

        dev_err(&pwm_ws281x_legacy.pdev->dev, "Failed to map user pages %d %d\n", retval, count);
        return -ENOBUFS;
    }

    retval = sg_alloc_table_from_pages(&pwm_ws281x_legacy.dma_table, pwm_ws281x_legacy.pages,
                                       count, offset, len, GFP_KERNEL);
    if (retval) {
        unpin_user_pages(pwm_ws281x_legacy.pages, count);

        return -ENOBUFS;
    }

    retval = dma_map_sgtable(&pwm_ws281x_legacy.pdev->dev, &pwm_ws281x_legacy.dma_table,
                             DMA_TO_DEVICE, 0);
    if (retval) {
        sg_free_table(&pwm_ws281x_legacy.dma_table);
        unpin_user_pages(pwm_ws281x_legacy.pages, count);

        return -ENOBUFS;
    }

    pwm_ws281x_legacy.active |= PWM_WS281X_LEGACY_DEVICE_ACTIVE;

    desc = dmaengine_prep_slave_sg(pwm_ws281x_legacy.chan, pwm_ws281x_legacy.dma_table.sgl,
                                   pwm_ws281x_legacy.dma_table.nents, DMA_MEM_TO_DEV, 0);
    if (!desc) {
        len = -ENOBUFS;
        goto cleanup;
    }

    desc->callback = legacy_ws281x_dma_callback;
    desc->callback_param = NULL;

    retval = dmaengine_submit(desc);
    if (retval < 0) {
        len = -ENOBUFS;

        goto cleanup;
    }
    dma_async_issue_pending(pwm_ws281x_legacy.chan);

    // Wait for the DMA to complete, this ensures the user can't mess with the pinned memory,
    // at least in the writer thread context.
    retval = wait_event_interruptible(*pwm_ws281x_legacy.wq,
                                      !(pwm_ws281x_legacy.active & PWM_WS281X_LEGACY_DEVICE_ACTIVE));
    if (retval) {  // Ctrl-C / Kill
        dmaengine_terminate_sync(pwm_ws281x_legacy.chan);
        pwm_ws281x_legacy.active &= ~PWM_WS281X_LEGACY_DEVICE_ACTIVE;
        len = -ERESTARTSYS;
        goto cleanup;
    }

cleanup:
    pwm_ws281x_legacy.active &= ~PWM_WS281X_LEGACY_DEVICE_ACTIVE;

    dma_unmap_sgtable(&pwm_ws281x_legacy.pdev->dev, &pwm_ws281x_legacy.dma_table,
                      DMA_TO_DEVICE, 0);
    sg_free_table(&pwm_ws281x_legacy.dma_table);
    unpin_user_pages(pwm_ws281x_legacy.pages, count);

    return len;
}

ssize_t pwm_ws281x_legacy_write(struct file *file, const char *buf, size_t total, loff_t *loff) {
    ssize_t len = 0;

    if (mutex_lock_interruptible(pwm_ws281x_legacy.lock)) {
        return -EINTR;
    }

    while (len < total) {
        ssize_t retval = legacy_ws281x_dma(buf + len, total - len);
        if (retval <= 0) {
            mutex_unlock(pwm_ws281x_legacy.lock);
            return retval;
        }

        len += retval;
    }

    mutex_unlock(pwm_ws281x_legacy.lock);

    return len;
}


static struct file_operations pwm_ws281x_legacy_fops = {
    .owner = THIS_MODULE,
    .open = pwm_ws281x_legacy_open,
    .release = pwm_ws281x_legacy_release,
    .write = pwm_ws281x_legacy_write,
    .unlocked_ioctl = pwm_ws281x_legacy_ioctl,
};


/*
 * Driver Probe / Init
 */
int pwm_ws281x_legacy_probe(struct platform_device *pdev) {
    struct dma_slave_config dma_conf = {
        .dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
        .src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
        .direction = DMA_MEM_TO_DEV,
        .dst_maxburst = 1,
        .dst_port_window_size = 1,
        .device_fc = false,
    };
    unsigned long rate;
    int result;

    pwm_ws281x_legacy.pdev = pdev;

    of_property_read_u32(pdev->dev.of_node, "brcm,pwm-channel", &pwm_ws281x_legacy.channel);
    if (pwm_ws281x_legacy.channel < PWM_LEGACY_CHANNEL_MIN ||
        pwm_ws281x_legacy.channel > PWM_LEGACY_CHANNEL_MAX) {
        dev_err(&pdev->dev, "%s: brcm,pwm-channel must be %d or %d, got %u\n",
                DEVICE_NAME, PWM_LEGACY_CHANNEL_MIN, PWM_LEGACY_CHANNEL_MAX,
                pwm_ws281x_legacy.channel);
        return -EINVAL;
    }

    result = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(36));
    if (result) {
        dev_err(&pdev->dev, "%s: Failed to set dma mask\n", DEVICE_NAME);
        return result;
    }

    pwm_ws281x_legacy.res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!pwm_ws281x_legacy.res) {
        dev_err(&pdev->dev, "%s: Failed to get platform memory region\n", DEVICE_NAME);
        return -EIO;
    }

    dma_conf.dst_addr = pwm_ws281x_legacy.res->start + offsetof(pwm_legacy_regs_t, fif1);

    pwm_ws281x_legacy.regs = devm_ioremap_resource(&pdev->dev, pwm_ws281x_legacy.res);
    if (IS_ERR(pwm_ws281x_legacy.regs)) {
        dev_err(&pdev->dev, "%s: Failed to request register space\n", DEVICE_NAME);
        return PTR_ERR(pwm_ws281x_legacy.regs);
    }

    pwm_ws281x_legacy.clk = devm_clk_get(&pdev->dev, NULL);
    if (IS_ERR(pwm_ws281x_legacy.clk)) {
        dev_err(&pdev->dev, "Clock not provided\n");
        return PTR_ERR(pwm_ws281x_legacy.clk);
    }

    result = clk_prepare_enable(pwm_ws281x_legacy.clk);
    if (result) {
        dev_err(&pdev->dev, "Clock could not be enabled\n");
        return result;
    }

    rate = clk_get_rate(pwm_ws281x_legacy.clk);
    dev_info(&pdev->dev, "%s: channel %u, pwm clock %lu Hz (%lu Hz ws281x bit rate)\n",
             DEVICE_NAME, pwm_ws281x_legacy.channel, rate, rate / 3);

    pwm_ws281x_legacy.chan = dma_request_chan(&pdev->dev, "pwm0");
    if (IS_ERR(pwm_ws281x_legacy.chan)) {
        result = PTR_ERR(pwm_ws281x_legacy.chan);
        dev_err(&pdev->dev, "Unable to allocate dma channel\n");
        goto err_clk;
    }

    result = dmaengine_slave_config(pwm_ws281x_legacy.chan, &dma_conf);
    if (result) {
        dev_err(&pdev->dev, "Unable to configure dma channel\n");
        goto err_dma;
    }

    pwm_ws281x_legacy.mdev.minor = MISC_DYNAMIC_MINOR;
    pwm_ws281x_legacy.mdev.name = DEVICE_NAME;
    pwm_ws281x_legacy.mdev.fops = &pwm_ws281x_legacy_fops;

    result = misc_register(&pwm_ws281x_legacy.mdev);
    if (result) {
        goto err_dma;
    }

    platform_set_drvdata(pdev, &pwm_ws281x_legacy);

    pwm_ws281x_legacy_init(0);

    return 0;

err_dma:
    dma_release_channel(pwm_ws281x_legacy.chan);
err_clk:
    clk_disable_unprepare(pwm_ws281x_legacy.clk);

    return result;
}

void pwm_ws281x_legacy_remove(struct platform_device *pdev) {
    pwm_ws281x_legacy_cleanup();

    misc_deregister(&pwm_ws281x_legacy.mdev);
    dmaengine_terminate_sync(pwm_ws281x_legacy.chan);
    dma_release_channel(pwm_ws281x_legacy.chan);
    clk_disable_unprepare(pwm_ws281x_legacy.clk);

    pwm_ws281x_legacy.pdev = NULL;
}

static const struct of_device_id pwm_ws281x_legacy_of_match[] = {
	{ .compatible = "pwm-ws281x-legacy" },
	{ }
};
MODULE_DEVICE_TABLE(of, pwm_ws281x_legacy_of_match);

static struct platform_driver pwm_ws281x_legacy_driver = {
	.driver = {
		.name = "pwm-ws281x-legacy",
		.of_match_table = pwm_ws281x_legacy_of_match,
	},
	.probe = pwm_ws281x_legacy_probe,
	.remove = pwm_ws281x_legacy_remove,
};
module_platform_driver(pwm_ws281x_legacy_driver);

MODULE_AUTHOR("Jeremy Garff <jer @ jers.net>");
MODULE_LICENSE("GPL");
