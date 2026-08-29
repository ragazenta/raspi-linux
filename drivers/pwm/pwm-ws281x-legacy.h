
#ifndef __PWM_WS281X_LEGACY_H__
#define __PWM_WS281X_LEGACY_H__


#define DEVICE_NAME                                 "ws281x_pwm"

#define PWM_WS281X_LEGACY_IOCTL_MAGIC               0x6a67
#define PWM_WS281X_LEGACY_IOCTL_VERSION             _IOR(PWM_WS281X_LEGACY_IOCTL_MAGIC, 0x0, uint32_t *)
#define PWM_WS281X_LEGACY_IOCTL_REG_READ            _IOWR(PWM_WS281X_LEGACY_IOCTL_MAGIC, 0x1, pwm_ws281x_legacy_ioctl_reg_t *)
#define PWM_WS281X_LEGACY_IOCTL_REG_WRITE           _IOW(PWM_WS281X_LEGACY_IOCTL_MAGIC, 0x2, pwm_ws281x_legacy_ioctl_reg_t *)


//
// Ioctl Structures
//
typedef struct {
    uint32_t flags;
    uint32_t reg_offset;
    uint32_t reg_value;
} pwm_ws281x_legacy_ioctl_reg_t;

typedef struct {
    void *addr;
    uint64_t len;
} pwm_ws281x_legacy_ioctl_xfer_t;

void pwm_ws281x_legacy_init(int invert);
void pwm_ws281x_legacy_cleanup(void);
int pwm_ws281x_legacy_open(struct inode *inode, struct file *file);
int pwm_ws281x_legacy_release(struct inode *inode, struct file *file);
long pwm_ws281x_legacy_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
void legacy_ws281x_dma_callback(void *param);
ssize_t legacy_ws281x_dma(const char *buf, ssize_t len);
ssize_t pwm_ws281x_legacy_write(struct file *file, const char *buf, size_t total, loff_t *loff);
int pwm_ws281x_legacy_probe(struct platform_device *pdev);
void pwm_ws281x_legacy_remove(struct platform_device *pdev);

#endif // __PWM_WS281X_LEGACY_H__
