
#ifndef __PWM_WS281X_RP1_H__
#define __PWM_WS281X_RP1_H__


#define DEVICE_NAME                              "ws281x_pwm"

#define PWM_WS281X_RP1_IOCTL_MAGIC               0x6a67
#define PWM_WS281X_RP1_IOCTL_VERSION             _IOR(PWM_WS281X_RP1_IOCTL_MAGIC, 0x0, uint32_t *)
#define PWM_WS281X_RP1_IOCTL_REG_READ            _IOWR(PWM_WS281X_RP1_IOCTL_MAGIC, 0x1, pwm_ws281x_rp1_ioctl_reg_t *)
#define PWM_WS281X_RP1_IOCTL_REG_WRITE           _IOW(PWM_WS281X_RP1_IOCTL_MAGIC, 0x2, pwm_ws281x_rp1_ioctl_reg_t *)


//
// Ioctl Structures
//
typedef struct {
    uint32_t flags;
    uint32_t reg_offset;
    uint32_t reg_value;
} pwm_ws281x_rp1_ioctl_reg_t;

typedef struct {
    void *addr;
    uint64_t len;
} pwm_ws281x_rp1_ioctl_xfer_t;

void pwm_ws281x_rp1_chan(int channel, int invert);
void pwm_ws281x_rp1_init(int channel, int invert);
void pwm_ws281x_rp1_cleanup(void);
int pwm_ws281x_rp1_open(struct inode *inode, struct file *file);
int pwm_ws281x_rp1_release(struct inode *inode, struct file *file);
long pwm_ws281x_rp1_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
void rp1_ws281x_dma_callback(void *param);
ssize_t rp1_ws281x_dma(const char *buf, ssize_t len);
ssize_t pwm_ws281x_rp1_write(struct file *file, const char *buf, size_t total, loff_t *loff);
int pwm_ws281x_rp1_probe(struct platform_device *pdev);
void pwm_ws281x_rp1_remove(struct platform_device *pdev);

#endif // __PWM_WS281X_RP1_H__

