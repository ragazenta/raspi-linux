
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


#endif // __PWM_WS281X_RP1_H__

