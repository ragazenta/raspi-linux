#ifndef _HWCFG_GPIO_DIP_H_
#define _HWCFG_GPIO_DIP_H_

struct gpio_dip_derived_data {
	const char *name;
	u32 value;
	struct list_head list;
	struct device_attribute attr;
};

struct gpio_dip_data {
	u32 value;
	struct list_head derived_values;
};

int gpio_dip_get(const char *name, u32 *result);

#endif
