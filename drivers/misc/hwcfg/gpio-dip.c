#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

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

static struct gpio_dip_data *g_provider_data;
static DEFINE_MUTEX(g_provider_mutex);

int gpio_dip_get(const char *name, u32 *result)
{
	struct gpio_dip_derived_data *derived;
	int ret = -ENOENT;

	if (!name)
		return -EINVAL;

	mutex_lock(&g_provider_mutex);
	if (!g_provider_data) {
		mutex_unlock(&g_provider_mutex);
		return -EPROBE_DEFER;
	}

	list_for_each_entry(derived, &g_provider_data->derived_values, list) {
		if (strcmp(derived->name, name) == 0) {
			*result = derived->value;
			ret = 0;
			break;
		}
	}

	mutex_unlock(&g_provider_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(gpio_dip_get);

static ssize_t value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct gpio_dip_data *data = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", data->value);
}
static DEVICE_ATTR_RO(value);

static ssize_t derived_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct gpio_dip_data *data = dev_get_drvdata(dev);
	struct gpio_dip_derived_data *derived;
	ssize_t ret = -ENOENT;

	list_for_each_entry(derived, &data->derived_values, list) {
		if (strcmp(attr->attr.name, derived->name) == 0) {
			ret = sysfs_emit(buf, "%u\n", derived->value);
			break;
		}
	}

	return ret;
}

static int gpio_dip_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	struct gpio_dip_data *data;
	struct gpio_descs *gpios;
	unsigned long bitmap = 0;
	u32 offset = 0;
	struct fwnode_handle *child;
	int err;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	INIT_LIST_HEAD(&data->derived_values);
	platform_set_drvdata(pdev, data);

	gpios = devm_gpiod_get_array(dev, NULL, GPIOD_IN);
	if (IS_ERR(gpios)) {
		dev_err(dev, "unable to acquire input gpios\n");
		return PTR_ERR(gpios);
	}

	if (gpios->ndescs < 1 || gpios->ndescs > BITS_PER_LONG) {
		dev_err(dev, "invalid number of gpios found: %d (must be 1 - %d)\n",
			gpios->ndescs, BITS_PER_LONG);
		return -EINVAL;
	}

	err = gpiod_get_array_value(gpios->ndescs, gpios->desc, NULL, &bitmap);
	if (err) {
		dev_err(dev, "failed to read gpio array value\n");
		return err;
	}

	err = fwnode_property_read_u32(fwnode, "offset", &offset);
	if (err) {
		dev_err(dev, "failed to read offet value\n");
		return err;
	}

	data->value = (u32)bitmap + offset;

	err = device_create_file(dev, &dev_attr_value);
	if (err) {
		dev_err(dev, "unable to create device file for value\n");
		return err;
	}
	dev_info(dev, "base value=%u\n", data->value);

	fwnode_for_each_child_node(fwnode, child) {
		struct gpio_dip_derived_data *derived;
		bool ignore_value;
		u32 derived_offset = 0;

		derived = devm_kzalloc(dev, sizeof(*derived), GFP_KERNEL);
		if (!derived) {
			fwnode_handle_put(child);
			return -ENOMEM;
		}

		derived->name = fwnode_get_name(child);

		err = fwnode_property_read_u32(child, "offset", &derived_offset);
		if (err) {
			dev_err(dev, "failed to read derived offet value\n");
			fwnode_handle_put(child);
			return err;
		}

		ignore_value = fwnode_property_read_bool(child, "ignore-hw-value");
		derived->value = (ignore_value ? 0 : data->value) + derived_offset;

		sysfs_attr_init(&derived->attr.attr);
		derived->attr.attr.name = derived->name;
		derived->attr.attr.mode = 0444;
		derived->attr.show = derived_value_show;
		err = device_create_file(dev, &derived->attr);
		if (err) {
			dev_err(dev,
				"unable to create device file for %s value\n",
				derived->name);
			fwnode_handle_put(child);
			return err;
		}

		list_add_tail(&derived->list, &data->derived_values);
		dev_info(dev, "providing '%s' with value=%u\n",
			 derived->name, derived->value);
	}

	mutex_lock(&g_provider_mutex);
	if (g_provider_data) {
		mutex_unlock(&g_provider_mutex);
		device_remove_file(dev, &dev_attr_value);
		return -EBUSY;
	}
	g_provider_data = data;
	mutex_unlock(&g_provider_mutex);

	return 0;
}

static int gpio_dip_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_dip_data *data = platform_get_drvdata(pdev);
	struct gpio_dip_derived_data *derived, *tmp;

	mutex_lock(&g_provider_mutex);
	g_provider_data = NULL;
	mutex_unlock(&g_provider_mutex);

	list_for_each_entry_safe(derived, tmp, &data->derived_values, list) {
		device_remove_file(dev, &derived->attr);
	}

	device_remove_file(dev, &dev_attr_value);
	return 0;
}

static const struct of_device_id gpio_dip_of_match[] = {
	{ .compatible = "gpio-dip", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gpio_dip_of_match);

static struct platform_driver gpio_dip_driver = {
	.driver = {
		.name = "gpio-dip",
		.of_match_table = gpio_dip_of_match,
	},
	.probe = gpio_dip_probe,
	.remove = gpio_dip_remove,
};
module_platform_driver(gpio_dip_driver);

MODULE_DESCRIPTION("GPIO DIP switches for hardware configuration");
MODULE_AUTHOR("Renjaya R Zenta <ragazenta@gmail.com>");
MODULE_LICENSE("GPL v2");
