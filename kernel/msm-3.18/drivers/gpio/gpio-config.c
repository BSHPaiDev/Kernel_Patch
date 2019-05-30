/*
 * GPIO Config driver:
 * Driver for configuring and exporing gpios from devicetree node
 * by default at boot time
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm.h>

static int gpio_config_remove(struct platform_device * pdev)
{
	struct device_node * np = pdev->dev.of_node;
	struct device_node * cur_np = NULL;
	int num_gpio = 0;
	int gpio = 0;
	int i = 0;

	for_each_child_of_node(np, cur_np) {
		num_gpio = of_gpio_count(cur_np);
		 for (i = 0; i < num_gpio; i++) {
			gpio = of_get_gpio(cur_np, i);
			if (gpio_is_valid(gpio))
				gpio_free(gpio);
		 }
	}

	return 0;
}
static int gpio_config_probe(struct platform_device * pdev)
{
	struct device_node * np = pdev->dev.of_node;
	struct device_node * cur_np = NULL;
	const char * name = NULL;
	const char * dir = "out";
	bool direction_may_change = false;
	bool gpio_init_high = false;
	int gpio = 0;
	int num_gpio = 0;
	int ret = 0;
	unsigned long flags = 0;
	int i = 0;

	do {
		if (!np) {
			dev_err(&pdev->dev,
				"GPIO-CONFIG: Device tree entry not found!!!\n");
			ret = -1;
			break;
		}

		for_each_child_of_node(np, cur_np) {

			flags = GPIOF_EXPORT;

			ret = of_property_read_string(cur_np, "gpio-exp-dir", &dir);

			gpio_init_high = of_property_read_bool(cur_np,
							       "gpio-exp-init-high");

			direction_may_change = of_property_read_bool(cur_np,
								     "gpio-exp-dir-may-change");

			flags |= (direction_may_change ?
				  GPIOF_EXPORT_CHANGEABLE : 0);

			if (!strcmp(dir, "out")) {
				flags |= GPIOF_DIR_OUT;
				flags |= (gpio_init_high ? GPIOF_INIT_HIGH : GPIOF_INIT_LOW);
			} else if (!strcmp(dir, "open_drain")) {
				flags |= GPIOF_OPEN_DRAIN;
			} else if (!strcmp(dir, "open_source")) {
				flags |= GPIOF_OPEN_SOURCE;
			} else {
				flags |= GPIOF_DIR_IN;
			}

			num_gpio = of_gpio_count(cur_np);

			name = (name ? name : of_node_full_name(cur_np));

			for (i = 0; i < num_gpio; i++) {
				gpio = of_get_gpio(cur_np, i);

				if (!gpio_is_valid(gpio) || ret != 0) {
					dev_err(&pdev->dev,
						"%s:%d invalid gpio in"
						" node: %s at index: %d\n",
						__func__, __LINE__,
						of_node_full_name(cur_np), i);
					continue;
				}

				if (gpio_request_one(gpio, flags, name)) {
					dev_err(&pdev->dev,
						"%s:%d could not request gpio %d\n",
						__func__, __LINE__, gpio);
				}
			}
			ret = 0;
			direction_may_change = 0;
			gpio_init_high = 0;
		}
	} while (0);
	printk ("GPIO Config driver probed with return value = %d\n",ret);
	return ret;
}

static const struct of_device_id gpio_config_ids[] = {
	{ .compatible = "gpio-config" },
	{},
};
MODULE_DEVICE_TABLE(of, gpio_config_ids);

static struct platform_driver gpio_config_driver = {
	.driver = {
		.name = "gpio-config",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(gpio_config_ids),
	},
	.probe = gpio_config_probe,
	.remove = gpio_config_remove,
};
module_platform_driver(gpio_config_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GPIO Configuration from devicetree");
MODULE_AUTHOR("e-infochips");

