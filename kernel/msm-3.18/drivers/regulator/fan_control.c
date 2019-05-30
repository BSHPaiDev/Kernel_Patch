/*
 * Regulator driver for PWM based Fan
 *
 * Copyright (C) 2018 - eInfochips
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/fan_control.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
//#undef DEBUG

#define POWER_CNTRL_GPIO  101

/* PWM period value 40 microsecnonds */
#define PWM_PERIOD 40

struct pwm_fan_regulator_data *drvdata;
struct FAN_dev *fan_chr_dev;
static fan_status_t _glob_fan_status = FAN_UNKNOWN;
struct mutex _lock = __MUTEX_INITIALIZER(_lock);

#if 1 /*Can be used for future case*/
static bool get_pwm_fan_regulator_state(struct device *dev)
{
	//struct pwm_fan_regulator_data *drvdata = dev_get_drvdata (dev);

	return drvdata->enabled;
}
#endif

/**
 * update_device_status() - sets the device status
 */
static void update_device_status(fan_status_t status)
{
    mutex_lock(&_lock);
	_glob_fan_status = status;
    mutex_unlock(&_lock);
}

int pwm_fan_regulator_config(unsigned int dutycycle, unsigned int period);

static ssize_t fan_period_show(struct class *cls, struct class_attribute *attr,char *buf) {
		return sprintf(buf, "%d\n",drvdata->pwm_fan_parameters.period);
}

static ssize_t fan_period_store(struct class *cls, struct class_attribute *attr,
	                                                  const char *buf, size_t size)
{
	int period = 0;
	if (kstrtos32(buf, 10, &period) < 0)
		return -1;
	else {
		if (!pwm_fan_regulator_config(drvdata->pwm_fan_parameters.dutycycle, period))
			return size;
		else
			return -1;
	}
}
static ssize_t fan_dutycycle_show(struct class *cls, struct class_attribute *attr,char *buf) {
		return sprintf(buf, "%d\n",drvdata->pwm_fan_parameters.dutycycle);
}

static ssize_t fan_dutycycle_store(struct class *cls, struct class_attribute *attr,
	                                                  const char *buf, size_t size)
{
	int duty_cycle = 0;
	if (kstrtos32(buf, 10, &duty_cycle) < 0)
		return -1;
	else {
		if (!pwm_fan_regulator_config(duty_cycle, drvdata->pwm_fan_parameters.period))
			return size;
		else
			return -1;
	}
}

static struct class_attribute fan_pwm_class_attr[] = {
	__ATTR(dutycycle, S_IRUGO | S_IWUSR,
	       fan_dutycycle_show,
	       fan_dutycycle_store),
	__ATTR(period, S_IRUGO | S_IWUSR,
	       fan_period_show,
	       fan_period_store),
	__ATTR_NULL,
};

static struct class fan_pwm_class  = {
	.name = "fan_control",
	.owner = THIS_MODULE,
	.class_attrs = fan_pwm_class_attr,
};

static int pwm_fan_regulator_disable(struct platform_device *pdev)
{
	pwm_disable(drvdata->pwm);

	pwm_put (drvdata->pwm);

	class_unregister(&fan_pwm_class);

	return 0;
}

int pwm_fan_regulator_config(unsigned int dutycycle, unsigned int period)
{
	int ret = 0;
	do {
		DBG ("Requested PWM Dutycycle %d - Period %d\n", dutycycle, period);

		//Change PWM config only if any parameter is modified
		if ((drvdata->pwm_fan_parameters.dutycycle != dutycycle)
		 || (drvdata->pwm_fan_parameters.period != period)){

			ret = pwm_config_us(drvdata->pwm, (PWM_PERIOD-dutycycle), period);	//configure OFF period - as per beta
			if (ret) {
				printk("Failed to configure PWM\n");
				update_device_status(FAN_CONFIG_ERR);
				break;
			}

			ret = pwm_enable(drvdata->pwm);
			if (ret) {
				printk("Failed to enable PWM\n");
				update_device_status(PWM_REG_ENABLE_ERR);
				break;
			}
			drvdata->enabled = true;

			drvdata->pwm_fan_parameters.dutycycle = dutycycle;
			drvdata->pwm_fan_parameters.period = period;

			update_device_status(FAN_OK);
			DBG ("Fan config done successfully\n");
		}
	} while (0);

	return ret;
}
EXPORT_SYMBOL(pwm_fan_regulator_config);

static int get_parameters_from_dt (struct device *dev, struct pwm_fan_regulator_data *pwm_regulator_data)
{
	struct device_node *np;
	char prop_name[MAX_PROP_SIZE];
	int ret = 0;

	do
	{
		if (NULL == dev || NULL == pwm_regulator_data)
		{
			printk ("Invalid data provided %s", __func__);
			ret = 1;
			break;
		}

		np = dev->of_node;
		if (np == NULL)
		{
			printk ("Invalid dtb node node for PWM fan regulator\n");
			ret = 1;
			break;
		}

		 /* Parse Parameters of PWM for fan control */
		memset (prop_name, 0x00,MAX_PROP_SIZE);
		snprintf(prop_name, MAX_PROP_SIZE, "period");
		ret = of_property_read_u32(np, prop_name, &(pwm_regulator_data->pwm_fan_parameters.period));
		if (ret < 0) {
			printk("%s property is not valid\n", prop_name);
			ret = 1;
			break;
		}

		memset (prop_name, 0x00,MAX_PROP_SIZE);
		snprintf(prop_name, MAX_PROP_SIZE, "dutycycle");
		ret = of_property_read_u32(np, prop_name, &(pwm_regulator_data->pwm_fan_parameters.dutycycle));
		if (ret < 0) {
			printk("%s property is not valid\n", prop_name);
			ret = 1;
			break;
		}

		DBG ("Device Tree data captured successfully for fan control\n");

	}while (0);
	return ret;
}

#if 1/*For future use*/
void get_fan_pwm_out (int *configured_pwm_output)
{
	if (configured_pwm_output != NULL)
		*configured_pwm_output = drvdata->pwm_fan_parameters.dutycycle;
}
EXPORT_SYMBOL(get_fan_pwm_out);
#endif

static int pwm_fan_regulator_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;

	do {
		if (!np) {
			printk("Device Tree node missing for fan control\n");
			ret = -EINVAL;
			break;
		}

		drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_KERNEL);
		if (!drvdata) {
			printk ("Failed to allocate memory drvdata for fan control\n");
			ret = -ENOMEM;
			break;
		}

		memset (&drvdata->pwm_fan_parameters, 0x00, sizeof (struct fan_pwm_parameters));

		ret = get_parameters_from_dt (&pdev->dev, drvdata);
		if (0 != ret) {
			printk("Failed to get parameters from dtb for fan control\n");
			break;
		}

		drvdata->pwm = of_pwm_get(pdev->dev.of_node, NULL);
		if (IS_ERR(drvdata->pwm)) {
			printk("Failed to get PWM for fan control\n");
			ret = -1;
			break;
		}

		/*API to config PWM at time of intialization*/
		ret = pwm_fan_regulator_config (drvdata->pwm_fan_parameters.dutycycle, drvdata->pwm_fan_parameters.period);
		if (0 != ret) {
			printk ("Failed to set fan PWM config for fan control\n");
			break;
		}

		class_register(&fan_pwm_class);

		/* Enable fan power signal */
		gpio_set_value(POWER_CNTRL_GPIO, 1);
		pwm_fan_regulator_config (10, drvdata->pwm_fan_parameters.period);

		update_device_status(FAN_READY);
		DBG("PWM Fan Regulator Driver probed successfully\n");
	}while (0);

	if (ret != 0)
		update_device_status(FAN_PROBE_ERR);
		
	return ret;
}

/**
 * set_dutycycle() - Change the dutycycle of pwm fan
 * @buffer: dutycycle value(in us) to set
 */
static ssize_t set_dutycycle(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
	unsigned int dutycycle;
	unsigned int dutycycle_us;
	char buf[sizeof(dutycycle_us)] = {0, };

	int ret = copy_from_user(buf, buffer, len);
	if (ret < 0) {
		dev_err(fan_chr_dev->dev, "err: copy_from_user failed\n");
		return 0;
	}

	if (kstrtos32(buf, 10, &dutycycle) < 0)
		return -1;

	/* Convert dutycycle percentage in to microseconds */
	dutycycle_us = ((dutycycle * PWM_PERIOD)/100);

	if (pwm_fan_regulator_config(dutycycle_us, drvdata->pwm_fan_parameters.period) != 0) {
		dev_err(fan_chr_dev->dev, "err: pwm_fan_regulator_config failed\n");
		return -1;
	}

	return len;
}

/**
 * get_dutycycle() - Send the recent dutycycle set for PWM fan
 * @buffer: holds the dutycycle value 
 */
static ssize_t get_dutycycle(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
	int data;
	char buf[sizeof(data)] = {0, };

	get_fan_pwm_out(&data);
	sprintf(buf, "%d", data);

	if (copy_to_user(buffer, buf, sizeof(buf)) != 0) {
		dev_err(fan_chr_dev->dev, "err: copy_to_user failed\n");
		return 0;
	}

	return len;
}

static long pwm_fan_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	uint8_t *buf = NULL;
	uint8_t data;

	switch (cmd) {
		case GET_FAN_STATE :
			buf = (uint8_t*)arg;
			data = (uint8_t)_glob_fan_status;
			break;

		case GET_FAN_REGULTR_STATE :
			buf = (uint8_t*)arg;
			data = (uint8_t)get_pwm_fan_regulator_state(NULL);
			break;

		default:
			return -EINVAL;
	}

	if (buf) {
		if (copy_to_user(buf, &data, 1) != 0) {
			printk(KERN_ERR "PAI pwm_fan_ioctl copy_to_user failed");
			return -1;
		}
	}

	return 0;
}

struct file_operations fan_fops = {
	.owner = THIS_MODULE,
	.write = set_dutycycle,
	.read = get_dutycycle,
	.unlocked_ioctl = pwm_fan_ioctl
};

static int __init pwm_fan_init(void)
{
	int rc;

	fan_chr_dev = (struct FAN_dev*) kzalloc(sizeof(struct FAN_dev), GFP_KERNEL);
	if (!fan_chr_dev)
		return -ENOMEM;

	rc = alloc_chrdev_region(&fan_chr_dev->devnum_fan, 0, 1, DRV_NAME);
	if (rc < 0) {
		dev_err(fan_chr_dev->dev, "err: alloc_chrdev_region failed\n");
		rc = -ENODEV;
		goto err_dev;
	}

	fan_chr_dev->cdev_fan = cdev_alloc();
	if (fan_chr_dev->cdev_fan == NULL) {
		rc = -ENOMEM;
		goto err_alloc;
	}

	cdev_init(fan_chr_dev->cdev_fan, &fan_fops);
	fan_chr_dev->cdev_fan->owner = THIS_MODULE;

	rc = cdev_add(fan_chr_dev->cdev_fan, fan_chr_dev->devnum_fan, 1);
	if (rc < 0) {
		dev_err(fan_chr_dev->dev, "err: cdev_add failed\n");
		goto err_add;
	}

	fan_chr_dev->dev = device_create(&fan_pwm_class, NULL, fan_chr_dev->devnum_fan, NULL, DRV_NAME);
	if (IS_ERR(fan_chr_dev->dev)) {
		dev_err(fan_chr_dev->dev, "err: device_create failed\n");
		goto err_device_create;
	}

	DBG("PWM fan init done\n");
	//update_device_status(FAN_INIT_DONE);

	return 0;

err_device_create:
	cdev_del(fan_chr_dev->cdev_fan);
err_add:
	kfree(fan_chr_dev->cdev_fan);
err_alloc:
	unregister_chrdev_region(fan_chr_dev->devnum_fan, 1);
err_dev:
	fan_chr_dev->dev = NULL;
	update_device_status(FAN_INIT_ERR);
	return rc;
}
module_init(pwm_fan_init);

static void __exit pwm_fan_exit(void)
{
	device_destroy(&fan_pwm_class, fan_chr_dev->devnum_fan);
	cdev_del(fan_chr_dev->cdev_fan);
	unregister_chrdev_region(fan_chr_dev->devnum_fan, 1);
	if (fan_chr_dev)
		kfree(fan_chr_dev);
}
module_exit(pwm_fan_exit);

static const struct of_device_id pwm_of_match[] = {
	{ .compatible = "pwm-fan-regulator" },
	{ },
};
MODULE_DEVICE_TABLE(of, pwm_of_match);

static struct platform_driver pwm_fan_regulator_driver = {
	.driver = {
		.name		= "pwm-fan-regulator",
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(pwm_of_match),
	},
	.probe = pwm_fan_regulator_probe,
	.remove = pwm_fan_regulator_disable,
};
module_platform_driver(pwm_fan_regulator_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("eInfochips");
MODULE_DESCRIPTION("PWM Fan Control");
