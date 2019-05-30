#include <linux/NTC_temperature.h>

#define NR_DEVS		1
#define FIRST_MINOR	0

struct NTC_VADC *_glob_chip = NULL;
struct NTC_dev *_glob_ntc_dev = NULL;
static ntc_status_t _glob_ntc_status = NTC_UNKNOWN;

#if 0
extern int pwm_fan_regulator_config(unsigned int dutycycle, unsigned int period);

typedef struct  {
	/* Lookup table Temperature value */
	long temp;				

	/* Lookup table PWM dutycycle percentage value */
	unsigned int dutycycle;
} temp_dutycycle_lookup_t;

#define TABLE_SIZE 30

/* PWM period value 40 microsecnonds */
#define PWM_PERIOD 40

/* Lookup table to choose dutycycle based on reported temerature from NTV sensor
 * For any NTC_Temp, for which t1 < NTC_Temp =< t2, dutycycle against t2 value will be selected */
temp_dutycycle_lookup_t table[TABLE_SIZE] = {
	{ .temp = 36, .dutycycle = 42 },	// For temperature less than this, PWM dytycycle will be 42%
	{ .temp = 37, .dutycycle = 44 },
	{ .temp = 38, .dutycycle = 46 },
	{ .temp = 39, .dutycycle = 48 },
	{ .temp = 40, .dutycycle = 50 },
	{ .temp = 41, .dutycycle = 52 },
	{ .temp = 42, .dutycycle = 54 },
	{ .temp = 43, .dutycycle = 56 },
	{ .temp = 44, .dutycycle = 58 },
	{ .temp = 45, .dutycycle = 60 },
	{ .temp = 46, .dutycycle = 62 },
	{ .temp = 47, .dutycycle = 64 },
	{ .temp = 48, .dutycycle = 66 },
	{ .temp = 49, .dutycycle = 68 },
	{ .temp = 50, .dutycycle = 70 },
	{ .temp = 51, .dutycycle = 72 },
	{ .temp = 52, .dutycycle = 74 },
	{ .temp = 53, .dutycycle = 76 },
	{ .temp = 54, .dutycycle = 78 },
	{ .temp = 55, .dutycycle = 80 },
	{ .temp = 56, .dutycycle = 82 },
	{ .temp = 57, .dutycycle = 84 },
	{ .temp = 58, .dutycycle = 86 },
	{ .temp = 59, .dutycycle = 88 },
	{ .temp = 60, .dutycycle = 90 },
	{ .temp = 61, .dutycycle = 92 },
	{ .temp = 62, .dutycycle = 94 },
	{ .temp = 63, .dutycycle = 96 },
	{ .temp = 64, .dutycycle = 98 },
	{ .temp = 65, .dutycycle = 100 },	// For temperature higher than this, PWM dytycycle will be 100%
};
#endif

static void update_device_status(ntc_status_t status)
{
    _glob_ntc_status = status;
}

static long ntc_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
    uint8_t *buf = NULL;
	uint8_t data;
	int ret;

	switch (cmd) {
		case IOCTL_NTC_STATUS :
			buf = (uint8_t*)arg;
			data = (uint8_t)_glob_ntc_status;
			if (buf) {
				ret = copy_to_user(buf, &data, 1);
				if (ret != 0) {
					printk(KERN_ERR "PAI ntc copy_to_user failed");
					return -1;
				}
				printk("PAI NTC send data[%d]", data);
			}
			break;

		default:
			return -EINVAL;
	}

	return 0;
}

/** 
 * ntc_read() - send the temperature value from NTC sensor in @buffer
 */
static ssize_t ntc_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
	struct qpnp_vadc_result result;
	long NTC_Temp;
	int ret = 0;
	char buf[sizeof(NTC_Temp)] = {0, };

	ret = qpnp_vadc_read(_glob_chip->vadc_dev, VADC_AMUX_THM5, &result);
	if (!ret) {
		update_device_status(NTC_OK);

		NTC_Temp = (int)result.physical;
		printk(KERN_INFO "PAI NTC_Temp value : %ld\n", NTC_Temp);

		sprintf(buf, "%ld", NTC_Temp);

		ret = copy_to_user(buffer, buf, len);
		if (ret != 0) {
			pr_err("PAI NTC copy_to_user failed[%d]", ret);
			return 0;
		}
	}
	else {
		pr_err("PAI can't read Temperature data\n");
		update_device_status(NTC_READ_ERR);
		return 0;
	}

	return len;
}

#if 0
static int thread_fn(void *data)
{
	struct NTC_VADC *chip = (struct NTC_VADC *)data;
	struct qpnp_vadc_result result;
	long NTC_Temp;
	int i, ret = 0;
	unsigned int dutycycle;
	unsigned int dutycycle_us;

	while(!kthread_should_stop())
	{
		/*Read the vadc channel*/
		ret = qpnp_vadc_read(chip->vadc_dev, VADC_AMUX_THM5, &result);
		if(!ret){
			update_device_status(NTC_OK);
			NTC_Temp = (int)result.physical;

			i = 0;
			while ((i < TABLE_SIZE) && (NTC_Temp > table[i].temp))
			{
				i++;
			}

			if (i == TABLE_SIZE)
			{
				dutycycle = table[i-1].dutycycle;
			}
			else
			{
				dutycycle = table[i].dutycycle;	
			}

			/* Convert dutycycle percentage in to microseconds */
			dutycycle_us = ((dutycycle * PWM_PERIOD)/100);

			//printk("NTC_Temp:%ld -> dutycycle %duS (%d percent)\n", NTC_Temp, dutycycle_us, dutycycle);

			/*ret = pwm_fan_regulator_config (dutycycle_us, PWM_PERIOD);
			if (0 != ret) {
				printk ("Failed to set fan PWM config for fan control\n");
				break;
			}*/
		}
		else {
			pr_err("Error : can't read Temperature data\n");
			update_device_status(NTC_READ_ERR);
		}

		ssleep(chip->delay);
	}
	return 0;
}
#endif

static int NTC_probe(struct spmi_device *spmi)
{
	struct NTC_VADC *chip = NULL;
	struct device_node *node = NULL;
	int rc = 0;

	printk("%s: NTC Temperature Sensor Probe Successful\n", __func__);
	do {
		if (!spmi || !(&spmi->dev) || !spmi->dev.of_node) {
			dev_err(&spmi->dev, "%s: device tree node not found\n", __func__);
			rc = -1;
			break;
		}

		node = spmi->dev.of_node;

		chip = kzalloc(sizeof(struct NTC_VADC), GFP_KERNEL);
		if (!chip) {
			dev_err(&spmi->dev, "%s: Can't allocate NTC_VADC\n", __func__);
			rc = -1;
			break;
		}

		dev_set_drvdata(&spmi->dev, chip);

		chip->dev = &(spmi->dev);

		rc = of_property_read_u32(node, "qcom,channel-num", &chip->adc_channel); /* Read qcom,channel-num = 0x11; */

		if (!rc) {
			if (chip->adc_channel < 0 || chip->adc_channel >= ADC_MAX_NUM){ 
				dev_err(&spmi->dev, "%s: invalid qcom,channel-num=%d specified\n",__func__, chip->adc_channel);
				rc = -1;
				break;
			}
			else{
				chip->vadc_dev = qpnp_get_vadc(&spmi->dev,"NTC");
				if (IS_ERR(chip->vadc_dev)) {
					rc = PTR_ERR(chip->vadc_dev);
					if (rc != -EPROBE_DEFER)
						pr_err("vadc property missing\n");
					rc = -1;
					break;
				}
			}
		}

		rc = of_property_read_u32(node, "qcom,delay", &chip->delay); /* Read qcom,delay = 5; */
		if (rc){
			pr_err("Enable To Read Delay\n");
			rc = -1;
			break;
		}

#if 0
		/*Create the kernel thread 'NTC thread'*/
		chip->thread_st = kthread_run(thread_fn, (void *)chip, "%s", "NTC_thread");
		if(chip->thread_st < 0){
			pr_err("kthread create failed for Temprature reading\n");
			rc = -1;
			break;
		}
#endif
	}while(0);

	if(rc == -1){
		if(chip)
			kfree(chip);
		update_device_status(NTC_PROBE_ERR);
	}
	else {
		update_device_status(NTC_READY);
		_glob_chip = chip;
	}	
	return rc;
}

static int NTC_remove(struct spmi_device *spmi)
{
	struct NTC_VADC *chip = dev_get_drvdata(&spmi->dev);

#if 0
	int ret = 0;
	ret = kthread_stop(chip->thread_st);
	if(!ret)
		printk("Thread stopped\n");
#endif		
	dev_set_drvdata(&spmi->dev, NULL);
	if(chip)
		kfree(chip);
	return 0;
}

struct file_operations ntc_fops = {
	.owner = THIS_MODULE,
	.read = ntc_read,
	.unlocked_ioctl = ntc_ioctl
};

static const struct of_device_id NTC_match_table[] = {
	{ .compatible = "qcom,NTC-tm-sensor" },
	{}
};

MODULE_DEVICE_TABLE(of, NTC_match_table);

static struct spmi_driver NTC_driver = {
	.driver = {
		.name = "qcom,NTC-tm-sensor",
		.owner = THIS_MODULE,
		.of_match_table = NTC_match_table,
	},
	.probe = NTC_probe,
	.remove = NTC_remove,
};

/** 
 * ntc_device_create() - Create and configure ntc char device
 * @ntc_dev: ntc device descriptor
 */
static int ntc_device_create(struct NTC_dev *ntc_dev)
{
	int rc;

	rc = alloc_chrdev_region(&ntc_dev->devnum_ntc, FIRST_MINOR, NR_DEVS, DRV_NAME);
	if (rc < 0) {
		dev_err(ntc_dev->dev, "err: alloc_chrdev_region failed\n");
		rc = -ENODEV;
		goto err_dev;
	}

	ntc_dev->cdev_ntc = cdev_alloc();
	if (ntc_dev->cdev_ntc == NULL) {
		rc = -ENOMEM;
		goto err_alloc;
	}

	cdev_init(ntc_dev->cdev_ntc, &ntc_fops);
	ntc_dev->cdev_ntc->owner = THIS_MODULE;

	rc = cdev_add(ntc_dev->cdev_ntc, ntc_dev->devnum_ntc, NR_DEVS);
	if (rc < 0) {
		dev_err(ntc_dev->dev, "err: cdev_add failed\n");
		goto err_add;
	}

	ntc_dev->class_ntc = class_create(THIS_MODULE, CLS_NAME);
	if (IS_ERR(ntc_dev->class_ntc)) {
		dev_err(ntc_dev->dev, "err: class_create failed\n");
		goto err_class_create;
	}

	ntc_dev->dev = device_create(ntc_dev->class_ntc, NULL, ntc_dev->devnum_ntc, NULL, DRV_NAME);
	if (IS_ERR(ntc_dev->dev)) {
		dev_err(ntc_dev->dev, "err: device_create failed\n");
		goto err_device_create;
	}
	return 0;

err_device_create:
	class_destroy(ntc_dev->class_ntc);
err_class_create:
	cdev_del(ntc_dev->cdev_ntc);
err_add:
	kfree(ntc_dev->cdev_ntc);
err_alloc:
	unregister_chrdev_region(ntc_dev->devnum_ntc, 1);
err_dev:
	ntc_dev->dev = NULL;
	return rc;
}

static int __init NTC_init(void)
{
	int error;

	error = spmi_driver_register(&NTC_driver);
	if (error) {
		pr_err("Unable to register vadc platform driver.\n");
		return error;
	}

	_glob_ntc_dev = (struct NTC_dev*)kzalloc(sizeof(struct NTC_dev), GFP_KERNEL);
	if (!_glob_ntc_dev)
		return -ENOMEM;

	error = ntc_device_create(_glob_ntc_dev);
	if (error != 0) {
		update_device_status(NTC_INIT_ERR);
		return error;
	}

	update_device_status(NTC_INIT_DONE);
	return 0;
};

static void __exit NTC_exit(void)
{
	device_destroy(_glob_ntc_dev->class_ntc, _glob_ntc_dev->devnum_ntc);
	class_destroy(_glob_ntc_dev->class_ntc);
	cdev_del(_glob_ntc_dev->cdev_ntc);
	unregister_chrdev_region(_glob_ntc_dev->devnum_ntc, NR_DEVS);

	if (_glob_ntc_dev)
		kfree(_glob_ntc_dev);

	spmi_driver_unregister(&NTC_driver);
}

module_init(NTC_init);
module_exit(NTC_exit);

MODULE_AUTHOR("eInfochips");
MODULE_DESCRIPTION("NTC Temperature driver");
MODULE_LICENSE("GPL");
