#include <linux/VOUT_temperature.h>

static int thread_fn(void *data)
{
	struct VOUT_VADC *chip = (struct VOUT_VADC *)data;
	struct qpnp_vadc_result result;
	long VOUT_Temp;
	int ret = 0;

	while(!kthread_should_stop())
	{
			/*Read the vadc channel*/
			ret = qpnp_vadc_read(chip->vadc_dev, VADC_AMUX_THM4, &result);
			if(!ret){
				VOUT_Temp = (int)result.physical;
				//printk("VOUT_Temp: %ld\n", VOUT_Temp);
			}
			else
				pr_err("Error : can't read VOUT Temperature data\n");

			ssleep(chip->delay);
	}
	return 0;
}

static int VOUT_probe(struct spmi_device *spmi)
{
	struct VOUT_VADC *chip = NULL;
	struct device_node *node = NULL;
	int rc = 0;

	//printk("%s: VOUT Temperature Sensor Probe Successful\n", __func__);
	do {
		if (!spmi || !(&spmi->dev) || !spmi->dev.of_node) {
		        dev_err(&spmi->dev, "%s: device tree node not found\n", __func__);
			rc = -1;
			break;
		}

		node = spmi->dev.of_node;

		chip = kzalloc(sizeof(struct VOUT_VADC), GFP_KERNEL);
		if (!chip) {
			dev_err(&spmi->dev, "%s: Can't allocate VOUT_VADC\n", __func__);
			rc = -1;
			break;
		}

		dev_set_drvdata(&spmi->dev, chip);

		chip->dev = &(spmi->dev);

		rc = of_property_read_u32(node, "qcom,channel-num", &chip->adc_channel); /* Read qcom,channel-num = 0x10; */
		//printk("VOUT_adc_channel: %d\n", chip->adc_channel);

		if (!rc) {
			if (chip->adc_channel < 0 || chip->adc_channel >= ADC_MAX_NUM){ 
				dev_err(&spmi->dev, "%s: invalid qcom,channel-num=%d specified\n",__func__, chip->adc_channel);
				rc = -1;
				break;
			}
			else{
				chip->vadc_dev = qpnp_get_vadc(&spmi->dev,"VOUT");
				if (IS_ERR(chip->vadc_dev)) {
					rc = PTR_ERR(chip->vadc_dev);
					if (rc != -EPROBE_DEFER)
						pr_err("VOUT vadc property missing\n");
					rc = -1;
					break;
				}
			}
		}

		rc = of_property_read_u32(node, "qcom,delay", &chip->delay); /* Read qcom,delay = 5; */
		if (rc){
			pr_err("Enable To Read VOUT Delay\n");
			rc = -1;
			break;
		}
		/*Create the kernel thread 'VOUT thread'*/
		chip->thread_st = kthread_run(thread_fn, (void *)chip, "%s", "VOUT_thread");
		if(chip->thread_st < 0){
		        pr_err("kthread create failed for VOUT Temprature reading\n");
			rc = -1;
		        break;
	        }

	}while(0);

	if(rc == -1){
		 if(chip)
			 kfree(chip);
	}
	return rc;
}

static int VOUT_remove(struct spmi_device *spmi)
{
	struct VOUT_VADC *chip = dev_get_drvdata(&spmi->dev);
	int ret = 0;
	ret = kthread_stop(chip->thread_st);
	if(!ret)
		printk("VOUT Thread stopped\n");
	dev_set_drvdata(&spmi->dev, NULL);
	if(chip)
		kfree(chip);

	return 0;
}

static const struct of_device_id VOUT_match_table[] = {
		{ .compatible = "qcom,VOUT-tm-sensor" },
		{}
};

MODULE_DEVICE_TABLE(of, VOUT_match_table);

static struct spmi_driver VOUT_driver = {
		.driver = {
			   .name = "qcom,VOUT-tm-sensor",
			   .owner = THIS_MODULE,
			   .of_match_table = VOUT_match_table,
			  },
		.probe = VOUT_probe,
		.remove = VOUT_remove,
};
static int __init VOUT_init(void){
	int error;

	error = spmi_driver_register(&VOUT_driver);
	if (error) {
		pr_err("Unable to register VOUT vadc platform driver.\n");
		return error;
	}
	return 0;
};
static void __exit VOUT_exit(void)
{
	spmi_driver_unregister(&VOUT_driver);
}

module_init(VOUT_init);
module_exit(VOUT_exit);

MODULE_AUTHOR("RBEI");
MODULE_DESCRIPTION("VOUT Temperature driver");
MODULE_LICENSE("GPL");

