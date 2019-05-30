#ifndef _PAI_PWM_H
#define _PAI_PWM_H

#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/qpnp/pwm.h>

#define DRV_NAME "pwm-gpio"
#define GPIO_PWM  36

#define NUM_PWM       1
#define NUM_PWM_CELL  2

struct gpio_pwm_data {
	struct hrtimer timer;
	struct gpio_desc *gpiod;
	bool polarity;
	bool pin_on;
	int on_time;
	int off_time;
	bool run;
};

struct gpio_pwm_chip {
	struct pwm_chip chip;
};

//#define DEBUG

#ifdef DEBUG
#define DBG(args...)    printk(args)
#else
#define DBG(args...)    do { } while(0)
#endif

#endif  //_PAI_PWM_H
