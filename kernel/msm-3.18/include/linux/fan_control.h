#ifndef __FAN_CONTROL_H
#define __FAN_CONTROL_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/qpnp/pwm.h>
#include <linux/stat.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/ioctl.h>

#define MAX_PROP_SIZE 64
#define DRV_NAME    "pwm_fan"

/* ioctl routines */
#define	FAN_IOC_MAGIC  			'k'
#define GET_FAN_STATE   		_IOR(FAN_IOC_MAGIC, 1, uint8_t*)	//to get the fan status
#define GET_FAN_REGULTR_STATE	_IOR(FAN_IOC_MAGIC, 2, uint8_t*)	//to get the fan regulator state(enabled/disabled).

//device status
typedef enum fan_status {
	FAN_UNKNOWN,
	FAN_INIT_ERR,
	FAN_INIT_DONE,
	FAN_PROBE_ERR,
	FAN_READY,
	FAN_CONFIG_ERR,
	PWM_REG_ENABLE_ERR,
	FAN_OK,
} fan_status_t;

/* pwm fan device descriptor */
struct FAN_dev {
	dev_t  devnum_fan;      /* major/minor num */
	struct device *dev;     /* for device creation */
	struct cdev *cdev_fan;  /* char device descr for FAN */
};

struct fan_pwm_parameters {
	unsigned int period;
	unsigned int dutycycle;
};

struct pwm_fan_regulator_data {
	struct pwm_device *pwm;
	struct fan_pwm_parameters pwm_fan_parameters;
	bool enabled;
};

//#define DEBUG

#ifdef DEBUG
#define DBG(args...)    printk(args)
#else
#define DBG(args...)    do { } while(0)
#endif

#endif	//__FAN_CONTROL_H
