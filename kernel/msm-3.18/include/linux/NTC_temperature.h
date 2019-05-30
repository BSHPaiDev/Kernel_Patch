#ifndef __NTC_SENSOR_H__
#define __NTC_SENSOR_H__

/**
 * @defgroup    drivers_VADC temperature sensor
 * @ingroup     drivers_sensors
 *
 * @{
 *
 * @file
 * @brief       Driver for ±1°C Accurate, 15-Bit NTC
 *              Temperature Sensor.
 *
 * @author	E-infochips
 */
#include <linux/module.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/spmi.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <uapi/linux/stat.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>

#define CLS_NAME    "tm_sensor"
#define DRV_NAME    "NTC"

#define NTC_IOC_MAGIC  		'g'
#define IOCTL_NTC_STATUS	_IOR(NTC_IOC_MAGIC, 1, uint8_t*)

//device status
typedef enum ntc_status {
	NTC_UNKNOWN,
	NTC_INIT_ERR,
	NTC_INIT_DONE,
	NTC_PROBE_ERR,
	NTC_READY,
	NTC_READ_ERR,
	NTC_OK
} ntc_status_t;

/* ntc sensor device descriptor */
struct NTC_dev {
	dev_t  devnum_ntc;       /* major/minor num */
	struct class *class_ntc; /* reference to class object */
	struct device *dev;      /* for device creation */
	struct cdev *cdev_ntc;   /* char device for NTC */
};

struct NTC_VADC {
	enum qpnp_vadc_channels adc_channel;
	struct device *dev;
	struct qpnp_vadc_chip *vadc_dev;
	struct spmi_device *spmi_dev;
	struct task_struct *thread_st;
	int	delay;
};

#endif	//__NTC_SENSOR_H__
