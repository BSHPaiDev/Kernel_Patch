/**
 * @defgroup    drivers_VADC temperature sensor
 * @ingroup     drivers_sensors
 *
 * @{
 *
 * @file
 * @brief       Driver for ±1°C Accurate, 15-Bit VOUT
 *              Temperature Sensor.
 *
 * @author	RBEI
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

struct VOUT_VADC{
	enum qpnp_vadc_channels         adc_channel;
	struct device			*dev;
	struct qpnp_vadc_chip		*vadc_dev;
	struct spmi_device		*spmi_dev;
	struct task_struct		*thread_st;
	int				delay;
};

