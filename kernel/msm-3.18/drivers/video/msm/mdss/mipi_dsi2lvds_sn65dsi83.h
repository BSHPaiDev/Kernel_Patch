#ifndef MSM_DSI2LVDS_SN65DSI83_H
#define MSM_DSI2LVDS_SN65DSI83_H

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

/* Bridge chip register set */
#define SN65DSI83_SOFT_RESET					        0x09
#define SN65DSI83_LVDS_CLK_RANGE				        0x0A
#define SN65DSI83_DIVIDER_MULTIPLIER					0x0B
#define SN65DSI83_PLL_EN						0x0D
#define SN65DSI83_DSI_LANES						0x10
#define SN65DSI83_DATA_EQ						0x11
#define SN65DSI83_DSI_CLK_RANGE						0x12
#define SN65DSI83_FORMAT						0x18
#define SN65DSI83_LVDS_SWING						0x19
#define SN65DSI83_LVDS_TERM						0x1A
#define SN65DSI83_LVDS_CM_ADJUST					0x1B
#define SN65DSI83_ACTIVE_LINE_LENGTH_LOW				0x20
#define SN65DSI83_ACTIVE_LINE_LENGTH_HIGH				0x21
#define SN65DSI83_VERTICAL_DISPLAY_SIZE_LOW				0x24
#define SN65DSI83_VERTICAL_DISPLAY_SIZE_HIGH				0x25
#define SN65DSI83_SYNC_DELAY_LOW					0x28
#define SN65DSI83_SYNC_DELAY_HIGH					0x29
#define SN65DSI83_HSYNC_PULSE_WIDTH_LOW					0x2C
#define SN65DSI83_HSYNC_PULSE_WIDTH_HIGH				0x2D
#define SN65DSI83_VSYNC_PULSE_WIDTH_LOW					0x30
#define SN65DSI83_VSYNC_PULSE_WIDTH_HIGH				0x31
#define SN65DSI83_H_BACK_PORCH						0x34
#define SN65DSI83_V_BACK_PORCH						0x36
#define SN65DSI83_H_FRONT_PORCH						0x38
#define SN65DSI83_V_FRONT_PORCH						0x3A
#define SN65DSI83_TEST_PATTERN						0x3C

typedef struct sn65dsi83_platform_data
{
	u32 backlight_en_gpio;
	u32 bridge_en_gpio;
	struct i2c_client *sn65dsi83_client;
}sn65dsi83_platform_data;

#endif /* MSM_DSI2LVDS_SN65DSI83_H */
