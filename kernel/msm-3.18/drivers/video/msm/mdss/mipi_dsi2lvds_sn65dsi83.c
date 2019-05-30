#include "mipi_dsi2lvds_sn65dsi83.h"

struct sn65dsi83_platform_data *pData = NULL;

void switch_backlight(int enable)
{
	gpio_set_value(pData->backlight_en_gpio, enable);
}
EXPORT_SYMBOL(switch_backlight);

int sn65dsi83_stop_video(void)
{
	int ret = 0;

	/* Disabling the PLL */
	ret = i2c_smbus_write_byte_data(pData->sn65dsi83_client, SN65DSI83_PLL_EN, 0x00);
	if(ret < 0) {
		dev_err(&pData->sn65dsi83_client->dev, "Failed to write reg: %02x", SN65DSI83_PLL_EN);
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL(sn65dsi83_stop_video);

int sn65dsi83_start_video(void)
{
	int ret = 0;

	/* Enabling the PLL */
	ret = i2c_smbus_write_byte_data(pData->sn65dsi83_client, SN65DSI83_PLL_EN, 0x01);
	if(ret < 0) {
		dev_err(&pData->sn65dsi83_client->dev, "Failed to write reg: %02x", SN65DSI83_PLL_EN);
		return ret;
	}

	/*As per sequence mentioned in Bridge Chip*/
	mdelay(10);

	/* Enabling the Reset condition of device */
	ret = i2c_smbus_write_byte_data(pData->sn65dsi83_client, SN65DSI83_SOFT_RESET, 0x01);
	if(ret < 0) {
		dev_err(&pData->sn65dsi83_client->dev, "Failed to write reg: %02x", SN65DSI83_SOFT_RESET);
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL(sn65dsi83_start_video);

/*Added option function to readout Bridge chip's register*/
#if 0
static int sn65dsi83_i2c_read(struct i2c_client *client, u8 reg, u8 *val)
{
	int ret = 0;

	/* Select register to write*/
	ret = i2c_smbus_write_byte(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed to write register %x\n",
			__func__, reg);
		return ret;
	}

	/*	 read the data */
	ret = i2c_smbus_read_byte(client);
	if(ret >= 0) {
		*val = (u8) ret;
	} else {
		dev_err(&client->dev, "%s: failed to read register %x\n",
			__func__, reg);
	}
	return ret;
}
#endif

static void sn65dsi83_asser_en(void)
{
	int ret = 0;

	ret = gpio_request(pData->bridge_en_gpio, "bridge_en");
	if(ret < 0){
		dev_err(&pData->sn65dsi83_client->dev, "Failed to assert bridge chip\n");
	}
	else
	{
		ret = gpio_export(pData->bridge_en_gpio, true);
		if(ret < 0)
			dev_err(&pData->sn65dsi83_client->dev, "Failed to export gpio: %d\n", pData->bridge_en_gpio);

		gpio_direction_output(pData->bridge_en_gpio, 1);
		mdelay(1);
		gpio_set_value(pData->bridge_en_gpio, 0);
		mdelay(1);
		gpio_set_value(pData->bridge_en_gpio, 1);
	}

	ret = gpio_request(pData->backlight_en_gpio, "backlight_en");
	if(ret < 0){
		        dev_err(&pData->sn65dsi83_client->dev, "Failed to enable backlight\n");
	}
	else {
		ret = gpio_export(pData->backlight_en_gpio, true);
		if(ret < 0)
			dev_err(&pData->sn65dsi83_client->dev, "Failed to export gpio: %d\n", pData->backlight_en_gpio);
                //Temp change will remove later
                gpio_direction_output(pData->backlight_en_gpio, 1);
	}
}



static int sn65dsi83_parse(struct device *dev, struct sn65dsi83_platform_data *pData)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	do {
		pData->backlight_en_gpio = of_get_named_gpio(np, "backlight_en", 0);
		if((!gpio_is_valid(pData->backlight_en_gpio))) {
			dev_err(dev,"%s: Invalid GPIO for backlight_en\n", __func__);
			ret = 1;
			break;
		}
		printk ("DEBUG : backlight_en GPIO is %d\n", pData->backlight_en_gpio);

		pData->bridge_en_gpio = of_get_named_gpio(np, "bridge_en", 0);
		if((!gpio_is_valid(pData->bridge_en_gpio))) {
			dev_err(dev,"%s: Invalid GPIO for bridge_en\n", __func__);
			ret = 1;
			break;
		}

	} while (0);
	return ret;
}

static int sn65dsi83_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	pData = kzalloc(sizeof(*pData), GFP_KERNEL);
	if(!pData) {
		dev_err(&client->dev, "kzalloc failed\n");
		return -ENOMEM;
	}

	pData->sn65dsi83_client = client;

	ret = sn65dsi83_parse(&client->dev, pData);
	if(ret)
	{
		dev_err(&client->dev,"%s: Failed to parse DT\n", __func__);
		return ret;
	}

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev, "adapter doesn't support SMBus word "
			"transactions\n");
		return -ENODEV;
	}

	i2c_set_clientdata(client, (void *)pData);

	sn65dsi83_asser_en();

	/* To set horizontal display resolution LSB bit 7:0 to 1280 */
	ret = i2c_smbus_write_byte_data(client, SN65DSI83_ACTIVE_LINE_LENGTH_LOW, 0x00);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to write reg: %02x", SN65DSI83_ACTIVE_LINE_LENGTH_LOW);
		return ret;
	}

	/* To set horizontal display resolution MSB bit 3:0 to 1280 */
	ret = i2c_smbus_write_byte_data(client, SN65DSI83_ACTIVE_LINE_LENGTH_HIGH, 0x05);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to write reg: %02x", SN65DSI83_ACTIVE_LINE_LENGTH_HIGH);
		return ret;
	}

	/* To set vertical display resolution LSB bit 7:0 to 800 */
	ret = i2c_smbus_write_byte_data(client, SN65DSI83_VERTICAL_DISPLAY_SIZE_LOW, 0x20);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to write reg: %02x", SN65DSI83_VERTICAL_DISPLAY_SIZE_LOW);
		return ret;
	}

	/* To set vertical display resolution MSB bit 3:0 to 800 */
	ret = i2c_smbus_write_byte_data(client, SN65DSI83_VERTICAL_DISPLAY_SIZE_HIGH, 0x03);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to write reg: %02x", SN65DSI83_VERTICAL_DISPLAY_SIZE_HIGH);
		return ret;
	}

	/* To set delay for Pixel clock LSB bits 7:0 32 pixel clock */
	ret = i2c_smbus_write_byte_data(client, SN65DSI83_SYNC_DELAY_LOW, 0x20);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to write reg: %02x", SN65DSI83_SYNC_DELAY_LOW);
		return ret;
	}

	/* To set delay for Pixel clock LSB bits 7:0 32 pixel clock */
	ret = i2c_smbus_write_byte_data(client, SN65DSI83_SYNC_DELAY_HIGH, 0x00);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to write reg: %02x", SN65DSI83_SYNC_DELAY_HIGH);
		return ret;
	}

	/* To set width of horizontal sync pulse of LSB bits to 80 pixel clock */
	ret = i2c_smbus_write_byte_data(client, SN65DSI83_HSYNC_PULSE_WIDTH_LOW, 0x50);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to write reg: %02x", SN65DSI83_HSYNC_PULSE_WIDTH_LOW);
		return ret;
	}

	/* To set width of horizontal sync pulse of MSB bits to 80 pixel clock */
	ret = i2c_smbus_write_byte_data(client, SN65DSI83_HSYNC_PULSE_WIDTH_HIGH, 0x00);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to write reg: %02x", SN65DSI83_HSYNC_PULSE_WIDTH_HIGH);
		return ret;
	}

	/* To set width of vertical sync pulse of LSB bits to 09 pixel clock */
	ret = i2c_smbus_write_byte_data(client, SN65DSI83_VSYNC_PULSE_WIDTH_LOW, 0x09);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to write reg: %02x", SN65DSI83_VSYNC_PULSE_WIDTH_LOW);
		return ret;
	}

	/* To set width of vertical sync pulse of MSB bits to 09 pixel clock */
	ret = i2c_smbus_write_byte_data(client, SN65DSI83_VSYNC_PULSE_WIDTH_HIGH, 0x00);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to write reg: %02x", SN65DSI83_VSYNC_PULSE_WIDTH_HIGH);
		return ret;
	}

	/* To set the back porch for horizontal active line 40 pixel clock */
	ret = i2c_smbus_write_byte_data(client, SN65DSI83_H_BACK_PORCH, 0x28);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to write reg: %02x", SN65DSI83_H_BACK_PORCH);
		return ret;
	}

	/* To set the back porch for vertical active line 07 pixel clock */
	ret = i2c_smbus_write_byte_data(client, SN65DSI83_V_BACK_PORCH, 0x07);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to write reg: %02x", SN65DSI83_V_BACK_PORCH);
		return ret;
	}

	/* To set the front porch for horizontal active line 40 pixel clock */
	ret = i2c_smbus_write_byte_data(client, SN65DSI83_H_FRONT_PORCH, 0x28);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to write reg: %02x", SN65DSI83_H_FRONT_PORCH);
		return ret;
	}

	/* To set the front porch for vertical active line 07 pixel clock */
	ret = i2c_smbus_write_byte_data(client, SN65DSI83_V_FRONT_PORCH, 0x07);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to write reg: %02x", SN65DSI83_V_FRONT_PORCH);
		return ret;
	}

	/* To set the LVDS output clock range between 62 MHz to 87 MHz */
	/* Setting the LVDS clock from DSI channel A */
	ret = i2c_smbus_write_byte_data(client, SN65DSI83_LVDS_CLK_RANGE, 0x05);
	//ret = i2c_smbus_write_byte_data(client, SN65DSI83_LVDS_CLK_RANGE, 0x04); /*FOr test pattern*/
	if(ret < 0) {
		dev_err(&client->dev, "Failed to write reg: %02x", SN65DSI83_LVDS_CLK_RANGE);
		return ret;
	}

	/* Setting the divider for LVDS clock range output to divide by 3 */
	ret = i2c_smbus_write_byte_data(client, SN65DSI83_DIVIDER_MULTIPLIER, 0x10);/*For divide by 3*/
	//ret = i2c_smbus_write_byte_data(client, SN65DSI83_DIVIDER_MULTIPLIER, 0x02);/*FOr test pattern*/
	if(ret < 0) {
		dev_err(&client->dev, "Failed to write reg: %02x", SN65DSI83_DIVIDER_MULTIPLIER);
		return ret;
	}

	/* Selection of input DSI lanes for data set to 4 lanes */
	ret = i2c_smbus_write_byte_data(client, SN65DSI83_DSI_LANES, 0x26);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to write reg: %02x", SN65DSI83_DSI_LANES);
		return ret;
	}

	/* Setting DSI clock at range between 210 MHz to 215 MHz */
	ret = i2c_smbus_write_byte_data(client, SN65DSI83_DSI_CLK_RANGE, 0x2A);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to write reg: %02x", SN65DSI83_DSI_CLK_RANGE);
		return ret;
	}

	/* Selecting data format for LVDS output from IC and setting the lanes */
	/* for LVDS output , set to 'format 2' and '4' output lanes enabled */
	ret = i2c_smbus_write_byte_data(client, SN65DSI83_FORMAT, 0x78);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to write reg: %02x", SN65DSI83_FORMAT);
		return ret;
	}

	/* For LVDS output, Test pattern generation only*/
	/*ret = i2c_smbus_write_byte_data(client, SN65DSI83_TEST_PATTERN, 0x10);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to write reg: %02x", SN65DSI83_TEST_PATTERN);
		return ret;
	}*/

	mdelay(100);

	return 0;
}

static int sn65dsi83_remove(struct i2c_client *client)
{
	struct sn65dsi83_platform_data *pData = NULL;

	pData = i2c_get_clientdata(client);
	if (!pData) {
		dev_err(&client->dev,
			"%s: device doesn't exist anymore\n", __func__);
		return -ENODEV;
	}
	sn65dsi83_stop_video();
	gpio_free(pData->backlight_en_gpio);
	gpio_free(pData->bridge_en_gpio);
	kfree(pData);
	return 0;
}

static const struct i2c_device_id sn65dsi83_idtable [] = {
	{ "sn65dsi83", 0},
	{}
};

#if CONFIG_OF
static const struct of_device_id sn65dsi83_of_idtable[] = {
	{
		.compatible = "ti,sn65dsi83"
	},
	{}
};
#else
#define sn65dsi83_of_idtable NULL
#endif

/* Driver definition */
static struct i2c_driver sn65dsi83_driver = {
	.driver = {
		.name = "sn65dsi83",
		.owner = THIS_MODULE,
		.of_match_table = sn65dsi83_of_idtable,
	},
	.probe = sn65dsi83_probe,
	.remove = sn65dsi83_remove,
	.id_table = sn65dsi83_idtable,
};

int sn65dsi83_init(void)
{
	return i2c_add_driver(&sn65dsi83_driver);
}
EXPORT_SYMBOL(sn65dsi83_init);

void __exit sn65dsi83_exit(void)
{
	i2c_del_driver(&sn65dsi83_driver);
}
EXPORT_SYMBOL(sn65dsi83_exit);

module_exit(sn65dsi83_exit);

MODULE_DESCRIPTION("I2C driver for sn65dsi83 MIPI DSI to LVDS convertor");
MODULE_LICENSE("GPL");
