/* Source for:
 * Cypress TrueTouch(TM) Standard Product I2C touchscreen driver.
 * drivers/input/touchscreen/cyttsp_i2c.c
 *
 * Copyright (C) 2009-2011 Cypress Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Cypress reserves the right to make changes without further notice
 * to the materials described herein. Cypress does not assume any
 * liability arising out of the application described herein.
 *
 * Contact Cypress Semiconductor at www.cypress.com
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/cyttsp.h>
#include "cyttsp3_core.h"

#define DBG(x)

struct cyttsp_i2c {
	struct cyttsp_bus_ops ops;
	struct i2c_client *client;
	void *ttsp_client;
};

static s32 ttsp_i2c_read_block_data(void *handle, u8 addr,
	u8 length, void *values)
{
	int retval = 0;
	struct cyttsp_i2c *ts = container_of(handle, struct cyttsp_i2c, ops);
	retval = i2c_master_send(ts->client, &addr, 1);
	if (retval < 0)
		return retval;
	retval = i2c_master_recv(ts->client, values, length);

	return (retval < 0) ? retval : 0;
}

static s32 ttsp_i2c_write_block_data(void *handle, u8 addr,
	u8 length, const void *values)
{
	u8 data[I2C_SMBUS_BLOCK_MAX+1];
	int num_bytes, count;
	int retval;
	struct cyttsp_i2c *ts = container_of(handle, struct cyttsp_i2c, ops);
	num_bytes = length;
	data[0] = addr;
	count = (num_bytes > I2C_SMBUS_BLOCK_MAX) ?
		I2C_SMBUS_BLOCK_MAX : num_bytes;
	memcpy(&data[1], values, count+1);
	num_bytes -= count;
	retval = i2c_master_send(ts->client, data, count+1);
	if (retval < 0)
		return retval;
	while (num_bytes > 0) {
		count = (num_bytes > I2C_SMBUS_BLOCK_MAX) ?
			I2C_SMBUS_BLOCK_MAX : num_bytes;
		memcpy(&data[0], values, count);
		num_bytes -= count;
		retval = i2c_master_send(ts->client, data, count);
		if (retval < 0)
			return retval;
	}

	return 0;
}

static s32 ttsp_i2c_tch_ext(void *handle, void *values)
{
	int retval = 0;
	struct cyttsp_i2c *ts = container_of(handle, struct cyttsp_i2c, ops);

	DBG(printk(KERN_INFO "%s: Enter\n", __func__);)

	/* Add custom touch extension handling code here */
	/* set: retval < 0 for any returned system errors,
		retval = 0 if normal touch handling is required,
		retval > 0 if normal touch handling is *not* required */
	if (!ts || !values)
		retval = -EIO;

	return retval;
}

static s32 cyttsp_check_device(struct i2c_client *client, u8 addr)
{
	int ret;
	u8 data;
	
	data = addr;

	/*client->addr = 0x00;*/
	/*ret = i2c_master_send(client, &addr, 1);*/
	/*printk(KERN_INFO "ret :%d, data:0x%02x, addr:0x%02x\n", ret, data, client->addr);*/
	ret = i2c_master_recv(client, &data, sizeof(data));
	if((ret != sizeof(data)) || (data == 0xff)) {
		printk(KERN_ERR "No FIND 0x%02x Touchscreen Device!\n", client->addr);
		return -EIO;
	}

	return 0;
}

static int __devinit cyttsp_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct cyttsp_i2c *ts;
	struct cyttsp_platform_data *pdata = client->dev.platform_data;
	int retval;

	if (!pdata) {
		dev_err(&client->dev, "platform data not available.\n");
		return -EINVAL;
	}

	if (pdata->init)
		retval = pdata->init(1);
	if (retval) {
		printk(KERN_ERR "%s: platform init failed! \n", __func__);
		goto error_init;
	}

	printk(KERN_INFO "====== check 0x%02x device ======\n", client->addr);
	retval = cyttsp_check_device(client, 0x00);
	if(retval < 0) {
		return -ENODEV;
	}

	/* allocate and clear memory */
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		printk(KERN_ERR "%s: Error, kzalloc.\n", __func__);
		retval = -ENOMEM;
		goto error_alloc_data_failed;
	}

	/* register driver_data */
	ts->client = client;
	i2c_set_clientdata(client, ts);
	ts->ops.write = ttsp_i2c_write_block_data;
	ts->ops.read = ttsp_i2c_read_block_data;
	ts->ops.ext = ttsp_i2c_tch_ext;
	if(client->addr == 0x24)
		ts->ops.touchscreen_type = TOUCHSCREEN_XC3;
	else
		ts->ops.touchscreen_type = TOUCHSCREEN_OED;

	device_init_wakeup(&client->dev, 1);

	ts->ttsp_client = cyttsp_core_init(&ts->ops, &client->dev);
	if (!ts->ttsp_client)
		goto ttsp_core_err;

	DBG(printk(KERN_INFO "%s: Registration complete %s\n",
		__func__, CY_I2C_NAME);)
	return 0;

ttsp_core_err:
	kfree(ts);
error_init:
	if (pdata->init)
		pdata->init(0);
error_alloc_data_failed:
	return retval;
}

/* registered in driver struct */
static int __devexit cyttsp_i2c_remove(struct i2c_client *client)
{
	struct cyttsp_i2c *ts;
	struct cyttsp_platform_data *pdata = client->dev.platform_data;

	DBG(printk(KERN_INFO"%s: Enter\n", __func__);)

	if (!pdata) {
		dev_err(&client->dev, "platform data not available.\n");
	}
	ts = i2c_get_clientdata(client);
	cyttsp_core_release(ts->ttsp_client);
	kfree(ts);
	if (pdata->init)
		pdata->init(0);
	return 0;
}

static const struct i2c_device_id cyttsp_i2c_id[] = {
	{ CY_I2C_NAME, 0 },  { }
};

static struct i2c_driver cyttsp_i2c_driver = {
	.driver = {
		.name = CY_I2C_NAME,
		.owner = THIS_MODULE,
	},
	.probe = cyttsp_i2c_probe,
	.remove = __devexit_p(cyttsp_i2c_remove),
	.id_table = cyttsp_i2c_id,
};

static int cyttsp_i2c_init(void)
{
	int retval;
	retval = i2c_add_driver(&cyttsp_i2c_driver);
	return retval;
}

static void cyttsp_i2c_exit(void)
{
	return i2c_del_driver(&cyttsp_i2c_driver);
}

module_init(cyttsp_i2c_init);
module_exit(cyttsp_i2c_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product I2C driver");
MODULE_AUTHOR("Cypress");
MODULE_DEVICE_TABLE(i2c, cyttsp_i2c_id);
