/* drivers/regulator/charge-regulator-adc.c
 *
 * Copyright (C) 2013-2014 ROCKCHIP, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


/*include*/
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/adc.h>
#include <linux/timer.h>
#include <linux/regulator/act8931.h>
#include <asm/gpio.h>

/*define*/
#define ACT8931_CHARGE_DEBUG (0)

#if ACT8931_CHARGE_DEBUG
#define charge_printk(fmt, args...)  printk(KERN_INFO "act charge " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)
#else
#define charge_printk(fmt, args...)
#endif
#define charge_printk_err(fmt, args...) printk(KERN_INFO "act charge " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)

#define CHARGE_DELAY_TIME (1000)
#define CHARGE_INVALID_TIME (0x4FFFFFFF)

#define CHECK_ON (1)
#define CHECK_OFF (0)

#define GPIO_CHGLEV RK30_PIN1_PA1
#define GPIO_ACIN 	RK30_PIN1_PB2

enum charge_level{
	INVALID_CURRENT = 0,
	USB_100MA,
	USB_450MA,
	AC_500MA,
	AC_2000MA
};

struct act8931_charge{
	int gpio_chglev;
	int gpio_acin;

	struct adc_client	*client; 
	unsigned int bat_check_del_time;
	unsigned int invalid_time;
	struct timer_list	timer;

	int vbus_status;
	int usb_status;

	unsigned int check_falg;

	enum charge_level  charge_level;
	enum charge_level  chg_lev_bk;
};

static struct act8931_charge *act8931_charge_info = NULL;

static int act8931_charge_charge_output(struct act8931_charge *act_charge_info)
{
	charge_printk("enter act_charge_info->charge_level:%d\n",act_charge_info->charge_level);
	switch(act_charge_info->charge_level)
	{
		case USB_100MA:
		{
			gpio_direction_output(act_charge_info->gpio_acin, GPIO_LOW);
			gpio_direction_output(act_charge_info->gpio_chglev, GPIO_LOW);
		}
			break;

		case USB_450MA:
		{
			gpio_direction_output(act_charge_info->gpio_acin, GPIO_LOW);
			gpio_direction_output(act_charge_info->gpio_chglev, GPIO_HIGH);
		}
			break;

		case AC_500MA:
		{
			gpio_direction_output(act_charge_info->gpio_acin, GPIO_HIGH);
			gpio_direction_output(act_charge_info->gpio_chglev, GPIO_LOW);
		}
			break;

		case AC_2000MA:
		{
			gpio_direction_output(act_charge_info->gpio_acin, GPIO_HIGH);
			gpio_direction_output(act_charge_info->gpio_chglev, GPIO_HIGH);
		}
			break;


		default:
		{
			gpio_direction_output(act_charge_info->gpio_acin, GPIO_LOW);
			gpio_direction_output(act_charge_info->gpio_chglev, GPIO_LOW);
		}	
			break;
	}

	act_charge_info->chg_lev_bk = act_charge_info->charge_level;

	return 0;
}

static int act8931_charge_check_charge_level(struct act8931_charge *act_charge_info)
{
	charge_printk("enter\n");
#if 0
	if(act_charge_info->vbus_status)
	{
		if(act_charge_info->usb_status)
			act_charge_info->charge_level = USB_100MA;
		else
			act_charge_info->charge_level = AC_2000MA;
	}
	else
	{
		act_charge_info->charge_level = USB_100MA;
	}
#endif
	if(act_charge_info->charge_level != act_charge_info->chg_lev_bk)
		act8931_charge_charge_output(act_charge_info);

	return 0;
}

static inline void act8931_charge_timer_callback(unsigned long data)
{
	int ret;
	int ad_result = 0;
	struct act8931_charge *act_charge_info = (struct act8931_charge *)data;

	charge_printk("enter\n");

	// check charge level 
	if(CHECK_ON == act8931_charge_info->check_falg)
	{
		act8931_charge_info->check_falg = CHECK_OFF;
		act8931_charge_check_charge_level(act_charge_info);
		ret = mod_timer( &act_charge_info->timer, jiffies + msecs_to_jiffies(act_charge_info->invalid_time));
		if(ret)
			charge_printk_err("Error in mod_timer\n");
	}

	return;
}

static int act8931_charge_timer_init(struct act8931_charge *act_charge_info)
{
	int ret;

	setup_timer(&act_charge_info->timer, act8931_charge_timer_callback, (unsigned long)act_charge_info);
	ret = mod_timer( &act_charge_info->timer, jiffies + msecs_to_jiffies(act_charge_info->invalid_time) );
	if(ret)
		charge_printk_err("Error in mod_timer\n");
	return 0;
}

static void act8931_charge_timer_deinit(struct act8931_charge *act_charge_info)
{
	del_timer(&act_charge_info->timer);

	return;
}

static int act8931_charge_io_init(struct act8931_charge *act_charge_info)
{
	int err;

	err = gpio_request(act_charge_info->gpio_acin, "gpio_acin");
	if (err) {
		gpio_free(act_charge_info->gpio_acin);
		charge_printk_err("request gpio :%d fail.\n",act_charge_info->gpio_acin);
		return -1;
	}

	err = gpio_request(act_charge_info->gpio_chglev, "gpio_chglev");
	if (err) {
		gpio_free(act_charge_info->gpio_acin);
		gpio_free(act_charge_info->gpio_chglev);
		charge_printk_err("request gpio :%d fail.\n",act_charge_info->gpio_chglev);
		return -1;
	}

	return 0;
}

static int act8931_charge_io_deinit(struct act8931_charge *act_charge_info)
{
	gpio_free(act_charge_info->gpio_acin);
	gpio_free(act_charge_info->gpio_chglev);

	return 0;
}

int act8931_charge_init(void)
{
	if(act8931_charge_info)
		return 0;

	act8931_charge_info = kzalloc(sizeof(struct act8931_charge), GFP_KERNEL);
	if(!act8931_charge_info)
	{
		charge_printk_err("no memory\n");
		return -1;
	}

	act8931_charge_info->usb_status = 0;
	act8931_charge_info->vbus_status = 0;

	act8931_charge_info->charge_level = USB_100MA;
	act8931_charge_info->chg_lev_bk = INVALID_CURRENT;

	act8931_charge_info->gpio_chglev = GPIO_CHGLEV;
	act8931_charge_info->gpio_acin = GPIO_ACIN;

	if(0 != act8931_charge_io_init(act8931_charge_info))
	{
		kfree(act8931_charge_info);
		act8931_charge_info = NULL;
		charge_printk_err("act8931_charge_io_init err\n");
		return -1;
	}
	
	act8931_charge_info->check_falg = CHECK_OFF;
	act8931_charge_info->bat_check_del_time = CHARGE_DELAY_TIME;
	act8931_charge_info->invalid_time = CHARGE_INVALID_TIME;
	if(0 != act8931_charge_timer_init(act8931_charge_info))
	{
		act8931_charge_io_deinit(act8931_charge_info);
		kfree(act8931_charge_info);
		act8931_charge_info = NULL;
		charge_printk_err("act8931_charge_timer_init err\n");
		return -1;
	}

	act8931_charge_check_charge_level(act8931_charge_info);

	charge_printk("ok.\n");
	return 0;
}

int act8931_charge_deinit(void)
{
	if(NULL == act8931_charge_info)
		return 0;

	act8931_charge_io_deinit(act8931_charge_info);
	act8931_charge_timer_deinit(act8931_charge_info);
	if(act8931_charge_info)
	{
		kfree(act8931_charge_info);
		act8931_charge_info = NULL;
	}

	return 0;
}

int act8931_charge_set_vbus_status(int vbus_status)
{
	if(act8931_charge_info)
	{	
		act8931_charge_info->vbus_status = vbus_status;
		charge_printk("current vbus status:%d\n",act8931_charge_info->vbus_status);
		act8931_charge_info->check_falg = CHECK_ON;
		mod_timer(&act8931_charge_info->timer, jiffies + msecs_to_jiffies(act8931_charge_info->bat_check_del_time));
	}

	return 0;
}

int act8931_charge_set_usb_status(int usb_status)
{
	if(act8931_charge_info)
	{	
		act8931_charge_info->usb_status = usb_status;
		charge_printk("current usb status:%d\n",act8931_charge_info->usb_status);
		act8931_charge_info->check_falg = CHECK_ON;
		mod_timer(&act8931_charge_info->timer, jiffies + msecs_to_jiffies(act8931_charge_info->bat_check_del_time));
	}

	return 0;
}

//
// level:0  INVALID_CURRENT
// level:1  USB_100MA
// level:2  USB_450MA
// level:3  AC_500MA
// level:4  AC_2000MA
int act8931_charge_charge_level_set(int level)
{
	if(NULL == act8931_charge_info)
		return 0;

	switch(level)
	{	
		case 0:
			act8931_charge_info->charge_level = INVALID_CURRENT;
			break;
		case 1:
			act8931_charge_info->charge_level = USB_100MA;
			break;
		case 2:
			act8931_charge_info->charge_level = USB_450MA;
			break;
		case 3:
			act8931_charge_info->charge_level = AC_500MA;
			break;
		case 4:
			act8931_charge_info->charge_level = AC_2000MA;
			break;
			
		default:
			act8931_charge_info->charge_level = USB_100MA;
			break;
	
	}

	if(act8931_charge_info->charge_level != act8931_charge_info->chg_lev_bk)
	{
		charge_printk("current charge_level:%d\n",act8931_charge_info->charge_level);
		act8931_charge_info->check_falg = CHECK_ON;
		mod_timer(&act8931_charge_info->timer, jiffies + msecs_to_jiffies(act8931_charge_info->bat_check_del_time));
	}

	return 0;
}




