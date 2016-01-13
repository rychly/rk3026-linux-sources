/*
 * Copyright (C) 2013 ROCKCHIP, Inc.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/skbuff.h>
#include <linux/spi/spi.h>
#include <linux/mmc/host.h>
#include <linux/ion.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <linux/rk_fb.h>
#include <linux/regulator/machine.h>
#include <linux/rfkill-rk.h>
#include <linux/sensor-dev.h>
#include <linux/mfd/tps65910.h>
#include <linux/regulator/act8846.h>
#include <linux/regulator/rk29-pwm-regulator.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/hardware/gic.h>

#include <mach/dvfs.h>
#include <mach/board.h>
#include <mach/hardware.h>
#include <mach/io.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <linux/memblock.h>
#include <plat/efuse.h>
#if defined(CONFIG_WAVEFORM_FROM_GPIO_SPI)
#include <linux/spi/spi_gpio.h>
#endif

#if defined(CONFIG_SPIM_RK29)
#include "../../../drivers/spi/rk29_spim.h"
#endif

#ifdef CONFIG_SND_SOC_RK3026
#include "../../../sound/soc/codecs/rk3026_codec.h"
#endif

#if defined(CONFIG_RK_HDMI)
        #include "../../../drivers/video/rockchip/hdmi/rk_hdmi.h"
#endif

#include "../../../drivers/video/rockchip/rk_epd/ebc.h"

#if defined (CONFIG_TOUCHSCREEN_XC3) || defined(CONFIG_TOUCHSCREEN_OED)
#include <linux/cyttsp.h>
#endif

#if defined (CONFIG_TOUCHSCREEN_CY8CTMA461)
#include <linux/input/cy8ctma461.h>
#endif

#if defined (CONFIG_RK_HEADSET_DET)
#include "../../../drivers/headset_observe/rk_headset.h"
#endif

#if defined(CONFIG_REGULATOR_ACT8931)
extern  int act8931_charge_det ;
extern  int act8931_charge_ok  ;
#endif

/***********************************************************
*	board config
************************************************************/
//system power on
#define POWER_ON_PIN		INVALID_GPIO   //PWR_HOLD

//rtc
#if defined(CONFIG_SPI)
#define RTC_INT_PIN		RK30_PIN1_PA0
#else
#define RTC_INT_PIN		RK30_PIN1_PB3
#endif

//touchscreen
#define TOUCH_RST_PIN		RK30_PIN2_PB4
#define TOUCH_RST_VALUE		GPIO_HIGH
#define TOUCH_PWR_PIN		INVALID_GPIO//RK2928_PIN2_PB3
#define TOUCH_PWR_VALUE		GPIO_LOW
#if defined(CONFIG_SPI)
  #if defined(CONFIG_MACH_RK3026_E601) || defined(CONFIG_MACH_RK3026_T63)
    #define TOUCH_INT_PIN	RK30_PIN1_PA4
  #else
    #define TOUCH_INT_PIN	RK30_PIN1_PA2
  #endif
#else
  #define TOUCH_INT_PIN		RK30_PIN1_PB0
#endif

#ifdef CONFIG_BACKLIGHT_RK29_BL
//backlight
#define LCD_DISP_ON_PIN
#define BL_PWM			0  // (0 ~ 2)
#define PWM_EFFECT_VALUE  	0
#if defined(CONFIG_SPI)
#define BL_EN_PIN		RK30_PIN0_PD3
#else
#define BL_EN_PIN		RK30_PIN1_PA1
#endif
#define BL_EN_VALUE		GPIO_HIGH
#endif

//fb
#define LCD_EN_PIN		INVALID_GPIO//RK2928_PIN1_PB3
#define LCD_EN_VALUE		GPIO_HIGH
#define LCD_CS_PIN		INVALID_GPIO
#define LCD_CS_VALUE		GPIO_HIGH

//gsensor
#define GS_INT_PIN		INVALID_GPIO//RK2928_PIN1_PB2

//sdmmc
//Reference to board-rk3026-tb-sdmmc-config.c

//keyboard
//#define RK31XX_MAINBOARD_V1      //if mainboard is RK31XX_MAINBOARD_V1.0
#define PLAY_ON_PIN		INVALID_GPIO	//wakeup key		

//pwm regulator
#define REG_PWM			0//1  // (0 ~ 2)

//pmic
#if defined(CONFIG_SPI)
  #if defined(CONFIG_MACH_RK3026_E601) || defined(CONFIG_MACH_RK3026_T63)
    #define PMU_INT_PIN		RK30_PIN1_PA2
  #else
    #define PMU_INT_PIN		RK30_PIN1_PA4
  #endif
#else
  #define PMU_INT_PIN		RK30_PIN1_PB1
#endif
#define PMU_SLEEP_PIN		INVALID_GPIO

//ion reserve memory
#define ION_RESERVE_SIZE        (20 * SZ_1M)

//ebc reserve memory
#define EBC_RESERVE_SIZE        (1 * SZ_1M)

static int pwm_mode[] = {PWM0, PWM1, PWM2};
static inline int rk_gpio_request(int gpio, int direction, int value, const char *label)
{
	int ret = 0;
	unsigned long flags = 0;

	if(!gpio_is_valid(gpio))
		return 0;

	if(direction == GPIOF_DIR_IN)
		flags = GPIOF_IN;
	else if(value == GPIO_LOW)
		flags = GPIOF_OUT_INIT_LOW;
	else
		flags = GPIOF_OUT_INIT_HIGH;

	ret = gpio_request_one(gpio, flags, label);
	if(ret < 0)
		pr_err("Failed to request '%s'\n", label);

	return ret;
}
#if defined(CONFIG_WAVEFORM_FROM_GPIO_SPI)
static void init_spi_gpio(void)
{
	iomux_set(GPIO2_B4);
	iomux_set(GPIO2_B5);
	iomux_set(GPIO2_B6);
	iomux_set(GPIO2_B7);
}

static struct spi_gpio_platform_data spi_gpio_plat_data = {
	.sck  = RK30_PIN2_PB6,
	.miso = RK30_PIN2_PB5,
	.mosi = RK30_PIN2_PB4,
	.num_chipselect = 1,
};
static struct platform_device spi_gpio_device = {
	.name	= "spi_gpio",
	.id 	= 0,
	.dev	= {
		.platform_data  = &spi_gpio_plat_data,
	}
};
#endif

static struct spi_board_info board_spi_devices[] = {
#if defined(CONFIG_WAVEFORM_FROM_GPIO_SPI)
{
	.modalias  = "panel_flash_spi",
	.max_speed_hz  = 1*1000*1000,
	.bus_num = 0,                   /* = device id */
	.chip_select   = 0,             /* not used for gpio spi */
	.controller_data = (void *)RK30_PIN2_PB7,
},
#endif
#if defined(CONFIG_WAVEFORM_FROM_RK_SPI)
{   /*spi flash add by Caesar 2014-05-30*/
	.modalias   = "epd_spi_flash",
	.chip_select    = 0,
	.max_speed_hz   = 12*1000000,//20*1000000,
	.bus_num    = 0,        
},
#endif

};

#if defined (CONFIG_RK_HEADSET_DET)
static int rk_headset_io_init(int gpio)
{
	int ret;

	iomux_set(GPIO1_A5);
	ret = gpio_request(gpio, "headset_input");
	if(ret) {
		gpio_free(gpio);
		printk("request headset input gpio error!\n");
		return -EIO;
	}

	gpio_pull_updown(gpio, PullDisable);
	gpio_direction_input(gpio);
	mdelay(50);
	return 0;
};
struct rk_headset_pdata rk_headset_info = {
	.Headset_gpio           = RK30_PIN1_PA5,
	.headset_in_type = HEADSET_IN_LOW,
	.headset_io_init = rk_headset_io_init,
};
struct platform_device rk_device_headset = {
	.name   = "rk_headsetdet",
	.id     = 0,
	.dev    = {
		.platform_data = &rk_headset_info,
	}
};
#endif


/***********************************************************
*	touchscreen
************************************************************/
#if defined(CONFIG_INPUT_TOUCHSCREEN)
static int tp_power_on(int on)
{
	struct regulator *ldo_tp = NULL;
	int ret = 0;

	ldo_tp = regulator_get(NULL, "vcc_tp");
	if (!ldo_tp) {
		printk("tp power not available.\n");
		return -1;
	}

	if(on) {
		if (!regulator_is_enabled(ldo_tp)) {
			regulator_enable(ldo_tp);
			printk("%s: tp power enable\n", __func__);
		}
	} else {
		while(regulator_is_enabled(ldo_tp)>0) {
			regulator_disable(ldo_tp);
			printk("%s: tp power disable\n", __func__);
		}
	}
	regulator_put(ldo_tp);
	msleep(50);

	return ret;
}
#endif

#if defined(CONFIG_TOUCHSCREEN_GT8XX)
static int goodix_init_platform_hw(void)
{
	int ret  = 0;

	ret = rk_gpio_request(TOUCH_PWR_PIN, GPIOF_DIR_OUT, TOUCH_PWR_VALUE, "touch_pwr");
	if(ret < 0)
		return ret; 
	msleep(100);

	ret = rk_gpio_request(TOUCH_RST_PIN, GPIOF_DIR_OUT, TOUCH_RST_VALUE, "touch_rst");
	if(ret < 0)
		return ret; 
	msleep(100);

	return 0;
}

struct goodix_platform_data goodix_info = {
	.model = 8105,
	.irq_pin = TOUCH_INT_PIN,
	.rest_pin = TOUCH_RST_PIN,
	.init_platform_hw = goodix_init_platform_hw,
};
#endif

#if defined (CONFIG_TOUCHSCREEN_CY8CTMA461)

#define CY_MAXX 758
#define CY_MAXY 1024
#define CY_MINX 0
#define CY_MINY 0

#define CY_ABS_MIN_X CY_MINX
#define CY_ABS_MIN_Y CY_MINY
#define CY_ABS_MAX_X CY_MAXX
#define CY_ABS_MAX_Y CY_MAXY
#define CY_ABS_MIN_P 0
#define CY_ABS_MAX_P 255
#define CY_ABS_MIN_W 0
#define CY_ABS_MAX_W 255

#define CY_ABS_MIN_T 0

#define CY_ABS_MAX_T 15

#define CY_IGNORE_VALUE 0xFFFF

static int cyttsp4_power(int on)
{
	int rc = 0;

	printk("%s, state: %d\n", __func__, on);

	rc = tp_power_on(on);

	return rc;
}

static int cyttsp4_xres(struct cy8ctma46x_platform_data *pdata)
{
	int rc = 0;

	int rst_gpio = pdata->reset_pin;

	cyttsp4_power(1);
	msleep(20);
	cyttsp4_power(0);
	msleep(40);
	cyttsp4_power(1);
	msleep(20);

	gpio_set_value(rst_gpio, 1);
	msleep(20);
	gpio_set_value(rst_gpio, 0);
	msleep(40);
	gpio_set_value(rst_gpio, 1);
	msleep(20);

	return rc;
}

static int cyttsp4_init_platform_hw(void)
{
	int ret  = 0;

	printk("%s, enter...\n", __func__);

	ret = rk_gpio_request(TOUCH_INT_PIN, GPIOF_DIR_IN, 0, "tp_int");
	if(ret < 0) {
		printk("%s, request gpio for tp_int failed!\n", __func__);
		return ret; 
	}
	gpio_pull_updown(TOUCH_INT_PIN, PullDisable);
	gpio_direction_input(TOUCH_INT_PIN);
	msleep(50);


	ret = rk_gpio_request(TOUCH_RST_PIN, GPIOF_DIR_OUT, TOUCH_RST_VALUE, "tp_rst");
	if(ret < 0) {
		printk("%s, request gpio for tp_rst failed!\n", __func__);
		return ret; 
	}
	gpio_direction_output(TOUCH_RST_PIN, 1);
	msleep(50);

	return 0;
}

static const uint16_t cyttsp4_abs[] = {
	ABS_MT_POSITION_X, CY_ABS_MIN_X, CY_ABS_MAX_X, 0, 0,
	ABS_MT_POSITION_Y, CY_ABS_MIN_Y, CY_ABS_MAX_Y, 0, 0,
	CY_IGNORE_VALUE, CY_ABS_MIN_P, CY_ABS_MAX_P, 0, 0,
	CY_IGNORE_VALUE, CY_ABS_MIN_W, CY_ABS_MAX_W, 0, 0,
	ABS_MT_TRACKING_ID, CY_ABS_MIN_T, CY_ABS_MAX_T, 0, 0,
	CY_IGNORE_VALUE, 0, 255, 0, 0,
	CY_IGNORE_VALUE, 0, 255, 0, 0,
	CY_IGNORE_VALUE, -128, 127, 0, 0,
};

struct touch_framework cyttsp4_framework = {
	.abs = (uint16_t *)&cyttsp4_abs[0],
	.size = ARRAY_SIZE(cyttsp4_abs),
	.enable_vkeys = 0,
};

static struct cyttsp4_mt_platform_data _cyttsp4_mt_platform_data = {
	.frmwrk = &cyttsp4_framework,
	.flags = 0x08 | 0x20,           /*flip+inv_y*/
	.inp_dev_name = CYTTSP4_MT_NAME,
};

struct cy8ctma46x_platform_data cyttsp4_info = {
	.irq_pin = TOUCH_INT_PIN,
	.reset_pin = TOUCH_RST_PIN,
	.power = cyttsp4_power,
	.xres = cyttsp4_xres,
	.init_platform_hw = cyttsp4_init_platform_hw,
	.md_platform_data = &_cyttsp4_mt_platform_data,
};
#endif

/***********************************************************
*	rk30  backlight
************************************************************/
#ifdef CONFIG_BACKLIGHT_RK29_BL
static int rk29_backlight_io_init(void)
{
	int ret = 0;

	iomux_set(pwm_mode[BL_PWM]);
#ifdef  LCD_DISP_ON_PIN
	ret = rk_gpio_request(BL_EN_PIN, GPIOF_DIR_OUT, !BL_EN_VALUE, "bl_en");
	if(ret < 0)
		return ret;
#endif
	return 0;
}

static int rk29_backlight_io_deinit(void)
{
	int pwm_gpio;
#ifdef  LCD_DISP_ON_PIN
	gpio_free(BL_EN_PIN);
#endif
	pwm_gpio = iomux_mode_to_gpio(pwm_mode[BL_PWM]);
	return rk_gpio_request(BL_EN_PIN, GPIOF_DIR_OUT, !BL_EN_VALUE, "BL_PWM");
}

static int rk29_backlight_pwm_suspend(void)
{
	int ret, pwm_gpio = iomux_mode_to_gpio(pwm_mode[BL_PWM]);

	ret = rk_gpio_request(pwm_gpio, GPIOF_DIR_OUT, GPIO_LOW, "BL_PWM");
	if(ret < 0)
		return ret;
#ifdef  LCD_DISP_ON_PIN
	gpio_direction_output(BL_EN_PIN, !BL_EN_VALUE);
#endif
	return ret;
}

static int rk29_backlight_pwm_resume(void)
{
	int pwm_gpio = iomux_mode_to_gpio(pwm_mode[BL_PWM]);

	gpio_free(pwm_gpio);
	iomux_set(pwm_mode[BL_PWM]);
#ifdef  LCD_DISP_ON_PIN
	msleep(180);
	gpio_direction_output(BL_EN_PIN, BL_EN_VALUE);
#endif
	return 0;
}

static struct rk29_bl_info rk29_bl_info = {
	.pwm_id = BL_PWM,
	.min_brightness=0,
	.max_brightness=255,
	.brightness_mode = BRIGHTNESS_MODE_CONIC,
	.bl_ref = PWM_EFFECT_VALUE,
	.io_init = rk29_backlight_io_init,
	.io_deinit = rk29_backlight_io_deinit,
	.pwm_suspend = rk29_backlight_pwm_suspend,
	.pwm_resume = rk29_backlight_pwm_resume,
};

static struct platform_device rk29_device_backlight = {
	.name	= "rk29_backlight",
	.id 	= -1,
	.dev	= {
		.platform_data  = &rk29_bl_info,
	}
};
#endif

/***********************************************************
*	fb
************************************************************/
#ifdef CONFIG_FB_ROCKCHIP
static int rk_fb_io_init(struct rk29_fb_setting_info *fb_setting)
{
	int ret = 0;

	ret = rk_gpio_request(LCD_CS_PIN, GPIOF_DIR_OUT, LCD_CS_VALUE, "lcd_cs");
	if(ret < 0)
		return ret;

	return rk_gpio_request(LCD_EN_PIN, GPIOF_DIR_OUT, LCD_EN_VALUE, "lcd_en");
}

static int rk_fb_io_disable(void)
{
	gpio_set_value(LCD_CS_PIN, !LCD_CS_VALUE);
	gpio_set_value(LCD_EN_PIN, !LCD_EN_VALUE);

	return 0;
}

static int rk_fb_io_enable(void)
{
	gpio_set_value(LCD_CS_PIN, LCD_CS_VALUE);
	gpio_set_value(LCD_EN_PIN, LCD_EN_VALUE);

	return 0;
}

#if defined(CONFIG_LCDC0_RK3066B) || defined(CONFIG_LCDC0_RK3188)
struct rk29fb_info lcdc0_screen_info = {
#if defined(CONFIG_RK_HDMI) && defined(CONFIG_HDMI_SOURCE_LCDC0) && defined(CONFIG_DUAL_LCDC_DUAL_DISP_IN_KERNEL)
	.prop	   = EXTEND,	//extend display device
	.io_init    = NULL,
	.io_disable = NULL,
	.io_enable = NULL,
	.set_screen_info = hdmi_init_lcdc,
#else
	.prop	   = PRMRY,		//primary display device
	.io_init   = rk_fb_io_init,
	.io_disable = rk_fb_io_disable,
	.io_enable = rk_fb_io_enable,
	.set_screen_info = set_lcd_info,
#endif
};
#endif

#if defined(CONFIG_LCDC1_RK3066B) || defined(CONFIG_LCDC1_RK3188)
struct rk29fb_info lcdc1_screen_info = {
#if defined(CONFIG_RK_HDMI) && defined(CONFIG_HDMI_SOURCE_LCDC1) && defined(CONFIG_DUAL_LCDC_DUAL_DISP_IN_KERNEL)
	.prop	   = EXTEND,	//extend display device
	.io_init    = NULL,
	.io_disable = NULL,
	.io_enable = NULL,
	.set_screen_info = hdmi_init_lcdc,
#else
	.prop	   = PRMRY,		//primary display device
	.io_init   = rk_fb_io_init,
	.io_disable = rk_fb_io_disable,
	.io_enable = rk_fb_io_enable,
	.set_screen_info = set_lcd_info,
#endif
};
#endif

static struct resource resource_fb[] = {
	[0] = {
		.name  = "fb0 buf",
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = "ipp buf",  //for rotate
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.name  = "fb2 buf",
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device device_fb = {
	.name		= "rk-fb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resource_fb),
	.resource	= resource_fb,
};
#endif

#if defined(CONFIG_LCDC0_RK3066B) || defined(CONFIG_LCDC0_RK3188)
static struct resource resource_lcdc0[] = {
	[0] = {
		.name  = "lcdc0 reg",
		.start = RK3026_LCDC0_PHYS,
		.end   = RK3026_LCDC0_PHYS + RK3026_LCDC0_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	
	[1] = {
		.name  = "lcdc0 irq",
		.start = IRQ_LCDC,
		.end   = IRQ_LCDC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device device_lcdc0 = {
	.name		  = "rk30-lcdc",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(resource_lcdc0),
	.resource	  = resource_lcdc0,
	.dev 		= {
		.platform_data = &lcdc0_screen_info,
	},
};
#endif

#if defined(CONFIG_LCDC1_RK3066B) || defined(CONFIG_LCDC1_RK3188)
static struct resource resource_lcdc1[] = {
	[0] = {
		.name  = "lcdc1 reg",
		.start = RK3026_LCDC1_PHYS,
		.end   = RK3026_LCDC1_PHYS + RK3026_LCDC1_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = "lcdc1 irq",
		.start = IRQ_LCDC1,
		.end   = IRQ_LCDC1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device device_lcdc1 = {
	.name		  = "rk30-lcdc",
	.id		  = 1,
	.num_resources	  = ARRAY_SIZE(resource_lcdc1),
	.resource	  = resource_lcdc1,
	.dev 		= {
		.platform_data = &lcdc1_screen_info,
	},
};
#endif
#ifdef CONFIG_EBC
static struct tps65185_platform_data tps65185_pdata = {
	.wake_up_pin = RK30_PIN3_PC1,
	.vcom_ctl_pin = RK30_PIN2_PC0,
	.pwr_up_pin = INVALID_GPIO,
};
#endif

#define ebc_writel(v,offset) 	do{ writel_relaxed(v, RK30_GRF_BASE + 0x150);dsb();} while (0)
static int ebc_io_init()
{
	int reg;
	iomux_set(EBC_SDCLK);
	iomux_set(EBC_SDLE);
	iomux_set(EBC_SDOE);
	iomux_set(EBC_GDCLK);
	iomux_set(EBC_SDCE2);
	iomux_set(EBC_SDCE3);
	iomux_set(EBC_SDCE4);
	iomux_set(EBC_SDCE5);
	iomux_set(EBC_GDOE);
	iomux_set(EBC_GDSP);
	iomux_set(EBC_GDPWR1);
	iomux_set(EBC_GDPWR2);
	reg = readl_relaxed(RK30_GRF_BASE + 0x0150);
	writel_relaxed(((reg|0x01)|0xffff0000), (RK30_GRF_BASE + 0x150));
}
static  int ebc_power_on(void)
{
	
}
static int ebc_power_off()
{
	
}
static int ebc_vcom_power_on()
{
}
static int ebc_vcom_power_off()
{
}
static int ebc_suspend(struct ebc_platform_data *ebc_data)
{
	int reg;
	struct regulator *ldo_lcdc;
	iomux_set(LCDC0_D16);
	writel_relaxed((0x300|0xffff0000), (RK30_GRF_BASE + 0x150));
	ldo_lcdc = regulator_get(NULL, ebc_data->regulator);
	if(ldo_lcdc){
		if (regulator_is_enabled(ldo_lcdc)){
			regulator_disable(ldo_lcdc);
			regulator_put(ldo_lcdc);
		}
	}

	return 0;
}
static int ebc_resume(struct ebc_platform_data *ebc_data)
{
	int reg;
	struct regulator *ldo_lcdc;
	ldo_lcdc = regulator_get(NULL, ebc_data->regulator);
	if(ldo_lcdc){
		if (!regulator_is_enabled(ldo_lcdc)) {
			regulator_enable(ldo_lcdc);
			regulator_put(ldo_lcdc);
		}
	}
	iomux_set(EBC_GDSP);
	reg = readl_relaxed(RK30_GRF_BASE + 0x0150);
	writel_relaxed(((reg|0x01)|0xffff0000), (RK30_GRF_BASE + 0x150));
	
	return 0;
}
static struct ebc_platform_data  ebc_data={
	.io_init = ebc_io_init,
	.power_on = ebc_power_on,
	.power_off = ebc_power_off,
	.vcom_power_on = ebc_vcom_power_on,
	.vcom_power_off =  ebc_vcom_power_off,
	.suspend = ebc_suspend,
	.resume = ebc_resume,
	.regulator = "vcc_lcd",
	
};

//#define   EBCIORESOURCEPHYS        0x7be00000  

static struct resource ebc_resource[] = {
	[0] = {
		.name  = "ebc reg",
		.start = RK3026_EBC_PHYS,
		.end   = RK3026_EBC_PHYS + RK3026_EBC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = "ebc irq",
		.start = IRQ_EBC,
		.end   = IRQ_EBC,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.name   = "ebc buf",
		.start  = 0,
		.end    = 0,
		.flags  = IORESOURCE_MEM,
	},
};

/*platform_device*/
struct platform_device rk29_device_ebc = {
	.name		  = "rk29-ebc",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(ebc_resource),
	.resource	  = ebc_resource,
	.dev            = {
		.platform_data  = &ebc_data,
	}
};
/* delete it by ykk, replace it in rk30_reserve() */
/*
static int __init get_ebc_waveform_addr(char *options)
{
	int waveform_phy_addr;
	waveform_phy_addr = simple_strtol(options, NULL, 0);
	printk("======== get_ebc_waveform_addr: 0x%x\n", waveform_phy_addr);
	ebc_resource[2].start = waveform_phy_addr;
	ebc_resource[2].end = ebc_resource[2].start + SZ_1M - 1;
	if(memblock_free(waveform_phy_addr , SZ_1M)){
		printk("=====memblock free ebc mem err=====\n");
		ebc_resource[2].start = 0;
		ebc_resource[2].end = 0;
		return 0;
	}
	if(memblock_remove(waveform_phy_addr , SZ_1M)){
		ebc_resource[2].start = 0;
		ebc_resource[2].end = 0;
		printk("memblock remove ebc mem err\n");
	}
	return 0;
}
__setup("waveform_addr=", get_ebc_waveform_addr);
*/


static int rk_platform_add_display_devices(void)
{
	struct platform_device *fb = NULL;  //fb
	struct platform_device *lcdc0 = NULL; //lcdc0
	struct platform_device *lcdc1 = NULL; //lcdc1
	struct platform_device *bl = NULL; //backlight
#ifdef CONFIG_FB_ROCKCHIP
	fb = &device_fb;
#endif

#if defined(CONFIG_LCDC0_RK3066B) || defined(CONFIG_LCDC0_RK3188)
	lcdc0 = &device_lcdc0,
#endif

#if defined(CONFIG_LCDC1_RK3066B) || defined(CONFIG_LCDC1_RK3188)
	lcdc1 = &device_lcdc1,
#endif

#ifdef CONFIG_BACKLIGHT_RK29_BL
	bl = &rk29_device_backlight,
#endif
	__rk_platform_add_display_devices(fb,lcdc0,lcdc1,bl);

	return 0;
}


/***********************************************************
*	gsensor
************************************************************/
#if defined (CONFIG_GS_MMA8452)
static int mma8452_init_platform_hw(void)
{
	return 0;
}

static struct sensor_platform_data mma8452_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 1,
	.poll_delay_ms = 30,
        .init_platform_hw = mma8452_init_platform_hw,
        .orientation = {-1, 0, 0, 0, -1, 0, 0, 0, 1},
};
#endif

/***********************************************************
*	keyboard
************************************************************/
#include <plat/key.h>

/******************************************************************************
 * Keyboard
 ******************************************************************************/
#if defined(CONFIG_KEYBOARD_MATRIX) || defined(CONFIG_KEYBOARD_MATRIX_MODULE)
#include <linux/input/matrix_keypad.h>
static const uint32_t matrix_matrix_keys[] = {
#if defined(CONFIG_MACH_RK3026_E602)
	KEY(0, 0, KEY_PAGEUP),
	KEY(0, 1, KEY_PAGEDOWN),
#else
	KEY(0, 0, KEY_PAGEDOWN),
	KEY(0, 1, KEY_PAGEUP),
#endif
};

const struct matrix_keymap_data matrix_keymap_data = {
	.keymap			= matrix_matrix_keys,
	.keymap_size		= ARRAY_SIZE(matrix_matrix_keys),
};

static const unsigned int matrix_keypad_row_gpios[] = {
#if defined(CONFIG_SPI)
	RK30_PIN1_PA1
#else
	RK30_PIN1_PB2//, RK30_PIN3_PB3
#endif
};

static const unsigned int matrix_keypad_col_gpios[] = {
	RK30_PIN0_PA2, RK30_PIN0_PA3
};

struct matrix_keypad_platform_data matrix_keypad_platform_data = {
	.keymap_data	= &matrix_keymap_data,
	.row_gpios	= matrix_keypad_row_gpios,
	.num_row_gpios	= ARRAY_SIZE(matrix_keypad_row_gpios),
	.col_gpios	= matrix_keypad_col_gpios,
	.num_col_gpios	= ARRAY_SIZE(matrix_keypad_col_gpios),
	.active_low	= 1,
	.wakeup = 1,
	.debounce_ms		= 20,
	.col_scan_delay_us	= 5,
};

static struct platform_device matrix_keyboard = {
	.name	= "matrix-keypad",
	.id	= -1,
	.dev	= {
		.platform_data = &matrix_keypad_platform_data,
	},
};

#endif

static struct rk29_keys_button key_button[] = {
#if 0
        {
                .desc   = "flush",
                .code   = KEY_F5,
                .gpio   = RK30_PIN3_PB3,
                .active_low = PRESS_LEV_LOW,
                .wakeup = 1,
        },
#endif
/* disable adc keyboard,
 * because rk280a adc reference voltage is 3.3V, but
 * rk30xx mainbord key's supply voltage is 2.5V and
 * rk31xx mainbord key's supply voltage is 1.8V.
 */
#if defined(CONFIG_MACH_RK3026_T63)
	{
		.desc		= "pageup",
		.code		= KEY_PAGEUP,
		.adc_value	= 1,
		.gpio		= INVALID_GPIO,
		.active_low	= PRESS_LEV_LOW,
	},
	{
		.desc		= "pagedown",
		.code		= KEY_PAGEDOWN,
		.gpio		= RK30_PIN1_PA1,
		.active_low	= PRESS_LEV_LOW,
	},
#else
	{
		.desc   = "flush",
		.code   = KEY_F5,
		.adc_value      = 1,
		.gpio   = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
	{
		.desc   = "esc",
		.code   = KEY_BACK,
		.adc_value      = 150,
		.gpio   = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
	{
		.desc   = "pageup",
		.code   = KEY_PAGEUP,
#if defined(CONFIG_MACH_RK3026_E602)
  #if defined(CONFIG_SWITCH_PAGEUP_PAGEDOWN_E602)
		.gpio = RK30_PIN0_PA3,
  #else
		.gpio = RK30_PIN0_PA2,
  #endif
#else
  #if defined(CONFIG_SWITCH_PAGEUP_PAGEDOWN_E601)
		.gpio = RK30_PIN0_PA2,
  #else
		.gpio = RK30_PIN0_PA3,
  #endif
#endif
		.active_low = PRESS_LEV_LOW,
	},
	{
		.desc   = "pagedown",
		.code   = KEY_PAGEDOWN,
#if defined(CONFIG_MACH_RK3026_E602)
  #if defined(CONFIG_SWITCH_PAGEUP_PAGEDOWN_E602)
		.gpio = RK30_PIN0_PA2,
  #else
		.gpio = RK30_PIN0_PA3,
  #endif
#else
  #if defined(CONFIG_SWITCH_PAGEUP_PAGEDOWN_E601)
		.gpio = RK30_PIN0_PA3,
  #else
		.gpio = RK30_PIN0_PA2,
  #endif
#endif
		.active_low = PRESS_LEV_LOW,
	},
#endif
};

struct rk29_keys_platform_data rk29_keys_pdata = {
	.buttons	= key_button,
	.nbuttons	= ARRAY_SIZE(key_button),
	.chn	= 3,  //chn: 0-7, if do not use ADC,set 'chn' -1
	.adc_irq_io = RK30_PIN3_PB3,
};

/***********************************************************
*	sdmmc
************************************************************/
#ifdef CONFIG_SDMMC_RK29
static void rtlusb_wifi_power(int on)
{
#if defined(CONFIG_KP_AXP22)
	struct regulator *ldo_wifi = NULL;

	ldo_wifi = regulator_get(NULL, "vwl_3v3");
	if (!ldo_wifi) {
		printk("rkusb wifi power not available.\n");
		return;
	}

	if(on) {
		if (!regulator_is_enabled(ldo_wifi)) {
			regulator_enable(ldo_wifi);
			printk("%s: rkusb wifi power enable\n", __func__);
		}
	} else {
		printk("%s: rkusb wifi power disable\n", __func__);
		while(regulator_is_enabled(ldo_wifi)>0)
			regulator_disable(ldo_wifi);
	}
	regulator_put(ldo_wifi);
	udelay(100);
#endif
}
#include "board-rk3026-tb-sdmmc-config.c"
#include "../plat-rk/rk-sdmmc-ops.c"
#include "../plat-rk/rk-sdmmc-wifi.c"
#endif //endif ---#ifdef CONFIG_SDMMC_RK29

#ifdef CONFIG_SDMMC0_RK29
#define CONFIG_SDMMC0_USE_DMA
static int rk29_sdmmc0_cfg_gpio(void)
{
	rk29_sdmmc_set_iomux(0, 0xFFFF);
	#if defined(CONFIG_SDMMC0_RK29_SDCARD_DET_FROM_GPIO)
        iomux_set_gpio_mode(iomux_gpio_to_mode(RK29SDK_SD_CARD_DETECT_N));
    	#else
        iomux_set(MMC0_DETN);
    	#endif	

	#if defined(CONFIG_SDMMC0_RK29_WRITE_PROTECT)
	gpio_request(SDMMC0_WRITE_PROTECT_PIN, "sdmmc-wp");
	gpio_direction_input(SDMMC0_WRITE_PROTECT_PIN);
	#endif
	return 0;
}

struct rk29_sdmmc_platform_data default_sdmmc0_data = {
	.host_ocr_avail =
	    (MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 | MMC_VDD_28_29 |
	     MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 |
	     MMC_VDD_33_34 | MMC_VDD_34_35 | MMC_VDD_35_36),
	.host_caps =
	    (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
	.io_init = rk29_sdmmc0_cfg_gpio,

	.set_iomux = rk29_sdmmc_set_iomux,

	.dma_name = "vmmc",
#ifdef CONFIG_SDMMC0_USE_DMA
	.use_dma = 1,
#else
	.use_dma = 0,
#endif

#if defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC) && defined(CONFIG_USE_SDMMC0_FOR_WIFI_DEVELOP_BOARD)
	.status = rk29sdk_wifi_mmc0_status,
	.register_status_notify = rk29sdk_wifi_mmc0_status_register,
#endif

#if defined(RK29SDK_SD_CARD_PWR_EN) && (INVALID_GPIO != RK29SDK_SD_CARD_PWR_EN)
	.power_en = RK29SDK_SD_CARD_PWR_EN,
	.power_en_level = RK29SDK_SD_CARD_PWR_EN_LEVEL,
#else
	.power_en = INVALID_GPIO,
	.power_en_level = GPIO_LOW,
	.regulator = "vmmc",
#endif    
	.enable_sd_wakeup = 0,

#if defined(CONFIG_SDMMC0_RK29_WRITE_PROTECT)
	.write_prt = SDMMC0_WRITE_PROTECT_PIN,
	.write_prt_enalbe_level = SDMMC0_WRITE_PROTECT_ENABLE_VALUE;
#else
	.write_prt = INVALID_GPIO,
#endif

	.det_pin_info = {    
    		#if defined(RK29SDK_SD_CARD_DETECT_N) && (INVALID_GPIO != RK29SDK_SD_CARD_DETECT_N)  
        	.io             = RK29SDK_SD_CARD_DETECT_N, //INVALID_GPIO,
        	.enable         = RK29SDK_SD_CARD_INSERT_LEVEL,
    		#else
        	.io             = INVALID_GPIO,
        	.enable         = GPIO_LOW,
    		#endif    
    	}, 

};
#endif // CONFIG_SDMMC0_RK29

#ifdef CONFIG_SDMMC1_RK29
#define CONFIG_SDMMC1_USE_DMA
static int rk29_sdmmc1_cfg_gpio(void)
{
#if defined(CONFIG_SDMMC1_RK29_WRITE_PROTECT)
	gpio_request(SDMMC1_WRITE_PROTECT_PIN, "sdio-wp");
	gpio_direction_input(SDMMC1_WRITE_PROTECT_PIN);
#endif
	return 0;
}

struct rk29_sdmmc_platform_data default_sdmmc1_data = {
	.host_ocr_avail =
	    (MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 | MMC_VDD_28_29 |
	     MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 |
	     MMC_VDD_33_34),

#if !defined(CONFIG_USE_SDMMC1_FOR_WIFI_DEVELOP_BOARD)
	.host_caps = (MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ |
		      MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
#else
	.host_caps =
	    (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
#endif

	.io_init = rk29_sdmmc1_cfg_gpio,

	.set_iomux = rk29_sdmmc_set_iomux,

	.dma_name = "sdio",
#ifdef CONFIG_SDMMC1_USE_DMA
	.use_dma = 1,
#else
	.use_dma = 0,
#endif

#if defined(CONFIG_WIFI_CONTROL_FUNC) || defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC)
	.status = rk29sdk_wifi_status,
	.register_status_notify = rk29sdk_wifi_status_register,
#endif

#if defined(CONFIG_SDMMC1_RK29_WRITE_PROTECT)
	.write_prt = SDMMC1_WRITE_PROTECT_PIN,
	.write_prt_enalbe_level = SDMMC1_WRITE_PROTECT_ENABLE_VALUE;
#else
	.write_prt = INVALID_GPIO,
#endif

    #if defined(CONFIG_RK29_SDIO_IRQ_FROM_GPIO)
	.sdio_INT_gpio = RK29SDK_WIFI_SDIO_CARD_INT,
    #endif

	.det_pin_info = {    
	#if defined(CONFIG_USE_SDMMC1_FOR_WIFI_DEVELOP_BOARD)
		#if defined(RK29SDK_SD_CARD_DETECT_N) && (INVALID_GPIO != RK29SDK_SD_CARD_DETECT_N)  
        	.io             = RK29SDK_SD_CARD_DETECT_N,
     		#else
         	.io             = INVALID_GPIO,
		#endif   

        	.enable         = RK29SDK_SD_CARD_INSERT_LEVEL,
	#else
        	.io             = INVALID_GPIO,
        	.enable         = GPIO_LOW,
	#endif
	},
	.enable_sd_wakeup = 0,
};
#endif //endif--#ifdef CONFIG_SDMMC1_RK29
#if defined(CONFIG_REGULATOR_ACT8931)
int rk30_battery_adc_is_dc_charging( ){
    return  act8931_charge_det;  
}
int rk30_battery_adc_charging_ok( ){
    return act8931_charge_ok;
}
#endif	

#ifdef CONFIG_BATTERY_RK30_ADC_FAC
static struct rk30_adc_battery_platform_data rk30_adc_battery_platdata = {
	.dc_det_pin      = INVALID_GPIO,//RK30_PIN1_PB2,
	.batt_low_pin    = INVALID_GPIO,
	.charge_set_pin  = INVALID_GPIO,//RK30_PIN1_PA1,
	.charge_ok_pin   = RK30_PIN0_PA2,
	.usb_det_pin = INVALID_GPIO,
	
	.dc_det_level    = GPIO_LOW,
	.charge_ok_level = GPIO_HIGH,
	.charge_set_level = GPIO_HIGH,
	.batt_low_level = GPIO_LOW,
	
	.reference_voltage = 3200, //  rk3026 ref value depend on vccio 
	.pull_up_res = 200,     //divider resistance ,  pull-up resistor
	.pull_down_res = 200, //divider resistance , pull-down resistor
	//.io_init = rk30_adc_battery_io_init,
#if defined(CONFIG_REGULATOR_ACT8931)
	.is_dc_charging	= rk30_battery_adc_is_dc_charging,
	.charging_ok	  = rk30_battery_adc_charging_ok ,
#endif	  
	.charging_sleep = 0,

	.is_reboot_charging = 1,
	.save_capacity   = 1 ,
	.low_voltage_protection = 3500,//3600, 
};

static struct platform_device rk30_device_adc_battery = {
        .name   = "rk30-battery",
        .id     = -1,
        .dev = {
                .platform_data = &rk30_adc_battery_platdata,
        },
};
#endif

/***********************************************************
*	ion
************************************************************/
#ifdef CONFIG_ION
static struct ion_platform_data rk30_ion_pdata = {
	.nr = 1,
	.heaps = {
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = ION_NOR_HEAP_ID,
			.name = "norheap",
			.size = ION_RESERVE_SIZE,
		}
	},
};

static struct platform_device device_ion = {
	.name = "ion-rockchip",
	.id = 0,
	.dev = {
		.platform_data = &rk30_ion_pdata,
	},
};
#endif

/***********************************************************
*	pwm regulator
************************************************************/
#ifdef CONFIG_RK30_PWM_REGULATOR
static int pwm_voltage_map[] = {
	800000,  825000,  850000,  875000,  900000,  925000 ,
	950000,  975000,  1000000, 1025000, 1050000, 1075000, 
	1100000, 1125000, 1150000, 1175000, 1200000, 1225000, 
	1250000, 1275000, 1300000, 1325000, 1350000, 1375000
};

static struct regulator_consumer_supply pwm_dcdc1_consumers[] = {
	{
		.supply = "vdd_core",
	}
};

struct regulator_init_data pwm_regulator_init_dcdc[1] = {
	{
		.constraints = {
			.name = "PWM_DCDC1",
			.min_uV = 600000,
			.max_uV = 1800000,      //0.6-1.8V
			.apply_uV = true,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies = ARRAY_SIZE(pwm_dcdc1_consumers),
		.consumer_supplies = pwm_dcdc1_consumers,
	},
};

static struct pwm_platform_data pwm_regulator_info[1] = {
	{
		.pwm_id = REG_PWM,
		.pwm_voltage = 1200000,
		.suspend_voltage = 1050000,
		.min_uV = 950000,
		.max_uV = 1400000,
		.coefficient = 504,     //50.4%
		.pwm_voltage_map = pwm_voltage_map,
		.init_data      = &pwm_regulator_init_dcdc[0],
	},
};
struct platform_device pwm_regulator_device[1] = {
	{
		.name = "pwm-voltage-regulator",
		.id = 0,
		.dev            = {
			.platform_data = &pwm_regulator_info[0],
		}
	},
};

static void pwm_regulator_init(void)
{
	pwm_regulator_info[0].pwm_gpio = iomux_mode_to_gpio(pwm_mode[REG_PWM]);
	pwm_regulator_info[0].pwm_iomux_pwm = pwm_mode[REG_PWM];
	pwm_regulator_info[0].pwm_iomux_gpio = iomux_switch_gpio_mode(pwm_mode[REG_PWM]);
}
#endif

#define GPIO_SWPORTA_DR   0x0000
#define GPIO_SWPORTA_DDR  0x0004
#define PWM_MUX_REG       (RK2928_GRF_BASE + GRF_GPIO0D_IOMUX)
#define PWM_DDR_REG       (RK2928_GPIO0_BASE + GPIO_SWPORTA_DDR)
#define PWM_DR_REG        (RK2928_GPIO0_BASE + GPIO_SWPORTA_DR)

#define mux_set_gpio_mode(id)  do { writel_relaxed( 1 << (20 + (id) * 2), PWM_MUX_REG); dsb(); } while (0)
#define mux_set_pwm_mode(id)  do { writel_relaxed( (1 << (20 + (id) * 2)) | (1 << (4 + (id) * 2)), PWM_MUX_REG); dsb(); } while (0)

#define pwm_output_high(id) do {\
                                writel_relaxed(readl_relaxed(PWM_DDR_REG) | (1 << (26 + (id))), PWM_DDR_REG); \
                                writel_relaxed(readl_relaxed(PWM_DR_REG) | (1 << (26 + (id))), PWM_DR_REG); \
                                dsb(); \
                                } while (0)

void __sramfunc rk30_pwm_logic_suspend_voltage(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR
	mux_set_gpio_mode(0);
	pwm_output_high(0);
#endif 
}
void __sramfunc rk30_pwm_logic_resume_voltage(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR
	mux_set_pwm_mode(0);
	sram_udelay(10000);
#endif
}

extern void pwm_suspend_voltage(void);
extern void pwm_resume_voltage(void);

void  rk30_pwm_suspend_voltage_set(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR
	pwm_suspend_voltage();
#endif
}

void  rk30_pwm_resume_voltage_set(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR
	pwm_resume_voltage();
#endif
}

/***********************************************************
*	pmic
************************************************************/
int __sramdata g_pmic_type =  0;

#ifdef CONFIG_MFD_TPS65910
#define TPS65910_HOST_IRQ 	PMU_INT_PIN
#define PMU_POWER_SLEEP		PMU_SLEEP_PIN
static struct pmu_info  tps65910_dcdc_info[] = {
	{
		.name = "vdd_cpu",
		.min_uv = 1200000,
		.max_uv = 1200000,
	},
	{
		.name = "vdd2",
		.min_uv = 1200000,
		.max_uv = 1200000,
	},
	{
		.name = "vio",
		.min_uv = 3300000,
		.max_uv = 3300000,
	},
};

static struct pmu_info tps65910_ldo_info[] = {
	{
		.name          = "vpll",   //vcc25
		.min_uv          = 2500000,
		.max_uv         = 2500000,
	},
	{
		.name          = "vdig1",    //vcc18_cif
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},

	{
		.name          = "vdig2",   //vdd11
		.min_uv          = 1100000,
		.max_uv         = 1100000,
	},
	{
		.name          = "vaux1",   //vcc28_cif
		.min_uv          = 2800000,
		.max_uv         = 2800000,
	},
	{
		.name          = "vaux2",   //vcca33
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "vaux33",   //vcc_tp
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "vmmc",   //vccio_wl
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
	{
		.name          = "vdac",   //
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
};
#include "../mach-rk30/board-pmu-tps65910.c"
#endif

void __sramfunc board_pmu_suspend(void)
{
	#if defined (CONFIG_MFD_TPS65910)
	if(pmic_is_tps65910())
		board_pmu_tps65910_suspend();
	#endif
}

void __sramfunc board_pmu_resume(void)
{
	#if defined (CONFIG_MFD_TPS65910)
	if(pmic_is_tps65910())
		board_pmu_tps65910_resume();
	#endif
}


#ifdef CONFIG_REGULATOR_ACT8931
//#define ACT8931_HOST_IRQ		PMU_INT_PIN//depend on your hardware


#define ACT8931_CHGSEL_PIN 	INVALID_GPIO //depend on your hardware


static struct pmu_info  act8931_dcdc_info[] = {
	{
		.name          = "act_dcdc1",   //vccio,the value by modified for adc battery 
		.min_uv          = 3200000,
		.max_uv         = 3200000,
		.enable = 1,
	},
	{
		.name          = "act_dcdc2",    //ddr
		.min_uv          = 1350000,
		.max_uv         = 1350000,
		.enable = 1,
	},
	{
		.name          = "vdd_cpu",   //vdd_arm
		.min_uv          = 1200000,
		.max_uv         = 1200000,
		.enable = 1,
	},
	
};
static  struct pmu_info  act8931_ldo_info[] = {
	{
		.name          = "act_ldo1",    //vcc_tp
		.min_uv          = 3300000,
		.max_uv         = 3300000,
		.enable = 1,
	},
	{
		.name          = "act_ldo2",    //vout5
		.min_uv          = 3000000,
		.max_uv         = 3000000,
		.enable = 0,
	},
	{
		.name          = "act_ldo3",    //vcc_sd
		.min_uv          = 3000000,
		.max_uv         = 3000000,
		.enable = 1,
	},
	{
		.name          = "act_ldo4",    //vcc_lcd
		.min_uv          = 1800000,
		.max_uv         = 1800000,
		.enable = 1,
	},
};
#include "board-rk30-sdk-act8931.c"
#endif

#ifdef CONFIG_SND_SOC_RK3026
static int rk3026_power_on(int enable)
{
#if defined(CONFIG_REGULATOR_ACT8931)
	struct regulator *ldo_sd=NULL;
	int ret = 0;
	
	ldo_sd = regulator_get(NULL, "act_ldo3");
	if (!ldo_sd) {
		pr_err("fail to get regulator act_ldo3\n");
		ret = -1;
	}
	if (enable) {
		regulator_enable(ldo_sd);
		regulator_put(ldo_sd);
		msleep(20);
	} else {
		regulator_disable(ldo_sd);
		regulator_put(ldo_sd);
	}
	return ret;
#endif
}

struct rk3026_codec_pdata rk3026_codec_pdata_info={
	.power_set = rk3026_power_on,
	.spk_ctl_gpio = INVALID_GPIO,
	.hp_ctl_gpio = INVALID_GPIO,//RK2928_PIN1_PA0,
	.delay_time = 10,
};

static struct resource resources_acodec[] = {
	{
		.start 	= RK2928_ACODEC_PHYS,
		.end 	= RK2928_ACODEC_PHYS + RK2928_ACODEC_SIZE - 1,
		.flags 	= IORESOURCE_MEM,
	},
};

static struct platform_device rk3026_codec = {
	.name	= "rk3026-codec",
	.id		= -1,
	.resource = resources_acodec,
    	.dev = {
        	.platform_data = &rk3026_codec_pdata_info,
    }
};
#endif
#if defined (CONFIG_TOUCHSCREEN_GT819) || defined(CONFIG_TOUCHSCREEN_GT813)
struct goodix_platform_data goodix_info;
static int gt819_init_platform_hw(void)
{
	printk("gt819_init_platform_hw\n");
    if(gpio_request(TOUCH_RST_PIN,NULL) != 0){
      gpio_free(TOUCH_RST_PIN);
      printk("gt819_init_platform_hw gpio_request error\n");
      return -EIO;
    }

    if(gpio_request(TOUCH_INT_PIN,NULL) != 0){
      gpio_free(TOUCH_INT_PIN);
      printk("gt819_init_platform_hw gpio_request error\n");
      return -EIO;
    }
	gpio_direction_output(TOUCH_RST_PIN, 0);
	gpio_set_value(TOUCH_RST_PIN,GPIO_LOW);
	mdelay(10);
//	gpio_set_value(TOUCH_RST_PIN,GPIO_HIGH);
//	mdelay(10);
//	gpio_set_value(TOUCH_RST_PIN,GPIO_LOW);
	gpio_direction_input(TOUCH_INT_PIN);
//	mdelay(10);
	gpio_set_value(TOUCH_RST_PIN,GPIO_HIGH);
	msleep(300);
    return 0;
}
static void gt819_exit_platform_hw(void)
{
	printk("gt819_exit_platform_hw\n");
	gpio_free(TOUCH_RST_PIN);
	gpio_free(TOUCH_INT_PIN);
}
static int gt819_platform_sleep(void)
{
	struct regulator *ldo_tp = NULL;
	
	printk("gt819_platform_sleep\n");
	gpio_set_value(TOUCH_RST_PIN,GPIO_LOW);
	ldo_tp = regulator_get(NULL, goodix_info.regulator);
	if (!ldo_tp)
	{
		printk("gt819_platform_sleep fail\n");
		return -1;
	}
	
	if (regulator_is_enabled(ldo_tp))
	{
		regulator_disable(ldo_tp);
		regulator_put(ldo_tp);
	}
	return 0;
}
static int gt819_platform_wakeup(void)
{
	struct regulator *ldo_tp;
	
	printk("gt819_platform_wakeup\n");
	ldo_tp = regulator_get(NULL, goodix_info.regulator);
	if (!ldo_tp)
	{
		printk("gt819_platform_wakeup fail\n");
		return -1;
	}

	if (!regulator_is_enabled(ldo_tp))
	{
		regulator_enable(ldo_tp);
		regulator_put(ldo_tp);
	}
	gpio_set_value(TOUCH_RST_PIN,GPIO_HIGH);
	return 0;
}
struct goodix_platform_data goodix_info = {
	.regulator = "act_ldo1",
	.init_platform_hw= gt819_init_platform_hw,
	.exit_platform_hw= gt819_exit_platform_hw,
	.platform_sleep  = gt819_platform_sleep,
	.platform_wakeup = gt819_platform_wakeup,
};
#endif

#if defined (CONFIG_KP_AXP22)
#include "board-rk3026-ebook-power.c"
#endif

#if defined(CONFIG_TOUCHSCREEN_TrueTouch)

struct cyttsp_board_data{         // don't change order, corresponding with tp driver
    u32 tp_int_port;
    u32 tp_rst_port;
	const char *regulator;
	s32 (*power_up)(struct cyttsp_board_data * board_data);
	s32 (*power_down)(struct cyttsp_board_data * board_data);
};

s32 cyttsp_power_up_device(struct cyttsp_board_data * board_data)
{
	struct regulator *ldo_tp;
	
	ldo_tp = regulator_get(NULL, board_data->regulator);
	if (!ldo_tp){
		printk("%s fail\n",__func__);
		return 0;
	}

	if (!regulator_is_enabled(ldo_tp)){
		printk("%s, enable power for cyttsp-truetouch\n", __func__);
		regulator_enable(ldo_tp);
		regulator_put(ldo_tp);
	}
	
	gpio_direction_output(board_data->tp_rst_port, 1);

	return 0;
}

s32 cyttsp_power_down_device(struct cyttsp_board_data * board_data)
{	
	struct regulator *ldo_tp = NULL;

	gpio_direction_output(board_data->tp_rst_port, 0);
	
	ldo_tp = regulator_get(NULL, board_data->regulator);
	if (!ldo_tp){
		printk("%s fail\n",__func__);
		return -1;
	}
	
	while(regulator_is_enabled(ldo_tp)>0) {
		printk("%s, disable power for cyttsp-truetouch\n", __func__);
		regulator_disable(ldo_tp);
	}
	regulator_put(ldo_tp);

	return 0;
}

static struct cyttsp_board_data cyttsp_data = {
    .tp_int_port 	= TOUCH_INT_PIN,
    .tp_rst_port 	= TOUCH_RST_PIN,
    .regulator      = "vcc_tp",
    .power_up		= cyttsp_power_up_device,
	.power_down 	= cyttsp_power_down_device,
};


#endif




/***********************************************************
*	i2c
************************************************************/
#ifdef CONFIG_I2C0_RK30
static struct i2c_board_info __initdata i2c0_info[] = {
#if defined (CONFIG_GS_MMA8452)
	{
		.type	        = "gs_mma8452",
		.addr	        = 0x1d,
		.flags	        = 0,
		.irq	        = GS_INT_PIN,
		.platform_data = &mma8452_info,
	},
#endif
#if defined (CONFIG_REGULATOR_ACT8931)
	{
		.type    		= "act8931",
		.addr           = 0x5b, 
		.flags			= 0,
		//.irq            = ACT8931_HOST_IRQ,
		.platform_data=&act8931_data,
	},
#endif
#if defined (CONFIG_RTC_HYM8563)
	{
		.type                   = "rtc_hym8563",
		.addr           = 0x51,
		.flags                  = 0,
		.irq            = RTC_INT_PIN,
	},
#endif
#if defined (CONFIG_KP_AXP22)
	{
		.type = "axp22_mfd",
		.flags	= 0,
		.addr = AXP_DEVICES_ADDR,
		.platform_data = &axp_pdata,
		.irq = PMU_INT_PIN,
	},
#endif

};
#endif

#if defined (CONFIG_TOUCHSCREEN_XC3) || defined(CONFIG_TOUCHSCREEN_OED)
#define CY_USE_MT
#define CY_RST_GPIO		TOUCH_RST_PIN
#define CY_I2C_IRQ_GPIO		TOUCH_INT_PIN
static int cyttsp_i2c_init(int on)
{
	int ret;

	/* add any special code to initialize any required system hw
	 * such as regulators or gpio pins
	 */
	if (on) {
		ret = tp_power_on(1);
		if (ret)
			return ret;

		//init IRQ
		ret = gpio_request(CY_I2C_IRQ_GPIO, "CYTTSP IRQ GPIO");
		if (ret) {
			printk(KERN_ERR "%s: Failed to request GPIO %d\n",
			       __func__, CY_I2C_IRQ_GPIO);
			return ret;
		}
		
		gpio_pull_updown(CY_I2C_IRQ_GPIO, PullDisable);
		gpio_direction_output(CY_I2C_IRQ_GPIO, GPIO_LOW);
		msleep(5);
		gpio_set_value(CY_I2C_IRQ_GPIO, 1);
		msleep(5);
		gpio_set_value(CY_I2C_IRQ_GPIO, 0);
		msleep(5);
		gpio_set_value(CY_I2C_IRQ_GPIO, 1);
		msleep(5);
		gpio_direction_input(CY_I2C_IRQ_GPIO);

		//init RST
		ret = gpio_request(CY_RST_GPIO, "CYTTSP RST GPIO");
		if (ret) {
			printk(KERN_ERR "%s: Failed to request GPIO %d\n",
			       __func__, CY_I2C_IRQ_GPIO);
			return ret;
		}
		
		gpio_pull_updown(CY_RST_GPIO, GPIONormal);
		gpio_direction_output(CY_RST_GPIO, GPIO_HIGH);
		msleep(5);
		gpio_set_value(CY_RST_GPIO, GPIO_LOW);
		msleep(5);
		gpio_set_value(CY_RST_GPIO, GPIO_HIGH);
		msleep(5);
	} else {
		gpio_free(CY_I2C_IRQ_GPIO);
		gpio_free(CY_RST_GPIO);
		ret = tp_power_on(0);
		if (ret)
			return ret;
	}
	return 0;
}

static int cyttsp_i2c_wakeup(void)
{
	gpio_pull_updown(CY_I2C_IRQ_GPIO, PullDisable);
	gpio_direction_output(CY_I2C_IRQ_GPIO, GPIO_LOW);
	msleep(5);
	gpio_set_value(CY_I2C_IRQ_GPIO, 1);
	msleep(5);
	gpio_set_value(CY_I2C_IRQ_GPIO, 0);
	msleep(5);
	gpio_set_value(CY_I2C_IRQ_GPIO, 1);
	gpio_direction_input(CY_I2C_IRQ_GPIO);
	return 0;
}

static struct cyttsp_platform_data cypress_i2c_ttsp_platform_data = {
	.wakeup = cyttsp_i2c_wakeup,
	.init = cyttsp_i2c_init,
#ifdef CY_USE_MT
	.mt_sync = input_mt_sync,
#endif
	.maxx = 1024,
	.maxy = 758,
	.oed_maxx = 2047,
	.oed_maxy = 2047,
	.flags = 0x05,
	.gen = CY_GEN3,
	.use_st = 0,
	.use_mt = 1,
	.use_trk_id = 1,
	.use_hndshk = 1,
	.use_timer = 0,
	.use_sleep = 1,
	.use_gestures = 0,
	.use_load_file = 0,
	.use_force_fw_update = 1,
	.use_virtual_keys = 0,
	/* activate up to 4 groups
	 * and set active distance
	 */
	.gest_set = CY_GEST_GRP_NONE | CY_ACT_DIST,
	/* change act_intrvl to customize the Active power state
	 * scanning/processing refresh interval for Operating mode
	 */
	.scn_typ = 0xA5,
	.act_intrvl = CY_ACT_INTRVL_DFLT,
	/* change tch_tmout to customize the touch timeout for the
	 * Active power state for Operating mode
	 */
	.tch_tmout = CY_TCH_TMOUT_DFLT,
	/* change lp_intrvl to customize the Low Power power state
	 * scanning/processing refresh interval for Operating mode
	 */
	.lp_intrvl = CY_LP_INTRVL_DFLT,
	.name = CY_I2C_NAME,
	.irq_gpio = CY_I2C_IRQ_GPIO,
};
#endif

#if defined (CONFIG_TOUCHSCREEN_CYPRESS)
#define CY_RST_PIN 		TOUCH_RST_PIN
#define CY_INT_PIN		TOUCH_INT_PIN
int gpio_cyttsp_init_pins(void)
{
	int ret = 0;
	//touch pins
	ret = gpio_request(CY_INT_PIN, "tp_int");
	if(unlikely(ret)) return ret;

	ret = gpio_request(CY_RST_PIN, "tp_rst");
	if(unlikely(ret)) goto free_int;

	gpio_direction_input(CY_INT_PIN);
	gpio_direction_output(CY_RST_PIN, 1);

	return ret;
free_int:
	gpio_free(CY_INT_PIN);
	return ret;

}
EXPORT_SYMBOL(gpio_cyttsp_init_pins);

/* Touch Controller IRQ */
void gpio_touchcntrl_request_irq(int enable)
{
	if (enable)
		gpio_direction_input(CY_INT_PIN);
}
EXPORT_SYMBOL(gpio_touchcntrl_request_irq);

int gpio_touchcntrl_irq(void)
{
        return gpio_to_irq(CY_INT_PIN);
}
EXPORT_SYMBOL(gpio_touchcntrl_irq);

int gpio_touchcntrl_irq_get_value(void)
{
	return gpio_get_value(CY_INT_PIN);
}
EXPORT_SYMBOL(gpio_touchcntrl_irq_get_value);

void gpio_cyttsp_wake_signal(void)
{
	mdelay(100);
	gpio_direction_output(CY_INT_PIN, 1);
	mdelay(1);
	gpio_set_value(CY_INT_PIN, 0);
	mdelay(1);
	gpio_set_value(CY_INT_PIN, 1);
	mdelay(1);
	gpio_set_value(CY_INT_PIN, 0);
	mdelay(1);
	gpio_set_value(CY_INT_PIN, 1);
	mdelay(100);
	gpio_direction_input(CY_INT_PIN);
}
EXPORT_SYMBOL(gpio_cyttsp_wake_signal);

/* power up cypress touch */
int gpio_cyttsp_hw_reset(void)
{
	gpio_direction_output(CY_RST_PIN, 1);
	gpio_set_value(CY_RST_PIN, 1);
	msleep(20);
	gpio_set_value(CY_RST_PIN, 0);
	msleep(40);
	gpio_set_value(CY_RST_PIN, 1);
	msleep(20);
	gpio_direction_input(CY_RST_PIN);
	printk(KERN_DEBUG "cypress reset");

	return 0;
}
EXPORT_SYMBOL(gpio_cyttsp_hw_reset);
#endif

#if defined (CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4)
/* cyttsp */
#include <linux/cyttsp4_core.h>
#include <linux/cyttsp4_platform.h>

#define CYTTSP4_USE_I2C
/* #define CYTTSP4_USE_SPI */

#ifdef CYTTSP4_USE_I2C
#define CYTTSP4_I2C_TCH_ADR 0x24
#define CYTTSP4_LDR_TCH_ADR 0x24
#if defined(CONFIG_MACH_RK3026_E601) || defined(CONFIG_MACH_RK3026_T63)
  #define CYTTSP4_I2C_IRQ_GPIO RK30_PIN1_PA4      // E601C
#else
  #define CYTTSP4_I2C_IRQ_GPIO RK30_PIN1_PA2
#endif
#define CYTTSP4_I2C_RST_GPIO RK30_PIN2_PB4
#endif

#ifdef CYTTSP4_USE_SPI
/* Change GPIO numbers when using I2C and SPI at the same time
 * Following is possible alternative:
 * IRQ: J6.17, C18, GPMC_AD12/GPIO_36
 * RST: J6.24, D17, GPMC_AD11/GPIO_35
 */
#define CYTTSP4_SPI_IRQ_GPIO 38 /* J6.9, C19, GPMC_AD14/GPIO_38 */
#define CYTTSP4_SPI_RST_GPIO 37 /* J6.10, D18, GPMC_AD13/GPIO_37 */
#endif

/* Check GPIO numbers if both I2C and SPI are enabled */
#if defined(CYTTSP4_USE_I2C) && defined(CYTTSP4_USE_SPI)
#if CYTTSP4_I2C_IRQ_GPIO == CYTTSP4_SPI_IRQ_GPIO || \
	CYTTSP4_I2C_RST_GPIO == CYTTSP4_SPI_RST_GPIO
#error "GPIO numbers should be different when both I2C and SPI are on!"
#endif
#endif

#ifndef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_DEVICETREE_SUPPORT
#define CY_VKEYS_X 720
#define CY_VKEYS_Y 1280
#define CY_MAXX 758
#define CY_MAXY 1024
#define CY_MINX 0
#define CY_MINY 0

#define CY_ABS_MIN_X CY_MINX
#define CY_ABS_MIN_Y CY_MINY
#define CY_ABS_MAX_X CY_MAXX
#define CY_ABS_MAX_Y CY_MAXY
#define CY_ABS_MIN_P 0
#define CY_ABS_MAX_P 255
#define CY_ABS_MIN_W 0
#define CY_ABS_MAX_W 255
#define CY_PROXIMITY_MIN_VAL	0
#define CY_PROXIMITY_MAX_VAL	1

#define CY_ABS_MIN_T 0

#define CY_ABS_MAX_T 15

/* Button to keycode conversion */
static u16 cyttsp4_btn_keys[] = {
	/* use this table to map buttons to keycodes (see input.h) */
	KEY_HOMEPAGE,		/* 172 */ /* Previously was KEY_HOME (102) */
				/* New Android versions use KEY_HOMEPAGE */
	KEY_MENU,		/* 139 */
	KEY_BACK,		/* 158 */
	KEY_SEARCH,		/* 217 */
	KEY_VOLUMEDOWN,		/* 114 */
	KEY_VOLUMEUP,		/* 115 */
	KEY_CAMERA,		/* 212 */
	KEY_POWER		/* 116 */
};

static struct touch_settings cyttsp4_sett_btn_keys = {
	.data = (uint8_t *)&cyttsp4_btn_keys[0],
	.size = ARRAY_SIZE(cyttsp4_btn_keys),
	.tag = 0,
};

static struct cyttsp4_core_platform_data _cyttsp4_core_platform_data = {
	.irq_gpio = CYTTSP4_I2C_IRQ_GPIO,
	.rst_gpio = CYTTSP4_I2C_RST_GPIO,
	.xres = cyttsp4_xres,
	.init = cyttsp4_init,
	.power = cyttsp4_power,
	.detect = cyttsp4_detect,
	.irq_stat = cyttsp4_irq_stat,
	.sett = {
		NULL,	/* Reserved */
		NULL,	/* Command Registers */
		NULL,	/* Touch Report */
		NULL,	/* Cypress Data Record */
		NULL,	/* Test Record */
		NULL,	/* Panel Configuration Record */
		NULL,	/* &cyttsp4_sett_param_regs, */
		NULL,	/* &cyttsp4_sett_param_size, */
		NULL,	/* Reserved */
		NULL,	/* Reserved */
		NULL,	/* Operational Configuration Record */
		NULL, /* &cyttsp4_sett_ddata, *//* Design Data Record */
		NULL, /* &cyttsp4_sett_mdata, *//* Manufacturing Data Record */
		NULL,	/* Config and Test Registers */
		&cyttsp4_sett_btn_keys,	/* button-to-keycode table */
	},
	.flags = CY_CORE_FLAG_WAKE_ON_GESTURE,
	.easy_wakeup_gesture = CY_CORE_EWG_TAP_TAP
		| CY_CORE_EWG_TWO_FINGER_SLIDE,
};

static const int16_t cyttsp4_abs[] = {
	ABS_MT_POSITION_X, CY_ABS_MIN_X, CY_ABS_MAX_X, 0, 0,
	ABS_MT_POSITION_Y, CY_ABS_MIN_Y, CY_ABS_MAX_Y, 0, 0,
	ABS_MT_PRESSURE, CY_ABS_MIN_P, CY_ABS_MAX_P, 0, 0,
	CY_IGNORE_VALUE, CY_ABS_MIN_W, CY_ABS_MAX_W, 0, 0,
	ABS_MT_TRACKING_ID, CY_ABS_MIN_T, CY_ABS_MAX_T, 0, 0,
	ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0,
	ABS_MT_TOUCH_MINOR, 0, 255, 0, 0,
	ABS_MT_ORIENTATION, -127, 127, 0, 0,
	ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0,
	ABS_DISTANCE, 0, 255, 0, 0,	/* Used with hover */
};

struct touch_framework cyttsp4_framework = {
	.abs = cyttsp4_abs,
	.size = ARRAY_SIZE(cyttsp4_abs),
	.enable_vkeys = 0,
};

static struct cyttsp4_mt_platform_data _cyttsp4_mt_platform_data = {
	.frmwrk = &cyttsp4_framework,
	.flags = CY_MT_FLAG_FLIP | CY_MT_FLAG_INV_Y,
	.inp_dev_name = CYTTSP4_MT_NAME,
	.vkeys_x = CY_VKEYS_X,
	.vkeys_y = CY_VKEYS_Y,
};

static struct cyttsp4_btn_platform_data _cyttsp4_btn_platform_data = {
	.inp_dev_name = CYTTSP4_BTN_NAME,
};

static const int16_t cyttsp4_prox_abs[] = {
	ABS_DISTANCE, CY_PROXIMITY_MIN_VAL, CY_PROXIMITY_MAX_VAL, 0, 0,
};

struct touch_framework cyttsp4_prox_framework = {
	.abs = cyttsp4_prox_abs,
	.size = ARRAY_SIZE(cyttsp4_prox_abs),
};

static struct cyttsp4_proximity_platform_data
		_cyttsp4_proximity_platform_data = {
	.frmwrk = &cyttsp4_prox_framework,
	.inp_dev_name = CYTTSP4_PROXIMITY_NAME,
};

struct cyttsp4_platform_data _cyttsp4_platform_data = {
	.core_pdata = &_cyttsp4_core_platform_data,
	.mt_pdata = &_cyttsp4_mt_platform_data,
	.btn_pdata = &_cyttsp4_btn_platform_data,
	.prox_pdata = &_cyttsp4_proximity_platform_data,
	.loader_pdata = &_cyttsp4_loader_platform_data,
};
EXPORT_SYMBOL(_cyttsp4_platform_data);

static ssize_t cyttsp4_virtualkeys_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		__stringify(EV_KEY) ":"
		__stringify(KEY_BACK) ":1360:90:160:180"
		":" __stringify(EV_KEY) ":"
		__stringify(KEY_MENU) ":1360:270:160:180"
		":" __stringify(EV_KEY) ":"
		__stringify(KEY_HOMEPAGE) ":1360:450:160:180"
		":" __stringify(EV_KEY) ":"
		__stringify(KEY_SEARCH) ":1360:630:160:180"
		"\n");
}

static struct kobj_attribute cyttsp4_virtualkeys_attr = {
	.attr = {
		.name = "virtualkeys.cyttsp4_mt",
		.mode = S_IRUGO,
	},
	.show = &cyttsp4_virtualkeys_show,
};

static struct attribute *cyttsp4_properties_attrs[] = {
	&cyttsp4_virtualkeys_attr.attr,
	NULL
};

static struct attribute_group cyttsp4_properties_attr_group = {
	.attrs = cyttsp4_properties_attrs,
};
#endif /* !CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_DEVICETREE_SUPPORT */

static void __init ebook_cyttsp4_init(void)
{
#ifndef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_DEVICETREE_SUPPORT
	struct kobject *properties_kobj;
	int ret = 0;

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
				&cyttsp4_properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("%s: failed to create board_properties\n", __func__);
#endif

	/* Initialize muxes for GPIO pins */
#ifdef CYTTSP4_USE_I2C
	ret = gpio_request(CYTTSP4_I2C_IRQ_GPIO, "touch_int");
	if(unlikely(ret)) {
		pr_err("%s: failed to request touch_int gpio\n", __func__);
		return;
	}
	ret = gpio_request(CYTTSP4_I2C_RST_GPIO, "touch_rst");
	if(unlikely(ret)) {
		gpio_free(CYTTSP4_I2C_IRQ_GPIO);
		pr_err("%s: failed to request touch_rst gpio\n", __func__);
		return;
	}
	//omap_mux_init_gpio(CYTTSP4_I2C_RST_GPIO, OMAP_PIN_OUTPUT);
	//omap_mux_init_gpio(CYTTSP4_I2C_IRQ_GPIO, OMAP_PIN_INPUT_PULLUP);
	gpio_pull_updown(CYTTSP4_I2C_IRQ_GPIO, PullDisable);
	gpio_direction_input(CYTTSP4_I2C_IRQ_GPIO);
	gpio_pull_updown(CYTTSP4_I2C_RST_GPIO, PullDisable);
	gpio_direction_output(CYTTSP4_I2C_RST_GPIO, 1);
#endif
#ifdef CYTTSP4_USE_SPI
	omap_mux_init_gpio(CYTTSP4_SPI_RST_GPIO, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(CYTTSP4_SPI_IRQ_GPIO, OMAP_PIN_INPUT_PULLUP);
#endif
}
#endif	//CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4

#ifdef CONFIG_I2C1_RK30
static struct i2c_board_info __initdata i2c1_info[] = {
#if defined (CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4) && defined(CONFIG_MACH_RK3026_T63)
	{
		.type = "cyttsp4_i2c_adapter",
		.addr = 0x24,
		.platform_data = &_cyttsp4_platform_data,
	},
#endif
#if defined (CONFIG_MFD_TPS65910)
	{
		.type           = "tps65910",
		.addr           = TPS65910_I2C_ID0,
		.flags          = 0,
		.irq            = TPS65910_HOST_IRQ,
		.platform_data = &tps65910_data,
	},
#endif
};
#endif

#ifdef CONFIG_I2C2_RK30
static struct i2c_board_info __initdata i2c2_info[] = {
#if defined (CONFIG_TOUCHSCREEN_TrueTouch)
	{
		.udelay         = 15,
		.type           = "cyttsp-i2c",
		.addr           = 0x24,
		.flags          = 0,
		.platform_data  = &cyttsp_data,
	},
#endif
#if defined (CONFIG_TOUCHSCREEN_CYPRESS)
	{
		.type = "cyttsp",
		.addr = 0x24,
	},
#endif
#if defined (CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4) && !defined(CONFIG_MACH_RK3026_T63)
	{
		.type = "cyttsp4_i2c_adapter",
		.addr = 0x24,
		.platform_data = &_cyttsp4_platform_data,
	},
#endif
#if defined (CONFIG_TOUCHSCREEN_CY8CTMA461)
	{
		.type		= CYTTSP4_I2C_NAME,
		.addr		= 0x24,
		.platform_data  = &cyttsp4_info,
	},
#endif
#if defined (CONFIG_TOUCHSCREEN_GT8XX)
	{
		.type           = "Goodix-TS",
		.addr           = 0x55,
		.flags          = 0,
		.irq            = TOUCH_INT_PIN,
		.platform_data  = &goodix_info,
	},
#endif
#if defined (CONFIG_EBC)
	{
		.type		    = "tps65185",
		.addr		    = 0x68,
		.flags		    = 0,
		.platform_data 	= &tps65185_pdata,
	},
#endif
#if defined (CONFIG_TOUCHSCREEN_GT819) || defined(CONFIG_TOUCHSCREEN_GT813)
    {
		.type	        = "Goodix-TS",
		.addr 	        = 0x5D,
		.flags          = 0,
		.irq		    = TOUCH_INT_PIN,
		.platform_data  = &goodix_info,
    },
#endif

#if (defined (CONFIG_EPD_LM75) )
	{
		.type	        = "epd_lm75",
		.addr	        = 0x48,    
		.flags	        = 0,
	},
#endif

#ifdef CONFIG_EPD_G871
	{
		.type	        = "epd_g871",
		.addr	        = 0x4c,    
		.flags	        = 0,
	},
#endif

#if defined(CONFIG_TOUCHSCREEN_XC3)
	{
		.type	=   CY_I2C_NAME,
		.addr 	=  0x24,
		.platform_data = &cypress_i2c_ttsp_platform_data,
		.irq			= CY_I2C_IRQ_GPIO,
	},
#endif

#if defined(CONFIG_TOUCHSCREEN_OED)
	{
		.type	=   CY_I2C_NAME,
		.addr 	=  0x71,
		.platform_data = &cypress_i2c_ttsp_platform_data,
		.irq			= CY_I2C_IRQ_GPIO,
	},
#endif

};
#endif

#ifdef CONFIG_I2C3_RK30
static struct i2c_board_info __initdata i2c3_info[] = {
};
#endif

#ifdef CONFIG_I2C_GPIO_RK30
#define I2C_SDA_PIN     INVALID_GPIO// RK2928_PIN2_PD6   //set sda_pin here
#define I2C_SCL_PIN     INVALID_GPIO//RK2928_PIN2_PD7   //set scl_pin here
static int rk30_i2c_io_init(void)
{
        return 0;
}

struct i2c_gpio_platform_data default_i2c_gpio_data = {
       .sda_pin = I2C_SDA_PIN,
       .scl_pin = I2C_SCL_PIN,
       .udelay = 5, // clk = 500/udelay = 100Khz
       .timeout = 100,//msecs_to_jiffies(100),
       .bus_num    = 5,
       .io_init = rk30_i2c_io_init,
};

static struct i2c_board_info __initdata i2c_gpio_info[] = {
};
#endif

static void __init rk30_i2c_register_board_info(void)
{
#ifdef CONFIG_I2C0_RK30
	i2c_register_board_info(0, i2c0_info, ARRAY_SIZE(i2c0_info));
#endif
#ifdef CONFIG_I2C1_RK30
	i2c_register_board_info(1, i2c1_info, ARRAY_SIZE(i2c1_info));
#endif
#ifdef CONFIG_I2C2_RK30
	i2c_register_board_info(2, i2c2_info, ARRAY_SIZE(i2c2_info));
#endif
#ifdef CONFIG_I2C3_RK30
	i2c_register_board_info(3, i2c3_info, ARRAY_SIZE(i2c3_info));
#endif
#ifdef CONFIG_I2C_GPIO_RK30
	i2c_register_board_info(4, i2c_gpio_info, ARRAY_SIZE(i2c_gpio_info));
#endif
}

/***********************************************************
*	board init
************************************************************/
static struct platform_device *devices[] __initdata = {
#ifdef CONFIG_KEYBOARD_MATRIX
	&matrix_keyboard,
#endif
#ifdef CONFIG_ION
	&device_ion,
#endif
#ifdef CONFIG_WIFI_CONTROL_FUNC
	&rk29sdk_wifi_device,
#endif
#ifdef CONFIG_SND_SOC_RK3026
 	&rk3026_codec,
#endif
#if defined (CONFIG_RK_HEADSET_DET)
	&rk_device_headset,
#endif
#if defined(CONFIG_BATTERY_RK30_ADC_FAC) 
 	&rk30_device_adc_battery,
#endif

#ifdef CONFIG_EBC
	&rk29_device_ebc,
#endif
#if defined(CONFIG_WAVEFORM_FROM_GPIO_SPI)
	&spi_gpio_device,
#endif

};

static void rk30_pm_power_off(void)
{
#if defined(CONFIG_MFD_TPS65910)
	tps65910_device_shutdown();//tps65910 shutdown
#endif
#if defined(CONFIG_REGULATOR_ACT8931)	
int count = 0;
if(pmic_is_act8931())
{
	if(act8931_charge_det){
		while (1) {
		if (gpio_get_value(POWER_ON_PIN) == GPIO_HIGH) {
			printk("POWER_ON_PIN is high\n");
			gpio_set_value(POWER_ON_PIN, GPIO_LOW);
			}	
		msleep(5);
		/* only normal power off can restart system safely */
		if (system_state != SYSTEM_POWER_OFF)
			continue;
		
		if (gpio_get_value(PLAY_ON_PIN) != GPIO_HIGH) {
			if (!count)
				printk("PLAY_ON_PIN is low\n");
			if (50 == count) /* break if keep low about 250ms */
				arm_pm_restart(0, NULL);
			count++;
		} else {
			count = 0;
		}
		}

	}

	act8931_device_shutdown();//act8931 shutdown
}
#endif

	gpio_direction_output(POWER_ON_PIN, GPIO_LOW);
	while(1);
}

static void __init machine_rk30_board_init(void)
{	
#ifdef CONFIG_RK30_PWM_REGULATOR
	pwm_regulator_init();
#endif
	avs_init();
	/*pm_power_off = rk30_pm_power_off;*/

#ifndef CONFIG_KEYBOARD_MATRIX
#if defined(CONFIG_SPI)
	iomux_set(GPIO1_A1);
	gpio_request(RK30_PIN1_PA1, "key_base");
	gpio_direction_output(RK30_PIN1_PA1, GPIO_LOW);
#else
	iomux_set(GPIO1_B2);
	gpio_request(RK30_PIN1_PB2, "key_base");
	gpio_direction_output(RK30_PIN1_PB2, GPIO_LOW);
#endif
#endif

	rk_gpio_request(POWER_ON_PIN, GPIOF_DIR_OUT, GPIO_HIGH, "system power on");
	rk30_i2c_register_board_info();
#if defined(CONFIG_WAVEFORM_FROM_GPIO_SPI)
	init_spi_gpio();
#endif
	spi_register_board_info(board_spi_devices, ARRAY_SIZE(board_spi_devices));
	platform_add_devices(devices, ARRAY_SIZE(devices));
	rk_platform_add_display_devices();	
#if defined (CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4)
	ebook_cyttsp4_init();
#endif
#if defined(CONFIG_WIFI_CONTROL_FUNC)
	rk29sdk_wifi_bt_gpio_control_init();
#elif defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC)
	rk29sdk_wifi_combo_module_gpio_init();
#endif
}

static void __init rk30_reserve(void)
{
#ifdef CONFIG_EBC
    ebc_resource[2].start = board_mem_reserve_add("ebc waveform buf", EBC_RESERVE_SIZE);
    ebc_resource[2].end = ebc_resource[2].start + EBC_RESERVE_SIZE -1;
#endif

    //fb reserve
#ifdef CONFIG_FB_ROCKCHIP
	resource_fb[0].start = board_mem_reserve_add("fb0 buf", get_fb_size());
	resource_fb[0].end = resource_fb[0].start + get_fb_size()- 1;
	#if 0
	resource_fb[1].start = board_mem_reserve_add("ipp buf", RK30_FB0_MEM_SIZE);
	resource_fb[1].end = resource_fb[1].start + RK30_FB0_MEM_SIZE - 1;
	#endif

	#if defined(CONFIG_FB_ROTATE) || !defined(CONFIG_THREE_FB_BUFFER)
	resource_fb[2].start = board_mem_reserve_add("fb2 buf",get_fb_size());
	resource_fb[2].end = resource_fb[2].start + get_fb_size() - 1;
	#endif
#endif
	//ion reserve
#ifdef CONFIG_ION
	rk30_ion_pdata.heaps[0].base = board_mem_reserve_add("ion", ION_RESERVE_SIZE);
#endif
#ifdef CONFIG_VIDEO_RK29
	rk30_camera_request_reserve_mem();
#endif
	board_mem_reserved();
}

/***********************************************************
*	clock
************************************************************/
static struct cpufreq_frequency_table dvfs_arm_table[] = {
	{.frequency = 312 * 1000,		.index = 950 * 1000},
	{.frequency = 504 * 1000,		.index = 1000 * 1000},
	{.frequency = 816 * 1000,		.index = 1200 * 1000},
	{.frequency = 912 * 1000,		.index = 1250 * 1000},
	{.frequency = 1008 * 1000,		.index = 1350 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_gpu_table[] = {
	{.frequency = 200 * 1000,		.index = 1100 * 1000},
	{.frequency = 266 * 1000,		.index = 1100 * 1000},
	{.frequency = 400 * 1000,		.index = 1200 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_ddr_table[] = {
	{.frequency = 200 * 1000 + DDR_FREQ_SUSPEND,	.index = 1100 * 1000},
	{.frequency = 300 * 1000 + DDR_FREQ_VIDEO,		.index = 1100 * 1000},
	{.frequency = 400 * 1000 + DDR_FREQ_NORMAL, 	.index = 1200 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

extern void adjust_dvfs_table(int soc_version, struct cpufreq_frequency_table *table);

void __init board_clock_init(void)
{
	rk2928_clock_data_init(periph_pll_default, codec_pll_default, RK30_CLOCKS_DEFAULT_FLAGS);
	//dvfs_set_arm_logic_volt(dvfs_cpu_logic_table, cpu_dvfs_table, dep_cpu2core_table);	
	
	printk(KERN_INFO "rk3026 soc version:%d\n", rk3026_version_val());
	adjust_dvfs_table(rk3026_version_val(), dvfs_arm_table);
	
	dvfs_set_freq_volt_table(clk_get(NULL, "cpu"), dvfs_arm_table);
	dvfs_set_freq_volt_table(clk_get(NULL, "gpu"), dvfs_gpu_table);
	dvfs_set_freq_volt_table(clk_get(NULL, "ddr"), dvfs_ddr_table);
}

/************************ end *****************************/
MACHINE_START(RK30, "RK30board")
	.boot_params	= PLAT_PHYS_OFFSET + 0x800,
	.fixup		= rk2928_fixup,
	.reserve	= &rk30_reserve,
	.map_io		= rk2928_map_io,
	.init_irq	= rk2928_init_irq,
	.timer		= &rk2928_timer,
	.init_machine	= machine_rk30_board_init,
MACHINE_END
