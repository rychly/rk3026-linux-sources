/*
 * RK29 ebook control driver rk29_ebc.h
 *
 * Copyright (C) 2010 RockChip, Inc.
 * Author: dlx@rock-chips.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef RK29_EBC_H
#define RK29_EBC_H

#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include "epdlut/epd_lut.h" 
#include "bufmanage/buf_manage.h"
#include "epdlut/epdtemperature/epd_temperature.h"
//#include "bootani/bootani.h"
#include "ebc_dbg/rk29_ebc_dbg.h"


/*define*/
 #define EBC_SUCCESS (0)
 #define EBC_ERROR (-1)

#define RKEBC_DRV_VERSION "2.00"

//#define DIRECT_MODE 
#define AUTO_MODE_ENABLE

/*GET LUT MODE    */
#define   LUT_FROM_GPIO_SPI_FLASH   (0)
#define   LUT_FROM_RK_SPI_FLASH     (1)
#define   LUT_FROM_NAND_FLASH       (2)
#define   LUT_FROM_WAVEFORM_FILE    (3)

/*Select WAVEFORM from nand or spi flash*/
#define NAND_WAVEFORM        (0)
#define SPI_WAVEFORM      (1)

/*SET END DISPLAY*/
#define END_RESET        (0)
#define END_PICTURE      (1)
 /* EPD work mode */

#define EPD_AUTO        	(0)
#define EPD_FULL	  		(1)
#define EPD_A2		  		(2)
#define EPD_PART	  		(3)
#define EPD_FULL_DITHER     (4)
#define EPD_RESET     		(5)
#define EPD_BLACK_WHITE     (6)
#define EPD_TEXT         	(7)
#define EPD_BLOCK           (8)
#define EPD_FULL_WIN        (9)
#define EPD_OED_PART		(10)
#define EPD_DIRECT_PART     (11)
#define EPD_DIRECT_A2        (12)


#define EBC_OFF      (0)
#define EBC_ON        (1)
struct logo_info
{
	int logo_pic_offset;
	int logo_end_offset;
	int logo_power_pic_offset;
};

// io ctl
#define GET_EBC_BUFFER (0x7000)
#define SET_EBC_SEND_BUFFER (0x7001)
#define GET_EBC_DRIVER_SN (0x7002)
#define GET_EBC_BUFFER_INFO (0x7003)
/*ebc_dsp_start*/
#define m_FRAME_NUM      (0x3f<<2)
#define m_FRAME_START   (1)

#define  v_FRAME_NUM(x)     (((x)&0x3f)<<2)
#define  v_FRAME_START(x)  (((x)&1)<<0)
/*ebc_dsp_ctrl*/
#define m_DISPLAY_SWAP  (3<<30)
#define m_DISPLAY_UPDATE_MODE (1<<29)
#define m_DISPLAY_MODE  (1<<28)
#define m_VCOM_MODE       (1<<27)
#define m_SCLK_DIVIDE_RATE (0xf<<16)
#define m_SDO_VALUE        (0xffff<<0)

#define  v_DISPLAY_SWAP(x)  (((x)&3)<<30)
#define  v_DISPLAY_UPDATE_MODE(x) (((x)&1)<<29)
#define  v_DISPLAY_MODE(x)  (((x)&1)<<28)
#define  v_VCOM_MODE(x)       (((x)&1)<<27)
#define  v_SCLK_DIVIDE_RATE(x) (((x)&0xf)<<16)
#define  v_SDO_VALUE(x)        ((x)&0xffff<<0)

/*ebc_epd_ctrl*/
#define  m_EPD_PANEL   (1<<5)
#define  m_POWER_ENABLE (0x07<<2)
#define  m_GATE_SCAN_DIRET (1<<1)
#define  m_SOURCE_SCAN_DIRET (1<<0)

#define  v_EPD_PANEL(x)   (((x)&1)<<5)
#define  v_POWER_ENABLE(x)  (((x)&0x07)<<2)
#define  v_GATE_SCAN_DIRET(x) (((x)&1)<<1)
#define  v_SOURCE_SCAN_DIRET(x) (((x)&1)<<0)

/*ebc_win_ctrl*/
#define  m_WINENABLE  (1<<17)
#define  m_AHB_MASTER_INCR (0x1f<<12)
#define  m_AHB_MASTER_BURST_TYPE (0x07<<9)
#define  m_WIN_FIFO    (0x7f<<2)
#define  m_WIN_FMT     (0x03<<0)

#define  v_WINENABLE(x)  (((x)&1)<<16)
#define  v_AHB_MASTER_INCR(x) (((x)&0x1f)<<11)
#define  v_AHB_MASTER_BURST_TYPE(x) (((x)&0x07)<<8)
#define  v_WIN_FIFO(x)    (((x)&0x3f)<<2)
#define  v_WIN_FMT(x)     (((x)&0x03)<<0)

/*ebc_int_status*/
#define  m_FRAMEEND_CLEAR      (1<<6)
#define  m_DISPLAYEND_CLEAR   (1<<7)
#define  m_FRAMEFLAG_CLEAR    (1<<8)
#define  m_DISPLAYEND_MASK    (1<<4)



#define  v_FRAMEEND_CLEAR(x)   (((x)&1)<<6)
#define  v_DISPLAYEND_CLEAR(x)   (((x)&1)<<7)
#define  v_FRAMEFLAG_CLEAR(x)   (((x)&1)<<8)
#define  v_DISPLAYEND_MASK(x)   (((x)&1)<<4)

#define EBC_DRIVER_SN "RK29_EBC_DRIVER_VERSION_1.00"
#define EBC_DRIVER_SN_LEN sizeof(EBC_DRIVER_SN)
#define ebc_printk(dir_of_file,lev,fmt) ebc_dbg_printk(dir_of_file,lev,fmt)


/*struct*/
typedef volatile struct tagEBC_REG
{
    unsigned int EBC_DSP_START;              //0x00 Frame statrt register
    unsigned int EBC_EPD_CTRL;                //0x04 EPD control register
    unsigned int EBC_DSP_CTRL;                //0x08 Display control register
    unsigned int EBC_DSP_HTIMING0;        //0x0c H-Timing setting register0
    unsigned int EBC_DSP_HTIMING1;        //0x10  H-Timing setting register1
    unsigned int EBC_DSP_VTIMING0;        //0x14 V-Timing setting register0
    unsigned int EBC_DSP_VTIMING1;        //0x18 V-Timing setting register1
    unsigned int EBC_DSP_ACT_INFO;        //0x1c ACTIVE width/height
    unsigned int EBC_WIN_CTRL;                //0x20 Window ctrl
    unsigned int EBC_WIN_MST0;               //0x24 Current win memory start
    unsigned int EBC_WIN_MST1;               //0x28 Next win memory start
    unsigned int EBC_WIN_VIR;                  //0x2c Window vir width/height
    unsigned int EBC_WIN_ACT;                 //0x30 Window act width/height
    unsigned int EBC_WIN_DSP;                 //0x34 Window dsp width/height
    unsigned int EBC_WIN_DSP_ST;           //0x38 Window display start piont
    unsigned int EBC_INT;                          //0x3c Interrupt register
    unsigned int EBC_VCOM0;                     //0x40 VCOM setting register0
    unsigned int EBC_VCOM1;                     //0x44  VCOM setting register1
    unsigned int EBC_VCOM2;                     //0x48 VCOM setting register2
    unsigned int EBC_VCOM3;                     //0x4c VCOM setting register3
    unsigned int EBC_CONFIG_DONE;         //0X50 Config done register
} EBC_REG, *pEBC_REG;
//ebc panel info
struct ebc_panel_info{
	int    width;
	int    height;
	int    hsync_len;//refer to eink spec LSL
	int    hstart_len;//refer to eink spec LBL
	int    vsync_len;//refer to eink spec FSL
	int    vend_len; //refer to eink spec FEL
	int    frame_rate;
	int    vir_width;
	int    vir_height;
	int 	refcount;//fixme
	int   fb_width;
	int   fb_height;
	int   color_panel;
	int   rotate;
	int   hend_len;//refer to eink spec LEL
	int   vstart_len; //refer to eink spec FBL
	int   gdck_sta;//refer to eink spec GDCK_STA
	int   lgonl;//refer to eink spec LGONL
};
/*struct*/
//ebc clocks info 
struct ebc_clk_info{
	struct clk      *dclk;            //ebc dclk
	struct clk      *dclk_parent;     //ebc dclk divider frequency source
	int dclk_status;
	int dclk_parent_status;

	struct clk	*hclk;
	struct clk	*hclk_lcdc;
	struct clk	*aclk_lcdc;
	struct clk      *hclk_disp_matrix;
	int hclk_status;
	int hclk_lcdc_status;
	int aclk_lcdc_status;
	int hclk_disp_matrix_status;

	struct clk	*aclk_ddr_lcdc;   //DDR LCDC AXI clock disable.
	struct clk	*aclk_disp_matrix;	//DISPLAY matrix AXI clock disable.
	struct clk	*hclk_cpu_display;	//CPU DISPLAY AHB bus clock disable.
	struct clk	*pd_display;		// display power domain
	int aclk_ddr_lcdc_status;  
	int aclk_disp_matrix_status;	
	int hclk_cpu_display_status;	
	int pd_display_status;

	int pixclock;
};
struct Epd_timing
{
	int    epd_w;
    	int    epd_h;
	int    lsl;
	int    lbl;
	int    gdck_sta;
	int    lel;
	int    lgonl;
	int    fsl;
	int    fbl;
	int    fel;
	int    h_count;
	int    v_count;
	int    line_w;
	int    line_h;
	int    top_frame;
	int    eink_end;
	int    top_data;
	int    eink_totle;
	int    line_rel;
	//int    *ink_group;
	//int    *ink_group_next;
	unsigned short int    *ink_group;
	unsigned short int    *ink_group_next;
};
struct ebc_lcdc_ops
{
	int (*get_lcdc_display_addr) (struct rk_lcdc_device_driver *dev_drv);
	int (*rk3188_lcdc_reg_ebc_update)(struct rk_lcdc_device_driver *dev_drv);
	int (*rk3188_lcdc_clk_enable)(struct rk3188_lcdc_device *lcdc_dev);
	int (*rk3188_lcdc_clk_disable)(struct rk3188_lcdc_device *lcdc_dev);
};
struct ebc_platform_data{
	int (*io_init)(void);
	int (*io_deinit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
	int (*vcom_power_on)(void);
	int (*vcom_power_off)(void);
	int (*suspend)(struct ebc_platform_data *ebc_data);
	int (*resume)(struct ebc_platform_data *ebc_data);
	const char *regulator;
};
struct ebc_pwr_ops
{
	int (*power_on)(void);
	int (*power_down)(void);
};
struct ebc_temperateure_ops
{
	int (*temperature_get)(char *temp);
};


/*android use struct*/
struct ebc_buf_info{
	int offset; 
	int epd_mode;
	int height;
	int width;
	int vir_height;
	int vir_width;
	int fb_width;
	int fb_height;
	int color_panel;
	int win_x1;
	int win_y1;
	int win_x2;
	int win_y2;
	int rotate;
};

//ebc reg info
struct ebc_reg_info{
	void __iomem *reg_vir_base;  // virtual basic address of ebc register
	u32 reg_phy_base;                 // physical basic address of ebc register
	u32 len;                                   // physical map length of ebc register

	EBC_REG *preg;
    	EBC_REG regbak;
};
// ebc sn
struct ebc_sn_info{
	u32 key;
	u32 sn_len;
	char cip_sn[EBC_DRIVER_SN_LEN];
};

//ebc info
#ifdef CONFIG_HARDWARE_EBC
	struct rk29_ebc_info{
		int ebc_status;
		char frame_total;
		char frame_bw_total;
		short int auto_need_refresh;
		int frame_left;
		int ebc_send_count;
		int ebc_mode;
		int pix_num_left;
		int *lut_addr;
		int height;
		int width;
		short int buffer_need_update;
		short int one_pix_end;
		int  buffer_need_check;
		int bits_per_pixel;
		int ebc_irq_status;
		int ebc_auto_work;
		int ebc_dsp_buf_status;     // 1:have display buffer 0:no display buffer
		struct fb_info *ebc_fb;
		struct ebc_clk_info ebc_clk_info;
		struct ebc_panel_info ebc_panel_info;
		struct ebc_reg_info ebc_reg_info;
		struct epd_lut_ops ebc_lut_ops;
		struct ebc_pwr_ops ebc_pwr_ops;
		struct ebc_temperateure_ops ebc_temp_ops;
		struct ebc_ops *ebc_ops;
		struct epd_lut_data lut_data;
		struct task_struct *ebc_task;
		struct ebc_platform_data *mach_info;

		int    *auto_image_new;
		int    *auto_image_old;
		int   *auto_frame_buffer;
		int   	*auto_image_fb;
		int    *auto_direct_buffer0;
		int    *auto_direct_buffer1;
		int ebc_power_status;
		int ebc_auto_power_off;
		int ebc_last_display;
		int lut_ddr_vir;
		struct ebc_buf_s  *prev_dsp_buf;
		struct ebc_buf_s  *curr_dsp_buf;

		struct wake_lock suspend_lock;
		int wake_lock_is_set;

		struct logo_info logo_info;
		int first_in;//first frame buf flag.
		/* timer to power off vdd */
		struct timer_list	vdd_timer;

		/* bootup animations timer. */
		struct timer_list	boot_logo_timer;

		/* timeout when start frame. */
		struct timer_list	frame_timer;
		struct work_struct work;
		struct ebc_buf_info buf_info;

		/*work*/
		struct work_struct	auto_buffer_work;
		struct work_struct	bootup_ani;
		struct workqueue_struct *bootup_ani_wq;
	};
#endif

#ifdef CONFIG_SOFTWARE_EBC
	struct rk29_ebc_info{
		int ebc_status;
		char frame_total;
		char frame_bw_total;
		short int auto_need_refresh;
		int frame_left;
		int ebc_send_count;
		int ebc_mode;
		int pix_num_left;
		int *lut_addr;
		int height;
		int width;
		short int buffer_need_update;
		short int one_pix_end;
		int  buffer_need_check;
		int bits_per_pixel;
		int ebc_irq_status;
		int ebc_auto_work;
		int ebc_dsp_buf_status;// 1:have display buffer 0:no display buffer
		struct fb_info *ebc_fb;
		struct ebc_clk_info ebc_clk_info;
		struct ebc_panel_info ebc_panel_info;
		struct ebc_reg_info ebc_reg_info;
		struct epd_lut_ops ebc_lut_ops;
		struct ebc_lcdc_ops ebc_lcdc_ops;
		struct ebc_pwr_ops ebc_pwr_ops;
		struct ebc_temperateure_ops ebc_temp_ops;
		struct ebc_ops *ebc_ops;
		struct epd_lut_data lut_data;
		struct task_struct *ebc_task;
		struct ebc_platform_data *mach_info;

		int    *auto_image_new;
		int    *auto_image_old;
		int   *auto_frame_buffer;
		int   	*auto_image_fb;
		int    *auto_direct_buffer0;
		int    *auto_direct_buffer1;
		int ebc_power_status;
		int ebc_last_display;
		int lut_ddr_vir;
		struct ebc_buf_s  *prev_dsp_buf;
		struct ebc_buf_s  *curr_dsp_buf;

		struct wake_lock suspend_lock;
		int wake_lock_is_set;

		struct logo_info logo_info;
		int first_in;//first frame buf flag.
		/* timer to power off vdd */
		struct timer_list	vdd_timer;

		/* bootup animations timer. */
		struct timer_list	boot_logo_timer;

		/* timeout when start frame. */
		struct timer_list	frame_timer;
		struct work_struct work;
		struct ebc_buf_info buf_info;

		/*work*/
		struct work_struct	auto_buffer_work;
		struct work_struct	bootup_ani;
		struct workqueue_struct *bootup_ani_wq;
	};

#endif

/* modiy for open ebc ioctl */
extern int rkebc_register_notifier(struct notifier_block *nb);
extern int rkebc_unregister_notifier(struct notifier_block *nb);
#ifdef CONFIG_SOFTWARE_EBC
extern void ebc_init_lcdc();
extern void ebc_timing_init(struct Epd_timing * time);
extern void register_ebc_timing();
#endif
extern int register_ebc_pwr_ops(struct ebc_pwr_ops *ops);
extern int register_ebc_lcdc_ops(struct ebc_lcdc_ops *ops);
extern int register_ebc_temp_ops(struct ebc_temperateure_ops *ops);
#if CONFIG_BATTERY_RK30_ADC_FAC
extern int rk30_adc_battery_get_bat_vol(void);
extern void rk30_adc_ebc_battery_check(void);
#endif
extern int rkebc_notify(unsigned long event);
extern int rk29ebc_notify(unsigned long event);
extern long ebc_io_ctl(struct file *file, unsigned int cmd, unsigned long arg);	
extern int ebc_sn_encode(char *sn,char *cip_sn,int sn_len,int key);


/* public function */
extern void set_epd_info(struct ebc_panel_info *epd_panel,struct ebc_clk_info *epd_clk,int *height,int *width);
extern int  get_lut_position();

extern int set_end_display();
extern int set_logo_info(struct logo_info *plogo_info);
extern int get_bootup_logo_cycle();
extern int is_bootup_ani_loop(void);
extern int is_need_show_lowpower_pic(void);
extern int support_double_thread_calcu(void);
extern int support_bootup_ani(void);
extern int support_tps_3v3_always_alive(void);
extern int get_bootup_ani_mode(void);



#endif
 
