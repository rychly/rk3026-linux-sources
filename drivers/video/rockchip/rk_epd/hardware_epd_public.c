
#include "ebc.h"
#include "bootani/bootani.h"
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/sched.h>

void set_epd_info(struct ebc_panel_info *epd_panel,struct ebc_clk_info *epd_clk,int *width,int *height)/* epd_panel->width must 8 align*/
{
	int panel_height;
	int panel_width;
		#ifdef CONFIG_V110_EINK_800X600
		if(epd_panel){
			epd_panel->hsync_len=2;
			epd_panel->hstart_len=3;
			epd_panel->vsync_len=4; 
			epd_panel->vend_len=1; 
			epd_panel->frame_rate=50;
			epd_panel->width= 800;
			epd_panel->height=600;
			epd_panel ->vir_width = 896;
			epd_panel ->vir_height = 600;
			epd_panel ->fb_width = 800;
			epd_panel ->fb_height = 600;
			epd_panel ->color_panel = 0;
			epd_panel ->rotate = 270;
		}
		if(epd_clk)
			epd_clk->pixclock = 10000000;
		if(width)
			*width = 800;
		if(height)
			*height = 600;
		#elif defined(CONFIG_V220_EINK_800X600)
		if(epd_panel){
			epd_panel->hsync_len=2; //refer to eink spec LSL
			epd_panel->hstart_len=4;//refer to eink spec LBL
			epd_panel->hend_len=43;//refer to eink spec LEL
			epd_panel->gdck_sta = 1;//refer to eink spec GDCK_STA
			epd_panel->lgonl = 204;//refer to eink spec LGONL
			epd_panel->vsync_len=1; //refer to eink spec FSL
			epd_panel->vstart_len=4; //refer to eink spec FBL
			epd_panel->vend_len=8; //refer to eink spec FEL
			epd_panel->frame_rate=85;
			epd_panel->width=800;
			epd_panel->height=600;
			epd_panel ->vir_width = 800;//896;
			epd_panel ->vir_height = 600;
			epd_panel ->fb_width = 800;
			epd_panel ->fb_height = 600;
			epd_panel ->color_panel = 0;
			epd_panel ->rotate = 270;
		}
		if(epd_clk)
			epd_clk->pixclock = 13000000;
		if(width)
			*width = 800;
		if(height)
			*height = 600;
		#elif defined(CONFIG_V220_EINK_1024X768)
		if(epd_panel){
			epd_panel->hsync_len=6; //refer to eink spec LSL
			epd_panel->hstart_len=6;//refer to eink spec LBL
			epd_panel->hend_len=38;//refer to eink spec LEL
			epd_panel->gdck_sta = 4;//refer to eink spec GDCK_STA
			epd_panel->lgonl = 262;//refer to eink spec LGONL
			epd_panel->vsync_len=2; //refer to eink spec FSL
			epd_panel->vstart_len=4; //refer to eink spec FBL
			epd_panel->vend_len=5; //refer to eink spec FEL
			epd_panel->frame_rate=85;
			epd_panel->width=1024;
			epd_panel->height=768;
			epd_panel ->vir_width = 1024;
			epd_panel ->vir_height = 768;
			epd_panel ->fb_width = 1024;
			epd_panel ->fb_height = 768;
			epd_panel ->color_panel = 0;
			epd_panel ->rotate = 270;
		}
		if(epd_clk)
			epd_clk->pixclock = 20000000;
		if(width)
			*width = 1024;
		if(height)
			*height = 768;
		#elif defined(CONFIG_V220_EINK_1024X758)
		if(epd_panel){
			epd_panel->hsync_len=6; //refer to eink spec LSL
			epd_panel->hstart_len=6;//refer to eink spec LBL
			epd_panel->hend_len=38;//refer to eink spec LEL
			epd_panel->gdck_sta = 4;//refer to eink spec GDCK_STA
			epd_panel->lgonl = 262;//refer to eink spec LGONL
			epd_panel->vsync_len=2; //refer to eink spec FSL
			epd_panel->vstart_len=4; //refer to eink spec FBL
			epd_panel->vend_len=5; //refer to eink spec FEL
			epd_panel->frame_rate=85;
			epd_panel->width=1024;
			epd_panel->height=758;
			epd_panel ->vir_width = 1024;
			epd_panel ->vir_height = 758;
			epd_panel ->fb_width = 1024;
			epd_panel ->fb_height = 758;
			epd_panel ->color_panel = 0;
			epd_panel ->rotate = 270;
		}
		if(epd_clk)
			epd_clk->pixclock = 20000000;
		if(width)
			*width = 1024;
		if(height)
			*height = 758;
		#elif defined(CONFIG_V110_EINK_1200X825)
		if(epd_panel){
			epd_panel->hsync_len=10; //refer to eink spec LSL
			epd_panel->hstart_len=6;//refer to eink spec LBL
			epd_panel->hend_len=69;//refer to eink spec LEL
			epd_panel->gdck_sta = 0;//refer to eink spec GDCK_STA
			epd_panel->lgonl = 301;//refer to eink spec LGONL
			epd_panel->vsync_len=4; //refer to eink spec FSL
			epd_panel->vstart_len=4; //refer to eink spec FBL
			epd_panel->vend_len=10; //refer to eink spec FEL
			epd_panel->frame_rate=50;
			epd_panel->width=1200;
			epd_panel->height=825;
			epd_panel ->vir_width = 1280;
			epd_panel ->vir_height = 825;
			epd_panel ->fb_width = 1200;
			epd_panel ->fb_height = 825;
			epd_panel ->color_panel = 0;
			epd_panel ->rotate = 270;
		}
		if(epd_clk)
			epd_clk->pixclock = 16250000;
		if(width)
			*width = 1200;
		if(height)
			*height = 825;
		#elif defined(CONFIG_V220_EINK_1200X825)
		if(epd_panel){
			epd_panel->hsync_len=15;
			epd_panel->hstart_len=10;
			epd_panel->vsync_len=4; 
			epd_panel->vend_len=30; 
			epd_panel->frame_rate=85;
			epd_panel->width=1200;
			epd_panel->height=825;
			epd_panel ->vir_width = 1280;
			epd_panel ->vir_height = 825;
			epd_panel ->fb_width = 1200;
			epd_panel ->fb_height = 825;
			epd_panel ->color_panel = 0;
			epd_panel ->rotate = 270;
		}
		if(epd_clk)
			epd_clk->pixclock = 25000000;
		if(width)
			*width = 1200;
		if(height)
			*height = 825;
		#elif defined(CONFIG_V220_EINK_800X480)
		if(epd_panel){
			epd_panel->hsync_len=2;
			epd_panel->hstart_len=3;
			epd_panel->vsync_len=4; 
			epd_panel->vend_len=60; 
			epd_panel->frame_rate=85;
			epd_panel->width=800;
			epd_panel->height=480;
			epd_panel ->vir_width = 896;
			epd_panel ->vir_height = 600;
			epd_panel ->fb_width = 800;
			epd_panel ->fb_height = 480;
			epd_panel ->color_panel = 0;
			epd_panel ->rotate = 270;
		}
		if(epd_clk)
			epd_clk->pixclock = 15000000;
		if(width)
			*width = 800;
		if(height)
			*height = 480;
		#elif defined(CONFIG_V110_EINK_1600X1200)
		if(epd_panel){
			epd_panel->hsync_len=6; //refer to eink spec LSL
			epd_panel->hstart_len=4;//refer to eink spec LBL
			epd_panel->hend_len=13;//refer to eink spec LEL
			epd_panel->gdck_sta = 2;//refer to eink spec GDCK_STA
			epd_panel->lgonl = 400;//refer to eink spec LGONL
			epd_panel->vsync_len=1; //refer to eink spec FSL
			epd_panel->vstart_len=4; //refer to eink spec FBL
			epd_panel->vend_len=7; //refer to eink spec FEL
			epd_panel->frame_rate=65;
			epd_panel->width=1600;
			epd_panel->height=1200;
			epd_panel ->vir_width = 1600;
			epd_panel ->vir_height = 1200;
			epd_panel ->fb_width = 1600;
			epd_panel ->fb_height = 1200;
			epd_panel ->color_panel = 0;
			epd_panel ->rotate = 270;
		}
		if(epd_clk)
			epd_clk->pixclock = 33330000;//150000000;
		if(width)
			*width = 1600;
		if(height)
			*height = 1200;
		#elif defined(CONFIG_COLOR_EINK_800X600)
		if(epd_panel){
			epd_panel->hsync_len=15;
			epd_panel->hstart_len=10;
			epd_panel->vsync_len=4; 
			epd_panel->vend_len=4; 
			epd_panel->frame_rate=85;
			epd_panel->width= 1808;//1800;
			epd_panel->height=800;
			epd_panel ->vir_width = 1920;//1920;//1920;
			epd_panel ->vir_height = 800;
			epd_panel ->fb_width = 600;
			epd_panel ->fb_height = 800;//800;
			epd_panel ->color_panel = 1;
			epd_panel ->rotate = 0;
		}
		if(epd_clk)
			epd_clk->pixclock = 38000000;
		if(width)
			*width = 600;
		if(height)
			*height =800;// 800;
		#endif

		return ;
}

extern struct rk29_ebc_info *prk29_ebc_info;
extern wait_queue_head_t ebc_thread_wq;
extern wait_queue_head_t ebc_poweroff_wq;

long ebc_io_ctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret=-1;
	switch (cmd) {
		case GET_EBC_BUFFER:
		{	
			char *temp_addr;
			int temp_offset;
			struct ebc_buf_s *buf;
			struct ebc_buf_info buf_info;
            
			/*stop kernel animation.*/
			if(support_bootup_ani() && prk29_ebc_info->bootup_ani_wq){
				del_timer(&prk29_ebc_info->boot_logo_timer);
				boot_ani_deinit();
				destroy_workqueue(prk29_ebc_info->bootup_ani_wq);
				prk29_ebc_info->bootup_ani_wq = NULL;
			}
			buf = ebc_empty_buf_get();
			temp_offset = buf->phy_addr - ebc_phy_buf_base_get();
			ebc_printk(_module_ebc_c_, EBC_INFO,("rk29 ebc io ctl,cmd=%d; arg=0x%x; temp_addr=0x%x;temp_offset=0x%x.\n",cmd,arg,temp_addr,temp_offset));
			buf_info.offset = temp_offset;
			buf_info.height = prk29_ebc_info->ebc_panel_info.height;
			buf_info.width = prk29_ebc_info->ebc_panel_info.width;
			buf_info.vir_width = prk29_ebc_info->ebc_panel_info.vir_width;
			buf_info.vir_height = prk29_ebc_info->ebc_panel_info.vir_height;
			buf_info.fb_width = prk29_ebc_info->ebc_panel_info.fb_width;
			buf_info.fb_height = prk29_ebc_info->ebc_panel_info.fb_height;
			buf_info.color_panel = prk29_ebc_info->ebc_panel_info.color_panel;
			buf_info.rotate = prk29_ebc_info->ebc_panel_info.rotate;
			if(arg){
				ret = copy_to_user(arg, &buf_info, sizeof(struct ebc_buf_info)) ? -EFAULT : 0;
			}
			if(prk29_ebc_info->ebc_status == 0){
				rk29ebc_notify(EBC_ON);
			}
            break;
		}
			
		case SET_EBC_SEND_BUFFER:
		{	
			char *temp_addr;
			struct ebc_buf_s *buf;
			struct ebc_buf_info buf_info;
			if (copy_from_user(&buf_info, arg, sizeof(struct ebc_buf_info))){
				printk("copy err\n");
				return -EFAULT;
			}
			temp_addr = ebc_phy_buf_base_get() + buf_info.offset;
			buf = ebc_find_buf_by_phy_addr(temp_addr);

			if(temp_addr && buf){
				buf->buf_mode = buf_info.epd_mode;// EPD_PART;//
				if(buf_info.color_panel == 1){
					buf->win_x1 = buf_info.win_x1*3;
					buf->win_x2 = buf_info.win_x2*3;
					buf->win_y1 = buf_info.win_y1;
					buf->win_y2 = buf_info.win_y2;
				} else {
					buf->win_x1 = buf_info.win_x1;
					buf->win_x2 = buf_info.win_x2;
					buf->win_y1 = buf_info.win_y1;
					buf->win_y2 = buf_info.win_y2;
				}
				if(buf->buf_mode == EPD_AUTO){
#ifndef AUTO_MODE_ENABLE
					buf->buf_mode = EPD_PART;
#endif
				}	
				ret=ebc_add_to_dsp_buf_list(buf);
				if(0 == prk29_ebc_info->ebc_dsp_buf_status){
					prk29_ebc_info->ebc_dsp_buf_status = 1;
					wake_up_interruptible_sync(&ebc_thread_wq);
				}
			} else {
				ebc_printk(_module_ebc_c_, EBC_INFO,("send buffer err.\n"));
			}
            
			if(buf_info.epd_mode == EPD_BLOCK){
				prk29_ebc_info->ebc_last_display = 1;
				wait_event_interruptible(ebc_poweroff_wq,(prk29_ebc_info->ebc_last_display==0));
			}
            break;
		}
			
		case GET_EBC_DRIVER_SN:
		{	
			struct ebc_sn_info sn_info;
				
			if (copy_from_user(&sn_info, arg, sizeof(struct ebc_sn_info))){
				printk("copy err\n");
				return -EFAULT;
			}
			ebc_sn_encode(EBC_DRIVER_SN, sn_info.cip_sn, EBC_DRIVER_SN_LEN, sn_info.key);
			//ebc_sn_decode(EBC_DRIVER_SN, sn_info.cip_sn, EBC_DRIVER_SN_LEN, sn_info.key);
			if(arg){
				ret = copy_to_user(arg, &sn_info, sizeof(struct ebc_sn_info)) ? -EFAULT : 0;
			}
			break;
		}
			
		case GET_EBC_BUFFER_INFO:
		{
			struct ebc_buf_info buf_info;
			buf_info.height = prk29_ebc_info->ebc_panel_info.height;
			buf_info.width = prk29_ebc_info->ebc_panel_info.width;
			buf_info.vir_width = prk29_ebc_info->ebc_panel_info.vir_width;
			buf_info.vir_height = prk29_ebc_info->ebc_panel_info.vir_height;
			buf_info.fb_height =  prk29_ebc_info->ebc_panel_info.fb_height;
			buf_info.fb_width =  prk29_ebc_info->ebc_panel_info.fb_width;
			buf_info.color_panel = prk29_ebc_info->ebc_panel_info.color_panel;
			buf_info.rotate = prk29_ebc_info->ebc_panel_info.rotate;
			if(arg){
				ret = copy_to_user(arg, &buf_info, sizeof(struct ebc_buf_info)) ? -EFAULT : 0;
			}
            break;
		}
			
		default:
        {
			ret = 0;
            break;
		}
	}

	return ret;
}

int support_pvi_waveform()
{
	return 1;
#ifdef CONFIG_PVI_WAVEFORM
	return SPI_WAVEFORM;
#else
	return NAND_WAVEFORM;
#endif
}

int  get_lut_position()
{
#ifdef CONFIG_WAVEFORM_FROM_GPIO_SPI
	return LUT_FROM_GPIO_SPI_FLASH;
#elif defined(CONFIG_WAVEFORM_FROM_RK_SPI)
	return LUT_FROM_RK_SPI_FLASH;
#elif defined(CONFIG_WAVEFORM_FROM_NAND_FLASH)
	return LUT_FROM_NAND_FLASH;
#elif defined(CONFIG_WAVEFORM_FROM_WAVEFORM_FILE)
	return LUT_FROM_WAVEFORM_FILE;
#endif
}

int set_end_display()
{
#ifdef CONFIG_END_PICTURE
	return END_PICTURE;
#else
	return END_RESET;
#endif
	
}

int get_bootup_logo_cycle(void)
{
	return CONFIG_EBC_ANI_CYC_TIME;
}

int is_bootup_ani_loop(void)
{
#if CONFIG_EBC_BOOT_ANI_LOOP
	return 1;
#else
	return 0;
#endif
}

int is_need_show_lowpower_pic(void)
{
#if CONFIG_SHOW_BATLOW_PIC
	return 1;
#else
	return 0;
#endif
}

int support_bootup_ani(void)
{
#if CONFIG_EBC_BOOT_ANI
	return 1;
#else
	return 0;
#endif
}

int get_bootup_ani_mode(void)
{
#if CONFIG_EBC_BOOT_ANI_A2
	return EPD_A2;
#elif CONFIG_EBC_BOOT_ANI_PART
	return EPD_PART;
#elif  CONFIG_EBC_BOOT_ANI_FULL
	return EPD_FULL;
#else
	return EPD_FULL;
#endif
}

int support_tps_3v3_always_alive(void) 
{
#if CONFIG_EPD_PMIC_VccEink_ALWAYS_ALIVE
	return 1;
#else
	return 0;
#endif
}



