
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/sched.h>

#include "ebc.h"
#include "bootani/bootani.h"
#include <linux/rk_fb.h>
#include <linux/rk_screen.h>
#include "../lcdc/rk3188_lcdc.h"

#define OUT_TYPE		SCREEN_MCU
#define OUT_FACE		OUT_P16BPP4



/* Timing */
#define H_PW			(1)
#define H_BP		    (1)

#define H_FP			(1)

#define V_PW			(1)
#define V_BP			(1)

#define V_FP			(1)
//#define LCD_WIDTH          	800
//#define LCD_HEIGHT         	600

//LSL+LBL+LEL必须为2的倍数
#ifdef CONFIG_V220_EINK_800X600//v220 800*600
#define PANELW          (800)  //(800)
#define PANELH          (600)  //(600)
#define P_WR           	(4)     // do not setting
#define OUT_CLK        	(13000000)
#define LSL        		(2)
#define LBL       		(4)
#define LEL       		(42)
#define GDCK_STA   		(1)
#define LGONL  			(204)
#define FSL      		(1)
#define FBL      		(4)
#define FEL     		(8)
#endif
#if CONFIG_V220_EINK_1024X758 // v220 1024*758
#define PANELW          (1024)   		// 1440
#define PANELH          (758)   		// 1080
#define P_WR           	(4)      		// do not setting
#define OUT_CLK        	(20000000) 
#define LSL        		(6)
#define LBL       		(6)
#define LEL       		(38)
#define GDCK_STA   		(4)
#define LGONL  			(262)
#define FSL      		(2)
#define FBL      		(4)
#define FEL     		(5)
#endif
#if CONFIG_V220_EINK_1200X825 //v220 1200*825
#define PANELW          (1200)
#define PANELH          (825)
#define P_WR            (4) 		// do not setting
#define OUT_CLK        	(16500000) 
#define LSL        		(10)
#define LBL       		(6)
#define LEL       		(70)
#define GDCK_STA   		(0)
#define LGONL  			(302)
#define FSL      		(4)
#define FBL      		(4)
#define FEL     		(10)
#endif
#if CONFIG_EINK_1200X825 //v220 1200*825
#define PANELW          (1600)
#define PANELH          (1200)
#define P_WR            (4) // do not setting
#define OUT_CLK         (33300000 )
#define LSL        		(6)
#define LBL       		(4)
#define LEL       		(12)
#define GDCK_STA   		(2)
#define LGONL  			(343)
#define FSL      		(1)
#define FBL      		(4)
#define FEL     		(7)
#endif

struct Epd_timing *p_time = NULL;
void ebc_init_lcdc(struct rk29fb_screen *screen, struct rk29lcd_info *lcd_info)
{
	screen->type = OUT_TYPE;
	screen->face = OUT_FACE;
	screen->pixclock = OUT_CLK * (P_WR+1);
	screen->left_margin = H_BP;
	screen->right_margin = H_FP;
	screen->hsync_len = H_PW;
	screen->upper_margin = V_BP;
	screen->lower_margin = V_FP;
	screen->vsync_len = V_PW;
	screen->mcu_wrperiod = P_WR;
	/* Pin polarity */
	screen->pin_hsync = 0;
	screen->pin_vsync = 0;
	screen->pin_den = 0;
	screen->pin_dclk = 0;

	/* Swap rule */
	screen->swap_rb = 0;
	screen->swap_rg = 0;
	screen->swap_gb = 0;
	screen->swap_delta = 0;
	screen->swap_dumy = 0;
	if(!p_time){
		p_time = kmalloc(sizeof(struct Epd_timing),GFP_KERNEL);
		p_time->epd_w=PANELW;
		p_time->epd_h=PANELH;
		p_time->lsl=LSL; 
		p_time->lbl=LBL;
		p_time->lel=LEL;
		p_time->gdck_sta=GDCK_STA;
		p_time->fsl=FSL; 
		p_time->fbl=FBL;
		p_time->fel=FEL;
		p_time->lgonl=LGONL;
		ebc_timing_init(p_time);
		screen->x_res = p_time->h_count;
		screen->y_res = p_time->v_count;
	}
	register_ebc_timing(p_time);
	//printk("hl p_time->h_count=%d, p_time->v_count=%d, p_time->top_frame=%d, p_time->line_rel=%d\n",p_time->h_count,p_time->v_count,p_time->top_frame,p_time->line_rel);
}


void set_epd_info(struct ebc_panel_info *epd_panel,struct ebc_clk_info *epd_clk,int *width,int *height)/* epd_panel->width must 8 align*/
{
	int panel_height;
	int panel_width;
	if(epd_panel){
		epd_panel->width= PANELW;
		epd_panel->height=PANELH;
		epd_panel ->vir_width = PANELW;
		epd_panel ->vir_height = PANELH;
		epd_panel ->fb_width = PANELW;
		epd_panel ->fb_height = PANELH;
		epd_panel ->color_panel = 0;
		epd_panel ->rotate = 270;
	}
	if(epd_clk)
		epd_clk->pixclock = 40000000;
	if(width)
		*width = PANELW;
	if(height)
		*height = PANELH;
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
			if(support_bootup_ani() && prk29_ebc_info->bootup_ani_wq)
			{
				del_timer(&prk29_ebc_info->boot_logo_timer);
				boot_ani_deinit();
				destroy_workqueue(prk29_ebc_info->bootup_ani_wq);
				prk29_ebc_info->bootup_ani_wq = NULL;
			}
			buf = ebc_empty_buf_get();
			if(buf == -1)
				return -1;
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
			if(arg)
				ret = copy_to_user(arg, &buf_info, sizeof(struct ebc_buf_info)) ? -EFAULT : 0;
			if(prk29_ebc_info->ebc_status == 0){
				rkebc_notify(EBC_ON);
			}
		}
			break;

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
			if(temp_addr && buf)
			{
				buf->buf_mode =buf_info.epd_mode;//EPD_FULL;//EPD_PART;// 
				if(buf_info.color_panel == 1){
					buf->win_x1 = buf_info.win_x1*3;
					buf->win_x2 = buf_info.win_x2*3;
					buf->win_y1 = buf_info.win_y1;
					buf->win_y2 = buf_info.win_y2;
				}
				else{
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
				if(0 == prk29_ebc_info->ebc_dsp_buf_status)
				{
					prk29_ebc_info->ebc_dsp_buf_status = 1;
					wake_up_interruptible_sync(&ebc_thread_wq);
				}
			}
			else
			{
				ebc_printk(_module_ebc_c_, EBC_INFO,("send buffer err.\n"));
			}
			
			if(buf->buf_mode == EPD_BLOCK){
				prk29_ebc_info->ebc_last_display = 1;
				wait_event_interruptible(ebc_poweroff_wq,(prk29_ebc_info->ebc_last_display==0));
			}
		}
			break;

		case GET_EBC_DRIVER_SN:
		{	
			struct ebc_sn_info sn_info;
				
			if (copy_from_user(&sn_info, arg, sizeof(struct ebc_sn_info))){
				printk("copy err\n");
				return -EFAULT;
			}
			ebc_sn_encode(EBC_DRIVER_SN, sn_info.cip_sn, EBC_DRIVER_SN_LEN, sn_info.key);
			//ebc_sn_decode(EBC_DRIVER_SN, sn_info.cip_sn, EBC_DRIVER_SN_LEN, sn_info.key);
			if(arg)
				ret = copy_to_user(arg, &sn_info, sizeof(struct ebc_sn_info)) ? -EFAULT : 0;
			
		}
			break;
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
			if(arg)
				ret = copy_to_user(arg, &buf_info, sizeof(struct ebc_buf_info)) ? -EFAULT : 0;
		}
			break;
		default:
			ret=0;
	}
	return ret;
}

int support_pvi_waveform()
{
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
int support_double_thread_calcu(void)
{
#if CONFIG_DOUBLE_THREAD_CALCU
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
#else
	return EPD_PART;
#endif
}




