/******************************************************************/
/*	Copyright (C)  ROCK-CHIPS FUZHOU . All Rights Reserved. 	  */
/*******************************************************************
File   :  RKPowerOnAnimation.c
Desc   :
Author :	Rockchip Power On Animation Packer For 2906 Eink SDK.
Date   :	Mon Jan 28 18:16:09 2013

Notes  :  If there has any questions, please contact with 
          yzc@rock-chips.com or dlx@rock-chips.com
********************************************************************/
#ifndef __RK_POWERON_ANI_H__
#define __RK_POWERON_ANI_H__
#include <linux/init.h>
#include <linux/syscalls.h>
#include <asm/io.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/stat.h>

/* This array stores the data for power on animation. */
static unsigned int gRKAniData[] __initdata = {0};
const int *grk_ani_data __initconst = gRKAniData;

static int __init boot_animation_init(void)
{
	int fd;
	u8 *bootani;
	struct kstat animation_info;

	bootani = "/animation.bin";
	vfs_stat(bootani, &animation_info);
	if(animation_info.size < 0)
		return -ENODATA;
	grk_ani_data = kmalloc(animation_info.size, GFP_KERNEL);
	if(!grk_ani_data)
		return -ENOMEM;
	fd = sys_open(bootani, O_RDONLY, 0600);
	sys_lseek(fd, 0, SEEK_SET);
	sys_read(fd, (char *)grk_ani_data, animation_info.size);
	sys_close(fd);

	return 0;
}

rootfs_initcall(boot_animation_init);
#endif
