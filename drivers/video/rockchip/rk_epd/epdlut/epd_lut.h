/*
 * RK29 ebook lut  epd_lut.h
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
#ifndef EPD_LUT_H
#define EPD_LUT_H
/*include*/
#include "spiflash/epd_spi_flash.h"


/*define*/
#define EPD_LUT_READ_FROM_SPI 1

#define LUT_SUCCESS (0)
#define LUT_ERROR (-1)


enum epd_lut_type
{
	WF_TYPE_RESET=1,
	WF_TYPE_GC16,
	WF_TYPE_GL16,
	WF_TYPE_GLR16,
	WF_TYPE_GLD16,
	WF_TYPE_GRAY4	,
	WF_TYPE_GRAY2	,
	WF_TYPE_AUTO,
	WF_TYPE_A2
	
};

struct epd_lut_data
{
	unsigned int  frame_num;
	unsigned int * data;
};

//ebc lut op
struct epd_lut_ops
{
	int (*lut_get)(struct epd_lut_data *,enum epd_lut_type,bool,int,int);
};


/*api*/
//read lut from spi flash.
//cache lut data.
int epd_lut_from_gpio_spi_init(char *nand_buffer);
int epd_lut_from_rk_spi_init(char *nand_buffer);
int epd_lut_from_nand_init(char *temp_buffer);
int epd_lut_from_file_init(char *temp_buffer);
int epd_lut_from_array_init();
//register lut op
int epd_lut_op_register(struct epd_lut_ops *op);

//register spi flash get spi op api.
int epd_spi_flash_register(struct epd_spi_flash_info *epd_spi_flash);
extern int support_pvi_waveform();
extern char *get_waveform_version();


#endif

