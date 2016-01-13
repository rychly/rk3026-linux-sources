/*
 * include/linux/input/cy8ctma463.h
 *
 * Copyright (C) 2014 Cypress, Inc.
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

#ifndef __CY8CTMA463_H__
#define __CY8CTMA463_H__

#define CYTTSP4_I2C_NAME	"CY-TS"

#define CY_MULTI_TOUCH
#define MAX_FINGER_NUM		10

#define CY_I2C_DATA_SIZE	(3 * 256)

#define CY_REG_BASE		0x00

#define CY_SYSINFO_MODE		0x10 /* rd/wr hst_mode */

#define SCREEN_MAX_HEIGHT	1024
#define SCREEN_MAX_WIDTH	768

/* maximum number of command data bytes */
#define CY_NUM_DAT		3

/* register field lengths */
#define CY_NUM_REVCTRL              8
#define CY_NUM_MFGID                8
#define CY_NUM_TCHREC               10
#define CY_NUM_DDATA                32
#define CY_NUM_MDATA                64

#define CY_IGNORE_VALUE		0xFFFF

/* helpers */
#define GET_NUM_TOUCHES(x)		((x) & 0x1F)
#define IS_LARGE_AREA(x)		(((x) & 0x20) >> 5)
#define IS_BAD_PKT(x)			((x) & 0x20)
#define IS_VALID_APP(x)			((x) & 0x01)
#define IS_OPERATIONAL_ERR(x)		((x) & 0x3F)
#define GET_HSTMODE(reg)		((reg & 0x70) >> 4)
#define GET_BOOTLOADERMODE(reg)		((reg & 0x10) >> 4)

/* touch event id codes */
#define CY_GET_EVENTID(reg)		((reg & 0x60) >> 5)
#define CY_GET_TRACKID(reg)		(reg & 0x1F)


/* exit bootloader mode */
static const u8 ldr_exit[] = {
	0xff, 0x01, 0x3b, 0x00, 0x00, 0x4f, 0x6d, 0x17
};

/* enter operating mode */
static const u8 opt_enter[] = {
	0x0C
};

/* TTSP System Information interface definitions */
struct cyttsp4_cydata {
	u8 ttpidh;
	u8 ttpidl;
	u8 fw_ver_major;
	u8 fw_ver_minor;
	u8 revctrl[CY_NUM_REVCTRL];
	u8 blver_major;
	u8 blver_minor;
	u8 jtag_si_id3;
	u8 jtag_si_id2;
	u8 jtag_si_id1;
	u8 jtag_si_id0;
	u8 mfgid_sz;
	u8 mfg_id[CY_NUM_MFGID];
	u8 cyito_idh;
	u8 cyito_idl;
	u8 cyito_verh;
	u8 cyito_verl;
	u8 ttsp_ver_major;
	u8 ttsp_ver_minor;
	u8 device_info;
} __attribute__((packed));

struct cyttsp4_test {
	u8 post_codeh;
	u8 post_codel;
	u8 reserved[2];	/* was bist_code */
} __attribute__((packed));

struct cyttsp4_pcfg {
	u8 electrodes_x;
	u8 electrodes_y;
	u8 len_xh;
	u8 len_xl;
	u8 len_yh;
	u8 len_yl;
	u8 axis_xh;
	u8 axis_xl;
	u8 axis_yh;
	u8 axis_yl;
	u8 max_zh;
	u8 max_zl;
} __attribute__((packed));

struct cyttsp4_opcfg {
	u8 cmd_ofs;
	u8 rep_ofs;
	u8 rep_szh;
	u8 rep_szl;
	u8 num_btns;
	u8 tt_stat_ofs;
	u8 obj_cfg0;
	u8 max_tchs;
	u8 tch_rec_siz;
	u8 tch_rec[CY_NUM_TCHREC];
	u8 reserved[4];
} __attribute__((packed));

struct cyttsp4_ddata {
	u8 ddata[CY_NUM_DDATA];
} __attribute__((packed));

struct cyttsp4_mdata {
	u8 mdata[CY_NUM_MDATA];
} __attribute__((packed));

struct cyttsp4_sysinfo_data {
	u8 hst_mode;
	u8 reserved;
	u8 map_szh;
	u8 map_szl;
	u8 cydata_ofsh;
	u8 cydata_ofsl;
	u8 test_ofsh;
	u8 test_ofsl;
	u8 pcfg_ofsh;
	u8 pcfg_ofsl;
	u8 opcfg_ofsh;
	u8 opcfg_ofsl;
	u8 ddata_ofsh;
	u8 ddata_ofsl;
	u8 mdata_ofsh;
	u8 mdata_ofsl;
	struct cyttsp4_cydata cydata;
	struct cyttsp4_test cytest;
	struct cyttsp4_pcfg pcfg;
	struct cyttsp4_opcfg opcfg;
	struct cyttsp4_ddata ddata;
	struct cyttsp4_mdata mdata;
} __attribute__((packed));

struct cyttsp4_ts {
	struct i2c_client *client;
	u8 wr_buf[CY_I2C_DATA_SIZE];
	struct hrtimer timer;
	struct mutex lock;
	atomic_t timeout;

	struct input_dev *input_dev;
	char phys[32];
	uint32_t irq_gpio;
	uint32_t irq;
	struct work_struct work;
	struct workqueue_struct *wq;
	struct cyttsp4_sysinfo_data sysinfo_data;
};

struct cyttsp4_touch {
	u8 xh;
	u8 xl;
	u8 yh;
	u8 yl;
	u8 z;
	u8 t;  //EventID:TouchID
	//u8 size;
	u8 majorAx;
	u8 minorAx;
	u8 orientation;
} __attribute__((packed));

struct cyttsp4_xydata {
	u8 hst_mode;
	u8 reserved;
	u8 cmd;
	u8 cmddat[CY_NUM_DAT];
	u8 rep_len;
	//u8 repdat[CY_REPNUM_DAT];
	u8 rep_stat;
	u8 button;
	u8 gest_id;
	u8 gest_cnt;
	u8 tt_stat;
	struct cyttsp4_touch tch[MAX_FINGER_NUM];
} __attribute__((packed));

#endif //__CY8CTMA463_H__
