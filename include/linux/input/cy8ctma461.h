/*
 * include/linux/input/cy8ctma461.h
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

#ifndef __CY8CTMA461_H__
#define __CY8CTMA461_H__

#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <mach/gpio.h>
#include <linux/input/mt.h>

#include <linux/suspend.h>
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#define CYTTSP4_I2C_NAME	"CY-TS"

#define CYTTSP4_MT_NAME		"cyttsp4_mt"

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

/* maximum number of concurrent tracks */
#define CY_NUM_TCH_ID               10

#define CY_ACTIVE_STYLUS_ID         10

/* helpers */
#define GET_NUM_TOUCHES(x)		((x) & 0x1F)
#define IS_LARGE_AREA(x)		(((x) & 0x20) >> 5)
#define IS_BAD_PKT(x)			((x) & 0x20)
#define IS_VALID_APP(x)			((x) & 0x01)
#define IS_OPERATIONAL_ERR(x)		((x) & 0x3F)
#define GET_HSTMODE(reg)		((reg & 0x70) >> 4)
#define GET_TOGGLE(reg)			((reg & 0x80) >> 7)
#define GET_BOOTLOADERMODE(reg)		((reg & 0x10) >> 4)

#define IS_BOOTLOADER(hst_mode, reset_detect)	((hst_mode) & 0x01 || (reset_detect) != 0)

/* touch event id codes */
#define CY_GET_EVENTID(reg)		((reg & 0x60) >> 5)
#define CY_GET_TRACKID(reg)		(reg & 0x1F)

/* x-axis resolution of panel in pixels */
#define CY_PCFG_RESOLUTION_X_MASK 0x7F

/* y-axis resolution of panel in pixels */
#define CY_PCFG_RESOLUTION_Y_MASK 0x7F

/* x-axis, 0:origin is on left side of panel, 1: right */
#define CY_PCFG_ORIGIN_X_MASK 0x80

/* y-axis, 0:origin is on top side of panel, 1: bottom */
#define CY_PCFG_ORIGIN_Y_MASK 0x80

#define CY_TOUCH_SETTINGS_MAX 32
#define CY_TOUCH_SETTINGS_PARAM_REGS 6

/* button to keycode support */
#define CY_NUM_BTN_PER_REG      4
#define CY_NUM_BTN_EVENT_ID     4
#define CY_BITS_PER_BTN         2

enum cyttsp4_btn_state {
	CY_BTN_RELEASED = 0,
	CY_BTN_PRESSED = 1,
	CY_BTN_NUM_STATE
};

#define CY_REG_CAT_CMD              2
#define CY_CMD_COMPLETE_MASK        (1 << 6)
#define CY_CMD_MASK                 0x3F

enum cyttsp4_ic_ebid {
	CY_TCH_PARM_EBID,
	CY_MDATA_EBID,
	CY_DDATA_EBID,
};

/* touch record system information offset masks and shifts */
#define CY_BYTE_OFS_MASK            0x1F
#define CY_BOFS_MASK                0xE0
#define CY_BOFS_SHIFT               5

#define CY_REQUEST_EXCLUSIVE_TIMEOUT    500
#define CY_COMMAND_COMPLETE_TIMEOUT     500

#define CY_TMA1036_TCH_REC_SIZE     6
#define CY_TMA4XX_TCH_REC_SIZE      9
#define CY_TMA1036_MAX_TCH          0x0E
#define CY_TMA4XX_MAX_TCH           0x1E

enum cyttsp4_cmd_op {
	CY_CMD_OP_NULL,
	CY_CMD_OP_RESERVED_1,
	CY_CMD_OP_GET_PARAM,
	CY_CMD_OP_SET_PARAM,
	CY_CMD_OP_RESERVED_2,
	CY_CMD_OP_GET_CRC,
};

#define CY_CMD_OP_NULL_CMD_SZ   1
#define CY_CMD_OP_NULL_RET_SZ   0
#define CY_CMD_OP_GET_CRC_CMD_SZ        2
#define CY_CMD_OP_GET_CRC_RET_SZ        3

#define CY_CORE_BL_HOST_SYNC_BYTE	0xFF

enum cyttsp4_hst_mode_bits {
        CY_HST_TOGGLE      = (1 << 7),
        CY_HST_MODE_CHANGE = (1 << 3),
        CY_HST_MODE        = (7 << 4),
        CY_HST_OPERATE     = (0 << 4),
        CY_HST_SYSINFO     = (1 << 4),
        CY_HST_CAT         = (2 << 4),
        CY_HST_LOWPOW      = (1 << 2),
        CY_HST_SLEEP       = (1 << 1),
        CY_HST_RESET       = (1 << 0),
};

enum cyttsp_cmd_bits {
	CY_CMD_COMPLETE    = (1 << 6),
};

enum cyttsp4_cmd_cat {
	CY_CMD_CAT_NULL,
	CY_CMD_CAT_RESERVED_1,
	CY_CMD_CAT_GET_CFG_ROW_SZ,
	CY_CMD_CAT_READ_CFG_BLK,
	CY_CMD_CAT_WRITE_CFG_BLK,
	CY_CMD_CAT_RESERVED_2,
	CY_CMD_CAT_LOAD_SELF_TEST_DATA,
	CY_CMD_CAT_RUN_SELF_TEST,           //0x07, 
	CY_CMD_CAT_GET_SELF_TEST_RESULT,    //0x08,
	CY_CMD_CAT_CALIBRATE_IDACS,         //0x09,
	CY_CMD_CAT_INIT_BASELINES,          //0x0A,
	CY_CMD_CAT_EXEC_PANEL_SCAN,         //0x0B,
	CY_CMD_CAT_RETRIEVE_PANEL_SCAN,     //0x0C,
	CY_CMD_CAT_START_SENSOR_DATA_MODE,  //0x0D,
	CY_CMD_CAT_STOP_SENSOR_DATA_MODE,   //0x0E,
	CY_CMD_CAT_INT_PIN_MODE,            //0x0F,
	CY_CMD_CAT_RETRIEVE_DATA_STRUCTURE, //0x10,
	CY_CMD_CAT_VERIFY_CFG_BLK_CRC,
	CY_CMD_CAT_RESERVED_N,
};

enum cyttsp4_mode {
	CY_MODE_UNKNOWN		= 0,
	CY_MODE_BOOTLOADER	= (1 << 1),
	CY_MODE_OPERATIONAL	= (1 << 2),
	CY_MODE_SYSINFO		= (1 << 3),
	CY_MODE_CAT		= (1 << 4),
	CY_MODE_STARTUP		= (1 << 5),
	CY_MODE_LOADER		= (1 << 6),
	CY_MODE_CHANGE_MODE	= (1 << 7),
	CY_MODE_CHANGED		= (1 << 8),
	CY_MODE_CMD_COMPLETE	= (1 << 9),
};

enum cyttsp4_int_state {
	CY_INT_NONE,
	CY_INT_IGNORE		= (1 << 0),
	CY_INT_MODE_CHANGE	= (1 << 1),
	CY_INT_EXEC_CMD		= (1 << 2),
	CY_INT_AWAKE		= (1 << 3),
};

enum cyttsp4_sleep_state {
	SS_SLEEP_OFF,
	SS_SLEEP_ON,
	SS_SLEEPING,
	SS_WAKING,
};

/* exit bootloader mode */
static const u8 ldr_exit[] = {
	0xFF, 0x01, 0x3B, 0x00, 0x00, 0x4F, 0x6D, 0x17
};

/* enter operating mode */
static const u8 opt_enter[] = {
	0x0C
};

#define CY_POST_CODEL_WDG_RST           0x01
#define CY_POST_CODEL_CFG_DATA_CRC_FAIL 0x02
#define CY_POST_CODEL_PANEL_TEST_FAIL   0x04

#define CY_TEST_CMD_NULL                0

/* test mode NULL command driver codes; D */
enum cyttsp4_null_test_cmd_code {
	CY_NULL_CMD_NULL,
	CY_NULL_CMD_MODE,
	CY_NULL_CMD_STATUS_SIZE,
	CY_NULL_CMD_HANDSHAKE,
	CY_NULL_CMD_LOW_POWER,
};

enum cyttsp4_test_mode {
	CY_TEST_MODE_NORMAL_OP,         /* Send touch data to OS; normal op */
	CY_TEST_MODE_CAT,               /* Configuration and Test */
	CY_TEST_MODE_SYSINFO,           /* System information mode */
	CY_TEST_MODE_CLOSED_UNIT,       /* Send scan data to sysfs */
};

struct cyttsp4_test_mode_params {
	int cur_mode;
	int cur_cmd;
	size_t cur_status_size;
};

/* GEN4/SOLO Operational interface definitions */
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
	u8 *mfg_id;
	u8 cyito_idh;
	u8 cyito_idl;
	u8 cyito_verh;
	u8 cyito_verl;
	u8 ttsp_ver_major;
	u8 ttsp_ver_minor;
	u8 device_info;
} __packed;

struct cyttsp4_test {
	u8 post_codeh;
	u8 post_codel;
} __packed;

struct cyttsp4_pcfg {
	u8 electrodes_x;
	u8 electrodes_y;
	u8 len_xh;
	u8 len_xl;
	u8 len_yh;
	u8 len_yl;
	u8 res_xh;
	u8 res_xl;
	u8 res_yh;
	u8 res_yl;
	u8 max_zh;
	u8 max_zl;
} __packed;

enum cyttsp4_tch_abs {  /* for ordering within the extracted touch data array */
	CY_TCH_X,       /* X */
	CY_TCH_Y,       /* Y */
	CY_TCH_P,       /* P (Z) */
	CY_TCH_T,       /* TOUCH ID */
	CY_TCH_E,       /* EVENT ID */
	CY_TCH_O,       /* OBJECT ID */
	CY_TCH_W,       /* SIZE */
	CY_TCH_MAJ,     /* TOUCH_MAJOR */
	CY_TCH_MIN,     /* TOUCH_MINOR */
	CY_TCH_OR,      /* ORIENTATION */
	CY_TCH_NUM_ABS
};

static const char * const cyttsp4_tch_abs_string[] = {
	[CY_TCH_X]      = "X",
	[CY_TCH_Y]      = "Y",
	[CY_TCH_P]      = "P",
	[CY_TCH_T]      = "T",
	[CY_TCH_E]      = "E",
	[CY_TCH_O]      = "O",
	[CY_TCH_W]      = "W",
	[CY_TCH_MAJ]    = "MAJ",
	[CY_TCH_MIN]    = "MIN",
	[CY_TCH_OR]     = "OR",
	[CY_TCH_NUM_ABS] = "INVALID"
};

#define CY_NUM_TCH_FIELDS       7
#define CY_NUM_EXT_TCH_FIELDS   3

struct cyttsp4_tch_rec_params {
	u8 loc;
	u8 size;
} __packed;

struct cyttsp4_opcfg {
	u8 cmd_ofs;
	u8 rep_ofs;
	u8 rep_szh;
	u8 rep_szl;
	u8 num_btns;
	u8 tt_stat_ofs;
	u8 obj_cfg0;
	u8 max_tchs;
	u8 tch_rec_size;
	struct cyttsp4_tch_rec_params tch_rec_old[CY_NUM_TCH_FIELDS];
	u8 btn_rec_size;/* btn record size (in bytes) */
	u8 btn_diff_ofs;/* btn data loc ,diff counts, (Op-Mode byte ofs) */
	u8 btn_diff_size;/* btn size of diff counts (in bits) */
	struct cyttsp4_tch_rec_params tch_rec_new[CY_NUM_EXT_TCH_FIELDS];
} __packed;

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
} __packed;

struct cyttsp4_ddata {
	u8 lab126_fw_ver0;
	u8 lab126_fw_ver1;
} __packed;

struct cyttsp4_sysinfo_ptr {
	struct cyttsp4_cydata *cydata;
	struct cyttsp4_test *test;
	struct cyttsp4_pcfg *pcfg;
	struct cyttsp4_opcfg *opcfg;
	struct cyttsp4_ddata *ddata;
	struct cyttsp4_mdata *mdata;
} __packed;

struct cyttsp4_tch_abs_params {
	size_t ofs;     /* abs byte offset */
	size_t size;    /* size in bits */
	size_t max;     /* max value */
	size_t bofs;    /* bit offset */
};

struct cyttsp4_sysinfo_ofs {
	size_t chip_type;
	size_t cmd_ofs;
	size_t rep_ofs;
	size_t rep_sz;
	size_t num_btns;
	size_t num_btn_regs;    /* ceil(num_btns/4) */
	size_t tt_stat_ofs;
	size_t tch_rec_size;
	size_t obj_cfg0;
	size_t max_tchs;
	size_t mode_size;
	size_t data_size;
	size_t map_sz;
	size_t max_x;
	size_t x_origin;        /* left or right corner */
	size_t max_y;
	size_t y_origin;        /* upper or lower corner */
	size_t max_p;
	size_t cydata_ofs;
	size_t test_ofs;
	size_t pcfg_ofs;
	size_t opcfg_ofs;
	size_t ddata_ofs;
	size_t mdata_ofs;
	size_t cydata_size;
	size_t test_size;
	size_t pcfg_size;
	size_t opcfg_size;
	size_t ddata_size;
	size_t mdata_size;
	size_t btn_keys_size;
	struct cyttsp4_tch_abs_params tch_abs[CY_TCH_NUM_ABS];
	size_t btn_rec_size; /* btn record size (in bytes) */
	size_t btn_diff_ofs;/* btn data loc ,diff counts, (Op-Mode byte ofs) */
	size_t btn_diff_size;/* btn size of diff counts (in bits) */
};

struct cyttsp4_btn {
	bool enabled;
	int state;      /* CY_BTN_PRESSED, CY_BTN_RELEASED */
	int key_code;
};

struct cyttsp4_crc {
	u8 ic_tt_cfg_crc[2];
};

#define CY_NORMAL_ORIGIN 0      /* upper, left corner */
#define CY_INVERT_ORIGIN 1      /* lower, right corner */

struct cyttsp4_sysinfo {
	bool ready;
	struct cyttsp4_sysinfo_data si_data;
	struct cyttsp4_sysinfo_ptr si_ptrs;
	struct cyttsp4_sysinfo_ofs si_ofs;
	struct cyttsp4_btn *btn;        /* button states */
	struct cyttsp4_crc crc;
#ifdef SHOK_SENSOR_DATA_MODE
	struct cyttsp4_sensor_monitor monitor;
#endif
	u8 *btn_rec_data;               /* button diff count data */
	u8 *xy_mode;                    /* operational mode and status regs */
	u8 *xy_data;                    /* operational touch regs */
};

#define CY_MAX_PRBUF_SIZE		PIPE_BUF
#define CY_PR_TRUNCATED			" truncated..."

enum cyttsp4_ic_grpnum {
	CY_IC_GRPNUM_RESERVED,
	CY_IC_GRPNUM_CMD_REGS,
	CY_IC_GRPNUM_TCH_REP,
	CY_IC_GRPNUM_DATA_REC,
	CY_IC_GRPNUM_TEST_REC,
	CY_IC_GRPNUM_PCFG_REC,
	CY_IC_GRPNUM_TCH_PARM_VAL,
	CY_IC_GRPNUM_TCH_PARM_SIZE,
	CY_IC_GRPNUM_RESERVED1,
	CY_IC_GRPNUM_RESERVED2,
	CY_IC_GRPNUM_OPCFG_REC,
	CY_IC_GRPNUM_DDATA_REC,
	CY_IC_GRPNUM_MDATA_REC,
	CY_IC_GRPNUM_TEST_REGS,
	CY_IC_GRPNUM_BTN_KEYS,
	CY_IC_GRPNUM_TTHE_REGS,
	CY_IC_GRPNUM_NUM
};

struct cyttsp4_touch {
	int abs[CY_TCH_NUM_ABS];
};

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

struct touch_settings {
	const uint8_t   *data;
	uint32_t         size;
	uint8_t         tag;
} __packed;

#define CY_VKEYS_X 720
#define CY_VKEYS_Y 1280

enum cyttsp4_flags {
	CY_FLAG_NONE = 0x00,
	CY_FLAG_HOVER = 0x04,
	CY_FLAG_FLIP = 0x08,
	CY_FLAG_INV_X = 0x10,
	CY_FLAG_INV_Y = 0x20,
	CY_FLAG_VKEYS = 0x40,
};

enum cyttsp4_event_id {
	CY_EV_NO_EVENT,
	CY_EV_TOUCHDOWN,
	CY_EV_MOVE,             /* significant displacement (> act dist) */
	CY_EV_LIFTOFF,          /* record reports last position */
};

enum cyttsp4_object_id {
	CY_OBJ_STANDARD_FINGER,
	CY_OBJ_LARGE_OBJECT,
	CY_OBJ_STYLUS,
	CY_OBJ_HOVER,
};

/* abs settings */
#define CY_IGNORE_VALUE             0xFFFF
/* abs signal capabilities offsets in the frameworks array */
enum cyttsp4_sig_caps {
	CY_SIGNAL_OST,
	CY_MIN_OST,
	CY_MAX_OST,
	CY_FUZZ_OST,
	CY_FLAT_OST,
	CY_NUM_ABS_SET  /* number of signal capability fields */
};

/* abs axis signal offsets in the framworks array  */
enum cyttsp4_sig_ost {
	CY_ABS_X_OST,
	CY_ABS_Y_OST,
	CY_ABS_P_OST,
	CY_ABS_W_OST,
	CY_ABS_ID_OST,
	CY_ABS_MAJ_OST,
	CY_ABS_MIN_OST,
	CY_ABS_OR_OST,
	CY_NUM_ABS_OST  /* number of abs signals */
};

struct touch_framework {
	const uint16_t  *abs;
	uint8_t         size;
	uint8_t         enable_vkeys;
} __packed;

struct cyttsp4_mt_platform_data {
	struct touch_framework *frmwrk;
	unsigned short flags;
	char const *inp_dev_name;
};

struct cyttsp4_mt_data;
struct cyttsp4_mt_function {
        void (*report_slot_liftoff)(struct cyttsp4_mt_data *md);
        void (*input_sync)(struct input_dev *input);
        void (*input_report)(struct input_dev *input, int sig, int t);

        /*return number of touches on the panel*/
        int (*final_sync)(struct input_dev *input, int max_tchs,
                        int mt_sync_count, int *ids);
        int (*input_register_device)(struct input_dev *input, int max_tchs);
};

struct cyttsp4_mt_data {
        struct input_dev *input;
        struct cyttsp4_sysinfo *si;
        bool input_device_registered;
        char phys[NAME_MAX];
        int num_prv_tch;
        int prv_tch_type;
	struct cyttsp4_mt_function mt_function;
#ifdef VERBOSE_DEBUG
        u8 pr_buf[CY_MAX_PRBUF_SIZE];
#endif
        struct cyttsp4_mt_platform_data *pdata;
};

struct cyttsp4_core_platform_data {
	int level_irq_udelay;
	struct touch_settings *sett[CY_TOUCH_SETTINGS_MAX];
};

struct cy8ctma46x_platform_data {
	int irq_pin;
	int reset_pin;
	int (*power)(int on);
	int (*xres)(struct cy8ctma46x_platform_data *pdata);
	int (*init_platform_hw)(void);
	void *md_platform_data;
};

struct cyttsp4_ts {
	struct i2c_client *client;
	struct hrtimer timer;
	struct mutex lock;
	int int_status;
	int cmd_toggle;
	enum cyttsp4_mode mode;
	enum cyttsp4_sleep_state sleep_state;
	atomic_t timeout;

	char phys[32];
	uint32_t irq;
	uint32_t rst;
	struct work_struct work;
	struct workqueue_struct *wq;
	wait_queue_head_t wait_q;
	struct wake_lock wakelock;
	struct cyttsp4_sysinfo sysinfo;
	u8 wr_buf[CY_I2C_DATA_SIZE];
	u8 pr_buf[CY_MAX_PRBUF_SIZE];
	struct cyttsp4_core_platform_data *core_pdata;
        struct cyttsp4_mt_data *md;
	struct cy8ctma46x_platform_data *pdata;
#if defined(CONFIG_HAS_EARLYSUSPEND)
        struct early_suspend es;
#endif
};

#endif //__CY8CTMA461_H__
