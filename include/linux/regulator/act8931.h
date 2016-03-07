
/* include/linux/regulator/act8931.h
 *
 * Copyright (C) 2011 ROCKCHIP, Inc.
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
#ifndef __LINUX_REGULATOR_act8931_H
#define __LINUX_REGULATOR_act8931_H

#include <linux/regulator/machine.h>

//#define ACT8931_START 30

#define ACT8931_LDO1  0                     //(0+ACT8931_START)
#define ACT8931_LDO2  1                    // (1+ACT8931_START)
#define ACT8931_LDO3  2                  //(2+ACT8931_START)
#define ACT8931_LDO4  3                //(3+ACT8931_START)


#define ACT8931_DCDC1 4                //(4+ACT8931_START)
#define ACT8931_DCDC2 5                //(5+ACT8931_START)
#define ACT8931_DCDC3 6                //(6+ACT8931_START)

/* ACT8931 registers address map */
/* SYS */
#define ACT_SYS_0 (0x00)
#define ACT_SYS_1 (0x01)

/* DC-DC 0 */
#define ACT_REG1_0 (0x20)
#define ACT_REG1_1 (0x21)
#define ACT_REG1_2 (0x22)

/* DC-DC 1 */
#define ACT_REG2_0 (0x30)
#define ACT_REG2_1 (0x31)
#define ACT_REG2_2 (0x32)

/* DC-DC 2 */
#define ACT_REG3_0 (0x40)
#define ACT_REG3_1 (0x41)
#define ACT_REG3_2 (0x42)

/* DC-DC 3 */
#define ACT_REG4_0 (0x50)
#define ACT_REG4_1 (0x51)

/* LDO0 */
#define ACT_REG5_0 (0x54)
#define ACT_REG5_1 (0x55)

/* LDO1 */
#define ACT_REG6_0 (0x61)

/* LDO2 */
#define ACT_REG7_0 (0x64)
#define ACT_REG7_1 (0x65)

/* APCH */
#define ACT_APCH_0 (0x78)
#define ACT_APCH_1 (0x79)
#define ACT_APCH_2 (0x7A)


#define ACT_BIT7 (0x01<<7)
#define ACT_BIT6 (0x01<<6)
#define ACT_BIT5 (0x01<<5)
#define ACT_BIT4 (0x01<<4)
#define ACT_BIT3 (0x01<<3)
#define ACT_BIT2 (0x01<<2)
#define ACT_BIT1 (0x01<<1)
#define ACT_BIT0 (0x01<<0)
#define ACT_BIT432 (0x07<<02)
#define ACT_BIT3210 (0x0F<<0)
#define ACT_BIT543210 (0x3F<<0)

#define MASK_TRST (ACT_BIT7)
#define MASK_SYSMODE (ACT_BIT6)
#define MASK_SYSLEVMSK (ACT_BIT5)
#define MASK_SYSSTAT (ACT_BIT4)
#define MASK_SYSLEV (ACT_BIT3210)
#define MASK_SCRATCH (ACT_BIT3210)
#define MASK_VSET1 (ACT_BIT543210)
#define MASK_VSET2 (ACT_BIT543210)
#define MASK_ON (ACT_BIT7)
#define MASK_PHASE (ACT_BIT6)
#define MASK_MODE (ACT_BIT5)
#define MASK_DELAY (ACT_BIT432)
#define MASK_FLTMSK (ACT_BIT1)
#define MASK_OK (ACT_BIT0)
#define MASK_VSET (ACT_BIT543210)
#define MASK_ON (ACT_BIT7)
#define MASK_DIS (ACT_BIT6)
#define MASK_LOWIQ (ACT_BIT5)

#define act8931_NUM_REGULATORS 7
struct act8931;

/*
 * Register definitions to all subdrivers
 */
#define act8931_BUCK1_SET_VOL_BASE 0x20
#define act8931_BUCK2_SET_VOL_BASE 0x30
#define act8931_BUCK3_SET_VOL_BASE 0x40
#define act8931_LDO1_SET_VOL_BASE 0x50
#define act8931_LDO2_SET_VOL_BASE 0x54
#define act8931_LDO3_SET_VOL_BASE 0x60
#define act8931_LDO4_SET_VOL_BASE 0x64

#define act8931_BUCK1_CONTR_BASE 0x22
#define act8931_BUCK2_CONTR_BASE 0x32
#define act8931_BUCK3_CONTR_BASE 0x42
#define act8931_LDO1_CONTR_BASE 0x51
#define act8931_LDO2_CONTR_BASE 0x55
#define act8931_LDO3_CONTR_BASE 0x61
#define act8931_LDO4_CONTR_BASE 0x65

#define BUCK_VOL_MASK 0x3f
#define LDO_VOL_MASK 0x3f

#define VOL_MIN_IDX 0x00
#define VOL_MAX_IDX 0x3f

int act8931_device_shutdown(void);

struct act8931_regulator_subdev {
	int id;
	struct regulator_init_data *initdata;
};

struct act8931_platform_data {
	int num_regulators;
	int (*set_init)(struct act8931 *act8931);
	struct act8931_regulator_subdev *regulators;
};

#if defined(CONFIG_REGULATOR_ACT8931_CHARGE)
int act8931_charge_init(void);
int act8931_charge_deinit(void);
#endif

#endif

