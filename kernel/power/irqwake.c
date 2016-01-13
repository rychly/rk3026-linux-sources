/*
	* kernel/power/irqwake.c - PM subsystem irq relation functionality.
	*
	* Copyright (C) 2012-2013  rock-chips <xjq @ rock-chips.com>
 
	* This program is free software; you can redistribute it and/or modify
	* it under the terms of the GNU General Public License as published by
	* the Free Software Foundation; either version 2 of the License, or
	* (at your option) any later version.

	* This program is distributed in the hope that it will be useful,
	* but WITHOUT ANY WARRANTY; without even the implied warranty of
	* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	* GNU General Public License for more details.

	* You should have received a copy of the GNU General Public License
	* along with this program;
 */
 
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/irqwake.h>

/* #cat /proc/interrupts
168:          1          0      GPIO  Goodix-TS, cyttsp3-TS                                                                                                                                                           
169:          1          0      GPIO  act8931, axp228                                                                                                                                                              
171:          0          0      GPIO  rtc_hym8563                                                                                                                                                          
199:          0          0      GPIO  sd_detect                                                                                                                                                            
*/

#if defined(CONFIG_SPI)
  #if defined(CONFIG_MACH_RK3026_E601) || defined(CONFIG_MACH_RK3026_T63)
    #define IRQ_WAKE_ID_TS (164)
    #define IRQ_WAKE_ID_PMU (162)
  #else
    #define IRQ_WAKE_ID_TS (162)
    #define IRQ_WAKE_ID_PMU (164)
  #endif
  #define IRQ_WAKE_ID_RTC (160)
#else
  #define IRQ_WAKE_ID_TS (168)
  #define IRQ_WAKE_ID_PMU (169)
  #define IRQ_WAKE_ID_RTC (171)
#endif

enum rk_irq_wake_id{
	IRQ_WAKE_TS = 0,	
	IRQ_WAKE_PMU,	
	IRQ_WAKE_RTC,
	IRQ_WAKE_MAX,
};

static int irq_wake_id[IRQ_WAKE_MAX] = {
	IRQ_WAKE_ID_TS,
	IRQ_WAKE_ID_PMU,
	IRQ_WAKE_ID_RTC,
};

int pm_irqwake_enable(void)
{
	int ret = 0;
	
	ret |= enable_irq_wake(irq_wake_id[IRQ_WAKE_TS]);
	if (ret)
		pr_err("%s:ret=%d\n",__FUNCTION__,ret);

	return ret;
}

int pm_irqwake_disable(void)
{
	int ret = 0;

	ret |= disable_irq_wake(irq_wake_id[IRQ_WAKE_TS]);
	if (ret)
		pr_err("%s:ret=%d\n",__FUNCTION__,ret);

	return ret;
}

int suspend_valid_idle_mem(suspend_state_t state)
{
	return (state == PM_SUSPEND_IDLE) || (state == PM_SUSPEND_MEM);
}

static inline bool irqwake_valid_state(suspend_state_t state)
{
	return (state == PM_SUSPEND_ON || suspend_valid_idle_mem(state));
}

static suspend_state_t old_state = PM_SUSPEND_ON;

int suspend_irqwake_set(suspend_state_t state)
{	
	int ret = 0;
	
	if (!irqwake_valid_state(state))
		goto err;
	
	//pr_info("%s: %d -> %d\n", __FUNCTION__,old_state,state);

	if (((old_state == PM_SUSPEND_ON)&&(state == PM_SUSPEND_IDLE)) ||
		 ((old_state == PM_SUSPEND_MEM)&&(state == PM_SUSPEND_IDLE)))
	{
		ret = pm_irqwake_enable();
		goto exit;
	}

	if (((old_state == PM_SUSPEND_IDLE)&&(state == PM_SUSPEND_ON)) ||
		 ((old_state == PM_SUSPEND_IDLE)&&(state == PM_SUSPEND_MEM)))
	{
		ret = pm_irqwake_disable();
		goto exit;
	}
	
	if (((old_state == PM_SUSPEND_ON)&&(state == PM_SUSPEND_MEM)) ||
		 ((old_state == PM_SUSPEND_MEM)&&(state == PM_SUSPEND_ON)))
		goto exit;
	
	goto err;
exit:
	old_state = state;
err:
	if (ret)
		pr_err("%s:ret=%d\n",__FUNCTION__,ret);
	return ret;
}

