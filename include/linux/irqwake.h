/*
    Public irq wake header
    Copyright (C) 2012-2013  rock-chips <xjq @ rock-chips.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program;
 */
#ifndef __LINUX_IRQ_WAKE_H
#define __LINUX_IRQ_WAKE_H


#ifdef CONFIG_IDLE
extern int suspend_irqwake_set(suspend_state_t state);
extern int suspend_valid_idle_mem(suspend_state_t state);
#endif



#endif /* __LINUX_IRQ_WAKE_H */
