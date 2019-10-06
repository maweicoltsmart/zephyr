/*
 * Copyright (c) 2016 Cadence Design Systems, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <debug/tracing.h>

/*
 * @brief Put the CPU in low-power mode
 *
 * This function always exits with interrupts unlocked.
 *
 * void z_arch_cpu_idle(void)
 */
void z_arch_cpu_idle(void)
{
	sys_trace_idle();
	__asm__ volatile ("waiti 0");
}
/*
 * @brief Put the CPU in low-power mode, entered with IRQs locked
 *
 * This function exits with interrupts restored to <key>.
 *
 * void z_arch_cpu_atomic_idle(unsigned int key)
 */
void z_arch_cpu_atomic_idle(unsigned int key)
{
	sys_trace_idle();
	__asm__ volatile ("waiti 0\n\t"
			  "wsr.ps %0\n\t"
			  "rsync" :: "a"(key));
}
