/*
 * Copyright (c) 2019 Intel Corp.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <gen_offset.h>
#include <kernel_structs.h>
#include <kernel_arch_data.h>
#include <arch/x86/multiboot.h>

#ifdef CONFIG_X86_LONGMODE
#include "intel64_offsets.c"
#else
#include "ia32_offsets.c"
#endif

GEN_OFFSET_SYM(_thread_arch_t, flags);

/* size of struct multiboot_info, used by crt0.S/locore.S */

GEN_ABSOLUTE_SYM(__MULTIBOOT_INFO_SIZEOF, sizeof(struct multiboot_info));

GEN_ABS_SYM_END
