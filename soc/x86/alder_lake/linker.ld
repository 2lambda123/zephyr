/*
 * Copyright (c) 2011-2014, Wind River Systems, Inc.
 * Copyright (c) 2019-2023 Intel Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/x86/memory.ld>

#ifdef CONFIG_X86_64
#include <zephyr/arch/x86/intel64/linker.ld>
#else
#include <zephyr/arch/x86/ia32/linker.ld>
#endif /* CONFIG_X86_64 */
