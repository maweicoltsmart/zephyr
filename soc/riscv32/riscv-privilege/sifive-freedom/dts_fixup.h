/*
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* GPIO 0 */
#define DT_SIFIVE_GPIO_0_BASE_ADDR	DT_SIFIVE_GPIO0_10012000_BASE_ADDRESS
#define DT_SIFIVE_GPIO_0_IRQ_0		DT_SIFIVE_GPIO0_10012000_IRQ_0
#define DT_SIFIVE_GPIO_0_IRQ_1		DT_SIFIVE_GPIO0_10012000_IRQ_1
#define DT_SIFIVE_GPIO_0_IRQ_2		DT_SIFIVE_GPIO0_10012000_IRQ_2
#define DT_SIFIVE_GPIO_0_IRQ_3		DT_SIFIVE_GPIO0_10012000_IRQ_3
#define DT_SIFIVE_GPIO_0_IRQ_4		DT_SIFIVE_GPIO0_10012000_IRQ_4
#define DT_SIFIVE_GPIO_0_IRQ_5		DT_SIFIVE_GPIO0_10012000_IRQ_5
#define DT_SIFIVE_GPIO_0_IRQ_6		DT_SIFIVE_GPIO0_10012000_IRQ_6
#define DT_SIFIVE_GPIO_0_IRQ_7		DT_SIFIVE_GPIO0_10012000_IRQ_7
#define DT_SIFIVE_GPIO_0_IRQ_8		DT_SIFIVE_GPIO0_10012000_IRQ_8
#define DT_SIFIVE_GPIO_0_IRQ_9		DT_SIFIVE_GPIO0_10012000_IRQ_9
#define DT_SIFIVE_GPIO_0_IRQ_10		DT_SIFIVE_GPIO0_10012000_IRQ_10
#define DT_SIFIVE_GPIO_0_IRQ_11		DT_SIFIVE_GPIO0_10012000_IRQ_11
#define DT_SIFIVE_GPIO_0_IRQ_12		DT_SIFIVE_GPIO0_10012000_IRQ_12
#define DT_SIFIVE_GPIO_0_IRQ_13		DT_SIFIVE_GPIO0_10012000_IRQ_13
#define DT_SIFIVE_GPIO_0_IRQ_14		DT_SIFIVE_GPIO0_10012000_IRQ_14
#define DT_SIFIVE_GPIO_0_IRQ_15		DT_SIFIVE_GPIO0_10012000_IRQ_15
#define DT_SIFIVE_GPIO_0_IRQ_16		DT_SIFIVE_GPIO0_10012000_IRQ_16
#define DT_SIFIVE_GPIO_0_IRQ_17		DT_SIFIVE_GPIO0_10012000_IRQ_17
#define DT_SIFIVE_GPIO_0_IRQ_18		DT_SIFIVE_GPIO0_10012000_IRQ_18
#define DT_SIFIVE_GPIO_0_IRQ_19		DT_SIFIVE_GPIO0_10012000_IRQ_19
#define DT_SIFIVE_GPIO_0_IRQ_20		DT_SIFIVE_GPIO0_10012000_IRQ_20
#define DT_SIFIVE_GPIO_0_IRQ_21		DT_SIFIVE_GPIO0_10012000_IRQ_21
#define DT_SIFIVE_GPIO_0_IRQ_22		DT_SIFIVE_GPIO0_10012000_IRQ_22
#define DT_SIFIVE_GPIO_0_IRQ_23		DT_SIFIVE_GPIO0_10012000_IRQ_23
#define DT_SIFIVE_GPIO_0_IRQ_24		DT_SIFIVE_GPIO0_10012000_IRQ_24
#define DT_SIFIVE_GPIO_0_IRQ_25		DT_SIFIVE_GPIO0_10012000_IRQ_25
#define DT_SIFIVE_GPIO_0_IRQ_26		DT_SIFIVE_GPIO0_10012000_IRQ_26
#define DT_SIFIVE_GPIO_0_IRQ_27		DT_SIFIVE_GPIO0_10012000_IRQ_27
#define DT_SIFIVE_GPIO_0_IRQ_28		DT_SIFIVE_GPIO0_10012000_IRQ_28
#define DT_SIFIVE_GPIO_0_IRQ_29		DT_SIFIVE_GPIO0_10012000_IRQ_29
#define DT_SIFIVE_GPIO_0_IRQ_30		DT_SIFIVE_GPIO0_10012000_IRQ_30
#define DT_SIFIVE_GPIO_0_IRQ_31		DT_SIFIVE_GPIO0_10012000_IRQ_31
#define DT_SIFIVE_GPIO_0_SIZE		DT_SIFIVE_GPIO0_10012000_SIZE

/* PLIC */
#define DT_PLIC_MAX_PRIORITY	\
	DT_RISCV_PLIC0_C000000_RISCV_MAX_PRIORITY
#define DT_PLIC_PRIO_BASE_ADDR	\
	DT_RISCV_PLIC0_C000000_PRIO_BASE_ADDRESS
#define DT_PLIC_IRQ_EN_BASE_ADDR \
	DT_RISCV_PLIC0_C000000_IRQ_EN_BASE_ADDRESS
#define DT_PLIC_REG_BASE_ADDR	\
	DT_RISCV_PLIC0_C000000_REG_BASE_ADDRESS

/* UART 0 */
#define DT_SIFIVE_UART_0_BASE_ADDR	DT_SIFIVE_UART0_10013000_BASE_ADDRESS
#define DT_SIFIVE_UART_0_CURRENT_SPEED	DT_SIFIVE_UART0_10013000_CURRENT_SPEED
#define DT_SIFIVE_UART_0_IRQ_0		DT_SIFIVE_UART0_10013000_IRQ_0
#define DT_SIFIVE_UART_0_LABEL		DT_SIFIVE_UART0_10013000_LABEL
#define DT_SIFIVE_UART_0_SIZE		DT_SIFIVE_UART0_10013000_SIZE
#define DT_SIFIVE_UART_0_CLK_FREQ		DT_SIFIVE_UART0_10013000_CLOCK_FREQUENCY

/* UART 1 */
#define DT_SIFIVE_UART_1_BASE_ADDR	DT_SIFIVE_UART0_10023000_BASE_ADDRESS
#define DT_SIFIVE_UART_1_CURRENT_SPEED	DT_SIFIVE_UART0_10023000_CURRENT_SPEED
#define DT_SIFIVE_UART_1_IRQ_0		DT_SIFIVE_UART0_10023000_IRQ_0
#define DT_SIFIVE_UART_1_LABEL		DT_SIFIVE_UART0_10023000_LABEL
#define DT_SIFIVE_UART_1_SIZE		DT_SIFIVE_UART0_10023000_SIZE
#define DT_SIFIVE_UART_1_CLK_FREQ		DT_SIFIVE_UART0_10023000_CLOCK_FREQUENCY

