/*
 * Copyright (C) 2012-2016 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX6QTETRA_CONFIG_H
#define __MX6QTETRA_CONFIG_H

#ifdef CONFIG_SPL
#include "imx6_spl.h"
#endif

#define CONFIG_MACH_TYPE	3980
#define CONFIG_MXC_UART_BASE	UART1_BASE
#define CONSOLE_DEV		"ttymxc0"
#define CONFIG_MMCROOT			"/dev/mmcblk1p2"  /* SDHC2 */

/* This is for the 4GB Prototype 1 board only. Will change to 2GB for Prod. */
#if defined(CONFIG_MX6Q)
#define PHYS_SDRAM_SIZE		(2u * 1024 * 1024 * 1024) // 4u --> 2u for 2GB
/* This is for the 1GB Dual boards. */
#elif defined(CONFIG_MX6D)
#define PHYS_SDRAM_SIZE		(1u * 1024 * 1024 * 1024)
/* This is for the 1GB DualLite boards. */
#elif defined(CONFIG_MX6DL)
#define PHYS_SDRAM_SIZE		(1u * 1024 * 1024 * 1024)
/* This is for the 512MB Solo boards. */
#elif defined(CONFIG_MX6SOLO)
#define PHYS_SDRAM_SIZE		(1u * 512 * 1024 * 1024)
#endif
#include "mx6tetra_common.h"

#define CONFIG_SYS_FSL_USDHC_NUM	2
#ifndef CONFIG_SYS_MMC_ENV_PART
#define CONFIG_SYS_MMC_ENV_PART                0       /* user partition */
#endif

/*
#ifdef CONFIG_SYS_USE_SPINOR
#define CONFIG_SF_DEFAULT_CS   0
#endif
*/

/*
 * imx6 q/dl/solo pcie would be failed to work properly in kernel, if
 * the pcie module is iniialized/enumerated both in uboot and linux
 * kernel.
 * rootcause:imx6 q/dl/solo pcie don't have the reset mechanism.
 * it is only be RESET by the POR. So, the pcie module only be
 * initialized/enumerated once in one POR.
 * Set to use pcie in kernel defaultly, mask the pcie config here.
 * Remove the mask freely, if the uboot pcie functions, rather than
 * the kernel's, are required.
 */
/* #define CONFIG_CMD_PCI */
#ifdef CONFIG_CMD_PCI
#define CONFIG_PCI_SCAN_SHOW
#define CONFIG_PCIE_IMX
#define CONFIG_PCIE_IMX_PERST_GPIO	IMX_GPIO_NR(7, 12)
#define CONFIG_PCIE_IMX_POWER_GPIO	IMX_GPIO_NR(3, 19)
#endif

/* USB Configs */
#ifdef CONFIG_CMD_USB
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0
#define CONFIG_USB_MAX_CONTROLLER_COUNT	1 /* Enabled USB controller number */
#endif

/*#define CONFIG_SPLASH_SCREEN*/
/*#define CONFIG_MXC_EPDC*/

/*
 * SPLASH SCREEN Configs
 */
#if defined(CONFIG_SPLASH_SCREEN) && defined(CONFIG_MXC_EPDC)
	/*
	 * Framebuffer and LCD
	 */
	#define CONFIG_SYS_CONSOLE_IS_IN_ENV
	#undef LCD_TEST_PATTERN
	/* #define CONFIG_SPLASH_IS_IN_MMC			1 */
	#define LCD_BPP					LCD_MONOCHROME
	/* #define CONFIG_SPLASH_SCREEN_ALIGN		1 */

	#define CONFIG_WAVEFORM_BUF_SIZE		0x200000
#endif /* CONFIG_SPLASH_SCREEN && CONFIG_MXC_EPDC */

#endif                         /* __MX6QTETRA_CONFIG_H */

