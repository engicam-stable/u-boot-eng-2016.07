/*
 * Copyright 2014 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6SX Sabresd board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_MX6SX
#define CONFIG_SMARCORE_M6SX
#define MAX_SDRAM_SIZE         0x80000000  /* Maximum 2GB for SmarCore M6SX */
#define CONFIG_FEC_CLOCK_FROM_ANATOP

#include "mx6_common.h"

#ifdef CONFIG_SPL
#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_MMC_SUPPORT
#include "imx6_spl.h"
#endif

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(3 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F

#define CONFIG_MXC_UART
#define CONFIG_CONSOLE_DEV		"ttymxc4"
#define CONFIG_MXC_UART_BASE		UART5_BASE


/* NETWORK SETTINGS */
#define CONFIG_SERVERIP		192.168.2.16
#define CONFIG_IPADDR		192.168.2.75
#define CONFIG_NETMASK		255.255.255.0
#define CONFIG_ETHADDR		9C:53:CD:11:21:6A

#define BOOTCMD_FROM_NAND \
	"bootargs_ubi=run bootargs_base; setenv bootargs ${bootargs_tmp} ${mtdparts} ubi.mtd=3 root=ubi0:rootfs rootfstype=ubifs\0"	\
	"bootcmd_ubi=run bootargs_ubi;nand read ${loadaddr} 0x400000 0x700000;nand read ${fdt_addr} 0xc00000 0x100000;bootm ${loadaddr} - ${fdt_addr} \0"
#define BOOTCMD_FROM_EMMC \
	"bootargs_emmc=run bootargs_base; setenv bootargs ${bootargs_tmp} ${mtdparts} root=/dev/mmcblk${mmcdev}p2 rootwait rw\0" \
	"bootcmd_emmc=setenv mmcdev 2; run bootargs_emmc; run loadfdt; run loaduImage; bootm ${loadaddr} - ${fdt_addr}\0"
#define BOOTCMD_FROM_NET	 "run bootargs_net; tftp uImage; tftp ${fdt_addr} uImage.dtb; bootm ${loadaddr} - ${fdt_addr} \0"

#if defined(CONFIG_SYS_BOOT_EMMC)
	#define CONFIG_BOOTCMD		"bootcmd=run bootcmd_emmc\0"
	#undef BOOTCMD_FROM_NAND
	#define BOOTCMD_FROM_NAND ""
#elif defined(CONFIG_SYS_BOOT_NET)
	#define CONFIG_BOOTCMD		"bootcmd=run bootcmd_net\0"
#else
	#define CONFIG_BOOTCMD		"bootcmd=run bootcmd_mmc\0"
	#undef BOOTCMD_FROM_NAND
	#define BOOTCMD_FROM_NAND ""
#endif

#define CONFIG_EXTRA_ENV_SETTINGS \
		"bootargs_base=setenv bootargs_tmp console=" CONFIG_CONSOLE_DEV ",115200 cma=16M video=${video_type},${lcd_panel}\0"			\
		"bootargs_mmc=run bootargs_base; setenv bootargs ${bootargs_tmp} ${mtdparts} root=/dev/mmcblk${mmcdev}p2 rootwait rw\0" \
		"bootcmd_mmc=setenv mmcdev 0; run bootargs_mmc; run loadfdt; run loaduImage; bootm ${loadaddr} - ${fdt_addr}\0"	\
		"bootargs_net=run bootargs_base; setenv bootargs ${bootargs_tmp} root=/dev/nfs ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" 				\
		"bootcmd_net="  BOOTCMD_FROM_NET	\
		BOOTCMD_FROM_NAND	\
		BOOTCMD_FROM_EMMC	\
		CONFIG_BOOTCMD	\
		"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0"											\
		"mmcpart=1\0"					\
		"loaduImage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} uImage;\0"	\
		"mtdparts=mtdparts=gpmi-nand:4m(boot),8m(kernel),1m(dtb),-(rootfs)\0"		\
		"video_type=mxcfb0:dev=lcd\0"		\
		"lcd_panel=Amp-WD\0" 			\
		"fdt_file= imx6sx-smarcore.dtb \0" 	\
		"netdev=eth0\0" 			\
		"ethprime=FEC0\0" 			\
		"nfsroot=/nfs_icore\0"			\
		"fdt_addr=0x83000000\0"			\
		"fdt_high=0xffffffff\0"


/* Miscellaneous configurable options */
#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x10000)

#define CONFIG_STACKSIZE		SZ_128K

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* MMC Configuration */
#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC4_BASE_ADDR

/* I2C Configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED		  100000


/* Network */
#ifdef CONFIG_CMD_NET
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP

#define CONFIG_FEC_MXC
#define CONFIG_MII
#define CONFIG_FEC_ENET_DEV 0

#if (CONFIG_FEC_ENET_DEV == 0)
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR          0x2
#elif (CONFIG_FEC_ENET_DEV == 1)
#define IMX_FEC_BASE			ENET2_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR          0x3
#endif

#define CONFIG_FEC_XCV_TYPE             RGMII
#define CONFIG_ETHPRIME                 "FEC"

#define CONFIG_PHYLIB

#endif

#ifdef CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS   0
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#endif

#define CONFIG_CMD_PCI
#ifdef CONFIG_CMD_PCI
#define CONFIG_PCI
#define CONFIG_PCI_PNP
#define CONFIG_PCI_SCAN_SHOW
#define CONFIG_PCIE_IMX
#define CONFIG_PCIE_IMX_PERST_GPIO	IMX_GPIO_NR(2, 0)
#define CONFIG_PCIE_IMX_POWER_GPIO	IMX_GPIO_NR(2, 1)
#endif

#define CONFIG_IMX_THERMAL

#ifdef CONFIG_FSL_QSPI
#define CONFIG_SYS_FSL_QSPI_LE
#define CONFIG_SYS_FSL_QSPI_AHB
#ifdef CONFIG_MX6SX_SABRESD_REVA
#define FSL_QSPI_FLASH_SIZE		SZ_16M
#else
#define FSL_QSPI_FLASH_SIZE		SZ_32M
#endif
#define FSL_QSPI_FLASH_NUM		2
#endif

#ifndef CONFIG_SPL_BUILD
#define CONFIG_VIDEO
#ifdef CONFIG_VIDEO
#define CONFIG_CFB_CONSOLE
#define CONFIG_VIDEO_MXS
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_SW_CURSOR
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_CMD_BMP
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_VIDEO_BMP_LOGO
#define MXS_LCDIF_BASE MX6SX_LCDIF1_BASE_ADDR
#endif
#endif

#define CONFIG_ENV_OFFSET		(8 * SZ_64K)
#define CONFIG_ENV_SIZE			SZ_8K
#define CONFIG_ENV_IS_IN_MMC

#define CONFIG_SYS_FSL_USDHC_NUM	3
#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_SYS_MMC_ENV_DEV		2  /*USDHC4*/
#endif

#endif				/* __CONFIG_H */
