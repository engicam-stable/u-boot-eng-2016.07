/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __ICOREM6DL_CONFIG_H
#define __ICOREM6DL_CONFIG_H

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>

#ifdef CONFIG_MX6SOLO
#define MAX_SDRAM_SIZE		0x40000000  /* Maximum 1GB for i.Core M6S */
#else
#define MAX_SDRAM_SIZE		0x80000000  /* Maximum 2GB for i.Core M6DL/D/Q */
#endif

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO
#define CONFIG_SYS_USE_NAND

#ifdef CONFIG_SPL
#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_MMC_SUPPORT
#include "imx6_spl.h"
#endif

#define CONFIG_MACH_TYPE	3980
#define CONFIG_MXC_UART_BASE	UART4_BASE
#define CONFIG_CONSOLE_DEV		"ttymxc3"

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_AUTO_COMPLETE

#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR

#define CONFIG_CMDLINE_EDITING

#define SHOW_ENGICAM_NOTE        "Note:    iCoreM6 default U-Boot\n"

#if defined(CONFIG_MX6Q)
	#define CONFIG_DEFAULT_FDT_FILE	"icoremx6q-starterkit.dtb"
#elif defined(CONFIG_MX6DL) || defined(CONFIG_MX6SOLO)
	#define CONFIG_DEFAULT_FDT_FILE	"icoremx6dl-starterkit.dtb"
#endif

#ifndef CONFIG_SYS_BOOT_EMMC
	#define CONFIG_SYS_FSL_USDHC_NUM	1
	#define CONFIG_SYS_MMC_ENV_DEV		0   /* USDHC1 */
	#define CONFIG_STR_MMC_DEV "0"
	#define CONFIG_MMCROOT			"/dev/mmcblk0p2"
#else  /* EMMC */
	#define CONFIG_SYS_FSL_USDHC_NUM	2
	#undef CONFIG_SYS_USE_NAND
	#define CONFIG_ENV_IS_IN_MMC
	#define CONFIG_SYS_MMC_ENV_DEV	1  /* USDHC3 */
	#define CONFIG_STR_MMC_DEV "1"
	#define CONFIG_MMCROOT			"/dev/mmcblk1p2"
#endif

/* NETWORK SETTINGS */
#define CONFIG_SERVERIP		192.168.2.96
#define CONFIG_IPADDR		192.168.2.75
#define CONFIG_NETMASK		255.255.255.0
#define CONFIG_ETHADDR		9C:53:CD:01:21:6A

#define BOOTCMD_FROM_EMMC \
  "bootargs_emmc=run bootargs_base; setenv bootargs ${bootargs_tmp} root=/dev/mmcblk${mmcdev}p2 rootwait rw\0" \
	"bootcmd_emmc=setenv mmcdev 1; run bootargs_emmc; run loadfdt; fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} uImage; bootm ${loadaddr} - ${fdt_addr}\0"
#define BOOTCMD_FROM_NAND \
	"bootargs_ubi=run bootargs_base; setenv bootargs ${bootargs_tmp} ${mtdparts} ubi.mtd=3 root=ubi0:rootfs rootfstype=ubifs\0"	\
	"bootcmd_ubi=run bootargs_ubi;nand read ${loadaddr} 0x400000 0x700000;nand read ${fdt_addr} 0xc00000 0x100000;bootm ${loadaddr} - ${fdt_addr} \0"

#ifdef CONFIG_SYS_BOOT_EMMC
	#undef BOOTCMD_FROM_NAND
	#define BOOTCMD_FROM_NAND ""
#else /* For NAND & SDCARD */
	#undef BOOTCMD_FROM_EMMC
	#define BOOTCMD_FROM_EMMC ""
#endif

#if defined(CONFIG_SYS_BOOT_NAND)
	#define CONFIG_BOOTCMD		"bootcmd=run bootcmd_ubi\0"
#elif defined(CONFIG_SYS_BOOT_SATA)
	#define CONFIG_BOOTCMD		"bootcmd=run bootcmd_sata\0"
#elif defined(CONFIG_SYS_BOOT_EMMC)
	#define CONFIG_BOOTCMD		"bootcmd=run bootcmd_emmc\0"
#else
	#define CONFIG_BOOTCMD		"bootcmd=run bootcmd_mmc\0"
#endif

#define CONFIG_SYS_MMC_ENV_PART		0	/* user area */

#define EXTRA_OPTION_SOLO	 " cma=96MB "
#define BOOTCMD_MMC_YOCTO	 "run loadfdt; fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} uImage; bootm ${loadaddr} - ${fdt_addr}"
#define YOCTO_BOOTCMD_MMC_ICORE	 "setenv mmcdev 0; run bootargs_mmc; " BOOTCMD_MMC_YOCTO
#define YOCTO_BOOTCMD_NET	 "run bootargs_net; tftp uImage; tftp ${fdt_addr} uImage.dtb; bootm ${loadaddr} - ${fdt_addr}"

/* Common parameter
 * For all modules SODIMM
 */
#define COMMON_PARAMETER 			\
	"netdev=eth0\0" 			\
	"ethprime=FEC0\0" 			\
	"lcd_panel=Amp-WD\0" 			\
	"nfsroot=/nfs_icore\0"			\
	BOOTCMD_FROM_NAND	\
	BOOTCMD_FROM_EMMC	\
	 CONFIG_BOOTCMD				\
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0"											\
	"bootargs_base=setenv bootargs_tmp console=" CONFIG_CONSOLE_DEV ",115200" EXTRA_OPTION_SOLO "video=${video_type},${lcd_panel}\0"			\
	"bootargs_net=run bootargs_base; setenv bootargs ${bootargs_tmp} ${mtdparts} root=/dev/nfs ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" 		\
	"bootcmd_net="  YOCTO_BOOTCMD_NET "\0"															\
	"mmcpart=1\0"																		\

/* Customized parameter
 * Customized parameter for SODIMM iCore modules
 */
#define	EXTRA_ENV_SETTINGS_ICORE 		\
	COMMON_PARAMETER 			\
	"bootargs_mmc=run bootargs_base; setenv bootargs ${bootargs_tmp} ${mtdparts} root=/dev/mmcblk0p2 rootwait rw\0" 				\
	"mtdparts=mtdparts=gpmi-nand:4m(boot),8m(kernel),1m(dtb),-(rootfs)\0"								\
	"bootcmd_mmc="  YOCTO_BOOTCMD_MMC_ICORE "\0"											\
	"video_type=mxcfb0:dev=lcd\0"													\
	"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" 											\
	"fdt_addr=0x18000000\0" 													\

#include "icorem6_common.h"

/* #define CONFIG_CMD_PCI */
#ifdef CONFIG_CMD_PCI
#define CONFIG_PCI
#define CONFIG_PCI_PNP
#define CONFIG_PCI_SCAN_SHOW
#define CONFIG_PCIE_IMX
#define CONFIG_PCIE_IMX_PERST_GPIO	IMX_GPIO_NR(7, 12)
#define CONFIG_PCIE_IMX_POWER_GPIO	IMX_GPIO_NR(3, 19)
#endif


/* I2C Configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED		  100000

/* USB Configs */
#ifdef CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0
#define CONFIG_USB_MAX_CONTROLLER_COUNT	2 /* Enabled USB controller number */
#endif

/* Network */
#define CONFIG_PHY_SMSC
#define CONFIG_FEC_MXC
#define CONFIG_MII

#define IMX_FEC_BASE			ENET_BASE_ADDR

#undef CONFIG_FEC_XCV_TYPE
#define CONFIG_FEC_XCV_TYPE             RMII
#define CONFIG_ETHPRIME                 "FEC"

#undef  CONFIG_FEC_MXC_PHYADDR
#define CONFIG_FEC_MXC_PHYADDR		0

#define CONFIG_PHYLIB
#define CONFIG_PHY_SMSC

/* NAND stuff */
#ifdef CONFIG_SYS_USE_NAND
	#define CONFIG_NAND_MXS
	#define CONFIG_SYS_MAX_NAND_DEVICE	1
	#define CONFIG_SYS_NAND_BASE		0x40000000
	#define CONFIG_SYS_NAND_5_ADDR_CYCLE
	#define CONFIG_SYS_NAND_ONFI_DETECTION

	/* UBI/UBI config options */
	#define CONFIG_CMD_FLASH
	#define CONFIG_MTD_DEVICE
	#define CONFIG_MTD_PARTITIONS
	#define CONFIG_CMD_MTDPARTS
	#define CONFIG_RBTREE
	#define CONFIG_CMD_UBI
	#define CONFIG_CMD_UBIFS
	#define CONFIG_LZO
	#define CONFIG_CMD_NAND
	#define CONFIG_CMD_NAND_TRIMFFS

	/* DMA stuff, needed for GPMI/MXS NAND support */
	#define CONFIG_APBH_DMA
	#define CONFIG_APBH_DMA_BURST
	#define CONFIG_APBH_DMA_BURST8
#endif

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH
#ifndef CONFIG_SYS_NOSMP
#define CONFIG_SYS_NOSMP
#endif

#undef CONFIG_ENV_OFFSET

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET		(16 * 64 * 1024)
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET		(768 * 1024)
#define CONFIG_ENV_SECT_SIZE		(64 * 1024)
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#elif defined(CONFIG_ENV_IS_IN_FLASH)
#undef CONFIG_ENV_SIZE
#define CONFIG_ENV_SIZE			CONFIG_SYS_FLASH_SECT_SIZE
#define CONFIG_ENV_SECT_SIZE		CONFIG_SYS_FLASH_SECT_SIZE
#define CONFIG_ENV_OFFSET		(4 * CONFIG_SYS_FLASH_SECT_SIZE)
#elif defined(CONFIG_ENV_IS_IN_NAND)
#undef CONFIG_ENV_SIZE
#define CONFIG_ENV_OFFSET		(0x1c0000)
#define CONFIG_ENV_SECT_SIZE		(0x20000)
#define CONFIG_ENV_SIZE			CONFIG_ENV_SECT_SIZE
#elif defined(CONFIG_ENV_IS_IN_SATA)
#define CONFIG_ENV_OFFSET		(768 * 1024)
#define CONFIG_SATA_ENV_DEV		0
#define CONFIG_SYS_DCACHE_OFF /* remove when sata driver support cache */
#endif


#endif                         /* __MX6QSABRESD_CONFIG_H */
