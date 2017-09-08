/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Engicam GEAM6UL SOM
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __MX6UL_EVK_CONFIG_H
#define __MX6UL_EVK_CONFIG_H


#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include "mx6_common.h"
#include <asm/imx-common/gpio.h>

#define CONFIG_CONSOLE_DEV		"ttymxc0"

#define CONFIG_SYS_GENERIC_BOARD
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

/* DCDC used on EVK, no PMIC */
#undef CONFIG_LDO_BYPASS_CHECK

/* uncomment for PLUGIN mode support */
/* #define CONFIG_USE_PLUGIN */

/* uncomment for SECURE mode support */
/* #define CONFIG_SECURE_BOOT */

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(16 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_GPIO

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200

#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

/* MMC Configs */
#define CONFIG_FSL_USDHC
#ifdef CONFIG_FSL_USDHC
#define CONFIG_FSL_ESDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC2_BASE_ADDR
#ifdef CONFIG_SYS_USE_NAND
#define CONFIG_SYS_FSL_USDHC_NUM	1	/* NAND Flash version */
#else
#define CONFIG_SYS_FSL_USDHC_NUM	2	/* eMMC version */
#endif

#define CONFIG_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_DOS_PARTITION
#define CONFIG_SUPPORT_EMMC_BOOT /* eMMC specific */
#endif

/* UBI/UBI config options */
#define CONFIG_MTD_DEVICE
#define CONFIG_MTD_PARTITIONS
#define CONFIG_CMD_MTDPARTS
#define CONFIG_RBTREE
#define CONFIG_CMD_UBI
#define CONFIG_CMD_UBIFS
#define CONFIG_LZO

#undef CONFIG_BOOTM_NETBSD
#undef CONFIG_BOOTM_PLAN9
#undef CONFIG_BOOTM_RTEMS

#undef CONFIG_CMD_EXPORTENV
#undef CONFIG_CMD_IMPORTENV

#ifdef CONFIG_CMD_NET
#define CONFIG_CMD_MII
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define CONFIG_FEC_ENET_DEV 0

#if (CONFIG_FEC_ENET_DEV == 0)
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR          0x0
#define CONFIG_FEC_XCV_TYPE             RMII
#elif (CONFIG_FEC_ENET_DEV == 1)
#define IMX_FEC_BASE			ENET2_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR          0x1
#define CONFIG_FEC_XCV_TYPE             RMII
#endif
#define CONFIG_ETHPRIME                 "FEC"

#define CONFIG_PHYLIB
#define CONFIG_PHY_SMSC
#define CONFIG_FEC_DMA_MINALIGN		64
#endif

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1

/* I2C configs */
#ifdef CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_SPEED		100000
#endif

/* Command definition */
#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY		3

#define CONFIG_LOADADDR			0x80800000
#define CONFIG_SYS_TEXT_BASE		0x87800000

#define CONFIG_SYS_MMC_IMG_LOAD_PART	1
#ifdef CONFIG_SYS_BOOT_NAND
#define CONFIG_MFG_NAND_PARTITION "mtdparts=gpmi-nand:64m(boot),16m(kernel),16m(dtb),-(rootfs) "
#else
#define CONFIG_MFG_NAND_PARTITION ""
#endif

/* NETWORK SETTINGS */
#define CONFIG_SERVERIP		192.168.2.96
#define CONFIG_IPADDR		192.168.2.75
#define CONFIG_NETMASK		255.255.255.0
#define CONFIG_ETHADDR		9C:53:CD:01:21:6A

#define BOOTCMD_FROM_EMMC \
  "bootargs_emmc=run bootargs_base; setenv bootargs ${bootargs_tmp} root=/dev/mmcblk${mmcdev}p2 rootwait rw\0" \
	"bootcmd_emmc=setenv mmcdev 1; run bootargs_emmc; run loadfdt; run loaduImage; bootm ${loadaddr} - ${fdt_addr}\0"
#define BOOTCMD_FROM_NAND \
	"bootargs_ubi=run bootargs_base; setenv bootargs ${bootargs_tmp} ${mtdparts} ubi.mtd=3 root=ubi0:rootfs rootfstype=ubifs\0"	\
	"bootcmd_ubi=run bootargs_ubi;nand read ${loadaddr} 0x400000 0x700000;nand read ${fdt_addr} 0xc00000 0x100000;bootm ${loadaddr} - ${fdt_addr} \0"
#define BOOTCMD_FROM_NET	 "run bootargs_net; tftp uImage; tftp ${fdt_addr} uImage.dtb; bootm ${loadaddr} - ${fdt_addr} \0"

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
#elif defined(CONFIG_SYS_BOOT_NET)
	#define CONFIG_BOOTCMD		"bootcmd=run bootcmd_net\0"
#else
	#define CONFIG_BOOTCMD		"bootcmd=run bootcmd_mmc\0"
#endif

#define CONFIG_EXTRA_ENV_SETTINGS \
		"bootargs_base=setenv bootargs_tmp console=" CONFIG_CONSOLE_DEV ",115200 cma=64M video=${video_type},${lcd_panel}\0"			\
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
		"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" 	\
		"netdev=eth0\0" 			\
		"ethprime=FEC0\0" 			\
		"nfsroot=/nfs_icore\0"			\
		"fdt_addr=0x83000000\0"			\
		"fdt_high=0xffffffff\0"

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT		"=> "
#define CONFIG_AUTO_COMPLETE

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x10000000)

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000

#define CONFIG_CMDLINE_EDITING
#define CONFIG_STACKSIZE		SZ_128K

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR
#define MAX_SDRAM_SIZE			0x20000000  /* Maximum 512MB for GEA M6UL */

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE			SZ_8K

#ifdef CONFIG_SYS_BOOT_NAND
#define CONFIG_ENV_IS_IN_NAND
#else
#define CONFIG_ENV_IS_IN_MMC
#endif

#ifdef CONFIG_SYS_USE_NAND
#define CONFIG_CMD_NAND
#define CONFIG_CMD_NAND_TRIMFFS

/* NAND stuff */
#define CONFIG_NAND_MXS
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_BASE		0x40000000
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_ONFI_DETECTION

/* DMA stuff, needed for GPMI/MXS NAND support */
#define CONFIG_APBH_DMA
#define CONFIG_APBH_DMA_BURST
#define CONFIG_APBH_DMA_BURST8
#endif

#ifdef CONFIG_FSL_QSPI
#define CONFIG_QSPI_BASE		QSPI1_BASE_ADDR
#define CONFIG_QSPI_MEMMAP_BASE		QSPI1_ARB_BASE_ADDR

#define	CONFIG_SPI_FLASH
#define	CONFIG_SPI_FLASH_STMICRO
#define	CONFIG_SPI_FLASH_BAR
#define	CONFIG_SF_DEFAULT_BUS		0
#define	CONFIG_SF_DEFAULT_CS		0
#define	CONFIG_SF_DEFAULT_SPEED		40000000
#define	CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
#endif



#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET		(16 * SZ_64K)
#elif defined(CONFIG_ENV_IS_IN_NAND)
#undef CONFIG_ENV_SIZE
#define CONFIG_ENV_OFFSET		(0x1c0000)
#define CONFIG_ENV_SECT_SIZE		(0x20000)
#define CONFIG_ENV_SIZE			CONFIG_ENV_SECT_SIZE
#endif

#define SHOW_ENGICAM_NOTE        "Note:    Geamx6ul default U-Boot\n"

#ifndef CONFIG_SYS_MMC_ENV_DEV
	#define CONFIG_SYS_MMC_ENV_DEV		0   /* USDHC1 */
#endif
#if CONFIG_SYS_MMC_ENV_DEV == 1	/* boot from eMMC */
	#define CONFIG_STR_MMC_DEV "1"
	#define CONFIG_MMCROOT			"/dev/mmcblk1p2"  /* USDHC2 */
	#define CONFIG_DEFAULT_FDT_FILE		"geamx6ul-starterkit-emmc.dtb"
#else
	#define CONFIG_STR_MMC_DEV "0"
	#define CONFIG_MMCROOT			"/dev/mmcblk0p2"  /* USDHC2 */
	#define CONFIG_DEFAULT_FDT_FILE		"geamx6ul-starterkit.dtb"
#endif
#define CONFIG_SYS_MMC_ENV_PART		0	/* user area */

#define CONFIG_CMD_BMODE

#ifdef CONFIG_VIDEO
#define	CONFIG_CFB_CONSOLE
#define	CONFIG_VIDEO_MXS
/*#define	CONFIG_VIDEO_LOGO Abilitare per splashscreen UBOOT*/
#define	CONFIG_VIDEO_SW_CURSOR
#define	CONFIG_VGA_AS_SINGLE_DEVICE
#define	CONFIG_SYS_CONSOLE_IS_IN_ENV
#define	CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define	CONFIG_CMD_BMP
#define	CONFIG_BMP_16BPP
#define	CONFIG_VIDEO_BMP_RLE8
#define CONFIG_VIDEO_BMP_LOGO
#endif

/* USB Configs */
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

#if defined(CONFIG_ANDROID_SUPPORT)
#include "mx6ulevkandroid.h"
#endif

#endif
