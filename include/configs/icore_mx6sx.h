/*
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6SX SABRE-SDB board.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define CONFIG_CONSOLE_DEV		"ttymxc0"

#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include "mx6_common.h"
#include <asm/imx-common/gpio.h>
#define CONFIG_DEFAULT_FDT_FILE "imx6sx-icore.dtb"

#define MAX_SDRAM_SIZE		0x20000000  /* Maximum 512MB for i.Core M6SX */
#define CONFIG_ROM_UNIFIED_SECTIONS
#define CONFIG_SYS_GENERIC_BOARD
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#define CONFIG_DBG_MONITOR

/* uncomment for PLUGIN mode support */
/* #define CONFIG_USE_PLUGIN */

/* #define CONFIG_ARCH_MISC_INIT */

/* uncomment for SECURE mode support */
/* #define CONFIG_SECURE_BOOT */

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(3 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_GPIO

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE

#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

/* MMC Configs */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR	0
#define CONFIG_SYS_FSL_USDHC_NUM	2

#define CONFIG_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_DOS_PARTITION

#define CONFIG_BAUDRATE			115200

#undef CONFIG_BOOTM_NETBSD
#undef CONFIG_BOOTM_PLAN9
#undef CONFIG_BOOTM_RTEMS

#undef CONFIG_CMD_EXPORTENV
#undef CONFIG_CMD_IMPORTENV

#define CONFIG_FEC_MXC
#define CONFIG_MII
#define CONFIG_FEC_ENET_DEV 0


#if (CONFIG_FEC_ENET_DEV == 0)
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR          0x0
#elif (CONFIG_FEC_ENET_DEV == 1)
#define IMX_FEC_BASE			ENET2_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR          0x2
#endif
#define CONFIG_FEC_XCV_TYPE             MII100
#define CONFIG_ETHPRIME                 "FEC"

#define CONFIG_PHYLIB
#define CONFIG_PHY_SMSC
#define CONFIG_FEC_DMA_MINALIGN		64
#define CONFIG_FEC_CLOCK_FROM_ANATOP
#define CONFIG_FEC_CLOCK_25M_REF
#define CONFIG_FEC_MXC_25M_REF_CLK

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1

/* I2C configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED		100000



#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY		1

#define CONFIG_LOADADDR			0x80800000
#define CONFIG_SYS_TEXT_BASE		0x87800000

#define CONFIG_SYS_AUXCORE_BOOTDATA 0x78000000 /* Set to QSPI2 B flash at default */
#ifndef CONFIG_SYS_AUXCORE_FASTUP
#define CONFIG_CMD_BOOTAUX /* Boot M4 by command, disable this when M4 fast up */
#endif

#ifdef CONFIG_CMD_BOOTAUX
#define UPDATE_M4_ENV \
	"m4image=m4_qspi.bin\0" \
	"loadm4image=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${m4image}\0" \
	"update_m4_from_sd=" \
		"if sf probe 1:0; then " \
			"if run loadm4image; then " \
				"setexpr fw_sz ${filesize} + 0xffff; " \
				"setexpr fw_sz ${fw_sz} / 0x10000; "	\
				"setexpr fw_sz ${fw_sz} * 0x10000; "	\
				"sf erase 0x0 ${fw_sz}; " \
				"sf write ${loadaddr} 0x0 ${filesize}; " \
			"fi; " \
		"fi\0" \
	"m4boot=sf probe 1:0; bootaux "__stringify(CONFIG_SYS_AUXCORE_BOOTDATA)"\0"
#else
#define UPDATE_M4_ENV ""
#endif

#define CONFIG_MFG_ENV_SETTINGS \
	"mfgtool_args=setenv bootargs console=${console},${baudrate} " \
		"rdinit=/linuxrc " \
		"g_mass_storage.stall=0 g_mass_storage.removable=1 " \
		"g_mass_storage.idVendor=0x066F g_mass_storage.idProduct=0x37FF "\
		"g_mass_storage.iSerialNumber=\"\" "\
		"\0" \
	"initrd_addr=0x83800000\0" \
	"initrd_high=0xffffffff\0" \
	"bootcmd_mfg=run mfgtool_args;bootm ${loadaddr} ${initrd_addr} ${fdt_addr};\0" \
	
#define CONFIG_EXTRA_ENV_SETTINGS \
    EXTRA_ENV_SETTINGS_ICORE \
    "fdt_addr=0x83000000 \0" \
	"fdt_high=0xffffffff\0"

#define CONFIG_BOOTCOMMAND \
	   "mmc dev ${mmcdev};" \
	   "if mmc rescan; then " \
		   "if run loadbootscript; then " \
			   "run bootscript; " \
		   "else " \
			   "if run loaduimage; then " \
				   "run mmcboot; " \
			   "else run netboot; " \
			   "fi; " \
		   "fi; " \
	   "else run netboot; fi"

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_AUTO_COMPLETE
 
/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x10000)

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000

#define CONFIG_CMDLINE_EDITING
#define CONFIG_STACKSIZE		SZ_128K

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR
/* MP: tolto #define PHYS_SDRAM_SIZE			SZ_1G */

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

#ifdef CONFIG_SYS_AUXCORE_FASTUP
/*#define CONFIG_MXC_RDC*/   /* Disable the RDC temporarily, will enable it in future */
#define CONFIG_ENV_IS_IN_MMC  /* Must disable QSPI driver, because M4 run on QSPI */
#elif defined CONFIG_SYS_BOOT_QSPI
/* MP: #define CONFIG_SYS_USE_QSPI */
/* MP: #define CONFIG_ENV_IS_IN_SPI_FLASH */
#else
/* define CONFIG_SYS_USE_QSPI */  /* Enable the QSPI flash at default */
#ifndef CONFIG_ENV_IS_IN_MMC 
	#define CONFIG_ENV_IS_IN_NAND	
#endif
	#define CONFIG_SYS_USE_NAND
	/* #define CONFIG_ENV_IS_IN_MMC */
	
#endif




#ifdef CONFIG_SYS_USE_QSPI
#define CONFIG_QSPI    /* enable the QUADSPI driver */
#define CONFIG_QSPI_BASE			QSPI2_BASE_ADDR
#define CONFIG_QSPI_MEMMAP_BASE		QSPI2_ARB_BASE_ADDR

#define CONFIG_CMD_SF
#define	CONFIG_SPI_FLASH
#define	CONFIG_SPI_FLASH_SPANSION
#define	CONFIG_SF_DEFAULT_BUS		0
#define	CONFIG_SF_DEFAULT_CS		0
#define	CONFIG_SF_DEFAULT_SPEED		40000000
#define	CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
#endif

#define CONFIG_SYS_MMC_ENV_DEV		0  /*USDHC1*/


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

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET		(8 * SZ_64K)
#elif defined(CONFIG_ENV_IS_IN_NAND)
#undef CONFIG_ENV_SIZE
#define CONFIG_ENV_OFFSET		(0x1c0000)
#define CONFIG_ENV_SECT_SIZE		(0x20000)
#define CONFIG_ENV_SIZE			CONFIG_ENV_SECT_SIZE
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET		(768 * 1024)
#define CONFIG_ENV_SECT_SIZE		(64 * 1024)
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#endif

/* #define CONFIG_SPLASH_SCREEN */

#define SHOW_ENGICAM_NOTE        "Note:    iCore6SX default U-Boot\n"



#define CONFIG_SYS_MMC_ENV_DEV		0   /* USDHC1 */
#define CONFIG_STR_MMC_DEV "0"
#define CONFIG_MMCROOT			"/dev/mmcblk0p2"
 

/* NETWORK SETTINGS */
#define CONFIG_SERVERIP		192.168.2.96
#define CONFIG_IPADDR		192.168.2.75
#define CONFIG_NETMASK		255.255.255.0
#define CONFIG_ETHADDR		9C:53:CD:01:21:6A

#define BOOTCMD_FROM_NAND \
	"bootargs_ubi=run bootargs_base; setenv bootargs ${bootargs_tmp} ${mtdparts} ubi.mtd=3 root=ubi0:rootfs rootfstype=ubifs\0"	\
	"bootcmd_ubi=run bootargs_ubi;nand read ${loadaddr} 0x400000 0x700000;nand read ${fdt_addr} 0xc00000 0x100000;bootm ${loadaddr} - ${fdt_addr} \0"

#if defined(CONFIG_SYS_BOOT_NAND)
	#define CONFIG_BOOTCMD		"bootcmd=run bootcmd_ubi\0"
#else
	#define CONFIG_BOOTCMD		"bootcmd=run bootcmd_mmc\0"
#endif

#define CONFIG_SYS_MMC_ENV_PART		0	/* user area */

#define EXTRA_OPTION_SOLO	 " cma=128MB "
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


#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS   0
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2

#endif				/* __CONFIG_H */
