/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __ICOREMX6DLRQS_CONFIG_H
#define __ICOREMX6DLRQS_CONFIG_H

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>

#ifdef CONFIG_MX6SOLO
#define MAX_SDRAM_SIZE		0x40000000  /* Maximum 1GB for i.Core M6S */
#else
#define MAX_SDRAM_SIZE		0x80000000  /* Maximum 2GB for i.Core M6DL/D/Q */
#endif

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#ifdef CONFIG_SPL
#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_MMC_SUPPORT
#include "imx6_spl.h"
#endif

#define CONFIG_MACH_TYPE	3980
#define CONFIG_MXC_UART_BASE	UART2_BASE
#define CONFIG_CONSOLE_DEV		"ttymxc1"
#define CONFIG_MMCROOT			"/dev/mmcblk0p2"

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_AUTO_COMPLETE

#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR

#define CONFIG_CMDLINE_EDITING

#if defined(CONFIG_MX6Q)
#define CONFIG_DEFAULT_FDT_FILE	"icoremx6q-starterkit-rqs.dtb"
#elif (defined(CONFIG_MX6SOLO) || defined(CONFIG_MX6DL))
#define CONFIG_DEFAULT_FDT_FILE	"icoremx6dl-starterkit-rqs.dtb"
#endif

/* NETWORK SETTINGS */
#define CONFIG_SERVERIP		192.168.2.96
#define CONFIG_IPADDR		192.168.2.75
#define CONFIG_NETMASK		255.255.255.0
#define CONFIG_ETHADDR		9C:53:CD:01:21:6A


#define CONFIG_SYS_FSL_USDHC_NUM	3


#if defined(CONFIG_SYS_BOOT_NAND)
	#define CONFIG_BOOTCMD		"bootcmd=run bootcmd_ubi\0"
#elif defined(CONFIG_SYS_BOOT_SATA)
	#define CONFIG_BOOTCMD		"bootcmd=run bootcmd_sata\0"
#elif defined(CONFIG_SYS_BOOT_EMMC)
	#define CONFIG_BOOTCMD		"bootcmd=run bootcmd_emmc\0"
	#define CONFIG_SYS_MMC_ENV_DEV		0
	#define CONFIG_SYS_MMC_ENV_PART		0	/* user partition */
#else
	#define CONFIG_BOOTCMD		"bootcmd=run bootcmd_mmc\0"
	#define CONFIG_SYS_MMC_ENV_DEV		1
	#define CONFIG_SYS_MMC_ENV_PART		0	/* user partition */
#endif



#define EXTRA_OPTION_SOLO	 " cma=128MB "
#define BOOTCMD_MMC_YOCTO	 "run loadfdt; fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} uImage; bootm ${loadaddr} - ${fdt_addr}"
#define YOCTO_BOOTCMD_MMC_ICORE	 "run bootargs_mmc; " BOOTCMD_MMC_YOCTO
#define YOCTO_BOOTCMD_MMC_RQS	 "run bootargs_base; run bootargs_mmc; setenv mmcdev 1; " BOOTCMD_MMC_YOCTO
#define YOCTO_BOOTCMD_EMMC_RQS	 "run bootargs_base; run bootargs_emmc; setenv mmcdev 0; " BOOTCMD_MMC_YOCTO
#define YOCTO_BOOTCMD_NET	 "run bootargs_net; tftp uImage; tftp ${fdt_addr} uImage.dtb; bootm ${loadaddr} - ${fdt_addr}"

/* Common parameter
 * For all modules SODIMM
 */
#define COMMON_PARAMETER 			\
	"netdev=eth0\0" 			\
	"ethprime=FEC0\0" 			\
	"lcd_panel=Amp-WD\0" 			\
	"nfsroot=/nfs_icore\0"			\
	 CONFIG_BOOTCMD				\
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0"											\
	"bootargs_base=setenv bootargs_tmp console=" CONFIG_CONSOLE_DEV ",115200" EXTRA_OPTION_SOLO "video=${video_type},${lcd_panel}\0"			\
	"bootargs_net=run bootargs_base; setenv bootargs ${bootargs_tmp} ${mtdparts} root=/dev/nfs ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" 		\
	"bootcmd_net="  YOCTO_BOOTCMD_NET "\0"															\
	"mmcdev=0\0"																		\
	"mmcpart=1\0"																		\

/* Customized parameter
 * Customized parameter for SODIMM iCore modules
 */
#define	EXTRA_ENV_SETTINGS_ICORE 		\
	COMMON_PARAMETER 			\
	"bootargs_emmc=setenv bootargs ${bootargs_tmp} root=/dev/mmcblk1p2 rootwait rw\0" 						\
	"bootargs_mmc=setenv bootargs ${bootargs_tmp} root=/dev/mmcblk0p2 rootwait rw\0" 						\
	"bootcmd_mmc="   YOCTO_BOOTCMD_MMC_RQS 	"\0"										\
	"bootcmd_emmc="  YOCTO_BOOTCMD_EMMC_RQS	"\0"										\
	"video_type=mxcfb0:dev=ldb,LDB-XGA,if=RGB666 video=mxcfb0:dev=hdmi,1920x1080M@60,if=RGB24\0"				\
	"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" 	\
	"mmcpart=1\0"				\
	"fdt_addr=0x18000000\0" 												\

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
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */:q

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
#define CONFIG_USB_MAX_CONTROLLER_COUNT	1 /* Enabled USB controller number */
#endif


/* Network */

#define CONFIG_PHY_SMSC

#define CONFIG_FEC_MXC
#define CONFIG_MII

#define IMX_FEC_BASE			ENET_BASE_ADDR

#undef CONFIG_FEC_XCV_TYPE
#define CONFIG_FEC_XCV_TYPE             RGMII
#define CONFIG_ETHPRIME                 "FEC"

#undef  CONFIG_FEC_MXC_PHYADDR
#define CONFIG_FEC_MXC_PHYADDR		3

#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
#define CONFIG_PHY_MICREL_KSZ9021

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


#define CONFIG_CMD_BMODE
#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY               1

#define CONFIG_LOADADDR                0x12000000
#define CONFIG_SYS_TEXT_BASE           0x17800000

#define CONFIG_ARP_TIMEOUT     200UL

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2     "> "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END         0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x10800000

#define CONFIG_SYS_LOAD_ADDR           CONFIG_LOADADDR

#define CONFIG_CMDLINE_EDITING
#define CONFIG_STACKSIZE               (128 * 1024)

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS           1
#define PHYS_SDRAM                     MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE          PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)


#define CONFIG_ENV_SIZE			(8 * 1024)

#define CONFIG_BAUDRATE                        115200

/* Framebuffer */
#define CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
/*#define CONFIG_VIDEO_LOGO Abilitare per splashscreen UBOOT*/
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IPUV3_CLK 260000000
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP

#ifndef CONFIG_SPL
#define CONFIG_CI_UDC
#define CONFIG_USBD_HS
#define CONFIG_USB_GADGET_DUALSPEED

#define CONFIG_USB_GADGET
#define CONFIG_CMD_USB_MASS_STORAGE
#define CONFIG_USB_GADGET_DOWNLOAD
#define CONFIG_USB_GADGET_MASS_STORAGE
#define CONFIG_USB_FUNCTION_MASS_STORAGE
#define CONFIG_USB_GADGET_VBUS_DRAW	2

#define CONFIG_G_DNL_VENDOR_NUM		0x0525
#define CONFIG_G_DNL_PRODUCT_NUM	0xa4a5
#define CONFIG_G_DNL_MANUFACTURER	"FSL"
#endif

#endif                         /* __ICOREMX6DLRQS_CONFIG_H */
