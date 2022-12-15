/*
 * Copyright (C) 2012-2014 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 * Author: Jason Liu <r64343@freescale.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/crm_regs.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <i2c.h>
#include <asm/imx-common/mxc_i2c.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mxc_hdmi.h>
#include "../../../drivers/video/mxcfb.h"
#include <linux/fb.h>
#include <ipu_pixfmt.h>

#include <malloc.h>
#include <asm/processor.h>
#include <asm/mach-types.h>
 


#ifdef CONFIG_FASTBOOT
#include <fastboot.h>
#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#endif
#endif /*CONFIG_FASTBOOT*/

DECLARE_GLOBAL_DATA_PTR;


#define EDIMM_VERSION IMX_GPIO_NR(6, 31)

#define I2C_EXP_RST IMX_GPIO_NR(1, 15)
#define I2C3_STEER  IMX_GPIO_NR(5, 4)
#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_HIGH |               \
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC1_PAD_CTRL USDHC_PAD_CTRL 

#define ENET_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED   |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS |				\
	PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define WEIM_NOR_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST)


#define GPMI_PAD_CTRL0 (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP)
#define GPMI_PAD_CTRL1 (PAD_CTL_DSE_40ohm | PAD_CTL_SPEED_MED | \
			PAD_CTL_SRE_FAST)
#define GPMI_PAD_CTRL2 (GPMI_PAD_CTRL0 | GPMI_PAD_CTRL1)

#define I2C_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#ifdef CONFIG_I2C_MXC
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

/* I2C2 */
struct i2c_pads_info i2c_pad_info1 = {
	/* don't support Sabreauto REVA */
	.scl = {
		.i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | PC,
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | PC,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | PC,
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | PC,
		.gp = IMX_GPIO_NR(4, 13)
	}
};

/* I2C3 */
struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO_5__I2C3_SCL | PC,
		.gpio_mode = MX6_PAD_GPIO_5__GPIO1_IO05 | PC,
		.gp = IMX_GPIO_NR(1, 5)
	},
	.sda = {
		.i2c_mode = MX6_PAD_EIM_D18__I2C3_SDA | PC,
		.gpio_mode = MX6_PAD_EIM_D18__GPIO3_IO18 | PC,
		.gp = IMX_GPIO_NR(3, 18)
	}
};

#endif


enum icore_modul_vers
{
        ICOREQ7_REVMINUS=0,
        ICOREQ7_REVA,

        ICOREQ7_VERS_LAST
};

static char* icore_module_vers_str[] =
{
        "Rev-",
        "RevA",
};

static unsigned int icore_module_vers = ICOREQ7_VERS_LAST-1;

iomux_v3_cfg_t const version_pads[] = {
	MX6_PAD_ENET_TXD1__GPIO1_IO29 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

/*!
 * Get the module revision reading GPIO
 */
void rqs_init_module_version (void)
{
	
	/* config pad iomux */
	imx_iomux_v3_setup_multiple_pads(version_pads, ARRAY_SIZE(version_pads));

	
	if(gpio_get_value(IMX_GPIO_NR(1, 29)) == 0)
		icore_module_vers=ICOREQ7_REVA;
	else
		icore_module_vers=ICOREQ7_REVMINUS;

}

int dram_init(void)
{
	gd->ram_size = (get_ram_size((long *)PHYS_SDRAM, MAX_SDRAM_SIZE));
	return 0;
}

iomux_v3_cfg_t const uart4_pads[] = {
	MX6_PAD_KEY_COL0__UART4_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_KEY_ROW0__UART4_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_ENET_MDC__ENET_MDC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDIO__ENET_MDIO	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),

	/* pin 35 - 1 (PHY_AD2) on reset */
	MX6_PAD_RGMII_RXC__GPIO6_IO30	| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 32 - 1 - (MODE0) all */
	MX6_PAD_RGMII_RD0__GPIO6_IO25	| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 31 - 1 - (MODE1) all */
	MX6_PAD_RGMII_RD1__GPIO6_IO27	| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 28 - 1 - (MODE2) all */
	MX6_PAD_RGMII_RD2__GPIO6_IO28	| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 27 - 1 - (MODE3) all */
	MX6_PAD_RGMII_RD3__GPIO6_IO29	| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 33 - 1 - (CLK125_EN) 125Mhz clockout enabled */
	MX6_PAD_RGMII_RX_CTL__GPIO6_IO24	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_GPIO_0__CCM_CLKO1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO_3__CCM_CLKO2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
};

iomux_v3_cfg_t const enet_reset_revmeno[] = {
	MX6_PAD_ENET_RX_ER__GPIO1_IO24	| MUX_PAD_CTRL(0x48),
};

iomux_v3_cfg_t const enet_reset_reva[] = {
	MX6_PAD_ENET_RXD1__GPIO1_IO26	| MUX_PAD_CTRL(0x48),
};

iomux_v3_cfg_t enet_pads_final[] = {
	MX6_PAD_RGMII_RXC__RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
};

static void setup_iomux_enet(void)
{

	gpio_direction_output(IMX_GPIO_NR(1, 26), 0);
	gpio_direction_output(IMX_GPIO_NR(6, 30), (CONFIG_FEC_MXC_PHYADDR >> 2));
	gpio_direction_output(IMX_GPIO_NR(6, 25), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 27), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 28), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 29), 1);

	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));
  	imx_iomux_v3_setup_multiple_pads(enet_reset_reva, ARRAY_SIZE(enet_reset_reva));

	gpio_direction_output(IMX_GPIO_NR(6, 24), 1);
	printf("ENET board init!!\n");
	/* Need delay 10ms according to KSZ9031 spec */
	udelay(1000 * 10);

  gpio_set_value(IMX_GPIO_NR(1, 26), 1);
	imx_iomux_v3_setup_multiple_pads(enet_pads_final,
					 ARRAY_SIZE(enet_pads_final));
}

iomux_v3_cfg_t const usdhc1_pads[] = {
	/*To avoid pin conflict with NAND, set usdhc1 to 4 pins*/
	MX6_PAD_SD1_CLK__SD1_CLK	| MUX_PAD_CTRL(USDHC1_PAD_CTRL),
	MX6_PAD_SD1_CMD__SD1_CMD	| MUX_PAD_CTRL(USDHC1_PAD_CTRL),
	MX6_PAD_SD1_DAT0__SD1_DATA0	| MUX_PAD_CTRL(USDHC1_PAD_CTRL),
	MX6_PAD_SD1_DAT1__SD1_DATA1	| MUX_PAD_CTRL(USDHC1_PAD_CTRL),
	MX6_PAD_SD1_DAT2__SD1_DATA2	| MUX_PAD_CTRL(USDHC1_PAD_CTRL),
	MX6_PAD_SD1_DAT3__SD1_DATA3	| MUX_PAD_CTRL(USDHC1_PAD_CTRL),

	/*CD pin*/
	MX6_PAD_GPIO_1__GPIO1_IO01 	| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* nPWE_EN */
	MX6_PAD_GPIO_4__GPIO1_IO04 	| MUX_PAD_CTRL(NO_PAD_CTRL),

};

iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

void ldo_mode_set(int ldo_bypass)
{
	printf("i.Core ldo_mode_set removed \n");
}

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
}

#ifdef CONFIG_FSL_ESDHC

#define USDHC1_CD_GPIO	IMX_GPIO_NR(1, 1)

struct fsl_esdhc_cfg usdhc_cfg[3] = {
       {USDHC4_BASE_ADDR, 1},	//eMMC
       {USDHC3_BASE_ADDR, 1},	//SD
       {USDHC1_BASE_ADDR, 1},	//WiFi
};

// Returns:
// 0 for SD card boot
// 1 for eMMC
int mmc_get_env_devno(void)
{
	u32 soc_sbmr = readl(SRC_BASE_ADDR + 0x4);
	u32 dev_no;

	/* BOOT_CFG2[3] and BOOT_CFG2[4] */
	dev_no = (soc_sbmr & 0x00001800) >> 11;

	/* on i.Core M6 RQS
	 */
	switch(dev_no)
	{
		case 2:
			dev_no = 1;	// SD card
			break;
		case 3:
			dev_no = 0;	// eMMC
			break;
		default:			
			printf("Wrong boot device\n");
			dev_no = 2;
			break;			
	}
//	printf("mmc_get_env_devno returns %d\n", dev_no);

	return dev_no;
}

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
		ret = !gpio_get_value(USDHC1_CD_GPIO);
		break;
	case USDHC3_BASE_ADDR:
	case USDHC4_BASE_ADDR:
		ret = 1;
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	int i;
	/*
	* According to the board_mmc_init() the following map is done:
	* (U-boot device node)    (Physical Port)
	* mmc0				USDHC4
	* mmc1				USDHC3
	* mmc2				USDHC1
	*/
	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
	usdhc_cfg[2].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);

	usdhc_cfg[0].max_bus_width = 8;
	usdhc_cfg[1].max_bus_width = 4;
	usdhc_cfg[2].max_bus_width = 4;

	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			break;
		case 2:
			imx_iomux_v3_setup_multiple_pads(
				usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
			gpio_direction_input(USDHC1_CD_GPIO);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) than supported by the board\n", i + 1);
			return 0;
			}

		if (fsl_esdhc_initialize(bis, &usdhc_cfg[i]))
			printf("Warning: failed to initialize mmc dev %d\n", i);
	}

	return 0;
}

void board_late_mmc_env_init(void)
{
	char cmd[32];
	u32 dev_no = mmc_get_env_devno();

	setenv_ulong("mmcdev", dev_no);

	sprintf(cmd, "mmc dev %d", dev_no);
	run_command(cmd, 0);
}

#endif

 

#ifdef CONFIG_CMD_SATA
int setup_sata(void)
{
	struct iomuxc_base_regs *const iomuxc_regs
		= (struct iomuxc_base_regs *) IOMUXC_BASE_ADDR;
	int ret = enable_sata_clock();
	if (ret)
		return ret;

	clrsetbits_le32(&iomuxc_regs->gpr[13],
			IOMUXC_GPR13_SATA_MASK,
			IOMUXC_GPR13_SATA_PHY_8_RXEQ_3P0DB
			|IOMUXC_GPR13_SATA_PHY_7_SATA2M
			|IOMUXC_GPR13_SATA_SPEED_3G
			|(3<<IOMUXC_GPR13_SATA_PHY_6_SHIFT)
			|IOMUXC_GPR13_SATA_SATA_PHY_5_SS_DISABLED
			|IOMUXC_GPR13_SATA_SATA_PHY_4_ATTEN_9_16
			|IOMUXC_GPR13_SATA_PHY_3_TXBOOST_0P00_DB
			|IOMUXC_GPR13_SATA_PHY_2_TX_1P104V
			|IOMUXC_GPR13_SATA_PHY_1_SLOW);

	return 0;
}
#endif

#define MICREL_KSZ9021_EXTREG_CTRL	0xB
#define MICREL_KSZ9021_EXTREG_DATA_WRITE	0xC
#define MICREL_KSZ9021_RGMII_CLK_CTRL_PAD_SCEW	0x104
#define MICREL_KSZ9021_RGMII_RX_DATA_PAD_SCEW	0x105

void ksz9021rn_phy_fixup(struct phy_device *phydev)
{
  printf("ksz9021rn_phy_fixup\n");
  phy_write(phydev, MDIO_DEVAD_NONE, MICREL_KSZ9021_EXTREG_CTRL, 0x8000 | MICREL_KSZ9021_RGMII_RX_DATA_PAD_SCEW);
  phy_write(phydev, MDIO_DEVAD_NONE, MICREL_KSZ9021_EXTREG_DATA_WRITE, 0x0000);

  phy_write(phydev, MDIO_DEVAD_NONE, MICREL_KSZ9021_EXTREG_CTRL, 0x8000 | MICREL_KSZ9021_RGMII_CLK_CTRL_PAD_SCEW);
  phy_write(phydev, MDIO_DEVAD_NONE, MICREL_KSZ9021_EXTREG_DATA_WRITE, 0xf0f0);
    
  phy_write(phydev, MDIO_DEVAD_NONE, MICREL_KSZ9021_EXTREG_CTRL, MICREL_KSZ9021_RGMII_CLK_CTRL_PAD_SCEW);
}

static void mmd_write_reg(struct phy_device *dev, int device, int reg, int val)
{
	phy_write(dev, MDIO_DEVAD_NONE, 0x0d, device);
	phy_write(dev, MDIO_DEVAD_NONE, 0x0e, reg);
	phy_write(dev, MDIO_DEVAD_NONE, 0x0d, (1 << 14) | device);
	phy_write(dev, MDIO_DEVAD_NONE, 0x0e, val);
}

static int ksz9031rn_phy_fixup(struct phy_device *phydev)
{
	//printk("Init ksz9031rn PHY\n");

	//write register 6 addr 2 TXD[0:3] skew
	mmd_write_reg(phydev, 2, 6, 0x4111);

	//write register 5 addr 2 RXD[0:3] skew
	mmd_write_reg(phydev, 2, 5, 0x47a7);

	//write register 4 addr 2 RX_DV TX_EN skew
	mmd_write_reg(phydev, 2, 4, 0x004A);

	//write register 8 addr 2 RX_CLK GTX_CLK skew
	mmd_write_reg(phydev, 2, 8, 0x0273);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
  unsigned short tmp1 = 0;
  unsigned short tmp2 = 0;

  if (miiphy_read("FEC", phydev->addr, MII_PHYSID2, &tmp2) != 0) {
	debug("PHY ID register 3 read failed\n");
  }  
  if (miiphy_read("FEC", phydev->addr, MII_PHYSID1, &tmp1) != 0) {
	debug("PHY ID register 2 read failed\n");
  }  

  unsigned short model = (tmp2>>4) & 0x3F;  
  if (model == 0x21) // KSZ9021
  {
	ksz9021rn_phy_fixup(phydev);
  }
  else if (model == 0x22) // KSZ9031
  {
	ksz9031rn_phy_fixup(phydev);
  }

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

#if defined(CONFIG_VIDEO_IPUV3)
static iomux_v3_cfg_t const rgb_pads[] = {
	MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
	MX6_PAD_DI0_PIN15__IPU1_DI0_PIN15,
	MX6_PAD_DI0_PIN2__IPU1_DI0_PIN02,
	MX6_PAD_DI0_PIN3__IPU1_DI0_PIN03,	        
	MX6_PAD_DISP0_DAT0__IPU1_DISP0_DATA00,
	MX6_PAD_DISP0_DAT1__IPU1_DISP0_DATA01,
	MX6_PAD_DISP0_DAT2__IPU1_DISP0_DATA02,
	MX6_PAD_DISP0_DAT3__IPU1_DISP0_DATA03,
	MX6_PAD_DISP0_DAT4__IPU1_DISP0_DATA04,
	MX6_PAD_DISP0_DAT5__IPU1_DISP0_DATA05,
	MX6_PAD_DISP0_DAT6__IPU1_DISP0_DATA06,
	MX6_PAD_DISP0_DAT7__IPU1_DISP0_DATA07,
	MX6_PAD_DISP0_DAT8__IPU1_DISP0_DATA08,
	MX6_PAD_DISP0_DAT9__IPU1_DISP0_DATA09,
	MX6_PAD_DISP0_DAT10__IPU1_DISP0_DATA10,
	MX6_PAD_DISP0_DAT11__IPU1_DISP0_DATA11,
	MX6_PAD_DISP0_DAT12__IPU1_DISP0_DATA12,
	MX6_PAD_DISP0_DAT13__IPU1_DISP0_DATA13,
	MX6_PAD_DISP0_DAT14__IPU1_DISP0_DATA14,
	MX6_PAD_DISP0_DAT15__IPU1_DISP0_DATA15,
	MX6_PAD_DISP0_DAT16__IPU1_DISP0_DATA16,
	MX6_PAD_DISP0_DAT17__IPU1_DISP0_DATA17,
};

#define RGB_BACKLIGHT_GP IMX_GPIO_NR(4, 20)

struct display_info_t {
	int	bus;
	int	addr;
	int	pixfmt;
	int	(*detect)(struct display_info_t const *dev);
	void	(*enable)(struct display_info_t const *dev);
	struct	fb_videomode mode;
};

static void enable_rgb(struct display_info_t const *dev)
{
	imx_iomux_v3_setup_multiple_pads(
		rgb_pads,
		 ARRAY_SIZE(rgb_pads));
	gpio_direction_output(RGB_BACKLIGHT_GP, 1);
}

static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	int reg = readl(&iomux->gpr[2]);

	reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
		 IOMUXC_GPR2_LVDS_CH1_MODE_MASK);

	writel(reg, &iomux->gpr[2]);
}

static void do_enable_hdmi(struct display_info_t const *dev)
{
	disable_lvds(dev);
	imx_enable_hdmi_phy();
}

struct display_info_t const displays[] = {{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= NULL,
	.enable	= enable_rgb,
	.mode	= {
		.name           = "Amp-WD",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 30000,
		.left_margin    = 30,
		.right_margin   = 30,
		.upper_margin   = 5,
		.lower_margin   = 5,
		.hsync_len      = 64,
		.vsync_len      = 20,
		.sync           = FB_SYNC_CLK_LAT_FALL,
		.vmode          = FB_VMODE_NONINTERLACED
} },{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= do_enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 640,
		.yres           = 480,
		.pixclock       = 39721,
		.left_margin    = 48,
		.right_margin   = 16,
		.upper_margin   = 33,
		.lower_margin   = 10,
		.hsync_len      = 96,
		.vsync_len      = 2,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} } };

size_t display_count = ARRAY_SIZE(displays);


iomux_v3_cfg_t const backlight_pads[] = {
	MX6_PAD_SD4_DAT1__GPIO2_IO09 | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

static void setup_iomux_backlight(void)
{
	imx_iomux_v3_setup_multiple_pads(backlight_pads,
					 ARRAY_SIZE(backlight_pads));
	gpio_direction_output(IMX_GPIO_NR(2, 9), 1);
}

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	setup_iomux_backlight();
	enable_ipu_clock();
	imx_setup_hdmi();
	/* Turn on LDB0,IPU,IPU DI0 clocks */
	reg = __raw_readl(&mxc_ccm->CCGR3);
	reg |=  (MXC_CCM_CCGR3_LDB_DI0_MASK | 0xffff); //TBD MM MP:
	writel(reg, &mxc_ccm->CCGR3);
	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 |MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3<<MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      |(3<<MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<<MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     |IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_HIGH
	     |IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     |IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     |IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     |IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
	     |IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~IOMUXC_GPR3_LVDS0_MUX_CTL_MASK)
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       <<IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}
#endif /* CONFIG_VIDEO_IPUV3 */

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

#define ENET_PHY_RST IMX_GPIO_NR(7, 12)
int board_eth_init(bd_t *bis)
{
	int ret;
//printf("FEC MXC: %s:failed\n", __func__);

	ret = cpu_eth_init(bis);
	if (ret)
		printf("FEC MXC: %s:failed\n", __func__);

	return 0;
}


u32 get_board_rev(void)
{
	return (0);
}

int board_early_init_f(void)
{
	setup_iomux_uart();
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif
	return 0;
}

#define MACH_TYPE_MX6Q_SABRELITE       3769

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	/* For kernel 3.0.35 */
	gd->bd->bi_dram[0].start = PHYS_SDRAM;
	gd->bd->bi_dram[0].size = gd->ram_size;
	gd->bd->bi_arch_number = MACH_TYPE_MX6Q_SABRELITE;
	
	setup_iomux_enet();

	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0", MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	{NULL,   0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_I2C_MXC
	setup_i2c(1, CONFIG_SYS_I2C_SPEED,
			CONFIG_SYS_I2C_SLAVE, &i2c_pad_info1);

	/*setup i2c info 2*/
	setup_i2c(2, CONFIG_SYS_I2C_SPEED,
					CONFIG_SYS_I2C_SLAVE + 1, &i2c_pad_info2);
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	return 0;
}

#ifdef CONFIG_FASTBOOT

void board_fastboot_setup(void)
{
	switch (get_boot_device()) {
#if defined(CONFIG_FASTBOOT_STORAGE_SATA)
	case SATA_BOOT:
		if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "sata");
		if (!getenv("bootcmd"))
			setenv("bootcmd", "booti sata");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_SATA*/
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD3_BOOT:
	case MMC3_BOOT:
		if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc0");
		if (!getenv("bootcmd"))
			setenv("bootcmd", "booti mmc0");
		break;
	case SD4_BOOT:
	case MMC4_BOOT:
		if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc1");
		if (!getenv("bootcmd"))
			setenv("bootcmd", "booti mmc1");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
#if defined(CONFIG_FASTBOOT_STORAGE_NAND)
	case NAND_BOOT:
		if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "nand");
		if (!getenv("fbparts"))
			setenv("fbparts",
			 "16m@16m(boot) 128m@32m(recovery) 810m@160m(android_root)ubifs");
		if (!getenv("bootcmd"))
			setenv("bootcmd",
				"nand read ${loadaddr} ${boot_nand_offset} "
				"${boot_nand_size};booti ${loadaddr}");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_NAND*/
	default:
		printf("unsupported boot devices\n");
		break;
	}
}

#ifdef CONFIG_ANDROID_RECOVERY

#define GPIO_VOL_DN_KEY IMX_GPIO_NR(5, 14)
iomux_v3_cfg_t const recovery_key_pads[] = {
	(MX6_PAD_DISP0_DAT20__GPIO5_IO14 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

int check_recovery_cmd_file(void)
{
	int button_pressed = 0;
	int recovery_mode = 0;

	recovery_mode = recovery_check_and_clean_flag();

	/* Check Recovery Combo Button press or not. */
	imx_iomux_v3_setup_multiple_pads(recovery_key_pads,
		ARRAY_SIZE(recovery_key_pads));

	gpio_direction_input(GPIO_VOL_DN_KEY);

	if (gpio_get_value(GPIO_VOL_DN_KEY) == 0) { /* VOL_DN key is low assert */
		button_pressed = 1;
		printf("Recovery key pressed\n");
	}

	return recovery_mode || button_pressed;
}

void board_recovery_setup(void)
{
	int bootdev = get_boot_device();

	switch (bootdev) {
#if defined(CONFIG_FASTBOOT_STORAGE_SATA)
	case SATA_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery", "booti sata recovery");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_SATA*/
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD1_BOOT:
	case MMC1_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery", "booti mmc0 recovery");
		break;
	case SD3_BOOT:
	case MMC3_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery", "booti mmc1 recovery");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
#if defined(CONFIG_FASTBOOT_STORAGE_NAND)
	case NAND_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
				"nand read ${loadaddr} ${recovery_nand_offset} "
				"${recovery_nand_size};booti ${loadaddr}");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_NAND*/
	default:
		printf("Unsupported bootup device for recovery: dev: %d\n",
			bootdev);
		return;
	}

	printf("setup env for recovery..\n");
	setenv("bootcmd", "run bootcmd_android_recovery");
}
#endif /*CONFIG_ANDROID_RECOVERY*/

#endif /*CONFIG_FASTBOOT*/

int checkboard(void)
{
	rqs_init_module_version();

        //enable SD3 PWR
	gpio_direction_output(IMX_GPIO_NR(1, 4), 0);
	gpio_set_value(IMX_GPIO_NR(1, 4), 0);

#if defined CONFIG_MX6Q
	printf("Board: MX6Q-RQS-i.Core Revision %s\n", icore_module_vers_str[icore_module_vers]);
#elif defined CONFIG_MX6D
	printf("Board: MX6D-RQS-i.Core Revision %s\n", icore_module_vers_str[icore_module_vers]);
#elif defined CONFIG_MX6DL
	printf("Board: MX6DL-RQS-i.Core Revision %s\n", icore_module_vers_str[icore_module_vers]);
#elif defined CONFIG_MX6SOLO
	printf("Board: MX6S-RQS-i.Core Revision %s\n", icore_module_vers_str[icore_module_vers]);
#endif
	return 0;
}

#ifdef CONFIG_IMX_UDC
iomux_v3_cfg_t const otg_udc_pads_revmeno[] = {
	(MX6_PAD_GPIO_1__USBOTG_ID | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

iomux_v3_cfg_t const otg_udc_pads_reva[] = {
	(MX6_PAD_ENET_RX_ER__USBOTG_ID | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

void udc_pins_setting(void)
{
	if(icore_mudule_vers == ICOREQ7_REVMINUS)
	{
		imx_iomux_v3_setup_multiple_pads(otg_udc_pads_revmeno,
			ARRAY_SIZE(otg_udc_pads_revmeno));
		/*set daisy chain for otg_pin_id on 6q. for 6dl, this bit is reserved*/
		imx_iomux_set_gpr_register(1, 13, 1, 0);
	}	
	else
	{
		imx_iomux_v3_setup_multiple_pads(otg_udc_pads_reva,
			ARRAY_SIZE(otg_udc_pads_reva));
		/*set daisy chain for otg_pin_id on 6q. for 6dl, this bit is reserved*/
		imx_iomux_set_gpr_register(1, 13, 0, 0);
	}
}

#endif /*CONFIG_IMX_UDC*/

#ifdef CONFIG_USB_EHCI_MX6
iomux_v3_cfg_t const usb_otg_pads[] = {
	MX6_PAD_ENET_RX_ER__USB_OTG_ID | MUX_PAD_CTRL(NO_PAD_CTRL),
};

int board_ehci_hcd_init(int port)
{
	switch (port) {
	case 0:
		imx_iomux_v3_setup_multiple_pads(usb_otg_pads,
			ARRAY_SIZE(usb_otg_pads));

		/*set daisy chain for otg_pin_id on 6q. for 6dl, this bit is reserved*/
		imx_iomux_set_gpr_register(1, 13, 1, 0);
		break;
	case 1:
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return 1;
	}
	return 0;
}

int board_ehci_power(int port, int on)
{
	switch (port) {
	case 0:
		// TBD
		break;
	case 1:
		if (on)
		{
			gpio_direction_output(IMX_GPIO_NR(1, 6), 0);

			// Clock
			unsigned int *addr = 0x20c8160;
			unsigned int val = 0xA40;
			writel(val, addr);
			udelay(1000 * 10);
			gpio_direction_output(IMX_GPIO_NR(1, 6), 1);
		}
		else
			gpio_direction_output(IMX_GPIO_NR(1, 6), 0);
		break;

	default:
		printf("MXC USB port %d not yet supported\n", port);
		return 1;
	}

	return 0;
}
#endif
