/*
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/io.h>
#include <asm/imx-common/mxc_i2c.h>
#include <linux/sizes.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <i2c.h>
#include <miiphy.h>
#include <netdev.h>
#include <usb.h>
#include <usb/ehci-ci.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE |     \
	PAD_CTL_SPEED_HIGH   |                                   \
	PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST)

#define ENET_CLK_PAD_CTRL  (PAD_CTL_SPEED_LOW | \
	PAD_CTL_DSE_120ohm   | PAD_CTL_SRE_SLOW)

#define ENET_RX_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |          \
	PAD_CTL_SPEED_HIGH   | PAD_CTL_SRE_FAST)

#define ENET_RX_PAD_CTRL_PRE  (PAD_CTL_PKE | PAD_CTL_PUE |          \
	PAD_CTL_SPEED_HIGH   | PAD_CTL_PUS_100K_DOWN | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE)

#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP | PAD_CTL_PUE | \
	PAD_CTL_PKE | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

int dram_init(void)
{
	gd->ram_size = (get_ram_size((long *)PHYS_SDRAM, MAX_SDRAM_SIZE));
	return 0;
}

static iomux_v3_cfg_t const uart5_pads[] = {
	MX6_PAD_KEY_COL3__UART5_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_KEY_ROW3__UART5_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_SD2_CLK__USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CMD__USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DATA0__USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DATA1__USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DATA2__USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DATA3__USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__USDHC3_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__USDHC3_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA0__USDHC3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA1__USDHC3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA2__USDHC3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA3__USDHC3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA4__USDHC3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA5__USDHC3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA6__USDHC3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA7__USDHC3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_NAND_CE1_B__USDHC3_RESET_B | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};


static iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_SD4_CLK__USDHC4_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__USDHC4_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DATA0__USDHC4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DATA1__USDHC4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DATA2__USDHC4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DATA3__USDHC4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DATA4__USDHC4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DATA5__USDHC4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DATA6__USDHC4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DATA7__USDHC4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_SD4_RESET_B__USDHC4_RESET_B | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

#ifdef CONFIG_FEC_MXC
static iomux_v3_cfg_t const fec1_pads[] = {
	MX6_PAD_ENET1_COL__ENET2_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_CRS__ENET2_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII1_RX_CTL__ENET1_RX_EN | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII1_RD0__ENET1_RX_DATA_0 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII1_RD1__ENET1_RX_DATA_1 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII1_RD2__ENET1_RX_DATA_2 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII1_RD3__ENET1_RX_DATA_3 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII1_RXC__ENET1_RX_CLK | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII1_TX_CTL__ENET1_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII1_TD0__ENET1_TX_DATA_0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII1_TD1__ENET1_TX_DATA_1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII1_TD2__ENET1_TX_DATA_2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII1_TD3__ENET1_TX_DATA_3 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII1_TXC__ENET1_RGMII_TXC | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

static iomux_v3_cfg_t const fec2_pads[] = {
	MX6_PAD_ENET1_COL__ENET2_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_CRS__ENET2_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII2_RX_CTL__ENET2_RX_EN | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII2_RD0__ENET2_RX_DATA_0 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII2_RD1__ENET2_RX_DATA_1 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII2_RD2__ENET2_RX_DATA_2 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII2_RD3__ENET2_RX_DATA_3 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII2_RXC__ENET2_RX_CLK | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII2_TX_CTL__ENET2_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII2_TD0__ENET2_TX_DATA_0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII2_TD1__ENET2_TX_DATA_1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII2_TD2__ENET2_TX_DATA_2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII2_TD3__ENET2_TX_DATA_3 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII2_TXC__ENET2_RGMII_TXC | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

static iomux_v3_cfg_t const fec_pre_reset_pads[] = {
    MX6_PAD_RGMII1_RXC__ENET1_RX_CLK | MUX_PAD_CTRL(ENET_RX_PAD_CTRL_PRE),
    MX6_PAD_RGMII2_RXC__ENET2_RX_CLK | MUX_PAD_CTRL(ENET_RX_PAD_CTRL_PRE),
};

static iomux_v3_cfg_t const fec_post_reset_pads[] = {
    MX6_PAD_RGMII1_RXC__ENET1_RX_CLK | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
    MX6_PAD_RGMII2_RXC__ENET2_RX_CLK | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
};


static iomux_v3_cfg_t const phy_control_pads[] = {
	/* Phy 25M Clock */
	MX6_PAD_ENET2_RX_CLK__ENET2_REF_CLK_25M | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),

	/* PHY1 Reset. */
	MX6_PAD_ENET2_CRS__GPIO2_IO_7 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* PHY2 Reset. */
	MX6_PAD_ENET1_MDIO__GPIO2_IO_3 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_fec(int fec_id)
{
	if (0 == fec_id)
		imx_iomux_v3_setup_multiple_pads(fec1_pads, ARRAY_SIZE(fec1_pads));
	else
		imx_iomux_v3_setup_multiple_pads(fec2_pads, ARRAY_SIZE(fec2_pads));
}
#endif

static iomux_v3_cfg_t const peri_3v3_pads[] = {
	MX6_PAD_QSPI1A_DATA0__GPIO4_IO_16 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart5_pads, ARRAY_SIZE(uart5_pads));
}


static int enable_enet_clock(void)
{
	u32 reg = 0;
	s32 timeout = 100000;

	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	/* Enable fec clock */
	reg = readl(&ccm->CCGR1);
	reg |= (0x3 << 10);
	writel(reg, &ccm->CCGR1);

	/* Enable PLLs and set 50MHz for enet */
	reg = readl(ANATOP_BASE_ADDR+0xE0);
	reg &= ~(1<<12);
	writel(reg, ANATOP_BASE_ADDR+0xE0);
	reg &= ~0x03;
	reg |= 1;	/* 50 MHz */
	writel(reg, ANATOP_BASE_ADDR+0xE0);
	reg |= (1<<13);	// BM_ANADIG_PLL_SYS_ENABLE
	while (timeout--) {
		if (readl(ANATOP_BASE_ADDR+0xE0) & (1<<31)/*BM_ANADIG_PLL_SYS_LOCK*/)
			break;
	}
	if (timeout <= 0)
	{
		printf("ENET PLL Not locked!!\n");
		return -EIO;
	}
	reg &= ~(1<<16); //BM_ANADIG_PLL_SYS_BYPASS;
	writel(reg, ANATOP_BASE_ADDR+0xE0);
	return 0 ;
}

static int setup_fec(int fec_id)
{
  printf("SETUP_FEC\n");
	struct iomuxc *iomuxc_gpr_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int ret;

#ifdef CONFIG_FEC_CLOCK_FROM_ANATOP
	if (0 == fec_id)
  {
    printf("STEP 0\n");
		/* Use 125M anatop loopback REF_CLK1 for ENET1, clear gpr1[13], gpr1[17]*/
		clrsetbits_le32(&iomuxc_gpr_regs->gpr[1], IOMUX_GPR1_FEC1_MASK, 0);
  }
	else
		/* Use 125M anatop loopback REF_CLK1 for ENET2, clear gpr1[14], gpr1[18]*/
		clrsetbits_le32(&iomuxc_gpr_regs->gpr[1], IOMUX_GPR1_FEC2_MASK, 0);
#else
	if (0 == fec_id) 
  {
		/* Set gpr1[13], and clear gpr1[17] */
		setbits_le32(&iomuxc_gpr_regs->gpr[1], IOMUX_GPR1_FEC1_CLOCK_MUX2_SEL_MASK);
		clrbits_le32(&iomuxc_gpr_regs->gpr[1], IOMUX_GPR1_FEC1_CLOCK_MUX1_SEL_MASK);
	} 
	else 
  {
		/* Set gpr1[14], and clear gpr1[18] */
		setbits_le32(&iomuxc_gpr_regs->gpr[1], IOMUX_GPR1_FEC2_CLOCK_MUX2_SEL_MASK);
		clrbits_le32(&iomuxc_gpr_regs->gpr[1], IOMUX_GPR1_FEC2_CLOCK_MUX1_SEL_MASK);
	}
#endif
printf("SETUP_FEC1\n");
	imx_iomux_v3_setup_multiple_pads(phy_control_pads, ARRAY_SIZE(phy_control_pads));

printf("SETUP_FEC2\n");
	ret = enable_fec_anatop_clock(fec_id, ENET_125MHZ);
printf("SETUP_FEC3\n");
	if (ret)
		return ret;

	enable_enet_clock();


	imx_iomux_v3_setup_multiple_pads(fec_pre_reset_pads, ARRAY_SIZE(fec_pre_reset_pads));

  /*RXC down for phy address*/
	gpio_direction_output(IMX_GPIO_NR(5, 5) , 0);
	gpio_direction_output(IMX_GPIO_NR(5, 17) , 0);
	gpio_set_value(IMX_GPIO_NR(5, 5), 0);
	gpio_set_value(IMX_GPIO_NR(5, 17), 0);

	/* Reset PHY1 */
	gpio_direction_output(IMX_GPIO_NR(2, 7) , 0);
  
	/* Reset PHY2 */
	gpio_direction_output(IMX_GPIO_NR(2, 3) , 0);

	udelay(500);

	/* Release Reset PHY1 */
	gpio_set_value(IMX_GPIO_NR(2, 7), 1);

	/* Release Reset PHY2 */
	gpio_set_value(IMX_GPIO_NR(2, 3), 1);

	imx_iomux_v3_setup_multiple_pads(fec_post_reset_pads,	ARRAY_SIZE(fec_post_reset_pads));

printf("SETUP_FEC_OFF\n");
	return 0;
}


int board_eth_init(bd_t *bis)
{	
  int ret;

  printf("CONFIG_FEC_ENET_DEV=%d\n", CONFIG_FEC_ENET_DEV);
  setup_iomux_fec(CONFIG_FEC_ENET_DEV);
  
	ret = fecmxc_initialize_multi(bis, CONFIG_FEC_ENET_DEV,
		CONFIG_FEC_MXC_PHYADDR, IMX_FEC_BASE);
	if (ret)
		printf("FEC%d MXC: %s:failed\n", CONFIG_FEC_ENET_DEV, __func__);
  
  return ret;
}

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
/* I2C1 for PMIC */
static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO1_IO00__I2C1_SCL | PC,
		.gpio_mode = MX6_PAD_GPIO1_IO00__GPIO1_IO_0 | PC,
		.gp = IMX_GPIO_NR(1, 0),
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO1_IO01__I2C1_SDA | PC,
		.gpio_mode = MX6_PAD_GPIO1_IO01__GPIO1_IO_1 | PC,
		.gp = IMX_GPIO_NR(1, 1),
	},
};

int power_init_board(void)
{
	return 0;
}

#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)

static iomux_v3_cfg_t const usb_otg_pads[] = {
	/* OGT1 */
	MX6_PAD_GPIO1_IO09__USB_OTG1_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_GPIO1_IO10__ANATOP_OTG1_ID | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* OTG2 */
	MX6_PAD_GPIO1_IO12__USB_OTG2_PWR | MUX_PAD_CTRL(NO_PAD_CTRL)
};

static void setup_usb(void)
{
	imx_iomux_v3_setup_multiple_pads(usb_otg_pads,
					 ARRAY_SIZE(usb_otg_pads));
}

int board_usb_phy_mode(int port)
{
	if (port == 1)
		return USB_INIT_HOST;
	else
		return usb_phy_mode(port);
}

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	if (port > 1)
		return -EINVAL;

	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
				 port * 4);

	/* Set Power polarity */
	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

	return 0;
}
#endif

int board_phy_config(struct phy_device *phydev)
{

	/*
	 * Enable 1.8V(SEL_1P5_1P8_POS_REG) on
	 * Phy control debug reg 0
	 */
#ifndef CONFIG_SMARCORE_M6SX
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	/* rgmii tx clock delay enable */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);
#endif
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

int board_early_init_f(void)
{
	setup_iomux_uart();

	/* Enable PERI_3V3, which is used by SD2, ENET, LVDS, BT */
	imx_iomux_v3_setup_multiple_pads(peri_3v3_pads,
					 ARRAY_SIZE(peri_3v3_pads));
#ifndef CONFIG_SMARCORE_M6SX
	/* Active high for ncp692 */
	gpio_direction_output(IMX_GPIO_NR(4, 16) , 1);
#endif

#ifdef CONFIG_USB_EHCI_MX6
	setup_usb();
#endif

	return 0;
}

static struct fsl_esdhc_cfg usdhc_cfg[CONFIG_SYS_FSL_USDHC_NUM] = {
	{USDHC2_BASE_ADDR, 0, 4},
	{USDHC3_BASE_ADDR, 0, 8},
	{USDHC4_BASE_ADDR, 0, 8},
};

#define USDHC3_CD_GPIO	IMX_GPIO_NR(2, 10)
#define USDHC3_PWR_GPIO	IMX_GPIO_NR(2, 11)
#define USDHC4_CD_GPIO	IMX_GPIO_NR(6, 21)

int board_mmc_get_env_dev(int devno)
{
	return devno - 1;
}

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
		ret = 1; /* Assume uSDHC1 is always present */
		break;
	case USDHC2_BASE_ADDR:
		ret = 1; /* Assume uSDHC2 is always present */
		break;
#if 0
	case USDHC3_BASE_ADDR:
		ret = !gpio_get_value(USDHC3_CD_GPIO);
		break;
	case USDHC4_BASE_ADDR:
		ret = !gpio_get_value(USDHC4_CD_GPIO);
		break;
#else
	case USDHC3_BASE_ADDR:
		ret = 1; /* Assume uSDHC3 is always present */
		break;
	case USDHC4_BASE_ADDR:
		ret = 1; /* Assume uSDHC4 is always present */
		break;
#endif
	}

	return ret;

}

int board_mmc_init(bd_t *bis)
{
#ifndef CONFIG_SPL_BUILD
	int i, ret;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * mmc0                    USDHC2
	 * mmc1                    USDHC3
	 * mmc2                    USDHC4
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
		case 2:
			imx_iomux_v3_setup_multiple_pads(
				usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
			usdhc_cfg[2].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) than supported by the board\n", i + 1);
			return 0;
			}

			ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
			if (ret) {
				printf("Warning: failed to initialize mmc dev %d\n", i);
				return ret;
			}
	}

	return 0;
#else
	struct src *src_regs = (struct src *)SRC_BASE_ADDR;
	u32 val;
	u32 port;

	val = readl(&src_regs->sbmr1);

	if ((val & 0xc0) != 0x40) {
		printf("Not boot from USDHC!\n");
		return -EINVAL;
	}

	port = (val >> 11) & 0x3;
	printf("port %d\n", port);
	switch (port) {
	case 1:
		imx_iomux_v3_setup_multiple_pads(
			usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
		usdhc_cfg[0].esdhc_base = USDHC2_BASE_ADDR;
		break;
	case 2:
		imx_iomux_v3_setup_multiple_pads(
			usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
		gpio_direction_input(USDHC3_CD_GPIO);
		gpio_direction_output(USDHC3_PWR_GPIO, 1);
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
		usdhc_cfg[0].esdhc_base = USDHC3_BASE_ADDR;
		break;
	case 3:
		imx_iomux_v3_setup_multiple_pads(
			usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
		gpio_direction_input(USDHC4_CD_GPIO);
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
		usdhc_cfg[0].esdhc_base = USDHC4_BASE_ADDR;
		break;
	}

	gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
#endif
}

#ifdef CONFIG_FSL_QSPI

#define QSPI_PAD_CTRL1	\
	(PAD_CTL_SRE_FAST | PAD_CTL_SPEED_HIGH | \
	 PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_47K_UP | PAD_CTL_DSE_40ohm)

static iomux_v3_cfg_t const quadspi_pads[] = {
	MX6_PAD_NAND_WP_B__QSPI2_A_DATA_0	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_READY_B__QSPI2_A_DATA_1	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_CE0_B__QSPI2_A_DATA_2	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_CE1_B__QSPI2_A_DATA_3	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_ALE__QSPI2_A_SS0_B		| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_CLE__QSPI2_A_SCLK		| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA07__QSPI2_A_DQS	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA01__QSPI2_B_DATA_0	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA00__QSPI2_B_DATA_1	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_WE_B__QSPI2_B_DATA_2	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_RE_B__QSPI2_B_DATA_3	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA03__QSPI2_B_SS0_B	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA02__QSPI2_B_SCLK	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA05__QSPI2_B_DQS	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
};

int board_qspi_init(void)
{
	/* Set the iomux */
	imx_iomux_v3_setup_multiple_pads(quadspi_pads,
					 ARRAY_SIZE(quadspi_pads));

	/* Set the clock */
	enable_qspi_clk(1);

	return 0;
}
#endif

#ifdef CONFIG_VIDEO_MXS
static iomux_v3_cfg_t const lcd_pads[] = {
	MX6_PAD_LCD1_CLK__LCDIF1_CLK | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_ENABLE__LCDIF1_ENABLE | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_HSYNC__LCDIF1_HSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_VSYNC__LCDIF1_VSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA00__LCDIF1_DATA_0 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA01__LCDIF1_DATA_1 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA02__LCDIF1_DATA_2 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA03__LCDIF1_DATA_3 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA04__LCDIF1_DATA_4 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA05__LCDIF1_DATA_5 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA06__LCDIF1_DATA_6 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA07__LCDIF1_DATA_7 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA08__LCDIF1_DATA_8 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA09__LCDIF1_DATA_9 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA10__LCDIF1_DATA_10 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA11__LCDIF1_DATA_11 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA12__LCDIF1_DATA_12 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA13__LCDIF1_DATA_13 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA14__LCDIF1_DATA_14 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA15__LCDIF1_DATA_15 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA16__LCDIF1_DATA_16 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA17__LCDIF1_DATA_17 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA18__LCDIF1_DATA_18 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA19__LCDIF1_DATA_19 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA20__LCDIF1_DATA_20 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA21__LCDIF1_DATA_21 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA22__LCDIF1_DATA_22 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA23__LCDIF1_DATA_23 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_RESET__GPIO3_IO_27 | MUX_PAD_CTRL(NO_PAD_CTRL),

	/* Use GPIO for Brightness adjustment, duty cycle = period */
	MX6_PAD_SD1_DATA2__GPIO6_IO_4 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static int setup_lcd(void)
{
	enable_lcdif_clock(LCDIF1_BASE_ADDR);

	imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));

	/* Reset the LCD */
	gpio_direction_output(IMX_GPIO_NR(3, 27) , 0);
	udelay(500);
	gpio_direction_output(IMX_GPIO_NR(3, 27) , 1);

	/* Set Brightness to high */
	gpio_direction_output(IMX_GPIO_NR(6, 4) , 1);

	return 0;
}
#endif

int board_init(void)
{
  printf("board_init\n");
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_I2C_MXC
  printf("CONFIG_SYS_I2C_MXC\n");
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info3);  
#endif

#ifdef CONFIG_FSL_QSPI
  printf("CONFIG_FSL_QSPI\n");
	board_qspi_init();
#endif

#ifdef CONFIG_VIDEO_MXS
  printf("CONFIG_VIDEO_MXS\n");
	setup_lcd();
#endif

#ifdef	CONFIG_FEC_MXC
  printf("CONFIG_FEC_MXC=%d\n", CONFIG_FEC_ENET_DEV);
	setup_fec(CONFIG_FEC_ENET_DEV);
#endif

	return 0;
}

int checkboard(void)
{
	puts("Board: MX6SX SABRE SDB\n");

	return 0;
}

#ifdef CONFIG_SPL_BUILD
#include <libfdt.h>
#include <spl.h>
#include <asm/arch/mx6-ddr.h>

const struct mx6sx_iomux_ddr_regs mx6_ddr_ioregs = {
	.dram_dqm0 = 0x00000028,
	.dram_dqm1 = 0x00000028,
	.dram_dqm2 = 0x00000028,
	.dram_dqm3 = 0x00000028,
	.dram_ras = 0x00000020,
	.dram_cas = 0x00000020,
	.dram_odt0 = 0x00000020,
	.dram_odt1 = 0x00000020,
	.dram_sdba2 = 0x00000000,
	.dram_sdcke0 = 0x00003000,
	.dram_sdcke1 = 0x00003000,
	.dram_sdclk_0 = 0x00000030,
	.dram_sdqs0 = 0x00000028,
	.dram_sdqs1 = 0x00000028,
	.dram_sdqs2 = 0x00000028,
	.dram_sdqs3 = 0x00000028,
	.dram_reset = 0x00000020,
};

const struct mx6sx_iomux_grp_regs mx6_grp_ioregs = {
	.grp_addds = 0x00000020,
	.grp_ddrmode_ctl = 0x00020000,
	.grp_ddrpke = 0x00000000,
	.grp_ddrmode = 0x00020000,
	.grp_b0ds = 0x00000028,
	.grp_b1ds = 0x00000028,
	.grp_ctlds = 0x00000020,
	.grp_ddr_type = 0x000c0000,
	.grp_b2ds = 0x00000028,
	.grp_b3ds = 0x00000028,
};

const struct mx6_mmdc_calibration mx6_mmcd_calib = {
	.p0_mpwldectrl0 = 0x00290025,
	.p0_mpwldectrl1 = 0x00220022,
	.p0_mpdgctrl0 = 0x41480144,
	.p0_mpdgctrl1 = 0x01340130,
	.p0_mprddlctl = 0x3C3E4244,
	.p0_mpwrdlctl = 0x34363638,
};

static struct mx6_ddr3_cfg mem_ddr = {
	.mem_speed = 1600,
	.density = 4,
	.width = 32,
	.banks = 8,
	.rowaddr = 15,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
};

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0xFFFFFFFF, &ccm->CCGR0);
	writel(0xFFFFFFFF, &ccm->CCGR1);
	writel(0xFFFFFFFF, &ccm->CCGR2);
	writel(0xFFFFFFFF, &ccm->CCGR3);
	writel(0xFFFFFFFF, &ccm->CCGR4);
	writel(0xFFFFFFFF, &ccm->CCGR5);
	writel(0xFFFFFFFF, &ccm->CCGR6);
	writel(0xFFFFFFFF, &ccm->CCGR7);
}

static void spl_dram_init(void)
{
	struct mx6_ddr_sysinfo sysinfo = {
		.dsize = mem_ddr.width/32,
		.cs_density = 24,
		.ncs = 1,
		.cs1_mirror = 0,
		.rtt_wr = 2,
		.rtt_nom = 2,		/* RTT_Nom = RZQ/2 */
		.walat = 1,		/* Write additional latency */
		.ralat = 5,		/* Read additional latency */
		.mif3_mode = 3,		/* Command prediction working mode */
		.bi_on = 1,		/* Bank interleaving enabled */
		.sde_to_rst = 0x10,	/* 14 cycles, 200us (JEDEC default) */
		.rst_to_cke = 0x23,	/* 33 cycles, 500us (JEDEC default) */
		.ddr_type = DDR_TYPE_DDR3,
	};

	mx6sx_dram_iocfg(mem_ddr.width, &mx6_ddr_ioregs, &mx6_grp_ioregs);
	mx6_dram_cfg(&sysinfo, &mx6_mmcd_calib, &mem_ddr);
}

void board_init_f(ulong dummy)
{
	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();

	/* iomux and setup of i2c */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	/* DDR initialization */
	spl_dram_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}
#endif
