/*
 * Copyright (C) 2016 Engicam
 * 
 * based on board/wandboard/spl.c
 * Copyright (C) 2014 Wandboard
 * Author: Tungyi Lin <tungyilin1127@gmail.com>
 *         Richard Hu <hakahu@gmail.com>
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/video.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <spl.h>

DECLARE_GLOBAL_DATA_PTR;

#if defined(CONFIG_SPL_BUILD)
#include <asm/arch/mx6-ddr.h>

/*
 * Driving strength:
 *   0x30 == 40 Ohm
 *   0x28 == 48 Ohm
 */

#define IMX6DQ_DRIVE_STRENGTH		0x30
#define IMX6SDL_DRIVE_STRENGTH		0x28

/* configure MX6Q/DUAL mmdc DDR io registers */
static struct mx6dq_iomux_ddr_regs mx6dq_ddr_ioregs = {
	.dram_sdqs0 = 0x28,
	.dram_sdqs1 = 0x28,
	.dram_sdqs2 = 0x28,
	.dram_sdqs3 = 0x28,
	.dram_sdqs4 = 0x28,
	.dram_sdqs5 = 0x28,
	.dram_sdqs6 = 0x28,
	.dram_sdqs7 = 0x28,
	.dram_dqm0 = 0x28,
	.dram_dqm1 = 0x28,
	.dram_dqm2 = 0x28,
	.dram_dqm3 = 0x28,
	.dram_dqm4 = 0x28,
	.dram_dqm5 = 0x28,
	.dram_dqm6 = 0x28,
	.dram_dqm7 = 0x28,
	.dram_cas = 0x30,
	.dram_ras = 0x30,
	.dram_sdclk_0 = 0x30,
	.dram_sdclk_1 = 0x30,
	.dram_reset = 0x30,
	.dram_sdcke0 = 0x3000,
	.dram_sdcke1 = 0x3000,
	.dram_sdba2 = 0x00000000,
	.dram_sdodt0 = 0x30,
	.dram_sdodt1 = 0x30,
};

/* configure MX6Q/DUAL mmdc GRP io registers */
static struct mx6dq_iomux_grp_regs mx6dq_grp_ioregs = {
	.grp_b0ds = 0x30,
	.grp_b1ds = 0x30,
	.grp_b2ds = 0x30,
	.grp_b3ds = 0x30,
	.grp_b4ds = 0x30,
	.grp_b5ds = 0x30,
	.grp_b6ds = 0x30,
	.grp_b7ds = 0x30,
	.grp_addds = 0x30,
	.grp_ddrmode_ctl = 0x00020000,
	.grp_ddrpke = 0x00000000,
	.grp_ddrmode = 0x00020000,
	.grp_ctlds = 0x30,
	.grp_ddr_type = 0x000c0000,
};

/* configure MX6SOLO/DUALLITE mmdc DDR io registers */
struct mx6sdl_iomux_ddr_regs mx6sdl_ddr_ioregs = {
	.dram_sdclk_0 = 0x30,
	.dram_sdclk_1 = 0x30,
	.dram_cas = 0x30,
	.dram_ras = 0x30,
	.dram_reset = 0x30,
	.dram_sdcke0 = 0x30,	// TO BE TESTED
	.dram_sdcke1 = 0x30,	// TO BE TESTED
	.dram_sdba2 = 0x00000000,
	.dram_sdodt0 = 0x30,
	.dram_sdodt1 = 0x30,
	.dram_sdqs0 = 0x28,
	.dram_sdqs1 = 0x28,
	.dram_sdqs2 = 0x28,
	.dram_sdqs3 = 0x28,
	.dram_sdqs4 = 0x28,
	.dram_sdqs5 = 0x28,
	.dram_sdqs6 = 0x28,
	.dram_sdqs7 = 0x28,
	.dram_dqm0 = 0x28,
	.dram_dqm1 = 0x28,
	.dram_dqm2 = 0x28,
	.dram_dqm3 = 0x28,
	.dram_dqm4 = 0x28,
	.dram_dqm5 = 0x28,
	.dram_dqm6 = 0x28,
	.dram_dqm7 = 0x28,
};

/* configure MX6SOLO/DUALLITE mmdc GRP io registers */
struct mx6sdl_iomux_grp_regs mx6sdl_grp_ioregs = {
	.grp_ddr_type = 0x000c0000,
	.grp_ddrmode_ctl = 0x00020000,
	.grp_ddrpke = 0x00000000,
	.grp_addds = 0x30,
	.grp_ctlds = 0x30,
	.grp_ddrmode = 0x00020000,
	.grp_b0ds = 0x28,
	.grp_b1ds = 0x28,
	.grp_b2ds = 0x28,
	.grp_b3ds = 0x28,
	.grp_b4ds = 0x28,
	.grp_b5ds = 0x28,
	.grp_b6ds = 0x28,
	.grp_b7ds = 0x28,
};

/* mt41j256 */
static struct mx6_ddr3_cfg mt41j256 = {
	.mem_speed = 1066,
	.density = 2,
	.width = 16,
	.banks = 8,
	.rowaddr = 15,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
	.SRT = 0,
};

static struct mx6_mmdc_calibration mx6dq_mmdc_calib = {
	.p0_mpwldectrl0 = 0x000E0009,
	.p0_mpwldectrl1 = 0x0018000E,
	.p1_mpwldectrl0 = 0x00000007,
	.p1_mpwldectrl1 = 0x00000000,
	.p0_mpdgctrl0 = 0x43280334,
	.p0_mpdgctrl1 = 0x031C0314,
	.p1_mpdgctrl0 = 0x4318031C,
	.p1_mpdgctrl1 = 0x030C0258,
	.p0_mprddlctl = 0x3E343A40,
	.p1_mprddlctl = 0x383C3844,
	.p0_mpwrdlctl = 0x40404440,
	.p1_mpwrdlctl = 0x4C3E4446,
};

/* DDR 64bit */
static struct mx6_ddr_sysinfo mem_q = {
	.ddr_type 	= DDR_TYPE_DDR3,
	.dsize		= 2,
	.cs1_mirror	= 0,
	/* config for full 4GB range so that get_mem_size() works */
	.cs_density	= 32,
	.ncs		= 1,
	.bi_on		= 1,
	.rtt_nom	= 2,
	.rtt_wr		= 2,
	.ralat		= 5,
	.walat		= 0,
	.mif3_mode	= 3,
	.rst_to_cke	= 0x23,
	.sde_to_rst	= 0x10,
};

static struct mx6_mmdc_calibration mx6dl_mmdc_calib = {
	.p0_mpwldectrl0 = 0x001F0024,
	.p0_mpwldectrl1 = 0x00110018,
	.p1_mpwldectrl0 = 0x001F0024,
	.p1_mpwldectrl1 = 0x00110018,
	.p0_mpdgctrl0 = 0x4230022C,
	.p0_mpdgctrl1 = 0x02180220,
	.p1_mpdgctrl0 = 0x42440248,
	.p1_mpdgctrl1 = 0x02300238,
	.p0_mprddlctl = 0x44444A48,
	.p1_mprddlctl = 0x46484A42,
	.p0_mpwrdlctl = 0x38383234,
	.p1_mpwrdlctl = 0x3C34362E,
};

/* DDR 64bit 1GB */
static struct mx6_ddr_sysinfo mem_dl = {
	.dsize		= 2,
	.cs1_mirror	= 0,
	/* config for full 4GB range so that get_mem_size() works */
	.cs_density	= 32,
	.ncs		= 1,
	.bi_on		= 1,
	.rtt_nom	= 1,
	.rtt_wr		= 1,
	.ralat		= 5,
	.walat		= 0,
	.mif3_mode	= 3,
	.rst_to_cke	= 0x23,
	.sde_to_rst	= 0x10,
};

/* DDR 32bit 512MB */
static struct mx6_ddr_sysinfo mem_s = {
	.dsize		= 1,
	.cs1_mirror	= 0,
	/* config for full 4GB range so that get_mem_size() works */
	.cs_density	= 32,
	.ncs		= 1,
	.bi_on		= 1,
	.rtt_nom	= 1,
	.rtt_wr		= 1,
	.ralat		= 5,
	.walat		= 0,
	.mif3_mode	= 3,
	.rst_to_cke	= 0x23,
	.sde_to_rst	= 0x10,
};

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0x00003F3F, &ccm->CCGR0);
	writel(0x0030FC00, &ccm->CCGR1);
	writel(0x000FC000, &ccm->CCGR2);
	writel(0x3F300000, &ccm->CCGR3);
	writel(0xFF00F300, &ccm->CCGR4);
	writel(0x0F0000C3, &ccm->CCGR5);
	writel(0x000003CC, &ccm->CCGR6);
}

static void gpr_init(void)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	/* enable AXI cache for VDOA/VPU/IPU */
	writel(0xF00000CF, &iomux->gpr[4]);
	/* set IPU AXI-id0 Qos=0xf(bypass) AXI-id1 Qos=0x7 */
	writel(0x007F007F, &iomux->gpr[6]);
	writel(0x007F007F, &iomux->gpr[7]);
}

static void spl_dram_init(void)
{
	if (is_cpu_type(MXC_CPU_MX6SOLO)) {
		mx6sdl_dram_iocfg(32, &mx6sdl_ddr_ioregs, &mx6sdl_grp_ioregs);
		mx6_dram_cfg(&mem_s, &mx6dl_mmdc_calib, &mt41j256);
	} else if (is_cpu_type(MXC_CPU_MX6DL)) {
		mx6sdl_dram_iocfg(64, &mx6sdl_ddr_ioregs, &mx6sdl_grp_ioregs);
		mx6_dram_cfg(&mem_dl, &mx6dl_mmdc_calib, &mt41j256);
	} else if (is_cpu_type(MXC_CPU_MX6Q)||is_cpu_type(MXC_CPU_MX6D)) {
		mx6dq_dram_iocfg(64, &mx6dq_ddr_ioregs, &mx6dq_grp_ioregs);
		mx6_dram_cfg(&mem_q, &mx6dq_mmdc_calib, &mt41j256);
	}

	udelay(100);
}

void simple_test_ram(void)
{
#ifdef ICORE_TEST_RAM
	int i;
	u32 *ptr;

	// Simple memory test
	printf("Starting memory test\n");
	printf("writing...\n");
	ptr = (u32 *)PHYS_SDRAM;
	for (i=0; i<0x100000; i++)
	{
		*ptr = (u32) i;
		ptr++;	
	}

	printf("reading...\n");
	ptr = (u32 *)PHYS_SDRAM;
	for (i=0; i<0x100000; i++)
	{
		if(*ptr != (u32) i)
		{
			printf("error i=%d...\n", i);
		}
		if(i<16)
			printf("0x%0x ", *ptr);		
		ptr++;	
	}
#endif
}

void board_init_f(ulong dummy)
{
	ccgr_init();

	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	gpr_init();

	/* iomux */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();
	
	/* DDR initialization */
	spl_dram_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	simple_test_ram();

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}
#endif
