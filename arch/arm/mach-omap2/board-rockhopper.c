/*
 * linux/arch/arm/mach-omap2/board-rockhopper.c
 *
 * Rockhopper board
 * (C) Copyright 2011 D&H Global Enterprise, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/opp.h>
#include <linux/spi/spi.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mmc/host.h>

#include <linux/regulator/machine.h>
#include <linux/i2c/twl.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <plat/board.h>
#include <plat/common.h>
#include <video/omapdss.h>
#include <video/omap-panel-generic-dpi.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/usb.h>
#include <plat/omap_device.h>
#include <plat/mcspi.h>

#include "mux.h"
#include "hsmmc.h"
#include "pm.h"
#include "common-board-devices.h"

#include "sdram-micron-mt46h32m32lf-6.h"

static struct mtd_partition rockhopper_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader",
		.offset		= 0,
		.size		= 4 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 14 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x240000 */
		.size		= 2 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 32 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x680000 */
		.size		= MTDPART_SIZ_FULL,
	},
};

/* DSS */

static int rockhopper_enable_dvi(struct omap_dss_device *dssdev)
{
	if (gpio_is_valid(dssdev->reset_gpio))
		gpio_set_value(dssdev->reset_gpio, 1);

	return 0;
}

static void rockhopper_disable_dvi(struct omap_dss_device *dssdev)
{
	if (gpio_is_valid(dssdev->reset_gpio))
		gpio_set_value(dssdev->reset_gpio, 0);
}

static struct panel_generic_dpi_data dvi_panel = {
	.name = "generic",
	.platform_enable = rockhopper_enable_dvi,
	.platform_disable = rockhopper_disable_dvi,
};

static struct omap_dss_device rockhopper_dvi_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "dvi",
	.driver_name = "generic_dpi_panel",
	.data = &dvi_panel,
	.phy.dpi.data_lines = 24,
        .reset_gpio = 170,
};

static struct omap_dss_device rockhopper_tv_device = {
	.name = "tv",
	.driver_name = "venc",
	.type = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type = OMAP_DSS_VENC_TYPE_SVIDEO,
};

static struct omap_dss_device *rockhopper_dss_devices[] = {
	&rockhopper_dvi_device,
	&rockhopper_tv_device,
};

static struct omap_dss_board_info rockhopper_dss_data = {
	.num_devices = ARRAY_SIZE(rockhopper_dss_devices),
	.devices = rockhopper_dss_devices,
	.default_device = &rockhopper_dvi_device,
};

static void __init rockhopper_display_init(void)
{
	int r;

	r = gpio_request_one(rockhopper_dvi_device.reset_gpio, GPIOF_OUT_INIT_LOW,
			     "DVI reset");
	if (r < 0)
		printk(KERN_ERR "Unable to get DVI reset GPIO\n");
}

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_wp	= 29,
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply rockhopper_vmmc1_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.0"),
};

static struct regulator_consumer_supply rockhopper_vsim_supply[] = {
	REGULATOR_SUPPLY("vmmc_aux", "omap_hsmmc.0"),
};

static struct gpio_led gpio_leds[];

static int rockhopper_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio.0 is "mmc0_cd" (input/IRQ) */
	mmc[0].gpio_cd = gpio + 0;
	omap2_hsmmc_init(mmc);

	/* TWL4030_GPIO_MAX + 1 == ledB, PMU_STAT (out, active low LED) */
	gpio_leds[0].gpio = gpio + TWL4030_GPIO_MAX + 1;

	return 0;
}

static struct twl4030_gpio_platform_data rockhopper_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.setup		= rockhopper_twl_gpio_setup,
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data rockhopper_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(rockhopper_vmmc1_supply),
	.consumer_supplies	= &rockhopper_vmmc1_supply,
};

/* VSIM for MMC1 pins DAT4..DAT7 (2 mA, plus card == max 50 mA) */
static struct regulator_init_data rockhopper_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= rockhopper_vsim_supply,
	.consumer_supplies	= &rockhopper_vsim_supply,
};

static struct twl4030_platform_data rockhopper_twldata = {
	/* platform_data for children goes here */
	.gpio		= &rockhopper_gpio_data,
	.vmmc1		= &rockhopper_vmmc1,
	.vsim		= &rockhopper_vsim,
};

static int __init rockhopper_i2c_init(void)
{
	omap3_pmic_get_config(&rockhopper_twldata,
			TWL_COMMON_PDATA_USB | TWL_COMMON_PDATA_AUDIO | TWL_COMMON_PDATA_MADC,
			TWL_COMMON_REGULATOR_VDAC | TWL_COMMON_REGULATOR_VPLL2);
	rockhopper_twldata.vpll2->constraints.name = "VDVI";
	omap3_pmic_init("twl4030", &rockhopper_twldata);

	omap_register_i2c_bus(2, 400, NULL, 0);
	/* Bus 3 is attached to the DVI port where devices like the pico DLP
	 * projector don't work reliably with 400kHz */
	omap_register_i2c_bus(3, 100, NULL, 0);
	return 0;
}

static struct spi_board_info rockhopper_spi_board_info[] __initdata = {
	{
		.modalias		= "spidev",
		.bus_num		= 2,
		.chip_select		= 0,
		.max_speed_hz		= 48000000,
		.mode			= SPI_MODE_0,
	},
};

static int __init rockhopper_spi_init(void)
{
	spi_register_board_info(rockhopper_spi_board_info,
			ARRAY_SIZE(rockhopper_spi_board_info));
	return 0;
}

static struct gpio_led gpio_leds[] = {
	{
		.name			= "rockhopperboard:green:com",
		.default_trigger	= "mmc0",
		.gpio			= -EINVAL,	/* gets replaced */
		.active_low		= true,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static struct gpio_keys_button gpio_buttons[] = {
	{
		.code			= BTN_EXTRA,
		.gpio			= 7,
		.desc			= "user",
		.wakeup			= 1,
	},
};

static struct gpio_keys_platform_data gpio_key_info = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};

static struct platform_device keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_key_info,
	},
};

static void __init rockhopper_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(mt46h32m32lf6_sdrc_params,
				  mt46h32m32lf6_sdrc_params);
}

static void __init rockhopper_init_irq(void)
{
	omap3_init_irq();
}

static struct platform_device *rockhopper_devices[] __initdata = {
	&leds_gpio,
	&keys_gpio,
};

static const struct usbhs_omap_board_data usbhs_bdata __initconst = {

	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,

	.phy_reset  = true,
	.reset_gpio_port[0]  = 27,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static void __init rockhopper_opp_init(void)
{
	if (omap3_opp_init())
		pr_err("%s: opp default init failed\n", __func__);
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
#if defined(CONFIG_USB_MUSB_OTG)
	.mode			= MUSB_OTG,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode			= MUSB_PERIPHERAL,
#else
	.mode			= MUSB_HOST,
#endif
	.power			= 100,
};

static void __init rockhopper_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	rockhopper_i2c_init();
	rockhopper_spi_init();
	platform_add_devices(rockhopper_devices,
			ARRAY_SIZE(rockhopper_devices));
	omap_display_init(&rockhopper_dss_data);
	omap_serial_init();

	usb_musb_init(&musb_board_data);
	usbhs_init(&usbhs_bdata);
	omap_nand_flash_init(NAND_BUSWIDTH_16, rockhopper_nand_partitions,
			     ARRAY_SIZE(rockhopper_nand_partitions));

	/* Ensure msecure is mux'd to be able to set the RTC. */
	omap_mux_init_signal("sys_drm_msecure", OMAP_PIN_OFF_OUTPUT_HIGH);

	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);

	rockhopper_display_init();
	rockhopper_opp_init();
}

MACHINE_START(ROCKHOPPER, "Rockhopper")
	.boot_params	= 0x80000100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= rockhopper_init_early,
	.init_irq	= rockhopper_init_irq,
	.init_machine	= rockhopper_init,
	.timer		= &omap3_secure_timer,
MACHINE_END
