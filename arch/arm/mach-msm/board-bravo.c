/* linux/arch/arm/mach-msm/board-bravo.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation.
 * Author: Dima Zavin <dima@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-msm.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/usb/android_composite.h>
#include <linux/android_pmem.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/input.h>
#include <linux/akm8973.h>
#include <linux/bma150.h>
#include <linux/capella_cm3602.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/tps65023.h>
#include <linux/spi/spi.h>
#include <linux/curcial_oj.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>
#include <../../../drivers/staging/android/timed_gpio.h>

#include <mach/board.h>
#include <mach/board_htc.h>
#include <mach/hardware.h>
#include <mach/atmega_microp.h>
#include <mach/camera.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_iomap.h>
#include <mach/htc_battery.h>
#include <mach/perflock.h>
#include <mach/msm_serial_debugger.h>
#include <mach/system.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_flashlight.h>
#include <mach/vreg.h>
#include <mach/msm_panel.h>
#include <mach/htc_usb.h>
//#include <mach/gpio.h>
#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>
#include <mach/htc_headset_microp.h>

#include "board-bravo.h"
#include "devices.h"
#include "proc_comm.h"
#include "smd_private.h"

#define SMEM_SPINLOCK_I2C      6

#ifdef CONFIG_ARCH_QSD8X50
extern unsigned char *get_bt_bd_ram(void);
#endif

void msm_init_pmic_vibrator(void);
extern void __init bravo_audio_init(void);
#ifdef CONFIG_MICROP_COMMON
void __init bravo_microp_init(void);
#endif

#define SAMSUNG_PANEL		0
/*Bitwise mask for SONY PANEL ONLY*/
#define SONY_PANEL		0x1		/*Set bit 0 as 1 when it is SONY PANEL*/
#define SONY_PWM_SPI		0x2		/*Set bit 1 as 1 as PWM_SPI mode, otherwise it is PWM_MICROP mode*/
#define SONY_GAMMA		0x4		/*Set bit 2 as 1 when panel contains GAMMA table in its NVM*/
#define SONY_RGB666		0x8		/*Set bit 3 as 1 when panel is 18 bit, otherwise it is 16 bit*/

extern int panel_type;

extern ssize_t htc_battery_show_attr(/*struct device *dev,*/
					 struct device_attribute *attr,
					 char *buf);
static struct htc_battery_platform_data htc_battery_pdev_data = {
	.func_show_batt_attr = htc_battery_show_attr,
	.gpio_mbat_in = BRAVO_GPIO_MBAT_IN,
	.gpio_mchg_en_n = BRAVO_GPIO_MCHG_EN_N,
	.gpio_iset = BRAVO_GPIO_ISET,
	.guage_driver = GUAGE_DS2784,
	.charger = LINEAR_CHARGER,
	.m2a_cable_detect = 1,
};

static struct platform_device htc_battery_pdev = {
	.name = "htc_battery",
	.id = -1,
	.dev	= {
		.platform_data = &htc_battery_pdev_data,
	},
};

static int capella_cm3602_power(int pwr_device, uint8_t enable);
static struct microp_function_config microp_functions[] = {
	{
		.name   = "microp_intrrupt",
		.category = MICROP_FUNCTION_INTR,
	},
	{
		.name   = "sdcard-detect",
		.category = MICROP_FUNCTION_SDCARD,
		.int_pin = 1 << 0,
		.mask_r = {0x80, 0x00, 0x00},
	},
	{
		.name   = "oj",
		.category = MICROP_FUNCTION_OJ,
		.int_pin = 1 << 12,
	},
};

static void curcial_oj_shutdown (int	enable)
{
	uint8_t cmd[3];
	memset(cmd, 0x00, sizeof(uint8_t)*3);
	/* microp firmware(v04) non-shutdown by default */
	cmd[2] = 0x20;
	microp_i2c_write(0x90, cmd,	3);
}


static int curcial_oj_poweron(int	on)
{
	uint8_t data[2];
	struct vreg	*oj_power = vreg_get(0, "gp2");
	if (IS_ERR(oj_power)) {
		printk(KERN_ERR"%s:Error power domain\n",__func__);
		return 0;
	}


	if (on) {
		vreg_set_level(oj_power, 2750);
		vreg_enable(oj_power);
		printk(KERN_ERR "%s:OJ	power	enable(%d)\n", __func__, on);
	} else {
	/* for microp firmware(v04) setting*/
		microp_i2c_read(MICROP_I2C_RCMD_VERSION, data, 2);
		if (data[0] < 4) {
			printk("Microp firmware version:%d\n",data[0]);
			return 1;
		}
		vreg_disable(oj_power);
		printk(KERN_ERR "%s:OJ	power	enable(%d)\n", __func__, on);
		}
	return 1;
}
static void curcial_oj_adjust_xy(uint8_t *data, int16_t *mSumDeltaX, int16_t *mSumDeltaY)
{
	int8_t 	deltaX;
	int8_t 	deltaY;


	if (data[2] == 0x80)
		data[2] = 0x81;
	if (data[1] == 0x80)
		data[1] = 0x81;
	if (1) {
		deltaX = (1)*((int8_t) data[2]); /*X=2*/
		deltaY = (-1)*((int8_t) data[1]); /*Y=1*/
	} else {
		deltaX = (-1)*((int8_t) data[1]);
		deltaY = (1)*((int8_t) data[2]);
	}
	*mSumDeltaX += -((int16_t)deltaX);
	*mSumDeltaY += -((int16_t)deltaY);
}
#define BRAVO_MICROP_VER	0x03

static struct curcial_oj_platform_data bravo_oj_data = {
	.oj_poweron = curcial_oj_poweron,
	.oj_shutdown = curcial_oj_shutdown,
	.oj_adjust_xy = curcial_oj_adjust_xy,
	.microp_version = BRAVO_MICROP_VER,
	.mdelay_time = 0,
	.normal_th = 8,
	.xy_ratio = 15,
	.interval = 0,
	.swap = 0,
	.x = 1,
	.y = -1,
	.share_power = false,
	.debugflag = 0,
	.ap_code = false,
	.sht_tbl = {0, 1000, 1250, 1500, 1750, 2000, 3000},
	.pxsum_tbl = {0, 0, 90, 100, 110, 120, 130},
	.degree = 7,
	.Xsteps = {0, 1, 2, 3, 4, 5, 6, 8, 10, 12,
		14, 16, 18, 20, 22, 24, 26, 27, 28, 29,
		9, 9, 9, 9, 9, 9, 9, 9, 9, 9},
	.Ysteps = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
		10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
		9, 9, 9, 9, 9, 9, 9, 9, 9, 9},
	.irq = MSM_uP_TO_INT(12),
};


static struct microp_function_config microp_lightsensor_function = {
	.name = "light_sensor",
	.category = MICROP_FUNCTION_LSENSOR,
	.levels = { 0x04, 0x21, 0x4D, 0xDC, 0x134, 0x18D, 0x1E5, 0x280, 0x340, 0x3FF },
	.channel = 6,
	.int_pin = 1 << 9,
	.golden_adc = 0xC0,
	.ls_power = capella_cm3602_power,
};

static struct lightsensor_platform_data lightsensor_data = {
	.config = &microp_lightsensor_function,
	.irq = MSM_uP_TO_INT(9),
};

static struct microp_led_config led_config[] = {
	{
		.name = "amber",
		.type = LED_RGB,
	},
	{
		.name = "green",
		.type = LED_RGB,
	},
	{
		.name = "blue",
		.type = LED_GPO,
		.mask_w = {0x00, 0x02, 0x00},
	},
	{
		.name = "button-backlight",
		.type = LED_PWM,
		.mask_w = {0x02, 0x00, 0x00},
		.led_pin = 1 << 2,
		.init_value = 0xFF,
		.fade_time = 5,
	},
};

static struct microp_led_platform_data microp_leds_data = {
	.num_leds	= ARRAY_SIZE(led_config),
	.led_config	= led_config,
};

static struct bma150_platform_data bravo_g_sensor_pdata = {
	.microp_new_cmd = 1,
	.chip_layout = 1,
};

static struct htc_headset_microp_platform_data htc_headset_microp_data = {
	.hpin_int		= 1 << 2,
	.hpin_irq		= MSM_uP_TO_INT(2),
	.hpin_mask		= {0x00, 0x00, 0x04},
	.remote_int		= 1 << 7,
	.remote_irq		= MSM_uP_TO_INT(7),
	.remote_enable_pin	= 0,
	.adc_channel		= 0x07,
	.adc_remote		= {0, 31, 43, 98, 129, 192},
};

static struct platform_device microp_devices[] = {
	{
		.name = "lightsensor_microp",
		.dev = {
			.platform_data = &lightsensor_data,
		},
	},
	{
		.name = "leds-microp",
		.id = -1,
		.dev = {
			.platform_data = &microp_leds_data,
		},
	},
	{
		.name = BMA150_G_SENSOR_NAME,
		.dev = {
			.platform_data = &bravo_g_sensor_pdata,
		},
	},
	{
		.name = CURCIAL_OJ_NAME,
		.id = -1,
		.dev = {
			.platform_data	= &bravo_oj_data,
		},
	},
	{
		.name	= "HTC_HEADSET_MICROP",
		.id	= -1,
		.dev	= {
			.platform_data	= &htc_headset_microp_data,
		},
	},
};

static struct microp_i2c_platform_data microp_data = {
	.num_functions   = ARRAY_SIZE(microp_functions),
	.microp_function = microp_functions,
	.num_devices = ARRAY_SIZE(microp_devices),
	.microp_devices = microp_devices,
	.gpio_reset = BRAVO_GPIO_UP_RESET_N,
	.spi_devices = SPI_OJ | SPI_GSENSOR,
};

static uint opt_usb_h2w_sw;
module_param_named(usb_h2w_sw, opt_usb_h2w_sw, uint, 0);

static uint32_t usb_phy_3v3_table[] = {
	PCOM_GPIO_CFG(BRAVO_USB_PHY_3V3_ENABLE, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA)
};

static uint32_t usb_ID_PIN_input_table[] = {
	PCOM_GPIO_CFG(BRAVO_GPIO_USB_ID_PIN, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_USB_ID1_PIN, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA),
};

static uint32_t usb_ID_PIN_ouput_table[] = {
	PCOM_GPIO_CFG(BRAVO_GPIO_USB_ID_PIN, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),
};

void config_bravo_usb_id_gpios(bool output)
{
	if (output) {
		config_gpio_table(usb_ID_PIN_ouput_table, ARRAY_SIZE(usb_ID_PIN_ouput_table));
		gpio_set_value(BRAVO_GPIO_USB_ID_PIN, 1);
		printk(KERN_INFO "%s %d output high\n",  __func__, BRAVO_GPIO_USB_ID_PIN);
	} else {
		config_gpio_table(usb_ID_PIN_input_table, ARRAY_SIZE(usb_ID_PIN_input_table));
		printk(KERN_INFO "%s %d input none pull\n",  __func__, BRAVO_GPIO_USB_ID_PIN);
	}
}
static int bravo_phy_init_seq[] = {
	0x0C, 0x31,
	0x31, 0x32,
	0x1D, 0x0D,
	0x1D, 0x10,
	-1 };

void bravo_usb_phy_reset(void)
{
	printk(KERN_INFO "%s\n", __func__);
	msm_hsusb_8x50_phy_reset();
	if (usb_phy_error) {
		printk(KERN_INFO "%s: power cycle usb phy\n",
			__func__);
		gpio_set_value(BRAVO_USB_PHY_3V3_ENABLE, 0);
		mdelay(10);
		gpio_set_value(BRAVO_USB_PHY_3V3_ENABLE, 1);
		mdelay(10);
		msm_hsusb_8x50_phy_reset();
	}
}

#ifdef CONFIG_USB_ANDROID
static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq		= bravo_phy_init_seq,
	.phy_reset		= bravo_usb_phy_reset,
	.usb_id_pin_gpio =  BRAVO_GPIO_USB_ID_PIN,
	.config_usb_id_gpios = config_bravo_usb_id_gpios,
	.accessory_detect = 1, /* detect by ID pin gpio */
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "HTC",
	.product	= "Android Phone",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x0bb4,
	.product_id	= 0x0c87,
	.version	= 0x0100,
	.product_name		= "Android Phone",
	.manufacturer_name	= "HTC",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
static void bravo_add_usb_devices(void)
{
	android_usb_pdata.products[0].product_id =
		android_usb_pdata.product_id;
	android_usb_pdata.serial_number = board_serialno();
	msm_hsusb_pdata.serial_number = board_serialno();
	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	config_bravo_usb_id_gpios(0);
	config_gpio_table(usb_phy_3v3_table, ARRAY_SIZE(usb_phy_3v3_table));
	gpio_set_value(BRAVO_USB_PHY_3V3_ENABLE, 1);
	platform_device_register(&msm_device_hsusb);
	platform_device_register(&usb_mass_storage_device);
	platform_device_register(&android_usb_device);
}
#endif

static struct platform_device bravo_rfkill = {
	.name = "bravo_rfkill",
	.id = -1,
};

static struct vreg *vreg_lcm_rftx_2v6;
static struct vreg *vreg_lcm_aux_2v6;

static struct resource qsd_spi_resources[] = {
	{
		.name   = "spi_irq_in",
		.start  = INT_SPI_INPUT,
		.end    = INT_SPI_INPUT,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_out",
		.start  = INT_SPI_OUTPUT,
		.end    = INT_SPI_OUTPUT,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_err",
		.start  = INT_SPI_ERROR,
		.end    = INT_SPI_ERROR,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_base",
		.start  = 0xA1200000,
		.end    = 0xA1200000 + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "spi_clk",
		.start  = 17,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_mosi",
		.start  = 18,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_miso",
		.start  = 19,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_cs0",
		.start  = 20,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_pwr",
		.start  = 21,
		.end    = 0,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_cs0",
		.start  = 22,
		.end    = 0,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device qsd_device_spi = {
	.name           = "spi_qsd",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(qsd_spi_resources),
	.resource       = qsd_spi_resources,
};

static struct resource msm_kgsl_resources[] = {
	{
		.name	= "kgsl_reg_memory",
		.start	= MSM_GPU_REG_PHYS,
		.end	= MSM_GPU_REG_PHYS + MSM_GPU_REG_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "kgsl_phys_memory",
		.start	= MSM_GPU_MEM_BASE,
		.end	= MSM_GPU_MEM_BASE + MSM_GPU_MEM_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_GRAPHICS,
		.end	= INT_GRAPHICS,
		.flags	= IORESOURCE_IRQ,
	},
};

#define PWR_RAIL_GRP_CLK		8
static int bravo_kgsl_power_rail_mode(int follow_clk)
{
	int mode = follow_clk ? 0 : 1;
	int rail_id = PWR_RAIL_GRP_CLK;

	return msm_proc_comm(PCOM_CLKCTL_RPC_RAIL_CONTROL, &rail_id, &mode);
}

static int bravo_kgsl_power(bool on)
{
	int cmd;
	int rail_id = PWR_RAIL_GRP_CLK;

	cmd = on ? PCOM_CLKCTL_RPC_RAIL_ENABLE : PCOM_CLKCTL_RPC_RAIL_DISABLE;
	return msm_proc_comm(cmd, &rail_id, NULL);
}

static struct platform_device msm_kgsl_device = {
	.name		= "kgsl",
	.id		= -1,
	.resource	= msm_kgsl_resources,
	.num_resources	= ARRAY_SIZE(msm_kgsl_resources),
};

static struct android_pmem_platform_data mdp_pmem_pdata = {
	.name		= "pmem",
	.start		= MSM_PMEM_MDP_BASE,
	.size		= MSM_PMEM_MDP_SIZE,
	.no_allocator	= 0,
	.cached		= 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name		= "pmem_adsp",
	.start		= MSM_PMEM_ADSP_BASE,
	.size		= MSM_PMEM_ADSP_SIZE,
	.no_allocator	= 0,
	.cached		= 1,
};


static struct android_pmem_platform_data android_pmem_venc_pdata = {
	.name		= "pmem_venc",
	.start		= MSM_PMEM_VENC_BASE,
	.size		= MSM_PMEM_VENC_SIZE,
	.no_allocator	= 0,
	.cached		= 1,
};

static struct platform_device android_pmem_mdp_device = {
	.name		= "android_pmem",
	.id		= 0,
	.dev		= {
		.platform_data = &mdp_pmem_pdata
	},
};

static struct platform_device android_pmem_adsp_device = {
	.name		= "android_pmem",
	.id		= 4,
	.dev		= {
		.platform_data = &android_pmem_adsp_pdata,
	},
};

static struct platform_device android_pmem_venc_device = {
	.name		= "android_pmem",
	.id		= 5,
	.dev		= {
		.platform_data = &android_pmem_venc_pdata,
	},
};

static struct resource ram_console_resources[] = {
	{
		.start	= MSM_RAM_CONSOLE_BASE,
		.end	= MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name		= "ram_console",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ram_console_resources),
	.resource	= ram_console_resources,
};

static int bravo_ts_power(int on)
{
	pr_info("%s: power %d\n", __func__, on);

	if (on) {
		/* level shifter should be off */
		gpio_set_value(BRAVO_GPIO_TP_EN, 1);
		msleep(120);
		/* enable touch panel level shift */
		gpio_set_value(BRAVO_GPIO_TP_LS_EN, 1);
		msleep(3);
	} else {
		gpio_set_value(BRAVO_GPIO_TP_LS_EN, 0);
		gpio_set_value(BRAVO_GPIO_TP_EN, 0);
		udelay(50);
	}

	return 0;
}

static struct synaptics_i2c_rmi_platform_data bravo_ts_data[] = {
	{
		.version = 0x0100,
		.power = bravo_ts_power,
		.sensitivity_adjust = 12,
		.flags = SYNAPTICS_FLIP_Y  | SYNAPTICS_SNAP_TO_INACTIVE_EDGE,
		.inactive_left = -1 * 0x10000 / 480,
		.inactive_right = -1 * 0x10000 / 480,
		.inactive_top = -5 * 0x10000 / 800,
		.inactive_bottom = -5 * 0x10000 / 800,
		.display_width = 480,
		.display_height = 800,
		.dup_threshold = 10,
		.margin_inactive_pixel = {8, 32, 32, 8},
	},
};

static struct regulator_consumer_supply tps65023_dcdc1_supplies[] = {
	{
		.supply = "acpu_vcore",
	},
};

static struct regulator_init_data tps65023_data[5] = {
	{
		.constraints = {
			.name = "dcdc1", /* VREG_MSMC2_1V29 */
			.min_uV = BRAVO_TPS65023_MIN_UV_MV * 1000,
			.max_uV = BRAVO_TPS65023_MAX_UV_MV * 1000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		},
		.consumer_supplies = tps65023_dcdc1_supplies,
		.num_consumer_supplies = ARRAY_SIZE(tps65023_dcdc1_supplies),
	},
	/* dummy values for unused regulators to not crash driver: */
	{
		.constraints = {
			.name = "dcdc2", /* VREG_MSMC1_1V26 */
			.min_uV = 1260000,
			.max_uV = 1260000,
		},
	},
	{
		.constraints = {
			.name = "dcdc3", /* unused */
			.min_uV = 800000,
			.max_uV = 3300000,
		},
	},
	{
		.constraints = {
			.name = "ldo1", /* unused */
			.min_uV = 1000000,
			.max_uV = 3150000,
		},
	},
	{
		.constraints = {
			.name = "ldo2", /* V_USBPHY_3V3 */
			.min_uV = 3300000,
			.max_uV = 3300000,
		},
	},
};

static struct htc_headset_mgr_platform_data htc_headset_mgr_data = {
};

static struct platform_device htc_headset_mgr = {
	.name	= "HTC_HEADSET_MGR",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_mgr_data,
	},
};

static struct htc_headset_gpio_platform_data htc_headset_gpio_data = {
	.hpin_gpio		= 0,
	.key_enable_gpio	= BRAVO_GPIO_35MM_KEY_INT_SHUTDOWN,
	.mic_select_gpio	= 0,
};

static struct platform_device htc_headset_gpio = {
	.name	= "HTC_HEADSET_GPIO",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_gpio_data,
	},
};

static uint32_t key_int_shutdown_gpio_table[] = {
	PCOM_GPIO_CFG(BRAVO_GPIO_35MM_KEY_INT_SHUTDOWN, 0,
		GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
};

static void headset_init(void)
{
	config_gpio_table(key_int_shutdown_gpio_table,
		ARRAY_SIZE(key_int_shutdown_gpio_table));
	gpio_set_value(BRAVO_GPIO_35MM_KEY_INT_SHUTDOWN, 0);
	return;
}

static struct akm8973_platform_data compass_platform_data = {
	.layouts = BRAVO_LAYOUTS,
	.project_name = BRAVO_PROJECT_NAME,
	.reset = BRAVO_GPIO_COMPASS_RST_N,
	.intr = BRAVO_GPIO_COMPASS_INT,
};

static void ds2482_set_slp_n(unsigned n)
{
	gpio_direction_output(BRAVO_GPIO_DQ_PWRDN_N, n);
}
static struct i2c_board_info i2c_devices[] = {
	{
		I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME, 0x40),
		.platform_data = &bravo_ts_data,
		.irq = MSM_GPIO_TO_INT(BRAVO_GPIO_TP_INT_N)
	},
	{
		I2C_BOARD_INFO(MICROP_I2C_NAME, 0xCC >> 1),
		.platform_data = &microp_data,
		.irq = MSM_GPIO_TO_INT(BRAVO_GPIO_UP_INT_N)
	},
	{
		I2C_BOARD_INFO("ds2482", 0x30 >> 1),
		.platform_data = ds2482_set_slp_n,
		//.irq = MSM_GPIO_TO_INT(PASSION_GPIO_UP_INT_N)
	},
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(BRAVO_GPIO_COMPASS_INT),
	},
#ifdef CONFIG_MSM_CAMERA
#ifdef CONFIG_S5K3E2FX
	{
		I2C_BOARD_INFO("s5k3e2fx", 0x20 >> 1),
	},
#endif
#endif/*CONIFIG_MSM_CAMERA*/
	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.platform_data = tps65023_data,
	},
};

#ifdef CONFIG_ARCH_QSD8X50
static char bdaddress[20];

static void bt_export_bd_address(void)
 {
        unsigned char cTemp[6];

        memcpy(cTemp, get_bt_bd_ram(), 6);
	sprintf(bdaddress, "%02x:%02x:%02x:%02x:%02x:%02x", cTemp[0], cTemp[1],cTemp[2],cTemp[3],cTemp[4],cTemp[5]);
        printk(KERN_INFO "YoYo--BD_ADDRESS=%s\n", bdaddress);
}

module_param_string(bdaddress, bdaddress, sizeof(bdaddress), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bdaddress, "BT MAC ADDRESS");
#endif

static uint32_t camera_off_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(0, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(2, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* PCLK */
	PCOM_GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* HSYNC */
	PCOM_GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* VSYNC */
	PCOM_GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(0, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT1 */
	PCOM_GPIO_CFG(2, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_16MA), /* PCLK */
	PCOM_GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* HSYNC */
	PCOM_GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* VSYNC */
/*	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_16MA),*/ /* MCLK */
	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* MCLK */
	/*steven yeh: modify MCLK driving strength to avoid overshot issue*/
};

void bravo_config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

void bravo_config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}


static struct resource msm_camera_resources[] = {
	{
		.start	= MSM_VFE_PHYS,
		.end	= MSM_VFE_PHYS + MSM_VFE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		 INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = bravo_config_camera_on_gpios,
	.camera_gpio_off = bravo_config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static int flashlight_control(int mode)
{
	return aat1271_flashlight_control(mode);
}

static struct camera_flash_cfg msm_camera_sensor_flash_cfg = {
	.camera_flash		= flashlight_control,
	.num_flash_levels	= FLASHLIGHT_NUM,
	.low_temp_limit		= 5,
	.low_cap_limit		= 15,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k3e2fx_data = {
	.sensor_name    = "s5k3e2fx",
	.sensor_reset	= 144, /* CAM1_RST */
	.sensor_pwd	= 143,  /* CAM1_PWDN, enabled in a9 */
	.pdata		= &msm_camera_device_data,
	.resource	= msm_camera_resources,
	.num_resources	= ARRAY_SIZE(msm_camera_resources),
	.flash_cfg	= &msm_camera_sensor_flash_cfg,
};

static struct platform_device msm_camera_sensor_s5k3e2fx = {
	.name		= "msm_camera_s5k3e2fx",
	.dev		= {
	.platform_data = &msm_camera_sensor_s5k3e2fx_data,
	},
};


static int __capella_cm3602_power(int on)
{
	printk(KERN_DEBUG "%s: Turn the capella_cm3602 power %s\n",
		__func__, (on) ? "on" : "off");
	if (on) {
		gpio_direction_output(BRAVO_GPIO_LS_EN, 0);
		gpio_direction_output(BRAVO_GPIO_PROXIMITY_EN, 1);
	} else {
		gpio_direction_output(BRAVO_GPIO_LS_EN, 1);
	}
	return 0;
}

static DEFINE_MUTEX(capella_cm3602_lock);
static int als_power_control;

static int capella_cm3602_power(int pwr_device, uint8_t enable)
{
	/* TODO eolsen Add Voltage reg control */
	unsigned int old_status = 0;
	int ret = 0, on = 0;
	mutex_lock(&capella_cm3602_lock);

	old_status = als_power_control;
	if (enable)
		als_power_control |= pwr_device;
	else
		als_power_control &= ~pwr_device;

	on = als_power_control ? 1 : 0;
	if (old_status == 0 && on)
		ret = __capella_cm3602_power(1);
	else if (!on)
		ret = __capella_cm3602_power(0);

	mutex_unlock(&capella_cm3602_lock);
	return ret;
}

static struct capella_cm3602_platform_data capella_cm3602_pdata = {
	.power = capella_cm3602_power,
	.p_en = BRAVO_GPIO_PROXIMITY_EN,
	.p_out = BRAVO_GPIO_PROXIMITY_INT_N,
	.irq = MSM_GPIO_TO_INT(BRAVO_GPIO_PROXIMITY_INT_N),
};

static struct platform_device capella_cm3602 = {
	.name = CAPELLA_CM3602,
	.id = -1,
	.dev = {
		.platform_data = &capella_cm3602_pdata
	}
};

static void config_bravo_flashlight_gpios(void)
{
	uint32_t flashlight_gpio_table[] = {
		PCOM_GPIO_CFG(BRAVO_GPIO_FLASHLIGHT_TORCH, 0, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_2MA),
		PCOM_GPIO_CFG(BRAVO_GPIO_FLASHLIGHT_FLASH, 0, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_2MA),
	};
	config_gpio_table(flashlight_gpio_table,
		ARRAY_SIZE(flashlight_gpio_table));
	return;
}

static struct flashlight_platform_data bravo_flashlight_data = {
	.gpio_init  = config_bravo_flashlight_gpios,
	.torch = BRAVO_GPIO_FLASHLIGHT_TORCH,
	.flash = BRAVO_GPIO_FLASHLIGHT_FLASH,
	.flash_duration_ms = 600
};

static struct platform_device bravo_flashlight_device = {
	.name = FLASHLIGHT_NAME,
	.dev		= {
		.platform_data	= &bravo_flashlight_data,
	},
};

static struct timed_gpio timed_gpios[] = {
	{
		.name = "vibrator",
		.gpio = BRAVO_VIB_3V3_EN,
		.max_timeout = 15000,
	},
};

static struct timed_gpio_platform_data timed_gpio_data = {
	.num_gpios	= ARRAY_SIZE(timed_gpios),
	.gpios		= timed_gpios,
};

static struct platform_device android_timed_gpios = {
	.name		= "timed-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &timed_gpio_data,
	},
};

static int amoled_power(int on)
{
        if (on) {
                gpio_set_value(BRAVO_LCD_RSTz, 1);
                mdelay(25);
                gpio_set_value(BRAVO_LCD_RSTz, 0);
                mdelay(10);
                gpio_set_value(BRAVO_LCD_RSTz, 1);
                mdelay(20);
        } else {
                gpio_set_value(BRAVO_LCD_RSTz, 0);
        }
        return 0;
}

static int sonywvga_power(int on)
{
	unsigned id, on_off;

	if (on) {
		on_off = 0;

		vreg_enable(vreg_lcm_aux_2v6);
		vreg_enable(vreg_lcm_rftx_2v6);

		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);

		id = PM_VREG_PDOWN_RFTX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);

		gpio_set_value(BRAVO_LCD_RSTz, 1);
		mdelay(10);
		gpio_set_value(BRAVO_LCD_RSTz, 0);
		udelay(500);
		gpio_set_value(BRAVO_LCD_RSTz, 1);
		mdelay(10);
	} else {
		on_off = 1;

		gpio_set_value(BRAVO_LCD_RSTz, 0);

		mdelay(120);

		vreg_disable(vreg_lcm_rftx_2v6);
		vreg_disable(vreg_lcm_aux_2v6);

		id = PM_VREG_PDOWN_RFTX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);

		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
	}
	return 0;
}

#define LCM_GPIO_CFG(gpio, func, str) \
PCOM_GPIO_CFG(gpio, func, GPIO_OUTPUT, GPIO_NO_PULL, str)
static uint32_t display_on_gpio_table_samsung[] = {
        LCM_GPIO_CFG(BRAVO_LCD_R1, 1, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_R2, 1, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_R3, 1, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_R4, 1, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_R5, 1, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_G0, 1, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_G1, 1, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_G2, 1, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_G3, 1, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_G4, 1, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_G5, 1, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_B1, 1, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_B2, 1, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_B3, 1, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_B4, 1, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_B5, 1, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_PCLK, 1, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_VSYNC, 1, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_HSYNC, 1, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_DE, 1, GPIO_4MA),
};


static uint32_t display_off_gpio_table_samsung[] = {
        LCM_GPIO_CFG(BRAVO_LCD_R1, 0, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_R2, 0, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_R3, 0, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_R4, 0, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_R5, 0, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_G0, 0, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_G1, 0, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_G2, 0, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_G3, 0, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_G4, 0, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_G5, 0, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_B1, 0, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_B2, 0, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_B3, 0, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_B4, 0, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_B5, 0, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_PCLK, 0, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_VSYNC, 0, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_HSYNC, 0, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_DE, 0, GPIO_4MA),
};

static uint32_t display_on_gpio_table_sony[] = {
        LCM_GPIO_CFG(BRAVO_LCD_R1, 1, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_R2, 1, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_R3, 1, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_R4, 1, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_R5, 1, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_G0, 1, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_G1, 1, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_G2, 1, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_G3, 1, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_G4, 1, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_G5, 1, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_B1, 1, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_B2, 1, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_B3, 1, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_B4, 1, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_B5, 1, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_PCLK, 1, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_VSYNC, 1, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_HSYNC, 1, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_DE, 1, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_SPI_CLK, 1, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_SPI_DO, 1, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_SPI_CSz, 1, GPIO_4MA),
};


static uint32_t display_off_gpio_table_sony[] = {
        LCM_GPIO_CFG(BRAVO_LCD_R1, 0, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_R2, 0, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_R3, 0, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_R4, 0, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_R5, 0, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_G0, 0, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_G1, 0, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_G2, 0, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_G3, 0, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_G4, 0, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_G5, 0, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_B1, 0, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_B2, 0, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_B3, 0, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_B4, 0, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_B5, 0, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_PCLK, 0, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_VSYNC, 0, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_HSYNC, 0, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_DE, 0, GPIO_8MA),
        LCM_GPIO_CFG(BRAVO_LCD_SPI_CLK, 0, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_SPI_DO, 0, GPIO_4MA),
        LCM_GPIO_CFG(BRAVO_LCD_SPI_CSz, 0, GPIO_4MA),
};


static int panel_gpio_switch_samsung(int on)
{
	config_gpio_table(
		!!on ? display_on_gpio_table_samsung : display_off_gpio_table_samsung,
		ARRAY_SIZE(display_on_gpio_table_samsung));

        return 0;
}

static int panel_gpio_switch_sony(int on)
{
	config_gpio_table(
		!!on ? display_on_gpio_table_sony : display_off_gpio_table_sony,
		ARRAY_SIZE(display_on_gpio_table_sony));

        return 0;
}


static struct resource resources_msm_fb[] = {
        {
                .start = MSM_FB_BASE,
                .end = MSM_FB_BASE + MSM_FB_SIZE - 1,
                .flags = IORESOURCE_MEM,
        },
};

static struct panel_platform_data amoled_data = {
        .fb_res = &resources_msm_fb[0],
        .power = amoled_power,
        .gpio_switch = panel_gpio_switch_samsung,
};

static struct platform_device amoled_panel = {
        .name = "panel-tl2796a",
        .id = -1,
        .dev = {
                .platform_data = &amoled_data,
        },
};

static struct panel_platform_data sonywvga_data = {
	.fb_res = &resources_msm_fb[0],
	.power = sonywvga_power,
	.gpio_switch = panel_gpio_switch_sony,
};

static struct platform_device sonywvga_panel = {
	.name = "panel-sonywvga-s6d16a0x21",
	.id = -1,
	.dev = {
		.platform_data = &sonywvga_data,
	},
};
static struct platform_device *devices[] __initdata = {
	&msm_device_uart1,
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm1,
#endif
	&htc_battery_pdev,
	&htc_headset_mgr,
	&htc_headset_gpio,
	&ram_console_device,
	&bravo_rfkill,
	&msm_device_smd,
	&msm_device_nand,
	&android_pmem_mdp_device,
	&android_pmem_adsp_device,
	&android_pmem_venc_device,
	/*&android_pmem_camera_device,*/
#ifdef CONFIG_MSM_CAMERA
#ifdef CONFIG_S5K3E2FX
	&msm_camera_sensor_s5k3e2fx,
#endif
#endif
	&msm_kgsl_device,
	&msm_device_i2c,
	&bravo_flashlight_device,
#if defined(CONFIG_SPI_QSD)
	&qsd_device_spi,
#endif
#ifdef CONFIG_INPUT_CAPELLA_CM3602
	&capella_cm3602,
#endif
};


static uint32_t bravo_serial_debug_table[] = {
	/* for uart debugger. It should be removed when support usb to serial function */
	/* RX */
	PCOM_GPIO_CFG(BRAVO_GPIO_UART3_RX, 3, GPIO_INPUT, GPIO_NO_PULL,
		      GPIO_4MA),
	/* TX , note here set GPIO to input!!! */
	PCOM_GPIO_CFG(BRAVO_GPIO_UART3_TX, 3, GPIO_INPUT, GPIO_NO_PULL,
		      GPIO_4MA),
};

static void bravo_config_serial_debug_gpios(void)
{
	config_gpio_table(bravo_serial_debug_table, ARRAY_SIZE(bravo_serial_debug_table));
}

static struct msm_i2c_device_platform_data msm_i2c_pdata = {
	.i2c_clock = 100000,
	.clock_strength = GPIO_8MA,
	.data_strength = GPIO_8MA,
};

static void __init msm_device_i2c_init(void)
{
	msm_i2c_gpio_init();
	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static struct msm_acpu_clock_platform_data bravo_clock_data = {
	.acpu_switch_time_us	= 20,
	.max_speed_delta_khz	= 256000,
	.vdd_switch_time_us	= 62,
	.power_collapse_khz	= 245000,
	.wait_for_irq_khz	= 245000,
};

static unsigned bravo_perf_acpu_table[] = {
	245000000,
	576000000,
	998400000,
};

static struct perflock_platform_data bravo_perflock_data = {
	.perf_acpu_table = bravo_perf_acpu_table,
	.table_size = ARRAY_SIZE(bravo_perf_acpu_table),
};

int bravo_init_mmc(int sysrev);

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	/* Chip to Device */
	.rx_wakeup_irq = MSM_GPIO_TO_INT(BRAVO_GPIO_BT_HOST_WAKE),
	.inject_rx_on_wakeup = 0,
	.cpu_lock_supported = 0,

	/* for bcm */
	.bt_wakeup_pin_supported = 1,
	.bt_wakeup_pin = BRAVO_GPIO_BT_CHIP_WAKE,
	.host_wakeup_pin = BRAVO_GPIO_BT_HOST_WAKE,

};
#endif

static void bravo_reset(void)
{
       gpio_set_value(BRAVO_GPIO_PS_HOLD, 0);
}

static int bravo_init_panel(void)
{
	int ret = 0;

	if (panel_type != SAMSUNG_PANEL) {
		ret = platform_device_register(&sonywvga_panel);

		//Here query power source of sony TFT, samsung won't use these parameters
		vreg_lcm_rftx_2v6 = vreg_get(0, "rftx");
		if (IS_ERR(vreg_lcm_rftx_2v6))
			return PTR_ERR(vreg_lcm_rftx_2v6);
		vreg_set_level(vreg_lcm_rftx_2v6, 2600);

		vreg_lcm_aux_2v6 = vreg_get(0, "gp4");
		if (IS_ERR(vreg_lcm_aux_2v6))
			return PTR_ERR(vreg_lcm_aux_2v6);
		vreg_set_level(vreg_lcm_aux_2v6, 2600);
	}
	else
		ret = platform_device_register(&amoled_panel);

	return ret;
}


static void __init bravo_init(void)
{
	int ret;

	printk("bravo_init() revision=%d\n", system_rev);
	printk(KERN_INFO "%s: microp version = %s\n", __func__, microp_ver);

	if (system_rev >= 2) {
		mdp_pmem_pdata.start = MSM_PMEM_MDP_BASE + MSM_MEM_128MB_OFFSET;
		android_pmem_adsp_pdata.start = MSM_PMEM_ADSP_BASE + MSM_MEM_128MB_OFFSET;
	}

	gpio_request(BRAVO_GPIO_LS_EN, "ls_en");

	msm_hw_reset_hook = bravo_reset;
	msm_acpu_clock_init(&bravo_clock_data);
	perflock_init(&bravo_perflock_data);
#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	msm_serial_debug_init(MSM_UART1_PHYS, INT_UART1,
			      &msm_device_uart1.dev, 1, MSM_GPIO_TO_INT(139));
#endif

#ifdef CONFIG_ARCH_QSD8X50
        bt_export_bd_address();
#endif

#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
	msm_device_uart_dm1.name = "msm_serial_hs_bcm";	/* for bcm */
#endif

	bravo_config_serial_debug_gpios();

	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
	gpio_request(BRAVO_GPIO_TP_LS_EN, "tp_ls_en");
	gpio_direction_output(BRAVO_GPIO_TP_LS_EN, 0);
	gpio_request(BRAVO_GPIO_TP_EN, "tp_en");
	gpio_direction_output(BRAVO_GPIO_TP_EN, 0);

	bravo_audio_init();
	msm_device_i2c_init();
#ifdef CONFIG_MICROP_COMMON
	bravo_microp_init();
#endif

	/* set the gpu power rail to manual mode so clk en/dis will not
	 * turn off gpu power, and hang it on resume */
	bravo_kgsl_power_rail_mode(0);
	bravo_kgsl_power(true);

	platform_add_devices(devices, ARRAY_SIZE(devices));
	bravo_init_panel();
	if (!opt_usb_h2w_sw) {
#ifdef CONFIG_USB_FUNCTION
		msm_register_usb_phy_init_seq(bravo_phy_init_seq);
		msm_add_usb_devices(bravo_usb_phy_reset, NULL);
#endif
#ifdef CONFIG_USB_ANDROID
		bravo_add_usb_devices();
#endif
	}

	for (ret = 0; ret < ARRAY_SIZE(i2c_devices); ret++) {
		if (!strcmp(i2c_devices[ret].type, AKM8973_I2C_NAME))
			i2c_devices[ret].irq = MSM_GPIO_TO_INT(BRAVO_GPIO_COMPASS_INT);
       }
	compass_platform_data.intr = BRAVO_GPIO_COMPASS_INT;

	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

	ret = bravo_init_mmc(system_rev);
	if (ret != 0)
		pr_crit("%s: Unable to initialize MMC\n", __func__);

	ret = platform_device_register(&android_timed_gpios);
	if (ret != 0)
		pr_err("failed to register vibrator\n");

	headset_init();

}

#define ATAG_REVISION 0x54410007
static void __init bravo_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
	int bravo_system_rev = 0, find = 0;
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if(t->hdr.tag == ATAG_REVISION) {
			find = 1;
			break;
		}
	}
	if (find)
		bravo_system_rev = t->u.revision.rev;

	mi->nr_banks = 2;
	mi->bank[0].start = MSM_EBI1_BANK0_BASE;
	mi->bank[0].node = PHYS_TO_NID(MSM_EBI1_BANK0_BASE);
	mi->bank[0].size = MSM_EBI1_BANK0_SIZE;
	mi->bank[1].start = MSM_EBI1_BANK1_BASE;
	mi->bank[1].node = PHYS_TO_NID(MSM_EBI1_BANK1_BASE);
	mi->bank[1].size = MSM_EBI1_BANK1_SIZE;
	if (bravo_system_rev >=2) {
		mi->bank[1].size = MSM_EBI1_BANK1_SIZE + MSM_MEM_128MB_OFFSET;
	}
}

static void __init bravo_map_io(void)
{
	msm_map_common_io();
	msm_clock_init();
}

extern struct sys_timer msm_timer;

MACHINE_START(BRAVO, "bravo")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= 0x20000100,
	.fixup		= bravo_fixup,
	.map_io		= bravo_map_io,
	.init_irq	= msm_init_irq,
	.init_machine	= bravo_init,
	.timer		= &msm_timer,
MACHINE_END
