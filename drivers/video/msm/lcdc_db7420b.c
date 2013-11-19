/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/delay.h>
#include <linux/lcd.h>
#include <linux/mfd/pmic8058.h>
#include <mach/gpio.h>
#include <linux/semaphore.h>
#include "msm_fb.h"
/*#include "../../../arch/arm/mach-msm/smd_private.h"*/

#include "lcdc_backlight_ic.h"

#define LCDC_DEBUG

#ifdef LCDC_DEBUG
#define DPRINT(x...)	printk(KERN_INFO "LCD(vital2refresh_db7420b) " x)
#else
#define DPRINT(x...)
#endif

static int spi_cs;
static int spi_sclk;
static int spi_sdi;
/*static int spi_dac;*/
static int lcd_reset;
static int spi_sdo;

#define USE_AUO_SEQ

/*#define ESD_RECOVERY*/
#ifdef ESD_RECOVERY
static irqreturn_t vital2refresh_disp_breakdown_det(int irq, void *handle);
static void lcdc_dsip_reset_work(struct work_struct *work_ptr);
#define GPIO_ESD_DET	39
static unsigned int lcd_det_irq;
static struct delayed_work lcd_reset_work;
static boolean irq_disabled = FALSE;
static boolean wa_first_irq = FALSE;
#endif

struct db7420b_state_type {
	boolean disp_initialized;
	boolean display_on;
	boolean disp_powered_up;
};

static struct db7420b_state_type db7420b_state = { 0 };
static struct msm_panel_common_pdata *lcdc_db7420b_pdata;

static int lcd_prf;

static DEFINE_SEMAPHORE(backlight_sem);
static DEFINE_MUTEX(spi_mutex);

#define PMIC_GPIO_LCD_BL_CTRL	PM8058_GPIO_PM_TO_SYS(PM8058_GPIO(29))
#define LCD_BL_CTRL(value)	gpio_set_value(144, value)

/*
 * Serial Interface
 */

#define DEFAULT_USLEEP	1

struct spi_cmd_desc {
	int dlen;
	char *payload;
	int wait;
};

static char sleep_out_seq[1] = { 0x11 };
static char sleep_in_seq[1] = { 0x10 };
static char disp_on_seq[1] = { 0x29 };
static char disp_off_seq[1] = { 0x28 };
static char id_read_seq[1] = { 0x04 };
/*static char sw_reset_seq[1] = { 0x01 };*/
/*
static char stanby_enable_seq[2] = {
	0xDE,
	0x01
};
*/
static char power_setting_seq1[3] = {
	0xEF,
	0x74, 0x20
};

static char power_setting_seq2[10] = {
	0xB1,
	0x01, 0x00, 0x22, 0x11, 0x73,
	0x70, 0xEC, 0x15, 0x25
};

static char power_setting_seq3[9] = {
	0xB2,
	0x66, 0x06, 0xAA, 0x88,
	0x88, 0x08, 0x08, 0x03
};

static char init_seq1[6] = {
	0xB4,
	0x10, 0x00, 0x32, 0x32, 0x32
};

static char init_seq2[9] = {
	0xB6,
	0x58, 0x72, 0x22, 0x00,
	0x22, 0x00, 0x22, 0x00
};

static char init_seq3[4] = {
	0xD5,
	0x00, 0x43, 0x01
};

static char init_seq4[2] = {
	0x36,
	0x0B
};

static char init_seq5[2] = {
	0x3A,
	0x77
};

static char init_seq6[2] = {
	0xF3,
	0x10
	/* PCLK Polarity Selection
		rising edge of the PCLK : (F3) 10
		falling edge of the PCLK : (F3) 11 */
};

/* Gamma : S-Curve */
static char gamma_set_seq1[35] = {
	0xE0,
	0x30, 0x30, 0x2F, 0x20, 0x38, 0x45, 0x2B, 0x44, 0x07, 0x1F,
	0x15, 0x1A, 0x1D, 0x1D, 0x17, 0x0D, 0x10,
	0x05, 0x06, 0x06, 0x0A, 0x26, 0x32, 0x05, 0x23, 0x02, 0x05,
	0x0F, 0x12, 0x35, 0x13, 0x16, 0x05, 0x06
};

static char gamma_set_seq2[35] = {
	0xE1,
	0x30, 0x30, 0x2F, 0x20, 0x38, 0x45, 0x28, 0x42, 0x07, 0x0F,
	0x13, 0x18, 0x1C, 0x1D, 0x16, 0x0D, 0x10,
	0x05, 0x06, 0x06, 0x0A, 0x26, 0x32, 0x02, 0x21, 0x02, 0x05,
	0x0D, 0x10, 0x32, 0x0E, 0x15, 0x05, 0x06
};

static char gamma_set_seq3[35] = {
	0xE2,
	0x30, 0x30, 0x2F, 0x20, 0x38, 0x45, 0x28, 0x42, 0x07, 0x0F,
	0x13, 0x18, 0x1C, 0x1D, 0x16, 0x0D, 0x10,
	0x05, 0x06, 0x06, 0x0A, 0x26, 0x32, 0x02, 0x21, 0x02, 0x05,
	0x0D, 0x10, 0x32, 0x0E, 0x15, 0x05, 0x06
};

static char additional_seq[3] = {
	0xEF,
	0x00, 0x00
};

static struct spi_cmd_desc display_on_cmds[] = {
	{sizeof(sleep_out_seq), sleep_out_seq, 10},
	{sizeof(id_read_seq), id_read_seq, 10},

	{sizeof(power_setting_seq1), power_setting_seq1, 0},
	{sizeof(power_setting_seq2), power_setting_seq2, 0},
	{sizeof(power_setting_seq3), power_setting_seq3, 0},

	{sizeof(init_seq1), init_seq1, 0},
	{sizeof(init_seq2), init_seq2, 0},
	{sizeof(init_seq3), init_seq3, 0},
	{sizeof(init_seq4), init_seq4, 0},
	{sizeof(init_seq5), init_seq5, 0},
	{sizeof(init_seq6), init_seq6, 0},

	{sizeof(gamma_set_seq1), gamma_set_seq1, 0},
	{sizeof(gamma_set_seq2), gamma_set_seq2, 0},
	{sizeof(gamma_set_seq3), gamma_set_seq3, 0},

	{sizeof(additional_seq), additional_seq, 0},
	{sizeof(disp_on_seq), disp_on_seq, 0},
};

static struct spi_cmd_desc display_off_cmds[] = {
	{sizeof(disp_off_seq), disp_off_seq, 0},
	{sizeof(sleep_in_seq), sleep_in_seq, 120},
};
/*
static struct spi_cmd_desc display_sleep_in[] = {
	{sizeof(disp_off_seq), disp_off_seq, 0},
	{sizeof(stanby_enable_seq), stanby_enable_seq, 0},
};

static struct spi_cmd_desc display_sleep_out[] = {
	{sizeof(power_setting_seq1), power_setting_seq1, 0},
	{sizeof(power_setting_seq2), power_setting_seq2, 0},
	{sizeof(power_setting_seq3), power_setting_seq3, 0},
	{sizeof(sleep_out_seq), sleep_out_seq, 120},
};
*/
/*
static struct spi_cmd_desc sw_rdy_cmds[] = {
	{sizeof(sleep_out_seq), sleep_out_seq, 0},
};
*/

static int lcdc_db7420b_panel_off(struct platform_device *pdev);

static void spi_cmds_tx(struct spi_cmd_desc *desc, int cnt)
{
	long i, j, p;
	/*unsigned long irqflags;*/

	mutex_lock(&spi_mutex);
	for (p = 0; p < cnt; p++) {
		gpio_set_value(spi_cs, 1);
		udelay(DEFAULT_USLEEP);
		gpio_set_value(spi_sclk, 1);
		udelay(DEFAULT_USLEEP);

		/* Write Command */
		gpio_set_value(spi_cs, 0);
		udelay(DEFAULT_USLEEP);
		gpio_set_value(spi_sclk, 0);
		udelay(DEFAULT_USLEEP);
		gpio_set_value(spi_sdi, 0);
		udelay(DEFAULT_USLEEP);

		gpio_set_value(spi_sclk, 1);
		udelay(DEFAULT_USLEEP);

		for (i = 7; i >= 0; i--) {
			gpio_set_value(spi_sclk, 0);
			udelay(DEFAULT_USLEEP);
			if (((char)*(desc+p)->payload >> i) & 0x1)
				gpio_set_value(spi_sdi, 1);
			else
				gpio_set_value(spi_sdi, 0);
			udelay(DEFAULT_USLEEP);
			gpio_set_value(spi_sclk, 1);
			udelay(DEFAULT_USLEEP);
		}

		gpio_set_value(spi_cs, 1);
		udelay(DEFAULT_USLEEP);

		/* Write Parameter */
		if ((desc+p)->dlen < 2)
			goto tx_done;

		for (j = 1; j < (desc+p)->dlen; j++) {
			gpio_set_value(spi_cs, 0);
			udelay(DEFAULT_USLEEP);

			gpio_set_value(spi_sclk, 0);
			udelay(DEFAULT_USLEEP);
			gpio_set_value(spi_sdi, 1);
			udelay(DEFAULT_USLEEP);
			gpio_set_value(spi_sclk, 1);
			udelay(DEFAULT_USLEEP);

			for (i = 7; i >= 0; i--) {
				gpio_set_value(spi_sclk, 0);
				udelay(DEFAULT_USLEEP);
				if (((char)*((desc+p)->payload+j) >> i) & 0x1)
					gpio_set_value(spi_sdi, 1);
				else
					gpio_set_value(spi_sdi, 0);
				udelay(DEFAULT_USLEEP);
				gpio_set_value(spi_sclk, 1);
				udelay(DEFAULT_USLEEP);
			}

			gpio_set_value(spi_cs, 1);
			udelay(DEFAULT_USLEEP);
		}
tx_done:
		if ((desc+p)->wait)
			msleep((desc+p)->wait);
	}
	mutex_unlock(&spi_mutex);
}

static void spi_init(void)
{
	/* Setting the Default GPIO's */
	spi_sclk = *(lcdc_db7420b_pdata->gpio_num);
	spi_cs   = *(lcdc_db7420b_pdata->gpio_num + 1);
	spi_sdi  = *(lcdc_db7420b_pdata->gpio_num + 2);
	lcd_reset  = *(lcdc_db7420b_pdata->gpio_num + 3);
	spi_sdo  = *(lcdc_db7420b_pdata->gpio_num + 4);


	/* Set the output so that we dont disturb the slave device */
	gpio_set_value(spi_sclk, 0);
	gpio_set_value(spi_sdi, 0);

	/* Set the Chip Select De-asserted */
	gpio_set_value(spi_cs, 0);

	gpio_set_value(spi_sdo, 0);
}

static void lcdc_panel_reset(int on) /* chief.lcd.timing */
{
	if (on) {
		pr_info("@LCDINIT@:LCD RST high with 0x%x\n", on);

#ifndef LCD_WAKEUP_PERFORMANCE
		msleep(300);
		pr_info("@LCDINIT@:DELAY with 300 msec!\n");
#endif
		gpio_set_value(lcd_reset, 0);
		msleep(70);
		gpio_set_value(lcd_reset, 1);
		usleep(5000);
	} else {
		gpio_set_value(lcd_reset, 0);
	}
}

static void db7420b_disp_powerup(void)
{
	DPRINT("start %s, lcd_reset:gpio %d\n", __func__, lcd_reset);

	if (!db7420b_state.disp_powered_up && !db7420b_state.display_on) {
		/* Reset the hardware first */
		/* Include DAC power up implementation here */
		lcdc_panel_reset(1);
	    db7420b_state.disp_powered_up = TRUE;
	}
}

static void db7420b_disp_powerdown(void)
{
	DPRINT("start %s, lcd_reset:gpio %d\n", __func__, lcd_reset);

	if (db7420b_state.disp_powered_up && !db7420b_state.display_on) {
		/* turn off LDO */
		/*TODO: turn off LDO*/
		db7420b_state.disp_powered_up = FALSE;
	}
}
/*
static void db7420b_init(void)
{
	mdelay(5);
}
*/
void db7420b_disp_on(void)
{
	/*int i;*/
	pr_info("@LCDINIT@:DISPON sequence\n");
	DPRINT("start %s\n", __func__);

	if (db7420b_state.disp_powered_up && !db7420b_state.display_on) {
		/*db7420b_init();*/
		/*mdelay(20);*/

		/* db7420b setting */
		spi_cmds_tx(display_on_cmds, ARRAY_SIZE(display_on_cmds));

		/*mdelay(1);*/
		udelay(1000);
		db7420b_state.display_on = TRUE;
	}
}

static int lcdc_db7420b_panel_on(struct platform_device *pdev)
{
	static int bring_up_condition;
	/*unsigned size;*/

	DPRINT("start %s, bring_up %d, disp_initialized %d\n",
		__func__, bring_up_condition, db7420b_state.disp_initialized);

	if (!bring_up_condition) {
		/* trick initalization for timing issue */
		bring_up_condition = 1;

		/* Configure reset GPIO that drives DAC */
		spi_init();	/* LCD needs SPI */
		lcdc_db7420b_pdata->panel_config_gpio(1);
		db7420b_state.disp_powered_up = TRUE;
		db7420b_state.display_on = TRUE;
		db7420b_state.disp_initialized = TRUE;
	} else {
		if (!db7420b_state.disp_initialized) {
			/* Configure reset GPIO that drives DAC */
			db7420b_disp_powerup();
			spi_init();	/* LCD needs SPI */
			lcdc_db7420b_pdata->panel_config_gpio(1);
			db7420b_disp_on();
			db7420b_state.disp_initialized = TRUE;

#ifdef ESD_RECOVERY
			if (irq_disabled) {
				enable_irq(lcd_det_irq);
				irq_disabled = FALSE;
			}
#endif
		}
	}

	return 0;
}

static int lcdc_db7420b_panel_off(struct platform_device *pdev)
{
	/*int i;*/

	DPRINT("start %s\n", __func__);

	if (db7420b_state.disp_powered_up && db7420b_state.display_on) {

#ifdef ESD_RECOVERY
		disable_irq_nosync(lcd_det_irq);
		irq_disabled = TRUE;
#endif

		spi_cmds_tx(display_off_cmds, ARRAY_SIZE(display_off_cmds));


		lcdc_db7420b_pdata->panel_config_gpio(0);
		db7420b_state.display_on = FALSE;
		db7420b_state.disp_initialized = FALSE;
		db7420b_disp_powerdown();
		lcd_prf = 0;
	}



	return 0;
}

static void lcdc_db7420b_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_value = mfd->bl_level;
	static int lockup_count;

	up(&backlight_sem);
	DPRINT("[BACLKIGHT] : %d\n", bl_value);
	if (!bl_value) {
		/*  Turn off Backlight, don't check disp_initialized value */
		lcd_prf = 1;

	} else {
		if (lcd_prf)
			return;

		while (!db7420b_state.disp_initialized) {
			msleep(100);
			lockup_count++;

			if (lockup_count > 50) {
				printk(KERN_ERR "Prevent infinite loop(wait for 5s)\n");
				printk(KERN_ERR "LCD can't initialize with in %d ms\n"
					, lockup_count*100);
				lockup_count = 0;

				down(&backlight_sem);
				return;
			}
		}
	}

	backlight_ic_set_brightness(bl_value);

	down(&backlight_sem);
}

static int __devinit db7420b_probe(struct platform_device *pdev)
{
	/*int err;*/
	/*int ret;*/
	/*unsigned size;*/
	DPRINT("start %s\n", __func__);

	db7420b_state.disp_initialized = TRUE; /*signal_timing*/
	db7420b_state.disp_powered_up = TRUE;
	db7420b_state.display_on = TRUE;

	if (pdev->id == 0) {
		lcdc_db7420b_pdata = pdev->dev.platform_data;
		if (!lcdc_db7420b_pdata)
			return -EINVAL;
#ifdef ESD_RECOVERY
		gpio_tlmm_config(GPIO_CFG(GPIO_ESD_DET,  0, GPIO_CFG_INPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		lcd_det_irq = MSM_GPIO_TO_INT(GPIO_ESD_DET);
		if (!lcd_det_irq)
			DPRINT("LCD_DETECT_IRQ is NULL!\n");

		INIT_DELAYED_WORK(&lcd_reset_work, lcdc_dsip_reset_work);
		ret = request_irq(lcd_det_irq, vital2refresh_disp_breakdown_det,
		IRQF_TRIGGER_RISING, "lcd_esd_det", NULL);

		if (ret) {
			pr_err("Request_irq failed for TLMM_MSM_SUMMARY_IRQ - %d\n",
				ret);
			return ret;
		}
#endif
		return 0;
	}
	msm_fb_add_device(pdev);

	return 0;
}

static void db7420b_shutdown(struct platform_device *pdev)
{
	DPRINT("start %s\n", __func__);

	lcdc_db7420b_panel_off(pdev);
}

static struct platform_driver this_driver = {
	.probe  = db7420b_probe,
	.shutdown	= db7420b_shutdown,
	.driver = {
		.name   = "lcdc_db7420b_hvga",
	},
};

static struct msm_fb_panel_data db7420b_panel_data = {
	.on = lcdc_db7420b_panel_on,
	.off = lcdc_db7420b_panel_off,
	.set_backlight = lcdc_db7420b_set_backlight,
};

static struct platform_device this_device = {
	.name   = "lcdc_panel",
	.id	= 1,
	.dev	= {
		.platform_data = &db7420b_panel_data,
	}
};

#define LCDC_FB_XRES	320
#define LCDC_FB_YRES	480
#define LCDC_HBP		32
#define LCDC_HPW		4
#define LCDC_HFP		4
#define LCDC_VBP		7
#define LCDC_VPW		4
#define LCDC_VFP		18

static int __init lcdc_db7420b_panel_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

#ifdef CONFIG_FB_MSM_LCDC_AUTO_DETECT
	if (msm_fb_detect_client("lcdc_db7420b_hvga")) {
		printk(KERN_ERR "%s: detect another lcd driver!\n", __func__);
		return 0;
	}
#endif
	DPRINT("start %s\n", __func__);

	update_backlight_table(BACKLIGHT_DB7420B);

	ret = platform_driver_register(&this_driver);
	if (ret) {
		printk(KERN_ERR "%s: platform_driver_register failed! ret=%d\n",
						__func__, ret);
		return ret;
	}

	pinfo = &db7420b_panel_data.panel_info;
	pinfo->xres = LCDC_FB_XRES;
	pinfo->yres = LCDC_FB_YRES;
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 24;
	pinfo->fb_num = 2;
	pinfo->clk_rate = 14746000;/*24576000;*/
	pinfo->bl_max = 255;
	pinfo->bl_min = 1;

	pinfo->lcdc.h_back_porch = LCDC_HBP;
	pinfo->lcdc.h_front_porch = LCDC_HFP;
	pinfo->lcdc.h_pulse_width = LCDC_HPW;
	pinfo->lcdc.v_back_porch = LCDC_VBP;
	pinfo->lcdc.v_front_porch = LCDC_VFP;
	pinfo->lcdc.v_pulse_width = LCDC_VPW;
	pinfo->lcdc.border_clr = 0;     /* blk */
	pinfo->lcdc.underflow_clr = 0xff;       /* blue */
	pinfo->lcdc.hsync_skew = 0;

	ret = platform_device_register(&this_device);
	if (ret) {
		printk(KERN_ERR "%s: platform_device_register failed! ret=%d\n",
						__func__, ret);
		platform_driver_unregister(&this_driver);
	}


	return ret;
}

#ifdef ESD_RECOVERY
static irqreturn_t vital2refresh_disp_breakdown_det(int irq, void *handle)
{
	if (db7420b_state.disp_initialized)
		schedule_delayed_work(&lcd_reset_work, 0);

	return IRQ_HANDLED;
}

static void lcdc_dsip_reset_work(struct work_struct *work_ptr)
{
	if (!wa_first_irq) {
		DPRINT("skip lcd reset\n");
		wa_first_irq = TRUE;
		return;
	}

	DPRINT("lcd reset\n");

	lcdc_db7420b_panel_off(NULL);

	lcdc_db7420b_panel_on(NULL);
}
#endif

module_init(lcdc_db7420b_panel_init);
