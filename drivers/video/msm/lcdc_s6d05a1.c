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
#include "mdnie_sysfs.h"
#ifdef CONFIG_SAMSUNG_DISPLAY_SYSFS
#include "samsung_display_sysfs.h"
#endif

#define LCDC_DEBUG

#ifdef LCDC_DEBUG
#define DPRINT(x...)	printk(KERN_INFO "LCD(s6d05a1) " x)
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
static irqreturn_t trebon_disp_breakdown_det(int irq, void *handle);
static void lcdc_dsip_reset_work(struct work_struct *work_ptr);
#define GPIO_ESD_DET	39
static unsigned int lcd_det_irq;
static struct delayed_work lcd_reset_work;
static boolean irq_disabled = FALSE;
static boolean wa_first_irq = FALSE;
#endif

struct s6d05a1_state_type {
	boolean disp_initialized;
	boolean display_on;
	boolean disp_powered_up;
	boolean negative;
};

static struct s6d05a1_state_type s6d05a1_state = { 0 };
static struct msm_panel_common_pdata *lcdc_s6d05a1_pdata;

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
static char disp_on_seq[1] = { 0x29 };
static char disp_off_seq[1] = { 0x28 };
static char sleep_in_seq[1] = { 0x10 };
/*static char sw_reset_seq[1] = { 0x01 };*/

static char set_negative_on[1] = { 0x21 };
static char set_negative_off[1] = { 0x20 };

static struct spi_cmd_desc set_negative_on_cmd[] = {
	{sizeof(set_negative_on), set_negative_on, 0},
};

static struct spi_cmd_desc set_negative_off_cmd[] = {
	{sizeof(set_negative_off), set_negative_off, 0},
};

#if !defined(USE_AUO_SEQ)

static char level2_command1[3] = {
	0xF0,
	0x5A, 0x5A
};
static char level2_command2[3] = {
	0xF1,
	0x5A, 0x5A
};

static char power_setting_seq1[20] = {
	0xF2,
	0x3B, 0x3A, 0x03, 0x04, 0x02,
	0x08, 0x08, 0x00, 0x08, 0x08,
	0x00, 0x00, 0x00, 0x00, 0x54,
	0x08, 0x08, 0x08, 0x08
};

static char power_setting_seq2[12] = {
	0xF4,
	0x0A, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x15, 0x6B,
	0x03
};

static char power_setting_seq3[10] = {
	0xF5,
	0x00, 0x47, 0x75, 0x00, 0x00,
	0x04, 0x00, 0x00, 0x04
};

static char init_seq1[2] = {
	0x3A,
	0x77
};

static char init_seq2[2] = {
	0x35,
	0x00
};

static char init_seq3[2] = {
	0x36,
	0xD0/*00*/
};

static char init_seq4[5] = {
	/*
	 * MADCTL
	 * User setting (LDI Upside)
	 */
	0x2A,
	0x00, 0x00, 0x01, 0x3F
};

static char init_seq5[5] = {
	/*
	 * COLMOD
	 * User setting (16M Color)
	 */
	0x2B,
	0x00, 0x00, 0x01, 0xDF
};

static char init_seq6[7] = {
	/*
	 * SETPOL
	 * User setting (DE / DCLK polarity)
	 */
	0xF6,
	0x04, 0x00, 0x08, 0x03, 0x01,
	0x00
};

static char init_seq7[6] = {
	0xF7,
	0x48, 0x01, 0xF0, 0x14, 0x00
};

static char init_seq8[3] = {
	0xF8,
	0x11, 0x00
};

static char gamma_set_seq1[2] = {
	0xF9,
	0x24
};

/*Gamma Setting 2.6 */
static char gamma_set_seq2[35] = {
	/* Blue gamma */
	0xFA,
	0x23, 0x00, 0x0A, 0x18, 0x1E,
	0x22, 0x29, 0x1D, 0x2A, 0x2F,
	0x3A, 0x3C, 0x30, 0x00, 0x2A,
	0x00
};

static char gamma_set_seq3[2] = {
	0xF9,
	0x22
};

/*Gamma Setting 2.6 */
static char gamma_set_seq4[35] = {
	/* Blue gamma */
	0xFA,
	0x30, 0x10, 0x08, 0x1B, 0x1B,
	0x1F, 0x25, 0x1A, 0x26, 0x24,
	0x25, 0x22, 0x2C, 0x00, 0x2A,
	0x00
};

static char gamma_set_seq5[2] = {
	0xF9,
	0x21
};

/*Gamma Setting 2.6 */
static char gamma_set_seq6[35] = {
	/* Blue gamma */
	0xFA,
	0x30, 0x10, 0x0A, 0x21, 0x31,
	0x33, 0x32, 0x10, 0x1D, 0x20,
	0x21, 0x21, 0x20, 0x00, 0x2A,
	0x00
};
/*
static char set_close_password[3] = {
	0xEF,
	0x00, 0x00
};

static char deep_standby_en[2] = {
	0xDE,
	0x01
};
*/
static struct spi_cmd_desc display_on_cmds[] = {
	{sizeof(level2_command1), level2_command1, 0},
	{sizeof(level2_command2), level2_command2, 0},
	{sizeof(power_setting_seq1), power_setting_seq1, 0},
	{sizeof(power_setting_seq2), power_setting_seq2, 0},
	{sizeof(power_setting_seq3), power_setting_seq3, 0},

	{sizeof(init_seq1), init_seq1, 0},
	{sizeof(init_seq2), init_seq2, 0},
	{sizeof(init_seq3), init_seq3, 0},
	{sizeof(init_seq4), init_seq4, 0},
	{sizeof(init_seq5), init_seq5, 0},
	{sizeof(init_seq6), init_seq6, 0},
	{sizeof(init_seq6), init_seq7, 0},
	{sizeof(init_seq6), init_seq8, 0},

	{sizeof(gamma_set_seq1), gamma_set_seq1, 0},
	{sizeof(gamma_set_seq2), gamma_set_seq2, 0},
	{sizeof(gamma_set_seq3), gamma_set_seq3, 0},
	{sizeof(gamma_set_seq4), gamma_set_seq4, 0},
	{sizeof(gamma_set_seq5), gamma_set_seq5, 0},
	{sizeof(gamma_set_seq6), gamma_set_seq6, 0},

	{sizeof(sleep_out_seq), sleep_out_seq, 120},
	{sizeof(disp_on_seq), disp_on_seq, 40},
};

static struct spi_cmd_desc display_off_cmds[] = {
	{sizeof(disp_off_seq), disp_off_seq, 40},
	{sizeof(sleep_in_seq), sleep_in_seq, 120},
};

static struct spi_cmd_desc display_sleep_in[] = {
	{sizeof(sleep_in_seq), sleep_in_seq, 120},
};

static struct spi_cmd_desc display_sleep_out[] = {
	{sizeof(power_setting_seq1), power_setting_seq1, 0},
	{sizeof(power_setting_seq2), power_setting_seq2, 0},
	{sizeof(power_setting_seq3), power_setting_seq3, 0},
	{sizeof(sleep_out_seq), sleep_out_seq, 120},
};
/*
static struct spi_cmd_desc sw_rdy_cmds[] = {
	{sizeof(sleep_out_seq), sleep_out_seq, 0},
};
*/
#else
static char level2_command1[3] = {
	0xF0,
	0x5A, 0x5A
};
static char level2_command2[3] = {
	0xF1,
	0x5A, 0x5A
};

static char power_setting_seq1[20] = {
	0xF2,
	0x3B, 0x34, 0x03, 0x04, 0x04,
	0x08, 0x08, 0x00, 0x08, 0x08,
	0x00, 0x00, 0x00, 0x08, 0x54,
	0x08, 0x08, 0x04, 0x04
};

static char power_setting_seq2[12] = {
	0xF4,
	0x0A, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x15, 0x70,
	0x03
};

static char power_setting_seq3[10] = {
	0xF5,
	0x00, 0x56, 0x60, 0x00, 0x00,
	0x02, 0x00, 0x00, 0x04
};

static char init_seq1[2] = {
	0x3A,
	0x77
};

static char init_seq2[2] = {
	0x36,
	0xD0/*00*/
};

static char init_seq3[5] = {
	/*
	 * MADCTL
	 * User setting (LDI Upside)
	 */
	0x2A,
	0x00, 0x00, 0x01, 0x3F
};

static char init_seq4[5] = {
	/*
	 * COLMOD
	 * User setting (16M Color)
	 */
	0x2B,
	0x00, 0x00, 0x01, 0xDF
};

static char init_seq5[7] = {
	/*
	 * SETPOL
	 * User setting (DE / DCLK polarity)
	 */
	0xF6,
	0x06, 0x00, 0x08, 0x03, 0x01,
	0x00
};

static char init_seq6[6] = {
	0xF7,
	0x48, 0x01, 0xD8, 0x14, 0x00
};

static char init_seq7[3] = {
	0xF8,
	0x11, 0x00
};

static char gamma_set_seq1[2] = {
	0xF9,
	0x14
};

/*Gamma Setting 2.6 */
static char gamma_set_seq2[35] = {
	/* Blue gamma */
	0xFA,
	0x19, 0x00, 0x00, 0x1D, 0x1C,
	0x22, 0x2B, 0x1A, 0x21, 0x26,
	0x1B, 0x24, 0x1A, 0x00, 0x00,
	0x01
};

static char gamma_set_seq3[35] = {
	/* Blue gamma */
	0xFB,
	0x00, 0x07, 0x1A, 0x24, 0x1B,
	0x26, 0x21, 0x19, 0x2A, 0x23,
	0x1C, 0x1D, 0x00, 0x00, 0x00,
	0x02
};

static char gamma_set_seq4[2] = {
	0xF9,
	0x12
};

/*Gamma Setting 2.6 */
static char gamma_set_seq5[35] = {
	/* Blue gamma */
	0xFA,
	0x1A, 0x06, 0x04, 0x20, 0x22,
	0x29, 0x2C, 0x17, 0x23, 0x24,
	0x21, 0x14, 0x00, 0x00, 0x00,
	0x01
};

static char gamma_set_seq6[35] = {
	/* Blue gamma */
	0xFB,
	0x06, 0x10, 0x00, 0x14, 0x21,
	0x24, 0x22, 0x17, 0x2D, 0x28,
	0x22, 0x20, 0x04, 0x00, 0x00,
	0x02
};

static char gamma_set_seq7[2] = {
	0xF9,
	0x11
};

/*Gamma Setting 2.6 */
static char gamma_set_seq8[35] = {
	/* Blue gamma */
	0xFA,
	0x18, 0x06, 0x1E, 0x36, 0x34,
	0x34, 0x36, 0x10, 0x1D, 0x21,
	0x15, 0x19, 0x11, 0x00, 0x00,
	0x01
};

static char gamma_set_seq9[35] = {
	/* Blue gamma */
	0xFB,
	0x06, 0x12, 0x11, 0x19, 0x15,
	0x21, 0x1D, 0x10, 0x36, 0x34,
	0x34, 0x36, 0x1E, 0x00, 0x00,
	0x02
};

static char level2_command1_block[3] = {
	0xF0,
	0xA5, 0xA5
};
static char level2_command2_block[3] = {
	0xF1,
	0xA5, 0xA5
};

static struct spi_cmd_desc display_on_cmds[] = {
	{sizeof(level2_command1), level2_command1, 0},
	{sizeof(level2_command2), level2_command2, 0},

	{sizeof(power_setting_seq1), power_setting_seq1, 0},
	{sizeof(power_setting_seq2), power_setting_seq2, 0},
	{sizeof(power_setting_seq3), power_setting_seq3, 0},

	{sizeof(init_seq1), init_seq1, 0},
	{sizeof(init_seq2), init_seq2, 0},
	{sizeof(init_seq3), init_seq3, 0},
	{sizeof(init_seq4), init_seq4, 0},
	{sizeof(init_seq5), init_seq5, 0},
	{sizeof(init_seq6), init_seq6, 0},
	{sizeof(init_seq7), init_seq7, 0},

	{sizeof(gamma_set_seq1), gamma_set_seq1, 0},
	{sizeof(gamma_set_seq2), gamma_set_seq2, 0},
	{sizeof(gamma_set_seq3), gamma_set_seq3, 0},
	{sizeof(gamma_set_seq4), gamma_set_seq4, 0},
	{sizeof(gamma_set_seq5), gamma_set_seq5, 0},
	{sizeof(gamma_set_seq6), gamma_set_seq6, 0},
	{sizeof(gamma_set_seq7), gamma_set_seq7, 0},
	{sizeof(gamma_set_seq8), gamma_set_seq8, 0},
	{sizeof(gamma_set_seq9), gamma_set_seq9, 0},

	{sizeof(level2_command1_block), level2_command1_block, 0},
	{sizeof(level2_command2_block), level2_command2_block, 0},

	{sizeof(sleep_out_seq), sleep_out_seq, 120},
	{sizeof(disp_on_seq), disp_on_seq, 40},
};

static struct spi_cmd_desc display_off_cmds[] = {
	{sizeof(disp_off_seq), disp_off_seq, 40},
	{sizeof(sleep_in_seq), sleep_in_seq, 120},
};

static struct spi_cmd_desc display_sleep_out[] = {
	{sizeof(power_setting_seq1), power_setting_seq1, 0},
	{sizeof(power_setting_seq2), power_setting_seq2, 0},
	{sizeof(power_setting_seq3), power_setting_seq3, 0},
	{sizeof(sleep_out_seq), sleep_out_seq, 120},
};

static struct spi_cmd_desc display_sleep_in[] = {
	{sizeof(sleep_in_seq), sleep_in_seq, 120},
};
#endif
static int lcdc_s6d05a1_panel_off(struct platform_device *pdev);

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
	spi_sclk = *(lcdc_s6d05a1_pdata->gpio_num);
	spi_cs   = *(lcdc_s6d05a1_pdata->gpio_num + 1);
	spi_sdi  = *(lcdc_s6d05a1_pdata->gpio_num + 2);
	lcd_reset  = *(lcdc_s6d05a1_pdata->gpio_num + 3);
	spi_sdo  = *(lcdc_s6d05a1_pdata->gpio_num + 4);


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
/*
		msleep(300);
		pr_info("@LCDINIT@:DELAY with 300 msec!\n");
*/
#endif
		gpio_set_value(lcd_reset, 0);
		msleep(70);
		gpio_set_value(lcd_reset, 1);
		usleep(15000);
	} else {
		gpio_set_value(lcd_reset, 0);
	}
}

static void s6d05a1_disp_powerup(void)
{
	DPRINT("start %s, lcd_reset:gpio %d\n", __func__, lcd_reset);

	if (!s6d05a1_state.disp_powered_up && !s6d05a1_state.display_on) {
		/* Reset the hardware first */
		/* Include DAC power up implementation here */
		lcdc_panel_reset(1);
	    s6d05a1_state.disp_powered_up = TRUE;
	}
}

static void s6d05a1_disp_powerdown(void)
{
	DPRINT("start %s, lcd_reset:gpio %d\n", __func__, lcd_reset);

	if (s6d05a1_state.disp_powered_up && !s6d05a1_state.display_on) {
		/* turn off LDO */
		/*TODO: turn off LDO*/
		s6d05a1_state.disp_powered_up = FALSE;
	}
}
/*
static void s6d05a1_init(void)
{
	mdelay(5);
}
*/
void s6d05a1_disp_on(void)
{
	/*int i;*/
	pr_info("@LCDINIT@:DISPON sequence\n");
	DPRINT("start %s\n", __func__);

	if (s6d05a1_state.disp_powered_up && !s6d05a1_state.display_on) {
		/*s6d05a1_init();*/
		/*mdelay(20);*/
		if (s6d05a1_state.negative == TRUE)
			spi_cmds_tx(set_negative_on_cmd,
					ARRAY_SIZE(set_negative_on_cmd));
		/* s6d05a1 setting */
		spi_cmds_tx(display_on_cmds, ARRAY_SIZE(display_on_cmds));

		/*mdelay(1);*/
		udelay(1000);
		s6d05a1_state.display_on = TRUE;
	}
}

void s6d05a1_sleep_off(void)
{
	/*int i = 0;*/
	DPRINT("start %s\n", __func__);

	spi_cmds_tx(display_sleep_out, ARRAY_SIZE(display_sleep_out));

	/*mdelay(1);*/
	udelay(1000);
}

void s6d05a1_sleep_in(void)
{
	/*int i = 0;*/
	DPRINT("start %s\n", __func__);

	spi_cmds_tx(display_sleep_in, ARRAY_SIZE(display_sleep_in));

	lcd_prf = 0;

	/*mdelay(1);*/
	udelay(1000);
}

static int lcdc_s6d05a1_panel_on(struct platform_device *pdev)
{
	static int bring_up_condition = 1;
	/*unsigned size;*/

	DPRINT("start %s, bring_up %d, disp_initialized %d\n",
		__func__, bring_up_condition, s6d05a1_state.disp_initialized);

	if (!bring_up_condition) {
		/* trick initalization for timing issue */
		bring_up_condition = 1;

		/* Configure reset GPIO that drives DAC */
		spi_init();	/* LCD needs SPI */
		lcdc_s6d05a1_pdata->panel_config_gpio(1);
		s6d05a1_state.disp_powered_up = TRUE;
		s6d05a1_state.display_on = TRUE;
		s6d05a1_state.disp_initialized = TRUE;
	} else {
		if (!s6d05a1_state.disp_initialized) {
			/* Configure reset GPIO that drives DAC */
			spi_init();	/* LCD needs SPI */
			s6d05a1_disp_powerup();
			lcdc_s6d05a1_pdata->panel_config_gpio(1);
			s6d05a1_disp_on();
			s6d05a1_state.disp_initialized = TRUE;

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

static int lcdc_s6d05a1_panel_off(struct platform_device *pdev)
{
	/*int i;*/

	DPRINT("start %s\n", __func__);

	if (s6d05a1_state.disp_powered_up && s6d05a1_state.display_on) {

#ifdef ESD_RECOVERY
		disable_irq_nosync(lcd_det_irq);
		irq_disabled = TRUE;
#endif

		spi_cmds_tx(display_off_cmds, ARRAY_SIZE(display_off_cmds));


		lcdc_s6d05a1_pdata->panel_config_gpio(0);
		s6d05a1_state.display_on = FALSE;
		s6d05a1_state.disp_initialized = FALSE;
		s6d05a1_disp_powerdown();
		lcd_prf = 0;
	}



	return 0;
}

int apply_negative_value_s6d0a1(enum eNegative_Mode negative_mode)
{
	if (negative_mode == NEGATIVE_ON_MODE) {
		spi_cmds_tx(set_negative_on_cmd,
					ARRAY_SIZE(set_negative_on_cmd));
		s6d05a1_state.negative = TRUE;
	} else {
		spi_cmds_tx(set_negative_off_cmd,
					ARRAY_SIZE(set_negative_off_cmd));
		s6d05a1_state.negative = FALSE;
	}
	return 0;
}

static void lcdc_s6d05a1_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_value = mfd->bl_level;
	static int lockup_count;

	up(&backlight_sem);
	DPRINT("[BACKLIGHT] : %d\n", bl_value);
	if (!bl_value) {
		/*  Turn off Backlight, don't check disp_initialized value */
		lcd_prf = 1;

	} else {
		if (lcd_prf)
			return;

		while (!s6d05a1_state.disp_initialized) {
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

static int __devinit s6d05a1_probe(struct platform_device *pdev)
{
	/*int err;*/
	int ret = 0;
	struct mdnie_ops p_mdnie_ops;
#ifdef CONFIG_SAMSUNG_DISPLAY_SYSFS
	static struct platform_device *msm_fb_added_dev;
#endif
	/*unsigned size;*/
	DPRINT("start %s\n", __func__);

	s6d05a1_state.disp_initialized = FALSE; /*signal_timing*/
	s6d05a1_state.disp_powered_up = FALSE;
	s6d05a1_state.display_on = FALSE;
	s6d05a1_state.negative = FALSE;

	if (pdev->id == 0) {
		lcdc_s6d05a1_pdata = pdev->dev.platform_data;
		if (!lcdc_s6d05a1_pdata)
			return -EINVAL;

#ifdef ESD_RECOVERY
		gpio_tlmm_config(GPIO_CFG(GPIO_ESD_DET,  0, GPIO_CFG_INPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		lcd_det_irq = MSM_GPIO_TO_INT(GPIO_ESD_DET);
		if (!lcd_det_irq)
			DPRINT("LCD_DETECT_IRQ is NULL!\n");

		INIT_DELAYED_WORK(&lcd_reset_work, lcdc_dsip_reset_work);
		ret = request_irq(lcd_det_irq, trebon_disp_breakdown_det,
		IRQF_TRIGGER_RISING, "lcd_esd_det", NULL);

		if (ret) {
			pr_err("Request_irq failed for TLMM_MSM_SUMMARY_IRQ - %d\n",
				ret);
			return ret;
		}
#endif
		return ret;
	}
#ifdef CONFIG_SAMSUNG_DISPLAY_SYSFS
	msm_fb_added_dev = msm_fb_add_device(pdev);
	samsung_display_sysfs_create(pdev, msm_fb_added_dev, "AUO");
#else
	msm_fb_add_device(pdev);
#endif
	p_mdnie_ops.apply_negative_value = apply_negative_value_s6d0a1;
	p_mdnie_ops.apply_cabc_value = NULL;
	mdnie_sysfs_init(&p_mdnie_ops);

	return ret;
}

static void s6d05a1_shutdown(struct platform_device *pdev)
{
	DPRINT("start %s\n", __func__);

	lcdc_s6d05a1_panel_off(pdev);
}

static struct platform_driver this_driver = {
	.probe  = s6d05a1_probe,
	.shutdown	= s6d05a1_shutdown,
	.driver = {
		.name   = "lcdc_s6d05a1_hvga",
	},
};

static struct msm_fb_panel_data s6d05a1_panel_data = {
	.on = lcdc_s6d05a1_panel_on,
	.off = lcdc_s6d05a1_panel_off,
	.set_backlight = lcdc_s6d05a1_set_backlight,
};

static struct platform_device this_device = {
	.name   = "lcdc_s6d05a1_hvga",
	.id	= 1,
	.dev	= {
		.platform_data = &s6d05a1_panel_data,
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

static int __init lcdc_s6d05a1_panel_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

#ifdef CONFIG_FB_MSM_LCDC_AUTO_DETECT
	if (msm_fb_detect_client("lcdc_s6d05a1_hvga")) {
		printk(KERN_ERR "%s: detect another lcd driver!\n", __func__);
		return 0;
	}
#endif
	DPRINT("start %s\n", __func__);

	update_backlight_table(BACKLIGHT_S6D05A1);

	ret = platform_driver_register(&this_driver);
	if (ret) {
		printk(KERN_ERR "%s: platform_driver_register failed! ret=%d\n",
						__func__, ret);
		return ret;
	}

	pinfo = &s6d05a1_panel_data.panel_info;
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
static irqreturn_t trebon_disp_breakdown_det(int irq, void *handle)
{
	if (s6d05a1_state.disp_initialized)
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

	lcdc_s6d05a1_panel_off(NULL);

	lcdc_s6d05a1_panel_on(NULL);
}
#endif

module_init(lcdc_s6d05a1_panel_init);
