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
/*#define USE_PREVAIL2*/

#ifdef LCDC_DEBUG
#define DPRINT(x...)	printk(KERN_INFO "LCD(nt35510) " x)
#else
#define DPRINT(x...)
#endif

static int spi_cs;
static int spi_sclk;
static int spi_sdi;
/*static int spi_dac;*/
static int lcd_reset;
static int spi_sdo;

/*#define FEATURE_LCD_ESD_DET*/


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


struct nt35510_state_type {
	boolean disp_initialized;
	boolean display_on;
	boolean disp_powered_up;
};

static struct nt35510_state_type nt35510_state = { 0 };
static struct msm_panel_common_pdata *lcdc_nt35510_pdata;

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
/*static char sleep_in_seq[1] = { 0x10 };*/
/*static char sw_reset_seq[1] = { 0x01 };*/

static char power_setting_seq1[6] = {
	0xF0,
	0x55, 0xAA, 0x52, 0x08, 0x01
};

static char power_setting_seq2[4] = {
	0xB0,
	0x09, 0x09, 0x09
};

static char power_setting_seq3[4] = {
	0xB6,
	0x34, 0x34, 0x34
};

static char power_setting_seq4[4] = {
	0xB1,
	0x09, 0x09, 0x09
};

static char power_setting_seq5[4] = {
	0xB7,
	0x24, 0x24, 0x24
};

static char power_setting_seq6[4] = {
	0xB3,
	0x05, 0x05, 0x05
};

static char power_setting_seq7[4] = {
	0xB9,
	0x24, 0x24, 0x24
};

static char power_setting_seq8[2] = {
	0xBF,
	0x01
};

static char power_setting_seq9[4] = {
	0xB5,
	0x0B, 0x0B, 0x0B
};

static char power_setting_seq10[4] = {
	0xBA,
	0x34, 0x34, 0x34
};

static char power_setting_seq11[4] = {
	0xBC,
	0x00, 0xA3, 0x00
};

static char power_setting_seq12[4] = {
	0xBD,
	0x00, 0xA3, 0x00
};

static char init_seq1[6] = {
	0xF0,
	0x55, 0xAA, 0x52, 0x08, 0x00
};

static char init_seq2[2] = {
	0xB0,
	0x0B
};

static char init_seq3[2] = {
	0xB6,
	0x0A
};

static char init_seq4[3] = {
	0xB7,
	0x00, 0x00
};

static char init_seq5[5] = {
	0xB8,
	0x01, 0x05, 0x05, 0x05
};

static char init_seq6[2] = {
	0xBA,
	0x01
};

static char init_seq7[4] = {
	0xBC,
	0x00, 0x00, 0x00
};

static char init_seq8[6] = {
	0xBD,
	0x00, 0xA3, 0x07, 0x31, 0x00
};

static char init_seq9[4] = {
	0xCC,
	0x03, 0x00, 0x00
};

static char init_seq10[3] = {
	0xB1,
	0x09, 0x06
};

static char init_seq11[2] = {
	0x36,
	0x00
};

static char init_seq12[2] = {
	0x3A,
	0x77
};


/*
static char display_parameter_setting1[5] = {
	0x2A,
	0x00, 0x00, 0x01, 0x3F
};

static char display_parameter_setting2[5] = {
	0x2B,
	0x00, 0x00, 0x01, 0xDF
};

static char display_parameter_setting3[2] = {
	0xB0,
	0x80
};

static char display_parameter_setting4[3] = {
	0xB1,
	0xB0, 0x11
};

static char display_parameter_setting5[2] = {
	0xB4,
	0x02
};

static char display_parameter_setting6[5] = {
	0xB5,
	0x08, 0x0C, 0x10, 0x0A
};

static char display_parameter_setting7[4] = {
	0xB6,
	0x20, 0x22, 0x3B
};

static char display_parameter_setting8[2] = {
	0xB7,
	0x07
};

static char display_parameter_setting9[1] = {
	0x20
};

static char display_parameter_setting10[2] = {
	0x13,
	0x38
};

static char display_parameter_setting11[2] = {
	0x3A,
	0x66
};

*/

/*Gamma Setting 2.6 */
static char gamma_set_seq1[53] = {
	/* Red+ gamma */
	0xD1,
	0x00, 0x37, 0x00, 0x64, 0x00,
	0x84, 0x00, 0xA9, 0x00, 0xBF,
	0x00, 0xE4, 0x01, 0x01, 0x01,
	0x35, 0x01, 0x5A, 0x01, 0x97,
	0x01, 0xC6, 0x02, 0x15, 0x02,
	0x52, 0x02, 0x54, 0x02, 0x8A,
	0x02, 0xC5, 0x02, 0xE8, 0x03,
	0x14, 0x03, 0x32, 0x03, 0x5D,
	0x03, 0x73, 0x03, 0x91, 0x03,
	0xA0, 0x03, 0xAF, 0x03, 0xBA,
	0x03, 0xC1
};

static char gamma_set_seq2[53] = {
	/* Red- gamma */
	0xD4,
	0x00, 0x37, 0x00, 0x64, 0x00,
	0x84, 0x00, 0xA9, 0x00, 0xBF,
	0x00, 0xE4, 0x01, 0x01, 0x01,
	0x35, 0x01, 0x5A, 0x01, 0x97,
	0x01, 0xC6, 0x02, 0x15, 0x02,
	0x52, 0x02, 0x54, 0x02, 0x8A,
	0x02, 0xC5, 0x02, 0xE8, 0x03,
	0x14, 0x03, 0x32, 0x03, 0x5D,
	0x03, 0x73, 0x03, 0x91, 0x03,
	0xA0, 0x03, 0xAF, 0x03, 0xBA,
	0x03, 0xC1

};

/*Gamma Setting 2.6 */
static char gamma_set_seq3[53] = {
	/* Green+ gamma */
	0xD2,
	0x00, 0x37, 0x00, 0x64, 0x00,
	0x84, 0x00, 0xA9, 0x00, 0xBF,
	0x00, 0xE4, 0x01, 0x01, 0x01,
	0x35, 0x01, 0x5A, 0x01, 0x97,
	0x01, 0xC6, 0x02, 0x15, 0x02,
	0x52, 0x02, 0x54, 0x02, 0x8A,
	0x02, 0xC5, 0x02, 0xE8, 0x03,
	0x14, 0x03, 0x32, 0x03, 0x5D,
	0x03, 0x73, 0x03, 0x91, 0x03,
	0xA0, 0x03, 0xAF, 0x03, 0xBA,
	0x03, 0xC1
};

static char gamma_set_seq4[53] = {
	/* Green- gamma */
	0xD5,
	0x00, 0x37, 0x00, 0x64, 0x00,
	0x84, 0x00, 0xA9, 0x00, 0xBF,
	0x00, 0xE4, 0x01, 0x01, 0x01,
	0x35, 0x01, 0x5A, 0x01, 0x97,
	0x01, 0xC6, 0x02, 0x15, 0x02,
	0x52, 0x02, 0x54, 0x02, 0x8A,
	0x02, 0xC5, 0x02, 0xE8, 0x03,
	0x14, 0x03, 0x32, 0x03, 0x5D,
	0x03, 0x73, 0x03, 0x91, 0x03,
	0xA0, 0x03, 0xAF, 0x03, 0xBA,
	0x03, 0xC1
};

/*Gamma Setting 2.6 */
static char gamma_set_seq5[53] = {
	/* Blue+ gamma */
	0xD3,
	0x00, 0x37, 0x00, 0x64, 0x00,
	0x84, 0x00, 0xA9, 0x00, 0xBF,
	0x00, 0xE4, 0x01, 0x01, 0x01,
	0x35, 0x01, 0x5A, 0x01, 0x97,
	0x01, 0xC6, 0x02, 0x15, 0x02,
	0x52, 0x02, 0x54, 0x02, 0x8A,
	0x02, 0xC5, 0x02, 0xE8, 0x03,
	0x14, 0x03, 0x32, 0x03, 0x5D,
	0x03, 0x73, 0x03, 0x91, 0x03,
	0xA0, 0x03, 0xAF, 0x03, 0xBA,
	0x03, 0xC1
};

static char gamma_set_seq6[53] = {
	/* Blue- gamma */
	0xD6,
	0x00, 0x37, 0x00, 0x64, 0x00,
	0x84, 0x00, 0xA9, 0x00, 0xBF,
	0x00, 0xE4, 0x01, 0x01, 0x01,
	0x35, 0x01, 0x5A, 0x01, 0x97,
	0x01, 0xC6, 0x02, 0x15, 0x02,
	0x52, 0x02, 0x54, 0x02, 0x8A,
	0x02, 0xC5, 0x02, 0xE8, 0x03,
	0x14, 0x03, 0x32, 0x03, 0x5D,
	0x03, 0x73, 0x03, 0x91, 0x03,
	0xA0, 0x03, 0xAF, 0x03, 0xBA,
	0x03, 0xC1
};

static struct spi_cmd_desc display_on_cmds[] = {
	{sizeof(power_setting_seq1), power_setting_seq1, 0},
	{sizeof(power_setting_seq2), power_setting_seq2, 0},
	{sizeof(power_setting_seq3), power_setting_seq3, 0},
	{sizeof(power_setting_seq4), power_setting_seq4, 0},
	{sizeof(power_setting_seq5), power_setting_seq5, 0},
	{sizeof(power_setting_seq6), power_setting_seq6, 0},
	{sizeof(power_setting_seq7), power_setting_seq7, 0},
	{sizeof(power_setting_seq8), power_setting_seq8, 0},
	{sizeof(power_setting_seq9), power_setting_seq9, 0},
	{sizeof(power_setting_seq10), power_setting_seq10, 0},
	{sizeof(power_setting_seq11), power_setting_seq11, 0},
	{sizeof(power_setting_seq12), power_setting_seq12, 150},

/*
	{sizeof(display_parameter_setting1), display_parameter_setting1, 0},
	{sizeof(display_parameter_setting2), display_parameter_setting2, 0},
	{sizeof(display_parameter_setting3), display_parameter_setting3, 0},
	{sizeof(display_parameter_setting4), display_parameter_setting4, 0},
	{sizeof(display_parameter_setting5), display_parameter_setting5, 0},
	{sizeof(display_parameter_setting6), display_parameter_setting6, 0},
	{sizeof(display_parameter_setting7), display_parameter_setting7, 0},
	{sizeof(display_parameter_setting8), display_parameter_setting8, 0},
	{sizeof(display_parameter_setting9), display_parameter_setting9, 0},
	{sizeof(display_parameter_setting10), display_parameter_setting10, 0},
	{sizeof(display_parameter_setting11), display_parameter_setting11, 0},
*/
	{sizeof(gamma_set_seq1), gamma_set_seq1, 0},
	{sizeof(gamma_set_seq2), gamma_set_seq2, 0},
	{sizeof(gamma_set_seq3), gamma_set_seq3, 0},
	{sizeof(gamma_set_seq4), gamma_set_seq4, 0},
	{sizeof(gamma_set_seq5), gamma_set_seq5, 0},
	{sizeof(gamma_set_seq6), gamma_set_seq6, 0},

	{sizeof(init_seq1), init_seq1, 0},
	{sizeof(init_seq2), init_seq2, 0},
	{sizeof(init_seq3), init_seq3, 0},
	{sizeof(init_seq4), init_seq4, 0},
	{sizeof(init_seq5), init_seq5, 0},
	{sizeof(init_seq6), init_seq6, 0},
	{sizeof(init_seq7), init_seq7, 0},
	{sizeof(init_seq8), init_seq8, 0},
	{sizeof(init_seq9), init_seq9, 0},
	{sizeof(init_seq10), init_seq10, 0},
	{sizeof(init_seq11), init_seq11, 0},
	{sizeof(init_seq12), init_seq12, 0},

	{sizeof(sleep_out_seq), sleep_out_seq, 200},
	{sizeof(disp_on_seq), disp_on_seq, 200},
};

static struct spi_cmd_desc display_off_cmds[] = {
	{sizeof(disp_off_seq), disp_off_seq, 0},
};
/*
static struct spi_cmd_desc display_standby_in_cmds[] = {
	{sizeof(disp_off_seq), disp_off_seq, 150},
	{sizeof(sleep_in_seq), sleep_in_seq, 150},
};

static struct spi_cmd_desc display_standby_out_cmds[] = {
	{sizeof(sleep_out_seq), sleep_out_seq, 0},
	{sizeof(disp_on_seq), disp_on_seq, 0},
};
*/
static int lcdc_nt35510_panel_off(struct platform_device *pdev);

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
	spi_sclk = *(lcdc_nt35510_pdata->gpio_num);
	spi_cs   = *(lcdc_nt35510_pdata->gpio_num + 1);
	spi_sdi  = *(lcdc_nt35510_pdata->gpio_num + 2);
	lcd_reset  = *(lcdc_nt35510_pdata->gpio_num + 3);
	spi_sdo  = *(lcdc_nt35510_pdata->gpio_num + 4);


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

		msleep(150);
		gpio_set_value(lcd_reset, 1);
		msleep(20);
		gpio_set_value(lcd_reset, 0);
		msleep(20);
		gpio_set_value(lcd_reset, 1);
		msleep(150);
	} else {
		gpio_set_value(lcd_reset, 0);
	}
}

static void nt35510_disp_powerup(void)
{
	DPRINT("start %s, lcd_reset:gpio %d\n", __func__, lcd_reset);

	if (!nt35510_state.disp_powered_up && !nt35510_state.display_on) {
		/* Reset the hardware first */

		lcdc_panel_reset(1);
		/* Include DAC power up implementation here */

	    nt35510_state.disp_powered_up = TRUE;
	}
}

static void nt35510_disp_powerdown(void)
{
	DPRINT("start %s, lcd_reset:gpio %d\n", __func__, lcd_reset);

	/* turn off LDO */
	/*TODO: turn off LDO*/

	nt35510_state.disp_powered_up = FALSE;
}

void nt35510_disp_on(void)
{
	/*int i;*/
	pr_info("@LCDINIT@:DISPON sequence\n");
	/*DPRINT("start %s\n", __func__);*/

	if (nt35510_state.disp_powered_up && !nt35510_state.display_on) {
		/*mdelay(20);*/

		/* nt35510 setting */
		spi_cmds_tx(display_on_cmds, ARRAY_SIZE(display_on_cmds));

		nt35510_state.display_on = TRUE;
	}
}

void nt35510_sleep_off(void)
{
	/*int i = 0;*/
	DPRINT("start %s\n", __func__);

	spi_cmds_tx(display_on_cmds, ARRAY_SIZE(display_on_cmds));
}

void nt35510_sleep_in(void)
{
	/*int i = 0;*/
	DPRINT("start %s\n", __func__);

	spi_cmds_tx(display_off_cmds, ARRAY_SIZE(display_off_cmds));

	lcd_prf = 0;
}

static int lcdc_nt35510_panel_on(struct platform_device *pdev)
{
	static int bring_up_condition;
	/*unsigned size;*/

	DPRINT("start %s, bring_up %d, disp_initialized %d\n",
		__func__, bring_up_condition, nt35510_state.disp_initialized);

	if (!bring_up_condition) {
		/* trick initalization for timing issue */
		bring_up_condition = 1;

		/* Configure reset GPIO that drives DAC */
		spi_init();	/* LCD needs SPI */
		lcdc_nt35510_pdata->panel_config_gpio(1);
		nt35510_state.disp_powered_up = TRUE;
		nt35510_state.display_on = TRUE;
		nt35510_state.disp_initialized = TRUE;
	} else {
		if (!nt35510_state.disp_initialized) {
			/* Configure reset GPIO that drives DAC */
			nt35510_disp_powerup();
			spi_init();	/* LCD needs SPI */
			lcdc_nt35510_pdata->panel_config_gpio(1);
			nt35510_disp_on();
			nt35510_state.disp_initialized = TRUE;

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

static int lcdc_nt35510_panel_off(struct platform_device *pdev)
{
	/*int i;*/

	DPRINT("start %s\n", __func__);


	if (nt35510_state.disp_powered_up && nt35510_state.display_on) {
#ifdef ESD_RECOVERY
		disable_irq_nosync(lcd_det_irq);
		irq_disabled = TRUE;
#endif

		spi_cmds_tx(display_off_cmds, ARRAY_SIZE(display_off_cmds));


		lcdc_nt35510_pdata->panel_config_gpio(0);
		nt35510_state.display_on = FALSE;
		nt35510_state.disp_initialized = FALSE;
		nt35510_disp_powerdown();
		lcd_prf = 0;
	}

	return 0;
}

static void lcdc_nt35510_set_backlight(struct msm_fb_data_type *mfd)
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

		while (!nt35510_state.disp_initialized) {
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

static int __devinit nt35510_probe(struct platform_device *pdev)
{
	/*int err;*/
	/*int ret;*/
	/*unsigned size;*/
	DPRINT("start %s\n", __func__);

	nt35510_state.disp_initialized = TRUE; /*signal_timing*/
	nt35510_state.disp_powered_up = TRUE;
	nt35510_state.display_on = TRUE;

	if (pdev->id == 0) {
		lcdc_nt35510_pdata = pdev->dev.platform_data;
		if (!lcdc_nt35510_pdata)
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
		return 0;
	}
	msm_fb_add_device(pdev);

	return 0;
}

static void nt35510_shutdown(struct platform_device *pdev)
{
	DPRINT("start %s\n", __func__);

	lcdc_nt35510_panel_off(pdev);
}

static struct platform_driver this_driver = {
	.probe  = nt35510_probe,
	.shutdown	= nt35510_shutdown,
	.driver = {
		.name   = "lcdc_nt35510_wvga",
	},
};

static struct msm_fb_panel_data nt35510_panel_data = {
	.on = lcdc_nt35510_panel_on,
	.off = lcdc_nt35510_panel_off,
	.set_backlight = lcdc_nt35510_set_backlight,
};

static struct platform_device this_device = {
	.name   = "lcdc_panel",
	.id	= 1,
	.dev	= {
		.platform_data = &nt35510_panel_data,
	}
};

#define LCDC_FB_XRES	480
#define LCDC_FB_YRES	800
#define LCDC_HBP		18
#define LCDC_HPW		2
#define LCDC_HFP		5
#define LCDC_VBP		5
#define LCDC_VPW		2
#define LCDC_VFP		5

static int __init lcdc_nt35510_panel_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

#ifdef CONFIG_FB_MSM_LCDC_AUTO_DETECT
	if (msm_fb_detect_client("lcdc_nt35510_wvga")) {
		printk(KERN_ERR "%s: detect another lcd driver!\n", __func__);
		return 0;
	}
#endif
	DPRINT("start %s\n", __func__);

	update_backlight_table(BACKLIGHT_NT35510);

	ret = platform_driver_register(&this_driver);
	if (ret) {
		printk(KERN_ERR "%s: platform_driver_register failed! ret=%d\n",
						__func__, ret);
		return ret;
	}

	pinfo = &nt35510_panel_data.panel_info;
	pinfo->xres = LCDC_FB_XRES;
	pinfo->yres = LCDC_FB_YRES;
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 24;
	pinfo->fb_num = 2;
	pinfo->clk_rate = 24576000;/*24576000;*/
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
	if (nt35510_state.disp_initialized)
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

	lcdc_nt35510_panel_off(NULL);

	lcdc_nt35510_panel_on(NULL);
}
#endif

module_init(lcdc_nt35510_panel_init);
