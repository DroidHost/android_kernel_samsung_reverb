/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#ifdef CONFIG_MACH_ICON
#include "sr130pc10_icon.h"
#else
#include "sr130pc10.h"
#endif
#include <linux/slab.h>
#include <mach/vreg.h>

#include <mach/camera.h>

/*#define SENSOR_DEBUG 0*/
#undef CONFIG_LOAD_FILE
/*#define CONFIG_LOAD_FILE*/

#define CAM_RESET 130
#define CAM_STANDBY 131
#define CAM_EN 3
#define CAM_EN_1 132
#define CAM_EN_2 132
#define VCAM_I2C_SCL 177
#define VCAM_I2C_SDA 174
#define CAM_VT_nSTBY 2		/* yjlee : add */
#define CAM_VT_RST 175		/* yjlee : add */
#define CAM_MCLK 15			/* yjlee : add */

#ifdef CONFIG_MACH_ICON
#define CAM_EN_1	143
#endif

#define PCAM_CONNECT_CHECK		0
#define PCAM_VT_MODE			1
#define PCAM_EXPOSURE_TIME		2
#define PCAM_ISO_SPEED			3
#define PCAM_FIXED_FRAME		4
#define PCAM_AUTO_FRAME			5
#define PCAM_NIGHT_SHOT			6
#define PCAM_FLASH_OFF			7
#define PCAM_MOVIE_FLASH_ON		8
#define PCAM_PREVIEW_FPS		9
#define PCAM_GET_FLASH			10
#define PCAM_LOW_TEMP			11

struct sr130pc10_work_t {
	struct work_struct work;
};
static struct sr130pc10_work_t *sr130pc10_sensorw;
#ifndef CONFIG_MACH_ICON
static struct i2c_client *sr130pc10_client;
#endif
struct sr130pc10_ctrl_t {
	const struct msm_camera_sensor_info *sensordata;
};
static struct sr130pc10_ctrl_t *sr130pc10_ctrl;

static DECLARE_WAIT_QUEUE_HEAD(sr130pc10_wait_queue);
/*DECLARE_MUTEX(sr130pc10_sem);*/

#ifdef CONFIG_LOAD_FILE
static int sr130pc10_regs_table_write(char *name);
#endif
static int cam_hw_init(void);
/*static int16_t sr130pc10_effect = CAMERA_EFFECT_OFF; */
static int rotation_status;
static int factory_test;
static int Flipmode;

struct device *sr130pc10_dev;

static ssize_t camtype_file_cmd_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char camType[] = "SILICONFILE_SR130PC10_NONE";

	return snprintf(buf, 30, "%s", camType);
}

static ssize_t camtype_file_cmd_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(
front_camtype, 0660, camtype_file_cmd_show, camtype_file_cmd_store);

static ssize_t front_camera_firmware_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	char cam_fw[] = "SR130PC10\n";
	return snprintf(buf, sizeof(cam_fw), "%s", cam_fw);
}

static DEVICE_ATTR(
front_camfw, 0664, front_camera_firmware_show, NULL);

/*=============================================================
	EXTERNAL DECLARATIONS
==============================================================*/
/*extern struct sr130pc10_reg sr130pc10_regs;*/

/*=============================================================*/

static int sr130pc10_sensor_read(unsigned short subaddr, unsigned short *data)
{
	/*printk(KERN_DEBUG "<=ASWOOGI=> sr130pc10_sensor_read\n");*/

	int ret;
	unsigned char buf[1] = { 0 };
	struct i2c_msg msg = { sr130pc10_client->addr, 0, 1, buf };

	buf[0] = subaddr;
/*      buf[1] = 0x0; */

	ret = i2c_transfer(sr130pc10_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO)
		goto error;

	msg.flags = I2C_M_RD;

	ret = i2c_transfer(sr130pc10_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO)
		goto error;

/*      *data = ((buf[0] << 8) | buf[1]); */
	*data = buf[0];

 error:
	/*printk(KERN_DEBUG "[ASWOOGI] on read func
		sr130pc10_client->addr : %x\n",  sr130pc10_client->addr); */
	/*printk(KERN_DEBUG "[ASWOOGI] on read func
		subaddr : %x\n", subaddr); */
	/*printk(KERN_DEBUG "[ASWOOGI] on read func
		data : %x\n", data); */

	return ret;
}

static int sr130pc10_sensor_write(unsigned short subaddr, unsigned short val)
{
	unsigned char buf[2] = { 0 };
	struct i2c_msg msg = { sr130pc10_client->addr, 0, 2, buf };
/*      printk(KERN_DEBUG "[SR130PC10] on write func
	sr130pc10_client->addr : %x\n", subaddr); */
/*      printk(KERN_DEBUG "[SR130PC10] on write func
	sr130pc10_client->adapter->nr : %x\n", val); */
	buf[0] = subaddr;
	buf[1] = val;

	if (i2c_transfer(sr130pc10_client->adapter, &msg, 1) == 1) {
		return 0;
	} else {
		printk(KERN_DEBUG "[sr130pc10] sr130pc10_sensor_write fail\n");
		return -EIO;
	}
}

static int
sr130pc10_sensor_write_list(struct sr130pc10_short_t *list,
							int size, char *name)
{
	int ret = 0;
#ifdef CONFIG_LOAD_FILE
	ret = sr130pc10_regs_table_write(name);
#else
	int i;

	pr_info("[%s] %s", __func__, name);
	for (i = 0; i < size; i++) {
		if (list[i].subaddr == 0xff) {
			printk(KERN_DEBUG "<=PCAM=> now SLEEP!!!!\n");
			msleep(list[i].value * 8);
		} else {
			if (sr130pc10_sensor_write
			    (list[i].subaddr, list[i].value) < 0) {
				printk(KERN_DEBUG
				    "<=PCAM=> sensor_write_list fail..\n");
				return -EINVAL;
			}
		}
	}
#endif
	return ret;
}

static long sr130pc10_set_sensor_mode(int mode)
{
	int cnt, vsync_value;
	printk(KERN_DEBUG "[CAM-SENSOR] =Sensor Mode\n ");

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		printk(KERN_DEBUG "[SR130PC10]-> Preview\n");
		factory_test = 0;
		for (cnt = 0; cnt < 200; cnt++) {
			vsync_value = gpio_get_value(14);
			if (vsync_value) {
				/*printk(KERN_DEBUG " on preview,
			start cnt:%d vsync_value:%d\n", cnt, vsync_value); */
				break;
			} else {
				printk(KERN_DEBUG
				    " on preview,  "
					"wait cnt:%d vsync_value:%d\n",
				     cnt, vsync_value);
				/*msleep(1);	changed for coding rule*/
				udelay(1000);
			}
		}

		sr130pc10_sensor_write_list(sr130pc10_preview_reg,
			sizeof(sr130pc10_preview_reg) /
			sizeof(sr130pc10_preview_reg[0]),
			"sr130pc10_preview_reg");	/* preview start */
		break;

	case SENSOR_SNAPSHOT_MODE:
		printk(KERN_DEBUG "[PGH}-> Capture\n");
		if (Flipmode) {
			sr130pc10_sensor_write_list(
				sr130pc10_capture_reg_X_Flip,
				sizeof(sr130pc10_capture_reg_X_Flip) /
				sizeof(sr130pc10_capture_reg_X_Flip[0]),
				"sr130pc10_capture_reg_X_Flip");
			/* preview start */
		} else {
		sr130pc10_sensor_write_list(sr130pc10_capture_reg,
			sizeof(sr130pc10_capture_reg) /
			sizeof(sr130pc10_capture_reg[0]),
			"sr130pc10_capture_reg");	/* preview start */
		}
		   /*SecFeature : for Android CCD preview mirror
			/ snapshot non-mirror
		   if(factory_test == 0) {
			   if(rotation_status == 90 || rotation_status
				== 270) {
				   sr130pc10_sensor_write(0x03, 0x00);
				   sr130pc10_sensor_write(0x11, 0x93);
			   } else {
				   sr130pc10_sensor_write(0x03, 0x00);
				   sr130pc10_sensor_write(0x11, 0x90);
			   }
		   }
		 */
		break;

	case SENSOR_RAW_SNAPSHOT_MODE:
		printk(KERN_DEBUG "[PGH}-> Capture RAW\n");
		break;
	default:
		return 0;
	}

	return 0;
}

static int sr130pc10_set_movie_mode(int mode)
{
	printk(KERN_DEBUG "[%s:%d][E]\n", __func__, __LINE__);

	if (mode == SENSOR_MOVIE) {
		printk(KERN_DEBUG
			"[%s:%d] Camcorder_Mode_ON\n", __func__, __LINE__);

		sr130pc10_sensor_write_list(sr130pc10_fps_15,
					    sizeof(sr130pc10_fps_15) /
					    sizeof(sr130pc10_fps_15[0]),
					    "sr130pc10_fps_15");
	} else

	if ((mode != SENSOR_CAMERA) && (mode != SENSOR_MOVIE))
		return -EINVAL;

	return 0;
}

static int sr130pc10_exif_iso(void)
{
	uint32_t iso_value = 0;
	int err = 0;

	unsigned short gain;
	int calc_gain;

	sr130pc10_sensor_write(0x03, 0x20);
	sr130pc10_sensor_read(0xb0, &gain);
	calc_gain = (gain/16) * 10000;

	if (calc_gain < 8750)
		iso_value = 50;
	else if (calc_gain < 17500)
		iso_value = 100;
	else if (calc_gain < 46250)
		iso_value = 200;
	else if (calc_gain < 69375)
		iso_value = 400;
	else if (calc_gain < 138750)
		iso_value = 800;
	else
		iso_value = 1600;

	pr_info("[%s]iso_value(%d)\n", __func__, iso_value);
	return (int)iso_value ;
}

static int sr130pc10_exif_shutter_speed(void)
{
	uint32_t shutter_speed = 0;
	int err = 0;

	unsigned short val1, val2, val3;

	sr130pc10_sensor_write(0x03, 0x20);
	sr130pc10_sensor_read(0x80, &val1);
	sr130pc10_sensor_read(0x81, &val2);
	sr130pc10_sensor_read(0x82, &val3);

	shutter_speed = 24000000 / ((val1<<19) + (val2<<11)+(val3<<3));

	pr_info("[%s]shutter_speed(%d)\n", __func__, shutter_speed);
	return (int)shutter_speed;
}

static int
sr130pc10_set_flipmode(int val)
{
	printk(KERN_DEBUG "Enter [value = %d]\n", val);
	Flipmode = val;
	return 0;
}

static int sr130pc10_get_exif(int *exif_cmd, int *val)
{
	switch (*exif_cmd) {
	case EXIF_TV:
		(*val) = sr130pc10_exif_shutter_speed();
		break;

	case EXIF_ISO:
		(*val) = sr130pc10_exif_iso();
		break;

	default:
		printk(KERN_DEBUG
			"[%s:%d] invalid(%d)\n",
			__func__, __LINE__, *exif_cmd);
		break;
	}

	return 0;
}


static long sr130pc10_set_effect(int mode, int effect)
{
	long rc = 0;

	pr_info("[%s]effect(%d)\n", __func__, effect);

	switch (effect) {
	case CAMERA_EFFECT_OFF:
		printk(KERN_DEBUG "[SR130PC10] CAMERA_EFFECT_OFF\n");
		sr130pc10_sensor_write_list(sr130pc10_effect_none,
					    sizeof(sr130pc10_effect_none) /
					    sizeof(sr130pc10_effect_none[0]),
					    "sr130pc10_effect_none");
		break;

	case CAMERA_EFFECT_MONO:
		printk(KERN_DEBUG "[SR130PC10] CAMERA_EFFECT_MONO\n");
		sr130pc10_sensor_write_list(sr130pc10_effect_gray,
					    sizeof(sr130pc10_effect_gray) /
					    sizeof(sr130pc10_effect_gray[0]),
					    "sr130pc10_effect_gray");
		break;

	case CAMERA_EFFECT_NEGATIVE:
		printk(KERN_DEBUG "[SR130PC10] CAMERA_EFFECT_NEGATIVE\n");
		sr130pc10_sensor_write_list(sr130pc10_effect_negative,
					    sizeof(sr130pc10_effect_negative) /
					    sizeof(sr130pc10_effect_negative
						   [0]),
					    "sr130pc10_effect_negative");
		break;

	case CAMERA_EFFECT_SEPIA:
		printk(KERN_DEBUG "[SR130PC10] CAMERA_EFFECT_SEPIA\n");
		sr130pc10_sensor_write_list(sr130pc10_effect_sepia,
					    sizeof(sr130pc10_effect_sepia) /
					    sizeof(sr130pc10_effect_sepia[0]),
					    "sr130pc10_effect_sepia");
		break;

	case CAMERA_EFFECT_AQUA:
		printk(KERN_DEBUG "[SR130PC10] CAMERA_EFFECT_AQUA\n");
		sr130pc10_sensor_write_list(sr130pc10_effect_aqua,
					    sizeof(sr130pc10_effect_aqua) /
					    sizeof(sr130pc10_effect_aqua[0]),
					    "sr130pc10_effect_aqua");
		break;

	default:
		printk(KERN_DEBUG "[SR130PC10] default .dsfsdf\n");
		sr130pc10_sensor_write_list(sr130pc10_effect_none,
					    sizeof(sr130pc10_effect_none) /
					    sizeof(sr130pc10_effect_none[0]),
					    "sr130pc10_effect_none");
		/*return -EINVAL; */
		return 0;
	}
	return rc;
}

static long sr130pc10_set_exposure_value(int mode, int exposure)
{
	long rc = 0;

	printk(KERN_DEBUG "mode : %d, exposure value  : %d\n", mode, exposure);

	switch (exposure) {

	case CAMERA_EXPOSURE_NEGATIVE_4:
		printk(KERN_DEBUG "CAMERA_EXPOSURE_VALUE_-4\n");
		sr130pc10_sensor_write_list(sr130pc10_ev_m4,
					    sizeof(sr130pc10_ev_m4) /
					    sizeof(sr130pc10_ev_m4[0]),
					    "sr130pc10_ev_m4");
		break;

	case CAMERA_EXPOSURE_NEGATIVE_3:
		printk(KERN_DEBUG "CAMERA_EXPOSURE_VALUE_-3\n");
		sr130pc10_sensor_write_list(sr130pc10_ev_m3,
					    sizeof(sr130pc10_ev_m3) /
					    sizeof(sr130pc10_ev_m3[0]),
					    "sr130pc10_ev_m3");
		break;

	case CAMERA_EXPOSURE_NEGATIVE_2:
		printk(KERN_DEBUG "CAMERA_EXPOSURE_VALUE_-2\n");
		sr130pc10_sensor_write_list(sr130pc10_ev_m2,
					    sizeof(sr130pc10_ev_m2) /
					    sizeof(sr130pc10_ev_m2[0]),
					    "sr130pc10_ev_m2");

		break;

	case CAMERA_EXPOSURE_NEGATIVE_1:
		printk(KERN_DEBUG "CAMERA_EXPOSURE_VALUE_-1\n");
		sr130pc10_sensor_write_list(sr130pc10_ev_m1,
					    sizeof(sr130pc10_ev_m1) /
					    sizeof(sr130pc10_ev_m1[0]),
					    "sr130pc10_ev_m1");

		break;

	case CAMERA_EXPOSURE_0:
		printk(KERN_DEBUG "CAMERA_EXPOSURE_VALUE_0\n");
		sr130pc10_sensor_write_list(sr130pc10_ev_default,
					    sizeof(sr130pc10_ev_default) /
					    sizeof(sr130pc10_ev_default[0]),
					    "sr130pc10_ev_default");

		break;

	case CAMERA_EXPOSURE_POSITIVE_1:
		printk(KERN_DEBUG "CAMERA_EXPOSURE_VALUE_1\n");
		sr130pc10_sensor_write_list(sr130pc10_ev_p1,
					    sizeof(sr130pc10_ev_p1) /
					    sizeof(sr130pc10_ev_p1[0]),
					    "sr130pc10_ev_p1");

		break;

	case CAMERA_EXPOSURE_POSITIVE_2:
		printk(KERN_DEBUG "CAMERA_EXPOSURE_VALUE_2\n");
		sr130pc10_sensor_write_list(sr130pc10_ev_p2,
					    sizeof(sr130pc10_ev_p2) /
					    sizeof(sr130pc10_ev_p2[0]),
					    "sr130pc10_ev_p2");
		break;

	case CAMERA_EXPOSURE_POSITIVE_3:
		printk(KERN_DEBUG "CAMERA_EXPOSURE_VALUE_3\n");
		sr130pc10_sensor_write_list(sr130pc10_ev_p3,
					    sizeof(sr130pc10_ev_p3) /
					    sizeof(sr130pc10_ev_p3[0]),
					    "sr130pc10_ev_p3");
		break;

	case CAMERA_EXPOSURE_POSITIVE_4:
		printk(KERN_DEBUG "CAMERA_EXPOSURE_VALUE_4\n");
		sr130pc10_sensor_write_list(sr130pc10_ev_p4,
					    sizeof(sr130pc10_ev_p4) /
					    sizeof(sr130pc10_ev_p4[0]),
					    "sr130pc10_ev_p4");
		break;
	default:
		printk(KERN_DEBUG "<=PCAM=> unexpected Exposure Value %s/%d\n",
			 __func__, __LINE__);
/*                      return -EINVAL; */
		return 0;
	}
	return rc;
}

static long sr130pc10_set_whitebalance(int mode, int wb)
{
	long rc = 0;

	printk(KERN_DEBUG "mode : %d,   whitebalance : %d\n", mode, wb);

	switch (wb) {
	case CAMERA_WB_AUTO:
		printk(KERN_DEBUG "CAMERA_WB_AUTO\n");
		sr130pc10_sensor_write_list(sr130pc10_wb_auto,
				    sizeof(sr130pc10_wb_auto) /
				    sizeof(sr130pc10_wb_auto[0]),
				    "sr130pc10_wb_auto");
		break;

	case CAMERA_WB_INCANDESCENT:
		printk(KERN_DEBUG "CAMERA_WB_INCANDESCENT\n");
		sr130pc10_sensor_write_list(sr130pc10_wb_tungsten,
				    sizeof(sr130pc10_wb_tungsten) /
				    sizeof(sr130pc10_wb_tungsten[0]),
				    "sr130pc10_wb_tungsten");
		break;

	case CAMERA_WB_FLUORESCENT:
		printk(KERN_DEBUG "CAMERA_WB_FLUORESCENT\n");
		sr130pc10_sensor_write_list(sr130pc10_wb_fluorescent,
				    sizeof(sr130pc10_wb_fluorescent) /
				    sizeof(sr130pc10_wb_fluorescent[0]),
				    "sr130pc10_wb_fluorescent");
		break;

	case CAMERA_WB_DAYLIGHT:
		printk(KERN_DEBUG "<=PCAM=> CAMERA_WB_DAYLIGHT\n");
		sr130pc10_sensor_write_list(sr130pc10_wb_sunny,
					    sizeof(sr130pc10_wb_sunny) /
					    sizeof(sr130pc10_wb_sunny[0]),
					    "sr130pc10_wb_sunny");
		break;

	case CAMERA_WB_CLOUDY_DAYLIGHT:
		printk(KERN_DEBUG "<=PCAM=> CAMERA_WB_CLOUDY_DAYLIGHT\n");
		sr130pc10_sensor_write_list(sr130pc10_wb_cloudy,
					    sizeof(sr130pc10_wb_cloudy) /
					    sizeof(sr130pc10_wb_cloudy[0]),
					    "sr130pc10_wb_cloudy");
		break;

	default:
		printk(KERN_DEBUG "<=PCAM=> unexpected WB mode %s/%d\n",
			__func__, __LINE__);
/*                      return -EINVAL; */
		return 0;
	}
	return rc;
}

static long sr130pc10_set_rotation(int rotation)
{
	rotation_status = rotation;
	printk(KERN_DEBUG "[SR130PC10] current rotation status : %d\n",
		rotation_status);

	return 0;
}

static int
sr130pc10_sensor_init_probe(const struct msm_camera_sensor_info *data)
{
	int err = 0;
	rotation_status = 0;
	factory_test = 0;
	Flipmode = 0;

	printk(KERN_DEBUG "%s/%d\n", __func__, __LINE__);

	err = sr130pc10_sensor_write_list(sr130pc10_reg_init,
					  sizeof(sr130pc10_reg_init) /
					  sizeof(sr130pc10_reg_init[0]),
					  "sr130pc10_reg_init");
	/*msleep(10);	changed for coding rule*/
	/*udelay(10000);*/

	return err;
}

static int sr130pc10_set_power(int enable)
{
	int rc = 0;
	struct vreg *vreg_L8;

#ifdef CONFIG_MACH_ICON
	if (enable == 1) {
		printk(KERN_DEBUG
				"<=PCAM=> ++++++++++++++++++++++++++"
				"sr130pc10 test driver"
				"++++++++++++++++++++++++++++++++++++\n");
		gpio_tlmm_config(GPIO_CFG(CAM_RESET, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
				GPIO_CFG_ENABLE);	/*CAM_RESET */
		gpio_tlmm_config(GPIO_CFG(CAM_STANDBY, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
				GPIO_CFG_ENABLE);	/*CAM_STANDBY */
		gpio_tlmm_config(GPIO_CFG(CAM_EN, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
				GPIO_CFG_ENABLE);	/*CAM_EN */
		gpio_tlmm_config(GPIO_CFG(CAM_EN_1, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
				GPIO_CFG_ENABLE);	/*CAM_EN_2 */
		gpio_tlmm_config(GPIO_CFG(CAM_EN_2, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
				GPIO_CFG_ENABLE);	/*CAM_EN_2 */
		gpio_tlmm_config(GPIO_CFG(CAM_VT_RST, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
				GPIO_CFG_ENABLE);	/*CAM_VT_RST */
		gpio_tlmm_config(GPIO_CFG(CAM_VT_nSTBY, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
				GPIO_CFG_ENABLE);	/*CAM_VT_nSTBY */

		vreg_L8 = vreg_get(NULL, "gp7");
		vreg_set_level(vreg_L8, 1800);
		vreg_disable(vreg_L8);

		gpio_set_value(CAM_RESET, 0);
		gpio_set_value(CAM_STANDBY, 0);
		gpio_set_value(CAM_EN, 0);
		gpio_set_value(CAM_EN_1, 0);
		gpio_set_value(CAM_EN_2, 0);
		gpio_set_value(CAM_VT_RST, 0);
		gpio_set_value(CAM_VT_nSTBY, 0);

		gpio_set_value(CAM_EN_2, 1);	/* CAM_D_1.2V */
		gpio_set_value(CAM_EN_1, 1);	/* CAM_IO_1.8V */
		gpio_set_value(CAM_EN, 1);	/* CAM_A_2.8V */
		vreg_enable(vreg_L8);	/* VCAM_1.8VDV */
		mdelay(1);
		gpio_set_value(CAM_VT_nSTBY, 1);
		mdelay(1);
	} else if (enable == 0) {
		mdelay(1);
		gpio_set_value(CAM_VT_nSTBY, 0);
		mdelay(1);
		vreg_disable(vreg_get(NULL, "gp7"));
		mdelay(1);
		gpio_set_value(CAM_EN, 0);	/* CAM_A_2.8V */
		mdelay(1);
		gpio_set_value(CAM_EN_1, 0);	/* CAM_IO_1.8V */
		mdelay(1);
		gpio_set_value(CAM_EN_2, 0);	/* CAM_D_1.2V */
	}
#else
	printk(KERN_DEBUG
	    "<=PCAM=> ++++++++++++++++++++++++++"
		"sr130pc10 test driver"
		"++++++++++++++++++++++++++++++++++++\n");
	gpio_tlmm_config(GPIO_CFG(CAM_RESET, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		GPIO_CFG_ENABLE);	/*CAM_RESET */
	gpio_tlmm_config(GPIO_CFG(CAM_STANDBY, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		GPIO_CFG_ENABLE);	/*CAM_STANDBY */
	gpio_tlmm_config(GPIO_CFG(CAM_EN, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		GPIO_CFG_ENABLE);	/*CAM_EN */
	gpio_tlmm_config(GPIO_CFG(CAM_EN_2, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		GPIO_CFG_ENABLE);	/*CAM_EN_2 */
	gpio_tlmm_config(GPIO_CFG(CAM_VT_RST, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		GPIO_CFG_ENABLE);	/*CAM_VT_RST */
	gpio_tlmm_config(GPIO_CFG(CAM_VT_nSTBY, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		GPIO_CFG_ENABLE);	/*CAM_VT_nSTBY */

	vreg_L8 = vreg_get(NULL, "gp7");
	vreg_set_level(vreg_L8, 1800);
	vreg_disable(vreg_L8);

	gpio_set_value(CAM_RESET, 0);
	gpio_set_value(CAM_STANDBY, 0);
	gpio_set_value(CAM_EN, 0);
	gpio_set_value(CAM_EN_2, 0);
	gpio_set_value(CAM_VT_RST, 0);
	gpio_set_value(CAM_VT_nSTBY, 0);

	/*mdelay(1);		changed for coding rule*/
	mdelay(1);

	gpio_set_value(CAM_EN_2, 1);	/*CAM_EN->UP */
	gpio_set_value(CAM_EN, 1); /*CAM_EN->UP*/
	vreg_enable(vreg_L8);
	udelay(10);

	gpio_set_value(CAM_VT_nSTBY, 1); /*VGA_STBY UP*/
	udelay(10);

	gpio_tlmm_config(GPIO_CFG(CAM_MCLK, 1, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		GPIO_CFG_ENABLE);	/*CAM_MCLK */
	msm_camio_clk_rate_set(24000000);	/*MCLK*/
	msm_camio_camif_pad_reg_reset();

	gpio_set_value(CAM_VT_RST, 1); /*VGA_RESET UP*/
	mdelay(1);
#endif
	return rc;
}
static int cam_hw_init()
{
	int rc = 0;
	struct vreg *vreg_L8;

#ifdef CONFIG_MACH_ICON
	msm_camio_clk_rate_set(24000000);	/*MCLK*/
	msm_camio_camif_pad_reg_reset();
	mdelay(10);

	gpio_set_value(CAM_VT_RST, 1);
	mdelay(1);
#else
	printk(KERN_DEBUG
	    "<=PCAM=> ++++++++++++++++++++++++++"
		"sr130pc10 test driver"
		"++++++++++++++++++++++++++++++++++++\n");
	gpio_tlmm_config(GPIO_CFG(CAM_RESET, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		GPIO_CFG_ENABLE);	/*CAM_RESET */
	gpio_tlmm_config(GPIO_CFG(CAM_STANDBY, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		GPIO_CFG_ENABLE);	/*CAM_STANDBY */
	gpio_tlmm_config(GPIO_CFG(CAM_EN, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		GPIO_CFG_ENABLE);	/*CAM_EN */
	gpio_tlmm_config(GPIO_CFG(CAM_EN_2, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		GPIO_CFG_ENABLE);	/*CAM_EN_2 */
	gpio_tlmm_config(GPIO_CFG(CAM_VT_RST, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		GPIO_CFG_ENABLE);	/*CAM_VT_RST */
	gpio_tlmm_config(GPIO_CFG(CAM_VT_nSTBY, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		GPIO_CFG_ENABLE);	/*CAM_VT_nSTBY */

	vreg_L8 = vreg_get(NULL, "gp7");
	vreg_set_level(vreg_L8, 1800);
	vreg_disable(vreg_L8);

	gpio_set_value(CAM_RESET, 0);
	gpio_set_value(CAM_STANDBY, 0);
	gpio_set_value(CAM_EN, 0);
	gpio_set_value(CAM_EN_2, 0);
	gpio_set_value(CAM_VT_RST, 0);
	gpio_set_value(CAM_VT_nSTBY, 0);

	/*mdelay(1);		changed for coding rule*/
	mdelay(1);

	gpio_set_value(CAM_EN_2, 1);	/*CAM_EN->UP */
	gpio_set_value(CAM_EN, 1); /*CAM_EN->UP*/
	vreg_enable(vreg_L8);
	udelay(10);

	gpio_set_value(CAM_VT_nSTBY, 1); /*VGA_STBY UP*/
	udelay(10);

	gpio_tlmm_config(GPIO_CFG(CAM_MCLK, 1, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		GPIO_CFG_ENABLE);	/*CAM_MCLK */
	msm_camio_clk_rate_set(24000000);	/*MCLK*/
	msm_camio_camif_pad_reg_reset();

	gpio_set_value(CAM_VT_RST, 1); /*VGA_RESET UP*/
	mdelay(1);
#endif
	return rc;
}

void sensor_rough_control_sr130pc10(void __user *arg)
{
	struct ioctl_pcam_info_8bit ctrl_info;

	/*
	int Exptime;
	int Expmax;
	unsigned short read_1, read_2, read_3;
	*/
	int rc = 0;

	printk(KERN_DEBUG "[SR130PC10] sensor_rough_control\n");

	if (copy_from_user
	    ((void *)&ctrl_info, (const void *)arg, sizeof(ctrl_info))) {
		printk(KERN_DEBUG
			"<=SR130PC10=> %s fail copy_from_user!\n", __func__);
	}
	printk(KERN_DEBUG "<=SR130PC10=> TEST %d %d %d %d %d\n",
		ctrl_info.mode, ctrl_info.address, ctrl_info.value_1,
		ctrl_info.value_2, ctrl_info.value_3);

	switch (ctrl_info.mode) {
	case PCAM_CONNECT_CHECK:
		printk(KERN_DEBUG "[SR130PC10] PCAM_CONNECT_CHECK\n");
		rc = sr130pc10_sensor_write(0x03, 0x00);
		if (rc < 0) {	/*check sensor connection */
			printk(KERN_DEBUG "[SR130PC10] Connect error\n");
			ctrl_info.value_1 = 1;
		}
		break;

	case PCAM_EXPOSURE_TIME:
		printk(KERN_DEBUG "[SR130PC10] PCAM_EXPOSURE_TIME\n");
		sr130pc10_sensor_write(0x03, 0x20);
		sr130pc10_sensor_read(0x80, &ctrl_info.value_1);
		sr130pc10_sensor_read(0x81, &ctrl_info.value_2);
		sr130pc10_sensor_read(0x82, &ctrl_info.value_3);
		printk(KERN_DEBUG
		    "[SR130PC10] PCAM_EXPOSURE_TIME : A(%x), B(%x), C(%x)\n]",
		     ctrl_info.value_1, ctrl_info.value_2, ctrl_info.value_3);
		break;

	case PCAM_ISO_SPEED:
		printk(KERN_DEBUG "[SR130PC10] PCAM_ISO_SPEED\n");
		sr130pc10_sensor_write(0x03, 0x20);
		sr130pc10_sensor_read(0xb0, &ctrl_info.value_1);
		break;

	case PCAM_PREVIEW_FPS:
		printk(KERN_DEBUG "[SR130PC10] PCAM_PREVIEW_FPS : %d\n",
		       ctrl_info.address);
		if (ctrl_info.address == 15)
			sr130pc10_sensor_write_list(sr130pc10_vt_fps_15,
					    sizeof(sr130pc10_vt_fps_15)
					    / sizeof(sr130pc10_vt_fps_15[0]),
						    "sr130pc10_vt_fps_15");
		break;

	default:
		printk(KERN_DEBUG "<=SR130PC10=> "
			"Unexpected mode on sensor_rough_control!!!\n");
		break;
	}

	if (copy_to_user
	    ((void *)arg, (const void *)&ctrl_info, sizeof(ctrl_info))) {
		printk(KERN_DEBUG
			"<=SR130PC10=> %s fail on copy_to_user!\n", __func__);
	}

}

#ifdef CONFIG_LOAD_FILE

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
/*#include <asm/uaccess.h>*/
#include <linux/uaccess.h>


static char *sr130pc10_regs_table;
static int sr130pc10_regs_table_size;

int sr130pc10_regs_table_init(void)
{
	struct file *filp;
	char *dp;
	long l;
	loff_t pos;
	int ret;
	mm_segment_t fs = get_fs();

	printk(KERN_DEBUG "%s %d\n", __func__, __LINE__);

	set_fs(get_ds());
/*
	//filp = filp_open("/data/camera/sr130pc10.h", O_RDONLY, 0);
	filp = filp_open("/data/sr130pc10.h", O_RDONLY, 0);
*/
	filp = filp_open("/mnt/sdcard/sr130pc10.h", O_RDONLY, 0);

	if (IS_ERR(filp)) {
		printk(KERN_DEBUG "file open error %d\n", PTR_ERR(filp));
		return -EINVAL;
	}
	l = filp->f_path.dentry->d_inode->i_size;
	printk(KERN_DEBUG "l = %ld\n", l);
	dp = kmalloc(l, GFP_KERNEL);
	if (dp == NULL) {
		printk(KERN_DEBUG "Out of Memory\n");
		filp_close(filp, current->files);
	}
	pos = 0;
	memset(dp, 0, l);
	ret = vfs_read(filp, (char __user *)dp, l, &pos);
	if (ret != l) {
		printk(KERN_DEBUG "Failed to read file ret = %d\n", ret);
		kfree(dp);
		filp_close(filp, current->files);
		return -EINVAL;
	}

	filp_close(filp, current->files);

	set_fs(fs);

	sr130pc10_regs_table = dp;

	sr130pc10_regs_table_size = l;

	*((sr130pc10_regs_table + sr130pc10_regs_table_size) - 1) = '\0';

/*      printk(KERN_DEBUG "sr130pc10_regs_table 0x%04x, %ld\n", dp, l); */
	return 0;
}

void sr130pc10_regs_table_exit(void)
{
	printk(KERN_DEBUG "%s %d\n", __func__, __LINE__);
/*
	if (sr130pc10_regs_table) {
		kfree(sr130pc10_regs_table);
		sr130pc10_regs_table = NULL;
	}
*/
	kfree(sr130pc10_regs_table);
	sr130pc10_regs_table = NULL;
}

static int sr130pc10_regs_table_write(char *name)
{
	char *start, *end, *reg;
	unsigned short addr, value;
	char reg_buf[5], data_buf[5];

	*(reg_buf + 4) = '\0';
	*(data_buf + 4) = '\0';

	start = strnstr(sr130pc10_regs_table, name, sr130pc10_regs_table_size);
	end = strnstr(start, "};", sr130pc10_regs_table_size);

	while (1) {
		/* Find Address */
		reg = strnstr(start, "{0x", sr130pc10_regs_table_size);
		if (reg)
			start = (reg + 11);
		if ((reg == NULL) || (reg > end))
			break;
		/* Write Value to Address */
		if (reg != NULL) {
			memcpy(reg_buf, (reg + 1), 4);
			memcpy(data_buf, (reg + 7), 4);
			kstrtol(data_buf, 16, &value);
			kstrtol(reg_buf, 16, &addr);

			if (addr == 0xdd) {
				/* msleep(value); */
				/* printk(KERN_DEBUG "delay 0x%04x,
					value 0x%04x\n", addr, value); */
			} else if (addr == 0xff) {
				msleep(value * 8);
				printk(KERN_DEBUG
					"delay 0x%04x, value 0x%04x\n", addr,
				       value);
			} else
				sr130pc10_sensor_write(addr, value);
		}
	}
	return 0;
}
#endif

int sr130pc10_sensor_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	printk(KERN_DEBUG "[SR130PC10] %s/%d\n", __func__, __LINE__);

#ifdef CONFIG_LOAD_FILE
	if (0 > sr130pc10_regs_table_init()) {
		CDBG("%s file open failed!\n", __func__);
		rc = -1;
		goto init_fail;
	}
#endif

	sr130pc10_ctrl = kzalloc(sizeof(struct sr130pc10_ctrl_t), GFP_KERNEL);
	if (!sr130pc10_ctrl) {
		printk(KERN_DEBUG "[SR130PC10]sr130pc10_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	if (data)
		sr130pc10_ctrl->sensordata = data;

	rc = cam_hw_init();
	if (rc < 0) {
		printk(KERN_DEBUG "[SR130PC10]<=PCAM=> cam_hw_init failed!\n");
		goto init_fail;
	}

	rc = sr130pc10_sensor_init_probe(data);
	if (rc < 0) {
		printk(KERN_DEBUG "[SR130PC10]sr130pc10_sensor_init failed!\n");
		goto init_fail;
	}
	printk(KERN_DEBUG "[SR130PC10] %s/%d\n", __func__, __LINE__);

 init_done:
	return rc;

 init_fail:
	kfree(sr130pc10_ctrl);
	return rc;
}

static int sr130pc10_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&sr130pc10_wait_queue);
	return 0;
}

int sr130pc10_sensor_ext_config(void __user *argp)
{
	struct sensor_ext_cfg_data cfg_data;
	int err = 0;

	printk(KERN_DEBUG "[teddy][%s][E]\n", __func__);

	if (copy_from_user(
		(void *)&cfg_data, (const void *)argp, sizeof(cfg_data)))
		printk(KERN_DEBUG "fail copy_from_user!\n");

	switch (cfg_data.cmd) {
	case EXT_CFG_SET_BRIGHTNESS:
		err = sr130pc10_set_exposure_value(0, cfg_data.value_1);
		break;

	case EXT_CFG_SET_ISO:
		/*err = s5k5ccg_set_ISO(0, cfg_data.value_1);*/
		break;

	case EXT_CFG_SET_MOVIE_MODE:
		sr130pc10_set_movie_mode(cfg_data.value_1);
		break;

	case EXT_CFG_GET_EXIF:
		sr130pc10_get_exif(&cfg_data.value_1, &cfg_data.value_2);
		break;
	case EXT_CAM_SET_FLIP:
		sr130pc10_set_flipmode(cfg_data.value_1);
		break;
	default:
		break;
	}

	if (copy_to_user(
		(void *)argp, (const void *)&cfg_data, sizeof(cfg_data)))
		printk(KERN_DEBUG "fail copy_to_user!\n");

	return err;
}


int sr130pc10_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	long rc = 0;

	if (copy_from_user(&cfg_data,
			   (void *)argp, sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	/* down(&sr130pc10_sem); */

	printk(KERN_DEBUG "sr130pc10_ioctl, cfgtype = %d, mode = %d\n",
	       cfg_data.cfgtype, cfg_data.mode);

	switch (cfg_data.cfgtype) {
	case CFG_SET_MODE:
		rc = sr130pc10_set_sensor_mode(cfg_data.mode);
		break;

	case CFG_SET_EFFECT:
		rc = sr130pc10_set_effect(cfg_data.mode, cfg_data.cfg.effect);
		break;

	case CFG_SET_EXPOSURE_VALUE:
		rc = sr130pc10_set_exposure_value(cfg_data.mode,
						  cfg_data.cfg.ev);
		break;

	case CFG_SET_WB:
		rc = sr130pc10_set_whitebalance(cfg_data.mode, cfg_data.cfg.wb);
		break;

	case CFG_SET_ROTATION:
		rc = sr130pc10_set_rotation(cfg_data.cfg.rotation);
		break;

	case CFG_SET_DATALINE_CHECK:
		if (cfg_data.cfg.dataline) {
			printk(KERN_DEBUG
				"[SR130PC10] CFG_SET_DATALINE_CHECK ON\n");
			sr130pc10_sensor_write(0x03, 0x00);
			sr130pc10_sensor_write(0x50, 0x05);
			factory_test = 1;
		} else {
			printk(KERN_DEBUG
				"[SR130PC10] CFG_SET_DATALINE_CHECK OFF\n");
			sr130pc10_sensor_write(0x03, 0x00);
			sr130pc10_sensor_write(0x50, 0x00);
		}
		break;

	case CFG_GET_AF_MAX_STEPS:
	default:
/*                      rc = -EINVAL; */
		rc = 0;
		break;
	}

	/* up(&sr130pc10_sem); */

	return rc;
}

int sr130pc10_sensor_release(void)
{
	int rc = 0;

#ifdef CONFIG_MACH_ICON
	sr130pc10_sensor_write(0x03, 0x02);
	sr130pc10_sensor_write(0x55, 0x10);
	mdelay(1);

	gpio_set_value(CAM_RESET, 0);
	gpio_set_value(CAM_STANDBY, 0);


	gpio_set_value(CAM_VT_RST, 0);
	mdelay(1);


#else
	gpio_tlmm_config(
		GPIO_CFG(VCAM_I2C_SCL, 0, GPIO_CFG_OUTPUT,
			GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		GPIO_CFG_ENABLE);
	gpio_tlmm_config(
		GPIO_CFG(VCAM_I2C_SDA, 0, GPIO_CFG_OUTPUT,
			GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		GPIO_CFG_ENABLE);
	mdelay(1);
	udelay(50);
	gpio_set_value(CAM_RESET, 0); /*REST -> DOWN*/
	gpio_set_value(CAM_STANDBY, 0); /*REST -> DOWN*/
	gpio_set_value(CAM_VT_RST, 0);	/*REST -> DOWN */
	mdelay(2);

	/*mclk disable*/
	gpio_tlmm_config(GPIO_CFG(CAM_MCLK, 0,
		GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		GPIO_CFG_16MA), GPIO_CFG_ENABLE); /*CAM_MCLK*/
	udelay(20);

	gpio_set_value(CAM_VT_nSTBY, 0); /*STBY -> DOWN*/
	udelay(10);

	gpio_set_value(CAM_EN_2, 0);
	vreg_disable(vreg_get(NULL, "gp7"));
	gpio_set_value(CAM_EN, 0); /*EN -> DOWN*/
#endif

	/* down(&sr130pc10_sem); */

	kfree(sr130pc10_ctrl);
	/* up(&sr130pc10_sem); */
	return rc;
}

static int
sr130pc10_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;
	printk(KERN_DEBUG "[SR130PC10] %s/%d\n", __func__, __LINE__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	sr130pc10_sensorw =
	    kzalloc(sizeof(struct sr130pc10_work_t), GFP_KERNEL);

	if (!sr130pc10_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, sr130pc10_sensorw);
	sr130pc10_init_client(client);
	sr130pc10_client = client;

	sr130pc10_dev = device_create(camera_class, NULL, 0, NULL, "front");
	if (IS_ERR(sr130pc10_dev)) {
		pr_err("Failed to create device!");
		goto probe_failure;
	}

	if (device_create_file(sr130pc10_dev, &dev_attr_front_camfw) < 0) {
		CDBG("failed to create device file, %s\n",
		dev_attr_front_camfw.attr.name);
	}
	if (device_create_file(sr130pc10_dev, &dev_attr_front_camtype) < 0) {
		CDBG("failed to create device file, %s\n",
		dev_attr_front_camtype.attr.name);
	}
	printk(KERN_DEBUG "[SR130PC10] sr130pc10_probe succeeded!  %s/%d\n",
		__func__, __LINE__);

	CDBG("sr130pc10_probe succeeded!\n");

	return 0;

 probe_failure:
	kfree(sr130pc10_sensorw);
	sr130pc10_sensorw = NULL;
	printk(KERN_DEBUG "[SR130PC10]sr130pc10_probe failed!\n");
	return rc;
}

static const struct i2c_device_id sr130pc10_i2c_id[] = {
	{"sr130pc10", 0},
	{},
};

static struct i2c_driver sr130pc10_i2c_driver = {
	.id_table = sr130pc10_i2c_id,
	.probe = sr130pc10_i2c_probe,
	.remove = __exit_p(sr130pc10_i2c_remove),
	.driver = {
		   .name = "sr130pc10",
		   },
};

static int
sr130pc10_sensor_probe(const struct msm_camera_sensor_info *info,
		       struct msm_sensor_ctrl *s)
{
	/*struct vreg *vreg_L8;*/

	int rc = i2c_add_driver(&sr130pc10_i2c_driver);
	if (rc < 0 || sr130pc10_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_done;
	}
	printk(KERN_DEBUG "[SR130PC10] %s/%d\n", __func__, __LINE__);
	printk(KERN_DEBUG "[SR130PC10] sr130pc10_client->addr : %x\n",
	       sr130pc10_client->addr);
	printk(KERN_DEBUG "[SR130PC10] sr130pc10_client->adapter->nr : %d\n",
	       sr130pc10_client->adapter->nr);

	s->s_init = sr130pc10_sensor_init;
	s->s_power = sr130pc10_set_power;
	s->s_release = sr130pc10_sensor_release;
	s->s_config = sr130pc10_sensor_config;
	s->s_ext_config	= sr130pc10_sensor_ext_config;
	s->s_camera_type = FRONT_CAMERA_2D;
	s->s_mount_angle = 270;
	/*SecFeature : for Android CCD preview mirror / snapshot non-mirror */

 probe_done:
	CDBG("%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
}

static int __sr130pc10_probe(struct platform_device *pdev)
{
	printk(KERN_DEBUG "[SR130PC10]  %s/%d\n", __func__, __LINE__);
	return msm_camera_drv_start(pdev, sr130pc10_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __sr130pc10_probe,
	.driver = {
		   .name = "msm_camera_sr130pc10",
		   .owner = THIS_MODULE,
		   },
};

static int __init sr130pc10_init(void)
{
	printk(KERN_DEBUG "[SR130PC10]  %s/%d\n", __func__, __LINE__);
	return platform_driver_register(&msm_camera_driver);
}

module_init(sr130pc10_init);
