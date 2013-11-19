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
#include "sr030pc30.h"
#include <linux/slab.h>
#include <mach/vreg.h>

#include <mach/camera.h>

/*#define SENSOR_DEBUG 0*/
#undef CONFIG_LOAD_FILE
/*#define CONFIG_LOAD_FILE*/

#define CAM_RESET 130
#define CAM_STANDBY 131
#define CAM_EN 3
#define CAM_EN_2 132
#define CAM_I2C_SCL 30
#define CAM_I2C_SDA 17
#define CAM_VT_nSTBY 2
#define CAM_VT_RST 175
#define CAM_MCLK 15

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

struct sr030pc30_work_t {
	struct work_struct work;
};

static struct  sr030pc30_work_t *sr030pc30_sensorw;
static struct  i2c_client *sr030pc30_client;

struct sr030pc30_ctrl_t {
	const struct msm_camera_sensor_info *sensordata;
};


static struct sr030pc30_ctrl_t *sr030pc30_ctrl;

static DECLARE_WAIT_QUEUE_HEAD(sr030pc30_wait_queue);
DECLARE_MUTEX(sr030pc30_sem);

#ifdef CONFIG_LOAD_FILE
static int sr030pc30_regs_table_write(char *name);
#endif
static int cam_hw_init(void);

/*static int16_t sr030pc30_effect = CAMERA_EFFECT_OFF;*/
static int rotation_status;
static int factory_test;

/*++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Factory Test */
/*++++++++++++++++++++++++++++++++++++++++++++++++++*/
struct device *sr030pc30_dev;
extern struct class *camera_class;
static ssize_t camtype_file_cmd_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char camType[] = "SILICONFILE_SR030PC30_NONE";

	return snprintf(buf, 30, "%s", camType);
}

static ssize_t camtype_file_cmd_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(
front_camtype, 0660, camtype_file_cmd_show, camtype_file_cmd_store);
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*=============================================================
	EXTERNAL DECLARATIONS
==============================================================*/
/*extern struct sr030pc30_reg sr030pc30_regs;*/


/*=============================================================*/
static int sr030pc30_sensor_read(unsigned short subaddr, unsigned short *data)
{
	/*pr_info("<=ASWOOGI=> sr030pc30_sensor_read\n");*/

	int ret;
	unsigned char buf[1] = {0};
	struct i2c_msg msg = { sr030pc30_client->addr, 0, 1, buf };

	buf[0] = subaddr;
/*	buf[1] = 0x0;*/

	ret = i2c_transfer(sr030pc30_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO)
		goto error;

	msg.flags = I2C_M_RD;

	ret = i2c_transfer(sr030pc30_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO)
		goto error;

/*	*data = ((buf[0] << 8) | buf[1]);*/
	*data = buf[0];

error:
	/*pr_info("[ASWOOGI] on read func  sr030pc30_sensor_read->addr : %x\n",
	  sr030pc30_client->addr);
	pr_info("[ASWOOGI] on read func  subaddr : %x\n", subaddr);
	pr_info("[ASWOOGI] on read func  data : %x\n", data);
	 */


	return ret;
}

static int sr030pc30_sensor_write(unsigned short subaddr, unsigned short val)
{
	unsigned char buf[2] = {0};
	struct i2c_msg msg = { sr030pc30_client->addr, 0, 2, buf };

	/*	pr_info("[sr030pc30] on write func
		sr030pc30_client->addr : %x\n", subaddr);*/
	/*	pr_info("[sr030pc30] on write func
		sr030pc30_client->adapter->nr : %x\n", val);*/

	buf[0] = subaddr;
	buf[1] = val;

	if (i2c_transfer(sr030pc30_client->adapter, &msg, 1) == 1) {
		/*pr_info("[sr030pc30] sensor_write success\n");*/
		return 0;
	} else {
		pr_info("[sr030pc30] sr030pc30_sensor_write fail\n");
		return -EIO;
	}
}


static int sr030pc30_sensor_write_list(struct sr030pc30_short_t *list,
		int size, char *name)
{
	int ret = 0;
#ifdef CONFIG_LOAD_FILE
	ret = sr030pc30_regs_table_write(name);
#else
	int i;
	pr_info("[sr030pc30]%s (%s)\n", __func__, name);

	for (i = 0; i < size; i++) {
		if (sr030pc30_sensor_write(list[i].subaddr,
					list[i].value) < 0) {
			pr_info("[sr030pc30] sensor_write_list fail...\n");
			return -EIO;
		}
	}
#endif
	return ret;
}


static long sr030pc30_set_sensor_mode(int mode)
{
	pr_info("[sr030pc30] Sensor Mode\n");
	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		pr_info("[sr030pc30]-> Preview\n");
		factory_test = 0;
		/*sr030pc30_sensor_write_list(reg_preview, sizeof(reg_preview)/\
					sizeof(reg_preview[0]),
					"reg_preview");//preview start
					msleep(100);*/
		sr030pc30_sensor_write(0x03, 0x00);
		sr030pc30_sensor_write(0x11, 0x93);
		break;
	case SENSOR_SNAPSHOT_MODE:
		pr_info("[sr030pc30]-> Capture\n");
		sr030pc30_sensor_write_list(reg_snapshot, sizeof(reg_snapshot)/
				sizeof(reg_snapshot[0]), "reg_snapshot");
/* //SecFeature : for Android CCD preview mirror / snapshot non-mirror
			   if(factory_test == 0)
			   {
			   if(rotation_status == 90 || rotation_status == 270)
			   {
			   sr030pc30_sensor_write(0x03, 0x00);
			   sr030pc30_sensor_write(0x11, 0x90);
			   }
			   else
			   {
			   sr030pc30_sensor_write(0x03, 0x00);
			   sr030pc30_sensor_write(0x11, 0x93);
			   }
			   msleep(100);
			   }
			 */
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		pr_info("[sr030pc30]-> Capture RAW\n");
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static long sr030pc30_set_effect(int mode, int effect)
{
	long rc = 0;

	switch (effect) {
	case CAMERA_EFFECT_OFF: {
	pr_info("[sr030pc30] CAMERA_EFFECT_OFF\n");
		sr030pc30_sensor_write_list(reg_effect_none,
				sizeof(reg_effect_none)/
				sizeof(reg_effect_none[0]),
				"reg_effect_none");
	}
			break;

	case CAMERA_EFFECT_MONO: {
	pr_info("[sr030pc30] CAMERA_EFFECT_MONO\n");
		sr030pc30_sensor_write_list(reg_effect_gray,
				sizeof(reg_effect_gray)/
				sizeof(reg_effect_gray[0]),
				"reg_effect_gray");
	}
		break;

	case CAMERA_EFFECT_NEGATIVE: {
	pr_info("[sr030pc30] CAMERA_EFFECT_NEGATIVE\n");
		sr030pc30_sensor_write_list(reg_effect_negative,
				sizeof(reg_effect_negative)/
				sizeof(reg_effect_negative[0]),
				"reg_effect_negative");
	}
		break;

	case CAMERA_EFFECT_SEPIA: {
	pr_info("[sr030pc30] CAMERA_EFFECT_SEPIA\n");
		sr030pc30_sensor_write_list(reg_effect_sepia,
				sizeof(reg_effect_sepia)/
				sizeof(reg_effect_sepia[0]),
				"reg_effect_sepia");
	}
		break;
	case CAMERA_EFFECT_AQUA: {
	pr_info("[sr030pc30] CAMERA_EFFECT_AQUA\n");
		sr030pc30_sensor_write_list(reg_effect_aqua,
				sizeof(reg_effect_aqua)/
				sizeof(reg_effect_aqua[0]),
				"reg_effect_aqua");
	}
		break;
	case CAMERA_EFFECT_WHITEBOARD: {
	pr_info("[sr030pc30] CAMERA_EFFECT_WHITEBOARD\n");
		sr030pc30_sensor_write_list(reg_effect_sketch,
				sizeof(reg_effect_sketch)/
				sizeof(reg_effect_sketch[0]),
				"reg_effect_sketch");
	}
		break;

	default: {

	pr_info("[sr030pc30] default .dsfsdf\n");
		sr030pc30_sensor_write_list(reg_effect_none,
				sizeof(reg_effect_none)/
				sizeof(reg_effect_none[0]),
				"reg_effect_none");

		return -EINVAL;
	}
	}
	return rc;
}

static long sr030pc30_set_exposure_value(int mode, int exposure)
{
	long rc = 0;

	pr_info("mode : %d, exposure value  : %d\n", mode, exposure);

	switch (exposure) {

	case CAMERA_EXPOSURE_NEGATIVE_2:
		pr_info("CAMERA_EXPOSURE_VALUE_-2\n");
		sr030pc30_sensor_write_list(reg_brightness_level_1,
				sizeof(reg_brightness_level_1)/
				sizeof(reg_brightness_level_1[0]),
				"reg_brightness_level_1");
		break;

	case CAMERA_EXPOSURE_NEGATIVE_1:
		pr_info("CAMERA_EXPOSURE_VALUE_-1\n");
		sr030pc30_sensor_write_list(reg_brightness_level_3,
				sizeof(reg_brightness_level_3)/
				sizeof(reg_brightness_level_3[0]),
				"reg_brightness_level_3");
		break;

	case CAMERA_EXPOSURE_0:
		pr_info("CAMERA_EXPOSURE_VALUE_0\n");
		sr030pc30_sensor_write_list(reg_brightness_level_5,
				sizeof(reg_brightness_level_5)/
				sizeof(reg_brightness_level_5[0]),
				"reg_brightness_level_5");
		break;

	case CAMERA_EXPOSURE_POSITIVE_1:
		pr_info("CAMERA_EXPOSURE_VALUE_1\n");
		sr030pc30_sensor_write_list(reg_brightness_level_7,
				sizeof(reg_brightness_level_7)/
				sizeof(reg_brightness_level_7[0]),
				"reg_brightness_level_7");
		break;

	case CAMERA_EXPOSURE_POSITIVE_2:
		pr_info("CAMERA_EXPOSURE_VALUE_2\n");
		sr030pc30_sensor_write_list(reg_brightness_level_9,
				sizeof(reg_brightness_level_9)/
				sizeof(reg_brightness_level_9[0]),
				"reg_brightness_level_9");
		break;

	default:
		pr_info("[sr030pc30] unexpected Exposure Value %s/%d\n",
				__func__, __LINE__);
		return -EINVAL;
	}
	return rc;
}

static long sr030pc30_set_whitebalance(int mode, int wb)
{
	long rc = 0;

	pr_info("mode : %d,   whitebalance : %d\n", mode, wb);

	switch (wb) {
	case CAMERA_WB_AUTO:
		pr_info("CAMERA_WB_AUTO\n");
		sr030pc30_sensor_write_list(reg_wb_auto,
		sizeof(reg_wb_auto)/
		sizeof(reg_wb_auto[0]),
		"reg_wb_auto");
		break;

	case CAMERA_WB_INCANDESCENT:
		pr_info("CAMERA_WB_INCANDESCENT\n");
		sr030pc30_sensor_write_list(reg_wb_incandescent,
				sizeof(reg_wb_incandescent)/
				sizeof(reg_wb_incandescent[0]),
				"reg_wb_incandescent");
		break;

	case CAMERA_WB_FLUORESCENT:
		pr_info("CAMERA_WB_FLUORESCENT\n");
		sr030pc30_sensor_write_list(reg_wb_fluorescent,
				sizeof(reg_wb_fluorescent)/
				sizeof(reg_wb_fluorescent[0]),
				"reg_wb_fluorescent");
		break;

	case CAMERA_WB_DAYLIGHT:
		pr_info("CAMERA_WB_DAYLIGHT\n");
		sr030pc30_sensor_write_list(reg_wb_daylight,
				sizeof(reg_wb_daylight)/
				sizeof(reg_wb_daylight[0]),
				"reg_wb_daylight");
		break;

	case CAMERA_WB_CLOUDY_DAYLIGHT:
		pr_info("CAMERA_WB_CLOUDY_DAYLIGHT\n");
		sr030pc30_sensor_write_list(reg_wb_cloudy,
				sizeof(reg_wb_cloudy)/
				sizeof(reg_wb_cloudy[0]),
				"reg_wb_cloudy");
		break;

	default:
		pr_info("[sr030pc30] unexpected WB mode %s/%d\n",
				__func__, __LINE__);
		return -EINVAL;
	}
	return rc;
}

static long sr030pc30_set_auto_exposure(int mode, int metering)
{
	long rc = 0;

	return 0;
}
static long sr030pc30_set_rotation(int rotation)
{
	long rc = 0;

	return 0;
}

static int sr030pc30_sensor_init_probe(
		const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	pr_info("%s/%d\n", __func__, __LINE__);

	sr030pc30_sensor_write_list(reg_init, sizeof(reg_init)/
		sizeof(reg_init[0]), "reg_init");

	usleep(10000);

	return rc;
}
static int cam_hw_init()
{

	int rc = 0;
	struct vreg *vreg_L8;
	int test_value;

	pr_info("[sr030pc30] %s/%d\n", __func__, __LINE__);

	gpio_tlmm_config(
			GPIO_CFG(CAM_RESET, 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_16MA), GPIO_CFG_ENABLE);/*CAM_RESET*/

	gpio_tlmm_config(
			GPIO_CFG(CAM_STANDBY, 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_16MA), GPIO_CFG_ENABLE);/*CAM_STANDBY*/

	gpio_tlmm_config(
			GPIO_CFG(CAM_EN, 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_16MA), GPIO_CFG_ENABLE);/*CAM_EN*/

	gpio_tlmm_config(
			GPIO_CFG(CAM_EN_2, 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_16MA), GPIO_CFG_ENABLE); /*CAM_EN_2*/

	gpio_tlmm_config(
			GPIO_CFG(CAM_VT_RST, 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_16MA), GPIO_CFG_ENABLE);/*CAM_VT_RST*/

	gpio_tlmm_config(
			GPIO_CFG(CAM_VT_nSTBY, 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_16MA), GPIO_CFG_ENABLE);

	vreg_L8 = vreg_get(NULL, "gp7");
	vreg_set_level(vreg_L8, 1800);

	gpio_set_value(CAM_RESET, 0);
	gpio_set_value(CAM_STANDBY, 0);
	gpio_set_value(CAM_EN, 0);

	gpio_set_value(CAM_EN_2, 0);


	gpio_set_value(CAM_VT_RST, 0);
	gpio_set_value(CAM_VT_nSTBY, 0);
	usleep(1000);

	gpio_set_value(CAM_EN, 1); /*CAM_A_2.8V //CAM_EN->UP*/
	usleep(1000);
	vreg_enable(vreg_L8);
	usleep(1000);
	gpio_set_value(CAM_EN_2, 1); /*CAM_IO, CAM_D, EN_2->UP*/

	usleep(1000);
	gpio_set_value(CAM_VT_nSTBY, 1); /*VGA_STBY UP*/

	usleep(1000);

	gpio_tlmm_config(GPIO_CFG(CAM_MCLK, 1,
				GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_14MA), GPIO_CFG_ENABLE); /*CAM_MCLK*/

	msm_camio_clk_rate_set(24000000);	/*MCLK*/
	msm_camio_camif_pad_reg_reset();
	usleep(1000);

	gpio_set_value(CAM_STANDBY, 1); /*3M_STBY UP*/
	usleep(10000);

	gpio_set_value(CAM_RESET, 1); /*3M_RESET UP*/
	usleep(10000);

	gpio_set_value(CAM_STANDBY, 0); /*3M_STBY DOWN*/
	usleep(1000);

	gpio_set_value(CAM_VT_RST, 1); /*VGA_RESET UP*/
	usleep(50000);

	return rc;
}

void sensor_rough_control_sr030pc30(void __user *arg)
{
	struct ioctl_pcam_info_8bit ctrl_info;

	int Exptime;
	int Expmax;
	unsigned short read_1, read_2, read_3;

	pr_info("[SR030PC30] sensor_rough_control\n");

	if (copy_from_user((void *)&ctrl_info,
				(const void *)arg,
				sizeof(ctrl_info))) {
		pr_info("<=SR030PC30=> %s fail copy_from_user!\n", __func__);
	}
	pr_info("<=SR030PC30=> TEST %d %d %d %d %d\n",
			ctrl_info.mode,
			ctrl_info.address,
			ctrl_info.value_1,
			ctrl_info.value_2,
			ctrl_info.value_3);


	switch (ctrl_info.mode) {
	case PCAM_CONNECT_CHECK:
		pr_info("[SR030PC30] PCAM_CONNECT_CHECK\n");
		int rc = 0;
		rc = sr030pc30_sensor_write(0x03, 0x00);
		/*check sensor connection*/
		if (rc < 0) {
			pr_info("[SR030PC30] Connect error\n");
			ctrl_info.value_1 = 1;
		}
		break;

	case PCAM_EXPOSURE_TIME:
		pr_info("[SR030PC30] PCAM_EXPOSURE_TIME\n");
		sr030pc30_sensor_write(0x03, 0x20);
		sr030pc30_sensor_read(0x80, &ctrl_info.value_1);
		sr030pc30_sensor_read(0x81, &ctrl_info.value_2);
		sr030pc30_sensor_read(0x82, &ctrl_info.value_3);
		pr_info("[SR030PC30] PCAM_EXPOSURE_TIME :"
				"A(%x), B(%x), C(%x)\n]",
				ctrl_info.value_1,
				ctrl_info.value_2,
				ctrl_info.value_3);
		break;

	case PCAM_ISO_SPEED:
		pr_info("[SR030PC30] PCAM_ISO_SPEED\n");
		sr030pc30_sensor_write(0x03, 0x20);
		sr030pc30_sensor_read(0xb0, &ctrl_info.value_1);
		break;

	case PCAM_FIXED_FRAME:
		pr_info("[SR030PC30] PCAM_FIXED_FRAME\n");
		sr030pc30_sensor_write_list(reg_15fps_fix,
				sizeof(reg_15fps_fix)/
				sizeof(reg_15fps_fix[0]),
				"reg_15fps_fix");
		break;

	case PCAM_AUTO_FRAME:
		pr_info("[SR030PC30] PCAM_AUTO_FRAME\n");
		sr030pc30_sensor_write_list(reg_var_fps,
				sizeof(reg_var_fps)/
				sizeof(reg_var_fps[0]),
				"reg_var_fps");
		break;

	default:
		pr_info("<=SR030PC30=> Unexpected mode on sensor_rough_control\n");
		break;
	}

	if (copy_to_user((void *)arg,
				(const void *)&ctrl_info,
				sizeof(ctrl_info))) {
		pr_info("<=SR030PC30=> %s fail on copy_to_user!\n", __func__);
	}

}


#ifdef CONFIG_LOAD_FILE

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>

static char *sr030pc30_regs_table;
static int sr030pc30_regs_table_size;

int sr030pc30_regs_table_init(void)
{
	struct file *filp;
	char *dp;
	long l;
	loff_t pos;
	int ret = -1;
	mm_segment_t fs = get_fs();

	pr_info("%s %d\n", __func__, __LINE__);

	set_fs(get_ds());
	filp = filp_open("/mnt/sdcard/sr030pc30.h", O_RDONLY, 0);
	if (IS_ERR(filp)) {
		pr_info("file open error %d\n", PTR_ERR(filp));
		return ret;
	}
	l = filp->f_path.dentry->d_inode->i_size;
	pr_info("l = %ld\n", l);
	dp = kmalloc(l, GFP_KERNEL);
	if (dp == NULL) {
		pr_info("Out of Memory\n");
		filp_close(filp, current->files);
		return ret;
	}
	pos = 0;
	memset(dp, 0, l);
	ret = vfs_read(filp, (char __user *)dp, l, &pos);
	if (ret != l) {
		pr_info("Failed to read file ret = %d\n", ret);
		kfree(dp);
		filp_close(filp, current->files);
		return ret;
	}

	filp_close(filp, current->files);

	set_fs(fs);

	sr030pc30_regs_table = dp;

	sr030pc30_regs_table_size = l;

	*((sr030pc30_regs_table + sr030pc30_regs_table_size) - 1) = '\0';

/*	pr_info("sr030pc30_regs_table 0x%04x, %ld\n", dp, l);*/
	return 0;
}

void sr030pc30_regs_table_exit(void)
{
	pr_info("%s %d\n", __func__, __LINE__);
		kfree(sr030pc30_regs_table);
		sr030pc30_regs_table = NULL;
}

static int sr030pc30_regs_table_write(char *name)
{
	char *start, *end, *reg;
	unsigned short addr, value;
	char reg_buf[5], data_buf[5];

	*(reg_buf + 4) = '\0';
	*(data_buf + 4) = '\0';

	start = strnstr(sr030pc30_regs_table, name, sr030pc30_regs_table_size);

	end = strnstr(start, "};", sr030pc30_regs_table_size);

	while (1) {
		/* Find Address */
		reg = strnstr(start, "{0x", sr030pc30_regs_table_size);
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

			/*pr_info("addr 0x%04x, value 0x%04x\n", addr, value);*/
			if (addr == 0xdd) {
				/*msleep(value);*/
				/*pr_info("delay 0x%04x, value 0x%04x\n",
				  addr, value);*/
			} else if (addr == 0xff) {
				msleep(value * 8);
				pr_info("delay 0x%04x, value 0x%04x\n",
						addr, value);
			} else
				sr030pc30_sensor_write(addr, value);
		}
	}
	return 0;
}
#endif


int sr030pc30_sensor_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	pr_info("[sr030pc30] %s/%d\n", __func__, __LINE__);

#ifdef CONFIG_LOAD_FILE
	sr030pc30_regs_table_init();
#endif

	sr030pc30_ctrl = kzalloc(sizeof(struct sr030pc30_ctrl_t), GFP_KERNEL);
	if (!sr030pc30_ctrl) {
		CDBG("sr030pc30_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	if (data)
		sr030pc30_ctrl->sensordata = data;


	rc = cam_hw_init();
	if (rc < 0) {
		pr_info("[sr030pc30] cam_hw_init failed!\n");
		goto init_fail;
	}

	rc = sr030pc30_sensor_init_probe(data);
	if (rc < 0) {
		CDBG("sr030pc30_sensor_init failed!\n");
		goto init_fail;
	}
	pr_info("[sr030pc30]  3333333 %s/%d\n", __func__, __LINE__);


init_done:
	return rc;

init_fail:
	kfree(sr030pc30_ctrl);
	return rc;
}

static int sr030pc30_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&sr030pc30_wait_queue);
	return 0;
}

int sr030pc30_sensor_ext_config(void __user *argp)
{
	struct sensor_ext_cfg_data cfg_data;
	int err = 0;

	printk(KERN_DEBUG "[teddy][%s][E]\n", __func__);

	if (copy_from_user(
		(void *)&cfg_data, (const void *)argp, sizeof(cfg_data)))
		printk(KERN_DEBUG "fail copy_from_user!\n");

	switch (cfg_data.cmd) {
	case EXT_CFG_SET_BRIGHTNESS:
		err = sr030pc30_set_exposure_value(0, cfg_data.value_1);
		break;

	case EXT_CFG_SET_ISO:
		/*err = s5k5ccg_set_ISO(0, cfg_data.value_1);*/
		break;

	default:
		break;
	}

	if (copy_to_user(
		(void *)argp, (const void *)&cfg_data, sizeof(cfg_data)))
		printk(KERN_DEBUG "fail copy_to_user!\n");

	return err;
}

int sr030pc30_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	long   rc = 0;

	if (copy_from_user(&cfg_data,
			(void *)argp,
			sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	/* down(&sr030pc30_sem); */

	pr_info("sr030pc30_ioctl, cfgtype = %d, mode = %d\n",
		cfg_data.cfgtype, cfg_data.mode);

	switch (cfg_data.cfgtype) {
	case CFG_SET_MODE:
		rc = sr030pc30_set_sensor_mode(
					cfg_data.mode);
		break;

	case CFG_SET_EFFECT:
		rc = sr030pc30_set_effect(cfg_data.mode,
					cfg_data.cfg.effect);
		break;

	case CFG_SET_EXPOSURE_VALUE:
		rc = sr030pc30_set_exposure_value(cfg_data.mode,
					cfg_data.cfg.ev);
		break;

	case CFG_SET_WB:
		rc = sr030pc30_set_whitebalance(cfg_data.mode,
					cfg_data.cfg.wb);
		break;

	case CFG_SET_EXPOSURE_MODE:
		rc = sr030pc30_set_auto_exposure(cfg_data.mode,
					cfg_data.cfg.metering);
		break;

	case CFG_SET_ROTATION:
		rc = sr030pc30_set_rotation(cfg_data.cfg.rotation);
		break;

	case CFG_SET_DATALINE_CHECK:
		if (cfg_data.cfg.dataline) {
			pr_info("[SR030PC30] CFG_SET_DATALINE_CHECK ON\n");
			sr030pc30_sensor_write_list(reg_dtp_on,
					sizeof(reg_dtp_on)/
					sizeof(reg_dtp_on[0]),
					"reg_dtp_on");
			factory_test = 1;
		} else {
			pr_info("[SR030PC30] CFG_SET_DATALINE_CHECK OFF\n");
			sr030pc30_sensor_write_list(reg_dtp_off,
					sizeof(reg_dtp_off)/
					sizeof(reg_dtp_off[0]),
					"reg_dtp_off");
		}
		break;

	case CFG_GET_AF_MAX_STEPS:
	default:
/*			rc = -EINVAL;*/
		rc = 0;
		break;
	}

	/* up(&sr030pc30_sem); */

	return rc;
}

int sr030pc30_sensor_release(void)
{
	int rc = 0;

	gpio_set_value(CAM_VT_RST, 0); /*REST -> DOWN*/
	usleep(1000);
	gpio_set_value(CAM_RESET, 0); /*REST -> DOWN*/
	usleep(1000);
	gpio_set_value(CAM_VT_nSTBY, 0); /*STBY -> DOWN*/
	usleep(1000);
	pr_info("[sr030pc30] %s/%d, disable vreg gp7!\n",
			__func__, __LINE__);
	vreg_disable(vreg_get(NULL, "gp7"));
	usleep(1000);
	gpio_set_value(CAM_EN_2, 0);
	usleep(1000);
	gpio_set_value(CAM_EN, 0); /*EN -> DOWN*/
	usleep(1000);

	/* down(&sr030pc30_sem); */

	kfree(sr030pc30_ctrl);
	/* up(&sr030pc30_sem); */
#ifdef CONFIG_LOAD_FILE
	sr030pc30_regs_table_exit();
#endif
	return rc;
}

static int sr030pc30_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	pr_info("[sr030pc30]  %s/%d\n", __func__, __LINE__);


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	sr030pc30_sensorw =
		kzalloc(sizeof(struct sr030pc30_work_t), GFP_KERNEL);

	if (!sr030pc30_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, sr030pc30_sensorw);
	sr030pc30_init_client(client);
	sr030pc30_client = client;

	sr030pc30_dev = device_create(camera_class, NULL, 0, NULL, "front");
	if (IS_ERR(sr030pc30_dev)) {
		pr_err("Failed to create device!");
		goto probe_failure;
	}

	if (device_create_file(sr030pc30_dev, &dev_attr_front_camtype) < 0) {
		CDBG("failed to create device file, %s\n",
		dev_attr_front_camtype.attr.name);
	}

	pr_info("[sr030pc30] sr030pc30_probe succeeded!  %s/%d\n",
			__func__, __LINE__);

	CDBG("sr030pc30_probe succeeded!\n");

	return 0;

probe_failure:
	kfree(sr030pc30_sensorw);
	sr030pc30_sensorw = NULL;
	CDBG("sr030pc30_probe failed!\n");
	return rc;
}

static const struct i2c_device_id sr030pc30_i2c_id[] = {
	{ "sr030pc30", 0},
	{ },
};

static struct i2c_driver sr030pc30_i2c_driver = {
	.id_table = sr030pc30_i2c_id,
	.probe  = sr030pc30_i2c_probe,
	.remove = __exit_p(sr030pc30_i2c_remove),
	.driver = {
		.name = "sr030pc30",
	},
};

static int sr030pc30_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
	/*struct vreg *vreg_L8;*/

	int rc = i2c_add_driver(&sr030pc30_i2c_driver);
	if (rc < 0 || sr030pc30_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_done;
	}
	pr_info("[sr030pc30]  %s/%d\n", __func__, __LINE__);

	pr_info("[sr030pc30] sr030pc30_client->addr : %x\n",
			sr030pc30_client->addr);
	pr_info("[sr030pc30] sr030pc30_client->adapter->nr : %d\n",
			sr030pc30_client->adapter->nr);

	s->s_init = sr030pc30_sensor_init;
	s->s_release = sr030pc30_sensor_release;
	s->s_config  = sr030pc30_sensor_config;
	s->s_ext_config	= sr030pc30_sensor_ext_config;
	s->s_camera_type = FRONT_CAMERA_2D;
	s->s_mount_angle = 270;


probe_done:
	CDBG("%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
}

static int __sr030pc30_probe(struct platform_device *pdev)
{
	pr_info("[sr030pc30]  %s/%d\n", __func__, __LINE__);
	return msm_camera_drv_start(pdev, sr030pc30_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __sr030pc30_probe,
	.driver = {
		.name = "msm_camera_sr030pc30",
		.owner = THIS_MODULE,
	},
};

static int __init sr030pc30_init(void)
{
	pr_info("[sr030pc30]  %s/%d\n", __func__, __LINE__);
	return platform_driver_register(&msm_camera_driver);
}

module_init(sr030pc30_init);
