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
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include "s5k5ccgx.h"
#include <mach/vreg.h>
#include <mach/camera.h>
#ifdef CONFIG_KEYBOARD_ADP5587
#include <linux/i2c/adp5587.h>
#endif
#include <linux/mfd/pmic8058.h>

#define SENSOR_DEBUG 0
#undef CONFIG_LOAD_FILE
/* #define CONFIG_LOAD_FILE */
#define I2C_BURST_MODE

#define CAM_RESET 130
#define CAM_STANDBY 131
#define CAM_EN_2 132
#define CAM_EN 3
#define CAM_I2C_SCL 30
#define CAM_I2C_SDA 17
#define CAM_VT_nSTBY 2
#define CAM_VT_RST 175
#define CAM_MCLK 15


/*#define PCAM_ENABLE_DEBUG*/

#ifdef PCAM_ENABLE_DEBUG
#define CAMDRV_DEBUG(fmt, arg...)\
	do {					\
		pr_info("[S5K5CCGX_DEBUG] %s:%d:" fmt "\n",	\
			__func__, __LINE__, ##arg);		\
	}							\
	while (0)
#else
#define CAMDRV_DEBUG(fmt, arg...)
#endif

#define PMIC_GPIO_CAM_FLASH_SET	PM8058_GPIO(27)
#define PMIC_GPIO_CAM_FLASH_EN	PM8058_GPIO(28)


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

#define MOVIEMODE_FLASH		17
#define FLASHMODE_FLASH		18
#define FLASHMODE_AUTO	2
#define FLASHMODE_ON	3
#define FLASHMODE_OFF	1


/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* FACTORY TEST */
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
struct class *camera_class;
struct device *s5k5ccgx_dev;
static int torch_mode = -1;
static ssize_t cameraflash_file_cmd_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	/* Reserved */
	return 0;
}

static ssize_t cameraflash_file_cmd_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int value = 0, i = 0;

	sscanf(buf, "%d", &value);

	if (value == 0) {
		printk(KERN_INFO "[Accessary] flash OFF\n");
		torch_mode = 0;
		gpio_set_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_CAM_FLASH_EN), 0);
		gpio_set_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_CAM_FLASH_SET), 0);
	} else {
		printk(KERN_INFO "[Accessary] flash ON\n");
		printk(KERN_DEBUG "FLASH MOVIE MODE LED_MODE_TORCH\n");
		torch_mode = 1;

		gpio_set_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_CAM_FLASH_EN), 0);

		for (i = 5; i > 1; i--) {
			gpio_set_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_CAM_FLASH_SET), 1);
			printk(KERN_DEBUG "PMIC_GPIO_CAM_FLASH_SET - 1\n");
			udelay(50);
			gpio_set_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_CAM_FLASH_SET), 0);
			printk(KERN_DEBUG "PMIC_GPIO_CAM_FLASH_SET - 0\n");
			udelay(50);
		}
		gpio_set_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_CAM_FLASH_SET), 1);
		usleep(2*1000);
	}

	return size;
}

static DEVICE_ATTR(
rear_flash, 0660, cameraflash_file_cmd_show, cameraflash_file_cmd_store);

static ssize_t camtype_file_cmd_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char camType[] = "LSI_S5K5CCGX_NONE";

	return snprintf(buf, 20, "%s", camType);
}

static ssize_t camtype_file_cmd_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(
rear_camtype, 0660, camtype_file_cmd_show, camtype_file_cmd_store);

static ssize_t back_camera_firmware_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	char cam_fw[] = "S5K5CCGX\n";
	return snprintf(buf, sizeof(cam_fw), "%s", cam_fw);
}

static DEVICE_ATTR(rear_camfw,
	0664, back_camera_firmware_show, NULL);

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
struct s5k5ccg_work {
	struct work_struct work;
};
static struct s5k5ccg_work *s5k5ccg_sensorw;

static struct i2c_client *s5k5ccg_client;

struct s5k5ccg_ctrl_t {
	const struct msm_camera_sensor_info *sensordata;
};
static struct s5k5ccg_ctrl_t *s5k5ccg_ctrl;


static int sceneNight;
static int fireWorks;
static int AELock;
static int AWBLock;
static int af_result;

static int s5k5ccg_set_flash(int set_val);

static DECLARE_WAIT_QUEUE_HEAD(s5k5ccg_wait_queue);
/*DECLARE_MUTEX(s5k5ccg_sem);*/


#ifdef CONFIG_LOAD_FILE
static int s5k5ccg_regs_table_write(char *name);
#endif
static int cam_hw_init(void);


/*static int previous_scene_mode = -1;*/
static int previous_WB_mode = -1;

static int16_t s5k5ccg_effect = CAMERA_EFFECT_OFF;

/*typedef enum {	changed for coding rule*/
enum AF_STATUS_DEF {
	af_stop = 0,
	af_running,
	af_status_max
};

static int af_status = af_status_max;

/*typedef enum {	changed for coding rule*/
/*
enum AF_POSITION_DEF {
	af_position_auto = 0,
	af_position_infinity,
	af_position_macro,
	af_position_max
};*/
/*static unsigned short AFPosition[af_position_max]
= { 0xFF, 0xFF, 0x48, 0xFF };*/
/*static unsigned short AFPosition[af_position_max] = { 0xFF, 0xFF, 0x48 };*/
/* auto, infinity, macro, default */

/*
static unsigned short
DummyAFPosition[af_position_max] = { 0xFE, 0xFE, 0x50, 0xFE };*/
/*DummyAFPosition[af_position_max] = { 0xFE, 0xFE, 0x50 };*/
/* auto, infinity, macro, default */
/*
static unsigned short set_AF_postion = 0xFF;
static unsigned short set_Dummy_AF_position = 0xFE;*/
/* static unsigned short lux_value = 0; */
static int preview_start;
static int flash_mode;
static int af_mode;
static int flash_set_on_af;
static int flash_set_on_snap;
static int preflash_on;
static int exif_flash_status;
static int vtmode = -1;
static int low_temp;

/*=============================================================
	EXTERNAL DECLARATIONS
==============================================================*/
/*extern struct s5k5ccg_reg s5k5ccg_regs;	changed for coding rule*/


/*=============================================================*/

static int
s5k5ccg_sensor_read(unsigned short subaddr, unsigned short *data)
{
	int ret;
	unsigned char buf[2];
	struct i2c_msg msg = { s5k5ccg_client->addr, 0, 2, buf };

	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);

	ret = i2c_transfer(s5k5ccg_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) {
		printk(KERN_DEBUG "[S5K5CCG] s5k5ccg_sensor_read fail : %d\n",
			ret);
		goto error;
	}

	msg.flags = I2C_M_RD;

	ret = i2c_transfer(s5k5ccg_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) {
		printk(KERN_DEBUG "[S5K5CCG] s5k5ccg_sensor_read fail : %d\n",
			ret);
		goto error;
	}

	*data = ((buf[0] << 8) | buf[1]);

error:
	return ret;
}

int
s5k5ccg_sensor_write(unsigned short subaddr, unsigned short val)
{
	unsigned char buf[4];
	struct i2c_msg msg = { s5k5ccg_client->addr, 0, 4, buf };
/*
	printk(KERN_DEBUG "[PGH] on write func s5k5ccg_client->addr
		: %x\n", subaddr);
	printk(KERN_DEBUG "[PGH] on write func  s5k5ccg_client->adapter->nr
		: %x\n", val);
*/
	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);
	buf[2] = (val >> 8);
	buf[3] = (val & 0xFF);

	if (i2c_transfer(s5k5ccg_client->adapter, &msg, 1) == 1) {
		return 0;
	} else {
		printk(KERN_DEBUG "[S5K5CCG] s5k5ccg_sensor_write fail\n");
		return -EIO;
	}
	/*return i2c_transfer(s5k5ccg_client->adapter, &msg, 1) == 1 ? 0
		: -EIO; */
}

int
s5k5ccg_sensor_write_list(struct s5k5ccg_short_t *list, char *name)
{
#ifndef CONFIG_LOAD_FILE
	int i;
#endif
	int ret;
	ret = 0;

#ifdef CONFIG_LOAD_FILE
	printk(KERN_DEBUG "[S5K5CCG]%s (%s)\n", __func__, name);
	s5k5ccg_regs_table_write(name);
#else
	printk(KERN_DEBUG "[S5K5CCG]%s (%s)\n", __func__, name);
	for (i = 0; list[i].subaddr != 0xffff; i++) {
		if (s5k5ccg_sensor_write(list[i].subaddr, list[i].value) < 0) {
			printk(KERN_DEBUG "[S5K5CCG] "
			"s5k5ccg_sensor_write_list fail : %d, %x, %x\n",
			i, list[i].value, list[i].subaddr);
			return -EINVAL;
		}
	}
#endif
	return ret;
}

void
sensor_rough_control(void __user *arg)
{
	struct ioctl_pcam_info_8bit ctrl_info;

	/*
	int Exptime;
	int Expmax;
	unsigned short read_1, read_2, read_3;
	*/
	int rc = 0;

	/*      printk(KERN_DEBUG "[S5K5CCG] sensor_rough_control\n");*/

	if (copy_from_user
	((void *) &ctrl_info, (const void *) arg, sizeof(ctrl_info))) {
		printk(KERN_DEBUG "<=PCAM=> %s fail copy_from_user!\n",
			 __func__);
	}

	CAMDRV_DEBUG("Enter [mode = %d, value1: %d, value2: %d]\n",
			ctrl_info.mode, ctrl_info.value_1, ctrl_info.value_2);

	switch (ctrl_info.mode)	{
	case PCAM_CONNECT_CHECK:
		rc = s5k5ccg_sensor_write(0x002C, 0x7000);
		if (rc < 0) {		/*check sensor connection */
			printk(KERN_DEBUG "[S5K5CCG] Connect error\n");
			ctrl_info.value_1 = 1;
		}
		break;

	case PCAM_VT_MODE:
		vtmode = ctrl_info.address;
		break;

	case PCAM_EXPOSURE_TIME:
		s5k5ccg_sensor_write(0xFCFC, 0xD000);
		s5k5ccg_sensor_write(0x002C, 0x7000);
		s5k5ccg_sensor_write(0x002E, 0x2A14);
		s5k5ccg_sensor_read(0x0F12, &ctrl_info.value_1);/* LSB */
		s5k5ccg_sensor_read(0x0F12, &ctrl_info.value_2);/* MSB */
		break;

	case PCAM_ISO_SPEED:
		s5k5ccg_sensor_write(0xFCFC, 0xD000);
		s5k5ccg_sensor_write(0x002C, 0x7000);
		s5k5ccg_sensor_write(0x002E, 0x2A18);
		s5k5ccg_sensor_read(0x0F12, &ctrl_info.value_1);
		break;

	case PCAM_FIXED_FRAME:
		/*msleep(50);*/
		s5k5ccg_sensor_write_list(s5k5ccg_30_fps, "s5k5ccg_30_fps");
		/*msleep(150);*/
		break;

	case PCAM_AUTO_FRAME:
		/*msleep(50);*/
		s5k5ccg_sensor_write_list(s5k5ccg_fps_nonfix,
			"s5k5ccg_fps_nonfix");
		/*msleep(150);*/
		break;

	case PCAM_NIGHT_SHOT:
		break;

/*	case PCAM_FLASH_OFF:
		s5k5ccg_set_flash(0);
		if (flash_set_on_snap) {
			s5k5ccg_sensor_write_list(S5K5CCG_Flash_End_EVT1,
			"S5K5CCG_Flash_End_EVT1");
			flash_set_on_snap = 0;
		}
		break;*/

/*	case PCAM_MOVIE_FLASH_ON:
		s5k5ccg_set_flash(MOVIEMODE_FLASH);
		break;*/

	case PCAM_PREVIEW_FPS:
		if (vtmode == 2 || vtmode == 3) {
			if (ctrl_info.address == 15)
				s5k5ccg_sensor_write_list(s5k5ccg_fps_15fix,
				"s5k5ccg_fps_15fix");
		}
		break;

	case PCAM_GET_FLASH:
		ctrl_info.value_1 = exif_flash_status;
		break;

	case PCAM_LOW_TEMP:
		low_temp = 1;
		break;

	default:
		printk(KERN_DEBUG "<=PCAM=> Unexpected mode on "
			"sensor_rough_control!!!\n");
		break;
	}

	if (copy_to_user
	((void *) arg, (const void *) &ctrl_info, sizeof(ctrl_info))) {
		printk(KERN_DEBUG "<=PCAM=> %s fail on copy_to_user!\n",
			__func__);
	}
}

#ifdef I2C_BURST_MODE		/* dha23 100325 */
#define BURST_MODE_SET			1
#define BURST_MODE_END			2
#define NORMAL_MODE_SET			3
/*#define MAX_INDEX				1000*/
#define MAX_INDEX				5000

static int
s5k5ccg_sensor_burst_write_list(struct s5k5ccg_short_t *list, char *name)
{
	__u8 temp_buf[MAX_INDEX];
	int index_overflow = 1;
	int new_addr_start = 0;
	int burst_mode = NORMAL_MODE_SET;
	unsigned short pre_subaddr = 0;
	struct i2c_msg msg = { s5k5ccg_client->addr, 0, 4, temp_buf };
	int i = 0, ret = 0;
	unsigned int index = 0;

	memset(temp_buf, 0x00, 1000);
	printk(KERN_DEBUG "s5k5ccg_sensor_burst_write_list( %s )\n", name);
	/*      printk(KERN_DEBUG "s5k5ccg_sensor_i2c_address(0x%02x)\n",
		s5k5ccg_client->addr); */
#ifdef CONFIG_LOAD_FILE
	s5k5ccg_regs_table_write(name);
#else
	for (i = 0; list[i].subaddr != 0xffff; i++) {
		if (list[i].subaddr == 0xdddd) {
			msleep(list[i].value);
			/*printk(KERN_DEBUG "delay 0x%04x, value 0x%04x\n",
				list[i].subaddr,
			list[i].value);*/
		} else {
			if (list[i].subaddr == list[i + 1].subaddr) {
				burst_mode = BURST_MODE_SET;
				if ((list[i].subaddr != pre_subaddr) ||
					(index_overflow == 1)) {
					new_addr_start = 1;
					index_overflow = 0;
				}
			} else {
				if (burst_mode == BURST_MODE_SET) {
					burst_mode = BURST_MODE_END;
					if (index_overflow == 1) {
						new_addr_start = 1;
						index_overflow = 0;
					}
				} else {
					burst_mode = NORMAL_MODE_SET;
				}
			}

			if ((burst_mode == BURST_MODE_SET)
			|| (burst_mode == BURST_MODE_END)) {
				if (new_addr_start == 1) {
					index = 0;
					/*memset(temp_buf, 0x00 ,1000); */
					index_overflow = 0;

					temp_buf[index] =
						(list[i].subaddr >> 8);
					temp_buf[++index] =
						(list[i].subaddr & 0xFF);

					new_addr_start = 0;
				}

				temp_buf[++index] = (list[i].value >> 8);
				temp_buf[++index] = (list[i].value & 0xFF);

				if (burst_mode == BURST_MODE_END) {
					msg.len = ++index;

					ret =
					i2c_transfer(s5k5ccg_client->adapter,
						&msg,
					1) == 1 ? 0 : -EIO;
					if (ret < 0) {
						printk(KERN_DEBUG
						"i2c_transfer fail !\n");
						return -EINVAL;
					}
				} else if (index >= MAX_INDEX - 1) {
					index_overflow = 1;
					msg.len = ++index;

					ret =
					i2c_transfer(s5k5ccg_client->adapter,
						&msg, 1) == 1 ? 0 : -EIO;
					if (ret < 0) {
						printk(KERN_DEBUG
						"I2C_transfer Fail !\n");
						return -EINVAL;
					}
				}
			} else {
				/*memset(temp_buf, 0x00 ,4);*/
				temp_buf[0] = (list[i].subaddr >> 8);
				temp_buf[1] = (list[i].subaddr & 0xFF);
				temp_buf[2] = (list[i].value >> 8);
				temp_buf[3] = (list[i].value & 0xFF);

				msg.len = 4;
				ret =
				i2c_transfer(s5k5ccg_client->adapter, &msg,
				1) == 1 ? 0 : -EIO;
				if (ret < 0) {
					printk(KERN_DEBUG "I2C_transfer "
						"Fail !\n");
					return -EINVAL;
				}
			}
		}
		pre_subaddr = list[i].subaddr;
	}
#endif
	return ret;
}
#endif

static int s5k5ccgx_exif_iso(void)
{
	uint32_t iso_value = 0;
	int err = 0;

	unsigned short gain;

	s5k5ccg_sensor_write(0xFCFC, 0xD000);
	s5k5ccg_sensor_write(0x002C, 0x7000);
	s5k5ccg_sensor_write(0x002E, 0x2A18);
	s5k5ccg_sensor_read(0x0F12, &gain);

	if ((256 <= gain) && (3*128 > gain))
		iso_value = 50;
	else if ((3*128 <= gain) && (5*128 > gain))
		iso_value = 100;
	else if ((5*128 <= gain) && (7*128 > gain))
		iso_value = 200;
	else if (7*128 <= gain)
		iso_value = 400;

	return (int)iso_value ;
}

static int s5k5ccgx_exif_shutter_speed(void)
{
	uint32_t shutter_speed = 0;
	int err = 0;

	unsigned short lsb, msb;

	s5k5ccg_sensor_write(0xFCFC, 0xD000);
	s5k5ccg_sensor_write(0x002C, 0x7000);
	s5k5ccg_sensor_write(0x002E, 0x2A14);
	s5k5ccg_sensor_read(0x0F12, &lsb);/* LSB */
	s5k5ccg_sensor_read(0x0F12, &msb);/* MSB */

	shutter_speed = ((msb << 16) | lsb) / 400;

	return (int)shutter_speed;
}


static int s5k5ccgx_get_exif(int *exif_cmd, int *val)
{
	switch (*exif_cmd) {
	case EXIF_TV:
		(*val) = s5k5ccgx_exif_shutter_speed();
		break;

	case EXIF_ISO:
		(*val) = s5k5ccgx_exif_iso();
		break;

	case EXIF_FLASH:
		(*val) = exif_flash_status;
		break;

	default:
		printk(KERN_DEBUG
			"[%s:%d] invalid(%d)\n",
			__func__, __LINE__, *exif_cmd);
		break;
	}

	return 0;
}

static long
s5k5ccg_set_effect(int mode, int effect)
{
	long rc = 0;

	CAMDRV_DEBUG("Enter [value = %d]\n", effect);
	switch (effect) {
	case CAMERA_EFFECT_OFF:
		s5k5ccg_sensor_write_list(s5k5ccg_effect_off,
			"s5k5ccg_effect_off");
		break;

	case CAMERA_EFFECT_MONO:
		s5k5ccg_sensor_write_list(s5k5ccg_effect_gray,
			"s5k5ccg_effect_gray");
		break;

	case CAMERA_EFFECT_NEGATIVE:
		s5k5ccg_sensor_write_list(s5k5ccg_effect_negative,
		"s5k5ccg_effect_negative");
		break;

	case CAMERA_EFFECT_SEPIA:
		s5k5ccg_sensor_write_list(s5k5ccg_effect_sepia,
		"s5k5ccg_effect_sepia");
		break;

	case CAMERA_EFFECT_SOLARIZE:
	case CAMERA_EFFECT_AQUA:
		s5k5ccg_sensor_write_list(s5k5ccg_effect_aqua,
		"s5k5ccg_effect_aqua");
		break;

	case CAMERA_EFFECT_POSTERIZE:
	case CAMERA_EFFECT_WHITEBOARD:
		s5k5ccg_sensor_write_list(s5k5ccg_effect_sketch,
		"s5k5ccg_effect_sketch");
		break;

	default:
		printk(KERN_DEBUG "unexpected effect : %d\n", effect);
		s5k5ccg_sensor_write_list(s5k5ccg_effect_off,
		"s5k5ccg_effect_off");
		/*return -EINVAL;*/
		return 0;
	}
	s5k5ccg_effect = effect;

	return rc;
}

static long
s5k5ccg_set_brightness(int mode, int brightness)
{
	long rc = 0;

	CAMDRV_DEBUG("Enter [value = %d]\n", brightness);

	switch (brightness) {
	case CAMERA_BRIGHTNESS_LV0:
		s5k5ccg_sensor_write_list(s5k5ccg_br_minus3,
		"s5k5ccg_br_minus3");
		break;

	case CAMERA_BRIGHTNESS_LV1:
		s5k5ccg_sensor_write_list(s5k5ccg_br_minus2,
		"s5k5ccg_br_minus2");
		break;

	case CAMERA_BRIGHTNESS_LV2:
		s5k5ccg_sensor_write_list(s5k5ccg_br_minus1,
		"s5k5ccg_br_minus1");
		break;

	case CAMERA_BRIGHTNESS_LV3:
		s5k5ccg_sensor_write_list(s5k5ccg_br_zero,
		"s5k5ccg_br_zero");
		break;

	case CAMERA_BRIGHTNESS_LV4:
		s5k5ccg_sensor_write_list(s5k5ccg_br_plus1,
		"s5k5ccg_br_plus1");
		break;

	case CAMERA_BRIGHTNESS_LV5:
		s5k5ccg_sensor_write_list(s5k5ccg_br_plus2,
		"s5k5ccg_br_plus2");
		break;

	case CAMERA_BRIGHTNESS_LV6:
		s5k5ccg_sensor_write_list(s5k5ccg_br_plus3,
		"s5k5ccg_br_plus3");
		break;

	default:
		printk(KERN_DEBUG "unexpected brightness %s/%d\n",
			__func__, __LINE__);
		/*return -EINVAL;*/
		return 0;
	}

	return rc;
}

static long
s5k5ccg_set_whitebalance(int mode, int wb)
{
	long rc = 0;

	CAMDRV_DEBUG("Enter [value = %d]\n", wb);
	switch (wb) {
	case CAMERA_WB_AUTO:
		previous_WB_mode = wb;
		s5k5ccg_sensor_write_list(s5k5ccg_wb_auto, "s5k5ccg_wb_auto");
		break;

	case CAMERA_WB_INCANDESCENT:
		previous_WB_mode = wb;
		s5k5ccg_sensor_write_list(s5k5ccg_wb_tungsten,
			"s5k5ccg_wb_tungsten");
		break;

	case CAMERA_WB_FLUORESCENT:
		previous_WB_mode = wb;
		s5k5ccg_sensor_write_list(s5k5ccg_wb_fluorescent,
		"s5k5ccg_wb_fluorescent");
		break;

	case CAMERA_WB_DAYLIGHT:
		previous_WB_mode = wb;
		s5k5ccg_sensor_write_list(s5k5ccg_wb_sunny,
		"s5k5ccg_wb_sunny");
		break;

	case CAMERA_WB_CLOUDY_DAYLIGHT:
		previous_WB_mode = wb;
		s5k5ccg_sensor_write_list(s5k5ccg_wb_cloudy,
		"s5k5ccg_wb_cloudy");
		break;

	default:
		printk(KERN_DEBUG "unexpected WB mode %s/%d\n",
			__func__, __LINE__);
		/*return -EINVAL;*/
		return 0;
	}

	return rc;
}


static long
s5k5ccg_set_exposure_value(int mode, int exposure)
{
	long rc = 0;

	CAMDRV_DEBUG("Enter [value = %d]\n", exposure);

	switch (exposure) {
	case CAMERA_EXPOSURE_NEGATIVE_4:
		s5k5ccg_sensor_write_list(S5K5CCGX_EV_N_4, "S5K5CCGX_EV_N_4");
		break;

	case CAMERA_EXPOSURE_NEGATIVE_3:
		s5k5ccg_sensor_write_list(S5K5CCGX_EV_N_3, "S5K5CCGX_EV_N_3");
		break;

	case CAMERA_EXPOSURE_NEGATIVE_2:
		s5k5ccg_sensor_write_list(S5K5CCGX_EV_N_2, "S5K5CCGX_EV_N_2");
		break;

	case CAMERA_EXPOSURE_NEGATIVE_1:
		s5k5ccg_sensor_write_list(S5K5CCGX_EV_N_1, "S5K5CCGX_EV_N_1");
		break;

	case CAMERA_EXPOSURE_0:
		s5k5ccg_sensor_write_list(S5K5CCGX_EV_0, "S5K5CCGX_EV_0");
		break;

	case CAMERA_EXPOSURE_POSITIVE_1:
		s5k5ccg_sensor_write_list(S5K5CCGX_EV_P_1, "S5K5CCGX_EV_P_1");
		break;

	case CAMERA_EXPOSURE_POSITIVE_2:
		s5k5ccg_sensor_write_list(S5K5CCGX_EV_P_2, "S5K5CCGX_EV_P_2");
		break;

	case CAMERA_EXPOSURE_POSITIVE_3:
		s5k5ccg_sensor_write_list(S5K5CCGX_EV_P_3, "S5K5CCGX_EV_P_3");
		break;

	case CAMERA_EXPOSURE_POSITIVE_4:
		s5k5ccg_sensor_write_list(S5K5CCGX_EV_P_4, "S5K5CCGX_EV_P_4");
		break;

	default:
		printk(KERN_DEBUG "unexpected Exposure Value %s/%d\n",
		__func__, __LINE__);
		/*return -EINVAL;*/
		return 0;
	}

	return rc;
}

static long
s5k5ccg_set_auto_exposure(int mode, int metering)
{
	long rc = 0;

	CAMDRV_DEBUG("Enter [value = %d]\n", metering);

	switch (metering) {
	case CAMERA_AEC_CENTER_WEIGHTED_S:
#ifdef I2C_BURST_MODE		/*dha23 100325*/
		s5k5ccg_sensor_burst_write_list(
			s5k5ccg_measure_brightness_center,
			"s5k5ccg_measure_brightness_center");
#else
		s5k5ccg_sensor_write_list(s5k5ccg_measure_brightness_center,
		"s5k5ccg_measure_brightness_center");
#endif
		break;

	case CAMERA_AEC_SPOT_METERING_S:
#ifdef I2C_BURST_MODE		/*dha23 100325*/
		s5k5ccg_sensor_burst_write_list(
		s5k5ccg_measure_brightness_spot,
		"s5k5ccg_measure_brightness_spot");
#else
		s5k5ccg_sensor_write_list(s5k5ccg_measure_brightness_spot,
		"s5k5ccg_measure_brightness_spot");
#endif
		break;

	case CAMERA_AEC_FRAME_AVERAGE_S:
#ifdef I2C_BURST_MODE		/*dha23 100325*/
		s5k5ccg_sensor_burst_write_list(
		s5k5ccg_measure_brightness_default,
		"s5k5ccg_measure_brightness_default");
#else
		s5k5ccg_sensor_write_list(s5k5ccg_measure_brightness_default,
		"s5k5ccg_measure_brightness_default");
#endif
		break;

	default:
		printk(KERN_DEBUG "unexpected Auto exposure %s/%d\n",
		__func__, __LINE__);
		/*return -EINVAL;*/
		return 0;
	}

	return rc;
}

static long
s5k5ccg_set_scene_mode(int mode, int scene)
{

	long rc = 0;
	if (scene == 6)
		sceneNight = 1;
	else
		sceneNight = 0;

	if (scene == 13)
		fireWorks = 1;
	else
		fireWorks = 0;

	CAMDRV_DEBUG("Enter [value = %d]\n", scene);

	switch (scene) {
	case CAMERA_SCENE_OFF:
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_OFF,
		"S5K5CCGX_CAM_SCENE_OFF");
		break;

	case CAMERA_SCENE_LANDSCAPE:
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_OFF,
		"S5K5CCGX_CAM_SCENE_OFF");
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_LANDSCAPE,
		"S5K5CCGX_CAM_SCENE_LANDSCAPE");
		break;

	case CAMERA_SCENE_NIGHT:
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_OFF,
		"S5K5CCGX_CAM_SCENE_OFF");
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_NIGHT,
		"S5K5CCGX_CAM_SCENE_NIGHT");
		break;

	case CAMERA_SCENE_BEACH:
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_OFF,
		"S5K5CCGX_CAM_SCENE_OFF");
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_BEACH,
		"S5K5CCGX_CAM_SCENE_BEACH");
		break;

	case CAMERA_SCENE_SUNSET:
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_OFF,
		"S5K5CCGX_CAM_SCENE_OFF");
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_SUNSET,
		"S5K5CCGX_CAM_SCENE_SUNSET");
		break;

	case CAMERA_SCENE_FIREWORKS:
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_OFF,
		"S5K5CCGX_CAM_SCENE_OFF");
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_FIRE,
		"S5K5CCGX_CAM_SCENE_FIRE");
		break;

	case CAMERA_SCENE_SPORTS:
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_OFF,
		"S5K5CCGX_CAM_SCENE_OFF");
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_SPORTS,
		"S5K5CCGX_CAM_SCENE_SPORTS");
		break;

	case CAMERA_SCENE_PARTY:
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_OFF,
		"S5K5CCGX_CAM_SCENE_OFF");
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_PARTY,
		"S5K5CCGX_CAM_SCENE_PARTY");
		break;

	case CAMERA_SCENE_CANDLE:
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_OFF,
		"S5K5CCGX_CAM_SCENE_OFF");
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_CANDLE,
		"S5K5CCGX_CAM_SCENE_CANDLE");
		break;

	case CAMERA_SCENE_AGAINST_LIGHT:
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_OFF,
		"S5K5CCGX_CAM_SCENE_OFF");
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_BACKLIGHT,
		"S5K5CCGX_CAM_SCENE_BACKLIGHT");
		break;

	case CAMERA_SCENE_DAWN:
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_OFF,
		"S5K5CCGX_CAM_SCENE_OFF");
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_DAWN,
		"S5K5CCGX_CAM_SCENE_DAWN");
		break;

	case CAMERA_SCENE_TEXT:
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_OFF,
		"S5K5CCGX_CAM_SCENE_OFF");
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_TEXT,
		"S5K5CCGX_CAM_SCENE_TEXT");
		break;

	case CAMERA_SCENE_FALL:
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_OFF,
		"S5K5CCGX_CAM_SCENE_OFF");
		s5k5ccg_sensor_write_list(S5K5CCGX_CAM_SCENE_FALL,
		"S5K5CCGX_CAM_SCENE_FALL");
		break;

	default:
		printk(KERN_DEBUG
		"<=PCAM=> unexpected scene %s/%d\n", __func__, __LINE__);
		/* return -EINVAL;*/
		return 0;
	}

	return rc;
}

static long
s5k5ccg_set_ISO(int mode, int iso)
{
	long rc = 0;

	CAMDRV_DEBUG("Enter [value = %d]\n", iso);

	switch (iso) {
	case CAMERA_ISOValue_AUTO:
		s5k5ccg_sensor_write_list(s5k5ccg_iso_auto,
			"s5k5ccg_iso_auto");
		break;

	case CAMERA_ISOValue_100:
		s5k5ccg_sensor_write_list(s5k5ccg_iso100, "s5k5ccg_iso100");
		break;

	case CAMERA_ISOValue_200:
		s5k5ccg_sensor_write_list(s5k5ccg_iso200, "s5k5ccg_iso200");
		break;

	case CAMERA_ISOValue_400:
		s5k5ccg_sensor_write_list(s5k5ccg_iso400, "s5k5ccg_iso400");
		break;

	default:
		printk(KERN_DEBUG "<=PCAM=> unexpected ISO value %s/%d\n",
			__func__, __LINE__);
		/*return -EINVAL;*/
		return 0;
	}

	return rc;
}

static int
s5k5ccg_set_flash(int set_val)
{
	int i = 0;
	int err = 0;
	int curt = 5;			/*current = 100%*/

	pr_info("[%s::%d] torch_mode[%d], set_val[%d]\n",
		__func__, __LINE__, torch_mode, set_val);

	if (torch_mode == 1)
		return err;

	if (low_temp == 1 && set_val == FLASHMODE_FLASH)
		set_val = MOVIEMODE_FLASH;

	if (set_val == MOVIEMODE_FLASH) {
		pr_info("[%s::%d] [MOVIEMODE_FLASH] set_val[%d]\n",
			__func__, __LINE__, set_val);
			/*movie mode*/
			gpio_set_value_cansleep(
				PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_CAM_FLASH_EN),
				0);	/*FLEN : LOW*/

			for (i = curt; i > 1; i--) {	/* Data */
				/* gpio on */
				gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS
				(PMIC_GPIO_CAM_FLASH_SET), 1);
				udelay(1);
				/* gpio off */
				gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS
				(PMIC_GPIO_CAM_FLASH_SET), 0);
				udelay(1);
			}
			gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS
			(PMIC_GPIO_CAM_FLASH_SET), 1);
			/*msleep(2);		changed for coding rule */
			udelay(2000);
	} else if (set_val == FLASHMODE_FLASH) {
		pr_info("[%s::%d] [FLASHMODE_FLASH] set_val[%d]\n",
			__func__, __LINE__, set_val);
			gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS
			(PMIC_GPIO_CAM_FLASH_EN), 1);
			gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS
			(PMIC_GPIO_CAM_FLASH_SET), 0);
		} else {
		pr_info("[%s::%d] [FLASH_OFF] set_val[%d]\n",
			__func__, __LINE__, set_val);
			/* flash off */
			gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS
			(PMIC_GPIO_CAM_FLASH_EN), 0);
			gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS
			(PMIC_GPIO_CAM_FLASH_SET), 0);
	}

	return err;
}

static int
s5k5ccg_set_fps(unsigned int mode, unsigned int fps)
{
	int rc = 0;

	CAMDRV_DEBUG("Enter [mode = %d, value = %d]\n", mode, fps);
	if (mode) {
		switch (fps) {
		case 15:
			s5k5ccg_sensor_write_list(s5k5ccg_fps_15fix,
					"s5k5ccgx_fps_15fix");
			break;

		case 30:
			s5k5ccg_sensor_write_list(s5k5ccg_30_fps,
					"s5k5ccgx_fps_30fix");
			break;
		default:
			pr_info("Invalid fps set %d. Change fps-30!\n", fps);
			s5k5ccg_sensor_write_list(s5k5ccg_30_fps,
					"s5k5ccgx_fps_30fix");
			break;
		}
	} else {
		s5k5ccg_sensor_write_list(s5k5ccg_fps_nonfix,
				"s5k5ccgx_fps_nonfix");
	}
	return rc;
}

static long
s5k5ccg_set_flash_mode(int mode, int flash)
{
	long rc = 0;
	pr_info("[%s::%d] mode[%d], flash[%d]\n",
		__func__, __LINE__, mode, flash);
	if (mode == 0) {
		s5k5ccg_set_flash(flash);
		if (flash == 0)	{
			if (flash_set_on_snap) {
				s5k5ccg_sensor_write_list(
					S5K5CCG_Flash_End_EVT1,
					"S5K5CCG_Flash_End_EVT1");
				   flash_set_on_snap = 0;
			}
		}
		return 0;
	} else if (mode == 50) {
		flash_mode = flash;

		switch (flash) {
		case FLASH_MODE_OFF_S:
				s5k5ccg_set_flash(0);
			break;
		case FLASH_MODE_ON_S:
			break;
		case FLASH_MODE_AUTO_S:
			break;
		case FLASH_TORCH_MODE_ON_S:	/* for 3rd party flash app */
			s5k5ccg_set_flash(MOVIEMODE_FLASH);
			break;
		default:
			printk(KERN_DEBUG "unexpected Flash value %s/%d\n",
				__func__, __LINE__);
		}
	}
	return rc;
}
/*
static void
s5k5ccg_sensor_reset_af_position(void)
{
	printk(KERN_DEBUG "RESET AF POSITION\n");
}
*/
static void
s5k5ccg_sensor_set_ae_lock(void)
{
	CAMDRV_DEBUG("Enter!\n");
	if (vtmode == 3)
		return;

	AELock = 1;
	s5k5ccg_sensor_write(0xFCFC, 0xD000);
	s5k5ccg_sensor_write(0x0028, 0x7000);
	s5k5ccg_sensor_write(0x002A, 0x2A5A);
	s5k5ccg_sensor_write(0x0F12, 0x0000);	/* Mon_AAIO_bAE */
}

static void
s5k5ccg_sensor_set_awb_lock(void)
{
	CAMDRV_DEBUG("Enter!\n");
	if (vtmode == 3)
		return;

	AWBLock = 1;
	s5k5ccg_sensor_write(0xFCFC, 0xD000);
	s5k5ccg_sensor_write(0x0028, 0x7000);
	s5k5ccg_sensor_write(0x002A, 0x11D6);
	s5k5ccg_sensor_write(0x0F12, 0xFFFF);	/*awbb_WpFilterMinThr */
}


static void
s5k5ccg_sensor_set_awb_unlock(void)
{
	CAMDRV_DEBUG("Enter!\n");
	if (AWBLock == 0)
		return;

	AWBLock = 0;
	s5k5ccg_sensor_write(0xFCFC, 0xD000);
	s5k5ccg_sensor_write(0x0028, 0x7000);
	s5k5ccg_sensor_write(0x002A, 0x11D6);
	s5k5ccg_sensor_write(0x0F12, 0x001E);	/*awbb_WpFilterMinThr */
}

static void
s5k5ccg_sensor_set_ae_unlock(void)
{
	CAMDRV_DEBUG("Enter!\n");
	if (AELock == 0)
		return;

	AELock = 0;
	s5k5ccg_sensor_write(0xFCFC, 0xD000);
	s5k5ccg_sensor_write(0x0028, 0x7000);
	s5k5ccg_sensor_write(0x002A, 0x2A5A);
	s5k5ccg_sensor_write(0x0F12, 0x0001);	/*MON_AAIO_bAE */
}

static int
s5k5ccg_set_ae_awb(int lock)
{
	CAMDRV_DEBUG("Enter [value = %d]\n", lock);
	if (lock == 1) {
		s5k5ccg_sensor_set_ae_lock();
		s5k5ccg_sensor_set_awb_lock();
	} else if (lock == 0) {
		s5k5ccg_sensor_set_ae_unlock();
		s5k5ccg_sensor_set_awb_unlock();
	}

	return 0;
}

static int
s5k5ccg_sensor_get_lux_value(void)
{
	int msb = 0;
	int lsb = 0;
	int cur_lux = 0;

	CAMDRV_DEBUG("Enter!\n");
	s5k5ccg_sensor_write(0xFCFC, 0xD000);
	s5k5ccg_sensor_write(0x002C, 0x7000);
	s5k5ccg_sensor_write(0x002E, 0x2A3C);
	s5k5ccg_sensor_read(0x0F12, (unsigned short *) &lsb);
	s5k5ccg_sensor_read(0x0F12, (unsigned short *) &msb);
	cur_lux = (msb << 16) | lsb;

	return cur_lux;
}

static int
s5k5ccg_af_set_status(int type)
{
	unsigned short read_value;
	int err, count;
	int ret = 0;
	int lux = 0;
	af_result = 1;

	CAMDRV_DEBUG("Enter [value = %d]\n", type);

	switch (type) {
	case 1:
		preflash_on = 0;

		lux = s5k5ccg_sensor_get_lux_value();	/*read rux */
		if ((lux < 0x0020) || (flash_mode == FLASH_MODE_ON_S)) {
			if (flash_mode != FLASH_MODE_OFF_S) {
				preflash_on = 1;
				flash_set_on_af = 1;
				s5k5ccg_sensor_write_list(S5K5CCG_AE_SPEEDUP,
							"S5K5CCG_AE_SPEEDUP");
				s5k5ccg_sensor_write_list(
						S5K5CCG_Pre_Flash_Start_EVT1,
						"S5K5CCG_Pre_Flash_Start_EVT1");
				s5k5ccg_set_flash(MOVIEMODE_FLASH);/*flash */
				for (count = 0; count < 40; count++) {
					/*Check AE stable check */
					err = s5k5ccg_sensor_write(
								0xFCFC, 0xD000);
					err = s5k5ccg_sensor_write(
								0x002C, 0x7000);
					err = s5k5ccg_sensor_write(
								0x002E, 0x1E3C);
					err = s5k5ccg_sensor_read(
						0x0F12, &read_value);
					if (err < 0) {
						printk(KERN_DEBUG
								"CAM 3M AE Status fail\n");
						return 0;
					}
					/*printk(KERN_DEBUG "CAM 3M AE Status
					Value = %x\n", read_value); */

					if (read_value == 0x0001) {
						/*AE stable */
						break;
					} else {
						/*msleep(10);
						changed for coding rule */
						/*mdelay(10);*/
						usleep(10*1000);
						continue;	/*progress */
					}
				}
			}
		}
		/* AF start */
		af_status = af_running;

		s5k5ccg_sensor_set_ae_lock();	/* lock AE */
		if (flash_set_on_af == 0)
			s5k5ccg_sensor_set_awb_lock();	/* lock AWB */

		s5k5ccg_sensor_write_list(s5k5ccg_af_on, "s5k5ccg_af_on");

		if (sceneNight == 1 || fireWorks == 1)
			msleep(600);/* delay 2frames before af status check */
		else
			msleep(260);/* delay 2frames before af status check */

		/*1st AF search */
		for (count = 0; count < 25; count++) {
			/*Check AF Result */
			err = s5k5ccg_sensor_write(0xFCFC, 0xD000);
			err = s5k5ccg_sensor_write(0x002C, 0x7000);
			err = s5k5ccg_sensor_write(0x002E, 0x2D12);
			err = s5k5ccg_sensor_read(0x0F12, &read_value);

			if (err < 0) {
				printk(KERN_DEBUG
					"CAM 3M 1st AF Status fail\n");
				af_result = 0;
				break;
			}
			printk(
				KERN_DEBUG "CAM 3M 1st AF Status Value = %x\n",
				read_value);

			if (read_value == 0x0001) {
				if (sceneNight == 1)
					msleep(200);
				else if (fireWorks == 1)
					msleep(300);
				else
					msleep(67);
				continue;		/*progress */
			} else {
				break;
			}
		}
		if (read_value == 0x0002) {
			ret = 1;
			printk(KERN_DEBUG
				"CAM 3M 1st AF_Single Mode SUCCESS. \r\n");
			msleep(130);/* delay 1frames before af status check */

			/*2nd AF search */
			for (count = 0; count < 15; count++) {
				/*Check AF Result */
				err = s5k5ccg_sensor_write(0xFCFC, 0xD000);
				err = s5k5ccg_sensor_write(0x002C, 0x7000);
				err = s5k5ccg_sensor_write(0x002E, 0x1F2F);
				err = s5k5ccg_sensor_read(0x0F12, &read_value);

				if (err < 0) {
					printk(KERN_DEBUG
						"CAM 3M 2nd AF Status fail\n");
					af_result = 0;
					break;
				}
				printk(
					KERN_DEBUG "CAM 3M 2nd AF "
					"Status Value = %x\n", read_value);
				if (read_value < 256) {
					printk(KERN_DEBUG
						"CAM 3M 2nd AF "
						"Status success = %x\n",
						read_value);
					break;
				} else {
					msleep(67);
					continue;	/*progress */
				}
			}
		} else if (read_value == 0x0004) {
			ret = 0;
			af_result = 2;
			printk(KERN_DEBUG
				"CAM 3M AF_Single Mode Fail.==> Cancel\n");
		} else {
			ret = 0;
			af_result = 0;
			printk(KERN_DEBUG
				"CAM 3M AF_Single Mode Fail.==> FAIL\n");

		}

		if (flash_set_on_af) {
			s5k5ccg_sensor_write_list(S5K5CCG_AE_SPEEDNORMAL,
			"S5K5CCG_AE_SPEEDNORMAL");
			s5k5ccg_sensor_write_list(S5K5CCG_Pre_Flash_End_EVT1,
			"S5K5CCG_Pre_Flash_End_EVT1");
			flash_set_on_af = 0;
		}

		if (preflash_on == 1)
			s5k5ccg_set_flash(0);	/*flash */

		if (af_result != 1) {
			/* AW & AWB UNLOCK */
			s5k5ccg_sensor_set_ae_unlock();
			s5k5ccg_sensor_set_awb_unlock();
			if (af_mode == 1) {
#ifdef I2C_BURST_MODE
				s5k5ccg_sensor_burst_write_list(
						s5k5ccg_af_macro,
						"s5k5ccg_af_macro");
#else
				s5k5ccg_sensor_write_list(s5k5ccg_af_macro,
						"s5k5ccg_af_macro");
#endif
			} else if (af_mode == 2) {
#ifdef I2C_BURST_MODE
				s5k5ccg_sensor_burst_write_list(s5k5ccg_af_auto,
				"s5k5ccg_af_auto");
#else
				s5k5ccg_sensor_write_list(s5k5ccg_af_auto,
				"s5k5ccg_af_auto");
#endif
			}
		}
		af_status = af_stop;
		printk(KERN_DEBUG "CAM:3M AF_SINGLE SET \r\n");
		break;


	case 0:	/* Cancel AF */
		af_status = af_stop;

		/* AW & AWB UNLOCK */
		s5k5ccg_sensor_set_ae_unlock();
		s5k5ccg_sensor_set_awb_unlock();
		if (af_mode == 1) {
#ifdef I2C_BURST_MODE
				s5k5ccg_sensor_burst_write_list(
						s5k5ccg_af_macro,
						"s5k5ccg_af_macro");
#else
				s5k5ccg_sensor_write_list(s5k5ccg_af_macro,
						"s5k5ccg_af_macro");
#endif
		} else if (af_mode == 2) {
#ifdef I2C_BURST_MODE
				s5k5ccg_sensor_burst_write_list(s5k5ccg_af_auto,
				"s5k5ccg_af_auto");
#else
				s5k5ccg_sensor_write_list(s5k5ccg_af_auto,
				"s5k5ccg_af_auto");
#endif
		}
		break;

	default:
		pr_info(" af status value : %d\n", type);
	}
	return 0;
}

static int
s5k5ccg_af_get_status(int type)
{
	CAMDRV_DEBUG("Enter [value = %d]\n", type);
	/* af_result = 0 -> fail
	 * af_result = 1 -> success
	 * af_result = 2 -> cancel
	 */
	return af_result;
}

static int
s5k5ccg_af_set_mode(int type)
{
	unsigned short read_value;
	int err, count;
	int ret = 0;
	/*int size = 0;
	int i = 0;
	unsigned short light = 0;*/
	int lux = 0;

	CAMDRV_DEBUG("Enter [value = %d]\n", type);
	/*printk(KERN_DEBUG
	"[CAM-SENSOR] s5k5ccg_sensor_af_control : %d, preview_start : %d\n",
	type, preview_start);*/

	switch (type) {
	case AF_MODE_MACRO_S:	/* macro */
		if (af_status == af_running || preview_start == 0)
			break;
		af_mode = 1;
#ifdef I2C_BURST_MODE
		s5k5ccg_sensor_burst_write_list(s5k5ccg_af_macro,
			"s5k5ccg_af_macro");
#else
		s5k5ccg_sensor_write_list(s5k5ccg_af_macro,
			"s5k5ccg_af_macro");
#endif
		break;


	case AF_MODE_AUTO_S:		/* auto */
		if (af_status == af_running || preview_start == 0)
			break;
		af_mode = 2;
#ifdef I2C_BURST_MODE
		s5k5ccg_sensor_burst_write_list(s5k5ccg_af_auto,
			"s5k5ccg_af_auto");
#else
		s5k5ccg_sensor_write_list(s5k5ccg_af_auto, "s5k5ccg_af_auto");
#endif
		break;

	default:
		pr_info("unexpected AF MODE : %d\n", type);
		break;
	}

	return ret;
}


int s5k5ccg_set_touchaf_pos(int x, int y)
{
	u16 mapped_x = 0;
	u16 mapped_y = 0;
	u16 inner_window_start_x = 0;
	u16 inner_window_start_y = 0;
	u16 outer_window_start_x = 0;
	u16 outer_window_start_y = 0;

	u16 sensor_width    = 0;
	u16 sensor_height   = 0;
	u16 inner_window_width = 0;
	u16 inner_window_height = 0;
	u16 outer_window_width = 0;
	u16 outer_window_height = 0;

	u16 touch_width = 0;
	u16 touch_height = 0;

	CAMDRV_DEBUG("Enter [x = %d, y = %d]\n", x, y);
/*
	int err = 0;
	int read_value = 0;

	err = s5k5ccg_sensor_write(0xFCFC, 0xD000);
	err = s5k5ccg_sensor_write(0x002C, 0x7000);
	err = s5k5ccg_sensor_write(0x002E, 0x0238);
	err = s5k5ccg_sensor_read(0x0F12, &read_value);
	pr_info("naruto, read value : %d", read_value);
	inner_window_width = read_value * 640 / 1024;

	err = s5k5ccg_sensor_write(0xFCFC, 0xD000);
	err = s5k5ccg_sensor_write(0x002C, 0x7000);
	err = s5k5ccg_sensor_write(0x002E, 0x023A);
	err = s5k5ccg_sensor_read(0x0F12, &read_value);
	pr_info("naruto, read value : %d", read_value);
	inner_window_height = read_value * 480 / 1024;

	err = s5k5ccg_sensor_write(0xFCFC, 0xD000);
	err = s5k5ccg_sensor_write(0x002C, 0x7000);
	err = s5k5ccg_sensor_write(0x002E, 0x0230);
	err = s5k5ccg_sensor_read(0x0F12, &read_value);
	pr_info("naruto, read value : %d", read_value);
	outer_window_width = read_value * 640 / 1024;

	err = s5k5ccg_sensor_write(0xFCFC, 0xD000);
	err = s5k5ccg_sensor_write(0x002C, 0x7000);
	err = s5k5ccg_sensor_write(0x002E, 0x0232);
	err = s5k5ccg_sensor_read(0x0F12, &read_value);
	pr_info("naruto, read value : %d", read_value);
	outer_window_height = read_value * 480 / 1024;
*/
		sensor_width		= 640;
		sensor_height		= 480;
		inner_window_width  = 230;
		inner_window_height = 306;
		outer_window_width  = 512;
		outer_window_height = 568;
		touch_width			= 640;
		touch_height		= 480;

	/*pr_info("xPos = %d, yPos = %d\n", x, y);*/


	/* mapping the touch position on the sensor display*/
	mapped_x = (x * sensor_width) / touch_width;
	mapped_y = (y * sensor_height) / touch_height;
	/*pr_info("mapped xPos = %d, mapped yPos = %d\n", mapped_x, mapped_y);*/

	/* set X axis*/
	if (mapped_x <= (inner_window_width / 2)) {
		inner_window_start_x    = 0;
		outer_window_start_x    = 0;
		/*pr_info("inbox over the left side. boxes are left
		  side align in_Sx = %d, out_Sx= %d",
		  inner_window_start_x, outer_window_start_x);*/
	} else if (mapped_x <= (outer_window_width / 2)) {
		inner_window_start_x    = mapped_x - (inner_window_width / 2);
		outer_window_start_x    = 0;
		/*pr_info("outbox only over the left side. outbox is only left
		  side align in_Sx = %d, out_Sx= %d",
		  inner_window_start_x, outer_window_start_x);*/
	} else if (mapped_x >= ((sensor_width - 1) -
				(inner_window_width / 2))) {
		inner_window_start_x = (sensor_width - 1) -
			inner_window_width;
		outer_window_start_x = (sensor_width - 1) -
			outer_window_width;
		/*pr_info("inbox over the right side. boxes are rightside
		  align in_Sx = %d, out_Sx= %d",
		  inner_window_start_x, outer_window_start_x);*/
	} else if (mapped_x >= ((sensor_width - 1) -
				(outer_window_width / 2))) {
		inner_window_start_x = mapped_x -
			(inner_window_width / 2);
		outer_window_start_x = (sensor_width - 1) -
			outer_window_width;
		/*pr_info("outbox only over the right side.
		  out box is only right
		  side align in_Sx = %d, out_Sx= %d",
		  inner_window_start_x, outer_window_start_x);*/
	} else {
		inner_window_start_x = mapped_x -
			(inner_window_width / 2);
		outer_window_start_x = mapped_x -
			(outer_window_width / 2);
		/*pr_info("boxes are in the sensor window. in_Sx = %d,
		  out_Sx= %d\n\n", inner_window_start_x,
		  outer_window_start_x);*/
	}


	/* set Y axis*/
	if (mapped_y <= (inner_window_height / 2)) {
		inner_window_start_y    = 0;
		outer_window_start_y    = 0;
		/*pr_info("inbox over the top side. boxes are top side
		  align in_Sy = %d, out_Sy= %d", inner_window_start_y,
		  outer_window_start_y);*/
	} else if (mapped_y <= (outer_window_height / 2)) {
		inner_window_start_y    = mapped_y - (inner_window_height / 2);
		outer_window_start_y    = 0;
		/*pr_info("outbox only over the top side. outbox is
		  only top side align in_Sy = %d, out_Sy= %d",
		  inner_window_start_y, outer_window_start_y);*/
	} else if (mapped_y >= ((sensor_height - 1) -
				(inner_window_height / 2))) {
		inner_window_start_y = (sensor_height - 1) -
			inner_window_height;
		outer_window_start_y = (sensor_height - 1) -
			outer_window_height;
		/*pr_info("inbox over the bottom side. boxes are bottom
		  side align in_Sy = %d, out_Sy= %d",
		  inner_window_start_y, outer_window_start_y);*/
	} else if (mapped_y >= ((sensor_height - 1) -
				(outer_window_height / 2))) {
		inner_window_start_y = mapped_y -
			(inner_window_height / 2);
		outer_window_start_y = (sensor_height - 1) -
			outer_window_height;
		/*pr_info("outbox only over the bottom side. out box is
		  only bottom side align in_Sy = %d, out_Sy= %d",
		  inner_window_start_y, outer_window_start_y);*/
	} else {
		inner_window_start_y = mapped_y -
			(inner_window_height / 2);
		outer_window_start_y = mapped_y -
			(outer_window_height / 2);
		/*pr_info("boxes are in the sensor window. in_Sy = %d,
		  out_Sy= %d\n\n", inner_window_start_y,
		  outer_window_start_y);*/
	}

	/*calculate the start position value*/
	inner_window_start_x = inner_window_start_x * 1024 / sensor_width;
	outer_window_start_x = outer_window_start_x * 1024 / sensor_width;
	inner_window_start_y = inner_window_start_y * 1024 / sensor_height;
	outer_window_start_y = outer_window_start_y * 1024 / sensor_height;
	/*
	pr_info("calculated value inner_window_start_x = %d\n",
	inner_window_start_x);
	pr_info("calculated value inner_window_start_y = %d\n",
	inner_window_start_y);
	pr_info("calculated value outer_window_start_x = %d\n",
	outer_window_start_x);
	pr_info("calculated value outer_window_start_y = %d\n",
	outer_window_start_y);
	*/

	/*Write register*/
	s5k5ccg_sensor_write(0x0028, 0x7000);

	/*inner_window_start_x*/
	s5k5ccg_sensor_write(0x002A, 0x0234);
	s5k5ccg_sensor_write(0x0F12, inner_window_start_x);

	/*outer_window_start_x*/
	s5k5ccg_sensor_write(0x002A, 0x022C);
	s5k5ccg_sensor_write(0x0F12, outer_window_start_x);

	/*inner_window_start_y*/
	s5k5ccg_sensor_write(0x002A, 0x0236);
	s5k5ccg_sensor_write(0x0F12, inner_window_start_y);

	/*outer_window_start_y*/
	s5k5ccg_sensor_write(0x002A, 0x022E);
	s5k5ccg_sensor_write(0x0F12, outer_window_start_y);

	/*Update AF window*/
	s5k5ccg_sensor_write(0x002A, 0x023C);
	s5k5ccg_sensor_write(0x0F12, 0x0001);

	/*pr_info("update AF window and sleep 100ms");*/
	msleep(100);

	return 0;
}

int s5k5ccg_reset_AF_region(void)
{
	u16 mapped_x = 320;
	u16 mapped_y = 240;
	u16 inner_window_start_x = 0;
	u16 inner_window_start_y = 0;
	u16 outer_window_start_x = 0;
	u16 outer_window_start_y = 0;

	CAMDRV_DEBUG("Enter!\n");
	/* mapping the touch position on the sensor display*/
	mapped_x = (mapped_x * 640) / 640;
	mapped_y = (mapped_y * 480) / 480;

	inner_window_start_x    = mapped_x - (230 / 2);
	outer_window_start_x    = mapped_x - (512 / 2);

	/*pr_info("boxes are in the sensor window. in_Sx = %d,
	  out_Sx= %d", inner_window_start_x, outer_window_start_x);*/

	inner_window_start_y = mapped_y - (306 / 2);
	outer_window_start_y = mapped_y - (568 / 2);
	/*pr_info("boxes are in the sensor window. in_Sy = %d,
	  out_Sy= %d", inner_window_start_y, outer_window_start_y);
	*/


	/*calculate the start position value*/
	inner_window_start_x = inner_window_start_x * 1024 / 640;
	outer_window_start_x = outer_window_start_x * 1024 / 640;
	inner_window_start_y = inner_window_start_y * 1024 / 480;
	outer_window_start_y = outer_window_start_y * 1024 / 480;
	/*pr_info("calculated value inner_window_start_x = %d",
	  inner_window_start_x);
	  pr_info("calculated value inner_window_start_y = %d",
	  inner_window_start_y);
	  pr_info("calculated value outer_window_start_x = %d",
	  outer_window_start_x);
	  pr_info("calculated value outer_window_start_y = %d",
	  outer_window_start_y);
	*/


	/*Write register*/
	s5k5ccg_sensor_write(0x0028, 0x7000);

	/*inner_window_start_x*/
	s5k5ccg_sensor_write(0x002A, 0x0234);
	s5k5ccg_sensor_write(0x0F12, inner_window_start_x);

	/*outer_window_start_x*/
	s5k5ccg_sensor_write(0x002A, 0x022C);
	s5k5ccg_sensor_write(0x0F12, outer_window_start_x);

	/*inner_window_start_y*/
	s5k5ccg_sensor_write(0x002A, 0x0236);
	s5k5ccg_sensor_write(0x0F12, inner_window_start_y);

	/*outer_window_start_y*/
	s5k5ccg_sensor_write(0x002A, 0x022E);
	s5k5ccg_sensor_write(0x0F12, outer_window_start_y);

	/*Update AF window*/
	s5k5ccg_sensor_write(0x002A, 0x023C);
	s5k5ccg_sensor_write(0x0F12, 0x0001);
	return 0;
}

static long
s5k5ccg_set_sensor_mode(int mode)
{
	int lux = 0;
	int cnt, vsync_value;

	CAMDRV_DEBUG("Enter [value = %d]\n", mode);
	/*      printk(KERN_DEBUG "[PGH] Sensor Mode\n"); */
	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		printk(KERN_DEBUG "Preview!!!\n");
		af_status = af_stop;
		low_temp = 0;

		/*                if(sceneNight == 1 || fireWorks == 1) */
		/*                { */
/*		for (cnt = 0; cnt < 200; cnt++) {
			vsync_value = gpio_get_value(14);

			if (vsync_value) {
				printk(KERN_DEBUG
				" on preview,  end cnt:%d vsync_value:%d\n",
				cnt, vsync_value);
				break;
			} else {*/
				/*printk(KERN_DEBUG " on preview,
				wait cnt:%d vsync_value:%d\n", cnt,
				vsync_value); */
				/*msleep(1);	changed for coding rule */
				/*udelay(1000);*/
/*			}
		}*/
		/*                } */
		if (preview_start == 0)
			preview_start = 1;
		else if (preview_start == 1) {
#ifdef I2C_BURST_MODE
			s5k5ccg_sensor_burst_write_list(s5k5ccg_preview,
					"s5k5ccg_preview");
#else
			s5k5ccg_sensor_write_list(s5k5ccg_preview,
					"s5k5ccg_preview");
#endif
			if (sceneNight == 1 || fireWorks == 1)
				msleep(300);
		}
		break;

	case SENSOR_SNAPSHOT_MODE:
		printk(KERN_DEBUG "[PGH] Capture\n");

		lux = s5k5ccg_sensor_get_lux_value();	/*read rux */

		if (((lux < 0x0020) || (flash_mode == FLASH_MODE_ON_S) ||
			preflash_on == 1) &&
			(flash_mode != FLASH_MODE_OFF_S)) {	/*flash on */
			preflash_on = 0;
			flash_set_on_snap = 1;
			s5k5ccg_sensor_write_list(S5K5CCG_Flash_Start_EVT1,
			"S5K5CCG_Flash_Start_EVT1");
			if (af_mode == 1)
				s5k5ccg_set_flash(MOVIEMODE_FLASH);
			else
				s5k5ccg_set_flash(FLASHMODE_FLASH);
			exif_flash_status = 1;
		} else {
			exif_flash_status = 0;
			s5k5ccg_sensor_set_ae_lock();	/* lock AE */
		}

		for (cnt = 0; cnt < 700; cnt++) {
			vsync_value = gpio_get_value(14);

			if (vsync_value) {
				printk(KERN_DEBUG
				" on snapshot,  end cnt:%d vsync_value:%d\n",
				cnt, vsync_value);
				break;
			} else {
				/*printk(KERN_DEBUG " on snapshot,
				wait cnt:%d vsync_value:%d\n", cnt,
				vsync_value); */
				/*msleep(3);	changed for coding rule */
				/*mdelay(3);*/
				usleep(3*1000);
			}
		}

		if (lux > 0xFFFE) {		/* highlight snapshot */
			/*printk(KERN_DEBUG "Snapshot : highlight
			snapshot\n"); */
#ifdef I2C_BURST_MODE
			s5k5ccg_sensor_burst_write_list(
			s5k5ccg_snapshot_normal_high,
			"s5k5ccg_snapshot_normal_high");
			s5k5ccg_sensor_set_ae_unlock();
			s5k5ccg_sensor_set_awb_unlock();
#else
			s5k5ccg_sensor_write_list(s5k5ccg_snapshot_normal_high,
			"s5k5ccg_snapshot_normal_high");
			s5k5ccg_sensor_set_ae_unlock();
			s5k5ccg_sensor_set_awb_unlock();
#endif
		} else if (lux > 0x0020) {	/* Normal snapshot */
			/*                printk(KERN_DEBUG "Snapshot
						: Normal snapshot\n"); */

#ifdef I2C_BURST_MODE
			s5k5ccg_sensor_burst_write_list(
				s5k5ccg_capture_normal_normal,
				"s5k5ccg_capture_normal_normal");
			s5k5ccg_sensor_set_ae_unlock();
			s5k5ccg_sensor_set_awb_unlock();
#else
			s5k5ccg_sensor_write_list(
				s5k5ccg_capture_normal_normal,
				"s5k5ccg_capture_normal_normal");
			s5k5ccg_sensor_set_ae_unlock();
			s5k5ccg_sensor_set_awb_unlock();
#endif
		} else {			/*lowlight snapshot */
			/*printk(KERN_DEBUG "Snapshot : lowlight
			snapshot\n"); */
#ifdef I2C_BURST_MODE
			if (sceneNight == 1 || fireWorks == 1) {
				s5k5ccg_sensor_burst_write_list(
					s5k5ccg_snapshot_nightmode,
					"s5k5ccg_snapshot_nightmode");
			} else {
				s5k5ccg_sensor_burst_write_list(
					s5k5ccg_snapshot_normal_low,
					"s5k5ccg_snapshot_normal_low");
			}

			s5k5ccg_sensor_set_ae_unlock();	/* aw unlock */
			s5k5ccg_sensor_set_awb_unlock();
#else
			if (sceneNight == 1 || fireWorks == 1) {
				s5k5ccg_sensor_write_list(
					s5k5ccg_snapshot_nightmode,
					"s5k5ccg_snapshot_nightmode");
			} else {
				s5k5ccg_sensor_write_list(
					s5k5ccg_snapshot_normal_low,
					"s5k5ccg_snapshot_normal_low");
			}
			s5k5ccg_sensor_set_ae_unlock();	/* aw unlock */
			s5k5ccg_sensor_set_awb_unlock();
#endif
		}
		s5k5ccg_reset_AF_region();
		pr_info(" reset_AF\n");
		break;

	case SENSOR_RAW_SNAPSHOT_MODE:
		printk(KERN_DEBUG "[PGH}-> Capture RAW\n");
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

int
s5k5ccg_sensor_init_probe(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	unsigned short id = 0;	/*PGH FOR TEST */

	sceneNight = 0;
	fireWorks = 0;
	AELock = 0;
	AWBLock = 0;
	preview_start = 0;
	af_mode = 0;
	flash_set_on_af = 0;
	flash_set_on_snap = 0;
	preflash_on = 0;
	exif_flash_status = 0;
	low_temp = 0;
	flash_mode = 0;

	pr_info("%s/%d\n", __func__, __LINE__);
/*
	s5k5ccg_sensor_write(0x002C, 0x7000);
	s5k5ccg_sensor_write(0x002E, 0x0152);
	s5k5ccg_sensor_read(0x0F12, &id);
	printk(KERN_DEBUG "[PGH] SENSOR FW => id 0x%04x\n", id);

	s5k5ccg_sensor_write(0x002C, 0xD000);
	s5k5ccg_sensor_write(0x002E, 0x0040);
	s5k5ccg_sensor_read(0x0F12, &id);
	printk(KERN_DEBUG "[PGH] SENSOR FW => id 0x%04x\n", id);
*/
#ifdef I2C_BURST_MODE		/*dha23 100325 */
	s5k5ccg_sensor_burst_write_list(s5k5ccg_init0, "s5k5ccg_init0");
#else
	s5k5ccg_sensor_write_list(s5k5ccg_init0, "s5k5ccg_init0");
#endif
	/*msleep(10);		changed for coding rule */
	/*mdelay(10);*/
/*	usleep(10*1000);*/

#ifdef I2C_BURST_MODE		/*dha23 100325 */
	s5k5ccg_sensor_burst_write_list(s5k5ccg_init1, "s5k5ccg_init1");
#else
	s5k5ccg_sensor_write_list(s5k5ccg_init1, "s5k5ccg_init1");
#endif

	return rc;
}

static int
cam_hw_init()
{
	int rc = 0;

	struct vreg *vreg_L16;
	struct vreg *vreg_L8;


	gpio_tlmm_config(
		GPIO_CFG(CAM_RESET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		GPIO_CFG_16MA), GPIO_CFG_ENABLE);	/*CAM_RESET */
	gpio_tlmm_config(
		GPIO_CFG(CAM_STANDBY, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		GPIO_CFG_16MA), GPIO_CFG_ENABLE);	/*CAM_STANDBY */
	gpio_tlmm_config(
		GPIO_CFG(CAM_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		GPIO_CFG_16MA), GPIO_CFG_ENABLE);	/*CAM_EN */
	gpio_tlmm_config(
		GPIO_CFG(CAM_EN_2, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		GPIO_CFG_16MA), GPIO_CFG_ENABLE);	/*CAM_EN_2 */
	gpio_tlmm_config(
		GPIO_CFG(CAM_VT_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		GPIO_CFG_16MA), GPIO_CFG_ENABLE);	/*CAM_VT_RST */
	gpio_tlmm_config(
		GPIO_CFG(CAM_VT_nSTBY, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		GPIO_CFG_16MA), GPIO_CFG_ENABLE);	/*CAM_VT_nSTBY */

	vreg_L16 = vreg_get(NULL, "gp10");
	vreg_set_level(vreg_L16, 3000);
	vreg_L8 = vreg_get(NULL, "gp7");
	vreg_set_level(vreg_L8, 1800);

	gpio_set_value(CAM_EN_2, 1); /*EN_2 -> UP 1.2V, 1.8V*/
	gpio_set_value(CAM_EN, 1);	/*EN -> UP */

	vreg_enable(vreg_L8);
	udelay(1);

	gpio_set_value(CAM_VT_nSTBY, 1); /*VT_nSTBY -> UP*/
	udelay(1);

	/* Input MCLK = 24MHz */
	gpio_tlmm_config(GPIO_CFG(CAM_MCLK, 1,
		GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		GPIO_CFG_ENABLE);	/*CAM_MCLK */
	msm_camio_clk_rate_set(24000000);	/*MCLK */
	msm_camio_camif_pad_reg_reset();
	/*mdelay(10);*/

	gpio_set_value(CAM_VT_RST, 1); /*VT_RST -> UP*/
	mdelay(2.4);

	gpio_set_value(CAM_VT_nSTBY, 0); /*VT_nSTBY -> DOWN*/
	/*mdelay(10);*/
	mdelay(5);

	gpio_set_value(CAM_STANDBY, 1); /*STBY -> UP*/
	udelay(15);

	gpio_set_value(CAM_RESET, 1); /*REST -> UP*/
	/*mdelay(10);*/
	mdelay(5);


	vreg_enable(vreg_L16);
	mdelay(1);

	return rc;
}


#ifdef CONFIG_LOAD_FILE

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>

/*#include <asm/uaccess.h>		changed for coding rule */
#include <linux/uaccess.h>

static char *s5k5ccg_regs_table;

static int s5k5ccg_regs_table_size;

int
s5k5ccg_regs_table_init(void)
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
	changed for coding rule
	//filp = filp_open("/data/camera/s5k5cc.h", O_RDONLY, 0);
	filp = filp_open("/data/s5k5cc.h", O_RDONLY, 0);
*/
	filp = filp_open("/mnt/sdcard/s5k5ccgx.h", O_RDONLY, 0);
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
	ret = vfs_read(filp, (char __user *) dp, l, &pos);
	if (ret != l) {
		printk(KERN_DEBUG "Failed to read file ret = %d\n", ret);
		kfree(dp);
		filp_close(filp, current->files);
		return -EINVAL;
	}

	filp_close(filp, current->files);

	set_fs(fs);

	s5k5ccg_regs_table = dp;

	s5k5ccg_regs_table_size = l;

	*((s5k5ccg_regs_table + s5k5ccg_regs_table_size) - 1) = '\0';

	/*printk(KERN_DEBUG "s5k5ccg_regs_table 0x%04x, %ld\n", dp, l); */
	return 0;
}

void
s5k5ccg_regs_table_exit(void)
{
	printk(KERN_DEBUG "%s %d\n", __func__, __LINE__);
/*
	changed for coding rule
	if (s5k5ccg_regs_table) {
		kfree(s5k5ccg_regs_table);
		s5k5ccg_regs_table = NULL;
	}
*/
	kfree(s5k5ccg_regs_table);
	s5k5ccg_regs_table = NULL;
}

static int
s5k5ccg_regs_table_write(char *name)
{
	char *start, *end, *reg;
	unsigned short addr, value;
	char reg_buf[7], data_buf[7];

	*(reg_buf + 6) = '\0';
	*(data_buf + 6) = '\0';


	start = strnstr(s5k5ccg_regs_table, name, s5k5ccg_regs_table_size);
	end = strnstr(start, "};", s5k5ccg_regs_table_size);

	while (1) {
		/* Find Address */
		reg = strnstr(start, "{ 0x", s5k5ccg_regs_table_size);
		if (reg)
			start = (reg + 16);
		if ((reg == NULL) || (reg > end))
			break;
		/* Write Value to Address */
		if (reg != NULL) {
			memcpy(reg_buf, (reg + 2), 6);
			memcpy(data_buf, (reg + 10), 6);
			kstrtol(data_buf, 16, &value);
			kstrtol(reg_buf, 16, &addr);

			if (addr == 0xdddd) {
				mdelay(value);
				printk(KERN_DEBUG "delay 0x%04x,"
					"value 0x%04x\n", addr, value);
			} else if (addr == 0xffff) {
				printk(KERN_DEBUG "0xffff\n");
			} else
				s5k5ccg_sensor_write(addr, value);
		}
	}

	return 0;
}

#endif



int
s5k5ccg_sensor_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	printk(KERN_DEBUG "%s/%d\n", __func__, __LINE__);

#ifdef CONFIG_LOAD_FILE
	if (0 > s5k5ccg_regs_table_init()) {
		pr_info("s5k5ccg_sensor_init file open failed!\n");
		rc = -1;
		goto init_fail;
	}
#endif

	s5k5ccg_ctrl = kzalloc(sizeof(struct s5k5ccg_ctrl_t), GFP_KERNEL);
	if (!s5k5ccg_ctrl) {
		pr_info("s5k5ccg_init alloc mem failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	if (data)
		s5k5ccg_ctrl->sensordata = data;

	rc = cam_hw_init();
	if (rc < 0) {
		printk(KERN_DEBUG "<=PCAM=> cam_hw_init failed!\n");
		goto init_fail;
	}

	rc = s5k5ccg_sensor_init_probe(data);
	if (rc < 0) {
		pr_info("s5k5ccg_sensor_init init probe failed!\n");
		goto init_fail;
	}

init_done:
	return rc;

init_fail:
	kfree(s5k5ccg_ctrl);
	return rc;
}

int
s5k5ccg_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&s5k5ccg_wait_queue);
	return 0;
}

int s5k5ccg_sensor_ext_config(void __user *argp)
{
	struct sensor_ext_cfg_data cfg_data;
	int err = 0;

/*	printk(KERN_DEBUG "s5k5ccg_sensor_ext_config\n");*/

	if (copy_from_user(
		(void *)&cfg_data, (const void *)argp, sizeof(cfg_data)))
		printk(KERN_DEBUG "fail copy_from_user!\n");


	CAMDRV_DEBUG("cmd: %d, value1: %d, value2: %d\n", cfg_data.cmd,
			cfg_data.value_1, cfg_data.value_2);

	switch (cfg_data.cmd) {
	case EXT_CFG_SET_FLASH:
		err = s5k5ccg_set_flash_mode(
			cfg_data.value_1, cfg_data.value_2);
		break;
	case EXT_CFG_SET_BRIGHTNESS:
		err = s5k5ccg_set_exposure_value(0, cfg_data.value_1);
		break;

	case EXT_CFG_SET_EFFECT:
		err = s5k5ccg_set_effect(0, cfg_data.value_1);
		break;

	case EXT_CFG_SET_SCENE:
		err = s5k5ccg_set_scene_mode(0, cfg_data.value_1);
		break;

	case EXT_CFG_SET_ISO:
		err = s5k5ccg_set_ISO(0, cfg_data.value_1);
		break;

	case EXT_CFG_SET_WB:
		err = s5k5ccg_set_whitebalance(0, cfg_data.value_1);
		break;

	case EXT_CFG_SET_METERING:
		err = s5k5ccg_set_auto_exposure(0, cfg_data.value_1);
		break;

	case EXT_CFG_SET_TOUCHAF_POS:
		err = s5k5ccg_set_touchaf_pos(cfg_data.value_1,
				cfg_data.value_2);
		break;

	case EXT_CFG_SET_AF_STATUS:
		err = s5k5ccg_af_set_status(cfg_data.value_1);
		break;

	case EXT_CFG_GET_AF_STATUS:
		err = s5k5ccg_af_get_status(cfg_data.value_1);
		cfg_data.value_1 = err;
		break;

	case EXT_CFG_SET_AF_MODE:
		err = s5k5ccg_af_set_mode(cfg_data.value_1);
		break;

	case EXT_CFG_GET_EXIF:
		s5k5ccgx_get_exif(&cfg_data.value_1, &cfg_data.value_2);
		break;

	case EXT_CFG_SET_FPS:
		err = s5k5ccg_set_fps(cfg_data.value_1, cfg_data.value_2);
		break;
	case EXT_CFG_SET_AE_AWB:
		err = s5k5ccg_set_ae_awb(cfg_data.value_1);
		break;

	default:
		printk(KERN_DEBUG
		"<=PCAM=> unexpected ext_config %s/%d\n", __func__, __LINE__);
		break;
	}

	if (copy_to_user(
		(void *)argp, (const void *)&cfg_data, sizeof(cfg_data)))
		printk(KERN_DEBUG "fail copy_to_user!\n");

	return err;
}

int
s5k5ccg_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	long rc = 0;

	if (copy_from_user(&cfg_data,
		(void *) argp, sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	CAMDRV_DEBUG("[value : %d]\n", cfg_data.cfgtype);

/*	printk(KERN_DEBUG "s5k5ccg_ioctl, cfgtype = %d, mode = %d\n",
		cfg_data.cfgtype, cfg_data.mode);
*/
	switch (cfg_data.cfgtype) {
	case CFG_SET_MODE:
		rc = s5k5ccg_set_sensor_mode(cfg_data.mode);
		break;

	case CFG_SET_EFFECT:
		rc = s5k5ccg_set_effect(cfg_data.mode, cfg_data.cfg.effect);
		break;

	case CFG_SET_BRIGHTNESS:
		rc = s5k5ccg_set_brightness(cfg_data.mode,
			cfg_data.cfg.brightness);
		break;

	case CFG_SET_WB:
		rc = s5k5ccg_set_whitebalance(cfg_data.mode, cfg_data.cfg.wb);
		break;

	case CFG_SET_ISO:
		rc = s5k5ccg_set_ISO(cfg_data.mode, cfg_data.cfg.iso);
		break;

	case CFG_SET_EXPOSURE_MODE:
		rc = s5k5ccg_set_auto_exposure(cfg_data.mode,
			cfg_data.cfg.metering);
		break;

	case CFG_SET_EXPOSURE_VALUE:
		rc = s5k5ccg_set_exposure_value(cfg_data.mode,
			cfg_data.cfg.ev);
		break;
/*
	case CFG_SET_FLASH:
		rc = s5k5ccg_set_flash_mode(cfg_data.mode, cfg_data.cfg.flash);
		break;
*/
	case CFG_MOVE_FOCUS:
	/*	rc = s5k5ccg_sensor_af_control(cfg_data.cfg.focus.steps);
		if (cfg_data.cfg.focus.steps == AF_MODE_START) {
			cfg_data.rs = rc;
			if (copy_to_user
			((void *) argp, &cfg_data,
				sizeof(struct sensor_cfg_data)))
				return -EFAULT;
		}*/
		break;

	case CFG_SET_DEFAULT_FOCUS:
	/*	rc = s5k5ccg_sensor_af_control(cfg_data.cfg.focus.steps);*/
		break;

	case CFG_GET_AF_MAX_STEPS:
		/*cfg_data.max_steps = MT9T013_TOTAL_STEPS_NEAR_TO_FAR; */
		if (copy_to_user
		((void *) argp, &cfg_data, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_SET_SCENE_MODE:
		rc = s5k5ccg_set_scene_mode(cfg_data.mode, cfg_data.cfg.scene);
		break;

	case CFG_SET_DATALINE_CHECK:
		if (cfg_data.cfg.dataline) {
			printk(KERN_DEBUG
				"[ASWOOGI] CFG_SET_DATALINE_CHECK ON\n");
			s5k5ccg_sensor_write(0xFCFC, 0xD000);
			s5k5ccg_sensor_write(0x0028, 0xD000);
			s5k5ccg_sensor_write(0x002A, 0xB054);
			s5k5ccg_sensor_write(0x0F12, 0x0001);
		} else {
			printk(KERN_DEBUG
				"[ASWOOGI] CFG_SET_DATALINE_CHECK OFF\n");
			s5k5ccg_sensor_write(0xFCFC, 0xD000);
			s5k5ccg_sensor_write(0x0028, 0xD000);
			s5k5ccg_sensor_write(0x002A, 0xB054);
			s5k5ccg_sensor_write(0x0F12, 0x0000);
		}
		break;

	default:
		printk(KERN_DEBUG "[PGH] undefined cfgtype %s/%d %d\n",
			 __func__, __LINE__, cfg_data.cfgtype);
		/*rc = -EINVAL; */
		rc = 0;
		break;
	}

	/* up(&s5k5ccg_sem); */

	return rc;
}

int
s5k5ccg_sensor_release(void)
{
	int rc = 0;
	preview_start = 0;

	s5k5ccg_set_flash(0);	/*baiksh temp fix */
	/* down(&s5k5ccg_sem); */

	gpio_set_value(CAM_VT_nSTBY, 0); /*STBY -> DOWN*/
	vreg_disable(vreg_get(NULL, "gp10"));

	gpio_set_value(CAM_VT_RST, 0);	/*REST -> DOWN */
	/*mdelay(1);*/
	usleep(1*1000);
	gpio_set_value(CAM_RESET, 0); /*REST -> DOWN*/
	udelay(80);
	/*Clock Disable*/
	gpio_tlmm_config(GPIO_CFG(CAM_MCLK, 0,
		GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		GPIO_CFG_16MA), GPIO_CFG_ENABLE); /*CAM_MCLK*/
	udelay(10);

	gpio_set_value(CAM_STANDBY, 0); /*STBY -> DOWN*/
	udelay(10);

	gpio_set_value(CAM_EN_2, 0);
	vreg_disable(vreg_get(NULL, "gp7"));
	gpio_set_value(CAM_EN, 0);

	kfree(s5k5ccg_ctrl);
	/* up(&s5k5ccg_sem); */
#ifdef CONFIG_LOAD_FILE
	s5k5ccg_regs_table_exit();
#endif
	return rc;
}

int
s5k5ccg_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;
	printk(KERN_DEBUG "[PGH] %s/%d\n", __func__, __LINE__);


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	s5k5ccg_sensorw = kzalloc(sizeof(struct s5k5ccg_work), GFP_KERNEL);

	if (!s5k5ccg_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}
	camera_class = class_create(THIS_MODULE, "camera");

	if (IS_ERR(camera_class))
		pr_err("Failed to create class(camera)!\n");

	s5k5ccgx_dev = device_create(camera_class, NULL, 0, NULL, "rear");
	if (IS_ERR(s5k5ccgx_dev)) {
		pr_err("Failed to create device!");
		goto probe_failure;
	}

	if (device_create_file(s5k5ccgx_dev, &dev_attr_rear_camfw) < 0) {
		pr_info("failed to create device file, %s\n",
		dev_attr_rear_camfw.attr.name);
	}
	if (device_create_file(s5k5ccgx_dev, &dev_attr_rear_camtype) < 0) {
		pr_info("failed to create device file, %s\n",
		dev_attr_rear_camtype.attr.name);
	}
	if (device_create_file(s5k5ccgx_dev, &dev_attr_rear_flash) < 0) {
		pr_info("failed to create device file, %s\n",
		dev_attr_rear_flash.attr.name);
	}

	i2c_set_clientdata(client, s5k5ccg_sensorw);
	s5k5ccg_init_client(client);
	s5k5ccg_client = client;
	pr_info("s5k5ccg_probe succeeded!  %s/%d\n",
		__func__, __LINE__);
	CDBG("s5k5ccg_probe succeeded!\n");
	return 0;

probe_failure:
	kfree(s5k5ccg_sensorw);
	s5k5ccg_sensorw = NULL;
	pr_info("s5k5ccg_probe failed!\n");
	return rc;
}

static const struct i2c_device_id s5k5ccg_i2c_id[] = {
	{"s5k5ccgx", 0},
	{},
};

static struct i2c_driver s5k5ccg_i2c_driver = {
	.id_table = s5k5ccg_i2c_id,
	.probe = s5k5ccg_i2c_probe,
	.remove = __exit_p(s5k5ccg_i2c_remove),
	.driver = {
		.name = "s5k5ccgx",
	},
};


int
s5k5ccg_sensor_probe(const struct msm_camera_sensor_info *info,
		      struct msm_sensor_ctrl *s)
{
	int rc = 0;

	printk(KERN_DEBUG "[PGH]  %s/%d\n", __func__, __LINE__);
	rc = i2c_add_driver(&s5k5ccg_i2c_driver);
	if (rc < 0 || s5k5ccg_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_done;
	}

	printk(KERN_DEBUG "[PGH] s5k5ccg_client->addr : %x\n",
		s5k5ccg_client->addr);
	printk(KERN_DEBUG "[PGH] s5k5ccg_client->adapter->nr : %d\n",
		s5k5ccg_client->adapter->nr);


	s->s_init = s5k5ccg_sensor_init;
	s->s_release = s5k5ccg_sensor_release;
	s->s_config = s5k5ccg_sensor_config;
	s->s_ext_config	= s5k5ccg_sensor_ext_config;
	s->s_camera_type = BACK_CAMERA_2D;
	s->s_mount_angle = 0;

probe_done:
	pr_info("%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
}

int
__s5k5ccg_probe(struct platform_device *pdev)
{
	pr_info("%s/%d\n", __func__, __LINE__);
	return msm_camera_drv_start(pdev, s5k5ccg_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __s5k5ccg_probe,
	.driver = {
		.name = "msm_camera_s5k5ccgx",
		.owner = THIS_MODULE,
	},
};

int __init
s5k5ccg_init(void)
{
	printk(KERN_DEBUG "[PGH]  %s/%d\n", __func__, __LINE__);
	return platform_driver_register(&msm_camera_driver);
}

module_init(s5k5ccg_init);
