/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
 *
 * File Name          : k3dh_acc.c
 * Authors            : MSH - Motion Mems BU - Application Team
 *		      : Carmine Iascone (carmine.iascone@st.com)
 *		      : Matteo Dameno (matteo.dameno@st.com)
 * Version            : V.1.0.5
 * Date               : 16/08/2010
 * Description        : K3DH accelerometer sensor API
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
 *
 ******************************************************************************
 Revision 1.0.0 05/11/09
 First Release
 Revision 1.0.3 22/01/2010
  Linux K&R Compliant Release;
 Revision 1.0.5 16/08/2010
 modified _get_acceleration_data function
 modified _update_odr function
 manages 2 interrupts

 ******************************************************************************/
#include<linux/slab.h>
#include<linux/err.h>
#include<linux/errno.h>
#include<linux/delay.h>
#include<linux/fs.h>
#include<linux/i2c.h>

#include<linux/input.h>
#include<linux/input-polldev.h>
#include<linux/miscdevice.h>
#include<linux/uaccess.h>

#include<linux/workqueue.h>
#include<linux/irq.h>
#include<linux/gpio.h>
#include<linux/interrupt.h>
#include <linux/sensors_core.h>
#include<linux/i2c/k2dm.h>



#define	DEBUG	1

#define	INTERRUPT_MANAGEMENT 1

#define	G_MAX		16000	/** Maximum polled-device-reported g value */

#define K2DM_THRESHOLD      100000 /*459682*/

#define SENSITIVITY_2G		1	/**	mg/LSB	*/
#define SENSITIVITY_4G		2	/**	mg/LSB	*/
#define SENSITIVITY_8G		4	/**	mg/LSB	*/
#define SENSITIVITY_16G		12	/**	mg/LSB	*/


#define	HIGH_RESOLUTION		0x08

#define	AXISDATA_REG		0x28
#define WHOAMI_K2DM_ACC		0x33	/*	Expctd content for WAI	*/

/*	CONTROL REGISTERS	*/
#define WHO_AM_I		0x0F	/*	WhoAmI register		*/
#define	TEMP_CFG_REG		0x1F	/*	temper sens control reg	*/
/* ctrl 1: ODR3 ODR2 ODR ODR0 LPen Zenable Yenable Zenable */
#define	CTRL_REG1		0x20	/*	control reg 1		*/
#define	CTRL_REG2		0x21	/*	control reg 2		*/
#define	CTRL_REG3		0x22	/*	control reg 3		*/
#define	CTRL_REG4		0x23	/*	control reg 4		*/
#define	CTRL_REG5		0x24	/*	control reg 5		*/
#define	CTRL_REG6		0x25	/*	control reg 6		*/

#define	FIFO_CTRL_REG		0x2E	/*	FiFo control reg	*/

#define	INT_CFG1		0x30	/*	interrupt 1 config	*/
#define	INT_SRC1		0x31	/*	interrupt 1 source	*/
#define	INT_THS1		0x32	/*	interrupt 1 threshold	*/
#define	INT_DUR1		0x33	/*	interrupt 1 duration	*/

#define	INT_CFG2		0x34	/*	interrupt 2 config	*/
#define	INT_SRC2		0x35	/*	interrupt 2 source	*/
#define	INT_THS2		0x36	/*	interrupt 2 threshold	*/
#define	INT_DUR2		0x37	/*	interrupt 2 duration	*/

#define	TT_CFG			0x38	/*	tap config		*/
#define	TT_SRC			0x39	/*	tap source		*/
#define	TT_THS			0x3A	/*	tap threshold		*/
#define	TT_LIM			0x3B	/*	tap time limit		*/
#define	TT_TLAT			0x3C	/*	tap time latency	*/
#define	TT_TW			0x3D	/*	tap time window		*/
/*	end CONTROL REGISTRES	*/


#define ENABLE_HIGH_RESOLUTION	1

#define K2DM_ACC_PM_OFF		0x00
#define K2DM_ACC_ENABLE_ALL_AXES	0x07

#define PMODE_MASK			0x08
#define ODR_MASK			0XF0

#define ODR1		0x10  /* 1Hz output data rate */
#define ODR10		0x20  /* 10Hz output data rate */
#define ODR25		0x30  /* 25Hz output data rate */
#define ODR50		0x40  /* 50Hz output data rate */
#define ODR100		0x50  /* 100Hz output data rate */
#define ODR200		0x60  /* 200Hz output data rate */
#define ODR400		0x70  /* 400Hz output data rate */
#define ODR1250		0x90  /* 1250Hz output data rate */

#define OUT_X_L			0x28
#define OUT_X_H			0x29
#define OUT_Y_L			0x2A
#define OUT_Y_H			0x2B
#define OUT_Z_L			0x2C
#define OUT_Z_H			0x2D

#define	IA			0x40
#define	ZH			0x20
#define	ZL			0x10
#define	YH			0x08
#define	YL			0x04
#define	XH			0x02
#define	XL			0x01
/* */
/* CTRL REG BITS*/
#define	CTRL_REG3_I1_AOI1	0x40
#define	CTRL_REG6_I2_TAPEN	0x80
#define	CTRL_REG6_HLACTIVE	0x02
/* */

/* TAP_SOURCE_REG BIT */
#define	DTAP			0x20
#define	STAP			0x10
#define	SIGNTAP			0x08
#define	ZTAP			0x04
#define	YTAP			0x02
#define	XTAZ			0x01


#define	FUZZ			32
#define	FLAT			32
#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5
#define	I2C_AUTO_INCREMENT	0x80

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RES_CTRL_REG6		5

#define	RES_INT_CFG1		6
#define	RES_INT_THS1		7
#define	RES_INT_DUR1		8
#define	RES_INT_CFG2		9
#define	RES_INT_THS2		10
#define	RES_INT_DUR2		11

#define	RES_TT_CFG		12
#define	RES_TT_THS		13
#define	RES_TT_LIM		14
#define	RES_TT_TLAT		15
#define	RES_TT_TW		16

#define	RES_TEMP_CFG_REG	17
#define	RES_REFERENCE_REG	18
#define	RES_FIFO_CTRL_REG	19

#define	RESUME_ENTRIES		20
/* end RESUME STATE INDICES */
#define CALIBRATION_FILE_PATH	"/data/calibration_data"
#define CALIBRATION_DATA_AMOUNT	20

#define CALIBRATION_RUNNING 2
#define CALIBRATION_DATA_HAVE 1
#define CALIBRATION_NONE 0
/*
 * Acceleration measurement
 */
#define K2DM_RESOLUTION			64
#define HAL_RESOLUTION			4
#define DRIVER_RESOLUTION		(K2DM_RESOLUTION/HAL_RESOLUTION)
/* ABS axes parameter range [um/s^2] (for input event) */
#define GRAVITY_EARTH                   9806550
#define ABSMIN_2G                       (-GRAVITY_EARTH * 2)
#define ABSMAX_2G                       (GRAVITY_EARTH * 2)

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} k2dm_acc_odr_table[] = {
			{ 1, ODR1250 },
			{ 3, ODR400 },
			{ 5, ODR200 },
			{ 10, ODR100 },
			{ 20, ODR50 },
			{ 40, ODR25 },
			{ 100, ODR10 },
			{ 1000, ODR1 },
};

struct acceleration {
	int x;
	int y;
	int z;
};

struct k2dm_acc_data {
	struct i2c_client *client;
	struct k2dm_acc_platform_data *pdata;

	struct mutex lock;
	struct mutex data_mutex;
	struct delayed_work input_work;

	struct input_dev *input_dev;

	atomic_t enable;                /* attribute value */
	atomic_t delay;                 /* attribute value */
	atomic_t position;              /* attribute value */
	atomic_t threshold;             /* attribute value */

	u8 sensitivity;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	int on_before_suspend;

	u8 resume_state[RESUME_ENTRIES];
	int cal_data[3];
	int k2dm_state;

};

/*
 * Because misc devices can not carry a pointer from driver register to
 * open, we keep this global.  This limits the driver to a single instance.
 */
struct k2dm_acc_data *k2dm_acc_misc_data;
static struct device *k2dm_device;

static int k2dm_acc_i2c_read(struct k2dm_acc_data *acc, u8 * buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg	msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf, },
		{
			.addr = acc->client->addr,
			.flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf, },
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&acc->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int k2dm_acc_i2c_write(struct k2dm_acc_data *acc, u8 * buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = len + 1,
			.buf = buf,
		},
	};
	do {
		err = i2c_transfer(acc->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&acc->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int k2dm_acc_hw_init(struct k2dm_acc_data *acc)
{
	int err = -1;
	u8 buf[7];

	printk(KERN_INFO "%s: hw init start\n", K2DM_ACC_DEV_NAME);

	buf[0] = WHO_AM_I;
	err = k2dm_acc_i2c_read(acc, buf, 1);
	if (err < 0)
		goto error_firstread;
	else
		acc->hw_working = 1;
	if (buf[0] != WHOAMI_K2DM_ACC) {
		err = -1; /* choose the right coded error */
		goto error_unknown_device;
	}

	buf[0] = CTRL_REG1;
	buf[1] = acc->resume_state[RES_CTRL_REG1];
	err = k2dm_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto error1;

	buf[0] = TEMP_CFG_REG;
	buf[1] = acc->resume_state[RES_TEMP_CFG_REG];
	err = k2dm_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto error1;

	buf[0] = FIFO_CTRL_REG;
	buf[1] = acc->resume_state[RES_FIFO_CTRL_REG];
	err = k2dm_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto error1;

	buf[0] = (I2C_AUTO_INCREMENT | TT_THS);
	buf[1] = acc->resume_state[RES_TT_THS];
	buf[2] = acc->resume_state[RES_TT_LIM];
	buf[3] = acc->resume_state[RES_TT_TLAT];
	buf[4] = acc->resume_state[RES_TT_TW];
	err = k2dm_acc_i2c_write(acc, buf, 4);
	if (err < 0)
		goto error1;
	buf[0] = TT_CFG;
	buf[1] = acc->resume_state[RES_TT_CFG];
	err = k2dm_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto error1;

	buf[0] = (I2C_AUTO_INCREMENT | INT_THS1);
	buf[1] = acc->resume_state[RES_INT_THS1];
	buf[2] = acc->resume_state[RES_INT_DUR1];
	err = k2dm_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto error1;
	buf[0] = INT_CFG1;
	buf[1] = acc->resume_state[RES_INT_CFG1];
	err = k2dm_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto error1;

	buf[0] = (I2C_AUTO_INCREMENT | INT_THS2);
	buf[1] = acc->resume_state[RES_INT_THS2];
	buf[2] = acc->resume_state[RES_INT_DUR2];
	err = k2dm_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto error1;
	buf[0] = INT_CFG2;
	buf[1] = acc->resume_state[RES_INT_CFG2];
	err = k2dm_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto error1;

	buf[0] = (I2C_AUTO_INCREMENT | CTRL_REG2);
	buf[1] = acc->resume_state[RES_CTRL_REG2];
	buf[2] = acc->resume_state[RES_CTRL_REG3];
	buf[3] = acc->resume_state[RES_CTRL_REG4];
	buf[4] = acc->resume_state[RES_CTRL_REG5];
	buf[5] = acc->resume_state[RES_CTRL_REG6];
	err = k2dm_acc_i2c_write(acc, buf, 5);
	if (err < 0)
		goto error1;

	acc->hw_initialized = 1;
	printk(KERN_INFO "%s: hw init done\n", K2DM_ACC_DEV_NAME);
	return 0;

error_firstread:
	acc->hw_working = 0;
	dev_warn(&acc->client->dev, "Error reading WHO_AM_I: is device "
		"available/working?\n");
	goto error1;
error_unknown_device:
	dev_err(&acc->client->dev,
		"device unknown. Expected: 0x%x,"
		" Replies: 0x%x\n", WHOAMI_K2DM_ACC, buf[0]);
error1:
	acc->hw_initialized = 0;
	dev_err(&acc->client->dev, "hw init error 0x%x,0x%x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void k2dm_acc_device_power_off(struct k2dm_acc_data *acc)
{
	int err;
	u8 buf[2] = { CTRL_REG1, K2DM_ACC_PM_OFF };

	err = k2dm_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power off failed: %d\n", err);

	if (acc->pdata->vreg_en) {
		acc->pdata->vreg_en(0);
		acc->hw_initialized = 0;
	}
	if (acc->hw_initialized)
		acc->hw_initialized = 0;
}

static int k2dm_acc_device_power_on(struct k2dm_acc_data *acc)
{
	int err = -1;

	if (acc->pdata->vreg_en) {
		err = acc->pdata->vreg_en(1);
		if (err < 0) {
			dev_err(&acc->client->dev,
					"power_on failed: %d\n", err);
			return err;
		}
	}

	if (!acc->hw_initialized) {
		err = k2dm_acc_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			k2dm_acc_device_power_off(acc);
			return err;
		}
	}

	if (acc->hw_initialized) {
		printk(KERN_INFO "%s: power on: irq enabled\n",
						K2DM_ACC_DEV_NAME);
	}
	return 0;
}

int k2dm_acc_update_g_range(struct k2dm_acc_data *acc, u8 new_g_range)
{
	int err;

	u8 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = K2DM_ACC_FS_MASK | HIGH_RESOLUTION;

	switch (new_g_range) {
	case K2DM_ACC_G_2G:

		sensitivity = SENSITIVITY_2G;
		break;
	case K2DM_ACC_G_4G:

		sensitivity = SENSITIVITY_4G;
		break;
	case K2DM_ACC_G_8G:

		sensitivity = SENSITIVITY_8G;
		break;
	case K2DM_ACC_G_16G:

		sensitivity = SENSITIVITY_16G;
		break;
	default:
		dev_err(&acc->client->dev, "invalid g range requested: %u\n",
				new_g_range);
		return -EINVAL;
	}

	if (atomic_read(&acc->enable)) {
		/* Set configuration register 4, which contains g range setting
		 *  NOTE: this is a straight overwrite because this driver does
		 *  not use any of the other configuration bits in this
		 *  register.  Should this become untrue, we will have to read
		 *  out the value and only change the relevant bits --XX----
		 *  (marked by X) */
		buf[0] = CTRL_REG4;
		err = k2dm_acc_i2c_read(acc, buf, 1);
		if (err < 0)
			goto error;
		init_val = buf[0];
		acc->resume_state[RES_CTRL_REG4] = init_val;
		new_val = new_g_range | HIGH_RESOLUTION;
		updated_val = ((mask & new_val) | ((~mask) & init_val));
		buf[1] = updated_val;
		buf[0] = CTRL_REG4;
		err = k2dm_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL_REG4] = updated_val;
		acc->sensitivity = sensitivity;
	}


	return 0;
error:
	dev_err(&acc->client->dev, "update g range failed 0x%x,0x%x: %d\n",
			buf[0], buf[1], err);

	return err;
}

int k2dm_acc_update_odr(struct k2dm_acc_data *acc, int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2];

	/* Convert the poll interval into an output data rate configuration
	 *  that is as low as possible.  The ordering of these checks must be
	 *  maintained due to the cascading cut off values - poll intervals are
	 *  checked from shortest to longest.  At each check, if the next lower
	 *  ODR cannot support the current poll interval, we stop searching */
	for (i = ARRAY_SIZE(k2dm_acc_odr_table) - 1; i >= 0; i--) {
		if (k2dm_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
			break;
	}
	config[1] = k2dm_acc_odr_table[i].mask;

	config[1] |= K2DM_ACC_ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&acc->enable)) {
		config[0] = CTRL_REG1;
		err = k2dm_acc_i2c_write(acc, config, 1);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL_REG1] = config[1];
	}

	return 0;

error:
	dev_err(&acc->client->dev, "update odr failed 0x%x,0x%x: %d\n",
			config[0], config[1], err);

	return err;
}

/* */
/*
static int k2dm_acc_register_write(struct k2dm_acc_data *acc, u8 *buf,
		u8 reg_address, u8 new_value)
{
	int err = -1;

	if (atomic_read(&acc->enable)) {
		 Sets configuration register at reg_address
		  NOTE: this is a straight overwrite
		buf[0] = reg_address;
		buf[1] = new_value;
		err = k2dm_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			return err;
	}
	return err;
}

static int k2dm_acc_register_read(struct k2dm_acc_data *acc, u8 *buf,
		u8 reg_address)
{

	int err = -1;
	buf[0] = (reg_address);
	err = k2dm_acc_i2c_read(acc, buf, 1);
	return err;
}

static int k2dm_acc_register_update(struct k2dm_acc_data *acc, u8 *buf,
		u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;
	err = k2dm_acc_register_read(acc, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[1];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = k2dm_acc_register_write(acc, buf, reg_address,
				updated_val);
	}
	return err;
}*/

/* */

static int k2dm_acc_get_acceleration_data(struct k2dm_acc_data *acc,
		int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };

	acc_data[0] = (I2C_AUTO_INCREMENT | AXISDATA_REG);
	err = k2dm_acc_i2c_read(acc, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (-1) * (s8)acc_data[1];
	hw_d[1] = (-1) * (s8)acc_data[3];
	hw_d[2] = (s8)acc_data[5];

	if (k2dm_acc_misc_data->k2dm_state == CALIBRATION_DATA_HAVE) {
		xyz[0] = (hw_d[0] * DRIVER_RESOLUTION)
			- acc->cal_data[0];
		xyz[1] = (hw_d[1] * DRIVER_RESOLUTION)
			- acc->cal_data[1];
		xyz[2] = (hw_d[2] * DRIVER_RESOLUTION)
			- acc->cal_data[2];
	} else {
		xyz[0] = hw_d[0] * DRIVER_RESOLUTION;
		xyz[1] = hw_d[1] * DRIVER_RESOLUTION;
		xyz[2] = hw_d[2] * DRIVER_RESOLUTION;
	}

	return err;
}

static void k2dm_acc_report_values(struct k2dm_acc_data *acc, int *xyz)
{
	input_report_abs(acc->input_dev, ABS_X, xyz[0]);
	input_report_abs(acc->input_dev, ABS_Y, xyz[1]);
	input_report_abs(acc->input_dev, ABS_Z, xyz[2]);
	input_sync(acc->input_dev);
}

static int k2dm_acc_enable(struct k2dm_acc_data *acc)
{
	int err;
	int delay = atomic_read(&acc->delay);

	if (!atomic_cmpxchg(&acc->enable, 0, 1)) {
		err = k2dm_acc_device_power_on(acc);
		if (err < 0) {
			atomic_set(&acc->enable, 0);
			return err;
		}
		schedule_delayed_work(&acc->input_work, msecs_to_jiffies(
				delay));
	}

	return 0;
}

static int k2dm_acc_disable(struct k2dm_acc_data *acc)
{
	if (atomic_cmpxchg(&acc->enable, 1, 0)) {
		cancel_delayed_work_sync(&acc->input_work);
		k2dm_acc_device_power_off(acc);
	}

	return 0;
}

static int k2dm_get_position(struct device *dev)
{
	struct k2dm_acc_data *k2dm = dev_get_drvdata(dev);
	return atomic_read(&k2dm->position);
}

static void k2dm_set_position(struct device *dev, unsigned long position)
{
	struct k2dm_acc_data *k2dm = dev_get_drvdata(dev);
	atomic_set(&k2dm->position, position);
}

static int k2dm_get_threshold(struct device *dev)
{
	struct k2dm_acc_data *k2dm = dev_get_drvdata(dev);
	return atomic_read(&k2dm->threshold);
}

static void k2dm_set_threshold(struct device *dev, int threshold)
{
	struct k2dm_acc_data *k2dm = dev_get_drvdata(dev);
	atomic_set(&k2dm->threshold, threshold);
}

/*
 * sysfs device attributes
 */
static ssize_t k2dm_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct k2dm_acc_data *k2dm = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&k2dm->enable));
}

static ssize_t k2dm_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	struct k2dm_acc_data *k2dm = dev_get_drvdata(dev);
	int64_t enable;
	err =  strict_strtoll(buf, 10, &enable);
	if (err < 0)
		return err;

	if (enable >= 1)
		err = k2dm_acc_enable(k2dm);

	else
		err = k2dm_acc_disable(k2dm);

	return count;
}

static ssize_t k2dm_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct k2dm_acc_data *k2dm = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&k2dm->delay));
}

static ssize_t k2dm_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	int64_t NewDelay;
	struct k2dm_acc_data *k2dm = dev_get_drvdata(dev);

	err = strict_strtoll(buf, 10, &NewDelay);
	if (err < 0)
		return err;

	if (NewDelay > K2DM_MAX_DELAY)
		NewDelay = K2DM_MAX_DELAY;

	atomic_set(&k2dm->delay, NewDelay);
	err = k2dm_acc_update_odr(k2dm, atomic_read(&k2dm->delay));
	/* TODO: if update fails poll is still set */
	if (err < 0)
		return err;

	return err;
}

static ssize_t k2dm_position_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", k2dm_get_position(dev));
}

static ssize_t k2dm_position_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	int64_t position;

	err = strict_strtoll(buf, 10, &position);
	if (err < 0)
		return err;

	if ((position >= 0) && (position <= 7))
		k2dm_set_position(dev, position);

	return count;
}

static ssize_t k2dm_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", k2dm_get_threshold(dev));
}

static ssize_t k2dm_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err;
	int64_t threshold;

	err = strict_strtoll(buf, 10, &threshold);
	if (err < 0)
		return err;

	if (threshold >= 0 && threshold <= ABSMAX_2G)
		k2dm_set_threshold(dev, threshold);

	return count;
}

static ssize_t k2dm_wake_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	static atomic_t serial = ATOMIC_INIT(0);

	input_report_abs(input, ABS_MISC, atomic_inc_return(&serial));

	return count;
}

static ssize_t k2dm_raw_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct k2dm_acc_data *k2dm = dev_get_drvdata(dev);
	int xyz[3] = { 0 };
	int err;

	mutex_lock(&k2dm->data_mutex);
	err = k2dm_acc_get_acceleration_data(k2dm, xyz);
	mutex_unlock(&k2dm->data_mutex);

	return snprintf(buf, PAGE_SIZE, "%d, %d, %d\n",
		xyz[0], xyz[1], xyz[2]);
}

static int k2dm_calibration(int is_cal_erase)
{
	int xyz[3], i, err = 0;
	int sum[3] = {0};
	mm_segment_t old_fs;
	struct file *cal_filp = NULL;

	k2dm_acc_misc_data->k2dm_state = CALIBRATION_RUNNING;

	if (is_cal_erase) {
		xyz[0] = 0;
		xyz[1] = 0;
		xyz[2] = 0;
		k2dm_acc_misc_data->cal_data[0] = 0;
		k2dm_acc_misc_data->cal_data[1] = 0;
		k2dm_acc_misc_data->cal_data[2] = 0;
	} else {
		for (i = 0 ; i < CALIBRATION_DATA_AMOUNT ; i++) {
			k2dm_acc_get_acceleration_data(k2dm_acc_misc_data, xyz);
			sum[0] += xyz[0];
			sum[1] += xyz[1];
			sum[2] += xyz[2];
		}
		k2dm_acc_misc_data->cal_data[0] =
			(sum[0] / CALIBRATION_DATA_AMOUNT);
		k2dm_acc_misc_data->cal_data[1] =
			(sum[1] / CALIBRATION_DATA_AMOUNT);
		if (sum[2] > 0)
			k2dm_acc_misc_data->cal_data[2]
			= (sum[2] / CALIBRATION_DATA_AMOUNT -
					(K2DM_RESOLUTION * DRIVER_RESOLUTION));
		else
			k2dm_acc_misc_data->cal_data[2]
			= (sum[2] / CALIBRATION_DATA_AMOUNT -
					(K2DM_RESOLUTION * DRIVER_RESOLUTION));
		msleep(20);
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH,
		O_CREAT | O_TRUNC | O_WRONLY, 0666);
	if (IS_ERR(cal_filp)) {
		pr_err("[k2dm] %s: Can't open calibration file\n",
			__func__);
		set_fs(old_fs);
		err = PTR_ERR(cal_filp);
		return err;
	}

	err = cal_filp->f_op->write(cal_filp,
		(char *)&k2dm_acc_misc_data->cal_data,
		3 * sizeof(int), &cal_filp->f_pos);
	if (err != 3 * sizeof(int)) {
		pr_err("[k2dm] %s: Can't write the cal data to file\n",
			__func__);
		err = -EIO;
	}

	if (err < 0)
		k2dm_acc_misc_data->k2dm_state = CALIBRATION_NONE;
	else
		k2dm_acc_misc_data->k2dm_state = CALIBRATION_DATA_HAVE;
	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return 0;
}

static int k2dm_open()
{
	int err = 0, i;
	mm_segment_t old_fs;
	struct file *cal_filp = NULL;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH,
		O_RDONLY, 0666);
	if (IS_ERR(cal_filp)) {
		pr_err("[k2dm] %s: Can't open calibration file\n",
			__func__);
		set_fs(old_fs);
		err = PTR_ERR(cal_filp);
		return err;
	}
	err = cal_filp->f_op->read(cal_filp,
		(char *)&k2dm_acc_misc_data->cal_data,
		3 * sizeof(int), &cal_filp->f_pos);

	if (err != 3 * sizeof(int)) {
		pr_err("[k2dm] %s: Can't read the cal data from file\n",
			__func__);
		filp_close(cal_filp, current->files);
		set_fs(old_fs);
		return -EIO;
	}
/*#ifdef ALPS_DEBUG*/
	pr_info("[k2dm] %s: (%d,%d,%d)\n", __func__,
		k2dm_acc_misc_data->cal_data[0],
		k2dm_acc_misc_data->cal_data[1],
		k2dm_acc_misc_data->cal_data[2]);
/*#endif*/
	for (i = 0 ; i < 3 ; i++) {
		if (k2dm_acc_misc_data->cal_data[i] < -128 ||
			128 < k2dm_acc_misc_data->cal_data[i]) {

			filp_close(cal_filp, current->files);
			k2dm_acc_misc_data->k2dm_state = CALIBRATION_NONE;
			return err = -EIO;
		}
	}
	if (k2dm_acc_misc_data->cal_data[0] == 0
		&& k2dm_acc_misc_data->cal_data[1] == 0
		&& k2dm_acc_misc_data->cal_data[2] == 0)
		k2dm_acc_misc_data->k2dm_state = CALIBRATION_NONE;
	else
		k2dm_acc_misc_data->k2dm_state = CALIBRATION_DATA_HAVE;
	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return 0;
}

static ssize_t k2dm_calibration_store(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err;
/*	struct k2dm_acc_data *k2dm = dev_get_drvdata(dev);*/
	int64_t enable;
	err =  strict_strtoll(buf, 10, &enable);
	if (err < 0)
		return err;

	if (enable >= 1) {
		err = k2dm_calibration(0);
		if (err < 0)
			pr_info("k2dm_calibration fail.\n");
		else
			pr_info("k2dm_calibration success.\n");

	} else {
		err = k2dm_calibration(1);
		if (err < 0)
			pr_info("k2dm_erase fail.\n");
		else
			pr_info("k2dm_erase success.\n");
	}
	if (err > 0)
		err = 0;
	return snprintf(buf, PAGE_SIZE, "%d\n", err);
}

static ssize_t k2dm_calibration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE,
		"%d,%d,%d\n",
		k2dm_acc_misc_data->cal_data[0],
		k2dm_acc_misc_data->cal_data[1],
		k2dm_acc_misc_data->cal_data[2]);
}
static ssize_t k2dm_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE,
		"%s\n", K2DM_ACC_DEV_VENDOR);
}
static ssize_t k2dm_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE,
		"%s\n", K2DM_ACC_DEV_NAME);
}

static ssize_t k2dm_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct k2dm_acc_data *k2dm = dev_get_drvdata(dev);
	int xyz[3] = { 0 };
	int err;

	mutex_lock(&k2dm->data_mutex);
	err = k2dm_acc_get_acceleration_data(k2dm, xyz);
	mutex_unlock(&k2dm->data_mutex);

	return snprintf(buf, PAGE_SIZE, "%d %d %d\n", xyz[0], xyz[1], xyz[2]);
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
		k2dm_enable_show, k2dm_enable_store);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
		k2dm_delay_show, k2dm_delay_store);
static DEVICE_ATTR(position, S_IRUGO|S_IWUSR,
		k2dm_position_show, k2dm_position_store);
static DEVICE_ATTR(threshold, S_IRUGO|S_IWUSR,
		k2dm_threshold_show, k2dm_threshold_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP,
		NULL, k2dm_wake_store);
static DEVICE_ATTR(raw_data, S_IRUGO,
		k2dm_raw_data_show, NULL);
static DEVICE_ATTR(calibration, S_IRUGO|S_IWUSR|S_IWGRP,
		k2dm_calibration_show, k2dm_calibration_store);
static DEVICE_ATTR(vendor, 0440,
		k2dm_vendor_show, NULL);
static DEVICE_ATTR(name, 0440,
		k2dm_name_show, NULL);
static DEVICE_ATTR(data, S_IRUGO,
		k2dm_data_show, NULL);
static struct device_attribute *k2dm_attrs[] = {
	&dev_attr_raw_data,
	&dev_attr_calibration,
	&dev_attr_vendor,
	&dev_attr_name,
	NULL,
};
static struct attribute *k2dm_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_position.attr,
	&dev_attr_threshold.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
	NULL
};

static struct attribute_group k2dm_attribute_group = {
	.attrs = k2dm_attributes
};

static void k2dm_acc_input_work_func(struct work_struct *work)
{
	struct k2dm_acc_data *acc;

	int xyz[3] = { 0 };
	int err;
	int delay;
	acc = container_of((struct delayed_work *)work,
			struct k2dm_acc_data, input_work);
	delay = atomic_read(&acc->delay);

	mutex_lock(&acc->lock);
	err = k2dm_acc_get_acceleration_data(acc, xyz);
	if (err < 0)
		dev_err(&acc->client->dev, "get_acceleration_data failed\n");
	else
		k2dm_acc_report_values(acc, xyz);

	schedule_delayed_work(&acc->input_work, msecs_to_jiffies(
			delay));
	mutex_unlock(&acc->lock);
}

#ifdef K3DH_OPEN_ENABLE
int k2dm_acc_input_open(struct input_dev *input)
{
	struct k2dm_acc_data *acc = input_get_drvdata(input);

	return k2dm_acc_enable(acc);
}

void k2dm_acc_input_close(struct input_dev *dev)
{
	struct k2dm_acc_data *acc = input_get_drvdata(dev);

	k2dm_acc_disable(acc);
}
#endif

static int k2dm_acc_input_init(struct k2dm_acc_data *acc)
{
	int err;

	INIT_DELAYED_WORK(&acc->input_work, k2dm_acc_input_work_func);
	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(&acc->client->dev, "input device allocate failed\n");
		goto err0;
	}

#ifdef K3DH_ACC_OPEN_ENABLE
	acc->input_dev->open = k2dm_acc_input_open;
	acc->input_dev->close = k2dm_acc_input_close;
#endif

	input_set_drvdata(acc->input_dev, acc);

	set_bit(EV_ABS, acc->input_dev->evbit);

	input_set_abs_params(acc->input_dev, ABS_X,
			-ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(acc->input_dev, ABS_Y,
			-ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(acc->input_dev, ABS_Z,
			-ABSMIN_2G, ABSMAX_2G, 0, 0);

	acc->input_dev->name = "accelerometer";

	err = input_register_device(acc->input_dev);
	if (err) {
		dev_err(&acc->client->dev,
				"unable to register input polled device %s\n",
				acc->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->input_dev);
err0:
	return err;
}

static void k2dm_acc_input_cleanup(struct k2dm_acc_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

static int k2dm_acc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct k2dm_acc_data *acc;

	int err = -1;
	int tempvalue;
	int delay;

	pr_info("[k2dm] k2dm_acc_probe\n");

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE |
					I2C_FUNC_SMBUS_BYTE_DATA |
					I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev, "client not smb-i2c capable:2\n");
		err = -EIO;
		goto exit_check_functionality_failed;
	}

	if (!i2c_check_functionality(client->adapter,
						I2C_FUNC_SMBUS_I2C_BLOCK)){
		dev_err(&client->dev, "client not smb-i2c capable:3\n");
		err = -EIO;
		goto exit_check_functionality_failed;
	}
	/*
	 * OK. From now, we presume we have a valid client. We now create the
	 * client structure, even though we cannot fill it completely yet.
	 */

	acc = kzalloc(sizeof(struct k2dm_acc_data), GFP_KERNEL);
	if (acc == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: "
					"%d\n", err);
		goto exit_alloc_data_failed;
	}

	mutex_init(&acc->lock);
	mutex_init(&acc->data_mutex);
	mutex_lock(&acc->lock);

	acc->client = client;
/*	i2c_set_clientdata(client, acc);*/
	acc->pdata = client->dev.platform_data;
	i2c_set_clientdata(client, acc);

	err = k2dm_acc_device_power_on(acc);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err2;
	}

	if (i2c_smbus_read_byte(client) < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte error!!\n");
		goto err_destoyworkqueue2;
	} else {
		printk(KERN_INFO "%s Device detected!\n", K2DM_ACC_DEV_NAME);
	}

	/* read chip id */
	tempvalue = i2c_smbus_read_word_data(client, WHO_AM_I);
	printk(KERN_INFO "chip id is %d\n", tempvalue);
	if ((tempvalue & 0x00FF) == WHOAMI_K2DM_ACC) {
		printk(KERN_INFO "%s I2C driver registered!\n",
							K2DM_ACC_DEV_NAME);
	} else {
		acc->client = NULL;
		printk(KERN_INFO "I2C driver not registered!"
				" Device unknown\n");
		goto err_destoyworkqueue2;
	}
/*
	acc->pdata = client->dev.platform_data;

	i2c_set_clientdata(client, acc);

	err = k2dm_acc_device_power_on(acc);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err2;
	}
*/
	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));

	acc->resume_state[RES_CTRL_REG1] = K2DM_ACC_ENABLE_ALL_AXES;
	acc->resume_state[RES_CTRL_REG2] = 0x00;
	acc->resume_state[RES_CTRL_REG3] = 0x00;
	acc->resume_state[RES_CTRL_REG4] = 0x00;
	acc->resume_state[RES_CTRL_REG5] = 0x00;
	acc->resume_state[RES_CTRL_REG6] = 0x00;

	acc->resume_state[RES_TEMP_CFG_REG] = 0x00;
	acc->resume_state[RES_FIFO_CTRL_REG] = 0x00;
	acc->resume_state[RES_INT_CFG1] = 0x00;
	acc->resume_state[RES_INT_THS1] = 0x00;
	acc->resume_state[RES_INT_DUR1] = 0x00;
	acc->resume_state[RES_INT_CFG2] = 0x00;
	acc->resume_state[RES_INT_THS2] = 0x00;
	acc->resume_state[RES_INT_DUR2] = 0x00;

	acc->resume_state[RES_TT_CFG] = 0x00;
	acc->resume_state[RES_TT_THS] = 0x00;
	acc->resume_state[RES_TT_LIM] = 0x00;
	acc->resume_state[RES_TT_TLAT] = 0x00;
	acc->resume_state[RES_TT_TW] = 0x00;

	atomic_set(&acc->enable, 1);
	k2dm_set_position(&client->dev, CONFIG_SENSORS_K2DM_POSITION);
	k2dm_set_threshold(&client->dev, K2DM_THRESHOLD);
	delay = atomic_set(&acc->delay, K2DM_DEFAULT_DELAY);
	err = k2dm_acc_update_odr(acc, delay);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	err = k2dm_acc_input_init(acc);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err_power_off;
	}
	k2dm_acc_misc_data = acc;

	err = sysfs_create_group(&acc->input_dev->dev.kobj,
			&k2dm_attribute_group);
	if (err < 0)
		goto err_input_cleanup;

	k2dm_acc_device_power_off(acc);

	/* As default, do not report information */
	atomic_set(&acc->enable, 0);

	mutex_unlock(&acc->lock);

	k2dm_open();
	sensors_register(k2dm_device,
		k2dm_acc_misc_data, k2dm_attrs,
		"accelerometer_sensor");
	dev_info(&client->dev, "%s: probed\n", K2DM_ACC_DEV_NAME);

	return 0;

err_input_cleanup:
	k2dm_acc_input_cleanup(acc);
err_power_off:
	k2dm_acc_device_power_off(acc);
err2:
	if (acc->pdata->vreg_en)
		acc->pdata->vreg_en(0);
/*exit_kfree_pdata:*/
err_destoyworkqueue2:
	kfree(acc->pdata);
/*err_destoyworkqueue1:*/
/*err_mutexunlockfreedata:*/
	mutex_unlock(&acc->lock);
	kfree(acc);
exit_alloc_data_failed:
exit_check_functionality_failed:
	printk(KERN_ERR "%s: Driver Init failed\n", K2DM_ACC_DEV_NAME);
	return err;
}

static int __devexit k2dm_acc_remove(struct i2c_client *client)
{
	/* TODO: revisit ordering here once _probe order is finalized */
	struct k2dm_acc_data *acc = i2c_get_clientdata(client);

	k2dm_acc_input_cleanup(acc);
	k2dm_acc_device_power_off(acc);
	if (acc->pdata->vreg_en)
		acc->pdata->vreg_en(0);
	kfree(acc->pdata);
	kfree(acc);

	return 0;
}

static int k2dm_acc_resume(struct i2c_client *client)
{
	struct k2dm_acc_data *acc = i2c_get_clientdata(client);

	if (acc->on_before_suspend)
		return k2dm_acc_enable(acc);
	return 0;
}

static int k2dm_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct k2dm_acc_data *acc = i2c_get_clientdata(client);

	acc->on_before_suspend = atomic_read(&acc->enable);
	return k2dm_acc_disable(acc);
}

static const struct i2c_device_id k2dm_acc_id[]
				= { { K2DM_ACC_DEV_NAME, 0 }, { }, };

MODULE_DEVICE_TABLE(i2c, k2dm_acc_id);

static struct i2c_driver k2dm_acc_driver = {
	.driver = {
			.name = K2DM_ACC_DEV_NAME,
		  },
	.probe = k2dm_acc_probe,
	.remove = __devexit_p(k2dm_acc_remove),
	.resume = k2dm_acc_resume,
	.suspend = k2dm_acc_suspend,
	.id_table = k2dm_acc_id,
};

static int __init k2dm_acc_init(void)
{
	printk(KERN_INFO "%s accelerometer driver: init\n",
						K2DM_ACC_DEV_NAME);
	return i2c_add_driver(&k2dm_acc_driver);
}

static void __exit k2dm_acc_exit(void)
{
	#if DEBUG
	printk(KERN_INFO "%s accelerometer driver exit\n", K2DM_ACC_DEV_NAME);
	#endif
	i2c_del_driver(&k2dm_acc_driver);
	return;
}

module_init(k2dm_acc_init);
module_exit(k2dm_acc_exit);

MODULE_DESCRIPTION("k2dm accelerometer misc driver");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL");
