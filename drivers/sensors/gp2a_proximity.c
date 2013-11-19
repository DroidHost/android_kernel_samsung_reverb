/*
 * Copyright (c) 2010 SAMSUNG
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */

#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <mach/hardware.h>
#include <linux/wakelock.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/mfd/pmic8058.h>
#include <linux/i2c/gp2a.h>
#include <linux/sensors_core.h>

#if defined(CONFIG_MACH_VITAL2) || defined(CONFIG_MACH_ROOKIE2) || \
	defined(CONFIG_MACH_VITAL2REFRESH)
#define IRQ_GP2A_INT	PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, (PM8058_GPIO(30)))
#define GPIO_PS_VOUT	PM8058_GPIO_PM_TO_SYS(PM8058_GPIO(30))
#endif

#define SENSOR_DEFAULT_DELAY            (200)   /* 200 ms */
#define SENSOR_MAX_DELAY                (2000)  /* 2000 ms */
#define ABS_STATUS                      (ABS_BRAKE)
#define ABS_WAKE                        (ABS_MISC)
#define ABS_CONTROL_REPORT              (ABS_THROTTLE)

/* global var */
static struct wake_lock prx_wake_lock;

static struct i2c_driver opt_i2c_driver;
static struct i2c_client *opt_i2c_client;
static struct gp2a_platform_data *gp2a_pdata;
static struct device *prox_sys_device;


#ifdef USE_MODE_B
static char mvo_value;
static char proximity_value;
#endif

/* driver data */
struct gp2a_data {
	struct input_dev *input_dev;
	struct delayed_work work;  /* for proximity sensor */
	struct mutex enable_mutex;
	struct mutex data_mutex;

	int   enabled;
	int   delay;
	int   prox_data;
	int   irq;


	struct kobject *uevent_kobj;
};


static struct gp2a_data *prox_data;

struct opt_state {
	struct i2c_client	*client;
};

struct opt_state *opt_state;

static int proximity_onoff(u8 onoff);

static ssize_t
proximity_enable_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct gp2a_data *data = input_get_drvdata(input_data);
	int enabled;

	enabled = data->enabled;

	return sprintf(buf, "%d\n", enabled);
}

static ssize_t
proximity_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct gp2a_data *data = input_get_drvdata(input_data);
	long value = 0;
	if (strict_strtol(buf, 10, &value) < 0)
		return -EINVAL;

	if (value != 0 && value != 1)
		return count;

	mutex_lock(&data->enable_mutex);

	if (data->enabled && !value) {

		disable_irq(IRQ_GP2A_INT);
		cancel_delayed_work_sync(&data->work);
		proximity_onoff(0);
	}

	if (!data->enabled && value) {

		proximity_onoff(1);
#ifdef USE_MODE_B
		enable_irq(IRQ_GP2A_INT);
#else
		schedule_delayed_work(&data->work, 0);
		enable_irq(IRQ_GP2A_INT);
#endif
		mvo_value = 0;
		input_report_abs(data->input_dev, ABS_DISTANCE, value);
	    input_sync(data->input_dev);
	}
	data->enabled = value;

	mutex_unlock(&data->enable_mutex);

	return count;
}

static ssize_t
proximity_data_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct gp2a_data *data = input_get_drvdata(input_data);
	int x;

	mutex_lock(&data->data_mutex);
#ifdef USE_MODE_B
	x = proximity_value;
#else
	x = data->prox_data;
#endif
	mutex_unlock(&data->data_mutex);

	return sprintf(buf, "%d\n", x);
}
static ssize_t
prox_name_show(struct device *dev, struct device_attribute *attr,
		   char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_DEV_NAME);
}
static ssize_t
prox_vendor_show(struct device *dev, struct device_attribute *attr,
		   char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_DEV_VENDOR);
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
		proximity_enable_show, proximity_enable_store);
static DEVICE_ATTR(raw_data, S_IRUGO, proximity_data_show, NULL);
static DEVICE_ATTR(state, S_IRUGO, proximity_data_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO,
	prox_vendor_show, NULL);
static DEVICE_ATTR(name, S_IRUGO,
	prox_name_show, NULL);

static struct attribute *proximity_attributes[] = {
	&dev_attr_enable.attr,
	NULL
};
static struct device_attribute *prox_attrs[] = {
	&dev_attr_raw_data,
	&dev_attr_state,
	&dev_attr_vendor,
	&dev_attr_name,
	NULL,
};
static struct attribute_group proximity_attribute_group = {
	.attrs = proximity_attributes
};


static char get_ps_vout_value(void)
{
	char value = 0;
	unsigned char int_val;

#if defined(CONFIG_MACH_CHIEF) || defined(CONFIG_MACH_VITAL2) || \
	defined(CONFIG_MACH_ROOKIE2) || defined(CONFIG_MACH_VITAL2REFRESH)
#ifdef USE_MODE_B
	int_val = REGS_PROX;
	opt_i2c_read((u8)(int_val), &value, 1);
#else
	value = gpio_get_value_cansleep(GPIO_PS_VOUT);
#endif
#else
	value = gpio_get_value(GPIO_PS_VOUT);
#endif

	return value;
}

static void gp2a_work_func_prox(struct work_struct *work)
{
	struct gp2a_data *gp2a = container_of((struct delayed_work *)work,
							struct gp2a_data, work);
	char value;

#ifdef USE_MODE_B

	value = 0x10;
	opt_i2c_write((u8)(REGS_CON), &value);

	disable_irq(IRQ_GP2A_INT);

	if (!mvo_value) {
		mvo_value = 1;
		value = 0x20;
		opt_i2c_write((u8)(REGS_HYS), &value);
	} else {
		mvo_value = 0;
		value = 0x40;
		opt_i2c_write((u8)(REGS_HYS), &value);
	}

	value = 0x18;
	opt_i2c_write((u8)(REGS_CON), &value);

	value = 0x01 & get_ps_vout_value();

	if (mvo_value == value) {

		if (value)
			value = 0;
		else
			value = 1;

		printk(KERN_INFO "%s mvo_value=%d, value=%d\n",
			__func__, mvo_value, value);

		input_report_abs(gp2a->input_dev, ABS_DISTANCE, value);
	    input_sync(gp2a->input_dev);

		if (value == 1)
			kobject_uevent(gp2a->uevent_kobj, KOBJ_OFFLINE);
		else if (value == 0)
			kobject_uevent(gp2a->uevent_kobj, KOBJ_ONLINE);

		gp2a->prox_data = value;
		proximity_value = value;

		enable_irq(IRQ_GP2A_INT);

		value = 0x00;
		opt_i2c_write((u8)(REGS_CON), &value);
	} else {
		printk(KERN_INFO "mvo_value = %d , value = %d\n",
			mvo_value, value);
		printk(KERN_INFO "mvo_value != value , need to reset\n");
		value = 0x02;
		opt_i2c_write((u8)(REGS_OPMOD), &value);
		mvo_value = 0;
		value = 0x03;
		opt_i2c_write((u8)(REGS_OPMOD), &value);
		proximity_onoff(1);
		enable_irq(IRQ_GP2A_INT);
		schedule_delayed_work(&gp2a->work,
			msecs_to_jiffies(gp2a->delay));
	}
#else
	value = get_ps_vout_value();
	input_report_abs(gp2a->input_dev, ABS_DISTANCE,  value);
	input_sync(gp2a->input_dev);

	if (value == 1)
		kobject_uevent(gp2a->uevent_kobj, KOBJ_OFFLINE);
	else if (value == 0)
		kobject_uevent(gp2a->uevent_kobj, KOBJ_ONLINE);

	gp2a->prox_data = value;

	schedule_delayed_work(&gp2a->work, msecs_to_jiffies(gp2a->delay));
#endif
}

irqreturn_t gp2a_irq_handler(int irq, void *dev_id)
{
	char value;

#ifdef USE_MODE_B
	cancel_delayed_work_sync(&prox_data->work);
	wake_lock_timeout(&prx_wake_lock, 3*HZ);
	schedule_delayed_work(&prox_data->work,
		msecs_to_jiffies(prox_data->delay));
	printk(KERN_INFO "[PROXIMITY] IRQ_HANDLED %d!\n", prox_data->delay);
#else
	value = get_ps_vout_value();

	cancel_delayed_work_sync(&prox_data->work);
	wake_lock_timeout(&prx_wake_lock, 3*HZ);

	input_report_abs(prox_data->input_dev, ABS_DISTANCE,  value);
	input_sync(prox_data->input_dev);

	if (value == 1)
		kobject_uevent(prox_data->uevent_kobj, KOBJ_OFFLINE);
	else if (value == 0)
		kobject_uevent(prox_data->uevent_kobj, KOBJ_ONLINE);

	prox_data->prox_data = value;

	schedule_delayed_work(&prox_data->work,
		msecs_to_jiffies(prox_data->delay));
	printk(KERN_INFO "[PROXIMITY] IRQ_HANDLED ! (value : %d)\n", value);
#endif

	return IRQ_HANDLED;
}

static int opt_i2c_init(void)
{
	if (i2c_add_driver(&opt_i2c_driver)) {
		printk(KERN_ERR "i2c_add_driver failed\n");
		return -ENODEV;
	}
	return 0;
}


int opt_i2c_read(u8 reg, u8 *val, unsigned int len)
{
	int err;
	u8 buf[2];
	struct i2c_msg msg[1];

	buf[0] = reg;

	msg[0].addr = opt_i2c_client->addr;
	msg[0].flags = 1;

	msg[0].len = 2;
	msg[0].buf = buf;
	err = i2c_transfer(opt_i2c_client->adapter, msg, 1);

	*val = buf[1];

	if (err >= 0)
		return 0;

	printk(KERN_INFO "%s %d i2c transfer error\n", __func__, __LINE__);
	return err;
}

int opt_i2c_write(u8 reg, u8 *val)
{
	int err = 0;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retry = 10;

	if ((opt_i2c_client == NULL) || (!opt_i2c_client->adapter))
		return -ENODEV;

	while (retry--) {
		data[0] = reg;
		data[1] = *val;

		msg->addr = opt_i2c_client->addr;
		msg->flags = I2C_M_WR;
		msg->len = 2;
		msg->buf = data;

		err = i2c_transfer(opt_i2c_client->adapter, msg, 1);

		if (err >= 0)
			return 0;
	}
	printk(KERN_ERR "%s i2c transfer (%d) error! reg:%d, val:%d\n",
		__func__, err, reg, *val);
	return err;
}

static int proximity_input_init(struct gp2a_data *data)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	set_bit(EV_ABS, dev->evbit);
	input_set_capability(dev, EV_ABS, ABS_DISTANCE);
	input_set_abs_params(dev, ABS_DISTANCE, 0, 1, 0, 0);

	dev->name = "proximity_sensor";
	input_set_drvdata(dev, data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	data->input_dev = dev;

	return 0;
}

static int gp2a_opt_probe(struct platform_device *pdev)
{
	struct gp2a_data *gp2a;
	u8 value;
	int err = 0;
	printk(KERN_INFO "gp2a_opt_probe proximity is start\n");
	/* allocate driver_data */
	gp2a = kzalloc(sizeof(struct gp2a_data), GFP_KERNEL);
	if (!gp2a) {
		pr_err("kzalloc couldn't allocate memory\n");
		return -ENOMEM;
	}

	gp2a->enabled = 0;
	gp2a->delay = SENSOR_DEFAULT_DELAY;

	/* Local value initialize */
#ifdef USE_MODE_B
	mvo_value = 0;
	proximity_value = 1;
#endif

	prox_data = gp2a;

	mutex_init(&gp2a->enable_mutex);
	mutex_init(&gp2a->data_mutex);

	INIT_DELAYED_WORK(&gp2a->work, gp2a_work_func_prox);
	cancel_delayed_work_sync(&gp2a->work);
	printk(KERN_INFO "gp2a_opt_probe proximity regiter workfunc!!\n");

	err = proximity_input_init(gp2a);
	if (err < 0)
		goto error_1;

	err = sysfs_create_group(&gp2a->input_dev->dev.kobj,
				&proximity_attribute_group);
	if (err < 0)
		goto error_2;


	/* set platdata */
	platform_set_drvdata(pdev, gp2a);

	gp2a->uevent_kobj = &pdev->dev.kobj;

	/* wake lock init */
	wake_lock_init(&prx_wake_lock, WAKE_LOCK_SUSPEND, "prx_wake_lock");

	/* init i2c */
	opt_i2c_init();

	if (opt_i2c_client == NULL) {
		pr_err("opt_probe failed : i2c_client is NULL\n");
		return -ENODEV;
	} else {
		printk(KERN_INFO "opt_i2c_client : (0x%p)\n", opt_i2c_client);
	}

	/* GP2A Regs INIT SETTINGS */
	value = 0x03;
	opt_i2c_write((u8)(REGS_OPMOD), &value);

	/* INT Settings */
	err = request_threaded_irq(IRQ_GP2A_INT ,
		NULL, gp2a_irq_handler,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		"proximity_int", gp2a);

	if (err < 0) {
		printk(KERN_ERR "failed to request proximity_irq\n");
		goto error_2;
	}

	device_init_wakeup(&pdev->dev, gp2a_pdata->wakeup);

	disable_irq(IRQ_GP2A_INT);
	value = 0x02;
	opt_i2c_write((u8)(REGS_OPMOD), &value);
	sensors_register(prox_sys_device,
		gp2a, prox_attrs,
		"proximity_sensor");
	printk(KERN_INFO "gp2a_opt_probe proximity is OK!!\n");

	return 0;

error_2:
	input_unregister_device(gp2a->input_dev);
	input_free_device(gp2a->input_dev);
error_1:
	kfree(gp2a);
	return err;
}

static int gp2a_opt_remove(struct platform_device *pdev)
{
	struct gp2a_data *gp2a = platform_get_drvdata(pdev);

	if (gp2a->input_dev != NULL) {
		sysfs_remove_group(&gp2a->input_dev->dev.kobj,
			&proximity_attribute_group);
		input_unregister_device(gp2a->input_dev);
		if (gp2a->input_dev != NULL)
			kfree(gp2a->input_dev);
	}

	kfree(gp2a);

	return 0;
}

static int proximity_onoff(u8 onoff)
{
	u8 value;
	int i, j, err = 0;

	printk(KERN_INFO "The proximity sensor is %s\n",
				(onoff) ? "ON" : "OFF");
	if (onoff) {
		if (gp2a_pdata && gp2a_pdata->power_en)
			gp2a_pdata->power_en(1);

#ifdef USE_MODE_B
		mvo_value = 0;
#endif
		for (j = 0; j < 3; j++) {
			if ((err < 0) && gp2a_pdata && gp2a_pdata->power_en) {
				printk(KERN_INFO "gp2a need to reset!\n");
				gp2a_pdata->power_en(0);
				gp2a_pdata->power_en(1);
			}
			for (i = 1; i < 5; i++) {
				err = opt_i2c_write((u8)(i),
						&gp2a_original_image[i]);
				if (err < 0)
					break;
			}
			if (err >= 0)
				break;
		}
#ifdef USE_MODE_B
		value = 0x00;
		opt_i2c_write((u8)(REGS_CON), &value);
#endif
	} else {
		/* set shutdown mode */
		value = 0x02;
		opt_i2c_write((u8)(REGS_OPMOD), &value);

		if (gp2a_pdata && gp2a_pdata->power_en)
			gp2a_pdata->power_en(0);
	}

	return 0;
}

static int opt_i2c_remove(struct i2c_client *client)
{
	struct opt_state *data = i2c_get_clientdata(client);

	kfree(data);
	opt_i2c_client = NULL;

	return 0;
}

static int opt_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct opt_state *opt;

	opt = kzalloc(sizeof(struct opt_state), GFP_KERNEL);
	if (opt == NULL) {
		printk(KERN_ERR "failed to allocate memory\n");
		return -ENOMEM;
	}

	opt->client = client;
	i2c_set_clientdata(client, opt);

	/* rest of the initialisation goes here. */

	printk(KERN_INFO "GP2A opt i2c attach success!!!\n");

	gp2a_pdata = client->dev.platform_data;

	opt_i2c_client = client;

	return 0;
}


static const struct i2c_device_id opt_device_id[] = {
	{"gp2a", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, opt_device_id);

static struct i2c_driver opt_i2c_driver = {
	.driver = {
		.name = "gp2a",
		.owner = THIS_MODULE,
	},
	.probe		= opt_i2c_probe,
	.remove		= opt_i2c_remove,
	.id_table	= opt_device_id,
};


static struct platform_driver gp2a_opt_driver = {
	.probe	 = gp2a_opt_probe,
	.remove = gp2a_opt_remove,
	.driver  = {
		.name = "gp2a-opt",
		.owner = THIS_MODULE,
	},
};

static int __init gp2a_opt_init(void)
{
	int ret;
	ret = platform_driver_register(&gp2a_opt_driver);

	return ret;
}

static void __exit gp2a_opt_exit(void)
{
	platform_driver_unregister(&gp2a_opt_driver);
}

module_init(gp2a_opt_init);
module_exit(gp2a_opt_exit);

MODULE_AUTHOR("SAMSUNG");
MODULE_DESCRIPTION("Optical Sensor driver for GP2AP002A00F");
MODULE_LICENSE("GPL");
