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
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/i2c/gp2a.h>
#include <mach/vreg.h>
#include <linux/sensors_core.h>

/* for debugging */
#define DEBUG 0

#define SENSOR_NAME "light_sensor"

#define SENSOR_DEFAULT_DELAY            (200)
#define SENSOR_MAX_DELAY                (2000)
#define SENSOR_MIN_DELAY                (10)

#define ABS_STATUS                      (ABS_BRAKE)
#define ABS_WAKE                        (ABS_MISC)
#define ABS_CONTROL_REPORT              (ABS_THROTTLE)

#if defined(CONFIG_MACH_CHIEF) || defined(CONFIG_MACH_VITAL2) || \
	defined(CONFIG_MACH_ROOKIE2) || defined(CONFIG_MACH_VITAL2REFRESH)
#define MSM_LIGHTSENSOR_ADC_READ
#endif
struct sensor_data {
	struct mutex mutex;
	struct delayed_work work;
	struct input_dev *input_dev;

	int enabled;
	int delay;

#if DEBUG
	int suspend;
#endif
};
/* global var */
static struct platform_device *sensor_pdev;
static struct input_dev *this_data;
static struct device *light_sys_device;
static int cur_adc_value;

static int buffering = 2;
static int adc_value_buf[ADC_BUFFER_NUM] = {0};

int autobrightness_mode = OFF;

/* Light Sysfs interface */
static ssize_t
light_delay_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
	int delay;

	delay = data->delay;

	return sprintf(buf, "%d\n", delay);
}

static ssize_t
light_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
	long delay = 0;
	int original_delay = -1;

	if (strict_strtol(buf, 10, &delay) < 0)
		return -EINVAL;

	if (delay < 0) {
		return count;
	} else {
		/* requested by HAL team */
		original_delay = delay;
		delay /= 1000000; /* ns -> ms */
		printk(KERN_INFO "light work - delay : %d ms\n", delay);
		if (delay < SENSOR_MIN_DELAY) {
			printk(KERN_INFO "light work - %d ms delay(original %d)"
				" is canceled\n",
				delay, original_delay);
			return count;
		}
	}

	if (SENSOR_MAX_DELAY < delay)
		delay = SENSOR_MAX_DELAY;

	data->delay = delay;

	mutex_lock(&data->mutex);

	if (data->enabled) {
		cancel_delayed_work_sync(&data->work);
		schedule_delayed_work(&data->work, msecs_to_jiffies(delay));
	}

	mutex_unlock(&data->mutex);

	return count;
}

static ssize_t
light_enable_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
	int enabled;

	enabled = data->enabled;

	return sprintf(buf, "%d\n", enabled);
}

static ssize_t
light_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
	long value = 0;
	if (strict_strtol(buf, 10, &value) < 0)
		return -EINVAL;

	if (value != 0 && value != 1)
		return count;

	mutex_lock(&data->mutex);

	if (data->enabled && !value) {
		cancel_delayed_work_sync(&data->work);
		printk(KERN_INFO"timer canceled.\n");
	}
	if (!data->enabled && value) {
		schedule_delayed_work(&data->work, 0);
		printk(KERN_INFO"timer started.\n");
	}
	data->enabled = value;

	mutex_unlock(&data->mutex);

	return count;
}

static ssize_t
light_data_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct sensor_data *data = input_get_drvdata(input_data);
	int light_lux;

	light_lux = lightsensor_get_adcvalue();

	return sprintf(buf, "%d\n", light_lux);
}


static ssize_t
light_name_show(struct device *dev, struct device_attribute *attr,
		   char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_DEV_NAME);
}
static ssize_t
light_vendor_show(struct device *dev, struct device_attribute *attr,
		   char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_DEV_VENDOR);
}

static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
	light_delay_show, light_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
	light_enable_show, light_enable_store);
static DEVICE_ATTR(raw_data, S_IRUGO,
	light_data_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO,
	light_vendor_show, NULL);
static DEVICE_ATTR(name, S_IRUGO,
	light_name_show, NULL);

static struct attribute *lightsensor_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	NULL
};
static struct device_attribute *light_attrs[] = {
	&dev_attr_raw_data,
	&dev_attr_vendor,
	&dev_attr_name,
	NULL,
};
static struct attribute_group lightsensor_attribute_group = {
	.attrs = lightsensor_attributes
};

static int
lightsensor_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct sensor_data *data = input_get_drvdata(this_data);
	struct vreg *vreg_L11;
	int rt = 0;

	mutex_lock(&data->mutex);

	if (data->enabled) {
		rt = cancel_delayed_work_sync(&data->work);
		printk(KERN_INFO": The timer is cancled.\n");
	}

	mutex_unlock(&data->mutex);

	vreg_L11 = vreg_get(NULL, "gp2");

	if (IS_ERR(vreg_L11)) {
		rt = PTR_ERR(vreg_L11);
		pr_err("%s: vreg_L11 get failed (%d)\n",
				__func__, rt);
		return rt;
	}

	rt = vreg_disable(vreg_L11);

	if (rt) {
		pr_err("%s: L11 vreg enable failed (%d)\n",
				__func__, rt);
		return rt;
	}

	printk(KERN_INFO"vreg_11 is off.\n");

	return rt;
}

static int
lightsensor_resume(struct platform_device *pdev)
{
	struct sensor_data *data = input_get_drvdata(this_data);
	struct vreg *vreg_L11;
	int rt = 0;

	mutex_lock(&data->mutex);

	if (data->enabled) {
		rt = schedule_delayed_work(&data->work, 0);
		printk(KERN_INFO": The timer is started.\n");
	}

	mutex_unlock(&data->mutex);

	vreg_L11 = vreg_get(NULL, "gp2");

	if (IS_ERR(vreg_L11)) {
		rt = PTR_ERR(vreg_L11);
		pr_err("%s: vreg_L11 get failed (%d)\n",
				__func__, rt);
		return rt;
	}

	rt = vreg_set_level(vreg_L11, 3000);
	if (rt) {
		pr_err("%s: vreg_L11 set level failed (%d)\n",
				__func__, rt);
		return rt;
	}

	rt = vreg_enable(vreg_L11);

	if (rt) {
		pr_err("%s: L11 vreg enable failed (%d)\n",
				__func__, rt);
		return rt;
	}

	printk(KERN_INFO"vreg_11 is on.\n");

	return rt;
}

int lightsensor_get_adcvalue(void)
{
	int value = 0;

#ifdef MSM_LIGHTSENSOR_ADC_READ
	value = lightsensor_get_adc();
#endif
	if (value < 60)
		value = value / 100;
	else if (value < 600)
		value = value / 10;
	else
		value = value / 5;
	return value;
}

static void gp2a_work_func_light(struct work_struct *work)
{
	struct sensor_data *data = container_of((struct delayed_work *)work,
			struct sensor_data, work);

	int adc = 0;
	int lux = 0;
	static int count ;

	/* read adc data from s5p110 */
	adc = lightsensor_get_adcvalue();

	input_report_abs(data->input_dev, ABS_MISC, adc);
	input_sync(data->input_dev);

	schedule_delayed_work(&data->work, msecs_to_jiffies(data->delay));
}

#ifdef MSM_LIGHTSENSOR_ADC_READ
void lightsensor_rpc_init(void)
{
	/* RPC initial sequence */
	int err = 1;

	printk(KERN_INFO"\n");

	err = msm_lightsensor_init_rpc();

	if (err < 0) {
		pr_err("%s: FAIL: msm_lightsensor_init_rpc. err=%d\n",
			__func__, err);
		msm_lightsensor_cleanup();
	}
}
#endif

static int
lightsensor_probe(struct platform_device *pdev)
{
	struct sensor_data *data = NULL;
	struct input_dev *input_data = NULL;
	int input_registered = 0, sysfs_created = 0;
	int rt;

	data = kzalloc(sizeof(struct sensor_data), GFP_KERNEL);
	if (!data) {
		rt = -ENOMEM;
		goto err;
	}
	data->enabled = 0;
	data->delay = SENSOR_DEFAULT_DELAY;

	INIT_DELAYED_WORK(&data->work, gp2a_work_func_light);

	input_data = input_allocate_device();
	if (!input_data) {
		rt = -ENOMEM;
		printk(KERN_ERR
				"sensor_probe: Failed to allocate input_data device\n");
		goto err;
	}

	set_bit(EV_ABS, input_data->evbit);
	input_set_capability(input_data, EV_ABS, ABS_MISC);
	input_set_abs_params(input_data, ABS_MISC, 0, 1, 0, 0);
	input_data->name = SENSOR_NAME;

	rt = input_register_device(input_data);
	if (rt) {
		printk(KERN_ERR
				"sensor_probe: Unable to register input_data device: %s\n",
				input_data->name);
		goto err;
	}
	input_set_drvdata(input_data, data);
	input_registered = 1;
	data->input_dev = input_data;
	rt = sysfs_create_group(&input_data->dev.kobj,
			&lightsensor_attribute_group);
	if (rt) {
		printk(KERN_ERR
				"sensor_probe: sysfs_create_group failed[%s]\n",
				input_data->name);
		goto err;
	}
	sysfs_created = 1;
	mutex_init(&data->mutex);
	this_data = input_data;

#ifdef MSM_LIGHTSENSOR_ADC_READ
	lightsensor_rpc_init();
#endif
	sensors_register(light_sys_device,
		data, light_attrs,
		"light_sensor");

	return 0;

err:
	if (data != NULL) {
		if (input_data != NULL) {
			if (sysfs_created) {
				sysfs_remove_group(&input_data->dev.kobj,
						&lightsensor_attribute_group);
			}
			if (input_registered)
				input_unregister_device(input_data);
			else
				input_free_device(input_data);

			input_data = NULL;
		}
		kfree(data);
	}

	return rt;
}

static int
lightsensor_remove(struct platform_device *pdev)
{
	struct sensor_data *data;

	if (this_data != NULL) {
		data = input_get_drvdata(this_data);
		sysfs_remove_group(&this_data->dev.kobj,
				&lightsensor_attribute_group);
		if (data != NULL) {
			cancel_delayed_work(&data->work);
			flush_scheduled_work();
			kfree(data);
		}
		input_unregister_device(this_data);
	}

	return 0;
}

/*
 * Module init and exit
 */
static struct platform_driver lightsensor_driver = {
	.probe      = lightsensor_probe,
	.remove     = lightsensor_remove,
	.suspend    = lightsensor_suspend,
	.resume     = lightsensor_resume,
#if !defined(CONFIG_MACH_CHIEF) && \
	!defined(CONFIG_MACH_VITAL2) && \
	!defined(CONFIG_MACH_ROOKIE2)
	.shutdown = lightsensor_remove,
#endif
	.driver = {
		.name   = SENSOR_NAME,
		.owner  = THIS_MODULE,
	},
};

static int __init lightsensor_init(void)
{
	sensor_pdev = platform_device_register_simple(SENSOR_NAME, 0, NULL, 0);
	if (IS_ERR(sensor_pdev))
		return -1;

	return platform_driver_register(&lightsensor_driver);
}
module_init(lightsensor_init);

static void __exit lightsensor_exit(void)
{
	platform_driver_unregister(&lightsensor_driver);
	platform_device_unregister(sensor_pdev);
}
module_exit(lightsensor_exit);

MODULE_AUTHOR("SAMSUNG");
MODULE_DESCRIPTION("Optical Sensor driver for GP2AP002A00F");
MODULE_LICENSE("GPL");
