/* Copyright (c) 2010, 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/mfd/pm8xxx/core.h>
#include <linux/leds-pmic8058.h>

#include <linux/pwm.h>

#define SSBI_REG_ADDR_DRV_KEYPAD	0x48
#define PM8058_DRV_KEYPAD_BL_MASK	0xf0
#define PM8058_DRV_KEYPAD_BL_SHIFT	0x04

#define SSBI_REG_ADDR_FLASH_DRV0        0x49
#define PM8058_DRV_FLASH_MASK           0xf0
#define PM8058_DRV_FLASH_SHIFT          0x04

#define SSBI_REG_ADDR_FLASH_DRV1        0xFB

#define SSBI_REG_ADDR_LED_CTRL_BASE	0x131
#define SSBI_REG_ADDR_LED_CTRL(n)	(SSBI_REG_ADDR_LED_CTRL_BASE + (n))
#define PM8058_DRV_LED_CTRL_MASK	0xf8
#define PM8058_DRV_LED_CTRL_SHIFT	0x03

#define MAX_FLASH_CURRENT	300
#define MAX_KEYPAD_CURRENT 300
#define MAX_KEYPAD_BL_LEVEL	(1 << 4)
#define MAX_LED_DRV_LEVEL	20 /* 2 * 20 mA */

#define PMIC8058_LED_OFFSET(id) ((id) - PMIC8058_ID_LED_0)

struct pmic8058_led_data {
	struct device		*dev;
	struct led_classdev	cdev;
	int			id;
	enum led_brightness	brightness;
	u8			flags;
	struct work_struct	work;
	struct mutex		lock;
	spinlock_t		value_lock;
	u8			reg_kp;
	u8			reg_led_ctrl[3];
	u8			reg_flash_led0;
	u8			reg_flash_led1;
	struct pwm_device *pwm_dev;
};

#define PM8058_MAX_LEDS		7
static struct pmic8058_led_data led_data[PM8058_MAX_LEDS];

static void kp_bl_set(struct pmic8058_led_data *led, enum led_brightness value)
{
	int rc;
	u8 level;
	unsigned long flags;

	spin_lock_irqsave(&led->value_lock, flags);
	level = (value << PM8058_DRV_KEYPAD_BL_SHIFT) &
				 PM8058_DRV_KEYPAD_BL_MASK;

	led->reg_kp &= ~PM8058_DRV_KEYPAD_BL_MASK;
	led->reg_kp |= level;
	spin_unlock_irqrestore(&led->value_lock, flags);

	rc = pm8xxx_writeb(led->dev->parent, SSBI_REG_ADDR_DRV_KEYPAD,
						led->reg_kp);
	if (rc)
		pr_err("%s: can't set keypad backlight level\n", __func__);
}

static enum led_brightness kp_bl_get(struct pmic8058_led_data *led)
{
	if ((led->reg_kp & PM8058_DRV_KEYPAD_BL_MASK) >>
			 PM8058_DRV_KEYPAD_BL_SHIFT)
		return LED_FULL;
	else
		return LED_OFF;
}

static void led_lc_set(struct pmic8058_led_data *led, enum led_brightness value)
{
	unsigned long flags;
	int rc, offset;
	u8 level, tmp;

	spin_lock_irqsave(&led->value_lock, flags);

	level = (led->brightness << PM8058_DRV_LED_CTRL_SHIFT) &
		PM8058_DRV_LED_CTRL_MASK;

	offset = PMIC8058_LED_OFFSET(led->id);
	tmp = led->reg_led_ctrl[offset];

	tmp &= ~PM8058_DRV_LED_CTRL_MASK;
	tmp |= level;
	spin_unlock_irqrestore(&led->value_lock, flags);

	rc = pm8xxx_writeb(led->dev->parent, SSBI_REG_ADDR_LED_CTRL(offset),
								tmp);
	if (rc) {
		dev_err(led->cdev.dev, "can't set (%d) led value\n",
				led->id);
		return;
	}

	spin_lock_irqsave(&led->value_lock, flags);
	led->reg_led_ctrl[offset] = tmp;
	spin_unlock_irqrestore(&led->value_lock, flags);
}

static enum led_brightness led_lc_get(struct pmic8058_led_data *led)
{
	int offset;
	u8 value;

	offset = PMIC8058_LED_OFFSET(led->id);
	value = led->reg_led_ctrl[offset];

	if ((value & PM8058_DRV_LED_CTRL_MASK) >>
			PM8058_DRV_LED_CTRL_SHIFT)
		return LED_FULL;
	else
		return LED_OFF;
}

static void
led_flash_set(struct pmic8058_led_data *led, enum led_brightness value)
{
	int rc;
	u8 level;
	unsigned long flags;
	u8 reg_flash_led;
	u16 reg_addr;

	spin_lock_irqsave(&led->value_lock, flags);
	level = (value << PM8058_DRV_FLASH_SHIFT) &
				 PM8058_DRV_FLASH_MASK;

	if (led->id == PMIC8058_ID_FLASH_LED_0) {
		led->reg_flash_led0 &= ~PM8058_DRV_FLASH_MASK;
		led->reg_flash_led0 |= level;
		reg_flash_led	    = led->reg_flash_led0;
		reg_addr	    = SSBI_REG_ADDR_FLASH_DRV0;
	} else {
		led->reg_flash_led1 &= ~PM8058_DRV_FLASH_MASK;
		led->reg_flash_led1 |= level;
		reg_flash_led	    = led->reg_flash_led1;
		reg_addr	    = SSBI_REG_ADDR_FLASH_DRV1;
	}
	spin_unlock_irqrestore(&led->value_lock, flags);

	rc = pm8xxx_writeb(led->dev->parent, reg_addr, reg_flash_led);
	if (rc)
		pr_err("%s: can't set flash led%d level %d\n", __func__,
			led->id, rc);
}

int pm8058_set_flash_led_current(enum pmic8058_leds id, unsigned mA)
{
	struct pmic8058_led_data *led;

	if ((id < PMIC8058_ID_FLASH_LED_0) || (id > PMIC8058_ID_FLASH_LED_1)) {
		pr_err("%s: invalid LED ID (%d) specified\n", __func__, id);
		return -EINVAL;
	}

	led = &led_data[id];
	if (!led) {
		pr_err("%s: flash led not available\n", __func__);
		return -EINVAL;
	}

	if (mA > MAX_FLASH_CURRENT)
		return -EINVAL;

	led_flash_set(led, mA / 20);

	return 0;
}
EXPORT_SYMBOL(pm8058_set_flash_led_current);

int pm8058_set_led_current(enum pmic8058_leds id, unsigned mA)
{
	struct pmic8058_led_data *led;
	int brightness = 0;

	if ((id < PMIC8058_ID_LED_KB_LIGHT) || (id > PMIC8058_ID_FLASH_LED_1)) {
		pr_err("%s: invalid LED ID (%d) specified\n", __func__, id);
		return -EINVAL;
	}

	led = &led_data[id];
	if (!led) {
		pr_err("%s: flash led not available\n", __func__);
		return -EINVAL;
	}

	switch (id) {
	case PMIC8058_ID_LED_0:
	case PMIC8058_ID_LED_1:
	case PMIC8058_ID_LED_2:
		brightness = mA / 2;
		if (brightness  > led->cdev.max_brightness)
			return -EINVAL;
		led_lc_set(led, brightness);
		break;

	case PMIC8058_ID_LED_KB_LIGHT:
	case PMIC8058_ID_FLASH_LED_0:
	case PMIC8058_ID_FLASH_LED_1:
		brightness = mA / 20;
		if (brightness  > led->cdev.max_brightness)
			return -EINVAL;
		if (id == PMIC8058_ID_LED_KB_LIGHT)
			kp_bl_set(led, brightness);
		else
			led_flash_set(led, brightness);
		break;
	}

	return 0;
}
EXPORT_SYMBOL(pm8058_set_led_current);

static void pmic8058_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct pmic8058_led_data *led;
	unsigned long flags;

	led = container_of(led_cdev, struct pmic8058_led_data, cdev);

	spin_lock_irqsave(&led->value_lock, flags);
	led->brightness = value;
	schedule_work(&led->work);
	spin_unlock_irqrestore(&led->value_lock, flags);
}

static int lpg_blink_set(struct led_classdev *led_cdev,
		unsigned long *delay_on, unsigned long *delay_off)
{
	struct pmic8058_led_data *led;

	led = container_of(led_cdev, struct pmic8058_led_data, cdev);

	pr_info("[%s] led_id=%d delay_on=%lu delay_off=%lu",
			__func__, led->id, *delay_on, *delay_off);

	if (*delay_on && *delay_off) {
		/* disable suspend */
		led_cdev->flags = led_cdev->flags & ~LED_CORE_SUSPENDRESUME;
		if (led->pwm_dev) {
			pwm_disable(led->pwm_dev);
			pwm_free(led->pwm_dev);
			led->pwm_dev = NULL;
		}

		led->pwm_dev = pwm_request(led->id+1, "pm8058_lpg_pwm");

		pwm_config(led->pwm_dev, (*delay_on)*1000,
				((*delay_off)+(*delay_on))*1000);
		pwm_enable(led->pwm_dev);
	} else {
		/* enable suspend */
		led_cdev->flags = led_cdev->flags | LED_CORE_SUSPENDRESUME;
		if (led->pwm_dev) {
			pwm_disable(led->pwm_dev);
			pwm_free(led->pwm_dev);
			led->pwm_dev = NULL;
		}
	}

	return 0;
}

static void pmic8058_led_work(struct work_struct *work)
{
	struct pmic8058_led_data *led = container_of(work,
					 struct pmic8058_led_data, work);

	mutex_lock(&led->lock);

	switch (led->id) {
	case PMIC8058_ID_LED_KB_LIGHT:
		kp_bl_set(led, led->brightness);
		break;
	case PMIC8058_ID_LED_0:
	case PMIC8058_ID_LED_1:
	case PMIC8058_ID_LED_2:
		if (led->brightness == LED_HALF)
			lpg_blink_set(&led->cdev, &led->cdev.blink_delay_on,
				&led->cdev.blink_delay_off);
		else
			led_lc_set(led, led->brightness);
		break;
	case PMIC8058_ID_FLASH_LED_0:
	case PMIC8058_ID_FLASH_LED_1:
		led_flash_set(led, led->brightness);
		break;
	}

	mutex_unlock(&led->lock);
}

static enum led_brightness pmic8058_led_get(struct led_classdev *led_cdev)
{
	struct pmic8058_led_data *led;

	led = container_of(led_cdev, struct pmic8058_led_data, cdev);

	switch (led->id) {
	case PMIC8058_ID_LED_KB_LIGHT:
		return kp_bl_get(led);
	case PMIC8058_ID_LED_0:
	case PMIC8058_ID_LED_1:
	case PMIC8058_ID_LED_2:
		return led_lc_get(led);
	}
	return LED_OFF;
}

static ssize_t led_pattern_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 4, "%u\n", 0);
}

static ssize_t led_pattern_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pmic8058_led_data *led;
	unsigned int index_led_red;
	unsigned int index_led_blue;

#ifdef CONFIG_MACH_PREVAIL2
	index_led_red = PMIC8058_ID_LED_1;
	index_led_blue = PMIC8058_ID_LED_2;
#else /*CONFIG_MACH_ICON*/
	if (system_rev >= 3) {
		index_led_red = PMIC8058_ID_LED_1;
		index_led_blue = PMIC8058_ID_LED_2;
	} else {
		index_led_red = PMIC8058_ID_LED_2;
		index_led_blue = PMIC8058_ID_LED_1;
	}
#endif
	if (buf[0] == '1') {
		pr_info("LED Battery Charging Pattern on\n");
		led = &led_data[index_led_blue];
		led->cdev.flags = led->cdev.flags | LED_CORE_SUSPENDRESUME;
		pmic8058_led_set(&led->cdev, LED_OFF);
		led = &led_data[index_led_red];
		led->cdev.flags = led->cdev.flags & ~LED_CORE_SUSPENDRESUME;
		pmic8058_led_set(&led->cdev, led->cdev.max_brightness);
	} else if (buf[0] == '2') {
		pr_info("LED Battery Charging Error Pattern on\n");
		led = &led_data[index_led_blue];
		led->cdev.flags = led->cdev.flags | LED_CORE_SUSPENDRESUME;
		pmic8058_led_set(&led->cdev, LED_OFF);
		led = &led_data[index_led_red];
		led->cdev.blink_delay_on = 500;
		led->cdev.blink_delay_off = 500;
		pmic8058_led_set(&led->cdev, LED_HALF);
	} else if (buf[0] == '3') {
		pr_info("LED LED Missed Call Notification Pattern on\n");
		led = &led_data[index_led_red];
		led->cdev.flags = led->cdev.flags | LED_CORE_SUSPENDRESUME;
		pmic8058_led_set(&led->cdev, LED_OFF);
		led = &led_data[index_led_blue];
		led->cdev.blink_delay_on = 500;
		led->cdev.blink_delay_off = 5000;
		pmic8058_led_set(&led->cdev, LED_HALF);
	} else if (buf[0] == '4') {
		pr_info("LED Low Battery Pattern on\n");
		led = &led_data[index_led_blue];
		led->cdev.flags = led->cdev.flags | LED_CORE_SUSPENDRESUME;
		pmic8058_led_set(&led->cdev, LED_OFF);
		led = &led_data[index_led_red];
		led->cdev.blink_delay_on = 500;
		led->cdev.blink_delay_off = 5000;
		pmic8058_led_set(&led->cdev, LED_HALF);
	} else if (buf[0] == '5') {
		pr_info("LED Full Battery Charging Pattern on\n");
		led = &led_data[index_led_red];
		led->cdev.flags = led->cdev.flags | LED_CORE_SUSPENDRESUME;
		pmic8058_led_set(&led->cdev, LED_OFF);
		led = &led_data[index_led_blue];
		led->cdev.flags = led->cdev.flags | LED_CORE_SUSPENDRESUME;
		pmic8058_led_set(&led->cdev, LED_OFF);
	} else if (buf[0] == '6') {
		pr_info("LED Power on Pattern\n");
	} else if (buf[0] == '7' || buf[0] == '0') {
		pr_info("LED Turn off\n");
		led = &led_data[index_led_red];
		led->cdev.flags = led->cdev.flags | LED_CORE_SUSPENDRESUME;
		pmic8058_led_set(&led->cdev, LED_OFF);
		led = &led_data[index_led_blue];
		led->cdev.flags = led->cdev.flags | LED_CORE_SUSPENDRESUME;
		pmic8058_led_set(&led->cdev, LED_OFF);
	}
	return size;
}

static DEVICE_ATTR(led_pattern, S_IRUGO | S_IWUSR | S_IWGRP,
		led_pattern_show, led_pattern_store);

static ssize_t led_r_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pmic8058_led_data *led;
	int brightness;
	unsigned int index_led_red;

#ifdef CONFIG_MACH_PREVAIL2
	index_led_red = PMIC8058_ID_LED_1;
#else /*CONFIG_MACH_ICON*/
	if (system_rev >= 3)
		index_led_red = PMIC8058_ID_LED_1;
	else
		index_led_red = PMIC8058_ID_LED_2;
#endif

	led = &led_data[index_led_red];
	brightness = (buf[0] == '0') ? LED_OFF : led->cdev.max_brightness ;
	pmic8058_led_set(&led->cdev, brightness);

	return size;
}

static DEVICE_ATTR(led_r, S_IRUGO | S_IWUSR | S_IWGRP,
		NULL, led_r_store);

static ssize_t led_b_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct pmic8058_led_data *led;
	int brightness;
	unsigned int index_led_blue;

#ifdef CONFIG_MACH_PREVAIL2
	index_led_blue = PMIC8058_ID_LED_2;
#else /*CONFIG_MACH_ICON*/
	if (system_rev >= 3)
		index_led_blue = PMIC8058_ID_LED_2;
	else
		index_led_blue = PMIC8058_ID_LED_1;
#endif
	led = &led_data[index_led_blue];
	brightness = (buf[0] == '0') ? LED_OFF : led->cdev.max_brightness ;
	pmic8058_led_set(&led->cdev, brightness);

	return size;
}

static DEVICE_ATTR(led_b, S_IRUGO | S_IWUSR | S_IWGRP,
		NULL, led_b_store);

static void led_virtual_dev(struct pmic8058_led_data *info)
{
	struct device *sec_led;
	int error = 0;

	sec_led = device_create(sec_class, NULL, 0, NULL, "led");
	error = dev_set_drvdata(sec_led, info);
	if (error)
		pr_err("Failed to set sec_led driver data");
	error = device_create_file(sec_led, &dev_attr_led_pattern);
	if (error)
		pr_err("Failed to create /sys/class/sec/led/led_pattern");
	error = device_create_file(sec_led, &dev_attr_led_r);
	if (error)
		pr_err("Failed to create /sys/class/sec/led/led_r");
	error = device_create_file(sec_led, &dev_attr_led_b);
	if (error)
		pr_err("Failed to create /sys/class/sec/led/led_b");
}
static int pmic8058_led_probe(struct platform_device *pdev)
{
	struct pmic8058_leds_platform_data *pdata = pdev->dev.platform_data;
	struct pmic8058_led_data *led_dat;
	struct pmic8058_led *curr_led;
	int rc, i = 0;
	u8			reg_kp;
	u8			reg_led_ctrl[3];
	u8			reg_flash_led0;
	u8			reg_flash_led1;

	if (pdata == NULL) {
		dev_err(&pdev->dev, "platform data not supplied\n");
		return -EINVAL;
	}

	rc = pm8xxx_readb(pdev->dev.parent, SSBI_REG_ADDR_DRV_KEYPAD, &reg_kp);
	if (rc) {
		dev_err(&pdev->dev, "can't get keypad backlight level\n");
		goto err_reg_read;
	}

	rc = pm8xxx_read_buf(pdev->dev.parent, SSBI_REG_ADDR_LED_CTRL_BASE,
							reg_led_ctrl, 3);
	if (rc) {
		dev_err(&pdev->dev, "can't get led levels\n");
		goto err_reg_read;
	}

	rc = pm8xxx_readb(pdev->dev.parent, SSBI_REG_ADDR_FLASH_DRV0,
						&reg_flash_led0);
	if (rc) {
		dev_err(&pdev->dev, "can't read flash led0\n");
		goto err_reg_read;
	}

	rc = pm8xxx_readb(pdev->dev.parent, SSBI_REG_ADDR_FLASH_DRV1,
						&reg_flash_led1);
	if (rc) {
		dev_err(&pdev->dev, "can't get flash led1\n");
		goto err_reg_read;
	}

	for (i = 0; i < pdata->num_leds; i++) {
		curr_led	= &pdata->leds[i];
		led_dat		= &led_data[curr_led->id];

		led_dat->cdev.name		= curr_led->name;
		led_dat->cdev.default_trigger   = curr_led->default_trigger;
		led_dat->cdev.brightness_set    = pmic8058_led_set;
		led_dat->cdev.brightness_get    = pmic8058_led_get;
		led_dat->cdev.brightness	= LED_OFF;
		led_dat->cdev.max_brightness	= curr_led->max_brightness;
		led_dat->cdev.flags		= LED_CORE_SUSPENDRESUME;

		led_dat->id		        = curr_led->id;
		led_dat->reg_kp			= reg_kp;
		memcpy(led_data->reg_led_ctrl, reg_led_ctrl,
					 sizeof(reg_led_ctrl));
		led_dat->reg_flash_led0		= reg_flash_led0;
		led_dat->reg_flash_led1		= reg_flash_led1;

		if (!((led_dat->id >= PMIC8058_ID_LED_KB_LIGHT) &&
				(led_dat->id <= PMIC8058_ID_FLASH_LED_1))) {
			dev_err(&pdev->dev, "invalid LED ID (%d) specified\n",
						 led_dat->id);
			rc = -EINVAL;
			goto fail_id_check;
		}

		led_dat->dev			= &pdev->dev;

		mutex_init(&led_dat->lock);
		spin_lock_init(&led_dat->value_lock);
		INIT_WORK(&led_dat->work, pmic8058_led_work);

		rc = led_classdev_register(&pdev->dev, &led_dat->cdev);
		if (rc) {
			dev_err(&pdev->dev, "unable to register led %d\n",
						 led_dat->id);
			goto fail_id_check;
		}
	}

	platform_set_drvdata(pdev, led_data);
	led_virtual_dev(led_data);

	return 0;

err_reg_read:
fail_id_check:
	if (i > 0) {
		for (i = i - 1; i >= 0; i--)
			led_classdev_unregister(&led_data[i].cdev);
	}
	return rc;
}

static int __devexit pmic8058_led_remove(struct platform_device *pdev)
{
	int i;
	struct pmic8058_leds_platform_data *pdata = pdev->dev.platform_data;
	struct pmic8058_led_data *led = platform_get_drvdata(pdev);

	for (i = 0; i < pdata->num_leds; i++) {
		led_classdev_unregister(&led[led->id].cdev);
		cancel_work_sync(&led[led->id].work);
	}

	return 0;
}

static struct platform_driver pmic8058_led_driver = {
	.probe		= pmic8058_led_probe,
	.remove		= __devexit_p(pmic8058_led_remove),
	.driver		= {
		.name	= "pm8058-led",
		.owner	= THIS_MODULE,
	},
};

static int __init pmic8058_led_init(void)
{
	return platform_driver_register(&pmic8058_led_driver);
}
module_init(pmic8058_led_init);

static void __exit pmic8058_led_exit(void)
{
	platform_driver_unregister(&pmic8058_led_driver);
}
module_exit(pmic8058_led_exit);

MODULE_DESCRIPTION("PMIC8058 LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:pmic8058-led");
