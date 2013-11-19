/*
 * cypress_touchkey.h - Platform data for cypress touchkey driver
 *
 * Copyright (C) 2011 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __LINUX_CYPRESS_TOUCHKEY_H
#define __LINUX_CYPRESS_TOUCHKEY_H
extern struct class *sec_class;
extern int ISSP_main(void);
extern int touch_is_pressed;
struct cypress_touchkey_platform_data {
	unsigned	gpio_int;
	unsigned	gpio_led_en;
	const u8	*touchkey_keycode;
	void	(*power_onoff) (int);
};
#endif /* __LINUX_CYPRESS_TOUCHKEY_H */
