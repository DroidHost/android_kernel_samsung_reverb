#ifndef _LINUX_FTK_I2C_H
#define _LINUX_FTK_I2C_H

struct ftk_i2c_platform_data {
	int (*power) (int on);
};

#define SEC_TSP_FACTORY_TEST
#define CONFIG_MULTI_TOUCH_PROTOCOL_TYPE_B
#define FTK_USE_CHARGER_DETECTION
/*
#define FTK_USE_POLLING_MODE
*/
#ifdef SEC_TSP_FACTORY_TEST
extern struct class *sec_class;
#endif

#ifdef FTK_USE_CHARGER_DETECTION
extern int ftk_charger_status;
#endif

extern int touch_is_pressed;
#endif
