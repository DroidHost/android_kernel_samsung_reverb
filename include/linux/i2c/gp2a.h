#ifndef __GP2A_H__
#define __GP2A_H__

#define I2C_M_WR 0 /* for i2c */
#define I2c_M_RD 1 /* for i2c */


#define I2C_DF_NOTIFY			0x01 /* for i2c */
#define CHIP_DEV_NAME	"GP2AP002A"
#define CHIP_DEV_VENDOR	"SHARP"

#define GP2A_ADDR		0x88 /* slave addr for i2c */

/* Internal Registers */
#define REGS_PROX		0x0
#define REGS_GAIN		0x1
#define REGS_HYS		0x2
#define REGS_CYCLE		0x3
#define REGS_OPMOD		0x4
#define REGS_CON		0x6

/* sensor type */
#define LIGHT			0
#define PROXIMITY		1
#define ALL				2

/* power control */
#define ON			1
#define OFF			0

/* IOCTL for proximity sensor */
#define SHARP_GP2AP_IOC_MAGIC   'C'
#define SHARP_GP2AP_OPEN    _IO(SHARP_GP2AP_IOC_MAGIC, 1)
#define SHARP_GP2AP_CLOSE   _IO(SHARP_GP2AP_IOC_MAGIC, 2)

/* input device for proximity sensor */
#define USE_INPUT_DEVICE	0  /* 0 : No Use  ,  1: Use  */

#if defined(CONFIG_MACH_VITAL2) || defined(CONFIG_MACH_ROOKIE2) || \
	defined(CONFIG_MACH_VITAL2REFRESH)
#define USE_MODE_B
#endif

#define INT_CLEAR    1
#define LIGHT_PERIOD 1 /* per sec */
#define ADC_CHANNEL  9 /* index for s5pC110 9번 channel adc */

/*for light sensor */
#define STATE_NUM			3   /* number of states */
#define LIGHT_LOW_LEVEL     1    /* brightness of lcd */
#define LIGHT_MID_LEVEL		11
#define LIGHT_HIGH_LEVEL	23

#define ADC_BUFFER_NUM		6


#define GUARDBAND_BOTTOM_ADC	700
#define GUARDBAND_TOP_ADC		800

#ifdef MSM_LIGHTSENSOR_ADC_READ
extern int __devinit msm_lightsensor_init_rpc(void);
extern u32 lightsensor_get_adc(void);
extern void msm_lightsensor_cleanup(void);
#endif

/*
 * STATE0 : 30 lux below
 * STATE1 : 31~ 3000 lux
 * STATE2 : 3000 lux over
 */


#define ADC_CUT_HIGH 1100 /* boundary line between STATE_0 and STATE_1 */
#define ADC_CUT_LOW  220  /* boundary line between STATE_1 and STATE_2 */
#define ADC_CUT_GAP  50   /* in order to prevent chattering condition */


#define LIGHT_FOR_16LEVEL

/* initial value for sensor register */
#ifdef USE_MODE_B
static u8 gp2a_original_image[5] = {
	0x00,
	0x08,
	0x40,
	0x04,
	0x03,
};
#else
static u8 gp2a_original_image[5] = {
	0x00,
	0x08,
	0xC2,
	0x04,
	0x01,
};
#endif

/* for state transition */
struct _light_state {
	int adc_bottom_limit;
	int adc_top_limit;
	int brightness;

};

struct gp2a_platform_data {
	int (*power_en)(int);
	unsigned int wakeup;
};


extern int __devinit msm_lightsensor_init_rpc(void);
extern u32 lightsensor_get_adc(void);
extern void msm_lightsensor_cleanup(void);
extern short gp2a_get_proximity_value(void);
extern bool gp2a_get_lightsensor_status(void);
int opt_i2c_read(u8 reg, u8 *val, unsigned int len);
int opt_i2c_write(u8 reg, u8 *val);
void lightsensor_adjust_brightness(int level);
int lightsensor_get_adcvalue(void);


#endif
