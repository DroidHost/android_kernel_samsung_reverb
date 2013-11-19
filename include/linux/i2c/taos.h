#ifndef __TAOS_H__
#define __TAOS_H__


/* i2c */
#define I2C_M_WR 0 /* for i2c */
#define I2c_M_RD 1 /* for i2c */

/* sensor gpio */
#define GPIO_SENSE_OUT	27

#define REGS_PROX	0x0 /* Read  Only */
#define REGS_GAIN	0x1 /* Write Only */
#define REGS_HYS	0x2 /* Write Only */
#define REGS_CYCLE	0x3 /* Write Only */
#define REGS_OPMOD	0x4 /* Write Only */
#define REGS_CON	0x6 /* Write Only */

/* sensor type */
#define TAOS_LIGHT		0
#define TAOS_PROXIMITY	1
#define TAOS_ALL		2

/* power control */
#define ON	1
#define OFF	0

/* IOCTL for proximity sensor */
#define SHARP_TAOSP_IOC_MAGIC   'C'
#define SHARP_TAOSP_OPEN    _IO(SHARP_TAOSP_IOC_MAGIC, 1)
#define SHARP_TAOSP_CLOSE   _IO(SHARP_TAOSP_IOC_MAGIC, 2)

/* IOCTL for light sensor */
#define SHARP_TAOSL_IOC_MAGIC   'L'
#define SHARP_TAOSL_OPEN    _IO(SHARP_TAOSL_IOC_MAGIC, 1)
#define SHARP_TAOSL_CLOSE   _IO(SHARP_TAOSL_IOC_MAGIC, 2)

#define	MAX_LUX				32768
/* for proximity adc avg */
#define PROX_READ_NUM 40
#define TAOS_PROX_MAX 1023
#define TAOS_PROX_MIN 0

/* input device for proximity sensor */
#define USE_INPUT_DEVICE	1	/* 0 : No Use,	1: Use  */
#define USE_INTERRUPT		1
#define INT_CLEAR    1 /* 0 = by polling operation,
						  1 = by interrupt operation */

/* Register value  for TMD2771x */
#define ATIME 0xff  /* 2.7ms - minimum ALS intergration time */
#define WTIME 0xff  /* 2.7ms - minimum Wait time */
#define PTIME  0xff /* 2.7ms - minimum Prox integration time */
#define PPCOUNT  1
#define PIEN 0x20	/* Enable Prox interrupt */
#define WEN  0x8	/* Enable Wait */
#define PEN  0x4	/* Enable Prox */
#define AEN  0x2	/* Enable ALS */
#define PON 0x1		/* Enable Power on */
#define PDRIVE 0
#define PDIODE 0x20
#define PGAIN 0
#define AGAIN 0

enum TAOS_ALS_FOPS_STATUS {
	TAOS_ALS_CLOSED = 0,
	TAOS_ALS_OPENED = 1,
};

enum TAOS_PRX_FOPS_STATUS {
	TAOS_PRX_CLOSED = 0,
	TAOS_PRX_OPENED = 1,
};

enum TAOS_CHIP_WORKING_STATUS {
	TAOS_CHIP_UNKNOWN = 0,
	TAOS_CHIP_WORKING = 1,
	TAOS_CHIP_SLEEP = 2
};

/* driver data */
struct taos_data {
	struct input_dev *prox_input_dev;
	struct input_dev *light_input_dev;
	struct i2c_client *client;
	struct work_struct work_prox;  /* for proximity sensor */
	struct work_struct work_light; /* for light_sensor     */
	struct work_struct work_ptime; /* for proximity reset    */
	struct class *lightsensor_class;
	struct class *proximity_class;
	struct hrtimer timer;
	struct hrtimer ptimer;
	struct workqueue_struct *taos_wq;
	struct workqueue_struct *taos_test_wq;
	struct wake_lock prx_wake_lock;
	struct taos_platform_data *pdata;
	struct mutex power_lock;
	struct mutex prox_mutex;
	struct i2c_client *opt_i2c_client;

	enum TAOS_ALS_FOPS_STATUS taos_als_status;
	enum TAOS_PRX_FOPS_STATUS taos_prx_status;
	enum TAOS_CHIP_WORKING_STATUS taos_chip_status;

	ktime_t light_polling_time;
	ktime_t prox_polling_time;
	bool light_enable;
	bool proximity_enable;
	short proximity_value;
	short  isTaosSensor;
	int irdata;		/* Ch[1] */
	int cleardata;	/* Ch[0] */
	u16 chipID;
	int	irq;
	int delay;
	int avg[3];
	/* Auto Calibration */
	u8 offset_value;
	u8 initial_offset;
	int cal_result;
	int threshold_high;
	int threshold_low;
};

/* platform data */
struct taos_platform_data {
	int (*power_en)(int);
	int (*power_led)(int);
	unsigned int wakeup;
};

/* prototype */
int taos_get_lux(struct taos_data *taos);
void taos_on(struct taos_data *taos, int type);
void taos_off(struct taos_data *taos, int type);

#endif
