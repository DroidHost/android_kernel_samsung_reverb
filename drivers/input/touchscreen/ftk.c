
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/serio.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/i2c/ftk_patch.h>
#include "ftk_fw.h"
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/power_supply.h>
#include <linux/firmware.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/cpufreq.h>

#ifdef CONFIG_MULTI_TOUCH_PROTOCOL_TYPE_B
#include <linux/input/mt.h>
#endif

#define SEC_TSP_DEBUG
#ifdef CONFIG_SEC_DVFS
#define TOUCH_BOOSTER			1
#define TOUCH_BOOSTER_OFF_TIME	100
#define TOUCH_BOOSTER_CHG_TIME	200
#endif

#define FIRMWARE_IC					"ftk_ic"
#define FIRMWARE_NAME				"/data/ftk.fw"
#define FIRMWARE_SDFILE				"/mnt/extSdCard/ftk.fw"

#define VERSION							3
#define FTK_TS_DRV_NAME					"ftk"
#define FTK_TS_DRV_VERSION				"1008"

#define X_AXIS_MAX						480
#define X_AXIS_MIN						0
#define Y_AXIS_MAX						800
#define Y_AXIS_MIN						0
#define PRESSURE_MIN					0
#define PRESSURE_MAX					127
#define P70_PATCH_ADDR_START			0x00420000
#define FINGER_MAX						10
#define AREA_MIN						PRESSURE_MIN
#define AREA_MAX						PRESSURE_MAX

#define EVENTID_NO_EVENT				0x00
#define EVENTID_ENTER_POINTER			0x03
#define EVENTID_LEAVE_POINTER			0x04
#define EVENTID_MOTION_POINTER			0x05
#define EVENTID_CONTROLLER_READY		0x10
#define EVENTID_BUTTON_STATUS			0x11
#define EVENTID_ERROR					0x0F
#define EVENTID_DUMMY					0x09

#define INT_ENABLE						0x40
#define INT_DISABLE						0x00
#define HANDSHAKE_START			1
#define HANDSHAKE_END				0

#define SLEEPIN					0x80
#define SLEEPOUT				0x81
#define SENSEOFF				0x82
#define SENSEON					0x83

#define FLUSHBUFFER				0x88
#define SOFTRESET				0x9E
#define FORCECALIBRATION		0xA0

#define FILTERED_DATA_ADDR_START	0xB10FF4
#define BASELINE_DATA_ADDR_START	0xB11C14

#define START_ROW		1
#define START_COL		13
#define MAX_ROW		24
#define MAX_COL			32
#define EFFECTIVE_NUM_OF_COL				13
#define EFFECTIVE_NUM_OF_ROW				20

#define ADJACENCY_ERROR_PERCENT 30

#ifdef SEC_TSP_FACTORY_TEST
enum {
	BUILT_IN = 0,
	UMS,
};

#define TSP_BUF_SIZE 1024
#define TSP_CMD_STR_LEN 32
#define TSP_CMD_RESULT_STR_LEN 512
#define TSP_CMD_PARAM_NUM 8

static void fw_update(void *device_data);
static void get_fw_ver_bin(void *device_data);
static void get_fw_ver_ic(void *device_data);
static void get_config_ver(void *device_data);
static void get_threshold(void *device_data);
static void module_off_master(void *device_data);
static void module_on_master(void *device_data);
static void get_chip_vendor(void *device_data);
static void get_chip_name(void *device_data);
static void get_x_num(void *device_data);
static void get_y_num(void *device_data);
static void run_reference_read(void *device_data);
static void get_reference(void *device_data);
static void run_raw_read(void *device_data);
static void get_raw(void *device_data);
static void run_delta_read(void *device_data);
static void get_delta(void *device_data);
static void run_selftest(void *device_data);
static void not_support_cmd(void *device_data);

static ssize_t store_cmd(struct device *dev, struct device_attribute *devattr,
		const char *buf, size_t count);
static ssize_t show_cmd_status(struct device *dev,
		struct device_attribute *devattr, char *buf);
static ssize_t show_cmd_result(struct device *dev, struct device_attribute
		*devattr, char *buf);

#define TSP_CMD(name, func)	.cmd_name = name, .cmd_func = func

struct tsp_cmd {
	struct list_head list;
	const char *cmd_name;
	void (*cmd_func)(void *device_data);
};

struct tsp_cmd tsp_cmds[] = {
	{TSP_CMD("fw_update", fw_update),},
	{TSP_CMD("get_fw_ver_bin", get_fw_ver_bin),},
	{TSP_CMD("get_fw_ver_ic", get_fw_ver_ic),},
	{TSP_CMD("get_config_ver", get_config_ver),},
	{TSP_CMD("get_threshold", get_threshold),},
	{TSP_CMD("module_off_master", module_off_master),},
	{TSP_CMD("module_on_master", module_on_master),},
	{TSP_CMD("module_off_slave", not_support_cmd),},
	{TSP_CMD("module_on_slave", not_support_cmd),},
	{TSP_CMD("get_chip_vendor", get_chip_vendor),},
	{TSP_CMD("get_chip_name", get_chip_name),},
	{TSP_CMD("get_x_num", get_x_num),},
	{TSP_CMD("get_y_num", get_y_num),},
	{TSP_CMD("run_reference_read", run_reference_read),},
	{TSP_CMD("get_reference", get_reference),},
	{TSP_CMD("run_raw_read", run_raw_read),},
	{TSP_CMD("get_raw", get_raw),},
	{TSP_CMD("run_delta_read", run_delta_read),},
	{TSP_CMD("get_delta", get_delta),},
	{TSP_CMD("run_selftest", run_selftest),},
	{TSP_CMD("not_support_cmd", not_support_cmd),},
};

static DEVICE_ATTR(cmd, S_IWUSR | S_IWGRP, NULL, store_cmd);
static DEVICE_ATTR(cmd_status, S_IRUGO, show_cmd_status, NULL);
static DEVICE_ATTR(cmd_result, S_IRUGO, show_cmd_result, NULL);

static struct attribute *sec_touch_facotry_attributes[] = {
	&dev_attr_cmd.attr,
	&dev_attr_cmd_status.attr,
	&dev_attr_cmd_result.attr,
	NULL,
};

static struct attribute_group sec_touch_factory_attr_group = {
	.attrs = sec_touch_facotry_attributes,
};
#endif

static unsigned char firmware_version;
static unsigned char firmware_month;
static unsigned char firmware_day;

static struct i2c_driver stm_ts_driver;
static struct workqueue_struct *stmtouch_wq;
static struct workqueue_struct *stmtouch_wq_charger;
static struct workqueue_struct *stmtouch_wq_firmware;

#ifndef CONFIG_MULTI_TOUCH_PROTOCOL_TYPE_B
static int cor_xyz[FINGER_MAX][3];
#endif
static unsigned char ID_Indx[FINGER_MAX];

struct ftk_charger_t {
	unsigned char reg;
	unsigned char normal;
	unsigned char charger;
};

static int ftk_charger_previous;
static int ftk_charger_cnt;
static int ftk_firmware_cnt;
static struct ftk_charger_t *pftk_charger;
static char firmwarefile;

struct ftk_ts_info {
	struct device *dev;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct hrtimer timer;
	struct hrtimer timer_charger;
	struct timer_list timer_firmware;
	struct work_struct work;
	struct work_struct work_charger;
	struct work_struct work_firmware;
	int irq;
	int (*power) (int on);

#if TOUCH_BOOSTER
	struct delayed_work work_dvfs_off;
	struct delayed_work work_dvfs_chg;
	bool	dvfs_lock_status;
	struct mutex dvfs_lock;
#endif

	struct mutex lock;
	bool enabled;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif

#ifdef SEC_TSP_FACTORY_TEST
	struct device *fac_dev_ts;
	struct list_head cmd_list_head;
	u8 cmd_state;
	char cmd[TSP_CMD_STR_LEN];
	int cmd_param[TSP_CMD_PARAM_NUM];
	char cmd_result[TSP_CMD_RESULT_STR_LEN];
	struct mutex cmd_lock;
	bool cmd_is_running;
	unsigned short baseline[EFFECTIVE_NUM_OF_COL*EFFECTIVE_NUM_OF_ROW];
	unsigned short filter[EFFECTIVE_NUM_OF_COL*EFFECTIVE_NUM_OF_ROW];
	short delta[EFFECTIVE_NUM_OF_COL*EFFECTIVE_NUM_OF_ROW];
#endif
};

#if TOUCH_BOOSTER
static void change_dvfs_lock(struct work_struct *work)
{
	struct ftk_ts_info *info = container_of(work,
				struct ftk_ts_info, work_dvfs_chg.work);
	int ret;
	mutex_lock(&info->dvfs_lock);
	ret = set_freq_limit(DVFS_TOUCH_ID, 806400);
	mutex_unlock(&info->dvfs_lock);

	if (ret < 0)
		pr_err("%s: 1booster stop failed(%d)\n",\
					__func__, __LINE__);
	else
		pr_info("[TSP] %s", __func__);
}

static void set_dvfs_off(struct work_struct *work)
{
	struct ftk_ts_info *info = container_of(work,
				struct ftk_ts_info, work_dvfs_off.work);
	mutex_lock(&info->dvfs_lock);
	set_freq_limit(DVFS_TOUCH_ID, -1);
	info->dvfs_lock_status = false;
	mutex_unlock(&info->dvfs_lock);

	pr_info("[TSP] DVFS Off!");
}

static void set_dvfs_lock(struct ftk_ts_info *info, uint32_t on)
{
	int ret = 0;

	mutex_lock(&info->dvfs_lock);
	if (on == 0) {
		if (info->dvfs_lock_status) {
			cancel_delayed_work(&info->work_dvfs_chg);
			schedule_delayed_work(&info->work_dvfs_off,
				msecs_to_jiffies(TOUCH_BOOSTER_OFF_TIME));
		}
	} else if (on == 1) {
		cancel_delayed_work(&info->work_dvfs_off);
		if (!info->dvfs_lock_status) {
			ret = set_freq_limit(DVFS_TOUCH_ID, 806400);
			if (ret < 0)
				pr_err("%s: cpu lock failed(%d)\n",\
							__func__, ret);

			info->dvfs_lock_status = true;
			pr_info("[TSP] DVFS On!");
		}
	} else if (on == 2) {
		cancel_delayed_work(&info->work_dvfs_off);
		schedule_work(&info->work_dvfs_off.work);
	}
	mutex_unlock(&info->dvfs_lock);
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void stm_ts_early_suspend(struct early_suspend *h);
static void stm_ts_late_resume(struct early_suspend *h);
#endif

static int ftk_write_reg(struct ftk_ts_info *info,
	unsigned char *reg, u16 num_com)
{
	struct i2c_msg xfer_msg[2];

	xfer_msg[0].addr = info->client->addr;
	xfer_msg[0].len = num_com;
	xfer_msg[0].flags = 0;
	xfer_msg[0].buf = reg;

	return i2c_transfer(info->client->adapter, xfer_msg, 1);
}

static int ftk_read_reg(struct ftk_ts_info *info, unsigned char *reg, int cnum,
			u8 *buf, int num)
{
	struct i2c_msg xfer_msg[2];

	xfer_msg[0].addr = info->client->addr;
	xfer_msg[0].len = cnum;
	xfer_msg[0].flags = 0;
	xfer_msg[0].buf = reg;

	xfer_msg[1].addr = info->client->addr;
	xfer_msg[1].len = num;
	xfer_msg[1].flags = I2C_M_RD;
	xfer_msg[1].buf = buf;

	return i2c_transfer(info->client->adapter, xfer_msg, 2);
}

static void ftk_delay(unsigned int ms)
{
	if (ms < 20)
		mdelay(ms);
	else
		msleep(ms);
}

static void ftk_command(struct ftk_ts_info *info, unsigned char cmd)
{
	unsigned char regAdd = 0;

	regAdd = cmd;
	ftk_write_reg(info, &regAdd, 1);
	if (cmd == SOFTRESET)
		ftk_delay(30);
	else
		ftk_delay(5);
}

static void ftk_softreset(struct ftk_ts_info *info)
{
	u8 regAdd[7] = {0xB3, 0xFF, 0xFF, 0xB1, 0xFC, 0x34, 0x01};

	printk(KERN_ERR "FTK Softreset\n");
	ftk_write_reg(info, &regAdd[0], 3);
	ftk_write_reg(info, &regAdd[3], 4);

	ftk_delay(30);
}

static void ftk_interrupt(struct ftk_ts_info *info, int enable)
{
	unsigned char regAdd[3] = {0};

	regAdd[0] = 0xB0;
	regAdd[1] = 0x06;
	regAdd[2] = enable;

	ftk_write_reg(info, &regAdd[0], 3);
	ftk_delay(5);
}

static void ftk_write_signature(struct ftk_ts_info *info, char clear)
{
	unsigned char pSigWrite[7] = {0xB1, 0x00, 0x00, 0x4A, 0x41, 0x4E, 0x47};
	unsigned char pSigClear[7] = {0xB1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	unsigned char *pSig = NULL;
	unsigned char regAdd[3] = {0};
	unsigned int writeAddr = P70_PATCH_ADDR_START + 0x4000 - 4;

	ftk_command(info, FLUSHBUFFER);

	regAdd[0] = 0xB3;
	regAdd[1] = (writeAddr >> 24) & 0xFF;
	regAdd[2] = (writeAddr >> 16) & 0xFF;
	ftk_write_reg(info, &regAdd[0], 3);

	if (clear)
		pSig = pSigClear;
	else
		pSig = pSigWrite;

	pSig[1] = (writeAddr >> 8) & 0xFF;
	pSig[2] = writeAddr & 0xFF;
	ftk_write_reg(info, &pSig[0], 7);
}

static u8 ftk_read_signature(struct ftk_ts_info *info)
{
	unsigned char pSignature[4] = {0x4A, 0x41, 0x4E, 0x47};
	unsigned int writeAddr = P70_PATCH_ADDR_START + 0x4000 - 4;
	unsigned char regAdd[3] = {0};
	unsigned char data[8] = {0};

	regAdd[0] = 0xB3;
	regAdd[1] = (writeAddr >> 24) & 0xFF;
	regAdd[2] = (writeAddr >> 16) & 0xFF;
	ftk_write_reg(info, &regAdd[0], 3);

	regAdd[0] = 0xB1;
	regAdd[1] = (writeAddr >> 8) & 0xFF;
	regAdd[2] = writeAddr & 0xFF;
	ftk_read_reg(info, &regAdd[0], 3, data, 5);

	if (strncmp(pSignature, &data[1], 4) == 0)
		return 1;

	return 0;
}

static u8 firmware_load_patch(struct ftk_ts_info *info,
	const struct firmware *firmware)
{

	u32 writeAddr, j = 0, i = 0;
	u16 patch_length = 0;
	u8 byteWork1[256 + 3] = { 0 };
	u8 regAdd[3] = { 0 };

	patch_length = firmware->data[VERSION + 0] * 256 +
		firmware->data[VERSION + 1];

	while (j < patch_length) {
		writeAddr = P70_PATCH_ADDR_START + j;

		regAdd[0] = 0xB3;
		regAdd[1] = (writeAddr >> 24) & 0xFF;
		regAdd[2] = (writeAddr >> 16) & 0xFF;
		ftk_write_reg(info, &regAdd[0], 3);

		byteWork1[0] = 0xB1;
		byteWork1[1] = (writeAddr >> 8) & 0xFF;
		byteWork1[2] = writeAddr & 0xFF;

		i = 0;
		while ((j < patch_length) && (i < 256)) {
			byteWork1[i + 3] = firmware->data[VERSION + j + 2];
			i++;
			j++;
		}
		ftk_write_reg(info, &byteWork1[0], i + 3);

	}

	return 0;
}

static u8 firmware_load_config(struct ftk_ts_info *info,
	const struct firmware *firmware)
{
	u16 one_group_length = 0;
	u16 patch_length = 0;
	u16 config_length = 0;
	u16 i = 0;
	u8 *pdata = NULL;
	u8 ret = 0;
	unsigned char regAdd[2] = {0};

	patch_length = firmware->data[VERSION + 0] * 256 +
		firmware->data[VERSION + 1];
	config_length =
		firmware->data[VERSION + patch_length + 2] * 256 +
		firmware->data[VERSION + patch_length + 2 + 1];
	pdata = (u8 *)&firmware->data[VERSION + patch_length + 4];

	while (i < config_length) {
		one_group_length = pdata[i] * 256 + pdata[i + 1];

		if ((pdata[i + 2] == 0xFF) && (pdata[i + 3] == 0xFF))
			ftk_delay(pdata[i + 4]);
		else {
			if (pdata[i + 2] == 0xB0 && pdata[i + 3] == 0x04)
				ftk_write_signature(info, 0);

			ftk_write_reg(info,
				&(pdata[i + 2]), one_group_length);

			if (pdata[i + 2] == 0xB0 && pdata[i + 3] == 0x04)
				ftk_delay(30);

			if (pdata[i + 2] == 0xB1 && pdata[i + 3] == 0xFC)
				ftk_delay(30);

			if (pdata[i + 3] == 0x00) {
				switch (pdata[i + 2]) {
				case SOFTRESET:
					ftk_delay(20);
				case 0x00:
				case SLEEPIN:
				case SLEEPOUT:
				case 0x82:
				case 0x83:
				case FLUSHBUFFER:
				case 0x89:
				case FORCECALIBRATION:
				case 0xA3:
				case 0xA4:
					ftk_delay(30);
					break;
				}
			}
		}

		i += (one_group_length + 2);
	}

	regAdd[0] = 0xB0;
	regAdd[1] = 0x03;
	ftk_read_reg(info, &regAdd[0], 2, &ret, 1);
	printk(KERN_ERR "FTK Patch Version %d\n", ret);

	return ret;
}

static u8 firmware_load_charger(struct ftk_ts_info *info,
	const struct firmware *firmware)
{
	u16 one_group_length = 0;
	u16 patch_length = 0;
	u16 config_length = 0;
	u16 charger_length = 0;
	u16 charger_cnt = 0;
	u16 i = 0;
	u8 *pdata;

	ftk_charger_cnt = 0;
	ftk_charger_previous = POWER_SUPPLY_STATUS_UNKNOWN;

	patch_length = firmware->data[VERSION + 0] * 256 +
		firmware->data[VERSION + 1];
	config_length =
		firmware->data[VERSION + patch_length + 2] * 256 +
		firmware->data[VERSION + patch_length + 2 + 1];

	if (patch_length + config_length + VERSION + 4 == firmware->size) {
		printk(KERN_ERR "FTK no charger info in fw\n");
		return 0;
	}

	charger_length =
		firmware->data[VERSION + patch_length + config_length + 4]
			* 256 +
		firmware->data[VERSION + patch_length + config_length + 4 + 1];

	pdata = (u8 *)&firmware->data[
		VERSION + patch_length + config_length + 6];

	if (charger_length == 0)
		ftk_charger_cnt = 0;
	else
		ftk_charger_cnt = charger_length / 6;

	if (ftk_charger_cnt == 0) {
		printk(KERN_ERR "FTK ftk_charger_cnt = 0\n");
		return 0;
	}

	pftk_charger = kzalloc(sizeof(struct ftk_charger_t) *
		ftk_charger_cnt, GFP_KERNEL);
	if (!pftk_charger) {
		printk(KERN_ERR "FTK pftk_charger = ENOMEM!\n");
		return 1; /* ENOMEM */
	}

	while (i < charger_length) {
		one_group_length = pdata[i] * 256 + pdata[i + 1];
		if (one_group_length != 4 || pdata[i + 2] != 0xB0) {
			printk(KERN_ERR "FTK ftk_charger mismatch\n");
			break;
		}
		pftk_charger[charger_cnt].reg = pdata[i + 3];
		pftk_charger[charger_cnt].normal = pdata[i + 4];
		pftk_charger[charger_cnt].charger = pdata[i + 5];
		/*
		printk(KERN_ERR "FTK ftk_charger B0%02X %02X %02X\n",
			pftk_charger[charger_cnt].reg,
			pftk_charger[charger_cnt].normal,
			pftk_charger[charger_cnt].charger);
		*/
		charger_cnt++;

		i += (one_group_length + 2);
	}

	if (ftk_charger_cnt != charger_cnt) {
		ftk_charger_cnt = 0;
		printk(KERN_ERR "FTK ftk_charger count mismatch\n");
	}

	return 0;
}

static int firmware_verify(const struct firmware *firmware)
{
	u16 firmware_length = 0;
	u16 patch_length = 0;
	u16 config_length = 0;
	u16 charger_length = 0;

	firmware_length = firmware->size;
	patch_length =
		firmware->data[VERSION + 0] * 256 + firmware->data[VERSION + 1];
	config_length =
		firmware->data[VERSION + patch_length + 2] * 256 +
		firmware->data[VERSION + patch_length + 2 + 1];

	if (patch_length + config_length + VERSION + 4 == firmware_length)
		return 0;

	charger_length =
		firmware->data[VERSION + patch_length + config_length + 4] *
		256 +
		firmware->data[VERSION + patch_length + config_length + 4 + 1];

	if (firmware_length ==
		patch_length + config_length + charger_length + VERSION + 6)
		return 0;
	else
		return 1;
}

static inline int firmware_request(const struct firmware **firmware_p,
					const char *name, struct device *device)
{
	struct firmware *firmware = NULL;
	struct file *filp = NULL;
	mm_segment_t old_fs = {0};
	struct inode *inode = NULL;
	long ret = 0;

	*firmware_p = firmware = kzalloc(sizeof(*firmware), GFP_KERNEL);
	if (!firmware) {
		printk(KERN_ERR "FTK request_firmwarefile alloc fail\n");
		goto ErrorExit;
	}

	if (strncmp(name, FIRMWARE_IC, sizeof(FIRMWARE_NAME)) == 0)
		goto ErrorExit;

	firmware->size = 0;
	firmware->data = kzalloc(32*1024, GFP_KERNEL);
	if (!firmware->data) {
		printk(KERN_ERR "FTK request_firmwarefile alloc fail 2\n");
		goto ErrorExit;
	}

	old_fs = get_fs();
	set_fs(get_ds());

	filp = filp_open(name, O_RDONLY, S_IRUSR);
	ret = IS_ERR(filp);
	if (ret) {
		printk(KERN_ERR "FTK firmware %ld\n", ret);
		goto ErrorExit;
	}

	inode = filp->f_path.dentry->d_inode;
	firmware->size = i_size_read(inode->i_mapping->host);
	if (filp->f_op->read(filp, (char *)firmware->data,
		firmware->size, &filp->f_pos) != firmware->size) {
		kfree(firmware->data);
		filp_close(filp, NULL);
		set_fs(old_fs);
		goto ErrorExit;
	}

	filp_close(filp, NULL);
	set_fs(old_fs);

	firmwarefile = 1;

	return 0;

ErrorExit:
	firmware->size = FirmwareSize;
	firmware->data = pFirmwareData;
	firmwarefile = 0;

	return 1;
}

static void firmware_release(const struct firmware *fw)
{
	if (firmwarefile == 1) {
		kfree(fw->data);
		kfree(fw);
	}
}

static int firmware_readdata(unsigned char *filename, unsigned char *data)
{
	int rc = 1;
	long ret = 0;
	struct file *filp = NULL;
	mm_segment_t old_fs = {0};

	old_fs = get_fs();
	set_fs(get_ds());

	filp = filp_open(filename, O_RDONLY, S_IRUSR);
	ret = IS_ERR(filp);

	if (ret == 0) {
		firmwarefile = 1;
		filp->f_op->read(filp, data, 3, &filp->f_pos);
		filp_close(filp, NULL);

		if (data[0] > firmware_version) {
			printk(KERN_ERR "FTK new firmware found\n");
			rc = 0;
		} else {
			printk(KERN_ERR "FTK firmware is same or old\n");
			rc = 1;
		}
	}

	set_fs(old_fs);

	return rc;
}

static u8 firmware_load(struct ftk_ts_info *info, unsigned char *name)
{
	const struct firmware *firmware;
	u8 rc = 0;

	ftk_softreset(info);

	rc = firmware_request(&firmware, name, info->dev);
	if (rc == 0)
		printk(KERN_ERR "FTK firmware name : %s\n", name);

	if (firmware_verify(firmware) == 0) {
		firmware_version = firmware->data[0];
		firmware_month = firmware->data[1] & 0x0F;
		firmware_day = firmware->data[2];

		printk(KERN_ERR "FTK firmware Version : %02d\n",
			firmware_version);
		printk(KERN_ERR "FTK firmware Date : %02d%02d%02d\n",
			firmware->data[1] >> 4, firmware_month, firmware_day);

		firmware_load_patch(info, firmware);
		rc = firmware_load_config(info, firmware);
		firmware_load_charger(info, firmware);
	} else
		printk(KERN_ERR "FTK firmware_verify fail\n");

	firmware_release(firmware);

	touch_is_pressed = 0;
	#if TOUCH_BOOSTER
	set_dvfs_lock(info, 2);
	pr_info("[TSP] dvfs_lock free.\n ");
	#endif

	return rc;
}

#ifdef SEC_TSP_FACTORY_TEST
static void firmware_update(char *srcname, char *dstname)
{
	struct file *filp = NULL;
	mm_segment_t old_fs = {0};
	struct inode *inode = NULL;
	long ret = 0;
	size_t size = 0;
	size_t writelen = 0;
	u8 *data = NULL;

	old_fs = get_fs();
	set_fs(get_ds());

	filp = filp_open(srcname, O_RDONLY, S_IRUSR);
	ret = IS_ERR(filp);
	if (ret) {
		printk(KERN_ERR "FTK filp_open(%s) error\n", srcname);
		goto ErrorExit;
	}

	inode = filp->f_path.dentry->d_inode;
	size = i_size_read(inode->i_mapping->host);
	data = kzalloc(size, GFP_KERNEL);
	if (data == NULL) {
		printk(KERN_ERR "FTK malloc error\n");
		goto ErrorExit;
	}

	if (filp->f_op->read(filp, data,
		size, &filp->f_pos) != size) {
		kfree(data);
		filp_close(filp, NULL);
		set_fs(old_fs);
		printk(KERN_ERR "FTK read error\n");
		goto ErrorExit;
	}

	filp_close(filp, NULL);

	filp = filp_open(dstname, O_CREAT | O_WRONLY |
		O_TRUNC | S_IRUGO, S_IRUGO | S_IWUGO);
	ret = IS_ERR(filp);
	if (ret) {
		printk(KERN_ERR "FTK filp_open(%s) error\n", dstname);
		goto ErrorExit;
	}

	writelen = filp->f_op->write(filp, data, size, &filp->f_pos);

	filp_close(filp, NULL);

ErrorExit:
	set_fs(old_fs);
	kfree(data);
}
#endif

static int init_ftk(struct ftk_ts_info *info)
{
	u8 val[8];
	u8 regAdd[7];
	int rc;

	regAdd[0] = 0xB0;
	regAdd[1] = 0x00;
	rc = ftk_read_reg(info, regAdd, 2, val, 3);

	if (rc < 0) {
		printk(KERN_ERR "FTK i2c_transfer failed\n");
		return 1;
	}

	printk(KERN_ERR "FTK Chip ID = %02x %02x %02x\n",
		val[0], val[1], val[2]);

	if (val[0] != 0x28 &&  val[1] != 0x55)
		return 1;

	ftk_write_signature(info, 0);
	firmware_load(info, FIRMWARE_IC);
	ftk_write_signature(info, 1);

	regAdd[0] = 0xB3;
	regAdd[1] = 0xFF;
	regAdd[2] = 0xFF;
	ftk_write_reg(info, &regAdd[0], 3);

	printk(KERN_ERR "FTK Initialised\n");

	return 0;
}

static enum hrtimer_restart st_ts_timer_func(struct hrtimer *timer)
{
	struct ftk_ts_info *info =
		container_of(timer, struct ftk_ts_info, timer);
	queue_work(stmtouch_wq, &info->work);
	return 0; /* HRTIMER_NORESTART */
}

static irqreturn_t ts_interrupt(int irq, void *handle)
{
	struct ftk_ts_info *info = handle;
	disable_irq_nosync(info->client->irq);
	queue_work(stmtouch_wq, &info->work);
	return 1; /* IRQ_HANDLED */
}

static void ts_controller_ready(struct ftk_ts_info *info, unsigned char data[])
{
	u8 TouchID = 0;

	for (TouchID = 0; TouchID < FINGER_MAX; TouchID++)
		ID_Indx[TouchID] = 0;

	printk(KERN_ERR "FTK ts_controller_ready : %02X\n", data[7]);

	#ifndef CONFIG_MULTI_TOUCH_PROTOCOL_TYPE_B
	input_mt_sync(info->input_dev);
	#endif

	input_sync(info->input_dev);

	ftk_command(info, FLUSHBUFFER);

	if (ftk_read_signature(info) == 0) {
		printk(KERN_ERR "FTK Restart\n");
		firmware_load(info, FIRMWARE_IC);
		ftk_write_signature(info, 1);
	}
}

static void ts_error_handler(struct ftk_ts_info *info, unsigned char data[])
{
	u8 regAdd[7] = {0xB3, 0xFF, 0xFF, 0xB1, 0xFC, 0x34, 0x01};

	printk(KERN_ERR "FTK ts_error_handler\n");
	printk(KERN_ERR "%02X %02X %02X %02X %02X %02X %02X %02X\n",
		data[0], data[1], data[2], data[3],
		data[4], data[5], data[6], data[7]);

	if (data[1] == 0x1 || data[1] == 0x2 ||
	   (data[1] == 0x3 && data[6] != 0x1)) {
		printk(KERN_ERR "FTK p70 restart\n");
		ftk_write_reg(info, &regAdd[0], 3);
		ftk_write_reg(info, &regAdd[3], 4);
	}
}

#ifdef CONFIG_MULTI_TOUCH_PROTOCOL_TYPE_B
static u8 decode_data_packet_type_b(struct ftk_ts_info *info,
				unsigned char data[], unsigned char LeftEvent)
{
	u8 EventNum = 0;
	u8 NumTouches = 0;
	u8 TouchID = 0, EventID = 0;
	u8 LastLeftEvent = 0;
	int x = 0, y = 0, z = 0;

	for (EventNum = 0; EventNum < LeftEvent; EventNum++) {

		LastLeftEvent = data[7 + EventNum * 8] & 0x0F;
		NumTouches = (data[1 + EventNum * 8] & 0xF0) >> 4;
		TouchID = data[1 + EventNum * 8] & 0x0F;
		EventID = data[EventNum * 8] & 0xFF;

		switch (EventID) {
		case EVENTID_ENTER_POINTER:
		case EVENTID_MOTION_POINTER:
			ID_Indx[TouchID] = 1;
			x = ((data[4 + EventNum * 8] & 0xF0) >> 4) |
				((data[2 + EventNum * 8]) << 4);
			y = ((data[4 + EventNum * 8] & 0x0F) |
				((data[3 + EventNum * 8]) << 4));
			z = data[5 + EventNum * 8];

			if (x == X_AXIS_MAX)
				x--;
			if (y == Y_AXIS_MAX)
				y--;

			input_mt_slot(info->input_dev, TouchID);
			input_report_abs(
				info->input_dev, ABS_MT_TRACKING_ID, TouchID);
			input_report_abs(info->input_dev,
				ABS_MT_POSITION_X, x);
			input_report_abs(info->input_dev,
				ABS_MT_POSITION_Y, y);
			input_report_abs(info->input_dev,
				ABS_MT_TOUCH_MAJOR, z);
			input_report_abs(info->input_dev,
				ABS_MT_PRESSURE, z);

#if defined(SEC_TSP_DEBUG2)
			printk(KERN_ERR "FTK ID[%d] X[%3d] Y[%3d] Z[%d]\n",
			TouchID, x, y, z);
			break;
#else
		if (EventID == 0x03)
			printk(KERN_ERR "FTK ID[%d] down\n",
			TouchID);
			break;
#endif

		case EVENTID_LEAVE_POINTER:
			ID_Indx[TouchID] = 0;
			input_mt_slot(info->input_dev, TouchID);
			input_report_abs(
				info->input_dev, ABS_MT_TRACKING_ID, -1);
			printk(KERN_ERR "FTK ID[%d] up\n",
			TouchID);
			break;

		case EVENTID_CONTROLLER_READY:
			ts_controller_ready(info, &data[EventNum * 8]);
			break;

		case EVENTID_ERROR:
			ts_error_handler(info, &data[EventNum * 8]);
			break;
		}

	}

	touch_is_pressed = 0;

	for (TouchID = 0; TouchID < FINGER_MAX; TouchID++) {
		if (ID_Indx[TouchID])
			touch_is_pressed++;
	}

	/*printk(KERN_ERR "FTK touch_is_pressed = %d\n", touch_is_pressed);*/

	input_sync(info->input_dev);

	#if TOUCH_BOOSTER
	set_dvfs_lock(info, !!touch_is_pressed);
	#endif

	return LastLeftEvent;
}
#else
static u8 decode_data_packet_type_a(struct ftk_ts_info *info,
				unsigned char data[], unsigned char LeftEvent)
{
	u8 EventNum = 0;
	u8 NumTouches = 0;
	u8 TouchID = 0, EventID = 0;
	u8 LastLeftEvent = 0;

	for (EventNum = 0; EventNum < LeftEvent; EventNum++) {

		LastLeftEvent = data[7 + EventNum * 8] & 0x0F;
		NumTouches = (data[1 + EventNum * 8] & 0xF0) >> 4;
		TouchID = data[1 + EventNum * 8] & 0x0F;
		EventID = data[EventNum * 8] & 0xFF;

		switch (EventID) {
		case EVENTID_ENTER_POINTER:
		case EVENTID_MOTION_POINTER:
			ID_Indx[TouchID] = 1;
		case EVENTID_LEAVE_POINTER:
			cor_xyz[TouchID][0] =
				((data[4 + EventNum * 8] & 0xF0) >> 4) |
				((data[2 + EventNum * 8]) << 4);
			cor_xyz[TouchID][1] =
				((data[4 + EventNum * 8] & 0x0F) |
				((data[3 + EventNum * 8]) << 4));
			cor_xyz[TouchID][2] = data[5 + EventNum * 8];

			if (cor_xyz[TouchID][0] == X_AXIS_MAX)
				cor_xyz[TouchID][0]--;
			if (cor_xyz[TouchID][1] == Y_AXIS_MAX)
				cor_xyz[TouchID][1]--;

			printk(KERN_ERR "FTK ID[%d] X[%3d] Y[%3d] Z[%3d]\n",
				TouchID, cor_xyz[TouchID][0],
				cor_xyz[TouchID][1], cor_xyz[TouchID][2]);

			break;

		case EVENTID_CONTROLLER_READY:
			ts_controller_ready(info, &data[EventNum * 8]);
			return 0;
		}

		if (EventID == EVENTID_LEAVE_POINTER) {
			ID_Indx[TouchID] = 0;
			input_report_abs(info->input_dev, ABS_MT_TRACKING_ID,
					TouchID);
			input_report_abs(info->input_dev, ABS_MT_POSITION_X,
					cor_xyz[TouchID][0]);
			input_report_abs(info->input_dev, ABS_MT_POSITION_Y,
					cor_xyz[TouchID][1]);
			input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR,
					cor_xyz[TouchID][2]);
			input_report_key(info->input_dev, BTN_TOUCH, 1);
			input_mt_sync(info->input_dev);
		}
	}

	touch_is_pressed = 0;

	for (TouchID = 0; TouchID < FINGER_MAX; TouchID++) {
		if (ID_Indx[TouchID]) {
			touch_is_pressed++;
			input_report_abs(info->input_dev, ABS_MT_TRACKING_ID,
					TouchID);
			input_report_abs(info->input_dev, ABS_MT_POSITION_X,
					cor_xyz[TouchID][0]);
			input_report_abs(info->input_dev, ABS_MT_POSITION_Y,
					cor_xyz[TouchID][1]);
			input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR,
					cor_xyz[TouchID][2]);
			input_report_key(info->input_dev, BTN_TOUCH, 1);
			input_mt_sync(info->input_dev);
		}
	}

	if (touch_is_pressed == 0) {
		input_report_key(info->input_dev, BTN_TOUCH, 0);
		input_mt_sync(info->input_dev);
	}

	input_sync(info->input_dev);

	return LastLeftEvent;
}
#endif

static void ts_event_handler(struct work_struct *work)
{
	struct ftk_ts_info *info = container_of(work, struct ftk_ts_info, work);

	u8 data[256];
	int rc;
	u8 status;
	u8 regAdd;
	u8 FirstLeftEvent = 0;
	u8 repeat_flag = 0;

	data[0] = 0xB0;
	data[1] = 0x07;
	rc = ftk_read_reg(info, &data[0], 2, &status, 1);

	if (status & 0x40) {
		do {
			memset(data, 0x0, 8);
			regAdd = 0x85;
			rc = ftk_read_reg(info, &regAdd, 1, data, 8);

			FirstLeftEvent = 0;
			#ifdef CONFIG_MULTI_TOUCH_PROTOCOL_TYPE_B
			FirstLeftEvent = decode_data_packet_type_b(
				info, data, 1);
			#else
			FirstLeftEvent = decode_data_packet_type_a(
				info, data, 1);
			#endif

			if (FirstLeftEvent > 0) {
				memset(data, 0x0, 8 * FirstLeftEvent);
				regAdd = 0x86;
				rc = ftk_read_reg(info, &regAdd, 1, data,
					8 * FirstLeftEvent);
				#ifdef CONFIG_MULTI_TOUCH_PROTOCOL_TYPE_B
				decode_data_packet_type_b(
					info, data, FirstLeftEvent);
				#else
				decode_data_packet_type_a(
					info, data, FirstLeftEvent);
				#endif
			}

			data[0] = 0xB0;
			data[1] = 0x07;
			rc = ftk_read_reg(info, &data[0], 2, &status, 1);

			if (status & 0x40)
				repeat_flag = 1;
			else
				repeat_flag = 0;
		} while (repeat_flag);
	} else
		ftk_command(info, FLUSHBUFFER);

	if (!info->irq)
		hrtimer_start(&info->timer,
			ktime_set(0, 10000000), HRTIMER_MODE_REL);
	else
		enable_irq(info->client->irq);
}

#ifdef FTK_USE_CHARGER_DETECTION
static enum hrtimer_restart ftk_charger_tmer_func(struct hrtimer *timer)
{
	struct ftk_ts_info *info =
	container_of(timer, struct ftk_ts_info, timer_charger);

	queue_work(stmtouch_wq_charger, &info->work_charger);
	return 0; /* HRTIMER_NORESTART */
}
#endif

static int ftk_charger_timer_start(struct ftk_ts_info *info)
{
	if (ftk_charger_cnt == 0)
		return 0;

	#ifdef FTK_USE_CHARGER_DETECTION
	hrtimer_start(&info->timer_charger, ktime_set(0, 500000000),
			HRTIMER_MODE_REL);
	/*printk(KERN_ERR "FTK Charger Timer Start %d\n", ftk_charger_cnt);*/
	#endif
	return 0;
}
static int ftk_charger_timer_stop(struct ftk_ts_info *info)
{
	int ret = 0;
	if (ftk_charger_cnt == 0)
		return ret;
	#ifdef FTK_USE_CHARGER_DETECTION
	printk(KERN_ERR "FTK Charger Timer Stop\n");
	hrtimer_cancel(&info->timer_charger);
	#endif
	return ret;
}

static void ts_charger_event_handler(struct work_struct *work_charger)
{
	struct ftk_ts_info *info = container_of(
		work_charger, struct ftk_ts_info, work_charger);

	#ifndef FTK_USE_CHARGER_DETECTION
	int ftk_charger_status = POWER_SUPPLY_STATUS_UNKNOWN;
	#endif
	unsigned char regAdd[3] = {0};
	int i = 0;

	if (ftk_charger_cnt == 0)
		return;

	if (ftk_charger_previous != ftk_charger_status) {
		ftk_charger_previous = ftk_charger_status;

		ftk_command(info, SENSEOFF);
		ftk_interrupt(info, INT_DISABLE);

		regAdd[0] = 0xB0;

		switch (ftk_charger_status) {
		case POWER_SUPPLY_STATUS_CHARGING:
		case POWER_SUPPLY_STATUS_FULL:
			printk(KERN_ERR "FTK Charger Pluged\n");
			for (i = 0; i < ftk_charger_cnt; i++) {
				regAdd[1] = pftk_charger[i].reg;
				regAdd[2] = pftk_charger[i].charger;
				ftk_write_reg(info, regAdd, 3);
			}
			break;

		case POWER_SUPPLY_STATUS_DISCHARGING:
		case POWER_SUPPLY_STATUS_NOT_CHARGING:
			printk(KERN_ERR "FTK Charger Unpluged\n");
			for (i = 0; i < ftk_charger_cnt; i++) {
				regAdd[1] = pftk_charger[i].reg;
				regAdd[2] = pftk_charger[i].normal;
				ftk_write_reg(info, regAdd, 3);
			}
			break;

		default:
			printk(KERN_ERR "FTK Charger status unknown\n");
			break;
		}

		printk(KERN_ERR "FTK Force Calibration\n");
		ftk_command(info, SENSEON);
		ftk_command(info, SLEEPIN);
		ftk_delay(300);
		ftk_command(info, FLUSHBUFFER);

		for (i = 0; i < FINGER_MAX; i++)
			ID_Indx[i] = 0;
		touch_is_pressed = 0;

#ifdef CONFIG_MULTI_TOUCH_PROTOCOL_TYPE_B
		for (i = 0; i < FINGER_MAX; i++) {
			input_mt_slot(info->input_dev, i);
			input_report_abs(info->input_dev,
				ABS_MT_TRACKING_ID, -1);
		}
#else /* TYPE A */
		input_mt_sync(info->input_dev);
#endif
		input_sync(info->input_dev);

		ftk_interrupt(info, INT_ENABLE);
	}

	if (hrtimer_active(&info->timer_charger))
		ftk_charger_timer_stop(info);
		ftk_charger_timer_start(info);
}

static void ftk_firmware_handler(struct work_struct *work_firmware)
{
	struct ftk_ts_info *info = container_of(
		work_firmware, struct ftk_ts_info, work_firmware);
	char data[4] = {0};
	int rc = 0;

	if (firmwarefile == 1)
		return;

	rc = firmware_readdata(FIRMWARE_NAME, data);

	if (rc == 0 && firmwarefile == 1) {
		ftk_charger_timer_stop(info);

		ftk_write_signature(info, 0);
		firmware_load(info, FIRMWARE_NAME);
		ftk_write_signature(info, 1);

		ftk_charger_timer_start(info);
	}

	return;
}

static void ftk_firmware_tmer_func(unsigned long arg)
{
	struct ftk_ts_info *info = (struct ftk_ts_info *)arg;
	int ret = 0;

	ret = queue_work(stmtouch_wq_firmware, &info->work_firmware);

	if (firmwarefile == 1 || ++ftk_firmware_cnt > 30)
		printk(KERN_ERR "FTK firmware Timer Done\n");
	else
		mod_timer(&info->timer_firmware, jiffies + HZ);
}

static int ftk_firmware_timer_start(struct ftk_ts_info *info)
{
	init_timer(&info->timer_firmware);
	info->timer_firmware.expires = jiffies + HZ;
	info->timer_firmware.function = ftk_firmware_tmer_func;
	info->timer_firmware.data = (ulong)info;
	add_timer(&info->timer_firmware);

	return 0;
}

static int ftk_firmware_timer_stop(struct ftk_ts_info *info)
{
	printk(KERN_ERR "FTK firmware Timer Stop\n");

	return del_timer(&info->timer_firmware);
}

static int stm_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *idp)
{
	struct ftk_ts_info *info = NULL;
	struct ftk_i2c_platform_data *pdata;
	static char ftk_ts_phys[64] = {0};
	int ret = 0;
	int err = 0;
#ifdef SEC_TSP_FACTORY_TEST
	int i = 0;
#endif

	printk(KERN_ERR "FTK system_rev = %d\n", system_rev);
	printk(KERN_ERR "FTK Driver [12%s] %s %s\n",
		FTK_TS_DRV_VERSION, __DATE__, __TIME__);

#ifdef CONFIG_MULTI_TOUCH_PROTOCOL_TYPE_B
	printk(KERN_ERR "FTK use Protocol Type B\n");
#else
	printk(KERN_ERR "FTK use Protocol Type A\n");
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "FTK err = EIO!\n");
		err = 1; /* EIO */
		goto fail;
	}

	info = kzalloc(sizeof(struct ftk_ts_info), GFP_KERNEL);
	if (!info) {
		printk(KERN_ERR "FTK err = ENOMEM!\n");
		err = 1; /* ENOMEM */
		goto fail;
	}

	INIT_WORK(&info->work, ts_event_handler);
	INIT_WORK(&info->work_charger, ts_charger_event_handler);
	INIT_WORK(&info->work_firmware, ftk_firmware_handler);

	info->client = client;
	i2c_set_clientdata(client, info);

	pdata = client->dev.platform_data;

	if (pdata)
		info->power = pdata->power;

	if (info->power) {
		ret = info->power(1);
		if (ret) {
			printk(KERN_ERR "FTK probe power on failed\n");
			goto fail;
		}
	}

	info->dev = &info->client->dev;
	info->input_dev = input_allocate_device();
	info->input_dev->dev.parent = &client->dev;
	if (!info->input_dev) {
		printk(KERN_ERR "FTK err = ENOMEM!\n");
		err = 1; /* ENOMEM */
		goto fail;
	}

	info->input_dev->name = FTK_TS_DRV_NAME;
	snprintf(ftk_ts_phys, sizeof(ftk_ts_phys), "%s/input0",
				info->input_dev->name);
	info->input_dev->phys = ftk_ts_phys;
	info->input_dev->id.bustype = BUS_I2C;
	info->input_dev->id.vendor = 0x0001;
	info->input_dev->id.product = 0x0002;
	info->input_dev->id.version = 0x0100;

	#ifndef CONFIG_MULTI_TOUCH_PROTOCOL_TYPE_B
	info->input_dev->evbit[0] = BIT_MASK(EV_KEY) |
		BIT_MASK(EV_ABS);
	info->input_dev->keybit[BIT_WORD(BTN_TOUCH)] =
		BIT_MASK(BTN_TOUCH);
	set_bit(BTN_TOUCH, info->input_dev->keybit);
	set_bit(BTN_2, info->input_dev->keybit);
	#endif
	set_bit(EV_SYN, info->input_dev->evbit);
	set_bit(EV_KEY, info->input_dev->evbit);

	set_bit(EV_ABS, info->input_dev->evbit);

	input_set_abs_params(info->input_dev, ABS_X,
				X_AXIS_MIN, X_AXIS_MAX, 0, 0);
	input_set_abs_params(info->input_dev, ABS_Y,
				Y_AXIS_MIN, Y_AXIS_MAX, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_TRACKING_ID,
				0, FINGER_MAX, 0, 0);
	input_set_abs_params(info->input_dev, ABS_PRESSURE,
				PRESSURE_MIN, PRESSURE_MAX, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_POSITION_X,
				X_AXIS_MIN, X_AXIS_MAX, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_POSITION_Y,
				Y_AXIS_MIN, Y_AXIS_MAX, 0, 0);
#ifdef CONFIG_MULTI_TOUCH_PROTOCOL_TYPE_B
	input_mt_init_slots(info->input_dev, FINGER_MAX);
	input_set_abs_params(info->input_dev, ABS_MT_TOUCH_MAJOR,
				AREA_MIN, AREA_MAX, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_PRESSURE,
				PRESSURE_MIN, PRESSURE_MAX, 0, 0);
#else /* TYPE A */
	input_set_abs_params(info->input_dev, ABS_MT_TOUCH_MAJOR,
				PRESSURE_MIN, PRESSURE_MAX, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_WIDTH_MAJOR,
				PRESSURE_MIN, PRESSURE_MAX, 0, 0);
#endif
	err = input_register_device(info->input_dev);
	if (err) {
		printk(KERN_ERR "FTK input_register_device fail!\n");
		goto fail;
	}

	#if TOUCH_BOOSTER
	mutex_init(&info->dvfs_lock);
	INIT_DELAYED_WORK(&info->work_dvfs_off, set_dvfs_off);
	INIT_DELAYED_WORK(&info->work_dvfs_chg, change_dvfs_lock);
	info->dvfs_lock_status = false;
	#endif

	for (i = 0; i < FINGER_MAX; i++)
		ID_Indx[i] = 0;
	touch_is_pressed = 0;

	mutex_init(&info->lock);
	info->enabled = false;
	mutex_lock(&info->lock);
	err = init_ftk(info);
	mutex_unlock(&info->lock);
	if (err) {
		printk(KERN_ERR "FTK init_ftk  fail!\n");
		goto fail;
	}
	info->enabled = true;

#ifdef FTK_USE_POLLING_MODE
	info->irq = 0;
#else
	info->irq = client->irq;
#endif

	if (!info->irq) {
		hrtimer_init(&info->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		info->timer.function = st_ts_timer_func;
		hrtimer_start(&info->timer, ktime_set(1, 0),
			HRTIMER_MODE_REL);
	} else {
		if (request_irq(info->irq, ts_interrupt,
			IRQF_TRIGGER_LOW, client->name, info)) {
			printk(KERN_ERR "FTK request_irq  fail!\n");
			err = 1; /* -EBUSY */
			goto fail;
		}
	}

	hrtimer_init(&info->timer_charger, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	info->timer_charger.function = ftk_charger_tmer_func;

#ifdef CONFIG_HAS_EARLYSUSPEND
	info->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 50;
	info->early_suspend.suspend = stm_ts_early_suspend;
	info->early_suspend.resume = stm_ts_late_resume;
	register_early_suspend(&info->early_suspend);
#endif

#ifdef SEC_TSP_FACTORY_TEST
	INIT_LIST_HEAD(&info->cmd_list_head);
	for (i = 0; i < ARRAY_SIZE(tsp_cmds); i++)
		list_add_tail(&tsp_cmds[i].list, &info->cmd_list_head);

	mutex_init(&info->cmd_lock);
	info->cmd_is_running = false;

	info->fac_dev_ts = device_create(sec_class,
			NULL, 0, info, "tsp");
	if (IS_ERR(info->fac_dev_ts))
		printk(KERN_ERR "FTK Failed to create device for the sysfs\n");

	ret = sysfs_create_group(&info->fac_dev_ts->kobj,
			&sec_touch_factory_attr_group);
	if (ret)
		printk(KERN_ERR "FTK Failed to create sysfs group\n");
#endif

	msleep(500);

	printk(KERN_ERR "FTK Charger Timer Start %d\n", ftk_charger_cnt);
	ftk_charger_timer_start(info);
	ftk_firmware_timer_start(info);

	return 0;

fail:
	if (info) {
		if (info->input_dev)
			input_free_device(info->input_dev);
		kfree(info);
	}

	return err;
}

static int stm_ts_remove(struct i2c_client *client)
{
	struct ftk_ts_info *info = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&info->early_suspend);
#endif

	ftk_charger_timer_stop(info);
	ftk_firmware_timer_stop(info);

	ftk_interrupt(info, INT_DISABLE);
	ftk_command(info, FLUSHBUFFER);

	if (info->irq)
		free_irq(client->irq, info);
	else
		hrtimer_cancel(&info->timer);

#ifdef SEC_TSP_FACTORY_TEST
	sysfs_remove_group(&info->fac_dev_ts->kobj,
		&sec_touch_factory_attr_group);
	device_destroy(sec_class, 0);
#endif

	input_unregister_device(info->input_dev);
	kfree(info);

	return 0;
}

static int stm_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret, i;

	struct ftk_ts_info *info = i2c_get_clientdata(client);

	printk(KERN_ERR "FTK enter suspend\n");

	ftk_charger_timer_stop(info);

	if (info->irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&info->timer);

	ret = cancel_work_sync(&info->work);

	ftk_interrupt(info, INT_DISABLE);
	ftk_command(info, FLUSHBUFFER);
	ftk_command(info, SLEEPIN);

	for (i = 0; i < FINGER_MAX; i++)
		ID_Indx[i] = 0;
	touch_is_pressed = 0;

	#if TOUCH_BOOSTER
	set_dvfs_lock(info, 2);
	pr_info("[TSP] dvfs_lock free.\n ");
	#endif

#ifdef CONFIG_MULTI_TOUCH_PROTOCOL_TYPE_B
	for (i = 0; i < FINGER_MAX; i++) {
		input_mt_slot(info->input_dev, i);
		input_report_abs(info->input_dev, ABS_MT_TRACKING_ID, -1);
	}
#else /* TYPE A */
	input_mt_sync(info->input_dev);
#endif

	input_sync(info->input_dev);

	mutex_lock(&info->lock);
	info->enabled = false;
	mutex_unlock(&info->lock);

	if (info->power) {
		ret = info->power(0);
		if (ret)
			printk(KERN_ERR "FTK stm_ts_resume power off failed\n");
	}

	if (ret < 0)
		printk(KERN_ERR "FTK stm_ts_suspend error\n");

	return 0;
}

static int stm_ts_resume(struct i2c_client *client)
{
	int ret = 0;
	u8 regAdd[3];
	unsigned char val = 0;
	struct ftk_ts_info *info = i2c_get_clientdata(client);

	printk(KERN_ERR "FTK wake-up\n");

	ftk_command(info, FLUSHBUFFER);
	ftk_command(info, SLEEPOUT);

	if (ftk_charger_cnt > 0) {
		regAdd[0] = 0xB0;
		regAdd[1] = 0x7C;
		ftk_read_reg(info, regAdd, 2, &val, 1);

		if (val != 0xAF) {
			printk(KERN_ERR "FTK Charger detection re-start\n");
			ftk_charger_timer_start(info);
		} else
			printk(KERN_ERR "FTK Charger detection stop\n");
	}

	if (info->power) {
		ret = info->power(1);
		if (ret)
			printk(KERN_ERR "FTK stm_ts_resume power on failed\n");
	}

	if (info->irq)
		enable_irq(client->irq);
	else
		hrtimer_start(&info->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	ftk_delay(50);
	ftk_command(info, FLUSHBUFFER);
	ftk_interrupt(info, INT_ENABLE);

	mutex_lock(&info->lock);
	info->enabled = true;
	mutex_unlock(&info->lock);

	if (ret < 0)
		printk(KERN_ERR "FTK stm_ts_resume error\n");

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void stm_ts_early_suspend(struct early_suspend *h)
{
	struct ftk_ts_info *info;
	info = container_of(h, struct ftk_ts_info, early_suspend);
	stm_ts_suspend(info->client, PMSG_SUSPEND);
}

static void stm_ts_late_resume(struct early_suspend *h)
{
	struct ftk_ts_info *info;
	info = container_of(h, struct ftk_ts_info, early_suspend);
	stm_ts_resume(info->client);
}
#endif

static const struct i2c_device_id stm_ts_id[] = {
	{FTK_TS_DRV_NAME, 0},
	{}
};

static struct i2c_driver stm_ts_driver = {
	.driver = {
		.name = FTK_TS_DRV_NAME,
	},
	.probe = stm_ts_probe,
	.remove = stm_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = stm_ts_suspend,
	.resume = stm_ts_resume,
#endif
	.id_table = stm_ts_id,
};

static int __init stm_ts_init(void)
{
	stmtouch_wq = create_singlethread_workqueue("stmtouch_wq");
	if (!stmtouch_wq)
		return 1; /* -ENOMEM */

	stmtouch_wq_charger = create_singlethread_workqueue(
		"stmtouch_wq_charger");

	stmtouch_wq_firmware = create_singlethread_workqueue(
		"stmtouch_wq_firmware");

	if (!stmtouch_wq_charger)
		return 1; /* -ENOMEM */

	return i2c_add_driver(&stm_ts_driver);
}

static void __exit stm_ts_exit(void)
{
	i2c_del_driver(&stm_ts_driver);
	if (stmtouch_wq)
		destroy_workqueue(stmtouch_wq);
	if (stmtouch_wq_charger)
		destroy_workqueue(stmtouch_wq_charger);
	if (stmtouch_wq_firmware)
		destroy_workqueue(stmtouch_wq_firmware);

	kfree(pftk_charger);
}

MODULE_DESCRIPTION("STM MultiTouch IC Driver");
MODULE_AUTHOR("JHJANG");
MODULE_LICENSE("GPL");

late_initcall(stm_ts_init);
module_exit(stm_ts_exit);

#ifdef SEC_TSP_FACTORY_TEST
static int handshake(struct ftk_ts_info *info, int type)
{
	unsigned char regAdd[4] = {0};
	unsigned char val = {0};
	unsigned char value[2] = {0};

	regAdd[0] = 0xB3;
	regAdd[1] = 0xFF;
	regAdd[2] = 0xFF;
	ftk_write_reg(info, &regAdd[0], 3);

	regAdd[0] = 0xB1;
	regAdd[1] = 0xFC;
	regAdd[2] = 0x2F;
	ftk_read_reg(info, &regAdd[0], 3, &value[0], 2);

	if (type == HANDSHAKE_START)
		regAdd[3] = (value[1] & 0xF0) + (0x02 & 0x0F);
	else
		regAdd[3] = (value[1] & 0xF0) + (0x01 & 0x0F);

	ftk_write_reg(info, &regAdd[0], 4);

	ftk_delay(10);

	while (1) {
		regAdd[0] = 0xB1;
		regAdd[1] = 0xFC;
		regAdd[2] = 0x2F;

		ftk_read_reg(info, &regAdd[0], 3, &val, 1);
		if ((val & 1) == type)
			break;
		else
			ftk_delay(5);
	}
	return 0;
}

static int ftk_check_index(void *device_data)
{
	struct ftk_ts_info *info = (struct ftk_ts_info *)device_data;

	char buff[TSP_CMD_STR_LEN] = {0};
	int node;

	if (info->cmd_param[0] < 0 ||
			info->cmd_param[0] >= EFFECTIVE_NUM_OF_COL  ||
			info->cmd_param[1] < 0 ||
			info->cmd_param[1] >= EFFECTIVE_NUM_OF_ROW) {
		snprintf(buff, sizeof(buff) , "%s", "NG");
		strncat(info->cmd_result, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = 3;

		dev_info(&info->client->dev, "%s: parameter error: %u,%u\n",
				__func__, info->cmd_param[0],
				info->cmd_param[1]);
		node = -1;
		return node;
	}
	node = info->cmd_param[1] * EFFECTIVE_NUM_OF_COL + info->cmd_param[0];
	dev_info(&info->client->dev, "%s: node = %d\n", __func__,
			node);
	return node;
}

void ftk_print_memory(short *pData)
{
	int i = 0;
	int j = 0;
	unsigned char pStr[6*MAX_COL+1] = {0};
	unsigned char pTmp[16] = {0};

	memset(pStr, 0x0, 6*MAX_COL+1);
	snprintf(pTmp, sizeof(pTmp), "        ");
	strncat(pStr, pTmp, 6*MAX_COL);
	for (i = 0; i < EFFECTIVE_NUM_OF_COL; i++) {
		snprintf(pTmp, sizeof(pTmp), "Rx%02d  ", i);
		strncat(pStr, pTmp, 6*MAX_COL);
	}
	printk(KERN_ERR "%s\n", pStr);

	memset(pStr, 0x0, 6*MAX_COL+1);
	snprintf(pTmp, sizeof(pTmp), "     +");
	strncat(pStr, pTmp, 6*MAX_COL);
	for (i = 0; i < EFFECTIVE_NUM_OF_COL; i++) {
		snprintf(pTmp, sizeof(pTmp), "------");
		strncat(pStr, pTmp, 6*MAX_COL);
	}
	printk(KERN_ERR "%s\n", pStr);

	for (i = 0; i < EFFECTIVE_NUM_OF_ROW; i++) {
		memset(pStr, 0x0, 6*MAX_COL+1);
		snprintf(pTmp, sizeof(pTmp), "Tx%02d | ", i);
		strncat(pStr, pTmp, 6*MAX_COL);
		for (j = 0; j < EFFECTIVE_NUM_OF_COL; j++) {
			snprintf(pTmp, sizeof(pTmp), "%5d ",
				pData[(i*EFFECTIVE_NUM_OF_COL) + j]);
			strncat(pStr, pTmp, 6*MAX_COL);
		}
		printk(KERN_ERR "%s\n", pStr);
	}

}

void ftk_read_memory(struct ftk_ts_info *info, unsigned int Address,
	unsigned short *min, unsigned short *max)
{
	unsigned int totalbytes = MAX_ROW * MAX_COL * 2;
	unsigned int writeAddr = 0;
	unsigned int dataposition = 0;
	unsigned int position = 0;
	unsigned int remained = 0;
	unsigned int readbytes = 0xFF;
	unsigned short buffer = 0;
	unsigned int i, k;
	unsigned char byteWork1[3] = {0};
	unsigned char regAdd[3] = {0};
	unsigned char pDump[256] = {0};
	unsigned char StartRow = 0, StartCol = 0;
	unsigned char EndRow = MAX_ROW - 1, EndCol = MAX_COL - 1;
	unsigned char row_counter = 0, col_counter = 0;
	unsigned int Row, Col, OuterColToSkip, InnerColToSkip;
	unsigned int EffectiveNoOfRows, EffectiveNoOfCols;
	unsigned char proceed;
	unsigned short *pMemoryAll = NULL;
	unsigned short *pMemory = NULL;
	unsigned int start_addr = Address;
	unsigned int end_addr = Address + totalbytes;

	ftk_interrupt(info, INT_DISABLE);

	pMemoryAll = kzalloc((MAX_ROW * MAX_COL * 2), GFP_KERNEL);
	if (!pMemoryAll) {
		ftk_interrupt(info, INT_ENABLE);
		printk(KERN_ERR "pMemoryAll = ENOMEM!\n");
		return;
	}

	regAdd[0] = 0xB0;
	regAdd[1] = 0x1C;
	ftk_read_reg(info, &regAdd[0], 2, &StartCol, 1);

	regAdd[0] = 0xB0;
	regAdd[1] = 0x1D;
	ftk_read_reg(info, &regAdd[0], 2, &col_counter, 1);

	regAdd[0] = 0xB0;
	regAdd[1] = 0x1E;
	ftk_read_reg(info, &regAdd[0], 2, &StartRow, 1);

	regAdd[0] = 0xB0;
	regAdd[1] = 0x1F;
	ftk_read_reg(info, &regAdd[0], 2, &row_counter, 1);

	EndCol = StartCol + col_counter - 1;
	EndRow = StartRow + row_counter - 1;

	Row = EndRow + 1;
	Col = EndCol + 1;

	OuterColToSkip = 31 - EndCol;
	InnerColToSkip = StartCol;

	EffectiveNoOfRows = col_counter;
	EffectiveNoOfCols = row_counter;

	remained = totalbytes;

	switch (Address) {
	case FILTERED_DATA_ADDR_START:
		pMemory = info->filter;
		break;
	case BASELINE_DATA_ADDR_START:
		pMemory = info->baseline;
		break;
	default:
		break;
	}

	handshake(info, HANDSHAKE_START);

	regAdd[0] = 0xB3;
	regAdd[1] = 0x00;
	regAdd[2] = 0x00;
	ftk_write_reg(info, &regAdd[0], 3);

	for (writeAddr = start_addr; writeAddr < end_addr; writeAddr += 0xFE) {
		byteWork1[0] = 0xB1;
		byteWork1[1] = (writeAddr >> 8) & 0xFF;
		byteWork1[2] = writeAddr & 0xFF;

		if (remained >= 0xFE)
			readbytes = 0xFE;
		else
			readbytes = remained;

		memset(pDump, 0x0, readbytes + 1);
		ftk_read_reg(info, &byteWork1[0], 3, pDump, readbytes + 1);

		remained -= readbytes;

		for (i = 1; i <= readbytes; i += 2) {
			buffer = pDump[i + 1];
			buffer = (buffer << 8) | pDump[i];

			pMemoryAll[dataposition++] = buffer;
		}
	}

	handshake(info, HANDSHAKE_END);

	ftk_interrupt(info, INT_ENABLE);

	col_counter = 0;
	row_counter = 0;
	position = 0;

	for (k = 0; k < (Row * MAX_COL); k++) {
		proceed = 0;
		if (col_counter == (EndCol + 1)) {
			k = k + OuterColToSkip;
			col_counter = 0;
			row_counter++;
		}

		if (row_counter >= StartRow)
			if (row_counter <= EndRow)
				if (col_counter >= StartCol)
					if (col_counter <= EndCol)
						proceed = 1;

		if (proceed) {
			pMemory[position] = pMemoryAll[k];
			position++;

			if (pMemoryAll[k] < *min)
				*min = pMemoryAll[k];
			if (pMemoryAll[k] > *max)
				*max = pMemoryAll[k];
		}
		col_counter++;
	}

	kfree(pMemoryAll);
}

void ftk_read_delta(struct ftk_ts_info *info, short *min, short *max)
{
	int i = 0;
	int j = 0;
	int position = 0;

	ftk_read_memory(info, BASELINE_DATA_ADDR_START, min, max);
	ftk_read_memory(info, FILTERED_DATA_ADDR_START, min, max);

	for (i = 0; i < EFFECTIVE_NUM_OF_ROW; i++) {
		for (j = 0; j < EFFECTIVE_NUM_OF_COL; j++) {
			info->delta[position] =
				info->baseline[position] -
				info->filter[position];

			if (info->delta[position] < *min)
				*min = info->delta[position];

			if (info->delta[position] > *max)
				*max = info->delta[position];

			position++;
		}
	}
}

static int check_adjacency_percent_error(struct ftk_ts_info *info)
{
	int x = 0;
	int y = 0;
	int pos = 0;
	unsigned short *pData = info->filter;
	unsigned short min = 0;
	unsigned short max = 0;
	unsigned short up = 0;
	unsigned short down = 0;
	unsigned short left = 0;
	unsigned short right = 0;
	int error = 0;

	ftk_read_memory(info, FILTERED_DATA_ADDR_START, &min, &max);

	/* [0, 0] */
	{
		pos = 0;

		min = pData[pos] * (100 - ADJACENCY_ERROR_PERCENT) / 100;
		max = pData[pos] * (100 + ADJACENCY_ERROR_PERCENT) / 100;

		down = pData[pos + EFFECTIVE_NUM_OF_COL];
		right = pData[pos + 1];

		if (down < min || down > max)
			error = 1;

		if (right < min || right > max)
			error = 1;
	}

	/* [MAX, 0] */
	{
		pos = EFFECTIVE_NUM_OF_COL - 1;

		min = pData[pos] * (100 - ADJACENCY_ERROR_PERCENT) / 100;
		max = pData[pos] * (100 + ADJACENCY_ERROR_PERCENT) / 100;

		down = pData[pos + EFFECTIVE_NUM_OF_COL];
		left = pData[pos - 1];

		if (down < min || down > max)
			error = 1;

		if (left < min || left > max)
			error = 1;
	}

	/* [0, MAX] */
	{
		pos =  EFFECTIVE_NUM_OF_COL * (EFFECTIVE_NUM_OF_ROW - 1);

		min = pData[pos] * (100 - ADJACENCY_ERROR_PERCENT) / 100;
		max = pData[pos] * (100 + ADJACENCY_ERROR_PERCENT) / 100;

		up = pData[pos - EFFECTIVE_NUM_OF_COL];
		right = pData[pos + 1];

		if (up < min || up > max)
			error = 1;

		if (right < min || right > max)
			error = 1;
	}

	/* [MAX, MAX] */
	{
		pos = (EFFECTIVE_NUM_OF_COL * EFFECTIVE_NUM_OF_ROW) - 1;

		min = pData[pos] * (100 - ADJACENCY_ERROR_PERCENT) / 100;
		max = pData[pos] * (100 + ADJACENCY_ERROR_PERCENT) / 100;

		up = pData[pos - EFFECTIVE_NUM_OF_COL];
		left = pData[pos - 1];

		if (up < min || up > max)
			error = 1;

		if (left < min || left > max)
			error = 1;
	}

	if (error)
		return error;

	for (y = 1; y < EFFECTIVE_NUM_OF_ROW - 1; y++) {
		for (x = 1; x < EFFECTIVE_NUM_OF_COL - 1; x++) {
			pos = (y*EFFECTIVE_NUM_OF_COL) + x;

			min = pData[pos]*(100 - ADJACENCY_ERROR_PERCENT) / 100;
			max = pData[pos]*(100 + ADJACENCY_ERROR_PERCENT) / 100;

			up = pData[pos - EFFECTIVE_NUM_OF_COL];
			down = pData[pos + EFFECTIVE_NUM_OF_COL];
			left = pData[pos - 1];
			right = pData[pos + 1];

			if (up < min || up > max)
				error = 1;

			if (down < min || down > max)
				error = 1;

			if (left < min || left > max)
				error = 1;

			if (right < min || right > max)
				error = 1;
		}
	}

	return error;
}

static ssize_t store_cmd(struct device *dev, struct device_attribute *devattr,
		const char *buf, size_t count)
{
	struct ftk_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	char *cur, *start, *end;
	char buff[TSP_CMD_STR_LEN] = {0};
	int len, i;
	struct tsp_cmd *tsp_cmd_ptr = NULL;
	char delim = ',';
	bool cmd_found = false;
	int param_cnt = 0;

	if (info->cmd_is_running == true) {
		dev_err(&info->client->dev, "tsp_cmd: other cmd is running.\n");
		goto err_out;
	}

	/* check lock	*/
	mutex_lock(&info->cmd_lock);
	info->cmd_is_running = true;
	mutex_unlock(&info->cmd_lock);

	info->cmd_state = 1;

	for (i = 0; i < ARRAY_SIZE(info->cmd_param); i++)
		info->cmd_param[i] = 0;

	len = (int)count;
	if (*(buf + len - 1) == '\n')
		len--;
	memset(info->cmd, 0x00, ARRAY_SIZE(info->cmd));
	memcpy(info->cmd, buf, len);

	cur = strchr(buf, (int)delim);
	if (cur)
		memcpy(buff, buf, cur - buf);
	else
		memcpy(buff, buf, len);

	/* find command */
	list_for_each_entry(tsp_cmd_ptr, &info->cmd_list_head, list) {
		if (!strncmp(buff, tsp_cmd_ptr->cmd_name, TSP_CMD_STR_LEN)) {
			cmd_found = true;
			break;
		}
	}

	/* set not_support_cmd */
	if (!cmd_found) {
		list_for_each_entry(tsp_cmd_ptr, &info->cmd_list_head, list) {
			if (!strncmp("not_support_cmd", tsp_cmd_ptr->cmd_name,
				TSP_CMD_STR_LEN))
				break;
		}
	}

	/* parsing parameters */
	if (cur && cmd_found) {
		cur++;
		start = cur;
		memset(buff, 0x00, ARRAY_SIZE(buff));
		do {
			if (*cur == delim || cur - buf == len) {
				end = cur;
				memcpy(buff, start, end - start);
				*(buff + strnlen(buff,
					ARRAY_SIZE(buff))) = '\0';
				if (kstrtoint(buff, 10,
					info->cmd_param + param_cnt) < 0)
					goto err_out;
				start = cur + 1;
				memset(buff, 0x00, ARRAY_SIZE(buff));
				param_cnt++;
			}
			cur++;
		} while (cur - buf <= len);
	}

	dev_info(&client->dev, "cmd = %s\n", tsp_cmd_ptr->cmd_name);
	for (i = 0; i < param_cnt; i++)
		dev_info(&client->dev, "cmd param %d= %d\n", i,
							info->cmd_param[i]);

	tsp_cmd_ptr->cmd_func(info);

err_out:
	return count;
}

static ssize_t show_cmd_status(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct ftk_ts_info *info = dev_get_drvdata(dev);
	char buff[16] = {0};

	dev_info(&info->client->dev, "tsp cmd: status:%d\n",
			info->cmd_state);

	if (info->cmd_state == 0)
		snprintf(buff, sizeof(buff), "WAITING");

	else if (info->cmd_state == 1)
		snprintf(buff, sizeof(buff), "RUNNING");

	else if (info->cmd_state == 2)
		snprintf(buff, sizeof(buff), "OK");

	else if (info->cmd_state == 3)
		snprintf(buff, sizeof(buff), "FAIL");

	else if (info->cmd_state == 4)
		snprintf(buff, sizeof(buff), "NOT_APPLICABLE");

	return snprintf(buf, TSP_BUF_SIZE, "%s\n", buff);
}

static ssize_t show_cmd_result(struct device *dev, struct device_attribute
		*devattr, char *buf)
{
	struct ftk_ts_info *info = dev_get_drvdata(dev);

	dev_info(&info->client->dev, "tsp cmd: result: %s\n", info->cmd_result);

	mutex_lock(&info->cmd_lock);
	info->cmd_is_running = false;
	mutex_unlock(&info->cmd_lock);

	info->cmd_state = 0;

	return snprintf(buf, TSP_BUF_SIZE, "%s\n", info->cmd_result);
}

static void set_default_result(struct ftk_ts_info *info)
{
	char delim = ':';

	memset(info->cmd_result, 0x00, ARRAY_SIZE(info->cmd_result));
	memcpy(info->cmd_result, info->cmd,
		strnlen(info->cmd, TSP_CMD_STR_LEN));
	strncat(info->cmd_result, &delim, 1);
}

static void set_cmd_result(struct ftk_ts_info *info, char *buff, int len)
{
	strncat(info->cmd_result, buff, len);
}

static void not_support_cmd(void *device_data)
{
	struct ftk_ts_info *info = (struct ftk_ts_info *)device_data;
	char buff[16] = {0};

	set_default_result(info);
	snprintf(buff, sizeof(buff), "%s", "NA");
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 4;
	dev_info(&info->client->dev, "%s: \"%s(%d)\"\n", __func__,
				buff, strnlen(buff, sizeof(buff)));
}

static void fw_update(void *device_data)
{
	struct ftk_ts_info *info = (struct ftk_ts_info *)device_data;
	struct i2c_client *client = info->client;
	char buff[64] = {0};
	char data[4] = {0};
	int rc = 0;

	set_default_result(info);

	switch (info->cmd_param[0]) {
	case BUILT_IN:
		snprintf(buff, sizeof(buff), "fw version update does not need");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		dev_info(&client->dev, "%s\n", buff);
		goto ErrorExit;

	case UMS:
		rc = firmware_readdata(FIRMWARE_SDFILE, data);
		if (rc == 0 && (data[1] > firmware_version)) {
			snprintf(buff, sizeof(buff), "ums fw is loaded!!");
			set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
			dev_info(&client->dev, "%s\n", buff);
		} else {
			snprintf(buff, sizeof(buff),
				"fw version update does not need");
			set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
			dev_info(&client->dev, "%s\n", buff);
			goto ErrorExit;
		}
		break;

	default:
		snprintf(buff, sizeof(buff), "invalid fw file type!!");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		dev_info(&client->dev, "%s\n", buff);
		goto ErrorExit;
	}

	ftk_charger_timer_stop(info);
	ftk_firmware_timer_stop(info);

	ftk_write_signature(info, 0);
	firmware_load(info, FIRMWARE_SDFILE);
	firmware_update(FIRMWARE_SDFILE, FIRMWARE_NAME);
	ftk_write_signature(info, 1);

	ftk_charger_timer_start(info);

	if (firmware_version == data[1]) {
		snprintf(buff, sizeof(buff), "fw update done. ver = %d\n",
			firmware_version);
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		dev_info(&client->dev, "%s\n", buff);
	} else {
		snprintf(buff, sizeof(buff),
			"ERROR :fw version is still wrong (%d != %d)\n",
			firmware_version, data[1]);
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		dev_info(&client->dev, "%s\n", buff);
	}

ErrorExit:
	info->cmd_state = 2;
	return;
}

static void get_fw_ver_bin(void *device_data)
{
	struct ftk_ts_info *info = (struct ftk_ts_info *)device_data;
	char buff[16] = {0};

	set_default_result(info);
	snprintf(buff, sizeof(buff), "%d", firmware_version);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_fw_ver_ic(void *device_data)
{
	struct ftk_ts_info *info = (struct ftk_ts_info *)device_data;
	char buff[16] = {0};

	set_default_result(info);
	snprintf(buff, sizeof(buff), "%d", firmware_version);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_config_ver(void *device_data)
{
	struct ftk_ts_info *info = (struct ftk_ts_info *)device_data;
	char buff[16] = {0};

	snprintf(buff, sizeof(buff), "M950_ST_%02d%02d",
		firmware_month, firmware_day);
	set_default_result(info);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_threshold(void *device_data)
{
	struct ftk_ts_info *info = (struct ftk_ts_info *)device_data;
	char buff[16] = {0};
	u8 value = 0;
	u8 regAdd[2] = {0};

	set_default_result(info);

	regAdd[0] = 0xB0;
	regAdd[1] = 0x2C;
	ftk_read_reg(info, regAdd, 2, &value, 1);

	snprintf(buff, sizeof(buff), "%d", value);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void module_off_master(void *device_data)
{
	struct ftk_ts_info *info = (struct ftk_ts_info *)device_data;

	char buff[3] = {0};
	int ret = 0;

	mutex_lock(&info->lock);
	if (info->enabled) {
		disable_irq(info->irq);
		info->enabled = false;
	}
	mutex_unlock(&info->lock);

	if (info->power)
		ret = info->power(0);

	if (ret == 0)
		snprintf(buff, sizeof(buff), "%s", "OK");
	else
		snprintf(buff, sizeof(buff), "%s", "NG");

	set_default_result(info);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	if (strncmp(buff, "OK", 2) == 0)
		info->cmd_state = 2;
	else
		info->cmd_state = 3;

	dev_info(&info->client->dev, "%s: %s\n", __func__, buff);
}

static void module_on_master(void *device_data)
{
	struct ftk_ts_info *info = (struct ftk_ts_info *)device_data;

	char buff[3] = {0};
	int ret = 0;

	mutex_lock(&info->lock);
	if (!info->enabled) {
		enable_irq(info->irq);
		info->enabled = true;
	}
	mutex_unlock(&info->lock);

	if (info->power)
		ret = info->power(1);

	if (ret == 0)
		snprintf(buff, sizeof(buff), "%s", "OK");
	else
		snprintf(buff, sizeof(buff), "%s", "NG");

	set_default_result(info);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	if (strncmp(buff, "OK", 2) == 0)
		info->cmd_state = 2;
	else
		info->cmd_state = 3;

	dev_info(&info->client->dev, "%s: %s\n", __func__, buff);
}

static void get_chip_vendor(void *device_data)
{
	struct ftk_ts_info *info = (struct ftk_ts_info *)device_data;
	char buff[16] = {0};

	strncpy(buff, "STM", sizeof(buff));
	set_default_result(info);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_chip_name(void *device_data)
{
	struct ftk_ts_info *info = (struct ftk_ts_info *)device_data;
	char buff[16] = {0};

	strncpy(buff, "STMT05C", sizeof(buff));
	set_default_result(info);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_x_num(void *device_data)
{
	struct ftk_ts_info *info = (struct ftk_ts_info *)device_data;
	char buff[16] = {0};

	set_default_result(info);
	snprintf(buff, sizeof(buff), "%d", EFFECTIVE_NUM_OF_COL);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_y_num(void *device_data)
{
	struct ftk_ts_info *info = (struct ftk_ts_info *)device_data;
	char buff[16] = {0};

	set_default_result(info);
	snprintf(buff, sizeof(buff), "%d", EFFECTIVE_NUM_OF_ROW);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void run_reference_read(void *device_data)
{
	struct ftk_ts_info *info = (struct ftk_ts_info *)device_data;
	char buff[TSP_CMD_STR_LEN] = {0};
	unsigned short min = 0xFFFF;
	unsigned short max = 0;

	set_default_result(info);
	ftk_read_memory(info, BASELINE_DATA_ADDR_START, &min, &max);
	ftk_print_memory(info->baseline);
	snprintf(buff, sizeof(buff), "%d,%d", min, max);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s\n", __func__,
			buff);
}

static void get_reference(void *device_data)
{
	struct ftk_ts_info *info = (struct ftk_ts_info *)device_data;
	char buff[16] = {0};
	unsigned short min = 0xFFFF;
	unsigned short max = 0;
	short val = 0;
	int node = 0;

	set_default_result(info);
	node = ftk_check_index(info);
	if (node < 0)
		return;

	ftk_read_memory(info, BASELINE_DATA_ADDR_START, &min, &max);

	val = info->baseline[node];
	snprintf(buff, sizeof(buff), "%d", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void run_raw_read(void *device_data)
{
	struct ftk_ts_info *info = (struct ftk_ts_info *)device_data;
	char buff[TSP_CMD_STR_LEN] = {0};
	unsigned short min = 0xFFFF;
	unsigned short max = 0;

	set_default_result(info);
	ftk_read_memory(info, FILTERED_DATA_ADDR_START, &min, &max);
	ftk_print_memory(info->filter);
	snprintf(buff, sizeof(buff), "%d,%d", min, max);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s\n", __func__,
			buff);
}

static void get_raw(void *device_data)
{
	struct ftk_ts_info *info = (struct ftk_ts_info *)device_data;
	char buff[16] = {0};
	unsigned short min = 0xFFFF;
	unsigned short max = 0;
	short val = 0;
	int node = 0;

	set_default_result(info);
	node = ftk_check_index(info);
	if (node < 0)
		return;

	ftk_read_memory(info, FILTERED_DATA_ADDR_START, &min, &max);

	val = info->filter[node];
	snprintf(buff, sizeof(buff), "%d", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));

}

static void run_delta_read(void *device_data)
{
	struct ftk_ts_info *info = (struct ftk_ts_info *)device_data;
	char buff[TSP_CMD_STR_LEN] = {0};
	short min = 0xFFFF;
	short max = -32767;

	set_default_result(info);
	ftk_read_delta(info, &min, &max);
	ftk_print_memory(info->delta);
	snprintf(buff, sizeof(buff), "%d,%d", min, max);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s\n", __func__,
			buff);
}

static void get_delta(void *device_data)
{
	struct ftk_ts_info *info = (struct ftk_ts_info *)device_data;
	char buff[16] = {0};
	short min = 0xFFFF;
	short max = -32767;
	short val = 0;
	int node = 0;

	set_default_result(info);
	node = ftk_check_index(info);
	if (node < 0)
		return;

	ftk_read_delta(info, &min, &max);

	val = info->delta[node];
	snprintf(buff, sizeof(buff), "%d", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void run_selftest(void *device_data)
{
	struct ftk_ts_info *info = (struct ftk_ts_info *)device_data;

	char buff[3] = {0};
	int ret = 0;

	ret = check_adjacency_percent_error(info);

	if (ret == 0)
		snprintf(buff, sizeof(buff), "%s", "OK");
	else
		snprintf(buff, sizeof(buff), "%s", "NG");

	set_default_result(info);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	if (strncmp(buff, "OK", 2) == 0)
		info->cmd_state = 2;
	else
		info->cmd_state = 3;

	dev_info(&info->client->dev, "%s: %s\n", __func__, buff);

}

#endif /* SEC_TSP_FACTORY_TEST */
