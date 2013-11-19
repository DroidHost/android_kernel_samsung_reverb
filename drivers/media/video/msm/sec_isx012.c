/*
  SEC ISX012
 */
/***************************************************************
CAMERA DRIVER FOR 5M CAM (SONY)
****************************************************************/

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>


#include "sec_isx012.h"

#include "sec_cam_dev.h"

#include "sec_isx012_reg.h"

#include <linux/clk.h>
#include <linux/io.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>


#include <asm/mach-types.h>
#include <mach/vreg.h>
#include <linux/io.h>
#include "msm.h"
#include <linux/mfd/pmic8058.h>

/*#define CONFIG_LOAD_FILE*/

#ifdef CONFIG_LOAD_FILE

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
/*#include <asm/uaccess.h>*/

static char *isx012_regs_table;
static int isx012_regs_table_size;
static int isx012_write_regs_from_sd(char *name);
#define TABLE_MAX_NUM 500
int gtable_buf[TABLE_MAX_NUM];
#endif

#define SONY_ISX012_BURST_DATA_LENGTH	1200

#define ISX012_WRITE_LIST(A)\
	isx012_i2c_write_list(A, (sizeof(A) / sizeof(A[0])), #A);
#define ISX012_BURST_WRITE_LIST(A)\
	isx012_i2c_burst_write_list(A, (sizeof(A) / sizeof(A[0])), #A);
#define CAM_nRST		130
#define CAM_nSTBY		131
#define CAM_EN			3
#define CAM_EN_1		143
#define CAM_EN_2		132
#define CAM_I2C_SCL		30
#define CAM_I2C_SDA		17
#define CAM_VT_nSTBY	2
#define CAM_VT_RST		175
#define CAM_MCLK		15
#define PMIC_GPIO_CAM_FLASH_SET	PM8058_GPIO(27)
#define PMIC_GPIO_CAM_FLASH_EN	PM8058_GPIO(28)

#define POLL_TIME_MS	10
/*native cmd code*/
#define CAM_AF		1
#define CAM_FLASH	2

#define FACTORY_TEST 1

struct isx012_work_t {
	struct work_struct work;
};

static struct isx012_work_t *isx012_sensorw;
static struct i2c_client *isx012_client;
struct i2c_client *sr130pc10_client;

static struct isx012_exif_data *isx012_exif;

static struct isx012_ctrl_t *isx012_ctrl;
int iscapture;
int gLowLight_check;
int DtpTest;
uint16_t g_ae_auto, g_ae_now;
int16_t g_ersc_auto, g_ersc_now;

/* for tuning */
int gERRSCL_AUTO, gUSER_AESCL_AUTO, gERRSCL_NOW;
int gUSER_AESCL_NOW, gAE_OFSETVAL, gAE_MAXDIFF, gGLOWLIGHT_DEFAULT;
int gGLOWLIGHT_ISO50, gGLOWLIGHT_ISO100;
int gGLOWLIGHT_ISO200, gGLOWLIGHT_ISO400;
/* for tuning */

static int accessibility_torch;
static DECLARE_WAIT_QUEUE_HEAD(isx012_wait_queue);
DECLARE_MUTEX(isx012_sem);

/*
#define ISX012_WRITE_LIST(A) \
    isx012_i2c_write_list(A,(sizeof(A) / sizeof(A[0])),#A);
*/

static int
isx012_i2c_read_multi
(unsigned short subaddr, unsigned short *data, unsigned short len)
{
	unsigned char buf[4];
	struct i2c_msg msg = {isx012_client->addr, 0, 2, buf};

	int err = 0;

	if (!isx012_client->adapter) {
		dev_err(&isx012_client->dev,
		"[%s:%d] can't find i2c client adapter\n", __func__, __LINE__);
		return -EIO;
	}

	buf[0] = subaddr >> 8;
	buf[1] = subaddr & 0xff;

	err = i2c_transfer(isx012_client->adapter, &msg, 1);
	if (unlikely(err < 0)) {
		dev_err(&isx012_client->dev,
			"[%s:%d] register read fail\n", __func__, __LINE__);
		return -EIO;
	}

	msg.flags = I2C_M_RD;
	msg.len = 2;

	err = i2c_transfer(isx012_client->adapter, &msg, 1);
	if (unlikely(err < 0)) {
		dev_err(&isx012_client->dev,
			"[%s:%d] register read fail\n", __func__, __LINE__);
		return -EIO;
	}

	/*
	* Data comes in Little Endian in parallel mode; So there
	* is no need for byte swapping here
	*/
	*data = *(unsigned long *)(&buf);

	return err;
}

static int isx012_i2c_read(unsigned short subaddr, unsigned short *data)
{
	unsigned char buf[2];
	struct i2c_msg msg = {isx012_client->addr, 0, 2, buf};

	int err = 0;

	if (!isx012_client->adapter) {
		dev_err(&isx012_client->dev,
		"[%s:%d] can't search i2c client adapter\n",
		__func__, __LINE__);
		return -EIO;
	}

	buf[0] = subaddr >> 8;
	buf[1] = subaddr & 0xff;

	err = i2c_transfer(isx012_client->adapter, &msg, 1);
	if (unlikely(err < 0)) {
		dev_err(&isx012_client->dev,
		"[%s:%d] register read fail\n", __func__, __LINE__);
		return -EIO;
	}

	msg.flags = I2C_M_RD;

	err = i2c_transfer(isx012_client->adapter, &msg, 1);
	if (unlikely(err < 0)) {
		dev_err(&isx012_client->dev,
		"[%s:%d] register read fail\n", __func__, __LINE__);
		return -EIO;
	}

	/*
	 * Data comes in Little Endian in parallel mode; So there
	 * is no need for byte swapping here
	 */
	*data = *(unsigned short *)(&buf);

	return err;
}
static int isx012_i2c_write_multi_temp
	(unsigned short subaddr, unsigned short val)
{
	unsigned char buf[2] = {0};
	struct i2c_msg msg = {0x20, 0, 2, buf};

	int retry_count = 5;
	int err = 0;

	if (!sr130pc10_client->adapter) {
		dev_err(&sr130pc10_client->dev,
		"[%s:%d]can't find i2c client adapter\n", __func__, __LINE__);
		return -EIO;
	}

	buf[0] = subaddr;
	buf[1] = val;

	/*
	 * Data should be written in Little Endian in parallel mode; So there
	 * is no need for byte swapping here
	 */
	while (retry_count--) {
		err  = i2c_transfer(sr130pc10_client->adapter, &msg, 1);
		if (likely(err == 1))
			break;
		else
			printk(KERN_ERR "[%s] error retry(%d)\n",
			 __func__, retry_count);

		msleep(POLL_TIME_MS);
	}

	return (err == 1) ? 0 : -EIO;
}

static int isx012_i2c_write_multi
	(unsigned short addr, unsigned int w_data, unsigned int w_len)
{
	unsigned char buf[w_len+2];
	struct i2c_msg msg = {isx012_client->addr, 0, w_len+2, buf};

	int retry_count = 5;
	int err = 0;

	if (!isx012_client->adapter) {
		dev_err(&isx012_client->dev,
			"[%s:%d] can't search i2c client adapter\n",
			__func__, __LINE__);
		return -EIO;
	}

	buf[0] = addr >> 8;
	buf[1] = addr & 0xff;

	/*
	 * Data should be written in Little Endian in parallel mode; So there
	 * is no need for byte swapping here
	 */
	if (w_len == 1)
		buf[2] = (unsigned char)w_data;
	else if (w_len == 2)
		*((unsigned short *)&buf[2]) = (unsigned short)w_data;
	else
		*((unsigned int *)&buf[2]) = w_data;

/*
//def ISX012_DEBUG
	{
		int j;
		CAM_DEBUG("isx012 i2c write W: ");
		for(j = 0; j <= w_len+1; j++)
		{
			CAM_DEBUG("0x%02x ", buf[j]);
		}
		CAM_DEBUG("\n");
	}
*/

	while (retry_count--) {
		err  = i2c_transfer(isx012_client->adapter, &msg, 1);
		if (likely(err == 1))
			break;
		else
			printk(KERN_ERR "[%s] error retry(%d)\n",
			__func__, retry_count);

		msleep(POLL_TIME_MS);
	}

	return (err == 1) ? 0 : -EIO;
}

static int
isx012_i2c_write_list(struct isx012_short_t regs[], int size, char *name)
{

#ifdef CONFIG_LOAD_FILE
	isx012_write_regs_from_sd(name);
#else
	int err = 0;
	int i = 0;

	printk(KERN_ERR "[%s:%d] - [%s]\n", __func__, __LINE__, name);

	if (!isx012_client->adapter) {
		printk(KERN_ERR "[%s:%d] can't search i2c client adapter\n",
			__func__, __LINE__);
		return -EIO;
	}

	for (i = 0; i < size; i++) {
		if (regs[i].subaddr == 0xFFFF) {
			mdelay(regs[i].value);
			/*CAM_DEBUG("delay 0x%04x, value 0x%04x\n",
			regs[i].subaddr, regs[i].value);*/
		} else {
				err = isx012_i2c_write_multi(regs[i].subaddr,
					regs[i].value, regs[i].len);

			if (unlikely(err < 0)) {
				printk(KERN_ERR "[%s] register set failed\n",
					__func__);
				return -EIO;
			}
		}
	}
#endif

	return 0;
}
static int isx012_i2c_burst_write_list
	(struct isx012_short_t regs[], int size, char *name)
{
#ifdef CONFIG_LOAD_FILE
		isx012_i2c_write_list(regs, size, name);
#else
	int i = 0;
	int iTxDataIndex = 0;
	int retry_count = 5;
	int err = 0;

	printk(KERN_ERR "[%s:%d] - [%s]\n", __func__, __LINE__, name);

	unsigned char buf[SONY_ISX012_BURST_DATA_LENGTH];
	struct i2c_msg msg = {isx012_client->addr, 0, 4, buf};

	if (!isx012_client->adapter) {
		printk(KERN_ERR "[%s:%d] can't search i2c client adapter\n",
			__func__, __LINE__);
		return -EIO;
	}

	while (i < size) { /*0<1*/
		if (0 == iTxDataIndex) {
			/*printk(KERN_DEBUG
			"11111111111 delay 0x%04x, value 0x%04x\n",
				regs[i].subaddr, regs[i].value);*/
			buf[iTxDataIndex++] = (regs[i].subaddr & 0xFF00) >> 8;
			buf[iTxDataIndex++] = (regs[i].subaddr & 0xFF);
		}

		if ((i < size - 1) \
			&& ((iTxDataIndex + regs[i].len) <= \
			(SONY_ISX012_BURST_DATA_LENGTH - regs[i+1].len)) && \
			(regs[i].subaddr + regs[i].len == regs[i+1].subaddr)) {
			if (1 == regs[i].len) {
				/*printk(KERN_DEBUG
				"2222222 delay 0x%04x, value 0x%04x\n",
					regs[i].subaddr, regs[i].value);*/
				buf[iTxDataIndex++] = (regs[i].value & 0xFF);
			} else {
				/* Little Endian*/
				buf[iTxDataIndex++] = (regs[i].value & 0x00FF);
				buf[iTxDataIndex++] =
					(regs[i].value & 0xFF00) >> 8;
				/*printk(KERN_DEBUG
				"3333333 delay 0x%04x, value 0x%04x\n",
				regs[i].subaddr, regs[i].value);*/
			}
		} else {
			if (1 == regs[i].len) {
				/*printk(KERN_DEBUG
				"4444444 delay 0x%04x, value 0x%04x\n",
				regs[i].subaddr, regs[i].value);*/
				buf[iTxDataIndex++] = (regs[i].value & 0xFF);
				/*printk(KERN_DEBUG
				"burst_index:%d\n", iTxDataIndex);*/
				msg.len = iTxDataIndex;
			} else {
				/*printk(KERN_DEBUG
				"555555 delay 0x%04x, value 0x%04x\n",
				regs[i].subaddr, regs[i].value);*/
				/* Little Endian*/
				buf[iTxDataIndex++] = (regs[i].value & 0x00FF);
				buf[iTxDataIndex++] =
					(regs[i].value & 0xFF00) >> 8;
				/*CAM_DEBUG("burst_index:%d\n",
				iTxDataIndex);*/
				msg.len = iTxDataIndex;
			}

			while (retry_count--) {
				err  = i2c_transfer(isx012_client->adapter,
					&msg, 1);
				if (likely(err == 1))
					break;
				else
					printk(KERN_DEBUG
					"[%s] error retry(%d)\n",
					__func__, retry_count);
			}
			retry_count = 5;
			iTxDataIndex = 0;
		}
		i++;
	}
#endif

	return 0;
}

#ifdef CONFIG_LOAD_FILE
void isx012_regs_table_init(void)
{
	struct file *filp;
	char *dp;
	long l;
	loff_t pos;
	int ret;
	mm_segment_t fs = get_fs();

	CAM_DEBUG("[%s:%d]\n", __func__, __LINE__);

	set_fs(get_ds());

	filp = filp_open("/mnt/sdcard/sec_isx012_reg.h", O_RDONLY, 0);

	if (IS_ERR_OR_NULL(filp)) {
		CAM_DEBUG("[%s:%d]file open error\n", __func__, __LINE__);
		return PTR_ERR(filp);
	}

	l = filp->f_path.dentry->d_inode->i_size;
	CAM_DEBUG("[%s:%d]l = %ld\n", __func__, __LINE__, l);
	/*dp = kmalloc(l, GFP_KERNEL);*/
	dp = vmalloc(l);
	if (dp == NULL) {
		CAM_DEBUG("[%s:%d] Out of Memory\n", __func__, __LINE__);
		filp_close(filp, current->files);
	}

	pos = 0;
	memset(dp, 0, l);
	ret = vfs_read(filp, (char __user *)dp, l, &pos);

	if (ret != l) {
		CAM_DEBUG(
			"[%s:%d] Failed to read file ret = %d\n",
			__func__, __LINE__, ret);
		/*kfree(dp);*/
		vfree(dp);
		filp_close(filp, current->files);
		return -EINVAL;
	}

	filp_close(filp, current->files);

	set_fs(fs);

	isx012_regs_table = dp;

	isx012_regs_table_size = l;

	*((isx012_regs_table + isx012_regs_table_size) - 1) = '\0';

	CAM_DEBUG("[%s:%d][X]\n", __func__, __LINE__);
	return 0;
}

void isx012_regs_table_exit(void)
{
	CAM_DEBUG("[%s:%d][E]\n", __func__, __LINE__);

	if (isx012_regs_table) {
		vfree(isx012_regs_table);
		isx012_regs_table = NULL;
	}
}

static int isx012_define_table()
{
	char *start, *end, *reg;
	char *start_token, *reg_token, *temp;
	char reg_buf[61], temp2[61];
	char token_buf[5];
	int token_value = 0;
	int index_1 = 0, index_2 = 0, total_index;
	int len = 0, total_len = 0;

	*(reg_buf + 60) = '\0';
	*(temp2 + 60) = '\0';
	*(token_buf + 4) = '\0';
	memset(gtable_buf, 9999, TABLE_MAX_NUM);

	CAM_DEBUG("[%s:%d][E]\n", __func__, __LINE__);

	start = strnstr(isx012_regs_table, "aeoffset_table",
					isx012_regs_table_size);
	end = strnstr(start, "};", isx012_regs_table_size);

	/* Find table */
	index_2 = 0;
	while (1) {
		reg = strnstr(start, "	", isx012_regs_table_size);
		if ((reg == NULL) || (reg > end)) {
			CAM_DEBUG(
				"[%s:%d]isx012_define_table read end!\n",
				__func__, __LINE__);
			break;
		}

		/* length cal */
		index_1 = 0;
		total_len = 0;
		temp = reg;
		if (temp != NULL) {
			memcpy(temp2, (temp + 1), 60);
			/*CAM_DEBUG("temp2 : %s\n", temp2);*/
		}
		start_token = strnstr(temp, ",", isx012_regs_table_size);
		while (index_1 < 10) {
			start_token = strnstr(temp, ",",
						isx012_regs_table_size);
			len = strcspn(temp, ",");
			/*CAM_DEBUG("len[%d]\n", len);	//Only debug*/
			total_len = total_len + len;
			temp = (temp + (len+2));
			index_1++;
		}
		total_len = total_len + 19;
		/*CAM_DEBUG("%d\n", total_len);	//Only debug*/

		/* read table */
		if (reg != NULL) {
			memcpy(reg_buf, (reg + 1), total_len);
			/*CAM_DEBUG("reg_buf : %s\n",
			reg_buf);	//Only debug*/
			start = (reg + total_len+1);
		}

		reg_token = reg_buf;

		index_1 = 0;
		start_token = strnstr(reg_token, ",", isx012_regs_table_size);
		while (index_1 < 10) {
			start_token = strnstr(reg_token, ",",
						isx012_regs_table_size);
			len = strcspn(reg_token, ",");
			/*CAM_DEBUG("len[%d]\n",
			len);	//Only debug*/
			memcpy(token_buf, reg_token, len);
			/*CAM_DEBUG("[%d]%s ",
			index_1, token_buf);	//Only debug*/
			kstrtol(token_buf, 10, &token_value);
			total_index = index_2 * 10 + index_1;
			/*CAM_DEBUG("[%d]%d ",
			total_index, token_value);	//Only debug*/
			gtable_buf[total_index] = token_value;
			index_1++;
			reg_token = (reg_token + (len + 2));
		}
		index_2++;
	}
/*
	//Only debug
	index_2 = 0;
	while (index_2 < TABLE_MAX_NUM) {
		CAM_DEBUG("[%d]%d ", index_2, gtable_buf[index_2]);
		index_2++;
	}
*/
	CAM_DEBUG("[%s:%d]isx012_define_table end!\n", __func__, __LINE__);

	return 0;
}

static int isx012_define_read(char *name, int len_size)
{
	char *start, *end, *reg;
	char reg_7[7], reg_5[5];
	int define_value = 0;

	*(reg_7 + 6) = '\0';
	*(reg_5 + 4) = '\0';

	/*CAM_DEBUG("isx012_define_read start!\n");*/

	start = strnstr(isx012_regs_table, name, isx012_regs_table_size);
	end = strnstr(start, "tuning", isx012_regs_table_size);

	reg = strnstr(start, " ", isx012_regs_table_size);

	if ((reg == NULL) || (reg > end)) {
		printk(KERN_DEBUG "isx012_define_read error %s : ", name);
		return -EFAULT;

		/*return -1;*/
	}

	/* Write Value to Address */
	if (reg != NULL) {
		if (len_size == 6) {
			memcpy(reg_7, (reg + 1), len_size);
			kstrtol(reg_7, 16, &define_value);
		} else {
			memcpy(reg_5, (reg + 1), len_size);
			kstrtol(reg_5, 10, &define_value);
		}
	}
	/*CAM_DEBUG("isx012_define_read end (0x%x)!\n", define_value);*/

	return define_value;
}

static int isx012_write_regs_from_sd(char *name)
{
	char *start, *end, *reg, *size;
	unsigned short addr;
	unsigned int len, value;
	char reg_buf[7], data_buf1[5], data_buf2[7], len_buf[5];

	*(reg_buf + 6) = '\0';
	*(data_buf1 + 4) = '\0';
	*(data_buf2 + 6) = '\0';
	*(len_buf + 4) = '\0';

	CAM_DEBUG("[%s:%d]regs_table_write start!\n", __func__, __LINE__);
	CAM_DEBUG("E string = %s\n", name);
	printk(KERN_ERR "[%s:%d] - [%s]\n", __func__, __LINE__, name);

	start = strnstr(isx012_regs_table, name, isx012_regs_table_size);
	end = strnstr(start, "};", isx012_regs_table_size);

	while (1) {
		if (start >= end)
			break;

		/* Find Address */
		reg = strnstr(start, "{0x", isx012_regs_table_size);

		if ((reg == NULL) || (reg > end))
			break;

		/* Write Value to Address */
		if (reg != NULL) {
			memcpy(reg_buf, (reg + 1), 6);
			memcpy(data_buf2, (reg + 9), 6);
			size = strnstr(data_buf2, ",", 6);
			if (size) { /* 1 byte write */
				memcpy(data_buf1, (reg + 9), 4);
				memcpy(len_buf, (reg + 15), 4);
				kstrtol(reg_buf, 16, &addr);
				kstrtol(data_buf1, 16, &value);
				kstrtol(len_buf, 16, &len);
				if (reg)/*{0x000b,0x04,0x01},*/
					start = (reg + 22);
			} else {/* 2 byte write */
				memcpy(len_buf, (reg + 17), 4);
				kstrtol(reg_buf, 16, &addr);
				kstrtol(data_buf2, 16, &value);
				kstrtol(len_buf, 16, &len);
				if (reg)/*{0x000b,0x0004,0x01},*/
					start = (reg + 24);
			}
			size = NULL;

			if (addr == 0xFFFF)
				msleep(value);
			else
				isx012_i2c_write_multi(addr, value, len);
		}
	}

	CAM_DEBUG("[%s:%d]isx012_regs_table_write end!\n", __func__, __LINE__);

	return 0;
}
#endif

void isx012_get_LuxValue(void)
{
	int err = -1;
	unsigned short read_val = 0;

	err = isx012_i2c_read(0x01A5, &read_val);
	if (err < 0)
		cam_err(" i2c read returned error, %d", err);

	isx012_ctrl->lux = 0x00FF & read_val;
	CAM_DEBUG(" Lux = %d", isx012_ctrl->lux);
}

void isx012_get_LowLightCondition_Normal(void)
{
	CAM_DEBUG("EV value is %d", isx012_ctrl->setting.brightness);

	switch (isx012_ctrl->setting.brightness) {
	case EV_MINUS_4:
		CAM_DEBUG(" EV_M4");
		if (isx012_ctrl->lux >= GLOWLIGHT_EV_M4)
			gLowLight_check = 1;
		else
			gLowLight_check = 0;
		break;

	case EV_MINUS_3:
		CAM_DEBUG(" EV_M3");
		if (isx012_ctrl->lux >= GLOWLIGHT_EV_M3)
			gLowLight_check = 1;
		else
			gLowLight_check = 0;
		break;

	case EV_MINUS_2:
		CAM_DEBUG(" EV_M2");
		if (isx012_ctrl->lux >= GLOWLIGHT_EV_M2)
			gLowLight_check = 1;
		else
			gLowLight_check = 0;
		break;

	case EV_MINUS_1:
		CAM_DEBUG(" EV_M1");
		if (isx012_ctrl->lux >= GLOWLIGHT_EV_M1)
			gLowLight_check = 1;
		else
			gLowLight_check = 0;
		break;

	case EV_PLUS_1:
		CAM_DEBUG(" EV_P1");
		if (isx012_ctrl->lux >= GLOWLIGHT_EV_P1)
			gLowLight_check = 1;
		else
			gLowLight_check = 0;
		break;

	case EV_PLUS_2:
		CAM_DEBUG(" EV_P2");
		if (isx012_ctrl->lux >= GLOWLIGHT_EV_P2)
			gLowLight_check = 1;
		else
			gLowLight_check = 0;
		break;

	case EV_PLUS_3:
		CAM_DEBUG(" EV_P3");
		if (isx012_ctrl->lux >= GLOWLIGHT_EV_P3)
			gLowLight_check = 1;
		else
			gLowLight_check = 0;
		break;

	case EV_PLUS_4:
		CAM_DEBUG(" EV_P4");
		if (isx012_ctrl->lux >= GLOWLIGHT_EV_P4)
			gLowLight_check = 1;
		else
			gLowLight_check = 0;
		break;

	default:
		CAM_DEBUG(" default");
		if (isx012_ctrl->lux >= GLOWLIGHT_DEFAULT)
			gLowLight_check = 1;
		else
			gLowLight_check = 0;
		break;
	}
}

static int isx012_get_LowLightCondition()
{
	int err = -1;
	unsigned char r_data2[2] = {0, 0};
	unsigned char l_data[2] = {0, 0}, h_data[2] = {0, 0};
	unsigned int LowLight_value = 0;
	unsigned int ldata_temp = 0, hdata_temp = 0;

	isx012_get_LuxValue();

	if (isx012_ctrl->setting.iso == 0) {	/*auto iso*/
		isx012_get_LowLightCondition_Normal();
	} else {	/*manual iso*/
		CAM_DEBUG("[%s:%d] manual iso %d\n",
			__func__, __LINE__, isx012_ctrl->setting.iso);
		/*SHT_TIME_OUT_L*/
		err = isx012_i2c_read_multi(0x019C, l_data, 2);
		ldata_temp = (l_data[1] << 8 | l_data[0]);

		/*SHT_TIME_OUT_H*/
		err = isx012_i2c_read_multi(0x019E, h_data, 2);
		hdata_temp =
			(h_data[1] << 8 | h_data[0]);
		LowLight_value =
			(h_data[1] << 24 | h_data[0] << 16 |
			l_data[1] << 8 | l_data[0]);
		/*printk(KERN_ERR
		"func(%s):line(%d) LowLight_value : 0x%x
		/ hdata_temp : 0x%x ldata_temp : 0x%x\n",
		__func__, __LINE__, LowLight_value, hdata_temp, ldata_temp);*/

		switch (isx012_ctrl->setting.iso) {
		case ISO_50:
			if (LowLight_value >= gGLOWLIGHT_ISO50)
				gLowLight_check = 1;
			else
				gLowLight_check = 0;
			break;

		case ISO_100:
			if (LowLight_value >= gGLOWLIGHT_ISO100)
				gLowLight_check = 1;
			else
				gLowLight_check = 0;
			break;

		case ISO_200:
			if (LowLight_value >= gGLOWLIGHT_ISO200)
				gLowLight_check = 1;
			else
				gLowLight_check = 0;
			break;

		case ISO_400:
			if (LowLight_value >= gGLOWLIGHT_ISO400)
				gLowLight_check = 1;
			else
				gLowLight_check = 0;
			break;

		default:
			printk(KERN_DEBUG
				"[%s:%d] invalid iso[%d]\n",
				__func__, __LINE__, isx012_ctrl->setting.iso);
			break;
		}
	}
	CAM_DEBUG(
		"func(%s):line(%d) LowLight_value : 0x%x gLowLight_check : %d\n",
		__func__, __LINE__, LowLight_value, gLowLight_check);

	return err;
}

int isx012_mode_transition_OM(void)
{
	int timeout_cnt = 0;
	int om_status = 0;
	int ret = 0;
	short unsigned int r_data[2] = {0, 0};

	CAM_DEBUG("%s[E] %d\n", __func__, __LINE__);

	timeout_cnt = 0;
	do {
		if (timeout_cnt > 0)
			mdelay(1);

		timeout_cnt++;
		ret = isx012_i2c_read_multi(0x000E, r_data, 1);
		om_status = r_data[0];
		/*CAM_DEBUG("%s  0x000E (1) read : 0x%x /
		om_status & 0x1 : 0x%x(origin:0x1)\n",
		__func__, om_status, om_status & 0x1);*/
		if (timeout_cnt > ISX012_DELAY_RETRIES_OM) {
			CAM_DEBUG("%s: %d :Entering OM_1 delay timed out\n",
				__func__, __LINE__);
			break;
		}
	} while (((om_status & 0x01) != 0x01));

	if (ret != 1) {
		pr_info("factory test error(%d)\n", ret);
		return -EIO; /*factory test*/
	}
	timeout_cnt = 0;

	do {
		if (timeout_cnt > 0)
			mdelay(1);

		timeout_cnt++;
		isx012_i2c_write_multi(0x0012, 0x01, 0x01);
		isx012_i2c_read_multi(0x000E, r_data, 1);
		om_status = r_data[0];
		/*CAM_DEBUG("%s  0x000E (2) read : 0x%x /
		om_status & 0x1 : 0x%x(origin:0x0)\n",
		__func__, om_status, om_status & 0x1);*/
		if (timeout_cnt > ISX012_DELAY_RETRIES_OM) {
			printk(KERN_DEBUG "%s: %d :Entering OM_2 delay timed out\n",
				__func__, __LINE__);
			break;
		}
	} while (((om_status & 0x01) != 0x00));

	CAM_DEBUG("[%s:%d][X]\n", __func__, __LINE__);
	return 0; /*factory test*/
}

void isx012_wait_for_VINT(void)
{
	int timeout_cnt = 0;
	int cm_status = 0;
	short unsigned int r_data[2] = {0, 0};

	CAM_DEBUG("[%s][%d][E]\n", __func__, __LINE__);

	timeout_cnt = 0;
	do {
		if (timeout_cnt > 0)
			mdelay(10);

		timeout_cnt++;
		isx012_i2c_read_multi(0x000E, r_data, 1);
		cm_status = r_data[0];
		/*printk(KERN_DEBUG
		"%s  0x000E (1) read : 0x%x /"
		" cm_status & 0x2 : 0x%x(origin:0x2)\n",
		__func__, cm_status, cm_status & 0x20);*/
		if (timeout_cnt > 1000) {
			printk(KERN_DEBUG "%s: %d :Entering CM_1 delay timed out\n",
				__func__, __LINE__);
			break;
		}
	} while (((cm_status & 0x20) != 0x20));
	CAM_DEBUG(
		"[%s]0x000E (1) read : 0x%x /cm_status & 0x2 "
		": 0x%x(origin:0x2) count(%d)\n",
		__func__, cm_status, cm_status & 0x20, timeout_cnt);

	timeout_cnt = 0;
	do {
		if (timeout_cnt > 0)
			mdelay(10);

		timeout_cnt++;
		isx012_i2c_write_multi(0x0012, 0x20, 0x01);
		isx012_i2c_read_multi(0x000E, r_data, 1);
		cm_status = r_data[0];
		/*CAM_DEBUG("%s  0x000E (2) read : 0x%x
		/cm_status & 0x2 : 0x%x(origin:0x0)\n",
		__func__, cm_status, cm_status & 0x20);*/
		if (timeout_cnt > 1000) {
			printk(KERN_DEBUG "%s: %d :Entering CM_2 delay timed out\n",
				__func__, __LINE__);
			break;
		}
	} while (((cm_status & 0x20) != 0x0));
	CAM_DEBUG(
		"%s  0x000E (2) read : 0x%x /"
		" cm_status & 0x2 : 0x%x(origin:0x0) count(%d)\n",
		__func__, cm_status, cm_status & 0x20, timeout_cnt);
	CAM_DEBUG("[%s:%d][X]\n", __func__, __LINE__);
}

void isx012_mode_transition_CM(void)
{
	int timeout_cnt = 0;
	int cm_status = 0;
	short unsigned int r_data[2] = {0, 0};

	CAM_DEBUG("[%s:%d][E]d\n", __func__, __LINE__);

	timeout_cnt = 0;
	do {
		if (timeout_cnt > 0)
			mdelay(10);

		timeout_cnt++;
		isx012_i2c_read_multi(0x000E, r_data, 1);
		cm_status = r_data[0];
		/*CAM_DEBUG("%s  0x000E (1) read : 0x%x
		/ cm_status & 0x2 : 0x%x(origin:0x2)\n",
		__func__, cm_status, cm_status & 0x2);*/
		if (timeout_cnt > ISX012_DELAY_RETRIES_CM) {
			printk(KERN_DEBUG "[%s:%d] Entering CM_1 delay timed out\n",
				__func__, __LINE__);
			break;
		}
	} while (((cm_status & 0x2) != 0x2));
	CAM_DEBUG(
		"[%s] 0x000E (1) read : 0x%x "
		"/cm_status & 0x2 : 0x%x(origin:0x2) count(%d)\n",
		__func__, cm_status, cm_status & 0x2, timeout_cnt);

	timeout_cnt = 0;
	do {
		if (timeout_cnt > 0)
			mdelay(10);

		timeout_cnt++;
		isx012_i2c_write_multi(0x0012, 0x02, 0x01);
		isx012_i2c_read_multi(0x000E, r_data, 1);
		cm_status = r_data[0];
		/*printk(KERN_DEBUG
		"%s  0x000E (2) read : 0x%x /"
		" cm_status & 0x2 : 0x%x(origin:0x0)\n",
		__func__, cm_status, cm_status & 0x2);*/
		if (timeout_cnt > ISX012_DELAY_RETRIES_CM) {
			printk(KERN_DEBUG "%s: %d :Entering CM_2 delay timed out\n",
				__func__, __LINE__);
			break;
		}
	} while (((cm_status & 0x2) != 0x0));
	CAM_DEBUG(
		"[%s] 0x000E (2) read : 0x%x /"
		" cm_status & 0x2 : 0x%x(origin:0x0) count(%d)\n",
		__func__, cm_status, cm_status & 0x2, timeout_cnt);
	CAM_DEBUG("[%s:%d][X]\n", __func__, __LINE__);
}

void isx012_Sensor_Calibration(void)
{
	/*int count = 0;*/
	int status = 0;
	int temp = 0;

	CAM_DEBUG("[%s:%d][E]\n", __func__, __LINE__);

	/* Read OTP1 */
	isx012_i2c_read(0x004F, (unsigned short *)&status);
	CAM_DEBUG("[%s:%d]0x004F read : %x\n", __func__, __LINE__, status);

	if ((status & 0x10) == 0x10) {
		/* Read ShadingTable */
		isx012_i2c_read(0x005C, (unsigned short *)&status);
		temp = (status&0x03C0)>>6;
		CAM_DEBUG(
			"[%s:%d] Read ShadingTable read : %x\n",
			__func__, __LINE__, temp);

		/* Write Shading Table */
		if (temp == 0x0) {
			ISX012_BURST_WRITE_LIST(ISX012_Shading_0);
			CAM_DEBUG(
			"[%s:%d]ISX012_Shading_0\n", __func__, __LINE__);
		} else if (temp == 0x1) {
			ISX012_BURST_WRITE_LIST(ISX012_Shading_1);
			CAM_DEBUG(
			"[%s:%d]ISX012_Shading_1\n", __func__, __LINE__);
		} else if (temp == 0x2) {
			ISX012_BURST_WRITE_LIST(ISX012_Shading_2);
			CAM_DEBUG(
			"[%s:%d]ISX012_Shading_2\n", __func__, __LINE__);
		}

		/* Write NorR */
		isx012_i2c_read(0x0054, (unsigned short *)&status);
		temp = status&0x3FFF;
		CAM_DEBUG("[%s:%d]NorR read : %x\n", __func__, __LINE__, temp);
		isx012_i2c_write_multi(0x6804, temp, 0x02);

		/* Write NorB */
		isx012_i2c_read(0x0056, (unsigned short *)&status);
		temp = status&0x3FFF;
		CAM_DEBUG("[%s:%d]NorB read : %x\n", __func__, __LINE__, temp);
		isx012_i2c_write_multi(0x6806, temp, 0x02);

		/* Write PreR */
		isx012_i2c_read(0x005A, (unsigned short *)&status);
		temp = (status&0x0FFC)>>2;
		CAM_DEBUG("[%s:%d]PreR read : %x\n", __func__, __LINE__, temp);
		isx012_i2c_write_multi(0x6808, temp, 0x02);

		/* Write PreB */
		isx012_i2c_read(0x005B, (unsigned short *)&status);
		temp = (status&0x3FF0)>>4;
		CAM_DEBUG("[%s:%d]PreB read : %x\n", __func__, __LINE__, temp);
		isx012_i2c_write_multi(0x680A, temp, 0x02);
	} else {
		/* Read OTP0 */
		isx012_i2c_read(0x0040, (unsigned short *)&status);
		CAM_DEBUG(
			"[%s:%d]0x0040 read : %x\n",
			__func__, __LINE__, status);

		if ((status & 0x10) == 0x10) {
			/* Read ShadingTable */
			isx012_i2c_read(0x004D, (unsigned short *)&status);
			temp = (status&0x03C0)>>6;
			CAM_DEBUG(
			"[%s:%d]Read ShadingTable read : %x\n",
			__func__, __LINE__, temp);

			/* Write Shading Table */
			if (temp == 0x0) {
				ISX012_BURST_WRITE_LIST(ISX012_Shading_0);
				CAM_DEBUG("[%s:%d]ISX012_Shading_0\n",
					__func__, __LINE__);
			} else if (temp == 0x1) {
				ISX012_BURST_WRITE_LIST(ISX012_Shading_1);
				CAM_DEBUG("[%s:%d]ISX012_Shading_1\n",
					__func__, __LINE__);
			} else if (temp == 0x2) {
				ISX012_BURST_WRITE_LIST(ISX012_Shading_2);
				CAM_DEBUG("[%s:%d]ISX012_Shading_2\n",
					__func__, __LINE__);
			}

			/* Write NorR */
			isx012_i2c_read(0x0045, (unsigned short *)&status);
			temp = status&0x3FFF;
			CAM_DEBUG("[%s:%d]NorR read : %x\n",
				__func__, __LINE__, temp);
			isx012_i2c_write_multi(0x6804, temp, 0x02);

			/* Write NorB */
			isx012_i2c_read(0x0047, (unsigned short *)&status);
			temp = status&0x3FFF;
			CAM_DEBUG("[%s:%d]NorB read : %x\n",
				__func__, __LINE__, temp);
			isx012_i2c_write_multi(0x6806, temp, 0x02);

			/* Write PreR */
			isx012_i2c_read(0x004B, (unsigned short *)&status);
			temp = (status&0x0FFC)>>2;
			CAM_DEBUG("[%s:%d]PreR read : %x\n",
				__func__, __LINE__, temp);
			isx012_i2c_write_multi(0x6808, temp, 0x02);

			/* Write PreB */
			isx012_i2c_read(0x004C, (unsigned short *)&status);
			temp = (status&0x3FF0)>>4;
			CAM_DEBUG("[%s:%d]PreB read : %x\n",
				__func__, __LINE__, temp);
			isx012_i2c_write_multi(0x680A, temp, 0x02);
		} else {
			ISX012_BURST_WRITE_LIST(ISX012_Shading_Nocal);
		}
	}
}

/*
static int32_t isx012_i2c_write_32bit(unsigned long packet)
{
	int32_t err = -EFAULT;
	//int retry_count = 1;
	//int i;

	unsigned char buf[4];
	struct i2c_msg msg = {
		.addr = isx012_client->addr,
		.flags = 0,
		.len = 4,
		.buf = buf,
	};
	*(unsigned long *)buf = cpu_to_be32(packet);

	//for(i=0; i< retry_count; i++) {
	err = i2c_transfer(isx012_client->adapter, &msg, 1);
	//}

	return err;
}
*/



/*


static int32_t isx012_i2c_write(unsigned short subaddr, unsigned short val)
{
	unsigned long packet;
	packet = (subaddr << 16) | (val&0xFFFF);

	return isx012_i2c_write_32bit(packet);
}


static int32_t isx012_i2c_read(unsigned short subaddr, unsigned short *data)
{

	int ret;
	unsigned char buf[2];

	struct i2c_msg msg = {
		.addr = isx012_client->addr,
		.flags = 0,
		.len = 2,
		.buf = buf,
	};

	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);

	ret = i2c_transfer(isx012_client->adapter, &msg, 1) == 1 ? 0 : -EIO;

	if (ret == -EIO)
	    goto error;

	msg.flags = I2C_M_RD;

	ret = i2c_transfer(isx012_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO)
	    goto error;

	*data = ((buf[0] << 8) | buf[1]);


error:
	return ret;

}

*/

/*
static int isx012_set_ae_lock(char value)
{
	int err = -EINVAL;
	CAM_DEBUG("%d", value);
	CAM_DEBUG("isx012_set_ae_lock ");
	switch (value) {
		case EXT_CFG_AE_LOCK:
			isx012_ctrl->setting.ae_lock = EXT_CFG_AE_LOCK;
			 ISX012_BURST_WRITE_LIST(isx012_ae_lock);
			break;
		case EXT_CFG_AE_UNLOCK:
			isx012_ctrl->setting.ae_lock = EXT_CFG_AE_UNLOCK;
			ISX012_BURST_WRITE_LIST(isx012_ae_unlock);
			break;
		case EXT_CFG_AWB_LOCK:
			isx012_ctrl->setting.awb_lock = EXT_CFG_AWB_LOCK;
		       ISX012_BURST_WRITE_LIST(isx012_awb_lock);
			break;
		case EXT_CFG_AWB_UNLOCK:
			isx012_ctrl->setting.awb_lock = EXT_CFG_AWB_UNLOCK;
			ISX012_BURST_WRITE_LIST(isx012_awb_unlock);
			break;
		default:
			cam_err("Invalid(%d)", value);
			break;
	}
	return err;
}
*/

static long isx012_set_effect(int8_t value)
{
	int err = -EINVAL;
	CAM_DEBUG("[%s:%d] value[%d]\n", __func__, __LINE__, value);

	switch (value) {
	case EFFECT_OFF:
		ISX012_BURST_WRITE_LIST(isx012_Effect_Normal);
		isx012_ctrl->setting.effect = value;
		err = 0;
		break;

	case EFFECT_SEPIA:
		ISX012_BURST_WRITE_LIST(isx012_Effect_Sepia);
		isx012_ctrl->setting.effect = value;
		err = 0;
		break;

	case EFFECT_MONO:
		ISX012_BURST_WRITE_LIST(isx012_Effect_Black_White);
		isx012_ctrl->setting.effect = value;
		err = 0;
		break;

	case EFFECT_NEGATIVE:
		ISX012_BURST_WRITE_LIST(ISX012_Effect_Negative);
		isx012_ctrl->setting.effect = value;
		err = 0;
		break;

	default:
		printk(KERN_DEBUG
			"[%s:%d] invalid[%d]\n",
			__func__, __LINE__, value);
		err = 0;
		break;
	}

	return err;
}

static int isx012_set_whitebalance(int8_t value)
{
	int err = 0;

	switch (value) {
	case WHITE_BALANCE_AUTO:
		ISX012_BURST_WRITE_LIST(isx012_WB_Auto);
		break;

	case WHITE_BALANCE_SUNNY:
		ISX012_BURST_WRITE_LIST(isx012_WB_Sunny);
		break;

	case WHITE_BALANCE_CLOUDY:
		ISX012_BURST_WRITE_LIST(isx012_WB_Cloudy);
		break;

	case WHITE_BALANCE_FLUORESCENT:
		ISX012_BURST_WRITE_LIST(isx012_WB_Fluorescent);
		break;

	case WHITE_BALANCE_INCANDESCENT:
		ISX012_BURST_WRITE_LIST(isx012_WB_Tungsten);
		break;

	default:
		printk(KERN_DEBUG "[%s:%d] invalid[%d]\n",
			__func__, __LINE__, value);
		break;
	}
	isx012_ctrl->setting.whiteBalance = value;
	return err;
}

static int isx012_exif_shutter_speed(void)
{
	uint32_t shutter_speed = 0;
	int err = 0;

	unsigned char l_data[2] = {0, 0}, h_data[2] = {0, 0};

	err = isx012_i2c_read_multi(0x019C, l_data, 2);	/*SHT_TIME_OUT_L*/
	err = isx012_i2c_read_multi(0x019E, h_data, 2);	/*SHT_TIME_OUT_H*/
	shutter_speed =
		(h_data[1] << 24 | h_data[0] << 16 |
		l_data[1] << 8 | l_data[0]);
	CAM_DEBUG(
		"[%s:%d]shutter_speed(%x/%d)\n",
		__func__, __LINE__, shutter_speed, shutter_speed);

	isx012_exif->shutterspeed = shutter_speed;

	return 0;
}

static int isx012_set_flash(int8_t value1, int8_t value2)
{
	int err = -EINVAL;
	int i = 0;
	int torch = 0, torch2 = 0, torch3 = 0;

	/* Accessary concept */
	if (accessibility_torch) {
		isx012_ctrl->setting.flash_mode = FLASH_MODE_OFF_S;
		CAM_DEBUG("[%s:%d] Accessibility torch(%d)\n",
			__func__, __LINE__, isx012_ctrl->setting.flash_mode);
		return 0;
	}

	if (((value1 == 50) && (value2 == FLASH_TORCH_MODE_ON_S)) ||
		((value1 == FLASH_TORCH_MODE_ON_S) && (value2 == 50))) {
		CAM_DEBUG(
			"[%s:%d] Movie Mode Flash v1/v2[%d/%d]\n",
			__func__, __LINE__, value1, value2);
		gpio_set_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(
				PMIC_GPIO_CAM_FLASH_EN), 0);

		for (i = 5; i > 1; i--) {
			gpio_set_value_cansleep(
				PM8058_GPIO_PM_TO_SYS(
					PMIC_GPIO_CAM_FLASH_SET), 1);
			udelay(1);
			gpio_set_value_cansleep(
				PM8058_GPIO_PM_TO_SYS(
					PMIC_GPIO_CAM_FLASH_SET), 0);
			udelay(1);
		}
		gpio_set_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(
				PMIC_GPIO_CAM_FLASH_SET), 1);
		usleep(2*1000);
	}
	if (((value1 == 50) && (value2 == FLASH_MODE_OFF_S)) ||
		((value1 == FLASH_MODE_OFF_S) && (value2 == 50))) {
		gpio_set_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(
				PMIC_GPIO_CAM_FLASH_EN), 0);
		gpio_set_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(
				PMIC_GPIO_CAM_FLASH_SET), 0);
	}
	if ((value1 > -1) && (value2 == 50)) {
		switch (value1) {
		case FLASH_MODE_AUTO_S: /*Auto*/
			if (gLowLight_check) {
				gpio_set_value_cansleep(
					PM8058_GPIO_PM_TO_SYS(
						PMIC_GPIO_CAM_FLASH_EN), 1);
				gpio_set_value_cansleep(
					PM8058_GPIO_PM_TO_SYS(
						PMIC_GPIO_CAM_FLASH_SET), 0);
				isx012_exif->flash = 1;
			} else {
				gpio_set_value_cansleep(
					PM8058_GPIO_PM_TO_SYS(
						PMIC_GPIO_CAM_FLASH_EN), 0);
				gpio_set_value_cansleep(
					PM8058_GPIO_PM_TO_SYS(
						PMIC_GPIO_CAM_FLASH_SET), 0);
				isx012_exif->flash = 0;
			}
			err = 0;
			break;

		case FLASH_MODE_ON_S:	/*on*/
			gpio_set_value_cansleep(
				PM8058_GPIO_PM_TO_SYS(
					PMIC_GPIO_CAM_FLASH_EN), 1);
			gpio_set_value_cansleep(
				PM8058_GPIO_PM_TO_SYS(
					PMIC_GPIO_CAM_FLASH_SET), 0);
			err = 0;
			break;
		default:
			printk(KERN_DEBUG
				"[%s:%d] invalid[%d/%d]\n",
				__func__, __LINE__, value1, value2);
			err = 0;
			break;
		}
	} else if ((value1 == 50 && value2 > -1)) {
		isx012_ctrl->setting.flash_mode = value2;
		CAM_DEBUG("[%s:%d]flash value1(%d) value2(%d)"
			" isx012_ctrl->setting.flash_mode(%d)",
			__func__, __LINE__, value1, value2,
			isx012_ctrl->setting.flash_mode);
		err = 0;
	}

	CAM_DEBUG("[%s:%d]"
		"FINAL flash value1(%d) value2(%d) "
		"isx012_ctrl->setting.flash_mode(%d)", __func__, __LINE__,
		value1, value2, isx012_ctrl->setting.flash_mode);

	return err;
}

static int  isx012_set_brightness(int8_t value)
{
	int err = -EINVAL;
	CAM_DEBUG("[%s:%d] value[%d]\n", __func__, __LINE__, value);

	switch (value) {
	case EV_MINUS_4:
		ISX012_BURST_WRITE_LIST(ISX012_ExpSetting_M4Step);
		isx012_ctrl->setting.brightness = value;
		err = 0;
		break;

	case EV_MINUS_3:
		ISX012_BURST_WRITE_LIST(ISX012_ExpSetting_M3Step);
		isx012_ctrl->setting.brightness = value;
		err = 0;
		break;

	case EV_MINUS_2:
		ISX012_BURST_WRITE_LIST(ISX012_ExpSetting_M2Step);
		isx012_ctrl->setting.brightness = value;
		err = 0;
		break;

	case EV_MINUS_1:
		ISX012_BURST_WRITE_LIST(ISX012_ExpSetting_M1Step);
		isx012_ctrl->setting.brightness = value;
		err = 0;
		break;

	case EV_DEFAULT:
		ISX012_BURST_WRITE_LIST(ISX012_ExpSetting_Default);
		isx012_ctrl->setting.brightness = value;
		err = 0;
		break;

	case EV_PLUS_1:
		ISX012_BURST_WRITE_LIST(ISX012_ExpSetting_P1Step);
		isx012_ctrl->setting.brightness = value;
		err = 0;
		break;

	case EV_PLUS_2:
		ISX012_BURST_WRITE_LIST(ISX012_ExpSetting_P2Step);
		isx012_ctrl->setting.brightness = value;
		err = 0;
		break;

	case EV_PLUS_3:
		ISX012_BURST_WRITE_LIST(ISX012_ExpSetting_P3Step);
		isx012_ctrl->setting.brightness = value;
		err = 0;
		break;

	case EV_PLUS_4:
		ISX012_BURST_WRITE_LIST(ISX012_ExpSetting_P4Step);
		isx012_ctrl->setting.brightness = value;
		err = 0;
		break;

	default:
		printk(KERN_DEBUG
			"[%s:%d] invalid[%d]\n",
			__func__, __LINE__, value);
		break;
	}

	return err;
}


static int  isx012_set_iso(int8_t value)
{
	int err = -EINVAL;
	CAM_DEBUG("[%s:%d] value[%d]\n", __func__, __LINE__, value);

	switch (value) {
	case ISO_AUTO:
		ISX012_BURST_WRITE_LIST(isx012_ISO_Auto);
		err = 0;
		break;

	case ISO_50:
		ISX012_BURST_WRITE_LIST(isx012_ISO_50);
		err = 0;
		break;

	case ISO_100:
		ISX012_BURST_WRITE_LIST(isx012_ISO_100);
		err = 0;
		break;

	case ISO_200:
		ISX012_BURST_WRITE_LIST(isx012_ISO_200);
		err = 0;
		break;

	case ISO_400:
		ISX012_BURST_WRITE_LIST(isx012_ISO_400);
		err = 0;
		break;

	default:
		printk(KERN_DEBUG
			"[%s:%d] invalid[%d]\n",
			__func__, __LINE__, value);
		break;
	}

	isx012_ctrl->setting.iso = value;
	return err;
}


static int isx012_set_metering(int8_t value)
{
	int err = -EINVAL;
	CAM_DEBUG("[%s:%d] value[%d]\n", __func__, __LINE__, value);

	switch (value) {
	case METERING_MATRIX:
		ISX012_BURST_WRITE_LIST(isx012_Metering_Matrix);
		isx012_ctrl->setting.metering = value;
		err = 0;
		break;

	case METERING_CENTER:
		ISX012_BURST_WRITE_LIST(isx012_Metering_Center);
		isx012_ctrl->setting.metering = value;
		err = 0;
		break;

	case METERING_SPOT:
		ISX012_BURST_WRITE_LIST(isx012_Metering_Spot);
		isx012_ctrl->setting.metering = value;
		err = 0;
		break;

	default:
		printk(KERN_DEBUG
			"[%s:%d] invalid[%d]\n",
			__func__, __LINE__, value);
		err = 0;
		break;
	}

	return err;
}



static int isx012_set_contrast(int8_t value)
{
	int err = -EINVAL;
	CAM_DEBUG("[%s:%d] value[%d]\n", __func__, __LINE__, value);

	switch (value) {
	case CONTRAST_MINUS_2:
		ISX012_BURST_WRITE_LIST(isx012_Contrast_Minus_2);
		isx012_ctrl->setting.contrast = value;
		err = 0;
		break;

	case CONTRAST_MINUS_1:
		ISX012_BURST_WRITE_LIST(isx012_Contrast_Minus_1);
		isx012_ctrl->setting.contrast = value;
		err = 0;
		break;

	case CONTRAST_DEFAULT:
		ISX012_BURST_WRITE_LIST(isx012_Contrast_Default);
		isx012_ctrl->setting.contrast = value;
		err = 0;
		break;

	case CONTRAST_PLUS_1:
		ISX012_BURST_WRITE_LIST(isx012_Contrast_Plus_1);
		isx012_ctrl->setting.contrast = value;
		err = 0;
		break;

	case CONTRAST_PLUS_2:
		ISX012_BURST_WRITE_LIST(isx012_Contrast_Plus_2);
		isx012_ctrl->setting.contrast = value;
		err = 0;
		break;

	default:
		printk(KERN_DEBUG
			"[%s:%d] invalid[%d]\n",
			__func__, __LINE__, value);
		err = 0;
		break;
	}

	return err;
}


static int isx012_set_saturation(int8_t value)
{
	int err = -EINVAL;
	CAM_DEBUG("[%s:%d] value[%d]\n", __func__, __LINE__, value);

	switch (value) {
	case SATURATION_MINUS_2:
		ISX012_BURST_WRITE_LIST(isx012_Saturation_Minus_2);
		isx012_ctrl->setting.saturation = value;
		err = 0;
		break;

	case SATURATION_MINUS_1:
		ISX012_BURST_WRITE_LIST(isx012_Saturation_Minus_1);
		isx012_ctrl->setting.saturation = value;
		err = 0;
		break;

	case SATURATION_DEFAULT:
		ISX012_BURST_WRITE_LIST(isx012_Saturation_Default);
		isx012_ctrl->setting.saturation = value;
		err = 0;
		break;

	case SATURATION_PLUS_1:
		ISX012_BURST_WRITE_LIST(isx012_Saturation_Plus_1);
		isx012_ctrl->setting.saturation = value;
		err = 0;
		break;

	case SATURATION_PLUS_2:
		ISX012_BURST_WRITE_LIST(isx012_Saturation_Plus_2);
		isx012_ctrl->setting.saturation = value;
		err = 0;
		break;

	default:
		printk(KERN_DEBUG
			"[%s:%d] invalid[%d]\n",
			__func__, __LINE__, value);
		err = 0;
		break;
	}

	return err;
}


static int isx012_set_sharpness(int8_t value)
{
	int err = -EINVAL;
	CAM_DEBUG("[%s:%d] value[%d]\n", __func__, __LINE__, value);

	switch (value) {
	case SHARPNESS_MINUS_2:
		ISX012_BURST_WRITE_LIST(isx012_Sharpness_Minus_2);
		isx012_ctrl->setting.sharpness = value;
		err = 0;
		break;

	case SHARPNESS_MINUS_1:
		ISX012_BURST_WRITE_LIST(isx012_Sharpness_Minus_1);
		isx012_ctrl->setting.sharpness = value;
		err = 0;
		break;

	case SHARPNESS_DEFAULT:
		ISX012_BURST_WRITE_LIST(isx012_Sharpness_Default);
		isx012_ctrl->setting.sharpness = value;
		err = 0;
		break;

	case SHARPNESS_PLUS_1:
		ISX012_BURST_WRITE_LIST(isx012_Sharpness_Plus_1);
		isx012_ctrl->setting.sharpness = value;
		err = 0;
		break;

	case SHARPNESS_PLUS_2:
		ISX012_BURST_WRITE_LIST(isx012_Sharpness_Plus_2);
		isx012_ctrl->setting.sharpness = value;
		err = 0;
		break;

	default:
		printk(KERN_DEBUG
			"[%s:%d] invalid[%d]\n",
			__func__, __LINE__, value);
		err = 0;
		break;
	}

	return err;
}




static int  isx012_set_scene(int8_t value)
{
	int err = -EINVAL;
	CAM_DEBUG("[%s:%d] value[%d]\n", __func__, __LINE__, value);

	if (value != SCENE_OFF)
		ISX012_BURST_WRITE_LIST(isx012_Scene_Default);

	switch (value) {
	case SCENE_OFF:
		ISX012_BURST_WRITE_LIST(isx012_Scene_Default);
		err = 0;
		break;

	case SCENE_PORTRAIT:
		ISX012_BURST_WRITE_LIST(isx012_Scene_Portrait);
		err = 0;
		break;

	case SCENE_LANDSCAPE:
		ISX012_BURST_WRITE_LIST(isx012_Scene_Landscape);
		err = 0;
		break;

	case SCENE_SPORTS:
		ISX012_BURST_WRITE_LIST(isx012_Scene_Sports);
		err = 0;
		break;

	case SCENE_PARTY:
		ISX012_BURST_WRITE_LIST(isx012_Scene_Party_Indoor);
		err = 0;
		break;

	case SCENE_BEACH:
		ISX012_BURST_WRITE_LIST(isx012_Scene_Beach_Snow);
		err = 0;
		break;

	case SCENE_SUNSET:
		ISX012_BURST_WRITE_LIST(isx012_Scene_Sunset);
		err = 0;
		break;

	case SCENE_DAWN:
		ISX012_BURST_WRITE_LIST(isx012_Scene_Duskdawn);
		err = 0;
		break;

	case SCENE_FALL:
		ISX012_BURST_WRITE_LIST(isx012_Scene_Fall_Color);
		err = 0;
		break;

	case SCENE_NIGHT:
		ISX012_BURST_WRITE_LIST(isx012_Scene_Nightshot);
		err = 0;
		break;

	case SCENE_AGAINST_LIGHT:
		ISX012_BURST_WRITE_LIST(isx012_Scene_Backlight);
		err = 0;
		break;

	case SCENE_FIREWORK:
		ISX012_BURST_WRITE_LIST(isx012_Scene_Fireworks);
		err = 0;
		break;

	case SCENE_TEXT:
		ISX012_BURST_WRITE_LIST(isx012_Scene_Text);
		err = 0;
		break;

	case SCENE_CANDLE:
		ISX012_BURST_WRITE_LIST(isx012_Scene_Candle_Light);
		err = 0;
		break;

	default:
		printk(KERN_DEBUG
			"[%s:%d] invalid[%d]\n",
			__func__, __LINE__, value);
		break;
	}


	isx012_ctrl->setting.scene = value;

	return err;
}

static int isx012_set_preview_size(int32_t value1, int32_t value2)
{
	CAM_DEBUG("[%s:%d] value1[%d], value2[%d]\n",
		__func__, __LINE__,  value1, value2);

	if (value1 == 1280 && value2 == 720) {
		CAM_DEBUG("[%s:%d]"
			"isx012_set_preview_size isx012_1280_Preview_E\n",
			__func__, __LINE__);
		ISX012_BURST_WRITE_LIST(isx012_1280_Preview_E);
	} else if (value1 == 800 && value2 == 480) {
		CAM_DEBUG("[%s:%d]"
			"isx012_set_preview_size isx012_800_Preview_E\n",
			__func__, __LINE__);
		ISX012_BURST_WRITE_LIST(isx012_800_Preview);
	} else if (value1 == 720 && value2 == 480) {
		CAM_DEBUG("[%s:%d]"
			"isx012_set_preview_size isx012_720_Preview_E\n",
			__func__, __LINE__);
		ISX012_BURST_WRITE_LIST(isx012_720_Preview);
	} else {
		CAM_DEBUG("[%s:%d]"
			"isx012_set_preview_size isx012_640_Preview_E\n",
			__func__, __LINE__);
		ISX012_BURST_WRITE_LIST(isx012_640_Preview);
	}

	ISX012_BURST_WRITE_LIST(ISX012_Preview_Mode);
	isx012_mode_transition_CM();

/*
	if (HD_mode) {
		    HD_mode = 0;
		    ISX012_BURST_WRITE_LIST(isx012_1280_Preview_D)
	}

	switch (value1) {
	case 1280:
		//HD_mode = 1;
		printk(KERN_DEBUG
		"isx012_set_preview_size isx012_1280_Preview_E");
		ISX012_BURST_WRITE_LIST(isx012_1280_Preview_E);
		ISX012_BURST_WRITE_LIST(ISX012_Preview_Mode);
		break;
	case 720:
		printk(KERN_DEBUG
		"isx012_set_preview_size isx012_720_Preview_E");
		ISX012_BURST_WRITE_LIST(isx012_720_Preview);
		ISX012_BURST_WRITE_LIST(ISX012_Preview_Mode);
		break;
	case 640:
		printk(KERN_DEBUG
		"isx012_set_preview_size isx012_640_Preview_E");
		ISX012_BURST_WRITE_LIST(isx012_640_Preview);
		ISX012_BURST_WRITE_LIST(ISX012_Preview_Mode);
		break;
	case 320:
		printk(KERN_DEBUG
		"isx012_set_preview_size isx012_320_Preview_E");
		ISX012_BURST_WRITE_LIST(isx012_320_Preview);
		ISX012_BURST_WRITE_LIST(ISX012_Preview_Mode);
		break;
	case 176:
		printk(KERN_DEBUG
		"isx012_set_preview_size isx012_176_Preview_E");
		ISX012_BURST_WRITE_LIST(isx012_176_Preview);
		ISX012_BURST_WRITE_LIST(ISX012_Preview_Mode);
		break;
	default:
		cam_err("Invalid");
		break;

}

*/

	isx012_ctrl->setting.preview_size = value1;


	return 0;
}




static int isx012_set_picture_size(int value)
{
	CAM_DEBUG("[%s:%d] value[%d]\n", __func__, __LINE__, value);
/*
	switch (value) {
	case EXT_CFG_SNAPSHOT_SIZE_2560x1920_5M:
		ISX012_BURST_WRITE_LIST(isx012_5M_Capture);
		//isx012_set_zoom(EXT_CFG_ZOOM_STEP_0);
		break;
	case EXT_CFG_SNAPSHOT_SIZE_2560x1536_4M_WIDE:
		ISX012_BURST_WRITE_LIST(isx012_4M_WIDE_Capture);
		break;
	case EXT_CFG_SNAPSHOT_SIZE_2048x1536_3M:
		ISX012_BURST_WRITE_LIST(isx012_5M_Capture);
		break;
	case EXT_CFG_SNAPSHOT_SIZE_2048x1232_2_4M_WIDE:
		ISX012_BURST_WRITE_LIST(isx012_2_4M_WIDE_Capture);
		break;
	case EXT_CFG_SNAPSHOT_SIZE_1600x1200_2M:
		ISX012_BURST_WRITE_LIST(isx012_5M_Capture);
		break;
	case EXT_CFG_SNAPSHOT_SIZE_1600x960_1_5M_WIDE:
		ISX012_BURST_WRITE_LIST(isx012_1_5M_WIDE_Capture);
		break;
	case EXT_CFG_SNAPSHOT_SIZE_1280x960_1M:
		ISX012_BURST_WRITE_LIST(isx012_1M_Capture);
		break;
	case EXT_CFG_SNAPSHOT_SIZE_800x480_4K_WIDE:
		ISX012_BURST_WRITE_LIST(isx012_4K_WIDE_Capture);
		break;
	case EXT_CFG_SNAPSHOT_SIZE_640x480_VGA:
		ISX012_BURST_WRITE_LIST(isx012_VGA_Capture);
		break;
	case EXT_CFG_SNAPSHOT_SIZE_320x240_QVGA:
		ISX012_BURST_WRITE_LIST(isx012_QVGA_Capture);
		break;
	default:
		cam_err("Invalid");
		return -EINVAL;
	}


//	if (size != EXT_CFG_SNAPSHOT_SIZE_2560x1920_5M &&
			isx012_status.zoom != EXT_CFG_ZOOM_STEP_0)
//		isx012_set_zoom(isx012_status.zoom);


	isx012_ctrl->setting.snapshot_size = value;
*/
	return 0;
}

static int isx012_set_frame_rate(int value)
{
	CAM_DEBUG("[%s:%d] value[%d]\n", __func__, __LINE__, value);
	switch (value) {
	case 15:
		ISX012_BURST_WRITE_LIST(isx012_FRAME_15FPS_Setting);
		break;
	case 30:
		ISX012_BURST_WRITE_LIST(isx012_FRAME_30FPS_Setting);
		break;
	default:
		cam_err("Invalid");
		return -EINVAL;
	}
	return 0;
}

static int isx012_exif_iso(void)
{
	int exif_iso = 0;
	int err = 0;
	short unsigned int r_data[1] = {0};
	unsigned int iso_table[19] = {
		25, 32, 40, 50, 64, 80, 100, 125, 160,
		200, 250, 320, 400, 500, 640, 800, 1000, 1250, 1600};

	err = isx012_i2c_read(0x019A, r_data);	/*ISOSENE_OUT*/
	exif_iso = r_data[0];
	/*printk(KERN_DEBUG
	"[exif_iso][%s:%d] r_data(%d) exif_iso(%d)\n",
	__func__, __LINE__, r_data[0], exif_iso);*/

	isx012_exif->iso = iso_table[exif_iso-1];

	return exif_iso;
}

static int isx012_get_exif(int *exif_cmd, int *val)
{
	switch (*exif_cmd) {
	case EXIF_TV:
		/*isx012_exif_shutter_speed();*/
		(*val) = isx012_exif->shutterspeed;
		break;

	case EXIF_ISO:
		isx012_exif_iso();
		(*val) = isx012_exif->iso;
		break;

	case EXIF_FLASH:
		(*val) = isx012_exif->flash;
		break;
	/*
	case EXIF_EXPOSURE_TIME:
		(*val) = isx012_exif->exptime;
		break;

	case EXIF_TV:
		(*val) = isx012_exif->tv;
		break;

	case EXIF_BV:
		(*val) = isx012_exif->bv;
		break;

	case EXIF_EBV:
		(*val) = isx012_exif->ebv;
		break;
	*/
	default:
		printk(KERN_DEBUG
			"[%s:%d] invalid(%d)\n",
			__func__, __LINE__, *exif_cmd);
		break;
	}

	return 0;
}

static int isx012_cancel_autofocus(void)
{
	CAM_DEBUG("[%s:%d][E]\n", __func__, __LINE__);

	switch (isx012_ctrl->setting.afmode) {
	case FOCUS_AUTO:
		ISX012_BURST_WRITE_LIST(ISX012_AF_Cancel_Macro_OFF);
		ISX012_BURST_WRITE_LIST(ISX012_AF_Macro_OFF);
		ISX012_BURST_WRITE_LIST(ISX012_AF_ReStart);
		break;

	case FOCUS_MACRO:
		ISX012_BURST_WRITE_LIST(ISX012_AF_Cancel_Macro_ON);
		ISX012_BURST_WRITE_LIST(ISX012_AF_Macro_ON);
		ISX012_BURST_WRITE_LIST(ISX012_AF_ReStart);
		break;

	default:
		printk(KERN_DEBUG
			"[%s:%d] invalid[%d]\n",
			__func__, __LINE__, isx012_ctrl->setting.afmode);
	}

	CAM_DEBUG("[%s:%d][X]\n", __func__, __LINE__);

	return 0;
}

static int isx012_set_focus_mode(int value1, int value2)
{
	switch (value1) {
	case FOCUS_AUTO:
		ISX012_BURST_WRITE_LIST(ISX012_AF_Macro_OFF);
		if (value2 == 1) {
			ISX012_BURST_WRITE_LIST(ISX012_AF_ReStart);
			isx012_ctrl->setting.afmode = FOCUS_AUTO;
		}
		break;

	case FOCUS_MACRO:
		ISX012_BURST_WRITE_LIST(ISX012_AF_Macro_ON);
		if (value2 == 1) {
			ISX012_BURST_WRITE_LIST(ISX012_AF_ReStart);
			isx012_ctrl->setting.afmode = FOCUS_MACRO;
		}
		break;

	default:
		printk(KERN_DEBUG
			"[%s:%d] invalid[%d/%d]\n",
			__func__, __LINE__, value1, value2);
		break;
	}

	return 0;
}

static int isx012_set_movie_mode(int mode)
{
	CAM_DEBUG("[%s:%d][E]\n", __func__, __LINE__);

	if (mode == SENSOR_MOVIE) {
		CAM_DEBUG("[%s:%d] Camcorder_Mode_ON\n", __func__, __LINE__);
		ISX012_BURST_WRITE_LIST(ISX012_Camcorder_Mode_ON);
		isx012_ctrl->status.camera_status = 1;
	} else
		isx012_ctrl->status.camera_status = 0;

	if ((mode != SENSOR_CAMERA) && (mode != SENSOR_MOVIE))
		return -EINVAL;

	return 0;
}


static int isx012_check_dataline(s32 val)
{
	int err = -EINVAL;

	CAM_DEBUG("[%s:%d] DTP TEST-%s\n",
		__func__, __LINE__, (val ? "ON" : "OFF"));
	if (val) {
		CAM_DEBUG("[FACTORY-TEST] DTP ON\n");
		CAM_DEBUG("[%s:%d][DTP-TEST] DTP ON\n", __func__, __LINE__);
		ISX012_BURST_WRITE_LIST(isx012_DTP_init);
		DtpTest = 1;
		err = 0;
	} else {
		CAM_DEBUG("[%s:%d][DTP-TEST] DTP OFF\n", __func__, __LINE__);
		ISX012_BURST_WRITE_LIST(isx012_DTP_stop);
		DtpTest = 0;
		err = 0;
	}

	return err;
}

static int isx012_mipi_mode(int mode)
{
	int err = 0;
	struct msm_camera_csi_params isx012_csi_params;

	CAM_DEBUG("[%s:%d][E]\n", __func__, __LINE__);

	if (!isx012_ctrl->status.config_csi1) {
		isx012_csi_params.lane_cnt = 2;
		isx012_csi_params.data_format = 0x1E;
		isx012_csi_params.lane_assign = 0xe4;
		isx012_csi_params.dpcm_scheme = 0;
		isx012_csi_params.settle_cnt = 24;/*// 0x14; //0x7; //0x14;*/
		err = msm_camio_csi_config(&isx012_csi_params);

		if (err < 0)
			printk(KERN_ERR
				"[%s:%d]config csi controller failed\n",
				__func__, __LINE__);

		isx012_ctrl->status.config_csi1 = 1;
	} else
		printk(KERN_ERR "[%s:%d]X : already started\n",
			__func__, __LINE__);

	CAM_DEBUG("[%s:%d][X]\n", __func__, __LINE__);
	return err;
}

static int isx012_start(void)
{
	int err = 0;

	CAM_DEBUG("[%s:%d][E]\n", __func__, __LINE__);

	if (isx012_ctrl->status.started) {
		printk(KERN_ERR "[%s:%d]X : already started\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	isx012_mipi_mode(1);
	msleep(30); /*=> Please add some delay*/


	/*ISX012_BURST_WRITE_LIST(isx012_init_reg1);*/
	mdelay(10);

	/*ISX012_BURST_WRITE_LIST(isx012_init_reg2);*/

	isx012_ctrl->status.initialized = 1;
	isx012_ctrl->status.started = 1;
	isx012_ctrl->status.AE_AWB_hunt = 1;
	isx012_ctrl->status.start_af = 0;
	isx012_ctrl->setting.afmode = FOCUS_AUTO;

	CAM_DEBUG("[%s:%d][X]\n", __func__, __LINE__);

	return err;
}

/*
static long isx012_video_config(int mode)
{
	int err = 0; //-EINVAL;
	CAM_DEBUG("E");

	return err;
}

static long isx012_snapshot_config(int mode)
{
	CAM_DEBUG("E");

	ISX012_BURST_WRITE_LIST(ISX012_Capture_SizeSetting);
	ISX012_BURST_WRITE_LIST(isx012_Capture_Start);

	isx012_mode_transition_CM();


	return 0;
}
*/

static long isx012_set_sensor_mode(int mode)
{
	int err = -EINVAL;
	short unsigned int r_data[1] = {0};
/*	short unsigned int r_data2[2] = {0, 0};*/
	char modesel_fix[1] = {0}, awbsts[1] = {0};
	int timeout_cnt = 0;
/*
	unsigned int hunt1 = 0;
	char hunt2[1] = {0};
*/
	CAM_DEBUG("[%s:%d][E]\n", __func__, __LINE__);

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		CAM_DEBUG("[%s:%d][SENSOR_PREVIEW_MODE START]\n",
			__func__, __LINE__);
		if (iscapture == 1) {
			if (((isx012_ctrl->setting.flash_mode
						== FLASH_MODE_AUTO_S)
				&& (gLowLight_check)) ||
				(isx012_ctrl->setting.flash_mode
						== FLASH_MODE_ON_S)) {
				isx012_set_flash(0, 50);
				ISX012_BURST_WRITE_LIST(ISX012_Flash_OFF);
				if (isx012_ctrl->setting.whiteBalance ==
					WHITE_BALANCE_AUTO)
					isx012_i2c_write_multi(
					0x0282, 0x20, 0x01);

				isx012_i2c_write_multi(0x8800, 0x01, 0x01);
				err = isx012_set_focus_mode(
					isx012_ctrl->setting.afmode, 0);
			} else {
				err = isx012_set_focus_mode(
					isx012_ctrl->setting.afmode, 0);
				if (isx012_ctrl->setting.scene ==
					SCENE_NIGHT)
					ISX012_BURST_WRITE_LIST(
					ISX012_Lowlux_Night_Reset);
			}

			ISX012_BURST_WRITE_LIST(ISX012_Preview_Mode);
			isx012_mode_transition_CM();

			iscapture = 0;
			isx012_ctrl->status.touchaf = 0;
			isx012_ctrl->status.touchaf_cnt = 0;
			isx012_ctrl->status.start_af = 0;
		} else
			isx012_start();

		if (isx012_ctrl->status.AE_AWB_hunt == 1) {
#ifdef CONFIG_LOAD_FILE
			gERRSCL_AUTO =
			isx012_define_read("ERRSCL_AUTO", 6);
			gUSER_AESCL_AUTO =
				isx012_define_read("USER_AESCL_AUTO", 6);
			gERRSCL_NOW =
				isx012_define_read("ERRSCL_NOW", 6);
			gUSER_AESCL_NOW =
				isx012_define_read("USER_AESCL_NOW", 6);
			gAE_OFSETVAL =
				isx012_define_read("AE_OFSETVAL", 4);
			gAE_MAXDIFF =
				isx012_define_read("AE_MAXDIFF", 4);
			gGLOWLIGHT_DEFAULT =
				isx012_define_read("GLOWLIGHT_DEFAULT", 6);
			gGLOWLIGHT_ISO50 =
				isx012_define_read("GLOWLIGHT_ISO50", 6);
			gGLOWLIGHT_ISO100 =
				isx012_define_read("GLOWLIGHT_ISO100", 6);
			gGLOWLIGHT_ISO200 =
				isx012_define_read("GLOWLIGHT_ISO200", 6);
			gGLOWLIGHT_ISO400 =
				isx012_define_read("GLOWLIGHT_ISO400", 6);
			isx012_define_table();
#else
			gERRSCL_AUTO = ERRSCL_AUTO;
			gUSER_AESCL_AUTO = USER_AESCL_AUTO;
			gERRSCL_NOW = ERRSCL_NOW;
			gUSER_AESCL_NOW = USER_AESCL_NOW;
			gAE_OFSETVAL = AE_OFSETVAL;
			gAE_MAXDIFF = AE_MAXDIFF;
			gGLOWLIGHT_DEFAULT = GLOWLIGHT_DEFAULT;
			gGLOWLIGHT_ISO50 = GLOWLIGHT_ISO50;
			gGLOWLIGHT_ISO100 = GLOWLIGHT_ISO100;
			gGLOWLIGHT_ISO200 = GLOWLIGHT_ISO200;
			gGLOWLIGHT_ISO400 = GLOWLIGHT_ISO400;
#endif
/*
			printk(KERN_DEBUG
			"gERRSCL_AUTO = 0x%x", gERRSCL_AUTO);
			printk(KERN_DEBUG
			"gUSER_AESCL_AUTO = 0x%x", gUSER_AESCL_AUTO);
			printk(KERN_DEBUG
			"gERRSCL_NOW = 0x%x", gERRSCL_NOW);
			printk(KERN_DEBUG
			"gUSER_AESCL_NOW = 0x%x", gUSER_AESCL_NOW);
			printk(KERN_DEBUG
			"gAE_OFSETVAL = %d", gAE_OFSETVAL);
			printk(KERN_DEBUG
			"gAE_MAXDIFF = %d", gAE_MAXDIFF);
			printk(KERN_DEBUG
			"gGLOWLIGHT_DEFAULT = 0x%x\n", gGLOWLIGHT_DEFAULT);
			printk(KERN_DEBUG
			"gGLOWLIGHT_ISO50 = 0x%x\n", gGLOWLIGHT_ISO50);
			printk(KERN_DEBUG
			"gGLOWLIGHT_ISO100 = 0x%x\n", gGLOWLIGHT_ISO100);
			printk(KERN_DEBUG
			"gGLOWLIGHT_ISO200 = 0x%x\n", gGLOWLIGHT_ISO200);
			printk(KERN_DEBUG
			"gGLOWLIGHT_ISO400 = 0x%x\n", gGLOWLIGHT_ISO400);
*/
/*
			timeout_cnt = 0;
			do {
			if (timeout_cnt > 0) {
			mdelay(10);
			}
			timeout_cnt++;

			//SHT_TIME_OUT_L
			err = isx012_i2c_read_multi(0x8A00, r_data2, 2);
			hunt1 = r_data2[1] << 8 | r_data2[0];
			if (timeout_cnt > ISX012_DELAY_RETRIES2) {
				CAM_DEBUG(
				"%s: %d :Entering hunt1 delay timed out\n",
				__func__, __LINE__);
			break;
			}
			} while ((hunt1 != 0x00));

			timeout_cnt = 0;
			do {
			if (timeout_cnt > 0) {
			mdelay(10);
			}
			timeout_cnt++;
			err = isx012_i2c_read(0x8A24, r_data);
			hunt2[0] = r_data[0];
			if (timeout_cnt > ISX012_DELAY_RETRIES2) {
				CAM_DEBUG(
				"%s: %d :Entering hunt2 delay timed out\n",
				__func__, __LINE__);
			break;
			}
			} while ((hunt2[0] != 0x2));
*/
			mdelay(30);
			isx012_ctrl->status.AE_AWB_hunt = 0;
		}
		/*if (isx012_ctrl->sensor_mode != SENSOR_MOVIE)
		//err= isx012_video_config(SENSOR_PREVIEW_MODE);*/
		break;

	case SENSOR_SNAPSHOT_MODE:
	case SENSOR_RAW_SNAPSHOT_MODE:
		CAM_DEBUG("[%s:%d][SENSOR_SNAPSHOT_MODE START]\n",
			__func__, __LINE__);

		if (((isx012_ctrl->setting.flash_mode == FLASH_MODE_AUTO_S)
			&& (gLowLight_check)) ||
			(isx012_ctrl->setting.flash_mode == FLASH_MODE_ON_S)) {
			if (isx012_ctrl->status.touchaf) {
				/*CAM_DEBUG(
					"%s: %d :[for tuning] touchaf :%d\n",
					__func__, __LINE__,
				isx012_ctrl->status.touchaf);*/
				isx012_i2c_write_multi(0x0294, 0x02, 0x01);
				isx012_i2c_write_multi(0x0297, 0x02, 0x01);
				isx012_i2c_write_multi(0x029A, 0x02, 0x01);
				isx012_i2c_write_multi(0x029E, 0x02, 0x01);

				/*wait 1V time (66ms)*/
				mdelay(66);
			}
		}

		isx012_set_flash(isx012_ctrl->setting.flash_mode, 50);


		CAM_DEBUG("[%s:%d][ISX012_Capture_Mode start]\n",
			__func__, __LINE__);
		if (isx012_ctrl->status.start_af == 0) {
			if (isx012_ctrl->status.touchaf) {
				/*touch-af window*/
				ISX012_WRITE_LIST(ISX012_AF_TouchSAF_OFF);
				isx012_ctrl->status.touchaf_cnt = 0;
			} else {
				ISX012_WRITE_LIST(ISX012_AF_SAF_OFF);
			}

			/*wait 1V time (66ms)*/
			mdelay(120);
			isx012_ctrl->status.touchaf = 0;
		}

		if ((isx012_ctrl->setting.scene == SCENE_NIGHT)
			&& (gLowLight_check)) {
			ISX012_BURST_WRITE_LIST
				(ISX012_Lowlux_Night_Capture_Mode);
		} else {
			ISX012_BURST_WRITE_LIST(ISX012_Capture_Mode);
		}

		/*isx012_i2c_write_multi(0x8800, 0x01, 0x01);*/
		isx012_mode_transition_CM();
		isx012_wait_for_VINT();

		isx012_exif_shutter_speed();
		CAM_DEBUG("[%s:%d][ISX012_Capture_Mode end]\n",
			__func__, __LINE__);

		if (((isx012_ctrl->setting.flash_mode == FLASH_MODE_AUTO_S)
			&& (gLowLight_check)) ||
			(isx012_ctrl->setting.flash_mode == FLASH_MODE_ON_S)) {
			/*wait 1V time (150ms)*/
			mdelay(210);

			timeout_cnt = 0;
			do {
				if (timeout_cnt > 0)
					mdelay(1);

				timeout_cnt++;
				err = isx012_i2c_read(0x8A24, r_data);
				awbsts[0] = r_data[0];
				if (timeout_cnt > ISX012_DELAY_RETRIES) {
					printk(KERN_ERR "[%s:%d] Entering delay awbsts timed out\n",
						__func__, __LINE__);
					break;
				}
				if (awbsts[0] == 0x6) {	/* Flash Saturation*/
					CAM_DEBUG(
					"[%s:%d]Enter delay awbsts[0x%x]\n",
					__func__, __LINE__, awbsts[0]);
					break;
				}
				if (awbsts[0] == 0x4) {	/* Flash Saturation*/
					CAM_DEBUG(
					"[%s:%d]Enter delay awbsts[0x%x]\n",
					__func__, __LINE__, awbsts[0]);
					break;
				}
			} while (awbsts[0] != 0x2);
		}
		iscapture = 1;
		/*err = isx012_snapshot_config(SENSOR_SNAPSHOT_MODE);*/


		break;


	case SENSOR_SNAPSHOT_TRANSFER:
		CAM_DEBUG("[%s:%d]SENSOR_SNAPSHOT_TRANSFER START\n",
			__func__, __LINE__);

	break;

	default:
		return 0;/*-EFAULT;*/
	}

	return 0;
}

#ifdef FACTORY_TEST
struct class *camera_class;
struct device *isx012_dev;

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
	int torch = 0, torch2 = 0, torch3 = 0;

	sscanf(buf, "%d", &value);

	if (value == 0) {
		accessibility_torch = 0;
		CAM_DEBUG("[%s:%d][Accessary] flash OFF\n", __func__, __LINE__);
		gpio_set_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(
				PMIC_GPIO_CAM_FLASH_EN), 0);
		gpio_set_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(
				PMIC_GPIO_CAM_FLASH_SET), 0);
	} else {
		accessibility_torch = 1;
		CAM_DEBUG("[%s:%d][Accessary] flash ON\n", __func__, __LINE__);
		gpio_set_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(
				PMIC_GPIO_CAM_FLASH_EN), 0);

		for (i = 5; i > 1; i--) {
			gpio_set_value_cansleep(
				PM8058_GPIO_PM_TO_SYS(
					PMIC_GPIO_CAM_FLASH_SET), 1);
			udelay(1);
			gpio_set_value_cansleep(
				PM8058_GPIO_PM_TO_SYS(
					PMIC_GPIO_CAM_FLASH_SET), 0);
			udelay(1);
		}
		gpio_set_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(
				PMIC_GPIO_CAM_FLASH_SET), 1);
		usleep(2*1000);
	}

	return size;
}

static DEVICE_ATTR(rear_flash,
	0660, cameraflash_file_cmd_show, cameraflash_file_cmd_store);

static ssize_t camtype_file_cmd_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char camType[] = "SONY_ISX012_NONE";

	return sprintf(buf, "%s", camType);
}

static ssize_t camtype_file_cmd_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(rear_camtype,
	0660, camtype_file_cmd_show, camtype_file_cmd_store);

static ssize_t back_camera_firmware_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	char cam_fw[] = "ISX012\n";
	return snprintf(buf, sizeof(cam_fw), "%s", cam_fw);
}

static DEVICE_ATTR(rear_camfw,
	0664, back_camera_firmware_show, NULL);

#endif

static int isx012_set_power(int enable)
{
	int err = 0;

	struct vreg *vreg_L16;
	struct vreg *vreg_L8;

	CAM_DEBUG("[%s:%d] POWER ON START\n", __func__, __LINE__);

	if (enable == 1) {
		gpio_tlmm_config(
				GPIO_CFG(CAM_nRST, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
				GPIO_CFG_ENABLE); /*CAM_RESET*/
		gpio_tlmm_config(
				GPIO_CFG(CAM_nSTBY, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
				GPIO_CFG_ENABLE); /*CAM_STANDBY*/
		gpio_tlmm_config(
				GPIO_CFG(CAM_EN, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
				GPIO_CFG_ENABLE); /*CAM_EN*/
		gpio_tlmm_config(
				GPIO_CFG(CAM_EN_1, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
				GPIO_CFG_ENABLE); /*CAM_EN_1*/
		gpio_tlmm_config(
				GPIO_CFG(CAM_EN_2, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
				GPIO_CFG_ENABLE); /*CAM_EN_2*/
		gpio_tlmm_config(
				GPIO_CFG(CAM_VT_RST, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
				GPIO_CFG_ENABLE); /*CAM_VT_RST*/
		gpio_tlmm_config(
				GPIO_CFG(CAM_VT_nSTBY, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
				GPIO_CFG_ENABLE); /*CAM_VT_nSTBY*/
		vreg_L8 = vreg_get(NULL, "gp7");
		vreg_set_level(vreg_L8, 1800);

		vreg_L16 = vreg_get(NULL, "gp10");
		vreg_set_level(vreg_L16, 3000);

		mdelay(5);

		/* CAM_AF_3.0V Set High*/
		vreg_enable(vreg_L16);

		/* CAM_D_1.2V Set High*/
		gpio_set_value(CAM_EN_2, 1);

		/* CAM_IO_1.8V Set High*/
		gpio_set_value(CAM_EN_1, 1);

		/* CAM_A_2.8V Set High*/
		gpio_set_value(CAM_EN, 1);

		/* VCAM_1.8VDV Set High*/
		vreg_enable(vreg_L8);
		mdelay(1);

		/* CAM_VT_nSTBY Set High*/
		gpio_set_value(CAM_VT_nSTBY, 1);
	} else if (enable == 0) {

		vreg_L8 = vreg_get(NULL, "gp7");

		/* VCAM_1.8VDV Set High*/
		vreg_disable(vreg_L8);
		/*mdelay(10);*/
		mdelay(1);

		/* CAM_A_2.8V Set High*/
		gpio_set_value(CAM_EN, 0);
		/*mdelay(10);*/
		mdelay(1);

		/* CAM_IO_1.8V Set High*/
		gpio_set_value(CAM_EN_1, 0);
		mdelay(1);

		/* CAM_D_1.2V Set High*/
		gpio_set_value(CAM_EN_2, 0);
		udelay(10);

	}

	return err;


}
static int isx012_sensor_init_probe(
const struct msm_camera_sensor_info *data)
{
	int err = 0;
	int temp = 0;

	CAM_DEBUG("[%s:%d] MCLK 24MHz Enable\n", __func__, __LINE__);

	mdelay(10);

	/* CAM_VT_RST Set High*/
	gpio_set_value(CAM_VT_RST, 1);
	CAM_DEBUG("[%s:%d] CAM_VT_RST Set High\n", __func__, __LINE__);
	mdelay(1);

	isx012_i2c_write_multi_temp(0x03, 0x02);
	isx012_i2c_write_multi_temp(0x55, 0x10);

	/* CAM_VT_nSTBY Set Low*/
	gpio_set_value(CAM_VT_nSTBY, 0);
	CAM_DEBUG("[%s:%d] CAM_VT_nSTBY Set Low\n", __func__, __LINE__);
	/*mdelay(1);*/
	udelay(10);

	/* CAM_nRST Set High*/
	gpio_set_value(CAM_nRST, 1);
	CAM_DEBUG("[%s:%d] CAM_nRST Set High\n", __func__, __LINE__);
	/*mdelay(10);*/
	mdelay(6);

	/* Second I2C */
	err = isx012_mode_transition_OM();
	if (err == -EIO)
		return -EIO;

	CAM_DEBUG("[%s:%d] Write I2C ISX012_Pll_Setting_4\n",
		__func__, __LINE__);
	ISX012_BURST_WRITE_LIST(ISX012_Pll_Setting_4);

	isx012_mode_transition_OM();
	/*mdelay(2);*/
	udelay(200);

	/* CAM_nSTBY Set High*/
	gpio_set_value(CAM_nSTBY, 1);
	CAM_DEBUG("[%s:%d] CAM_nSTBY Set High\n", __func__, __LINE__);
	/*mdelay(20);*/
	mdelay(12);

	isx012_mode_transition_OM();
	isx012_mode_transition_CM();

	isx012_i2c_write_multi(0x5008, 0x00, 0x01);
	isx012_Sensor_Calibration();
	mdelay(1);

	ISX012_BURST_WRITE_LIST(ISX012_Init_Reg);

	CAM_DEBUG("[%s:%d] POWER ON END\n", __func__, __LINE__);

	return err;
}

int isx012_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int err = 0;
	CAM_DEBUG("[%s:%d][E]\n", __func__, __LINE__);

	isx012_ctrl = kzalloc(sizeof(struct isx012_ctrl_t), GFP_KERNEL);
	if (!isx012_ctrl) {
		printk(KERN_DEBUG "[%s:%d] failed!", __func__, __LINE__);
		err = -ENOMEM;
		goto init_done;
	}

	isx012_exif = kzalloc(sizeof(struct isx012_exif_data), GFP_KERNEL);
	if (!isx012_exif) {
		printk(KERN_DEBUG
			"[%s:%d] allocate isx012_exif_data failed!\n",
			__func__, __LINE__);
		err = -ENOMEM;
		goto init_done;
	}

	if (data)
		isx012_ctrl->sensordata = data;

#ifdef CONFIG_LOAD_FILE
		isx012_regs_table_init();
#endif

	err = isx012_sensor_init_probe(data);
	if (err < 0) {
		printk(KERN_DEBUG "[%s:%d]isx012_sensor_open_init failed!",
			__func__, __LINE__);
		goto init_fail;
	}

	isx012_ctrl->status.started = 0;
	isx012_ctrl->status.initialized = 0;
	isx012_ctrl->status.config_csi1 = 0;
	isx012_ctrl->status.cancel_af = 0;
	isx012_ctrl->status.camera_status = 0;
	isx012_ctrl->status.initialized = 0;
	isx012_ctrl->status.apps = 0;

	isx012_ctrl->setting.check_dataline = 0;
	isx012_ctrl->setting.camera_mode = SENSOR_CAMERA;

	CAM_DEBUG("[%s:%d][X]\n", __func__, __LINE__);
init_done:
	return err;

init_fail:
	kfree(isx012_ctrl);
	return err;
}

static int isx012_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&isx012_wait_queue);
	return 0;
}

static int calculate_AEgain_offset(
	uint16_t ae_auto, uint16_t ae_now, int16_t ersc_auto, int16_t ersc_now)
{
	int err = -EINVAL;
	int16_t aediff, aeoffset;

	/*AE_Gain_Offset = Target - ERRSCL_NOW*/
	aediff = (ae_now + ersc_now) - (ae_auto + ersc_auto);

	if (aediff < 0)
		aediff = 0;


	if (ersc_now < 0) {
		if (aediff >= AE_MAXDIFF) {
			aeoffset = -AE_OFSETVAL - ersc_now;
		} else {
#ifdef CONFIG_LOAD_FILE
			aeoffset = -gtable_buf[aediff/10] - ersc_now;
#else
			aeoffset = -aeoffset_table[aediff/10] - ersc_now;
#endif
		}
	} else {
		if (aediff >= AE_MAXDIFF) {
			aeoffset = -AE_OFSETVAL;
		} else {
#ifdef CONFIG_LOAD_FILE
			aeoffset = -gtable_buf[aediff/10];
#else
			aeoffset = -aeoffset_table[aediff/10];
#endif
		}
	}
	/*CAM_DEBUG("[For tuning] aeoffset(%d) |
	aediff(%d) = (ae_now(%d) + ersc_now(%d)) -
	(ae_auto(%d) + ersc_auto(%d))\n",
	aeoffset, aediff, ae_now, ersc_now, ae_auto, ersc_auto);*/


	/* SetAE Gain offset*/
	err = isx012_i2c_write_multi(CAP_GAINOFFSET, aeoffset, 2);

	return err;
}

static int isx012_sensor_af_status(void)
{
	int err = 0, status = 0;
	unsigned char r_data[2] = {0, 0};

	/*CAM_DEBUG(" %s/%d\n", __func__, __LINE__);*/

	err = isx012_i2c_read_multi(0x8B8A, r_data, 1);
	/*CAM_DEBUG("%s:%d status(0x%x) r_data(0x%x %x)\n",
	__func__, __LINE__, status, r_data[1], r_data[0]);*/
	status = r_data[0];
	/*CAM_DEBUG(" %s/%d status(0x%x)\n",
	__func__, __LINE__, status);*/
	if (status == 0x8) {
		CAM_DEBUG("[%s:%d] success\n", __func__, __LINE__);
		return 1;
	}

	return 0;
}

static int isx012_set_af_stop(int af_check)
{
	int err = -EINVAL;

	CAM_DEBUG(
		"[%s:%d] isx012_ctrl->status.cancel_af_running(%d)\n",
		__func__, __LINE__, isx012_ctrl->status.cancel_af_running);

	isx012_ctrl->status.start_af = 0;

	if (af_check == 1) {	/*After AF check*/
		isx012_ctrl->status.cancel_af_running++;
		if (isx012_ctrl->status.cancel_af_running == 1) {
			if ((isx012_ctrl->status.touchaf)
				&& (isx012_ctrl->status.touchaf_cnt)) {
				/*touch-af window*/
				ISX012_BURST_WRITE_LIST(ISX012_AF_TouchSAF_OFF);
			} else {
				ISX012_BURST_WRITE_LIST(ISX012_AF_SAF_OFF);
			}
			isx012_ctrl->status.touchaf_cnt = 0;

			/*wait 1V time (66ms)*/
			mdelay(66);
			isx012_ctrl->status.touchaf = 0;

			if (((isx012_ctrl->setting.flash_mode
						== FLASH_MODE_AUTO_S)
				&& (gLowLight_check)) ||
				(isx012_ctrl->setting.flash_mode
						== FLASH_MODE_ON_S)) {
				isx012_set_flash(0, 50);
				ISX012_BURST_WRITE_LIST(ISX012_Flash_OFF);
				if (isx012_ctrl->setting.whiteBalance ==
					WHITE_BALANCE_AUTO)
					isx012_i2c_write_multi(
					0x0282, 0x20, 0x01);

				isx012_i2c_write_multi(0x8800, 0x01, 0x01);
			}
			err = isx012_cancel_autofocus();
		}
	} else {
		isx012_ctrl->status.cancel_af = 1;
		err = 0;
	}

	return 0;
}

static int isx012_set_af_start(void)
{
	int err = -EINVAL;
	int timeout_cnt = 0;
	short unsigned int r_data[1] = {0};
	uint16_t ae_data[1] = {0};
	int16_t ersc_data[1] = {0};
	char modesel_fix[1] = {0}, half_move_sts[1] = {0};

	CAM_DEBUG("[%s:%d]\n", __func__, __LINE__);

	isx012_ctrl->status.start_af++;

	if (isx012_ctrl->status.cancel_af == 1) {
		CAM_DEBUG("[%s:%d]AF START : af cancel...\n",
			__func__, __LINE__);
		return 10;
	}

	if (isx012_ctrl->status.touchaf) {
		/*touch-af window*/
		ISX012_WRITE_LIST(ISX012_Flash_OFF);
		CAM_DEBUG("[%s:%d]touch-af window\n", __func__, __LINE__);
	} else {
		ISX012_BURST_WRITE_LIST(ISX012_AF_Window_Reset);
		CAM_DEBUG("[%s:%d]ISX012_AF_Window_Reset\n",
			__func__, __LINE__);
	}

	if (((isx012_ctrl->setting.flash_mode == FLASH_MODE_AUTO_S)
		&& (gLowLight_check)) ||
		(isx012_ctrl->setting.flash_mode == FLASH_MODE_ON_S)) {
		/*AE line change - AE line change value write*/
		ISX012_BURST_WRITE_LIST(ISX012_Flash_AELINE);

		/*wait 1V time (60ms)*/
		mdelay(60);

		/*Read AE scale - ae_auto, ersc_auto*/
		err = isx012_i2c_read(0x01CE, ae_data);
		g_ae_auto = ae_data[0];
		err = isx012_i2c_read(0x01CA, ersc_data);
		g_ersc_auto = ersc_data[0];

		/*Flash On set*/
		if (isx012_ctrl->setting.whiteBalance == WHITE_BALANCE_AUTO)
			isx012_i2c_write_multi(0x0282, 0x00, 0x01);

		ISX012_BURST_WRITE_LIST(ISX012_Flash_ON);

		/*Fast AE, AWB, AF start*/
		isx012_i2c_write_multi(0x8800, 0x01, 0x01);

		/*wait 1V time (40ms)*/
		mdelay(40);

		isx012_set_flash(3 , 50);

		timeout_cnt = 0;
		do {
			if (isx012_ctrl->status.cancel_af == 1) {
				CAM_DEBUG(
				"[%s:%d]AF STOP : af canncel..modesel_fix\n",
				__func__, __LINE__);
				break;
			}
			if (timeout_cnt > 0)
				mdelay(10);

			timeout_cnt++;
			err = isx012_i2c_read(0x0080, r_data);
			modesel_fix[0] = r_data[0];
			if (timeout_cnt > ISX012_DELAY_RETRIES) {
				printk(KERN_DEBUG "[%s:%d] Entering delay modesel_fix timed out\n",
					__func__, __LINE__);
				break;
			}
		} while ((modesel_fix[0] != 0x1));

		timeout_cnt = 0;
		do {
			if (isx012_ctrl->status.cancel_af == 1) {
				CAM_DEBUG(
				"[%s:%d]AF STOP : af canncel..half_move_sts\n",
				__func__, __LINE__);
				break;
			}
			if (timeout_cnt > 0)
				mdelay(10);

			timeout_cnt++;
			err = isx012_i2c_read(0x01B0, r_data);
			half_move_sts[0] = r_data[0];
			if (timeout_cnt > ISX012_DELAY_RETRIES) {
				printk(KERN_DEBUG "[%s:%d] Entering half_move_sts delay timed out\n",
					__func__, __LINE__);
				break;
			}
		} while ((half_move_sts[0] != 0x0));
	} else {
		if (isx012_ctrl->setting.afmode == FOCUS_MACRO) {
			ISX012_WRITE_LIST(isx012_AF_Init_Macro_ON);
		} else {
			ISX012_WRITE_LIST(isx012_AF_Init_Macro_OFF);
		}

		if ((isx012_ctrl->setting.scene == SCENE_NIGHT)
			&& (gLowLight_check)) {
			ISX012_BURST_WRITE_LIST
				(ISX012_Lowlux_night_Halfrelease_Mode);
		} else if ((!isx012_ctrl->status.apps)
			&& (isx012_ctrl->status.start_af > 1)) {
			if (isx012_ctrl->status.start_af == 2) {
				isx012_set_af_stop(1);
				isx012_ctrl->status.start_af = 2;
			}
			ISX012_BURST_WRITE_LIST(ISX012_Barcode_SAF);
		} else {
			ISX012_BURST_WRITE_LIST(ISX012_Halfrelease_Mode);
		}


		/*wait 1V time (40ms)*/
		mdelay(40);
	}

	return 0;
}

static int isx012_sensor_af_result(void)
{
	int ret = 0;
	int status = 0;

	CAM_DEBUG("[%s:%d]\n", __func__, __LINE__);

	isx012_i2c_read(0x8B8B, (unsigned short *)&status);
	if ((status & 0x1) == 0x1) {
		CAM_DEBUG("[%s:%d]AF success\n", __func__, __LINE__);
		ret = 1;
	} else if ((status & 0x1) == 0x0) {
		CAM_DEBUG("[%s:%d]AF fail\n", __func__, __LINE__);
		ret = 2;
	}

	return ret;
}

static int isx012_get_af_status(void)
{
	int err = -EINVAL;
	int af_result = -EINVAL, af_status = -EINVAL;
	uint16_t ae_data[1] = {0};
	int16_t ersc_data[1] = {0};
	int16_t aescl_data[1] = {0};
	int16_t ae_scl = 0;

	/*CAM_DEBUG(" %s/%d\n", __func__, __LINE__);*/
	if (isx012_ctrl->status.cancel_af == 1) {
		CAM_DEBUG("[%s:%d]AF STATUS : af cancel...\n",
			__func__, __LINE__);
		return 10;
	}

	af_status = isx012_sensor_af_status();
	if (af_status == 1) {
		isx012_i2c_write_multi(0x0012, 0x10, 0x01);
		af_result = isx012_sensor_af_result();

		if (((isx012_ctrl->setting.flash_mode == FLASH_MODE_AUTO_S) &&
			(gLowLight_check)) ||
			(isx012_ctrl->setting.flash_mode == FLASH_MODE_ON_S)) {
			/*Read AE scale - ae_now, ersc_now*/
			err = isx012_i2c_read(0x01CC, ersc_data);
			g_ersc_now = ersc_data[0];
			err = isx012_i2c_read(0x01D0, ae_data);
			g_ae_now = ae_data[0];
			err = isx012_i2c_read(0x8BC0, aescl_data);
			ae_scl = aescl_data[0];
		}

		if ((isx012_ctrl->status.touchaf)
			&& (isx012_ctrl->status.touchaf_cnt)) {
			/*touch-af window*/
			ISX012_BURST_WRITE_LIST(ISX012_AF_TouchSAF_OFF);
		} else {
			ISX012_BURST_WRITE_LIST(ISX012_AF_SAF_OFF);
		}
		isx012_ctrl->status.touchaf_cnt = 0;

		/*wait 1V time (66ms)*/
		mdelay(66);

		/* AE SCL*/
		if (((isx012_ctrl->setting.flash_mode == FLASH_MODE_AUTO_S) &&
			(gLowLight_check)) ||
			(isx012_ctrl->setting.flash_mode == FLASH_MODE_ON_S)) {
			ae_scl = ae_scl - 4129;
			/*printk(KERN_DEBUG
			"[For tuning] original ae_scl(%d)"
			" - 4129 => ae_scl(%d)\n",
			ae_scl + 4129, ae_scl);*/
			isx012_i2c_write_multi(0x5E02, ae_scl, 0x02);

			err = calculate_AEgain_offset(
				g_ae_auto, g_ae_now, g_ersc_auto, g_ersc_now);
			/*///isx012_i2c_write_multi(0x5000, 0x0A, 0x01);*/
			isx012_set_flash(0 , 50);
		}
	} else {
		af_result = af_status;
	}

	return af_result;
}


static int isx012_set_touch_auto_focus(int value1)
{
	CAM_DEBUG("[%s:%d] value1[%d]\n", __func__, __LINE__, value1);

	/*isx012_ctrl->status.touchaf++;*/

	int x = isx012_ctrl->status.pos_x;
	int y = isx012_ctrl->status.pos_y;


	unsigned int H_ratio = 324;	/*//H_RATIO : 3.24 = 2592 / 800*/
	unsigned int V_ratio = 405;	/*//V_RATIO : 4.05 = 1944 / 480*/

	/*//AE value
	//OPD4 Not touchAF value
	//H : 2048 = 896 + 256 + 896
	//V : 1536 = 512 + 512 +512*/
	unsigned int AF_OPD4_HDELAY = 486;
	unsigned int AF_OPD4_VDELAY = 259;
	unsigned int AF_OPD4_HVALID = 259;
	unsigned int AF_OPD4_VVALID = 324;


	if (value1 == 0) {/* Stop touch AF*/
		if (iscapture != 1) {
			ISX012_BURST_WRITE_LIST(ISX012_AF_TouchSAF_OFF);

			/*wait 1V time (66ms)*/
			mdelay(66);

			if (((isx012_ctrl->setting.flash_mode
						== FLASH_MODE_AUTO_S) &&
				(gLowLight_check)) ||
				(isx012_ctrl->setting.flash_mode
						== FLASH_MODE_ON_S)) {
				isx012_set_flash(0, 50);
				ISX012_BURST_WRITE_LIST(ISX012_Flash_OFF);
				if (isx012_ctrl->setting.whiteBalance ==
					WHITE_BALANCE_AUTO)
					isx012_i2c_write_multi(
					0x0282, 0x20, 0x01);

				isx012_i2c_write_multi(
					0x8800, 0x01, 0x01);
			}

			isx012_cancel_autofocus();
			if (isx012_ctrl->status.touchaf_cnt > 0)
				isx012_ctrl->status.touchaf_cnt--;
		}
	} else {	/* Start touch AF
		// Calcurate AF window size & position*/
		AF_OPD4_HVALID = 259;
		AF_OPD4_VVALID = 324;

		/*AF_OPD_HDELAY =
		X position + 8(color processing margin)+41(isx012 resttiction)*/
		AF_OPD4_HDELAY =
		((x * H_ratio) / 100) - (AF_OPD4_HVALID / 2) + 8 + 41;
		/*AF_OPD_VDELAY = Y position +5 (isx012 resttiction)*/
		AF_OPD4_VDELAY =
		((y * V_ratio) / 100) - (AF_OPD4_VVALID / 2) + 5;

		/* out of boundary... min.*/
		/*WIDSOWN H size min : X position + 8 is really min.*/
		if (AF_OPD4_HDELAY < 8 + 41)
			AF_OPD4_HDELAY = 8 + 41 ;
		/*WIDSOWN V size min : Y position is really min.*/
		if (AF_OPD4_VDELAY < 5)
			AF_OPD4_VDELAY = 5;

		/* out of boundary... max.*/
		if (AF_OPD4_HDELAY > 2608 - AF_OPD4_HVALID - 8 - 41)
			AF_OPD4_HDELAY = 2608 - AF_OPD4_HVALID - 8 - 41;
		if (AF_OPD4_VDELAY > 1944 - AF_OPD4_VVALID - 5)
			AF_OPD4_VDELAY = 1944 - AF_OPD4_VVALID - 5;


		/* AF window setting*/
		isx012_i2c_write_multi(0x6A50, AF_OPD4_HDELAY, 2);
		isx012_i2c_write_multi(0x6A52, AF_OPD4_VDELAY, 2);
		isx012_i2c_write_multi(0x6A54, AF_OPD4_HVALID, 2);
		isx012_i2c_write_multi(0x6A56, AF_OPD4_VVALID, 2);

		isx012_i2c_write_multi(0x6A80, 0x0000, 1);
		isx012_i2c_write_multi(0x6A81, 0x0000, 1);
		isx012_i2c_write_multi(0x6A82, 0x0000, 1);
		isx012_i2c_write_multi(0x6A83, 0x0000, 1);
		isx012_i2c_write_multi(0x6A84, 0x0008, 1);
		isx012_i2c_write_multi(0x6A85, 0x0000, 1);
		isx012_i2c_write_multi(0x6A86, 0x0000, 1);
		isx012_i2c_write_multi(0x6A87, 0x0000, 1);
		isx012_i2c_write_multi(0x6A88, 0x0000, 1);
		isx012_i2c_write_multi(0x6A89, 0x0000, 1);
		isx012_i2c_write_multi(0x6646, 0x0000, 1);
	}

	return 0;
}


int isx012_sensor_ext_config(void __user *argp)
{
	struct sensor_ext_cfg_data cfg_data;
	int err = 0;

	/*CAM_DEBUG("isx012_sensor_ext_config ");*/

	if (copy_from_user(
		(void *)&cfg_data, (const void *)argp, sizeof(cfg_data)))
		printk(KERN_DEBUG "[%s:%d]fail copy_from_user!",
			__func__, __LINE__);


/*
	if (cfg_data.cmd != EXT_CFG_GET_AF_STATUS) {
		printk(KERN_DEBUG
		"[%s][line:%d] cfg_data.cmd(%d) cfg_data.value_1(%d)"
		",cfg_data.value_2(%d)\n", __func__, __LINE__,
		cfg_data.cmd, cfg_data.value_1, cfg_data.value_2);
	}
*/

	if (DtpTest == 1) {
		if ((cfg_data.cmd != EXT_CFG_SET_VT_MODE)
		&& (cfg_data.cmd != EXT_CFG_SET_MOVIE_MODE)
		&& (cfg_data.cmd != EXT_CFG_SET_DTP)
		&& (!isx012_ctrl->status.initialized)) {
			printk(KERN_DEBUG
				"[%s:%d] camera isn't initialized(status.initialized:%d)\n",
				__func__, __LINE__,
				isx012_ctrl->status.initialized);
			printk(KERN_DEBUG
				"[%s:%d] cfg_data.cmd(%d) cfg_data.value_1(%d),"
				"cfg_data.value_2(%d)\n", __func__, __LINE__,
				cfg_data.cmd, cfg_data.value_1,
				cfg_data.value_2);
			return 0;
		}
	}

	switch (cfg_data.cmd) {
	case EXT_CFG_SET_FLASH:
		err = isx012_set_flash(cfg_data.value_1, cfg_data.value_2);
		break;

	case EXT_CFG_SET_BRIGHTNESS:
		err = isx012_set_brightness(cfg_data.value_1);
		break;

	case EXT_CFG_SET_EFFECT:
		err = isx012_set_effect(cfg_data.value_1);
		break;

	case EXT_CFG_SET_ISO:
		err = isx012_set_iso(cfg_data.value_1);
		break;

	case EXT_CFG_SET_WB:
		err = isx012_set_whitebalance(cfg_data.value_1);
		break;

	case EXT_CFG_SET_SCENE:
		err = isx012_set_scene(cfg_data.value_1);
		break;

	case EXT_CFG_SET_METERING:
		/* auto exposure mode*/
		err = isx012_set_metering(cfg_data.value_1);
		break;

	case EXT_CFG_SET_CONTRAST:
		err = isx012_set_contrast(cfg_data.value_1);
		break;

	case EXT_CFG_SET_SHARPNESS:
		err = isx012_set_sharpness(cfg_data.value_1);
		break;

	case EXT_CFG_SET_SATURATION:
		err = isx012_set_saturation(cfg_data.value_1);
		break;

	case EXT_CFG_SET_PREVIEW_SIZE:
		err = isx012_set_preview_size(
			cfg_data.value_1, cfg_data.value_2);
		break;

	case EXT_CFG_SET_PICTURE_SIZE:
		err = isx012_set_picture_size(cfg_data.value_1);
		break;

	case EXT_CFG_SET_JPEG_QUALITY:
		/*err = isx012_set_jpeg_quality(
		cfg_data.value_1);*/
		break;

	case EXT_CFG_SET_FPS:
		err = isx012_set_frame_rate(cfg_data.value_2);
		break;

	case EXT_CFG_SET_DTP:
		CAM_DEBUG("[%s:%d] DTP mode [%d]\n",
			__func__, __LINE__, cfg_data.value_1);
		err = isx012_check_dataline(cfg_data.value_1);
		break;

	case EXT_CFG_SET_VT_MODE:
		CAM_DEBUG("[%s:%d] VTCall mode [%d]\n",
			__func__, __LINE__, cfg_data.value_1);
		break;

	case EXT_CFG_SET_MOVIE_MODE:
		CAM_DEBUG("[%s:%d] MOVIE mode [%d]\n",
			__func__, __LINE__, cfg_data.value_1);
		isx012_set_movie_mode(cfg_data.value_1);
		break;

	case EXT_CFG_SET_AF_OPERATION:
		CAM_DEBUG("[%s:%d] AF mode [%d]\n",
			__func__, __LINE__, cfg_data.value_1);
		break;

	case EXT_CFG_SET_AF_START:
		if (isx012_ctrl->status.touchaf_cnt == 1) {
			isx012_set_touch_auto_focus(1);
		} else if (isx012_ctrl->status.touchaf_cnt > 1) {
			isx012_set_touch_auto_focus(0);
			isx012_set_touch_auto_focus(1);
		}
		cfg_data.value_2 = isx012_set_af_start();
		break;

	case EXT_CFG_GET_AF_STATUS:
		cfg_data.value_2 = isx012_get_af_status();
		break;

	case EXT_CFG_SET_AF_STOP:
		CAM_DEBUG("[%s:%d] AF STOP mode [%d]\n",
			__func__, __LINE__, cfg_data.value_1);
		isx012_set_af_stop(cfg_data.value_1);
		break;

	case EXT_CFG_SET_AF_MODE:
		CAM_DEBUG("[%s:%d] Focus mode [%d]\n",
			__func__, __LINE__, cfg_data.value_1);
		err = isx012_set_focus_mode(cfg_data.value_1, 1);
		break;

	case EXT_CFG_SET_TOUCHAF_MODE: /* set touch AF mode on/off */
		err = isx012_set_touch_auto_focus(cfg_data.value_1);
		break;

	case EXT_CFG_SET_TOUCHAF_POS:	/* set touch AF position */
		isx012_ctrl->status.touchaf = 1;
		isx012_ctrl->status.touchaf_cnt++;
		isx012_ctrl->status.pos_x = cfg_data.value_1;
		isx012_ctrl->status.pos_y = cfg_data.value_2;
		break;

	case EXT_CFG_SET_LOW_LEVEL:
		isx012_ctrl->status.cancel_af = 0;
		isx012_ctrl->status.cancel_af_running = 0;
		if (isx012_ctrl->setting.flash_mode > FLASH_MODE_OFF_S)
			err = isx012_get_LowLightCondition();
		else if (isx012_ctrl->setting.scene == SCENE_NIGHT)
			err = isx012_get_LowLightCondition();
		else
			CAM_DEBUG("[%s:%d] Not Low light check\n",
				__func__, __LINE__);

		break;

	case EXT_CFG_GET_EXIF:
		isx012_get_exif(&cfg_data.value_1, &cfg_data.value_2);
		break;

	case EXT_CFG_SET_APPS:
		isx012_ctrl->status.apps = cfg_data.value_1;
		CAM_DEBUG("[%s:%d] isx012_ctrl->status.apps\n",
			__func__, __LINE__, isx012_ctrl->status.apps);
		break;

	default:
		break;
	}

	if (copy_to_user(
		(void *)argp, (const void *)&cfg_data, sizeof(cfg_data)))
		printk(KERN_DEBUG "[%s:%d] fail copy_from_user!",
			__func__, __LINE__);

	return err;
}

int isx012_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	int err = 0;

	if (copy_from_user(
		&cfg_data, (void *)argp, sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	CAM_DEBUG("[%s:%d] cfgtype = %d, mode = %d\n",
		__func__, __LINE__, cfg_data.cfgtype, cfg_data.mode);

	switch (cfg_data.cfgtype) {
	case CFG_SET_MODE:
		err = isx012_set_sensor_mode(cfg_data.mode);
		break;

	default:
		err = 0;/*-EFAULT;*/
		break;
	}

	return err;
}

int isx012_sensor_release(void)
{
	int err = 0;
	struct vreg *vreg_L16;

	CAM_DEBUG("[%s:%d] POWER OFF START\n", __func__, __LINE__);

	/*Soft landing*/
	ISX012_BURST_WRITE_LIST(ISX012_Sensor_Off_VCM);

	isx012_set_flash(0, 50);
	isx012_ctrl->setting.flash_mode = FLASH_MODE_OFF_S;

/*	vreg_L8 = vreg_get(NULL, "gp7");*/
	vreg_L16 = vreg_get(NULL, "gp10");

	/* CAM_AF_3.0V Set Low */
	vreg_disable(vreg_L16);
	mdelay(1);

	/* CAM_VT_RST Set Low */
	gpio_set_value(CAM_VT_RST, 0);
	udelay(10);

	/* CAM_nSTBY Set Low */
	gpio_set_value(CAM_nSTBY, 0);
	mdelay(150);

	/* CAM_nRST Set Low */
	gpio_set_value(CAM_nRST, 0);
	udelay(50);

	isx012_ctrl->status.initialized = 0;
	isx012_ctrl->status.cancel_af = 0;
	isx012_ctrl->status.pos_x = 0;
	isx012_ctrl->status.pos_y = 0;
	isx012_ctrl->status.touchaf = 0;
	isx012_ctrl->status.touchaf_cnt = 0;
	isx012_ctrl->status.apps = 0;
	isx012_ctrl->status.start_af = 0;
	iscapture = 0;

	DtpTest = 0;
	kfree(isx012_ctrl);
	kfree(isx012_exif);
	isx012_exif = NULL;

#ifdef CONFIG_LOAD_FILE
	isx012_regs_table_exit();
#endif
	CAM_DEBUG("[%s:%d] POWER OFF END\n", __func__, __LINE__);

	return err;
}


static int isx012_i2c_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;

	CAM_DEBUG("[%s:%d][E]\n", __func__, __LINE__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENOTSUPP;
		goto probe_failure;
	}

	isx012_sensorw = kzalloc(sizeof(struct isx012_work_t), GFP_KERNEL);

	if (!isx012_sensorw) {
		err = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, isx012_sensorw);
	isx012_init_client(client);
	isx012_client = client;

	gpio_set_value_cansleep(
		PM8058_GPIO_PM_TO_SYS(
			PMIC_GPIO_CAM_FLASH_SET), 0);
	gpio_set_value_cansleep(
		PM8058_GPIO_PM_TO_SYS(
			PMIC_GPIO_CAM_FLASH_EN), 0);
#ifdef FACTORY_TEST
	camera_class = class_create(THIS_MODULE, "camera");

	if (IS_ERR(camera_class))
		printk(KERN_DEBUG
			"[%s:%d] Failed to create class(camera)!\n",
			__func__, __LINE__);

	isx012_dev = device_create(camera_class, NULL, 0, NULL, "rear");
	if (IS_ERR(isx012_dev)) {
		printk(KERN_DEBUG "[%s:%d] Failed to create device!\n",
			__func__, __LINE__);
		goto probe_failure;
	}

	if (device_create_file(isx012_dev, &dev_attr_rear_camfw) < 0) {
		printk(KERN_DEBUG "[%s:%d] failed to create device file, %s\n",
			__func__, __LINE__, dev_attr_rear_camfw.attr.name);
	}
	if (device_create_file(isx012_dev, &dev_attr_rear_camtype) < 0) {
		printk(KERN_DEBUG "[%s:%d] failed to create device file, %s\n",
			__func__, __LINE__, dev_attr_rear_camtype.attr.name);
	}
	if (device_create_file(isx012_dev, &dev_attr_rear_flash) < 0) {
		printk(KERN_DEBUG "[%s:%dfailed to create device file, %s\n",
			__func__, __LINE__, dev_attr_rear_flash.attr.name);
	}
#endif

	CAM_DEBUG("[%s:%d][X]\n", __func__, __LINE__);

	return err;

probe_failure:
	kfree(isx012_sensorw);
	isx012_sensorw = NULL;
	printk(KERN_DEBUG "[%s:%d]isx012_i2c_probe failed!",
		__func__, __LINE__);
	return err;
}


static int __exit isx012_i2c_remove(struct i2c_client *client)
{
	struct isx012_work_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);

#ifdef FACTORY_TEST
	device_remove_file(isx012_dev, &dev_attr_rear_camtype);
	device_remove_file(isx012_dev, &dev_attr_rear_flash);
#endif

/*	i2c_detach_client(client);*/
	isx012_client = NULL;
	isx012_sensorw = NULL;
	kfree(sensorw);
	return 0;

}


static const struct i2c_device_id isx012_id[] = {
	{"isx012_i2c", 0},
	{}
};

/*PGH MODULE_DEVICE_TABLE(i2c, isx012);*/

static struct i2c_driver isx012_i2c_driver = {
	.id_table = isx012_id,
	.probe = isx012_i2c_probe,
	.remove = __exit_p(isx012_i2c_remove),
	.driver = {
		.name = "isx012",
	},
};


int32_t isx012_i2c_init(void)
{
	int32_t err = 0;

	CAM_DEBUG("[%s:%d][E]\n", __func__, __LINE__);


	err = i2c_add_driver(&isx012_i2c_driver);

	if (IS_ERR_VALUE(err))
		goto init_failure;

	return err;



init_failure:
	printk(KERN_DEBUG "[%s:%d]failed to isx012_i2c_init, err = %d",
		__func__, __LINE__, err);
	return err;
}


void isx012_exit(void)
{
	i2c_del_driver(&isx012_i2c_driver);
}


int isx012_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
	int err = 0;

	CAM_DEBUG("[%s:%d][E]\n", __func__, __LINE__);

/*	struct msm_camera_sensor_info *info =
		(struct msm_camera_sensor_info *)dev;

	struct msm_sensor_ctrl *s =
		(struct msm_sensor_ctrl *)ctrl;
*/


	err = isx012_i2c_init();
	if (err < 0)
		goto probe_done;

	s->s_init	= isx012_sensor_open_init;
	s->s_release	= isx012_sensor_release;
	s->s_config	= isx012_sensor_config;
	s->s_ext_config	= isx012_sensor_ext_config;
	s->s_power = isx012_set_power;
	s->s_camera_type = BACK_CAMERA_2D;
	s->s_mount_angle = 0;

probe_done:
	CAM_DEBUG("[%s:%d]Sensor probe done(%d)\n", __func__, __LINE__, err);
	return err;

}


static int __sec_isx012_probe(struct platform_device *pdev)
{
	CAM_DEBUG("[%s:%d][E]\n", __func__, __LINE__);

	return msm_camera_drv_start(pdev, isx012_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __sec_isx012_probe,
	.driver = {
		.name = "msm_camera_isx012",
		.owner = THIS_MODULE,
	},
};

static int __init sec_isx012_camera_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

static void __exit sec_isx012_camera_exit(void)
{
	platform_driver_unregister(&msm_camera_driver);
}

module_init(sec_isx012_camera_init);
module_exit(sec_isx012_camera_exit);
