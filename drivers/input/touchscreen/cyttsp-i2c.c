/* Source for:
 * Cypress TrueTouch(TM) Standard Product I2C touchscreen driver.
 * drivers/input/touchscreen/cyttsp-i2c.c
 *
 * Copyright (C) 2009, 2010 Cypress Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Cypress reserves the right to make changes without further notice
 * to the materials described herein. Cypress does not assume any
 * liability arising out of the application described herein.
 *
 * Contact Cypress Semiconductor at www.cypress.com
 *
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <mach/gpio.h> 
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/byteorder/generic.h>
#include <linux/bitops.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif /* CONFIG_HAS_EARLYSUSPEND */
#include <linux/regulator/consumer.h>
#include <mach/vreg.h> 
#include <linux/wakelock.h>

#define CYTTSP_DECLARE_GLOBALS
#include <linux/miscdevice.h>
#include <linux/cyttsp.h>
#ifdef FEATURE_CYTTSP_FIRMWAREUPGRADE
#include <linux/issp_defs.h>
#include <linux/issp_extern.h>
#endif

#include <linux/uaccess.h>

uint32_t cyttsp_tsdebug1;

module_param_named(tsdebug1, cyttsp_tsdebug1, uint, 0664);

#if defined(CONFIG_EF33_BOARD) || defined(CONFIG_EF34_BOARD) 
#define FEATURE_TOUCH_KEY
#endif
/* -------------------------------------------------------------------- */
/* EF33S gpio & resolution & key area*/
/* -------------------------------------------------------------------- */
#define GPIO_TOUCH_RST			95
#define GPIO_TOUCH_CHG			61
#define GPIO_TOUCH_SDA			64
#define GPIO_TOUCH_SCL			65
#define GPIO_TOUCH_ID			93
#define IRQ_TOUCH_INT			gpio_to_irq(GPIO_TOUCH_CHG)


/* -------------------------------------------------------------------- */
/* debug option */
/* -------------------------------------------------------------------- */
//#define TOUCH_DBG_ENABLE
#ifdef TOUCH_DBG_ENABLE
#define dbg(fmt, args...)   printk("[TOUCH]" fmt, ##args)
#else
#define dbg(fmt, args...)
#endif
#define dbg_func_in()       dbg("[FUNC_IN] %s\n", __func__)
#define dbg_func_out()      dbg("[FUNC_OUT] %s\n", __func__)
#define dbg_line()          dbg("[LINE] %d(%s)\n", __LINE__, __func__)
/* -------------------------------------------------------------------- */

#define FEATURE_SKY_PROCESS_CMD_KEY

#ifdef FEATURE_TOUCH_KEY
#define X_MAX	480
#define Y_MAX	800

#define NULL_KEY_AREA	840

#define MENU_KEY_MIN	40
#define MENU_KEY_MAX	140

#define HOME_KEY_MIN	210
#define HOME_KEY_MAX	280

#define BACK_KEY_MIN	360
#define BACK_KEY_MAX	460
#endif

#ifdef FEATURE_CYTTSP_HEARTBEAT
#define CYTTSP_HEARTBEAT_TIME	3
#endif

#define TOUCH_MAX_NUM	2

#define SENSOR_X	12
#define SENSOR_Y	20
#define MAX_NODE 	SENSOR_X*SENSOR_Y

#define CYTTSP_BASE_MIN	65
#define CYTTSP_BASE_MAX	135

/* ****************************************************************************
 * static value
 * ************************************************************************** */
static struct cyttsp_gen3_xydata_t g_xy_data;
static struct cyttsp_bootloader_data_t g_bl_data;
static struct cyttsp_sysinfo_data_t g_sysinfo_data;
static struct cyttsp_gen3_xydata_t g_wake_data;
static const struct i2c_device_id cyttsp_id[] = {
	{ CYTTSP_I2C_NAME, 0 },  { }
};

/* CY TTSP I2C Driver private data */
struct cyttsp {
	struct i2c_client *client;
	struct input_dev *input;
	struct work_struct work;
#ifdef FEATURE_CYTTSP_HEARTBEAT
	struct work_struct work2;
#endif
#ifdef FEATURE_CYTTSP_FIRMWAREUPGRADE
	struct work_struct work3;
#endif
	struct timer_list timer;
	struct mutex mutex;
	char phys[32];
	struct cyttsp_platform_data *platform_data;
	u8 num_prev_st_touch;
	u16 active_track[CYTTSP_NUM_TRACK_ID];
	u16 prev_st_touch[CYTTSP_NUM_ST_TOUCH_ID];
	u16 prev_mt_touch[CYTTSP_NUM_MT_TOUCH_ID];
	u16 prev_mt_pos[CYTTSP_NUM_MT_TOUCH_ID][2];
	atomic_t irq_enabled;
	struct early_suspend early_suspend;
};

#ifdef FEATURE_CYTTSP_HEARTBEAT
static int start_heartbeat_timer = false;
#endif

#ifdef FEATURE_SKY_PROCESS_CMD_KEY
struct cyttsp *cyttsp_data = NULL;
#endif

/* To check touch chip */
static int Touch_Dbg_Enable =0;
static u16 prev_mt_pos[CYTTSP_NUM_TRACK_ID][2];
static struct wake_lock touch_wake_lock;

typedef enum touch_status {
        TOUCH_POWERON,
        TOUCH_POWEROFF,
        TOUCH_UPDATE
} touch_status;

typedef enum
{
        BATTERY_PLUGGED_NONE = 0,
        BATTERY_PLUGGED_AC = 1,
        BATTERY_PLUGGED_USB = 2,
        BATTERY_PLUGGED_SLEEP = 10
} CHARGER_MODE;

typedef enum touch_ioctl {
	TOUCH_CHARGER_MODE = 701,

       TOUCH_IOCTL_READ_LASTKEY = 1001,
       TOUCH_IOCTL_DO_KEY,
       TOUCH_IOCTL_RELEASE_KEY,

	TOUCH_IOCTL_PRESS_TOUCH = 1007,
	TOUCH_IOCTL_RELEASE_TOUCH,

	TOUCH_IOCTL_SENSOR_X = 2005,
	TOUCH_IOCTL_SENSOR_Y,
	TOUCH_IOCTL_CHECK_BASE,
	TOUCH_IOCTL_READ_IC_VERSION,
	TOUCH_IOCTL_READ_FW_VERSION,	
	TOUCH_IOCTL_START_UPDATE,
	TOUCH_IOCTL_SELF_TEST,
	TOUCH_IOCTL_SET_COLOR
} touch_ioctl;


static int Touch_Status =TOUCH_POWERON;

static int Touch_ChagerMode = BATTERY_PLUGGED_NONE;

static unsigned char bBlack=false;

struct cyttsp *ts_temp;

#if defined(CONFIG_APACHE_BOARD) 
struct delayed_work    work_delay_firmware;
#endif

MODULE_DEVICE_TABLE(i2c, cyttsp_id);

/* ****************************************************************************
 * Prototypes for static functions
 * ************************************************************************** */
static void cyttsp_xy_worker(struct work_struct *work);
#ifdef FEATURE_CYTTSP_HEARTBEAT
static void cyttsp_check_heartbeat(struct work_struct *work2);
#endif
#ifdef FEATURE_CYTTSP_FIRMWAREUPGRADE
void check_firmware_update(struct work_struct *work3);
#endif
static irqreturn_t cyttsp_irq(int irq, void *handle);
static int cyttsp_inlist(u16 prev_track[], u8 curr_track_id, u8 *prev_loc, u8 num_touches);
static int cyttsp_next_avail_inlist(u16 curr_track[], u8 *new_loc, u8 num_touches);
static int cyttsp_putbl(struct cyttsp *ts, int show, int show_status, int show_version, int show_cid);
static int __devinit cyttsp_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __devexit cyttsp_remove(struct i2c_client *client);
static int cyttsp_resume(struct i2c_client *client);
static int cyttsp_suspend(struct i2c_client *client, pm_message_t message);

#ifdef FEATURE_SKY_PROCESS_CMD_KEY
static int ts_fops_ioctl(struct inode *inode, struct file *filp,unsigned int cmd, unsigned long arg);
static int ts_fops_open(struct inode *inode, struct file *filp);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cyttsp_early_suspend(struct early_suspend *handler);
static void cyttsp_late_resume(struct early_suspend *handler);
#endif /* CONFIG_HAS_EARLYSUSPEND */

static u8 *pantech_auto_check(void);
static int firmware_update_by_user(void);
static int firmware_version_check(void);

static int pantech_selftest_check(void);

void Change_Active_Distance(u8 value); //test

/* ****************************************************************************
 * 
 * ************************************************************************** */
 
static struct i2c_driver cyttsp_driver = {
	.driver = {
		.name = CYTTSP_I2C_NAME,
		.owner = THIS_MODULE,
	},
	.probe = cyttsp_probe,
	.remove = __devexit_p(cyttsp_remove),
//	.suspend = cyttsp_suspend,
//	.resume = cyttsp_resume,
	.id_table = cyttsp_id,
};

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard touchscreen driver");
MODULE_AUTHOR("Cypress");

#ifdef FEATURE_SKY_PROCESS_CMD_KEY
struct cyttsp *sky_process_cmd_ts=NULL;

static struct file_operations ts_fops = {
	.owner = THIS_MODULE,
	.open = ts_fops_open,
//	.release = ts_fops_close,
	.ioctl = ts_fops_ioctl,
};

static struct miscdevice touch_event = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "touch_fops",
	.fops = &ts_fops,
};

static int ts_fops_open(struct inode *inode, struct file *filp)
{
	//filp->private_data = cyttsp_data;
	return 0;
}


static int ts_fops_ioctl(struct inode *inode, struct file *filp,
	       unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	if(cyttsp_data ==NULL)
	{
	       cyttsp_debug("Null Device\n");			
		return 0;
	}

       cyttsp_debug("cmd = %d, argp = 0x%x\n", cmd, (unsigned int)argp);	
//	printk("cmd = %d, argp = 0x%x\n", cmd, (unsigned int)argp);	

	switch (cmd) 
	{
		case TOUCH_IOCTL_READ_LASTKEY:
			break;
		case TOUCH_IOCTL_DO_KEY:
			if ( (int)argp == 0x20a )
				input_report_key(cyttsp_data->input, 0xe3, 1);
			else if ( (int)argp == 0x20b )
				input_report_key(cyttsp_data->input, 0xe4, 1);
			else
				input_report_key(cyttsp_data->input, (int)argp, 1);

			if((int)argp == KEY_9)
			{
				printk("Enable Touch Debug!!\n");
				Touch_Dbg_Enable = true;
			}
			else if((int)argp == KEY_8)
			{
				printk("Disable Touch Debug!!\n");
				Touch_Dbg_Enable = false;
			}
			/*
			else if((int)argp == KEY_F2)
			{
				int ret = 0;
				printk("Start Touch Firmware update!!\n");
				ret = firmware_update_by_user();
			}
			*/
			break;
		case TOUCH_IOCTL_RELEASE_KEY:		
			if ( (int)argp == 0x20a )
				input_report_key(cyttsp_data->input, 0xe3, 0);
			else if ( (int)argp == 0x20b )
				input_report_key(cyttsp_data->input, 0xe4, 0);
			else
				input_report_key(cyttsp_data->input, (int)argp, 0);
			break;
		// +++ FEATURE_P_VZW_PS_STABILITY_AT_CMD	
		case TOUCH_IOCTL_PRESS_TOUCH:
			{
				int touchX=arg&0x0000FFFF;
				int touchY= (arg >> 16) & 0x0000FFFF;
				
				input_report_abs(cyttsp_data->input, ABS_MT_TOOL_TYPE , 1);
				input_report_abs(cyttsp_data->input, ABS_MT_TOUCH_MAJOR, CYTTSP_TOUCH);
	            input_report_abs(cyttsp_data->input, ABS_MT_WIDTH_MAJOR, CYTTSP_SMALL_TOOL_WIDTH); 
				input_report_abs(cyttsp_data->input, ABS_MT_POSITION_X, touchX);
	            input_report_abs(cyttsp_data->input, ABS_MT_POSITION_Y, touchY);
				CYTTSP_MT_SYNC(cyttsp_data->input);
				input_sync(cyttsp_data->input);
     	   		}
			break;
		
		case TOUCH_IOCTL_RELEASE_TOUCH:		
			{
				int touchX=arg&0x0000FFFF;
				int touchY= (arg >> 16) & 0x0000FFFF;
				
				input_report_abs(cyttsp_data->input, ABS_MT_TOOL_TYPE , 1);
				input_report_abs(cyttsp_data->input, ABS_MT_TOUCH_MAJOR, CYTTSP_NOTOUCH);
	            input_report_abs(cyttsp_data->input, ABS_MT_WIDTH_MAJOR, CYTTSP_SMALL_TOOL_WIDTH);
				input_report_abs(cyttsp_data->input, ABS_MT_POSITION_X, touchX);
	            input_report_abs(cyttsp_data->input, ABS_MT_POSITION_Y, touchY);

				CYTTSP_MT_SYNC(cyttsp_data->input);
				input_sync(cyttsp_data->input);
			}
	       	break;

		// --- 
		case TOUCH_IOCTL_SENSOR_X:
			{
				int send_data;

				send_data = SENSOR_X;
				
				if (copy_to_user(argp, &send_data, sizeof(send_data)))
					return false;
			}
			break;
			
		case TOUCH_IOCTL_SENSOR_Y:
			{
				int send_data;

				send_data = SENSOR_Y;
				
				if (copy_to_user(argp, &send_data, sizeof(send_data)))
					return false;
			}
			break;

		case TOUCH_IOCTL_CHECK_BASE:
			{
				u8 *send_byte;
				//printk("TOUCH_IOCTL_CHECK_BASE!!\n");
				
				disable_irq_nosync(ts_temp->client->irq);
				send_byte = pantech_auto_check();
				enable_irq(ts_temp->client->irq);
					
				if (copy_to_user(argp, send_byte, MAX_NODE))
					return false;
			} 
			break;

		case TOUCH_IOCTL_READ_IC_VERSION:
			{
				int ret = 0;
				ret = firmware_version_check();
				if (copy_to_user(argp, &ret, sizeof(ret)))
					return false;
			}
			break;

		case TOUCH_IOCTL_READ_FW_VERSION:
			{
				int ret =0;
				
				if(bBlack == false)	// White Model etc..
					ret = CYTTPS_NONBLACK_FIRMWARE_VER_ID;
				else					// Black Model
					ret = CYTTPS_FIRMWARE_VER_ID;
				
				if (copy_to_user(argp, &ret, sizeof(ret)))
					return false;
			}
			break;

		case TOUCH_IOCTL_START_UPDATE:
			{
				int ret = 0;
				ret = firmware_update_by_user(); // if ret == 0 success, or not fail
				printk("TOUCH_IOCTL_START_UPDATE ret  : %d\n", ret);
				if (copy_to_user(argp, &ret, sizeof(ret)))
					return false;
			}
			break;

		case TOUCH_CHARGER_MODE:
			printk("TOUCH_CHARGER_MODE Setting : %d\n", arg);
			Touch_ChagerMode = arg;
			break;

		case TOUCH_IOCTL_SELF_TEST:
			{
				int ret = 0;
				ret = pantech_selftest_check();
				if (copy_to_user(argp, &ret, sizeof(ret)))
					return false;
			}
			break;
		case TOUCH_IOCTL_SET_COLOR:
			bBlack = arg;
			break;

		default:
	     		break;
	}

	return true;
}
#endif

void Change_Active_Distance(u8 value)
{
	int rc = -1;
	u8 byte_data;
	struct cyttsp *ts = ts_temp;
	
	rc = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_GEST_SET,sizeof(byte_data), &byte_data);

	printk("Chage_Active_Distance : %02x\n", byte_data);

	byte_data = value;

	rc = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_GEST_SET, sizeof(byte_data), &byte_data);

	return;
}

static ssize_t cyttsp_irq_status(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev,
	                                         struct i2c_client, dev);
	struct cyttsp *ts = i2c_get_clientdata(client);
	return sprintf(buf, "%u\n", atomic_read(&ts->irq_enabled));
}

static ssize_t cyttsp_irq_enable(struct device *dev,
                                 struct device_attribute *attr,
                                 const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev,
	                                         struct i2c_client, dev);
	struct cyttsp *ts = i2c_get_clientdata(client);
	int err = 0;
	unsigned long value;
/*
	struct qtm_obj_message *msg;
*/

	if (size > 2)
		return -EINVAL;

	err = strict_strtoul(buf, 10, &value);
	if (err != 0)
		return err;

	switch (value) {
	case 0:
		if (atomic_cmpxchg(&ts->irq_enabled, 1, 0)) {
			pr_info("touch irq disabled!\n");
			disable_irq_nosync(ts->client->irq);
		}
		err = size;
		break;
	case 1:
		if (!atomic_cmpxchg(&ts->irq_enabled, 0, 1)) {
			pr_info("touch irq enabled!\n");
/*
			msg = cyttsp_read_msg(ts);
			if (msg == NULL)
				pr_err("%s: Cannot read message\n", __func__);
*/
			enable_irq(ts->client->irq);
		}
		err = size;
		break;
	default:
		pr_info("cyttsp_irq_enable failed -> irq_enabled = %d\n",
		atomic_read(&ts->irq_enabled));
		err = -EINVAL;
		break;
	}

	return err;
}

static DEVICE_ATTR(irq_enable, 0664, cyttsp_irq_status, cyttsp_irq_enable);

int pantech_ctl_update(int cmd, int value)
{
	int rt = -1;
	struct regulator *vreg_touch, *vreg_touch_temp;
	
	switch(cmd)
	{
		case ISSP_IOCTL_SCLK_TO_GPIO:
			if(value){
				gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
			}
			else{
				gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_DISABLE);
				gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_SCL, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
			}
			rt = 1;
			break;
		
		case ISSP_IOCTL_DATA_TO_GPIO:
			if(value){
				gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
			}
			else{
				gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_DISABLE);
				gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_SDA, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
			}
			rt = 1;
			break;
		
		case ISSP_IOCTL_SCLK:
			gpio_set_value(GPIO_TOUCH_SCL, value);
			rt = 1;
			break;

		case ISSP_IOCTL_DATA:
			gpio_set_value(GPIO_TOUCH_SDA, value);
			rt = 1;
			break;

		case ISSP_IOCTL_RESET:
			break;
			
		case ISSP_IOCTL_POWER:

			printk("Touch Power: %d, cmd: %d\n", value, cmd);

#if EF33S_BDVER_GE(WS20) || EF34K_BDVER_GE(WS20)
			vreg_touch = regulator_get(NULL, "8058_l11");
			regulator_set_voltage(vreg_touch, 1900000, 1900000);
#else
			vreg_touch = regulator_get(NULL, "8058_lvs0");
#endif
			if(value)
				rt = regulator_enable(vreg_touch);
			else	
				rt = regulator_disable(vreg_touch);

			regulator_put(vreg_touch);
			break;

		case ISSP_IOCTL_POWER_ALL:

			printk("Touch Power All: %d, cmd: %d\n", value, cmd);

			vreg_touch_temp = regulator_get(NULL, "8058_l19");

#if EF33S_BDVER_GE(WS20) || EF34K_BDVER_GE(WS20)
			vreg_touch = regulator_get(NULL, "8058_l11");
			regulator_set_voltage(vreg_touch, 1900000, 1900000);
#else
 			vreg_touch = regulator_get(NULL, "8058_lvs0");
#endif

			regulator_set_voltage(vreg_touch_temp, 3000000, 3000000);	

			if(value)
			{
				rt = regulator_enable(vreg_touch);
				rt = regulator_enable(vreg_touch_temp);
			}
			else	
			{
				rt = regulator_disable(vreg_touch);
				rt = regulator_disable(vreg_touch_temp);
			}

			regulator_put(vreg_touch);
			regulator_put(vreg_touch_temp);		
			break;

		
		case ISSP_IOCTL_READ_DATA_PIN:
			rt = gpio_get_value(GPIO_TOUCH_SDA);
			break;
		
		case ISSP_IOCTL_WAIT:
			udelay(value);
			break;
		
		case ISSP_IOCTL_INTR:
			gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_CHG, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);		
			gpio_set_value(GPIO_TOUCH_CHG, value);
			rt = 1;
			break;

		case ISSP_TEST_READ_SCL:
			rt = gpio_get_value(GPIO_TOUCH_SCL);
			break;

		case ISSP_TEST_READ_SDA:
			rt = gpio_get_value(GPIO_TOUCH_SDA);
			break;
		
		case ISSP_TEST_READ_RESET:
			rt = gpio_get_value(GPIO_TOUCH_RST);
			break;
		
		case ISSP_COMPLITED_UPDATA:
			gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_CHG, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
			gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_DISABLE);
			gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_DISABLE);
			gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
			gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);                    
			rt = 1;
			break;                        
			
		default:
			dbg("UNKNOWN CMD\n");
			break;
	}
	return rt;
}

#ifdef FEATURE_CYTTSP_FIRMWAREUPGRADE
void check_firmware_update(struct work_struct *work3)
{
	int retry_cnt = 3;
	u8 byte_data[4];
	int rc = -1, check_update_pass = 0, curr_version =0;
	struct cyttsp *ts = ts_temp;
		
	// If phone enter a poweroff, Stop firmware update
	if(Touch_Status >= TOUCH_POWEROFF)
		return;

#ifdef FEATURE_CYTTSP_HEARTBEAT
	start_heartbeat_timer = false;
#endif

	wake_lock(&touch_wake_lock);

	disable_irq(ts->client->irq);

	do {
		rc = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_READ_VER_ID,sizeof(byte_data), (u8 *)&byte_data);
		udelay(2*1000);
	} while ((rc < CYTTSP_OPERATIONAL) && --retry_cnt);

	dbg("i2c communcation1 %s, byte_data = %d, %d, %d, %d\n", (rc < CYTTSP_OPERATIONAL) ? "FAIL" : "PASS",byte_data[0],byte_data[1],byte_data[2],byte_data[3]);

	if((int)byte_data[0] == 0 || rc < CYTTSP_OPERATIONAL)
	{
		dbg("Retry read firmware version!\n");
	
		msleep(200);
		pantech_ctl_update(ISSP_IOCTL_POWER, 0);
		msleep(100);
		pantech_ctl_update(ISSP_IOCTL_POWER, 1);	
		msleep(200);

		retry_cnt = 3;

		do {
			rc = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_READ_VER_ID,sizeof(byte_data), (u8 *)&byte_data);
			udelay(2*1000);
		} while ((rc < CYTTSP_OPERATIONAL) && --retry_cnt);			
	}

	pantech_ctl_update(ISSP_IOCTL_INTR,0);

#ifdef FEATURE_SKY_NONBLACK_FIRMWARE
	if(rc >= CYTTSP_OPERATIONAL) // read success
		bBlack =  (int)byte_data[0] % 2;

	if(bBlack == false)	// White Model etc..
		curr_version = CYTTPS_NONBLACK_FIRMWARE_VER_ID;
	else					// Black Model
		curr_version = CYTTPS_FIRMWARE_VER_ID;
#else
	curr_version = CYTTPS_FIRMWARE_VER_ID;
#endif

	dbg("[Touch] Model Black Check: %d, Current Version: %d\n", bBlack, curr_version);

	if(((curr_version > byte_data[0])  && (byte_data[0] != 0)) || (rc < CYTTSP_OPERATIONAL))
	{
		retry_cnt = 5;
		
		dbg("Start Firmware Update chip id: %d\n", byte_data[0]);				
		
		do{
			check_update_pass = touch_update_main(bBlack);
			udelay(2*1000);
		}while((check_update_pass != 0) && --retry_cnt);
	}

	pantech_ctl_update(ISSP_IOCTL_INTR,1);
	pantech_ctl_update(ISSP_COMPLITED_UPDATA,0);

	if(check_update_pass != 0)
	{
		msleep(200);
		pantech_ctl_update(ISSP_IOCTL_POWER, 0);
		msleep(100);
		pantech_ctl_update(ISSP_IOCTL_POWER, 1);	
		msleep(100);
		cancel_work_sync(&ts->work);
	}

	dbg("check_firmware_update end!!\n");
	
	enable_irq(ts->client->irq);

#ifdef FEATURE_CYTTSP_HEARTBEAT
	start_heartbeat_timer = true;
#endif

	wake_unlock(&touch_wake_lock);

	return;
}

static int firmware_version_check(void)
{
	int rc = -1, retry_cnt = 3;
	u8 byte_data[4];
	struct cyttsp *ts = ts_temp;

	if(Touch_Status >= TOUCH_POWEROFF)
	{
		pantech_ctl_update(ISSP_IOCTL_POWER_ALL , 1);
		pantech_ctl_update(ISSP_IOCTL_INTR, 1);
		pantech_ctl_update(ISSP_COMPLITED_UPDATA, 0);
		msleep(300);
//		return false;
	}	

	do {
		rc = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_READ_VER_ID,sizeof(byte_data), (u8 *)&byte_data);
		udelay(2*1000);
	}
	while ((rc < CYTTSP_OPERATIONAL) && --retry_cnt);

	if(rc < CYTTSP_OPERATIONAL)
	{
		printk("Can't read Touch Firmware Version\n");
		return 1;
	}

	printk("Touch Firmware Update Version : %d, Current Version: %d\n" ,
		CYTTPS_FIRMWARE_VER_ID,(int)byte_data[0]);

	if(Touch_Status >= TOUCH_POWEROFF)
	{
		pantech_ctl_update(ISSP_IOCTL_POWER_ALL , 0);
	}

	return (int)byte_data[0];	// Need not 
}


static int firmware_set_charger_mode(int mode)
{
	int rc = -1, retry_cnt = 3;
	u8 byte_data[4], send_byte = 0x00;
	struct cyttsp *ts = ts_temp;

	do {
		rc = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_CHARGER_MODE,sizeof(byte_data), (u8 *)&byte_data);
		udelay(2*1000);
	}
	while ((rc < CYTTSP_OPERATIONAL) && --retry_cnt);

	if(rc < CYTTSP_OPERATIONAL)
	{
		printk("Can't read Touch Charger Mode\n");
		return 1;
	}

	if(Touch_Dbg_Enable)
		printk("Touch IC Charger Mode %02x\n" ,(int)byte_data[0]);

	if(mode > BATTERY_PLUGGED_NONE)	// charger mode on
	{
		if((int)byte_data[0] != CYTTPS_CHARGER_MODE)
		{
			send_byte = CYTTPS_CHARGER_MODE;
			rc = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_CHARGER_MODE, sizeof(send_byte), &send_byte);
		}
	}
	else // charger mode off
	{
		if((int)byte_data[0] != 0x00)
		{
			send_byte = 0x00;
			rc = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_CHARGER_MODE, sizeof(send_byte), &send_byte);
		}
	}
	
	return 0;
}


static int firmware_update_by_user(void)
{
	struct cyttsp *ts = ts_temp;

	int check_update_pass = -1;

	// If phone enter a poweroff, Stop firmware update
	if(Touch_Status >= TOUCH_POWEROFF)
	{
		pantech_ctl_update(ISSP_IOCTL_POWER_ALL , 1);
		msleep(300);
//		return false;
	}

#ifdef FEATURE_CYTTSP_HEARTBEAT
	start_heartbeat_timer = false;
#endif

	Touch_Status= TOUCH_UPDATE;

	wake_lock(&touch_wake_lock);

	/*���ͷ��� PIN HIGH ���¸� �����ϱ� ���� IRQ�� ������ */
	disable_irq(ts->client->irq);

	/* ��Ʈ���� PIN HIGH ���·� ���Ͽ� 2.6V_TOUCH ���� OFF�Ͽ��� 1.2V ���� �帣�� �������� Value���� 0���� ���� */
	pantech_ctl_update(ISSP_IOCTL_INTR,0);

	check_update_pass = touch_update_main(bBlack);

	pantech_ctl_update(ISSP_IOCTL_INTR,1);
	pantech_ctl_update(ISSP_COMPLITED_UPDATA,0);

	msleep(100);
	pantech_ctl_update(ISSP_IOCTL_POWER, 0);
	msleep(100);
	pantech_ctl_update(ISSP_IOCTL_POWER, 1);	
	msleep(100);

	cancel_work_sync(&ts->work);

	enable_irq(ts->client->irq);

#ifdef FEATURE_CYTTSP_HEARTBEAT
	start_heartbeat_timer = true;
#endif

	wake_unlock(&touch_wake_lock);

	Touch_Status= TOUCH_POWERON;

	return check_update_pass;
}

#endif	

/* The cyttsp_xy_worker function reads the XY coordinates and sends them to
 * the input layer.  It is scheduled from the interrupt (or timer).
 */
#ifdef FEATURE_TOUCH_KEY
#define CYTTSP_MENU_KEY	0x01
#define CYTTSP_BACK_KEY	0x02
#define CYTTSP_HOME_KEY	0x04
#define CYTTSP_NULL_KEY	0x08

static int key_status = 0x00;
#endif

#ifdef FEATURE_SKY_TOUCH_DELTA_DEBUG
static u16 pre_x_data;
static u16 pre_y_data;
static u16 delta_x;
static u16 delta_y;
#endif

#ifdef FEATURE_CYTTSP_HEARTBEAT
void cyttsp_check_heartbeat(struct work_struct *work2)
{
	struct cyttsp *ts = container_of(work2,struct cyttsp,work2);
	
	int retry_cnt = 3;
	u8 new_heartbeart_data[4];
	int rc = -1;
	static u8 old_heartbeat_data = 0xFF;
	
	memset((void*)new_heartbeart_data,0x00,sizeof(new_heartbeart_data));

	if(start_heartbeat_timer == false)
		return;
	
	do {
		/* Read Heartbeat Count */
		rc = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_READ_HEARTBEAT,sizeof(new_heartbeart_data), (u8 *)&new_heartbeart_data);
	}
	while ((rc < CYTTSP_OPERATIONAL) && --retry_cnt);

	if(Touch_Dbg_Enable)
		printk("##### Check Count = %s, byte_data = %d, %d, %d, %d\n", (rc < CYTTSP_OPERATIONAL) ? "FAIL" : "PASS",new_heartbeart_data[0],new_heartbeart_data[1],new_heartbeart_data[2],new_heartbeart_data[3]);

	if(start_heartbeat_timer == false)
		return;

	if(rc < CYTTSP_OPERATIONAL || old_heartbeat_data == new_heartbeart_data[0])
	{
		/* I2c error or Touch Chip's heartbeat value is not change */
		disable_irq(ts->client->irq);	
		pantech_ctl_update(ISSP_IOCTL_INTR,0);
		pantech_ctl_update(ISSP_IOCTL_POWER,0);
		msleep(200);
		pantech_ctl_update(ISSP_IOCTL_INTR,1);
		gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_CHG, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);			
		enable_irq(ts->client->irq);
		pantech_ctl_update(ISSP_IOCTL_POWER,1);

		if(Touch_Dbg_Enable)
			printk("HeartBeat Fail old_data = %d, new_data = %d",old_heartbeat_data, new_heartbeart_data[0]);
	}

	if(!start_heartbeat_timer)
		old_heartbeat_data = 0xFF;	

	// Set Charger Mode
	firmware_set_charger_mode(Touch_ChagerMode);

	return;
}
#endif

void cyttsp_xy_worker(struct work_struct *work)
{
	struct cyttsp *ts = container_of(work,struct cyttsp,work);
	u8 id, tilt, reverse_x, reverse_y;
	u8 i, loc;
	u8 prev_touches;
	u8 curr_touches;
	u16 temp_track[CYTTSP_NUM_MT_TOUCH_ID];
	u16 send_track[CYTTSP_NUM_MT_TOUCH_ID];
	u16 curr_track[CYTTSP_NUM_TRACK_ID];
	u16 curr_st_touch[CYTTSP_NUM_ST_TOUCH_ID];
	u16 curr_mt_touch[CYTTSP_NUM_MT_TOUCH_ID];
	u16 curr_mt_pos[CYTTSP_NUM_TRACK_ID][2];	/* if NOT CYTTSP_USE_TRACKING_ID then only uses CYTTSP_NUM_MT_TOUCH_ID positions */
	u8 curr_mt_z[CYTTSP_NUM_TRACK_ID];			/* if NOT CYTTSP_USE_TRACKING_ID then only uses CYTTSP_NUM_MT_TOUCH_ID positions */
	u8 curr_tool_width;
	u16 st_x1, st_y1;
	u8 st_z1;
	u16 st_x2, st_y2;
	u8 st_z2;
	s32 retval;
#ifdef FEATURE_TOUCH_KEY	
	int key_relese = true;
#endif

	if(Touch_Status >= TOUCH_POWEROFF)
		goto exit_xy_worker;

	/* get event data from CYTTSP device */
	i = CYTTSP_NUM_RETRY;
	do {
		retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE,
			sizeof(struct cyttsp_gen3_xydata_t), (u8 *)&g_xy_data);
	}
	while ((retval < CYTTSP_OPERATIONAL) && --i);
	
	/* return immediately on failure to read device on the i2c bus */
	if (retval < CYTTSP_OPERATIONAL) {
		cyttsp_debug("exit_xy_worker 1");
		goto exit_xy_worker;
	}
	
	cyttsp_xdebug("TTSP worker start 2:\n");

	if ((curr_touches = GET_NUM_TOUCHES(g_xy_data.tt_stat)) > CYTTSP_NUM_MT_TOUCH_ID) {
		/* if the number of fingers on the touch surface is more than the maximum
		 * then there will be no new track information even for the orginal
		 * touches. Therefore, ignore this touch event.
		 */
		cyttsp_debug("exit_xy_worker 2");		 
		 goto exit_xy_worker;
	}
	else if (IS_LARGE_AREA(g_xy_data.tt_stat)==1) {
		/* terminate all active tracks */
		curr_touches = CYTTSP_NOTOUCH;
		cyttsp_debug("Large object detected. Terminating active tracks\n");
	}

	/* set tool size */
	curr_tool_width = CYTTSP_SMALL_TOOL_WIDTH;

	/* translate Gen2 interface data into comparable Gen3 data */
	if(ts->platform_data->gen == CYTTSP_GEN2) {
		struct cyttsp_gen2_xydata_t *pxy_gen2_data;
		pxy_gen2_data = (struct cyttsp_gen2_xydata_t *)(&g_xy_data);

		/* use test data? */
		cyttsp_testdat(&g_xy_data, &tt_gen2_testray, sizeof(struct cyttsp_gen3_xydata_t));

		if (pxy_gen2_data->evnt_idx == CYTTSP_GEN2_NOTOUCH) {
			curr_touches = 0;
		}
		else if (curr_touches == CYTTSP_GEN2_GHOST) {
			curr_touches = 0;
		}
		else if (curr_touches == CYTTSP_GEN2_2TOUCH) {
			g_xy_data.touch12_id = 0x12;	/* stuff artificial track ID1 and ID2 */
			g_xy_data.z1 = CYTTSP_MAXZ;
			g_xy_data.z2 = CYTTSP_MAXZ;
			curr_touches--;			/* 2 touches */
		}
		else if (curr_touches == CYTTSP_GEN2_1TOUCH) {
			g_xy_data.touch12_id = 0x12;	/* stuff artificial track ID1 and ID2 */
			g_xy_data.z1 = CYTTSP_MAXZ;
			g_xy_data.z2 = CYTTSP_NOTOUCH;
			if (pxy_gen2_data->evnt_idx == CYTTSP_GEN2_TOUCH2) {
				/* push touch 2 data into touch1 (first finger up; second finger down) */
				g_xy_data.touch12_id = 0x20;	/* stuff artificial track ID1 for touch 2 info */
				g_xy_data.x1 = g_xy_data.x2;	/* stuff touch 1 with touch 2 coordinate data */
				g_xy_data.y1 = g_xy_data.y2;
			}
		}
		else {
			curr_touches = 0;
		}
	}
	else {
		/* use test data? */
		cyttsp_testdat(&g_xy_data, &tt_gen3_testray, sizeof(struct cyttsp_gen3_xydata_t));
	}
	
	/* clear current active track ID array and count previous touches */
	for (id = 0, prev_touches = CYTTSP_NOTOUCH; id < CYTTSP_NUM_TRACK_ID; id++) {
		curr_track[id] = CYTTSP_NOTOUCH;
		prev_touches += ts->active_track[id];
	}

	if(Touch_Dbg_Enable)
		printk("[Touch] stat: %02x, X1: %d, Y1: %d, X2: %d, Y2: %d prev_touches: %d, curr_touches: %d, Touch ID1: %02x, Touch ID2: %02x\n", 
			g_xy_data.tt_stat, be16_to_cpu(g_xy_data.x1),  be16_to_cpu(g_xy_data.y1), 
			be16_to_cpu(g_xy_data.x2),  be16_to_cpu(g_xy_data.y2), prev_touches, curr_touches, 
			g_xy_data.touch12_id, g_xy_data.touch34_id);		

	if(Touch_Dbg_Enable) // Debug Code
	{
		if(curr_touches == 1 && (be16_to_cpu(g_xy_data.x1) > ts->platform_data->maxx || be16_to_cpu(g_xy_data.y1) > ts->platform_data->maxy))
			printk("[Touch] stat: %02x, X1: %d, Y1: %d, prev_touches: %d, curr_touches: %d\n", 
			g_xy_data.tt_stat, be16_to_cpu(g_xy_data.x1),  be16_to_cpu(g_xy_data.y1), prev_touches, curr_touches);				

		if(curr_touches == 2 && (be16_to_cpu(g_xy_data.x2) > ts->platform_data->maxx || be16_to_cpu(g_xy_data.y2) > ts->platform_data->maxy))
			printk("[Touch] stat: %02x, X2: %d, Y2: %d, prev_touches: %d, curr_touches: %d\n", 
			g_xy_data.tt_stat, be16_to_cpu(g_xy_data.x2),  be16_to_cpu(g_xy_data.y2), prev_touches, curr_touches);				
	}
	
#ifdef FEATURE_TOUCH_KEY

	for(i=0; i<curr_touches; i++)
	{
		int x =0, y=0;
	
		switch(i)
		{
			case 0:
				x = be16_to_cpu(g_xy_data.x1);
				y = be16_to_cpu(g_xy_data.y1);
				break;
			case 1:
				x = be16_to_cpu(g_xy_data.x2);
				y = be16_to_cpu(g_xy_data.y2);
				break;
			case 2:
				x = be16_to_cpu(g_xy_data.x3);
				y = be16_to_cpu(g_xy_data.y3);
				break;
			case 3:
				x = be16_to_cpu(g_xy_data.x4);
				y = be16_to_cpu(g_xy_data.y4);
				break;
			default:
				break;
		}

		if(y > Y_MAX)
		{
			key_relese = false;

			if(y < NULL_KEY_AREA && (!key_status || key_status == CYTTSP_NULL_KEY))
			{
				key_status = CYTTSP_NULL_KEY;
				dbg("Down TOUCH NULL\n");
			}
			else if((MENU_KEY_MIN < x &&  x < MENU_KEY_MAX) && (!key_status || key_status == CYTTSP_MENU_KEY))
			{
				key_status = CYTTSP_MENU_KEY;
				input_report_key(ts->input, KEY_MENU, CYTTSP_TOUCH);
				dbg("Down TOUCH MENU\n");
				input_sync(ts->input);
			}
			else if((HOME_KEY_MIN < x &&  x < HOME_KEY_MAX) && (!key_status || key_status == CYTTSP_HOME_KEY))
			{
				key_status = CYTTSP_HOME_KEY;	
				input_report_key(ts->input, KEY_HOME, CYTTSP_TOUCH);
				dbg("Down TOUCH HOME\n");
				input_sync(ts->input);
			}		
			else if((BACK_KEY_MIN < x &&  x < BACK_KEY_MAX) && (!key_status || key_status == CYTTSP_BACK_KEY))
			{
				key_status = CYTTSP_BACK_KEY;				
				input_report_key(ts->input, KEY_BACK, CYTTSP_TOUCH);
				dbg("Down TOUCH BACK\n");
				input_sync(ts->input);
			}
			else if(!key_status)
				key_status = CYTTSP_NULL_KEY;
		}
	}

	if(key_relese && (curr_touches < prev_touches) && key_status)
	{
		if(key_status == CYTTSP_MENU_KEY)
			input_report_key(ts->input, KEY_MENU, CYTTSP_NOTOUCH);

		if(key_status == CYTTSP_HOME_KEY)
			input_report_key(ts->input, KEY_HOME, CYTTSP_NOTOUCH);

		if(key_status == CYTTSP_BACK_KEY)
			input_report_key(ts->input, KEY_BACK, CYTTSP_NOTOUCH);

		if(key_status != CYTTSP_NULL_KEY)
			input_sync(ts->input);

		dbg("Up Key: %02x\n", key_status);

		key_status = 0;

	}
	
#endif

	/* send no events if there were no previous touches and no new touches */
	if ((prev_touches == CYTTSP_NOTOUCH) && 
		((curr_touches == CYTTSP_NOTOUCH) || (curr_touches > CYTTSP_NUM_MT_TOUCH_ID))) {
		cyttsp_debug("exit_xy_worker 3");
		goto exit_xy_worker;
	}

	cyttsp_debug("prev=%d  curr=%d\n", prev_touches, curr_touches);

	/* clear current single touches array */
	for (id = 0; id < CYTTSP_NUM_ST_TOUCH_ID; id++) {
		curr_st_touch[id] = CYTTSP_IGNORE_TOUCH;
	}

	/* clear single touch positions */
	st_x1 = CYTTSP_NOTOUCH;
	st_y1 = CYTTSP_NOTOUCH;
	st_z1 = CYTTSP_NOTOUCH;
	st_x2 = CYTTSP_NOTOUCH;
	st_y2 = CYTTSP_NOTOUCH;
	st_z2 = CYTTSP_NOTOUCH;

	/* clear current multi-touches array and multi-touch positions/z */
	for (id = 0; id < CYTTSP_NUM_MT_TOUCH_ID; id++) {
		curr_mt_touch[id] = CYTTSP_IGNORE_TOUCH;
	}

	if (ts->platform_data->use_trk_id) {
		for (id = 0; id < CYTTSP_NUM_MT_TOUCH_ID; id++) {
			curr_mt_pos[id][CYTTSP_XPOS] = 0;
			curr_mt_pos[id][CYTTSP_YPOS] = 0;
			curr_mt_z[id] = 0;
		}
	}
	else {
		for (id = 0; id < CYTTSP_NUM_TRACK_ID; id++) {
			curr_mt_pos[id][CYTTSP_XPOS] = 0;
			curr_mt_pos[id][CYTTSP_YPOS] = 0;
			curr_mt_z[id] = 0;
		}
	}

	/* Determine if display is tilted */
	if (FLIP_DATA(ts->platform_data->flags)) {
		tilt = true;
	}
	else {
		tilt = false;
	}
	/* Check for switch in origin */
	if (REVERSE_X(ts->platform_data->flags)) {
		reverse_x = true;
	}
	else {
		reverse_x = false;
	}
	if (REVERSE_Y(ts->platform_data->flags)) {
		reverse_y = true;
	}
	else {
		reverse_y = false;
	}

	if (curr_touches) {
		struct cyttsp_gen2_xydata_t *pxy_gen2_data;
		struct cyttsp_gen3_xydata_t *pxy_gen3_data;
		switch (ts->platform_data->gen) {
			case CYTTSP_GEN2: {
				pxy_gen2_data = (struct cyttsp_gen2_xydata_t *)(&g_xy_data);
				cyttsp_xdebug("TTSP Gen2 report:\n");
				cyttsp_xdebug("%02X %02X %02X  %04X %04X %02X  %02X  %04X %04X %02X\n", \
						pxy_gen2_data->hst_mode, pxy_gen2_data->tt_mode, pxy_gen2_data->tt_stat, \
						pxy_gen2_data->x1, pxy_gen2_data->y1, pxy_gen2_data->z1, \
						pxy_gen2_data->evnt_idx, \
						pxy_gen2_data->x2, pxy_gen2_data->y2, pxy_gen2_data->tt_undef1);
				cyttsp_xdebug("%02X %02X %02X\n", \
						pxy_gen2_data->gest_cnt, pxy_gen2_data->gest_id, pxy_gen2_data->gest_set);
				break;
			}
			case CYTTSP_GEN3:
			default: {
				pxy_gen3_data = (struct cyttsp_gen3_xydata_t *)(&g_xy_data);
				cyttsp_xdebug("TTSP Gen3 report:\n");
				cyttsp_xdebug("%02X %02X %02X  %04X %04X %02X  %02X  %04X %04X %02X\n", \
						pxy_gen3_data->hst_mode, pxy_gen3_data->tt_mode, pxy_gen3_data->tt_stat, \
						pxy_gen3_data->x1, pxy_gen3_data->y1, pxy_gen3_data->z1, \
						pxy_gen3_data->touch12_id, \
						pxy_gen3_data->x2, pxy_gen3_data->y2, pxy_gen3_data->z2);
				cyttsp_xdebug("%02X %02X %02X  %04X %04X %02X  %02X  %04X %04X %02X\n", \
						pxy_gen3_data->gest_cnt, pxy_gen3_data->gest_id, pxy_gen3_data->gest_set, \
						pxy_gen3_data->x3, pxy_gen3_data->y3, pxy_gen3_data->z3, \
						pxy_gen3_data->touch34_id, \
						pxy_gen3_data->x4, pxy_gen3_data->y4, pxy_gen3_data->z4);
				break;
			}
		}
	}

	/* process the touches */
	switch (curr_touches) {
		case 4: {
			g_xy_data.x4 = be16_to_cpu(g_xy_data.x4);
			g_xy_data.y4 = be16_to_cpu(g_xy_data.y4);
			if (tilt) { 
				FLIP_XY(g_xy_data.x4, g_xy_data.y4);
			}
			if (reverse_x) {
				g_xy_data.x4 = INVERT_X(g_xy_data.x4, ts->platform_data->maxx);
			}
			if (reverse_y) {
				g_xy_data.y4 = INVERT_X(g_xy_data.y4, ts->platform_data->maxy);
			}
			id = GET_TOUCH4_ID(g_xy_data.touch34_id);
			if (ts->platform_data->use_trk_id) {
				curr_mt_pos[CYTTSP_MT_TOUCH4_IDX][CYTTSP_XPOS] = g_xy_data.x4;
				curr_mt_pos[CYTTSP_MT_TOUCH4_IDX][CYTTSP_YPOS] = g_xy_data.y4;
				curr_mt_z[CYTTSP_MT_TOUCH4_IDX] = g_xy_data.z4;
			}
			else {
				curr_mt_pos[id][CYTTSP_XPOS] = g_xy_data.x4;
				curr_mt_pos[id][CYTTSP_YPOS] = g_xy_data.y4;
				curr_mt_z[id] = g_xy_data.z4;
			}
			curr_mt_touch[CYTTSP_MT_TOUCH4_IDX] = id;
			curr_track[id] = CYTTSP_TOUCH;
			if (ts->prev_st_touch[CYTTSP_ST_FINGER1_IDX] < CYTTSP_NUM_TRACK_ID) {
				if (ts->prev_st_touch[CYTTSP_ST_FINGER1_IDX] == id) {
					st_x1 = g_xy_data.x4;
					st_y1 = g_xy_data.y4;
					st_z1 = g_xy_data.z4;
					curr_st_touch[CYTTSP_ST_FINGER1_IDX] = id;
				}
				else if (ts->prev_st_touch[CYTTSP_ST_FINGER2_IDX] == id) {
					st_x2 = g_xy_data.x4;
					st_y2 = g_xy_data.y4;
					st_z2 = g_xy_data.z4;
					curr_st_touch[CYTTSP_ST_FINGER2_IDX] = id;
				}
			}
			cyttsp_xdebug("4th XYZ:% 3d,% 3d,% 3d  ID:% 2d\n\n", \
				g_xy_data.x4, g_xy_data.y4, g_xy_data.z4, (g_xy_data.touch34_id & 0x0F));
			/* do not break */
		}
		case 3: {
			g_xy_data.x3 = be16_to_cpu(g_xy_data.x3);
			g_xy_data.y3 = be16_to_cpu(g_xy_data.y3);
			if (tilt) { 
				FLIP_XY(g_xy_data.x3, g_xy_data.y3);
			}
			if (reverse_x) {
				g_xy_data.x3 = INVERT_X(g_xy_data.x3, ts->platform_data->maxx);
			}
			if (reverse_y) {
				g_xy_data.y3 = INVERT_X(g_xy_data.y3, ts->platform_data->maxy);
			}
			id = GET_TOUCH3_ID(g_xy_data.touch34_id);
			if (ts->platform_data->use_trk_id) {
				curr_mt_pos[CYTTSP_MT_TOUCH3_IDX][CYTTSP_XPOS] = g_xy_data.x3;
				curr_mt_pos[CYTTSP_MT_TOUCH3_IDX][CYTTSP_YPOS] = g_xy_data.y3;
				curr_mt_z[CYTTSP_MT_TOUCH3_IDX] = g_xy_data.z3;
			}
			else {
				curr_mt_pos[id][CYTTSP_XPOS] = g_xy_data.x3;
				curr_mt_pos[id][CYTTSP_YPOS] = g_xy_data.y3;
				curr_mt_z[id] = g_xy_data.z3;
			}
			curr_mt_touch[CYTTSP_MT_TOUCH3_IDX] = id;
			curr_track[id] = CYTTSP_TOUCH;
			if (ts->prev_st_touch[CYTTSP_ST_FINGER1_IDX] < CYTTSP_NUM_TRACK_ID) {
				if (ts->prev_st_touch[CYTTSP_ST_FINGER1_IDX] == id) {
					st_x1 = g_xy_data.x3;
					st_y1 = g_xy_data.y3;
					st_z1 = g_xy_data.z3;
					curr_st_touch[CYTTSP_ST_FINGER1_IDX] = id;
				}
				else if (ts->prev_st_touch[CYTTSP_ST_FINGER2_IDX] == id) {
					st_x2 = g_xy_data.x3;
					st_y2 = g_xy_data.y3;
					st_z2 = g_xy_data.z3;
					curr_st_touch[CYTTSP_ST_FINGER2_IDX] = id;
				}
			}
			cyttsp_xdebug("3rd XYZ:% 3d,% 3d,% 3d  ID:% 2d\n", \
				g_xy_data.x3, g_xy_data.y3, g_xy_data.z3, ((g_xy_data.touch34_id >> 4) & 0x0F));
			/* do not break */

		}
		case 2: {
			g_xy_data.x2 = be16_to_cpu(g_xy_data.x2);
			g_xy_data.y2 = be16_to_cpu(g_xy_data.y2);
			if (tilt) {
				FLIP_XY(g_xy_data.x2, g_xy_data.y2);
			}
			if (reverse_x) {
				g_xy_data.x2 = INVERT_X(g_xy_data.x2, ts->platform_data->maxx);
			}
			if (reverse_y) {
				g_xy_data.y2 = INVERT_X(g_xy_data.y2, ts->platform_data->maxy);
			}
			id = GET_TOUCH2_ID(g_xy_data.touch12_id);
/*
			// touch ic is wrong work
			if(GET_TOUCH2_ID(g_xy_data.touch12_id) >= TOUCH_MAX_NUM || GET_TOUCH1_ID(g_xy_data.touch12_id) >= TOUCH_MAX_NUM)
				id = 1;
*/			
			if (ts->platform_data->use_trk_id) {
				curr_mt_pos[CYTTSP_MT_TOUCH2_IDX][CYTTSP_XPOS] = g_xy_data.x2;
				curr_mt_pos[CYTTSP_MT_TOUCH2_IDX][CYTTSP_YPOS] = g_xy_data.y2;
				curr_mt_z[CYTTSP_MT_TOUCH2_IDX] = g_xy_data.z2;
			}
			else {
				curr_mt_pos[id][CYTTSP_XPOS] = g_xy_data.x2;
				curr_mt_pos[id][CYTTSP_YPOS] = g_xy_data.y2;
				curr_mt_z[id] = g_xy_data.z2;
			}
			curr_mt_touch[CYTTSP_MT_TOUCH2_IDX] = id;
			curr_track[id] = CYTTSP_TOUCH;
			if (ts->prev_st_touch[CYTTSP_ST_FINGER1_IDX] < CYTTSP_NUM_TRACK_ID) {
				if (ts->prev_st_touch[CYTTSP_ST_FINGER1_IDX] == id) {
					st_x1 = g_xy_data.x2;
					st_y1 = g_xy_data.y2;
					st_z1 = g_xy_data.z2;
					curr_st_touch[CYTTSP_ST_FINGER1_IDX] = id;
				}
				else if (ts->prev_st_touch[CYTTSP_ST_FINGER2_IDX] == id) {
					st_x2 = g_xy_data.x2;
					st_y2 = g_xy_data.y2;
					st_z2 = g_xy_data.z2;
					curr_st_touch[CYTTSP_ST_FINGER2_IDX] = id;
				}
			}
			cyttsp_xdebug("2nd XYZ:% 3d,% 3d,% 3d  ID:% 2d\n", \
				g_xy_data.x2, g_xy_data.y2, g_xy_data.z2, (g_xy_data.touch12_id & 0x0F));
			/* do not break */

		}
		case 1:	{
			g_xy_data.x1 = be16_to_cpu(g_xy_data.x1);
			g_xy_data.y1 = be16_to_cpu(g_xy_data.y1);
			if (tilt) {
				FLIP_XY(g_xy_data.x1, g_xy_data.y1);
			}
			if (reverse_x) {
				g_xy_data.x1 = INVERT_X(g_xy_data.x1, ts->platform_data->maxx);
			}
			if (reverse_y) {
				g_xy_data.y1 = INVERT_X(g_xy_data.y1, ts->platform_data->maxy);
			}
			id = GET_TOUCH1_ID(g_xy_data.touch12_id);
/*
			// touch ic is wrong work
			if(GET_TOUCH2_ID(g_xy_data.touch12_id) >= TOUCH_MAX_NUM || GET_TOUCH1_ID(g_xy_data.touch12_id) >= TOUCH_MAX_NUM)
				id = 0;
*/			
			if (ts->platform_data->use_trk_id) {
				curr_mt_pos[CYTTSP_MT_TOUCH1_IDX][CYTTSP_XPOS] = g_xy_data.x1;
				curr_mt_pos[CYTTSP_MT_TOUCH1_IDX][CYTTSP_YPOS] = g_xy_data.y1;
				curr_mt_z[CYTTSP_MT_TOUCH1_IDX] = g_xy_data.z1;
			}
			else {
				curr_mt_pos[id][CYTTSP_XPOS] = g_xy_data.x1;
				curr_mt_pos[id][CYTTSP_YPOS] = g_xy_data.y1;
				curr_mt_z[id] = g_xy_data.z1;
			}
			curr_mt_touch[CYTTSP_MT_TOUCH1_IDX] = id;
			curr_track[id] = CYTTSP_TOUCH;
			if (ts->prev_st_touch[CYTTSP_ST_FINGER1_IDX] < CYTTSP_NUM_TRACK_ID) {
				if (ts->prev_st_touch[CYTTSP_ST_FINGER1_IDX] == id) {
					st_x1 = g_xy_data.x1;
					st_y1 = g_xy_data.y1;
					st_z1 = g_xy_data.z1;
					curr_st_touch[CYTTSP_ST_FINGER1_IDX] = id;
				}
				else if (ts->prev_st_touch[CYTTSP_ST_FINGER2_IDX] == id) {
					st_x2 = g_xy_data.x1;
					st_y2 = g_xy_data.y1;
					st_z2 = g_xy_data.z1;
					curr_st_touch[CYTTSP_ST_FINGER2_IDX] = id;
				}
			}

#ifdef FEATURE_SKY_TOUCH_DELTA_DEBUG
			if(prev_touches != curr_touches) 
			{
				pre_x_data = g_xy_data.x1;
				pre_y_data = g_xy_data.y1;
			}
#endif
			
			cyttsp_xdebug("1st XYZ:% 3d,% 3d,% 3d  ID:% 2d\n", \
				g_xy_data.x1, g_xy_data.y1, g_xy_data.z1, ((g_xy_data.touch12_id >> 4) & 0x0F));

			break;
		}
		case 0:
		default:{
			break;
		}
	}

#ifdef FEATURE_SKY_TOUCH_DELTA_DEBUG
	if((prev_touches == curr_touches) && (curr_touches == 1))
	{
		cyttsp_xdebug("##### x:% 3d, curr_x: % 3d, delta_x:% 3d \n", pre_x_data,g_xy_data.x1,delta_x);
		cyttsp_xdebug("##### y:% 3d, curr_y: % 3d, delta_y:% 3d \n", pre_y_data,g_xy_data.y1,delta_y);
						
		if(pre_x_data <= g_xy_data.x1)
		{
			delta_x = g_xy_data.x1 - pre_x_data;
		}
		else
		{
			delta_x = pre_x_data - g_xy_data.x1;
		}

		if(pre_y_data <= g_xy_data.y1)
		{
			delta_y = g_xy_data.y1 - pre_y_data;
		}
		else
		{
			delta_y = pre_y_data - g_xy_data.y1;
		}

		if((delta_x > TOUCH_DELTA_LIMIT) || (delta_y > TOUCH_DELTA_LIMIT))
		{
			pre_x_data = g_xy_data.x1;
			pre_y_data = g_xy_data.y1;
			cyttsp_debug("###### x,y data changed\n");
		}
		else
		{
			goto	exit_touch_event;
		}
	}
#endif

	/* handle Single Touch signals */
	if (ts->platform_data->use_st) {
		cyttsp_xdebug("ST STEP 0 - ST1 ID=%d  ST2 ID=%d\n", \
			curr_st_touch[CYTTSP_ST_FINGER1_IDX], curr_st_touch[CYTTSP_ST_FINGER2_IDX]);
		if (curr_st_touch[CYTTSP_ST_FINGER1_IDX] > CYTTSP_NUM_TRACK_ID) {
			/* reassign finger 1 and 2 positions to new tracks */
			if (curr_touches > 0) {
				/* reassign st finger1 */
				if (ts->platform_data->use_trk_id) {
					id = CYTTSP_MT_TOUCH1_IDX;
					curr_st_touch[CYTTSP_ST_FINGER1_IDX] = curr_mt_touch[id];
				}
				else {
					id = GET_TOUCH1_ID(g_xy_data.touch12_id);
					curr_st_touch[CYTTSP_ST_FINGER1_IDX] = id;
				}
				st_x1 = curr_mt_pos[id][CYTTSP_XPOS];
				st_y1 = curr_mt_pos[id][CYTTSP_YPOS];
				st_z1 = curr_mt_z[id];
				cyttsp_xdebug("ST STEP 1 - ST1 ID=%3d\n", curr_st_touch[CYTTSP_ST_FINGER1_IDX]);
				if (curr_touches > 1) {
					if (curr_st_touch[CYTTSP_ST_FINGER2_IDX] > CYTTSP_NUM_TRACK_ID) {
						/* reassign st finger2 */
						if (curr_touches > 1) {
							if (ts->platform_data->use_trk_id) {
								id = CYTTSP_MT_TOUCH2_IDX;
								curr_st_touch[CYTTSP_ST_FINGER2_IDX] = curr_mt_touch[id];
							}
							else {
								id = GET_TOUCH2_ID(g_xy_data.touch12_id);
								curr_st_touch[CYTTSP_ST_FINGER2_IDX] = id;
							}
							st_x2 = curr_mt_pos[id][CYTTSP_XPOS];
							st_y2 = curr_mt_pos[id][CYTTSP_YPOS];
							st_z2 = curr_mt_z[id];
							cyttsp_xdebug("ST STEP 2 - ST2 ID=%3d\n", curr_st_touch[CYTTSP_ST_FINGER2_IDX]);
						}
					}
				}
			}
		}
		else if (curr_st_touch[CYTTSP_ST_FINGER2_IDX] > CYTTSP_NUM_TRACK_ID) {
			if (curr_touches > 1) {
				/* reassign st finger2 */
				if (ts->platform_data->use_trk_id) {
					id = CYTTSP_MT_TOUCH2_IDX;
					curr_st_touch[CYTTSP_ST_FINGER2_IDX] = curr_mt_touch[id];	/* reassign st finger2 */
				}
				else {
					id = GET_TOUCH2_ID(g_xy_data.touch12_id);
					curr_st_touch[CYTTSP_ST_FINGER2_IDX] = id;			/* reassign st finger2 */
				}
				st_x2 = curr_mt_pos[id][CYTTSP_XPOS];
				st_y2 = curr_mt_pos[id][CYTTSP_YPOS];
				st_z2 = curr_mt_z[id];
				cyttsp_xdebug("ST STEP 3 - ST2 ID=%3d\n", curr_st_touch[CYTTSP_ST_FINGER2_IDX]);
			}
		}
		/* if the first touch is missing and there is a second touch, 
		 * then set the first touch to second touch and terminate second touch
		 */
		if ((curr_st_touch[CYTTSP_ST_FINGER1_IDX] > CYTTSP_NUM_TRACK_ID) && 
		    (curr_st_touch[CYTTSP_ST_FINGER2_IDX] < CYTTSP_NUM_TRACK_ID)) {
			st_x1 = st_x2;
			st_y1 = st_y2;
			st_z1 = st_z2;
			curr_st_touch[CYTTSP_ST_FINGER1_IDX] = curr_st_touch[CYTTSP_ST_FINGER2_IDX];
			curr_st_touch[CYTTSP_ST_FINGER2_IDX] = CYTTSP_IGNORE_TOUCH;
		}
		/* if the second touch ends up equal to the first touch, then just report a single touch */
		if (curr_st_touch[CYTTSP_ST_FINGER1_IDX] == curr_st_touch[CYTTSP_ST_FINGER2_IDX]) {
			curr_st_touch[CYTTSP_ST_FINGER2_IDX] = CYTTSP_IGNORE_TOUCH;
		}

//		dbg("ID: %d, X: %d, Y: %d\n", curr_st_touch[CYTTSP_ST_FINGER1_IDX], st_x1, st_y1);
			
		/* set Single Touch current event signals */
		if (curr_st_touch[CYTTSP_ST_FINGER1_IDX] < CYTTSP_NUM_TRACK_ID) {
			input_report_abs(ts->input, ABS_X, st_x1);
			input_report_abs(ts->input, ABS_Y, st_y1);
			input_report_abs(ts->input, ABS_PRESSURE, st_z1);
			input_report_key(ts->input, BTN_TOUCH, CYTTSP_TOUCH);
			input_report_abs(ts->input, ABS_TOOL_WIDTH, curr_tool_width);
			cyttsp_debug("ST ->  F1:%3d  X:%3d  Y:%3d  Z:%3d  \n", \
				curr_st_touch[CYTTSP_ST_FINGER1_IDX], st_x1, st_y1, st_z1);
			if (curr_st_touch[CYTTSP_ST_FINGER2_IDX] < CYTTSP_NUM_TRACK_ID) {
				input_report_key(ts->input, BTN_2, CYTTSP_TOUCH);
				input_report_abs(ts->input, ABS_HAT0X, st_x2);
				input_report_abs(ts->input, ABS_HAT0Y, st_y2);
				cyttsp_debug("ST ->  F2:%3d  X:%3d  Y:%3d  Z:%3d  \n", \
					curr_st_touch[CYTTSP_ST_FINGER2_IDX], st_x2, st_y2, st_z2);
			}
			else {
				input_report_key(ts->input, BTN_2, CYTTSP_NOTOUCH);
			}
		}
		else {
			input_report_abs(ts->input, ABS_PRESSURE, CYTTSP_NOTOUCH);
			input_report_key(ts->input, BTN_TOUCH, CYTTSP_NOTOUCH);
			input_report_key(ts->input, BTN_2, CYTTSP_NOTOUCH);
		}
		/* update platform data for the current single touch information */
		ts->prev_st_touch[CYTTSP_ST_FINGER1_IDX] = curr_st_touch[CYTTSP_ST_FINGER1_IDX];
		ts->prev_st_touch[CYTTSP_ST_FINGER2_IDX] = curr_st_touch[CYTTSP_ST_FINGER2_IDX];

	}

	/* handle Multi-touch signals */
	if (ts->platform_data->use_mt) {
		if (ts->platform_data->use_trk_id) {
			/* terminate any previous touch where the track is missing from the current event */
			for (id = 0; id < CYTTSP_NUM_TRACK_ID; id++) 
			{
				if ((ts->active_track[id] != CYTTSP_NOTOUCH) && (curr_track[id] == CYTTSP_NOTOUCH) && 
					(prev_mt_pos[id][CYTTSP_YPOS] <= ts->platform_data->maxy) )
				{				
					input_report_abs(ts->input, ABS_MT_TRACKING_ID, id);
					input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, CYTTSP_NOTOUCH);
					input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, curr_tool_width);
					input_report_abs(ts->input, ABS_MT_POSITION_X, prev_mt_pos[id][CYTTSP_XPOS]);
					input_report_abs(ts->input, ABS_MT_POSITION_Y, prev_mt_pos[id][CYTTSP_YPOS]);				
//					input_report_abs(ts->input, ABS_MT_POSITION_X, curr_mt_pos[id][CYTTSP_XPOS]);
//					input_report_abs(ts->input, ABS_MT_POSITION_Y, curr_mt_pos[id][CYTTSP_YPOS]);
				
					if(Touch_Dbg_Enable)
						printk("[Touch Up] id: %d, width: %d, x: %d, y: %d\n", 
							id, curr_tool_width, prev_mt_pos[id][CYTTSP_XPOS], prev_mt_pos[id][CYTTSP_YPOS]);						

					// If Release Prev key... If not , Touch Key is wrong work
					prev_mt_pos[id][CYTTSP_YPOS] = ts->platform_data->maxy + 1;

					CYTTSP_MT_SYNC(ts->input);
				}
			}
		
			/* set Multi-Touch current event signals */
			for (id = 0; id < CYTTSP_NUM_MT_TOUCH_ID; id++) 
			{
				if (curr_mt_touch[id] < CYTTSP_NUM_TRACK_ID &&	(curr_mt_pos[id][CYTTSP_YPOS] <= ts->platform_data->maxy)
				) 
				{				
					input_report_abs(ts->input, ABS_MT_TRACKING_ID, curr_mt_touch[id]);
					input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, curr_mt_z[id]);
					input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, curr_tool_width);
					input_report_abs(ts->input, ABS_MT_POSITION_X, curr_mt_pos[id][CYTTSP_XPOS]);
					input_report_abs(ts->input, ABS_MT_POSITION_Y, curr_mt_pos[id][CYTTSP_YPOS]);
					CYTTSP_MT_SYNC(ts->input);
					prev_mt_pos[curr_mt_touch[id]][CYTTSP_XPOS] = curr_mt_pos[id][CYTTSP_XPOS];
					prev_mt_pos[curr_mt_touch[id]][CYTTSP_YPOS] = curr_mt_pos[id][CYTTSP_YPOS];

					if(Touch_Dbg_Enable)
						printk("[Touch Down] id: %d, z: %d, width: %d, x: %d, y: %d\n", 
							curr_mt_touch[id], curr_mt_z[id], curr_tool_width, curr_mt_pos[id][CYTTSP_XPOS], curr_mt_pos[id][CYTTSP_YPOS]);						
				}

#ifdef FEATURE_TOUCH_KEY
				if( curr_mt_touch[id] < CYTTSP_NUM_TRACK_ID  && key_status == CYTTSP_NULL_KEY && 
					curr_mt_pos[id][CYTTSP_YPOS] > ts->platform_data->maxy)
				{
					prev_mt_pos[curr_mt_touch[id]][CYTTSP_XPOS] = 0;
					prev_mt_pos[curr_mt_touch[id]][CYTTSP_YPOS] = 0;
				}
#endif
				
/*
				if(curr_mt_pos[id][CYTTSP_YPOS] > ts->platform_data->maxy)
				{
					prev_mt_pos[id][CYTTSP_XPOS] = curr_mt_pos[id][CYTTSP_XPOS];
	//				prev_mt_pos[id][CYTTSP_YPOS] = curr_mt_pos[id][CYTTSP_YPOS];	
					prev_mt_pos[id][CYTTSP_YPOS] = ts->platform_data->maxy;
				}
*/
			}
		}
		else {
			/* set temporary track array elements to voids */
			for (id = 0; id < CYTTSP_NUM_MT_TOUCH_ID; id++) {
				temp_track[id] = CYTTSP_IGNORE_TOUCH;
				send_track[id] = CYTTSP_IGNORE_TOUCH;
			}

			/* get what is currently active */
			for (i = 0, id = 0; id < CYTTSP_NUM_TRACK_ID && i < CYTTSP_NUM_MT_TOUCH_ID; id++) {
				if (curr_track[id] == CYTTSP_TOUCH) {
					temp_track[i] = id;
					i++; /* only increment counter if track found */
				}
			}
			cyttsp_xdebug("T1: t0=%d, t1=%d, t2=%d, t3=%d\n", \
				temp_track[0], temp_track[1], temp_track[2], temp_track[3]);
			cyttsp_xdebug("T1: p0=%d, p1=%d, p2=%d, p3=%d\n", \
				ts->prev_mt_touch[0], ts->prev_mt_touch[1], \
				ts->prev_mt_touch[2], ts->prev_mt_touch[3]);

			/* pack in still active previous touches */
			for (id = 0, prev_touches = 0; id < CYTTSP_NUM_MT_TOUCH_ID; id++) {
				if (temp_track[id] < CYTTSP_NUM_TRACK_ID) {
					if (cyttsp_inlist(ts->prev_mt_touch, temp_track[id], &loc,
						 CYTTSP_NUM_MT_TOUCH_ID)) {
						loc &= CYTTSP_NUM_MT_TOUCH_ID-1;
						send_track[loc] = temp_track[id];
						prev_touches++;
						cyttsp_xdebug("in list s[%d]=%d t[%d]=%d, loc=%d p=%d\n", \
							loc, send_track[loc], \
							id, temp_track[id], loc, prev_touches);
					}
					else {
						cyttsp_xdebug("is not in list s[%d]=%d t[%d]=%d loc=%d\n", \
							id, send_track[id], id, temp_track[id], loc);
					}
				}
			}
			cyttsp_xdebug("S1: s0=%d, s1=%d, s2=%d, s3=%d p=%d\n", \
				send_track[0], send_track[1], send_track[2], send_track[3], \
				prev_touches);

			/* pack in new touches */
			for (id = 0; id < CYTTSP_NUM_MT_TOUCH_ID; id++) {
				if (temp_track[id] < CYTTSP_NUM_TRACK_ID) {
					if (!cyttsp_inlist(send_track, temp_track[id], &loc, CYTTSP_NUM_MT_TOUCH_ID)) {
						cyttsp_xdebug("not in list t[%d]=%d, loc=%d\n", \
							id, temp_track[id], loc);
						if (cyttsp_next_avail_inlist(send_track, &loc, CYTTSP_NUM_MT_TOUCH_ID)) {
							loc &= CYTTSP_NUM_MT_TOUCH_ID-1;
							send_track[loc] = temp_track[id];
							cyttsp_xdebug("put in list s[%d]=%d t[%d]=%d\n", 
								loc, send_track[loc], id, temp_track[id]);
						}
					}
					else {
						cyttsp_xdebug("is in list s[%d]=%d t[%d]=%d loc=%d\n", \
							id, send_track[id], id, temp_track[id], loc);
					}
				}
			}
			cyttsp_xdebug("S2: s0=%d, s1=%d, s2=%d, s3=%d\n", \
				send_track[0], send_track[1], send_track[2], send_track[3]);

			/* synchronize motion event signals for each current touch */
			for (id = 0; id < CYTTSP_NUM_MT_TOUCH_ID; id++) {
				/* z will either be 0 (NOTOUCH) or some pressure (TOUCH) */
				cyttsp_xdebug("MT0 prev[%d]=%d temp[%d]=%d send[%d]=%d\n", \
					id, ts->prev_mt_touch[id], \
					id, temp_track[id], \
					id, send_track[id]);
				if (send_track[id] < CYTTSP_NUM_TRACK_ID) {
					input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, curr_mt_z[send_track[id]]);	
					input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, curr_tool_width);
					input_report_abs(ts->input, ABS_MT_POSITION_X, curr_mt_pos[send_track[id]][CYTTSP_XPOS]);
					input_report_abs(ts->input, ABS_MT_POSITION_Y, curr_mt_pos[send_track[id]][CYTTSP_YPOS]);
					CYTTSP_MT_SYNC(ts->input);
					cyttsp_debug("MT1->TID:%2d X:%3d Y:%3d Z:%3d act touch-sent\n", \
						send_track[id], curr_mt_pos[send_track[id]][CYTTSP_XPOS], \
						curr_mt_pos[send_track[id]][CYTTSP_YPOS],  curr_mt_z[send_track[id]]);
				}
				else if (ts->prev_mt_touch[id] < CYTTSP_NUM_TRACK_ID) {
					input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, CYTTSP_NOTOUCH);	/* void out this touch */
					input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, curr_tool_width);
					input_report_abs(ts->input, ABS_MT_POSITION_X, ts->prev_mt_pos[ts->prev_mt_touch[id]][CYTTSP_XPOS]);
					input_report_abs(ts->input, ABS_MT_POSITION_Y, ts->prev_mt_pos[ts->prev_mt_touch[id]][CYTTSP_YPOS]);
					CYTTSP_MT_SYNC(ts->input);
					cyttsp_debug("MT2->TID:%2d X:%3d Y:%3d Z:%3d lift off-sent\n", \
						ts->prev_mt_touch[id], \
						ts->prev_mt_pos[ts->prev_mt_touch[id]][CYTTSP_XPOS], \
						ts->prev_mt_pos[ts->prev_mt_touch[id]][CYTTSP_YPOS], \
						CYTTSP_NOTOUCH);
				}
				else {
					/* do not stuff any signals for this previously and currently void touches */
					cyttsp_xdebug("MT3-> send[%d]=%d - No touch - NOT sent\n", \
							id, send_track[id]);
				}
			}

			/* save current posted tracks to previous track memory */
			for (id = 0; id < CYTTSP_NUM_MT_TOUCH_ID; id++) {
				ts->prev_mt_touch[id] = send_track[id];
				ts->prev_mt_pos[send_track[id]][CYTTSP_XPOS] = curr_mt_pos[send_track[id]][CYTTSP_XPOS];
				ts->prev_mt_pos[send_track[id]][CYTTSP_YPOS] = curr_mt_pos[send_track[id]][CYTTSP_YPOS];
				cyttsp_xdebug("MT4->TID:%2d X:%3d Y:%3d Z:%3d save for previous\n", \
					send_track[id], \
					ts->prev_mt_pos[send_track[id]][CYTTSP_XPOS], \
					ts->prev_mt_pos[send_track[id]][CYTTSP_YPOS], \
					CYTTSP_NOTOUCH);
			}
		}
	}

#ifdef FEATURE_SKY_TOUCH_DELTA_DEBUG
exit_touch_event:
#endif
	/* handle gestures */
	if (ts->platform_data->use_gestures) {
		if (g_xy_data.gest_id) {
			input_report_key(ts->input, BTN_3, CYTTSP_TOUCH);
			input_report_abs(ts->input, ABS_HAT1X, g_xy_data.gest_id);
			input_report_abs(ts->input, ABS_HAT2Y, g_xy_data.gest_cnt);
		}
	}

	/* signal the view motion event */
	input_sync(ts->input);

	/* update platform data for the current multi-touch information */
	for (id = 0; id < CYTTSP_NUM_TRACK_ID; id++) {
		ts->active_track[id] = curr_track[id];
	}

exit_xy_worker:
	if(cyttsp_disable_touch) {
		cyttsp_debug("Not enabling touch\n");
	}
	else {
		if(ts->client->irq == 0) {
			/* restart event timer */
			mod_timer(&ts->timer, jiffies + TOUCHSCREEN_TIMEOUT);
		}
		else {
			/* re-enable the interrupt after processing */
			enable_irq(ts->client->irq);
		}
	}
	return;
}

static int cyttsp_inlist(u16 prev_track[], u8 curr_track_id, u8 *prev_loc, u8 num_touches)
{
	u8 id =0;

	*prev_loc = CYTTSP_IGNORE_TOUCH;

		cyttsp_xdebug("IN p[%d]=%d c=%d n=%d loc=%d\n", \
			id, prev_track[id], curr_track_id, num_touches, *prev_loc);
	for (id = 0, *prev_loc = CYTTSP_IGNORE_TOUCH;
		(id < num_touches); id++) {
		cyttsp_xdebug("p[%d]=%d c=%d n=%d loc=%d\n", \
			id, prev_track[id], curr_track_id, num_touches, *prev_loc);
		if (prev_track[id] == curr_track_id) {
			*prev_loc = id;
			break;
		}
	}
		cyttsp_xdebug("OUT p[%d]=%d c=%d n=%d loc=%d\n", \
			id, prev_track[id], curr_track_id, num_touches, *prev_loc);

	return ((*prev_loc < CYTTSP_NUM_TRACK_ID) ? true : false);
}

static int cyttsp_next_avail_inlist(u16 curr_track[], u8 *new_loc, u8 num_touches)
{
	u8 id;

	for (id = 0, *new_loc = CYTTSP_IGNORE_TOUCH;
		(id < num_touches); id++) {
		if (curr_track[id] > CYTTSP_NUM_TRACK_ID) {
			*new_loc = id;
			break;
		}
	}

	return ((*new_loc < CYTTSP_NUM_TRACK_ID) ? true : false);
}

/* Timer function used as dummy interrupt driver */
static void cyttsp_timer(unsigned long handle)
{
	struct cyttsp *ts = (struct cyttsp *) handle;

	cyttsp_xdebug("TTSP Device timer event\n");
 #ifdef FEATURE_CYTTSP_HEARTBEAT
	/* schedule motion signal handling */
	if(start_heartbeat_timer)
	{
		schedule_work(&ts->work2);
		mod_timer(&ts->timer, jiffies + CYTTSP_HEARTBEAT_TIME * HZ);
	}
			
#else
	/* schedule motion signal handling */
	schedule_work(&ts->work);
#endif
	return;
}



/* ************************************************************************
 * ISR function. This function is general, initialized in drivers init
 * function
 * ************************************************************************ */
static irqreturn_t cyttsp_irq(int irq, void *handle)
{
	struct cyttsp *ts = (struct cyttsp *) handle;

	cyttsp_xdebug("%s: Got IRQ\n", CYTTSP_I2C_NAME);

	if(Touch_Status >= TOUCH_POWEROFF)
		return IRQ_HANDLED;
	
	/* disable further interrupts until this interrupt is processed */
	disable_irq_nosync(ts->client->irq);

	/* schedule motion signal handling */
	schedule_work(&ts->work);
	return IRQ_HANDLED;
}

/* ************************************************************************
 * Probe initialization functions
 * ************************************************************************ */
static int cyttsp_putbl(struct cyttsp *ts, int show, int show_status, int show_version, int show_cid)
{
	int retval = CYTTSP_OPERATIONAL;

	int num_bytes = (show_status * 3) + (show_version * 6) + (show_cid * 3);

	if (show_cid) {
		num_bytes = sizeof(struct cyttsp_bootloader_data_t);
	}
	else if (show_version) {
		num_bytes = sizeof(struct cyttsp_bootloader_data_t) - 3;
	}
	else {
		num_bytes = sizeof(struct cyttsp_bootloader_data_t) - 9;
	}

	if (show) {
		retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE,
			num_bytes, (u8 *)&g_bl_data);
	
		if (show_status) {
			cyttsp_debug("BL%d: f=%02X s=%02X err=%02X bl=%02X%02X bld=%02X%02X\n", \
				show, \
				g_bl_data.bl_file, g_bl_data.bl_status, g_bl_data.bl_error, \
				g_bl_data.blver_hi, g_bl_data.blver_lo, \
				g_bl_data.bld_blver_hi, g_bl_data.bld_blver_lo);
		}
		if (show_version) {
			cyttsp_debug("BL%d: ttspver=0x%02X%02X appid=0x%02X%02X appver=0x%02X%02X\n", \
				show, \
				g_bl_data.ttspver_hi, g_bl_data.ttspver_lo, \
				g_bl_data.appid_hi, g_bl_data.appid_lo, \
				g_bl_data.appver_hi, g_bl_data.appver_lo);
		}
		if (show_cid) {
			cyttsp_debug("BL%d: cid=0x%02X%02X%02X\n", \
				show, \
				g_bl_data.cid_0, g_bl_data.cid_1, g_bl_data.cid_2);
		}
		mdelay(CYTTSP_DELAY_DFLT);
	}

	return retval;
}

#ifdef CYTTSP_INCLUDE_LOAD_FILE
#define CYTTSP_MAX_I2C_LEN	256
#define CYTTSP_MAX_TRY		10
#define CYTTSP_BL_PAGE_SIZE	16
#define CYTTSP_BL_NUM_PAGES	5
static int cyttsp_i2c_write_block_data(struct i2c_client *client, u8 command,
			       u8 length, const u8 *values)
{
	int retval = CYTTSP_OPERATIONAL;

	u8 dataray[CYTTSP_MAX_I2C_LEN];
	u8 try;
	dataray[0] = command;
	if (length) {
		memcpy(&dataray[1], values, length);
	}

	try = CYTTSP_MAX_TRY;
	do {
		retval = i2c_master_send(client, dataray, length+1);
		mdelay(CYTTSP_DELAY_DFLT*2);
	}
	while ((retval != length+1) && try--);

	return retval;
}

static int cyttsp_i2c_write_block_data_chunks(struct cyttsp *ts, u8 command,
			       u8 length, const u8 *values)
{
	int retval = CYTTSP_OPERATIONAL;
	int block = 1;

	u8 dataray[CYTTSP_MAX_I2C_LEN];

	/* first page already includes the bl page offset */
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
		CYTTSP_BL_PAGE_SIZE+1, values);
	mdelay(10);
	values += CYTTSP_BL_PAGE_SIZE+1;
	length -= CYTTSP_BL_PAGE_SIZE+1;

	/* rem blocks require bl page offset stuffing */
	while (length && (block < CYTTSP_BL_NUM_PAGES) && !(retval < CYTTSP_OPERATIONAL)) {
		dataray[0] = CYTTSP_BL_PAGE_SIZE*block;
		memcpy(&dataray[1], values, 
			length >= CYTTSP_BL_PAGE_SIZE ? CYTTSP_BL_PAGE_SIZE : length);
		retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
			length >= CYTTSP_BL_PAGE_SIZE ? CYTTSP_BL_PAGE_SIZE+1 : length+1, dataray);
		mdelay(10);
		values += CYTTSP_BL_PAGE_SIZE;
		length = length >= CYTTSP_BL_PAGE_SIZE ? length - CYTTSP_BL_PAGE_SIZE : 0;
		block++;
	}

	return retval;
}

static int cyttsp_bootload_app(struct cyttsp *ts)
{
	int retval = CYTTSP_OPERATIONAL;
	int i, tries;
	u8 host_reg;

	cyttsp_debug("load new firmware \n");
	/* reset TTSP Device back to bootloader mode */
	host_reg = CYTTSP_SOFT_RESET_MODE;
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
		sizeof(host_reg), &host_reg);
	/* wait for TTSP Device to complete reset back to bootloader */
//	mdelay(CYTTSP_DELAY_DFLT);
	mdelay(1000);
	cyttsp_putbl(ts,3, true, true, true);
	cyttsp_debug("load file -- tts_ver=0x%02X%02X  app_id=0x%02X%02X  app_ver=0x%02X%02X\n", \
		cyttsp_fw_tts_verh, cyttsp_fw_tts_verl, \
		cyttsp_fw_app_idh, cyttsp_fw_app_idl, \
		cyttsp_fw_app_verh, cyttsp_fw_app_verl);

	/* download new TTSP Application to the Bootloader
	 *
	 */
	if (!(retval < CYTTSP_OPERATIONAL)) {
		i = 0;
		/* send bootload initiation command */
		if (cyttsp_fw[i].Command == CYTTSP_BL_INIT_LOAD) {
			g_bl_data.bl_file = 0;
			g_bl_data.bl_status = 0;
			g_bl_data.bl_error = 0;
			retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
				cyttsp_fw[i].Length, cyttsp_fw[i].Block);
			/* delay to allow bootloader to get ready for block writes */
			i++;
			tries = 0;
			cyttsp_debug("wait init f=%02X, s=%02X, e=%02X t=%d\n",g_bl_data.bl_file,
				g_bl_data.bl_status, g_bl_data.bl_error, tries);
			do {
				mdelay(1000);
				cyttsp_putbl(ts,4, true, false, false);
			}
			while (g_bl_data.bl_status != 0x10 && 
				g_bl_data.bl_status != 0x11 && 
				tries++ < 10);
			/* send bootload firmware load blocks - 
			 * kernel limits transfers to I2C_SMBUS_BLOCK_MAX(32) bytes
			 */
			if (!(retval < CYTTSP_OPERATIONAL)) {
				while (cyttsp_fw[i].Command == CYTTSP_BL_WRITE_BLK) {
					retval = cyttsp_i2c_write_block_data_chunks(ts,
						CYTTSP_REG_BASE, 
						cyttsp_fw[i].Length, cyttsp_fw[i].Block);
//					if (cyttsp_fw[i].Address & 0x01) {
//						mdelay(CYTTSP_DELAY_DNLOAD);
//					}
//					else {
//						mdelay(CYTTSP_DELAY_DNLOAD);
//					}
					/* bootloader requires delay after odd block addresses */
					mdelay(100);
					cyttsp_debug("BL DNLD Rec=% 3d Len=% 3d Addr=%04X\n", 
						cyttsp_fw[i].Record, cyttsp_fw[i].Length, 
						cyttsp_fw[i].Address);
					i++;
					if (retval < CYTTSP_OPERATIONAL) {
						cyttsp_debug("BL fail Rec=%3d retval=%d\n",cyttsp_fw[i-1].Record, retval);
						break;
					}
					else {
						/* reset TTSP I2C counter */
						retval = cyttsp_i2c_write_block_data(ts->client,
							CYTTSP_REG_BASE, 
							0, NULL);
						mdelay(10);
						/* set arg2 to non-0 to activate */
						cyttsp_putbl(ts,5, true, false, false);
					}
				}
				if (!(retval < CYTTSP_OPERATIONAL)) {
					while (i < cyttsp_fw_records) {
						retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
							cyttsp_fw[i].Length, cyttsp_fw[i].Block);
						i++;
						tries = 0;
						cyttsp_debug("wait init f=%02X, s=%02X, e=%02X t=%d\n",g_bl_data.bl_file,
							g_bl_data.bl_status, g_bl_data.bl_error, tries);
						do {
							mdelay(1000);
							cyttsp_putbl(ts,6, true, false, false);
						}
						while (g_bl_data.bl_status != 0x10 && 
							g_bl_data.bl_status != 0x11 && 
							tries++ < 10);
						cyttsp_putbl(ts,7, true, false, false);
						if (retval < CYTTSP_OPERATIONAL) {
							break;
						}
					}
				}
			}
		}
	}

	/* Do we need to reset TTSP Device back to bootloader mode?? */
	/*
	*/
	host_reg = CYTTSP_SOFT_RESET_MODE;
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
		sizeof(host_reg), &host_reg);
	/* wait for TTSP Device to complete reset back to bootloader */
	/*
	*/
	mdelay(1000);

	/* set arg2 to non-0 to activate */
	retval = cyttsp_putbl(ts, 8, true, true, true);

	return retval;
}
#else 
static int cyttsp_bootload_app(struct cyttsp *ts)
{
	cyttsp_debug("no-load new firmware \n");
	return CYTTSP_OPERATIONAL;
}
#endif /* CYTTSP_INCLUDE_LOAD_FILE */


static int cyttsp_power_on(struct cyttsp *ts)
{
	int retval = CYTTSP_OPERATIONAL;
	u8 host_reg;
	int tries;
	static u8 bl_cmd[] = {
		CYTTSP_BL_FILE0, CYTTSP_BL_CMD, CYTTSP_BL_EXIT,
		CYTTSP_BL_KEY0, CYTTSP_BL_KEY1, CYTTSP_BL_KEY2,
		CYTTSP_BL_KEY3, CYTTSP_BL_KEY4, CYTTSP_BL_KEY5,
		CYTTSP_BL_KEY6, CYTTSP_BL_KEY7};

	cyttsp_debug("Power up \n");

	/* check if the TTSP device has a bootloader installed */
	host_reg = CYTTSP_SOFT_RESET_MODE;
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
		sizeof(host_reg), &host_reg);

	tries = 0;
	do {
		mdelay(1000);

		/* set arg2 to non-0 to activate */
		retval = cyttsp_putbl(ts, 1, true, true, true);
	
		cyttsp_info("BL%d: f=%02X s=%02X err=%02X bl=%02X%02X bld=%02X%02X R=%d\n", \
			101, \
			g_bl_data.bl_file, g_bl_data.bl_status, g_bl_data.bl_error, \
			g_bl_data.blver_hi, g_bl_data.blver_lo, \
			g_bl_data.bld_blver_hi, g_bl_data.bld_blver_lo,
			retval);
		cyttsp_info("BL%d: tver=%02X%02X a_id=%02X%02X aver=%02X%02X\n", \
			102, \
			g_bl_data.ttspver_hi, g_bl_data.ttspver_lo, \
			g_bl_data.appid_hi, g_bl_data.appid_lo, \
			g_bl_data.appver_hi, g_bl_data.appver_lo);
		cyttsp_info("BL%d: c_id=%02X%02X%02X\n", \
			103, \
			g_bl_data.cid_0, g_bl_data.cid_1, g_bl_data.cid_2);
	}
	while (!(retval < CYTTSP_OPERATIONAL) &&
		!GET_BOOTLOADERMODE(g_bl_data.bl_status) && 
		!(g_bl_data.bl_file == CYTTSP_OPERATE_MODE + CYTTSP_LOW_POWER_MODE) &&
		tries++ < 10);

	/* is bootloader missing? */
	if (!(retval < CYTTSP_OPERATIONAL)) {
		cyttsp_xdebug("Retval=%d  Check if bootloader is missing...\n", retval);
		if (!GET_BOOTLOADERMODE(g_bl_data.bl_status)) {
			/* skip all bootloader and sys info and go straight to operational mode */
			if (!(retval < CYTTSP_OPERATIONAL)) {
				cyttsp_xdebug("Bootloader is missing (retval = %d)\n", retval);
				host_reg = CYTTSP_OPERATE_MODE/* + CYTTSP_LOW_POWER_MODE*/;
				retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
					sizeof(host_reg), &host_reg);
				/* wait for TTSP Device to complete switch to Operational mode */
				mdelay(1000);
				goto bypass;
			}
		}
	}


	/* take TTSP out of bootloader mode; go to TrueTouch operational mode */
	if (!(retval < CYTTSP_OPERATIONAL)) {
		cyttsp_xdebug1("exit bootloader; go operational\n");
		retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
			sizeof(bl_cmd), bl_cmd);
		tries = 0;
		do {
			mdelay(1000);
			cyttsp_putbl(ts,4, true, false, false);
			cyttsp_info("BL%d: f=%02X s=%02X err=%02X bl=%02X%02X bld=%02X%02X\n", \
				104, \
				g_bl_data.bl_file, g_bl_data.bl_status, g_bl_data.bl_error, \
				g_bl_data.blver_hi, g_bl_data.blver_lo, \
				g_bl_data.bld_blver_hi, g_bl_data.bld_blver_lo);
		}
		while (GET_BOOTLOADERMODE(g_bl_data.bl_status) && 
			tries++ < 10);
	}



	if (!(retval < CYTTSP_OPERATIONAL) &&
		cyttsp_app_load()) {
		mdelay(1000);
		if (CYTTSP_DIFF(g_bl_data.ttspver_hi, cyttsp_tts_verh())  ||
			CYTTSP_DIFF(g_bl_data.ttspver_lo, cyttsp_tts_verl())  ||
			CYTTSP_DIFF(g_bl_data.appid_hi, cyttsp_app_idh())  ||
			CYTTSP_DIFF(g_bl_data.appid_lo, cyttsp_app_idl())  ||
			CYTTSP_DIFF(g_bl_data.appver_hi, cyttsp_app_verh())  ||
			CYTTSP_DIFF(g_bl_data.appver_lo, cyttsp_app_verl())  ||
			CYTTSP_DIFF(g_bl_data.cid_0, cyttsp_cid_0())  ||
			CYTTSP_DIFF(g_bl_data.cid_1, cyttsp_cid_1())  ||
			CYTTSP_DIFF(g_bl_data.cid_2, cyttsp_cid_2())  ||
			cyttsp_force_fw_load()) {
			cyttsp_debug("blttsp=0x%02X%02X flttsp=0x%02X%02X force=%d\n", \
				g_bl_data.ttspver_hi, g_bl_data.ttspver_lo, \
				cyttsp_tts_verh(), cyttsp_tts_verl(), cyttsp_force_fw_load());
			cyttsp_debug("blappid=0x%02X%02X flappid=0x%02X%02X\n", \
				g_bl_data.appid_hi, g_bl_data.appid_lo, \
				cyttsp_app_idh(), cyttsp_app_idl());
			cyttsp_debug("blappver=0x%02X%02X flappver=0x%02X%02X\n", \
				g_bl_data.appver_hi, g_bl_data.appver_lo, \
				cyttsp_app_verh(), cyttsp_app_verl());
			cyttsp_debug("blcid=0x%02X%02X%02X flcid=0x%02X%02X%02X\n", \
				g_bl_data.cid_0, g_bl_data.cid_1, g_bl_data.cid_2, \
				cyttsp_cid_0(), cyttsp_cid_1(), cyttsp_cid_2());
			/* enter bootloader to load new app into TTSP Device */
			retval = cyttsp_bootload_app(ts);
			/* take TTSP device out of bootloader mode; switch back to TrueTouch operational mode */
			if (!(retval < CYTTSP_OPERATIONAL)) {
				retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
					sizeof(bl_cmd), bl_cmd);
				/* wait for TTSP Device to complete switch to Operational mode */
				mdelay(1000);
			}
		}
	}

bypass:
	/* switch to System Information mode to read versions and set interval registers */
	if (!(retval < CYTTSP_OPERATIONAL)) {
		cyttsp_debug("switch to sysinfo mode \n");
		host_reg = CYTTSP_SYSINFO_MODE;
		retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
			sizeof(host_reg), &host_reg);
		/* wait for TTSP Device to complete switch to SysInfo mode */
		mdelay(1000);
		if (!(retval < CYTTSP_OPERATIONAL)) {
			retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
				sizeof(struct cyttsp_sysinfo_data_t), (u8 *)&g_sysinfo_data);
			cyttsp_debug("SI2: hst_mode=0x%02X mfg_cmd=0x%02X mfg_stat=0x%02X\n", \
				g_sysinfo_data.hst_mode, g_sysinfo_data.mfg_cmd, \
				g_sysinfo_data.mfg_stat);
			cyttsp_debug("SI2: bl_ver=0x%02X%02X\n", \
				g_sysinfo_data.bl_verh, g_sysinfo_data.bl_verl);
			cyttsp_debug("SI2: sysinfo act_int=0x%02X tch_tmout=0x%02X lp_int=0x%02X\n", \
				g_sysinfo_data.act_intrvl, g_sysinfo_data.tch_tmout, \
				g_sysinfo_data.lp_intrvl);
			cyttsp_info("SI%d: tver=%02X%02X a_id=%02X%02X aver=%02X%02X\n", \
				102, \
				g_sysinfo_data.tts_verh, g_sysinfo_data.tts_verl, \
				g_sysinfo_data.app_idh, g_sysinfo_data.app_idl, \
				g_sysinfo_data.app_verh, g_sysinfo_data.app_verl);
			cyttsp_info("SI%d: c_id=%02X%02X%02X\n", \
				103, \
				g_sysinfo_data.cid[0], g_sysinfo_data.cid[1], g_sysinfo_data.cid[2]);
			if (!(retval < CYTTSP_OPERATIONAL) &&
				(CYTTSP_DIFF(ts->platform_data->act_intrvl, CYTTSP_ACT_INTRVL_DFLT)  ||
				CYTTSP_DIFF(ts->platform_data->tch_tmout, CYTTSP_TCH_TMOUT_DFLT) ||
				CYTTSP_DIFF(ts->platform_data->lp_intrvl, CYTTSP_LP_INTRVL_DFLT))) {
				if (!(retval < CYTTSP_OPERATIONAL)) {
					u8 intrvl_ray[sizeof(ts->platform_data->act_intrvl) + 
						sizeof(ts->platform_data->tch_tmout) + 
						sizeof(ts->platform_data->lp_intrvl)];
					u8 i = 0;

					intrvl_ray[i++] = ts->platform_data->act_intrvl;
					intrvl_ray[i++] = ts->platform_data->tch_tmout;
					intrvl_ray[i++] = ts->platform_data->lp_intrvl;

					cyttsp_debug("SI2: platinfo act_intrvl=0x%02X tch_tmout=0x%02X lp_intrvl=0x%02X\n", \
						ts->platform_data->act_intrvl, ts->platform_data->tch_tmout, \
						ts->platform_data->lp_intrvl);
					// set intrvl registers
					retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_ACT_INTRVL, 
						sizeof(intrvl_ray), intrvl_ray);
					mdelay(CYTTSP_DELAY_SYSINFO);
				}
			}
		}
		/* switch back to Operational mode */
		cyttsp_debug("switch back to operational mode \n");
		if (!(retval < CYTTSP_OPERATIONAL)) {
			host_reg = CYTTSP_OPERATE_MODE/* + CYTTSP_LOW_POWER_MODE*/;
			retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, 
				sizeof(host_reg), &host_reg);
			/* wait for TTSP Device to complete switch to Operational mode */
			mdelay(1000);
		}
	}
	/* init gesture setup; 
	 * this is required even if not using gestures
	 * in order to set the active distance */
	if (!(retval < CYTTSP_OPERATIONAL)) {
		u8 gesture_setup;
		cyttsp_debug("init gesture setup \n");
		gesture_setup = ts->platform_data->gest_set;
		retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_GEST_SET, 
			sizeof(gesture_setup), &gesture_setup);
		mdelay(CYTTSP_DELAY_DFLT);
	}

	if (!(retval < CYTTSP_OPERATIONAL)) {
		ts->platform_data->power_state = CYTTSP_ACTIVE_STATE;
	}
	else {
		ts->platform_data->power_state = CYTTSP_IDLE_STATE;
	}
	cyttsp_debug("Retval=%d Power state is %s\n", retval, (ts->platform_data->power_state == CYTTSP_ACTIVE_STATE) ? "ACTIVE" : "IDLE");

	return retval;
}

/* cyttsp_initialize: Driver Initialization. This function takes
 * care of the following tasks:
 * 1. Create and register an input device with input layer
 * 2. Take CYTTSP device out of bootloader mode; go operational
 * 3. Start any timers/Work queues.  */
static int cyttsp_initialize(struct i2c_client *client, struct cyttsp *ts)
{
	struct input_dev *input_device;
	int error = 0;
	int retval = CYTTSP_OPERATIONAL;
	u8 id;
	
#ifdef FEATURE_SKY_PROCESS_CMD_KEY	
	cyttsp_data = ts;
#endif	

	/* Create the input device and register it. */
	input_device = input_allocate_device();
	if (!input_device) {
		error = -ENOMEM;
		cyttsp_xdebug1("err input allocate device\n");
		goto error_free_device;
	}

	if (!client) {
		error = ~ENODEV;
		cyttsp_xdebug1("err client is Null\n");
		goto error_free_device;
	}

	if (!ts) {
		error = ~ENODEV;
		cyttsp_xdebug1("err context is Null\n");
		goto error_free_device;
	}

	ts->input = input_device;
	input_device->name = CYTTSP_I2C_NAME;
	input_device->phys = ts->phys;
	input_device->dev.parent = &client->dev;

	/* init the touch structures */
	ts->num_prev_st_touch = CYTTSP_NOTOUCH;
	for (id = 0; id < CYTTSP_NUM_TRACK_ID; id++) {
		ts->active_track[id] = CYTTSP_NOTOUCH;
	}
	for (id = 0; id < CYTTSP_NUM_MT_TOUCH_ID; id++) {
		ts->prev_mt_touch[id] = CYTTSP_IGNORE_TOUCH;
		ts->prev_mt_pos[id][CYTTSP_XPOS] = 0;
		ts->prev_mt_pos[id][CYTTSP_YPOS] = 0;
	}
	for (id = 0; id < CYTTSP_NUM_ST_TOUCH_ID; id++) {
		ts->prev_st_touch[id] = CYTTSP_IGNORE_TOUCH;
	}

	set_bit(EV_SYN, input_device->evbit);
	set_bit(EV_KEY, input_device->evbit);
	set_bit(EV_ABS, input_device->evbit);
	set_bit(BTN_TOUCH, input_device->keybit);
	set_bit(BTN_2, input_device->keybit);

#ifdef FEATURE_SKY_PROCESS_CMD_KEY
	// +++ FEATURE_P_VZW_PS_STABILITY_AT_CMD
	set_bit(KEY_MENU, input_device->keybit);
	set_bit(KEY_BACK, input_device->keybit);
	set_bit(KEY_POWER, input_device->keybit);
	set_bit(KEY_HOME, input_device->keybit);
	// --- 
    set_bit(KEY_SEARCH, input_device->keybit);

    set_bit(KEY_0, input_device->keybit);
    set_bit(KEY_1, input_device->keybit);
    set_bit(KEY_2, input_device->keybit);
    set_bit(KEY_3, input_device->keybit);
    set_bit(KEY_4, input_device->keybit);
    set_bit(KEY_5, input_device->keybit);
    set_bit(KEY_6, input_device->keybit);
    set_bit(KEY_7, input_device->keybit);
    set_bit(KEY_8, input_device->keybit);
    set_bit(KEY_9, input_device->keybit);
    set_bit(0xe3, input_device->keybit); /* '*' */
    set_bit(0xe4, input_device->keybit); /* '#' */

    set_bit(KEY_LEFTSHIFT, input_device->keybit);
    set_bit(KEY_RIGHTSHIFT, input_device->keybit);


    set_bit(KEY_LEFT, input_device->keybit);
    set_bit(KEY_RIGHT, input_device->keybit);
    set_bit(KEY_UP, input_device->keybit);
    set_bit(KEY_DOWN, input_device->keybit);
    set_bit(KEY_ENTER, input_device->keybit);

    set_bit(KEY_SEND, input_device->keybit);
    set_bit(KEY_END, input_device->keybit);

    set_bit(KEY_VOLUMEUP, input_device->keybit);
    set_bit(KEY_VOLUMEDOWN, input_device->keybit);

    set_bit(KEY_CLEAR, input_device->keybit);

	set_bit(KEY_CAMERA, input_device->keybit);
	set_bit(KEY_DELETE, input_device->keybit);
	set_bit(KEY_WWW, input_device->keybit);
	
#endif // FEATURE_SKY_PROCESS_CMD_KEY

	
	if (ts->platform_data->use_gestures) {
		set_bit(BTN_3, input_device->keybit);
	}

	input_set_abs_params(input_device, ABS_X, 0, ts->platform_data->maxx, 0, 0);
	input_set_abs_params(input_device, ABS_Y, 0, ts->platform_data->maxy, 0, 0);
	input_set_abs_params(input_device, ABS_TOOL_WIDTH, 0, CYTTSP_LARGE_TOOL_WIDTH, 0 ,0);
	input_set_abs_params(input_device, ABS_PRESSURE, 0, CYTTSP_MAXZ, 0, 0);
	input_set_abs_params(input_device, ABS_HAT0X, 0, ts->platform_data->maxx, 0, 0);
	input_set_abs_params(input_device, ABS_HAT0Y, 0, ts->platform_data->maxy, 0, 0);
	if (ts->platform_data->use_gestures) {
		input_set_abs_params(input_device, ABS_HAT1X, 0, CYTTSP_MAXZ, 0, 0);
		input_set_abs_params(input_device, ABS_HAT1Y, 0, CYTTSP_MAXZ, 0, 0);
	}
	if (ts->platform_data->use_mt) {
		input_set_abs_params(input_device, ABS_MT_POSITION_X, 0, ts->platform_data->maxx, 0, 0);
		input_set_abs_params(input_device, ABS_MT_POSITION_Y, 0, ts->platform_data->maxy, 0, 0);
		input_set_abs_params(input_device, ABS_MT_TOUCH_MAJOR, 0, CYTTSP_MAXZ, 0, 0);
		input_set_abs_params(input_device, ABS_MT_WIDTH_MAJOR, 0, CYTTSP_LARGE_TOOL_WIDTH, 0, 0);
		if (ts->platform_data->use_trk_id) {
			input_set_abs_params(input_device, ABS_MT_TRACKING_ID, 0, CYTTSP_NUM_TRACK_ID, 0, 0);
		}
	}
	// +++ FEATURE_P_VZW_PS_STABILITY_AT_CMD
	input_set_abs_params(input_device, ABS_MT_TOOL_TYPE, 0, 1, 0, 0);
	// ---
	cyttsp_info("%s: Register input device\n", CYTTSP_I2C_NAME);

	error = input_register_device(input_device);
	if (error) {
		cyttsp_alert("%s: Failed to register input device\n", CYTTSP_I2C_NAME);
		retval = error;
		goto error_free_device;
	}

	/* Prepare our worker structure prior to setting up the timer/ISR */
	INIT_WORK(&ts->work,cyttsp_xy_worker);
#ifdef FEATURE_CYTTSP_HEARTBEAT
	INIT_WORK(&ts->work2,cyttsp_check_heartbeat);
#endif
#ifdef FEATURE_CYTTSP_FIRMWAREUPGRADE
	INIT_WORK(&ts->work3,check_firmware_update);
#endif
	/* Power on the chip and make sure that I/Os are set as specified
	 * in the platform 
	 */
#if 0 //Cypress ��ü ��û����.
	retval = cyttsp_power_on(ts);
	if (retval < 0) {
		goto error_free_device;
	}
#endif
	/* Timer or Interrupt setup */
	if(ts->client->irq == 0) {
		cyttsp_info("Setting up timer\n");
		setup_timer(&ts->timer, cyttsp_timer, (unsigned long) ts);
		mod_timer(&ts->timer, jiffies + TOUCHSCREEN_TIMEOUT);
	}
	else {
#ifdef FEATURE_CYTTSP_HEARTBEAT
		start_heartbeat_timer = true;
		setup_timer(&ts->timer, cyttsp_timer, (unsigned long) ts);
		mod_timer(&ts->timer, jiffies + CYTTSP_HEARTBEAT_TIME * 20 * HZ); //ó�� ������ 60�� �ڿ� Ÿ�̸� ���� �� ���ýð��� ������ ���� �ʱ� ����.
#endif
		cyttsp_info("Setting up interrupt\n");
		/* request_irq() will also call enable_irq() */
		error = request_irq (client->irq,cyttsp_irq,IRQF_TRIGGER_FALLING,
			client->dev.driver->name,ts);
		if (error) {
			cyttsp_alert("error: could not request irq\n");
			retval = error;
			goto error_free_irq;
		}
	}

	atomic_set(&ts->irq_enabled, 1);
	retval = device_create_file(&ts->client->dev, &dev_attr_irq_enable);
	if (retval < CYTTSP_OPERATIONAL) {
		cyttsp_alert("File device creation failed: %d\n", retval);
		retval = -ENODEV;
		goto error_free_irq;
	}

	cyttsp_info("%s: Successful registration\n", CYTTSP_I2C_NAME);
	goto success;

error_free_irq:
	cyttsp_alert("Error: Failed to register IRQ handler\n");
	free_irq(client->irq,ts);

error_free_device:
	if (input_device) {
		input_free_device(input_device);
	}

success:
	return retval;
}

static u8 *pantech_auto_check(void)
{
	u8 host_reg, byte_data[4], prev_data=0xff, byte_node1[MAX_NODE], byte_node2[MAX_NODE], send_byte[MAX_NODE];
	int retval = CYTTSP_OPERATIONAL, retry_cnt = 100, i;
	struct cyttsp *ts = ts_temp;

	printk("pantech_auto_check!! start\n");

	// If phone enter a poweroff, Stop firmware update
	if(Touch_Status >= TOUCH_POWEROFF)
		return NULL;

	// Enter Test Mode
	host_reg = CYTTSP_TEST_MODE;
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, sizeof(host_reg), &host_reg);

	msleep(100);

	// Read Raw counts or baseline

	byte_data[0]  = 0x00;

	do {
		/* Read Count  */
		retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_MODE , sizeof(byte_data), (u8 *)&byte_data);
		msleep(10);
	}
	while (byte_data[0] == prev_data && --retry_cnt);

	prev_data = byte_data[0];

	do {
		/* 	Read Count 
			Must set a i2c.h	I2C_SMBUS_BLOCK_MAX	32 -> I2C_SMBUS_BLOCK_MAX	256
		*/
		
		retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_SENSOR_BASE , sizeof(byte_node1), (u8 *)&byte_node1);
		msleep(10);
	}
	while (retval < CYTTSP_OPERATIONAL && --retry_cnt);	

	//  Read Raw counts or baseline
	
	host_reg = CYTTSP_T_TEST_MODE;
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, sizeof(host_reg), &host_reg);

	msleep(100);

	byte_data[0]  = 0x00;
	retry_cnt = 100;
	
	do {
		/* Read Count  */
		retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_MODE , sizeof(byte_data), (u8 *)&byte_data);
		msleep(10);
	}
	while (byte_data[0] == prev_data && --retry_cnt);

	prev_data = byte_data[0];

	do {
		/* Read Count  */
		retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_SENSOR_BASE , sizeof(byte_node2), (u8 *)&byte_node2);
		msleep(10);
	}
	while (retval < CYTTSP_OPERATIONAL && --retry_cnt);	
	
	for(i=0; i<MAX_NODE; i++)
	{
		if(byte_node1[i] >= CYTTSP_BASE_MIN && byte_node1[i] <= CYTTSP_BASE_MAX && 
			byte_node2[i] >= CYTTSP_BASE_MIN && byte_node2[i] <= CYTTSP_BASE_MAX)
			send_byte[i] = 0;
		else
			send_byte[i] = 1;

//		printk("Check Valid %d, byte_node1 %d, byte_node2 %d : %d\n", i , byte_node1[i], byte_node2[i], send_byte[i]);
	}

	// Retrun Operate Mode
	host_reg = CYTTSP_OPERATE_MODE;
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, sizeof(host_reg), &host_reg);	
	msleep(100);

	printk("pantech_auto_check!! end\n");

	return send_byte;

}

static int pantech_selftest_check(void)
{
	u8 host_reg, byte_data[2];
	int retval = CYTTSP_OPERATIONAL, ret =0;
	struct cyttsp *ts = ts_temp;

	printk("pantech_selftest_check!! start\n");

	// If phone enter a poweroff, Stop firmware update
	if(Touch_Status >= TOUCH_POWEROFF)
		return NULL;

	// Enter system information Mode
	host_reg = CYTTSP_SYSINFO_MODE;
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, sizeof(host_reg), &host_reg);

	msleep(100);

	// Start self test
	host_reg = 0x01;
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_SELF_TEST, sizeof(host_reg), &host_reg);

	msleep(1000);

	// Read test result
	retval = i2c_smbus_read_i2c_block_data(ts->client, CYTTSP_REG_SELF_TEST , sizeof(byte_data), (u8 *)&byte_data);

	printk("0x18 test: %02x, 0x19 test: %02x\n", byte_data[0], byte_data[1]);
		
	// Retrun Operate Mode
	host_reg = CYTTSP_OPERATE_MODE;
	retval = i2c_smbus_write_i2c_block_data(ts->client, CYTTSP_REG_BASE, sizeof(host_reg), &host_reg);	
	msleep(100);

	printk("pantech_selftest_check!! end\n");	

	if(byte_data[0] != 0)
		return CYTTSP_BIST_PROCESS;
	else if(byte_data[1] != 0xff)
	{
		if(!(byte_data[1] & 0x01))
			return CYTTSP_OPEN_TEST;
		else if(!(byte_data[1] & 0x02))
			return CYTTSP_SHORT_GND;
		else if(!(byte_data[1] & 0x04))
			return CYTTSP_SHORT_VDD;
		else if(!(byte_data[1] & 0x08))
			return CYTTSP_SHORT_PIN;
		else if(!(byte_data[1] & 0x10))
			return CYTTSP_LOCAL_IDAC;
		else if(!(byte_data[1] & 0x20))
			return CYTTSP_GLOBAL_IDAC;
		else if(!(byte_data[1] & 0x40))
			return CYTTSP_BASELINE_TEST;		
		else if(!(byte_data[1] & 0x80))
			return CYTTSP_COMPLETE_BIT;				
	}
	else	
		return 0;

}


static void init_hw_setting(void)
{
	int rc; 
	struct regulator *vreg_touch, *vreg_power_1_8;
        
	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_CHG, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(GPIO_TOUCH_CHG, 0);	

	// Power On, AVDD
	vreg_touch = regulator_get(NULL, "8058_l19");
	if (IS_ERR(vreg_touch))
	{
		rc = PTR_ERR(vreg_touch);
		printk(KERN_ERR "%s: regulator get of %s failed (%d)\n",
					__func__, (char *) vreg_touch, rc);
	}

	rc = regulator_set_voltage(vreg_touch, 3000000, 3000000);
       if (rc) {
		printk(KERN_ERR "%s: vreg set level failed (%d)\n", __func__, rc);
		return;
	}
		
	rc = regulator_enable(vreg_touch);

	if (rc) {
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",__func__, rc);
		return;
	}     

#if EF33S_BDVER_GE(WS20) || EF34K_BDVER_GE(WS20)
	vreg_power_1_8 = regulator_get(NULL, "8058_l11");
	if (IS_ERR(vreg_power_1_8))
	{
		rc = PTR_ERR(vreg_power_1_8);
		printk(KERN_ERR "%s: regulator get of %s failed (%d)\n",
					__func__, (char *) vreg_power_1_8, rc);
	}

	rc = regulator_set_voltage(vreg_power_1_8, 1900000, 1900000);
       if (rc) {
		printk(KERN_ERR "%s: vreg set level failed (%d)\n", __func__, rc);
		return;
	}
		
	rc = regulator_enable(vreg_power_1_8);

	if (rc) {
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",__func__, rc);
		return;
	}
	
#else

	// Power On DVDD
	vreg_power_1_8 = regulator_get(NULL, "8058_lvs0");
	if (IS_ERR(vreg_power_1_8))
	{
		rc = PTR_ERR(vreg_power_1_8);
			printk(KERN_ERR "%s: regulator get of %s failed (%d)\n",
				__func__, (char *) vreg_power_1_8, rc);
	}

	rc = regulator_enable(vreg_power_1_8);
#endif

	gpio_set_value(GPIO_TOUCH_CHG, 1);
	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_CHG, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_ID, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	gpio_set_value(GPIO_TOUCH_ID, 0);

	regulator_put(vreg_touch);
	regulator_put(vreg_power_1_8);

	msleep(100);
	
	Touch_Status = TOUCH_POWERON;
}


/* I2C driver probe function */
static int __devinit cyttsp_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cyttsp *ts;
	int error;
	int retval = CYTTSP_OPERATIONAL;
#ifdef FEATURE_SKY_PROCESS_CMD_KEY
	int rc;
#endif

	cyttsp_info("Start Probe\n");

	/* allocate and clear memory */
	ts = kzalloc (sizeof(struct cyttsp),GFP_KERNEL);
	if (ts == NULL) {
		cyttsp_xdebug1("err kzalloc for cyttsp\n");
		retval = -ENOMEM;
	}
	
	init_hw_setting();
	if (!(retval < CYTTSP_OPERATIONAL)) {
		/* register driver_data */
		ts->client = client;
		ts->platform_data = client->dev.platform_data;
		i2c_set_clientdata(client,ts);
	       ts->client->irq = IRQ_TOUCH_INT;
		error = cyttsp_initialize(client, ts);
		if (error) {
			cyttsp_xdebug1("err cyttsp_initialize\n");
			if (ts != NULL) {
				/* deallocate memory */
				kfree(ts);	
			}
/*
			i2c_del_driver(&cyttsp_driver);
*/
			retval = -ENODEV;
		}
		else {
			cyttsp_openlog();
		}
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (!(retval < CYTTSP_OPERATIONAL)) {
		ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		ts->early_suspend.suspend = cyttsp_early_suspend;
		ts->early_suspend.resume = cyttsp_late_resume;
		register_early_suspend(&ts->early_suspend);
	}
#endif /* CONFIG_HAS_EARLYSUSPEND */

#ifdef FEATURE_SKY_PROCESS_CMD_KEY
	rc = misc_register(&touch_event);
	if (rc) {
		pr_err("::::::::: can''t register touch_fops\n");
	}
#endif    
	cyttsp_info("Start Probe %s\n", (retval < CYTTSP_OPERATIONAL) ? "FAIL" : "PASS");

	ts_temp = ts;

	
#ifdef FEATURE_CYTTSP_FIRMWAREUPGRADE
  #if defined(CONFIG_APACHE_BOARD) 
  schedule_work(&ts->work3); 
//  	INIT_DELAYED_WORK(&work_delay_firmware,check_firmware_update);
//	schedule_delayed_work(&work_delay_firmware, msecs_to_jiffies(30000));
  #elif defined(CONFIG_EF33_BOARD) || defined(CONFIG_EF34_BOARD)
    #if EF33S_BDVER_GE(WS20) || EF34K_BDVER_GE(WS20)
	schedule_work(&ts->work3);
    #endif
  #else
 	schedule_work(&ts->work3); 
  #endif
#endif	

	wake_lock_init(&touch_wake_lock, WAKE_LOCK_SUSPEND, "touch");

	return retval;
}

/* Function to manage power-on resume */
static int cyttsp_resume(struct i2c_client *client)
{
	struct cyttsp *ts;
	int retval = CYTTSP_OPERATIONAL;

	dbg("Wake Up\n");

	ts = (struct cyttsp *) i2c_get_clientdata(client);

	if(ts == NULL)
		return retval;

	pantech_ctl_update(ISSP_IOCTL_POWER_ALL , 1);
	pantech_ctl_update(ISSP_IOCTL_INTR, 1);
	pantech_ctl_update(ISSP_COMPLITED_UPDATA, 0);

	Touch_Status = TOUCH_POWERON;
	
	msleep(100);

	request_irq (client->irq,cyttsp_irq,IRQF_TRIGGER_FALLING, client->dev.driver->name,ts);

#ifdef FEATURE_CYTTSP_HEARTBEAT
	mod_timer(&ts->timer, jiffies + CYTTSP_HEARTBEAT_TIME * HZ);
	start_heartbeat_timer = true;
#endif	

	/* re-enable the interrupt after resuming */
//	enable_irq(ts->client->irq);

	cyttsp_debug("Wake Up %s\n", (retval < CYTTSP_OPERATIONAL) ? "FAIL" : "PASS" );
	return retval;
}


/* Function to manage low power suspend */
static int cyttsp_suspend(struct i2c_client *client, pm_message_t message)
{
	struct cyttsp *ts; 
	int retval = CYTTSP_OPERATIONAL, id =0;

	dbg("Enter Sleep\n");

	ts = (struct cyttsp *) i2c_get_clientdata(client); 

	if(ts == NULL)
		return retval;

	/* disable worker */
	disable_irq_nosync(ts->client->irq);

#ifdef FEATURE_CYTTSP_HEARTBEAT
	start_heartbeat_timer = false;
	retval = cancel_work_sync(&ts->work2);
	del_timer(&ts->timer);
#endif
#ifdef FEATURE_CYTTSP_FIRMWAREUPGRADE
//	cancel_work_sync(&ts->work3);
#endif
	Touch_Status = TOUCH_POWEROFF;
	retval = cancel_work_sync(&ts->work);

	pantech_ctl_update(ISSP_IOCTL_POWER_ALL, 0);

	free_irq(client->irq,ts);

	pantech_ctl_update(ISSP_IOCTL_SCLK_TO_GPIO, 1);
	pantech_ctl_update(ISSP_IOCTL_DATA_TO_GPIO, 1);
	
	pantech_ctl_update(ISSP_IOCTL_INTR, 0);
	pantech_ctl_update(ISSP_IOCTL_SCLK, 0);	
	pantech_ctl_update(ISSP_IOCTL_DATA, 0);

	for (id = 0; id < CYTTSP_NUM_TRACK_ID; id++) 
	{
		if (ts->active_track[id] != CYTTSP_NOTOUCH)
		{
			dbg("Touch Release ID: %d, X: %d, Y: %d\n", 
				id, prev_mt_pos[id][CYTTSP_XPOS], prev_mt_pos[id][CYTTSP_YPOS]);
			input_report_abs(ts->input, ABS_MT_TRACKING_ID, id);
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, CYTTSP_NOTOUCH);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, 10);
			input_report_abs(ts->input, ABS_MT_POSITION_X, prev_mt_pos[id][CYTTSP_XPOS]);
			input_report_abs(ts->input, ABS_MT_POSITION_Y, prev_mt_pos[id][CYTTSP_YPOS]);
			CYTTSP_MT_SYNC(ts->input);
			input_sync(ts->input);
		}
	}
	return retval;
}

/* registered in driver struct */
static int __devexit cyttsp_remove(struct i2c_client *client)
{
	struct cyttsp *ts;
	int err;

	cyttsp_alert("Unregister\n");

	/* clientdata registered on probe */
	ts = i2c_get_clientdata(client);
	device_remove_file(&ts->client->dev, &dev_attr_irq_enable);

	/* Start cleaning up by removing any delayed work and the timer */
	if (cancel_delayed_work((struct delayed_work *)&ts->work)<0) {
		cyttsp_alert("error: could not remove work from workqueue\n");
	}

	/* free up timer or irq */
    if(ts->client->irq == 0) {	
		err = del_timer(&ts->timer);
		if (err < 0) {
			cyttsp_alert("error: failed to delete timer\n");
		}
	}
	else {
		
#ifdef FEATURE_CYTTSP_HEARTBEAT
	start_heartbeat_timer = false;
	del_timer(&ts->timer);
#endif		
		free_irq(client->irq,ts);
	}

	/* housekeeping */
	if (ts != NULL) {
		kfree(ts);
	}

	/* clientdata registered on probe */
	cyttsp_alert("Leaving\n");

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cyttsp_early_suspend(struct early_suspend *handler)
{
	struct cyttsp *ts;

	ts = container_of(handler, struct cyttsp, early_suspend);
	cyttsp_suspend(ts->client, PMSG_SUSPEND);
}

static void cyttsp_late_resume(struct early_suspend *handler)
{
	struct cyttsp *ts;

	ts = container_of(handler, struct cyttsp, early_suspend);
	cyttsp_resume(ts->client);
}
#endif  /* CONFIG_HAS_EARLYSUSPEND */

static int cyttsp_init(void)
{
	int ret;

	cyttsp_info("Cypress TrueTouch(R) Standard Product I2C Touchscreen Driver (Built %s @ %s)\n",__DATE__,__TIME__);
	ret = i2c_add_driver(&cyttsp_driver);

	return ret;
}

static void cyttsp_exit(void)
{
	return i2c_del_driver(&cyttsp_driver);
}

module_init(cyttsp_init);
module_exit(cyttsp_exit);

