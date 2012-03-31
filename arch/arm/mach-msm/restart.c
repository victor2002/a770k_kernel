/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/pmic8901.h>

#include <mach/msm_iomap.h>
#include <mach/scm-io.h>
#include <mach/restart.h>

#include "sky_sys_reset.h"

#define TCSR_WDT_CFG 0x30

#define WDT0_RST       (MSM_TMR0_BASE + 0x38)
#define WDT0_EN        (MSM_TMR0_BASE + 0x40)
#define WDT0_BARK_TIME (MSM_TMR0_BASE + 0x4C)
#define WDT0_BITE_TIME (MSM_TMR0_BASE + 0x5C)

#define PSHOLD_CTL_SU (MSM_TLMM_BASE + 0x820)

#define RESTART_REASON_ADDR 0x2A05F65C

static int restart_mode;

#define SDCARD_REBOOT_OK		0xCECEECEC

#ifdef CONFIG_SW_RESET
#if defined(CONFIG_PANTECH_ERROR_LOG)
extern void pantech_errlog_display_add_log(const char *log, int size);
extern void pantech_errlog_noti_crash_to_marm(uint32_t reason);
extern void pantech_errlog_display_with_errlog(bool bmArm, bool do_panic);
extern void pantech_errlog_add_log(const char *log, int size);
#endif

#endif
#ifdef CONFIG_SKY_CHARGING
#define NORMAL_RESET_MAGIC_NUM 0xbaabcddc
#endif
#ifdef CONFIG_MSM_DLOAD_MODE
static int in_panic;
static int reset_detection;

static int panic_prep_restart(struct notifier_block *this,
			      unsigned long event, void *ptr)
{
	in_panic = 1;
	return NOTIFY_DONE;
}

static struct notifier_block panic_blk = {
	.notifier_call	= panic_prep_restart,
};

#ifdef MODEL_SKY
#define set_dload_mode(x) do {} while (0)
#else
static void set_dload_mode(int on)
{
	void *dload_mode_addr;
	dload_mode_addr = ioremap_nocache(0x2A05F000, SZ_4K);
	if (dload_mode_addr) {
		writel(on ? 0xE47B337D : 0, dload_mode_addr);
		writel(on ? 0xCE14091A : 0,
		       dload_mode_addr + sizeof(unsigned int));
		dsb();
		iounmap(dload_mode_addr);
	}
}
#endif

static int reset_detect_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val = reset_detection;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	switch (reset_detection) {

	case 0:
		/*
		*  Deactivate reset detection. Unset the download mode flag only
		*  if someone hasn't already set restart_mode to something other
		*  than RESTART_NORMAL.
		*/
		if (restart_mode == RESTART_NORMAL)
			set_dload_mode(0);
	break;

	case 1:
#ifndef MODEL_SKY
		set_dload_mode(1);
#endif
	break;

	default:
		reset_detection = old_val;
		return -EINVAL;
	break;

	}

	return 0;
}

module_param_call(reset_detection, reset_detect_set, param_get_int,
			&reset_detection, 0644);
#else
#define set_dload_mode(x) do {} while (0)
#endif

void msm_set_restart_mode(int mode)
{
	restart_mode = mode;
}
EXPORT_SYMBOL(msm_set_restart_mode);

static void msm_power_off(void)
{
#ifdef CONFIG_SKY_CHARGING
	void *restart_reason;

	restart_reason = ioremap_nocache(RESTART_REASON_ADDR, SZ_4K);
    	writel(0x00, restart_reason);
    	writel(0x00, restart_reason+4);
        iounmap(restart_reason);
#endif
	printk(KERN_NOTICE "Powering off the SoC\n");
	pm8058_reset_pwr_off(0);
	pm8901_reset_pwr_off(0);
	writel(0, PSHOLD_CTL_SU);
	mdelay(10000);
	printk(KERN_ERR "Powering off has failed\n");
	return;
}
int sky_reset_reason=SYS_RESET_REASON_UNKNOWN;
void arch_reset(char mode, const char *cmd)
{
	void *restart_reason;

#ifdef CONFIG_MSM_DLOAD_MODE
#ifndef CONFIG_PANTECH_ERR_CRASH_LOGGING
	if (in_panic || restart_mode == RESTART_DLOAD)
		set_dload_mode(1);
#endif
	/*
	*  If we're not currently panic-ing, and if reset detection is active,
	*  unset the download mode flag. However, do this only if the current
	*  restart mode is RESTART_NORMAL.
	*/
// paiksun...
#ifndef MODEL_SKY
	if (reset_detection && !in_panic && restart_mode == RESTART_NORMAL)
#endif
		set_dload_mode(0);
#endif

	printk(KERN_NOTICE "Going down for restart now\n");

	pm8058_reset_pwr_off(1);
	printk(KERN_NOTICE "arch_reset start allydrop   in_panic:%x ,cmd '%s'\n",in_panic,cmd);

	if (cmd != NULL) {
		restart_reason = ioremap_nocache(RESTART_REASON_ADDR, SZ_4K);
		if (!strncmp(cmd, "bootloader", 10)) {
			writel(0x77665500, restart_reason);
		} else if (!strncmp(cmd, "recovery", 8)) {
			writel(0x77665502, restart_reason);
		} else if (!strncmp(cmd, "sdcard", 5)) {
			printk(KERN_NOTICE "allydrop arch_reset:%x\n",mode);
			writel(SDCARD_REBOOT_OK, restart_reason);
#ifdef CONFIG_SW_RESET
		} else if (!strncmp(cmd, "androidpanic", 12)) {
			printk(KERN_NOTICE "allydrop android panic!!!!in_panic:%x\n",in_panic);
			sky_reset_reason=SYS_RESET_REASON_ANDROID;
#if defined(CONFIG_PANTECH_ERROR_LOG)   
			pantech_errlog_display_add_log("android panic\n", strlen("android panic in_panic\n"));
			pantech_errlog_add_log("android framework error\n", strlen("android framework error\n"));
			pantech_errlog_display_with_errlog(false,false);
#endif   
			panic("android framework error\n"); 
#endif   
		} else if (!strncmp(cmd, "oem-", 4)) {
			unsigned long code;
			strict_strtoul(cmd + 4, 16, &code);
			code = code & 0xff;
			writel(0x6f656d00 | code, restart_reason);
		} else {
				if(in_panic){//sky_reset_reason
					writel(sky_reset_reason, restart_reason);
				}
				else
					writel(0x77665501, restart_reason);
                }
#ifdef CONFIG_SKY_CHARGING
		writel(NORMAL_RESET_MAGIC_NUM, restart_reason+4);
#endif
		iounmap(restart_reason);
	}
	else //20110521 reboot err temp
	{
		restart_reason = ioremap_nocache(RESTART_REASON_ADDR, SZ_4K);
				if(in_panic){//sky_reset_reason
					writel(sky_reset_reason, restart_reason);
				}
				else
					writel(0x77665501, restart_reason);
#ifdef CONFIG_SKY_CHARGING
		writel(NORMAL_RESET_MAGIC_NUM, restart_reason+4);
#endif
		iounmap(restart_reason);
	}

	writel(0, WDT0_EN);
	dsb();
	writel(0, PSHOLD_CTL_SU); /* Actually reset the chip */
	mdelay(5000);

	printk(KERN_NOTICE "PS_HOLD didn't work, falling back to watchdog\n");

	writel(5*0x31F3, WDT0_BARK_TIME);
	writel(0x31F3, WDT0_BITE_TIME);
	writel(3, WDT0_EN);
	dmb();
	secure_writel(3, MSM_TCSR_BASE + TCSR_WDT_CFG);

	mdelay(10000);
	printk(KERN_ERR "Restarting has failed\n");
}

static int __init msm_restart_init(void)
{
#ifdef CONFIG_MSM_DLOAD_MODE
	atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);

	/* Reset detection is switched on below.*/
	set_dload_mode(1);
	reset_detection = 1;
#endif

	pm_power_off = msm_power_off;

	return 0;
}

late_initcall(msm_restart_init);
