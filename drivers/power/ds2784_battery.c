/* drivers/power/ds2784_battery.c
 *
 * Copyright (C) 2009 HTC Corporation
 * Author: Justin Lin <Justin_Lin@htc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/android_alarm.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/wakelock.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/ds2784_battery.h>
#include <mach/htc_battery.h>
#include <asm/mach-types.h>
#include "../../arch/arm/mach-msm/proc_comm.h"

#include "../w1/w1.h"
#include "../w1/slaves/w1_ds2784.h"
#include <linux/time.h>
#include <linux/rtc.h>

struct battery_info {
	u8 batt_id;		/* Battery ID from ADC */
	int batt_vol;		/* Battery voltage from ADC */
	int batt_temp;		/* Battery Temperature (C) from formula and ADC */
	int batt_current;	/* Battery current from ADC */
	int batt_current_avg;
	u8 level;		/* formula */
	u8 level_last;
	u8 charging_source;	/* 0: no cable, 1:usb, 2:AC */
	u8 charging_enabled;	/* 0: Disable, 1: Slow charge,
							2: Fast charge*/
	u32 full_bat;		/* Full capacity of battery (mAh) */
	u8  guage_status_reg;/* guage status*/
	u32 acr;
	u32 active_empty;
	u8 battery_full;
	u8 cooldown;		/* was overtemp */
	u8 last_charge_mode; /* previous charger state */
	u8 charge_mode;
	u8 OT_critical;
	u8 OT_hot;
	u8 OT_warm;
	u8 OT_cold;
};
struct ds2784_device_info {
	struct device *dev;
	struct device *w1_dev;
	struct workqueue_struct *monitor_wqueue;
	struct work_struct monitor_work;
	struct battery_info rep;

	/* lock to protect the battery info */
	struct mutex lock;

	/* DS2784 data, valid after calling ds2784_battery_read_status() */
	unsigned long update_time;	/* jiffies when data read */
	char raw[DS2784_DATA_SIZE];	/* raw DS2784 data */
	int voltage_mV;			/* units of mV */
	int current_mA;			/* units of mA */
	int current_avg_mA;		/* unit of avg mA */
	int temp_C;				/* units of 0.1 C */
	int charge_status;		/* POWER_SUPPLY_STATUS_* */
	int percentage;			/* battery percentage */
	int guage_status_reg;	/* battery status register offset=01h*/
	long full_charge_count;  /* Full charge counter */
	int acr;
	int active_empty;
	struct alarm alarm;
	struct wake_lock work_wake_lock;
	u8 slow_poll;
	ktime_t last_poll;
	ktime_t last_charge_seen;
};
static struct wake_lock vbus_wake_lock;

/* Battery ID */
#define BATT_FIRST_SOURCE     (1)  /* 1: Main source battery */
#define BATT_UNKNOWN        (255)  /* Other: Unknown battery */

#define BATT_RSNSP			(67)	/*Passion battery source 1*/
#define BATT_CHECK_TIME_CHARGING_FULL  (3600)

static struct ds2784_device_info htc_batt_info;
static int htc_battery_initial;

#define SOURCE_NONE	0
#define SOURCE_USB	1
#define SOURCE_AC	2

#define CHARGE_OFF	0
#define CHARGE_SLOW	1
#define CHARGE_FAST	2
#define CHARGE_BATT_DISABLE     3 /* disable charging at battery */

#define TEMP_CRITICAL	600 /* no charging at all */
#define TEMP_HOT	500 /* no fast charge, no charge > 4.1v */
#define TEMP_WARM	450 /* no fast charge above this */

#define TEMP_HOT_MAX_MV	4100 /* stop charging here when hot */
#define TEMP_HOT_MIN_MV	3800 /* resume charging here when hot */
#define CE_DISABLE_MIN_MV 4100

/* When we're awake or running on wall power, sample the battery
 * gauge every FAST_POLL seconds.  If we're asleep and on battery
 * power, sample every SLOW_POLL seconds
 */
#define FAST_POLL	(1 * 60)
#define SLOW_POLL	(10 * 60)

static BLOCKING_NOTIFIER_HEAD(ds2784_notifier_list);
int ds2784_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&ds2784_notifier_list, nb);
}

int ds2784_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&ds2784_notifier_list, nb);
}

static int ds2784_blocking_notify(unsigned long val, void *v)
{
	int chg_ctl;

	if (val == DS2784_CHARGING_CONTROL) {
		chg_ctl = *(int *)v;
		if (htc_batt_info.rep.batt_id != BATT_UNKNOWN) {
			/* only notify at changes */
			if (htc_batt_info.rep.charging_enabled == chg_ctl)
				return 0;
			else
				htc_batt_info.rep.charging_enabled = chg_ctl;
		} else {
			htc_batt_info.rep.charging_enabled = DISABLE;
			v = DISABLE;
			pr_info("[HTC_BATT] Unknow battery\n");
		}

	}
	return blocking_notifier_call_chain(&ds2784_notifier_list, val, v);

}

int ds2784_get_battery_info(struct battery_info_reply *batt_info)
{
	batt_info->batt_id = htc_batt_info.rep.batt_id;
	batt_info->batt_vol = htc_batt_info.rep.batt_vol;
	batt_info->batt_temp = htc_batt_info.rep.batt_temp;
	batt_info->batt_current = htc_batt_info.rep.batt_current;
	batt_info->level = htc_batt_info.rep.level;
	batt_info->charging_source = htc_batt_info.rep.charging_source;
	batt_info->charging_enabled = htc_batt_info.rep.charging_enabled;
	batt_info->full_bat = htc_batt_info.rep.acr;
/* DS2784 did not support this over vchg, but we need to have reply */
	batt_info->over_vchg = 0;
	return 0;
}
ssize_t htc_battery_show_attr(struct device_attribute *attr,
					 char *buf)
{
	int len = 0;

	if (!strcmp(attr->attr.name, "batt_attr_text")) {
	len += scnprintf(buf + len, PAGE_SIZE - len,
		"batt_id: %d;\n"
		"batt_vol(mV): %d;\n"
		"batt_temp(C): %d;\n"
		"batt_current(mA): %d;\n"
		"batt_current_avg(mA): %d;\n"
		"level(%%): %d;\n"
		"charging_source: %d;\n"
		"charging_enabled: %d;\n"
		"acr(mAh): %d;\n"
		"active_empty(mAh): %d;\n"
		"guage_status_reg: %x;\n",
		htc_batt_info.rep.batt_id,
		htc_batt_info.rep.batt_vol,
		htc_batt_info.rep.batt_temp,
		htc_batt_info.rep.batt_current,
		htc_batt_info.rep.batt_current_avg,
		htc_batt_info.rep.level,
		htc_batt_info.rep.charging_source,
		htc_batt_info.rep.charging_enabled,
		htc_batt_info.rep.acr,
		htc_batt_info.rep.active_empty,
		htc_batt_info.rep.guage_status_reg
		);
		}
	return len;
}

static int cable_status_handler_func(struct notifier_block *nfb,
		unsigned long action, void *param)
{
	u32 cable_type = (u32)action;
	pr_info("[HTC_BATT] cable change to %d\n", cable_type);
/* When the cable plug out, reset all the related flag,
Let algorithm machine to judge latest state */
	if (cable_type == 0){
		htc_batt_info.full_charge_count = 0;
	}else if (cable_type == 0xff){
//		if (param)
//			htc_batt_info.rep.full_level = *(u32 *) param;
		return NOTIFY_OK;
	}else if (cable_type == 0x10){
		return NOTIFY_OK;
	}
	htc_batt_info.rep.charging_source = cable_type;

	ds2784_blocking_notify(DS2784_CHARGING_CONTROL,
		&htc_batt_info.rep.charging_source);

	return NOTIFY_OK;
}

static struct notifier_block cable_status_handler = {
	.notifier_call = cable_status_handler_func,
};

static int ds2784_set_cc(struct ds2784_device_info *di, bool enable)
{
	int ret;

	if (enable)
		di->raw[DS2784_REG_PORT] |= 0x02;
	else
		di->raw[DS2784_REG_PORT] &= ~0x02;
	ret = w1_ds2784_write(di->w1_dev, di->raw + DS2784_REG_PORT,
			      DS2784_REG_PORT, 1);
	if (ret != 1) {
		dev_warn(di->dev, "call to w1_ds2784_write failed (0x%p)\n",
			 di->w1_dev);
		return 1;
	}
	return 0;
}

static int ds2784_battery_read_status(struct ds2784_device_info *di)
{
	short result;
	int ret, start, count;

	/* The first time we read the entire contents of SRAM/EEPROM,
	 * but after that we just read the interesting bits that change. */
	if (htc_battery_initial == 0) {
		start = 0;
		count = DS2784_DATA_SIZE;
	} else {
		start = DS2784_REG_PORT;
		count = DS2784_REG_STBY_EMPTY_LSB - start + 1;
	}

	ret = w1_ds2784_read(di->w1_dev, di->raw + start, start, count);
	if (ret < 0) {
		pr_info("batt:abort this time to read gauge, ret = %d\n",ret);
		return 1;
	}
	if (ret != count) {
		dev_warn(di->dev, "call to w1_ds2784_read failed (0x%p)\n",
			 di->w1_dev);
		return 1;
	}
/*
Check if dummy battery in.
Workaround for dummy battery
Write ACR MSB to 0x05, ensure there must be 500mAH .
ONLY check when battery driver init.
*/
	if (htc_battery_initial == 0) {
		if (!memcmp(di->raw + 0x20, "DUMMY!", 6)) {
			unsigned char acr[2];

			pr_info("batt: dummy battery detected\n");

			/* reset ACC register to ~500mAh, since it may have zeroed out */
			acr[0] = 0x05;
			acr[1] = 0x06;
			mutex_lock(&htc_batt_info.lock);
			ret = w1_ds2784_write(di->w1_dev, acr,DS2784_REG_ACCUMULATE_CURR_MSB, 2);
			if (ret < 0) {
				msleep(5);
				if (w1_ds2784_write(di->w1_dev, acr,DS2784_REG_ACCUMULATE_CURR_MSB, 2) < 0)
					pr_info("batt: Write dummy ACR fail, ret = %d\n",ret);
			}
			mutex_unlock(&htc_batt_info.lock);
			}
		}
/*
Get Rsns, get from offset 69H . Rsnsp=1/Rsns
Judge if this is supported battery
*/
	mutex_lock(&htc_batt_info.lock);
	if (di->raw[DS2784_REG_RSNSP] != BATT_RSNSP)
		htc_batt_info.rep.batt_id = BATT_UNKNOWN;
	else
		htc_batt_info.rep.batt_id = BATT_FIRST_SOURCE;
	mutex_unlock(&htc_batt_info.lock);

/*
Get status reg
*/
	mutex_lock(&htc_batt_info.lock);
	di->guage_status_reg= di->raw[DS2784_REG_STS];
/*
Get Level
*/
	di->percentage = di->raw[DS2784_REG_RARC];

/*
Get Voltage
Unit=4.886mV, range is 0V to 4.99V
*/
	di->voltage_mV = (((di->raw[DS2784_REG_VOLT_MSB]<<8)
			|(di->raw[DS2784_REG_VOLT_LSB])) >> 5)*4886/1000;

/*
Get Current
Unit= 1.5625uV x Rsnsp(67)=104.68
*/
	result = ((di->raw[DS2784_REG_CURR_MSB]) << 8) |
		di->raw[DS2784_REG_CURR_LSB];
	di->current_mA = (((result * 15625) / 10000) * 67)/1000;

	result = ((di->raw[DS2784_REG_AVG_CURR_MSB]) << 8) |
		di->raw[DS2784_REG_AVG_CURR_LSB];
	di->current_avg_mA = (((result * 15625) / 10000) * 67)/1000;

/*
Get Temperature
Unit=0.125 degree C,therefore, give up LSB ,
just caculate MSB for temperature only.
*/
	result = (((signed char)di->raw[DS2784_REG_TEMP_MSB]) << 3) |
				     (di->raw[DS2784_REG_TEMP_LSB] >> 5);

	di->temp_C = result + (result / 4);

/*
Get ACR and Active Empty
*/
	result = ((di->raw[DS2784_REG_ACTIVE_EMPTY_MSB]<<8)|
		(di->raw[DS2784_REG_ACTIVE_EMPTY_LSB]));
	di->active_empty = ((result*625/100)*67)/1000;

	/* RAAC is in units of 1.6mAh */
	di->acr = (((di->raw[DS2784_REG_RAAC_MSB] << 8) |
			  di->raw[DS2784_REG_RAAC_LSB]) * 1600)/1000;

/*
Set to local for reply to framework
*/
	htc_batt_info.rep.batt_current = di->current_mA;
	htc_batt_info.rep.batt_current_avg = di->current_avg_mA;
	htc_batt_info.rep.batt_temp = di->temp_C;
	htc_batt_info.rep.batt_vol = di->voltage_mV;
	htc_batt_info.rep.level_last = htc_batt_info.rep.level;
	htc_batt_info.rep.level = di->percentage;
	htc_batt_info.rep.guage_status_reg = di->guage_status_reg;
	htc_batt_info.rep.acr = di->acr;
	htc_batt_info.rep.active_empty = di->active_empty;


	/* After battery driver gets initialized, send rpc request to inquiry
	 * the battery status in case of we lost some info
	 */
	if (htc_battery_initial == 0)
	{
		htc_batt_info.full_charge_count = 0;
		htc_battery_initial = 1;
	}
pr_info("[HTC_BATT]RSNSP=%d,RARC=%d,Vol=%dmV,Current=%dmA,Temp=%dC(1/10)\n"
		,di->raw[DS2784_REG_RSNSP]
		,di->raw[DS2784_REG_RARC]
		,di->voltage_mV
		,di->current_mA
		,di->temp_C);
	mutex_unlock(&htc_batt_info.lock);
	return 0;
}

static DEFINE_MUTEX(charge_state_lock);

static bool check_timeout(ktime_t now, ktime_t last, int seconds)
{
	ktime_t timeout = ktime_add(last, ktime_set(seconds, 0));
	return ktime_sub(timeout, now).tv64 < 0;
}

static int battery_adjust_charge_state(struct ds2784_device_info *di)
{
	unsigned source;
	int rc = 0;
	int temp, volt;
	u8 charge_mode;
	bool charge_timeout = false;
	int chg_ctl = DISABLE;

	mutex_lock(&charge_state_lock);

	temp = htc_batt_info.rep.batt_temp;
	volt = htc_batt_info.rep.batt_vol;

	source = htc_batt_info.rep.charging_source;

	/* initially our charge mode matches our source:
	 * NONE:OFF, USB:SLOW, AC:FAST
	 */
	charge_mode = source;


	if (htc_batt_info.rep.level <= 99)
		htc_batt_info.rep.battery_full = 0;

	if ((htc_batt_info.rep.guage_status_reg & 0x80) &&
	    (htc_batt_info.rep.batt_current_avg <= 40) &&
	    (htc_batt_info.rep.level == 100)) {
		htc_batt_info.rep.battery_full = 1;
		charge_mode = CHARGE_BATT_DISABLE;
	}

	if (temp >= TEMP_HOT) {
		if (temp >= TEMP_CRITICAL)
			charge_mode = CHARGE_BATT_DISABLE;

		/* once we charge to max voltage when hot, disable
		 * charging until the temp drops or the voltage drops
		 */
		if (volt >= TEMP_HOT_MAX_MV)
			htc_batt_info.rep.cooldown = 1;
	}

	/* when the battery is warm, only charge in slow charge mode */
	if ((temp >= TEMP_WARM) && (charge_mode == CHARGE_FAST))
		charge_mode = CHARGE_SLOW;

	if (htc_batt_info.rep.cooldown) {
		if ((temp < TEMP_WARM) || (volt <= TEMP_HOT_MIN_MV))
			htc_batt_info.rep.cooldown = 0;
		else
			charge_mode = CHARGE_BATT_DISABLE;
	}

	if (htc_batt_info.rep.battery_full == 1)
		di->last_charge_seen = di->last_poll;
	else if (htc_batt_info.rep.last_charge_mode != CHARGE_OFF &&
		check_timeout(di->last_poll, di->last_charge_seen, BATT_CHECK_TIME_CHARGING_FULL)) {
		if (htc_batt_info.rep.last_charge_mode == CHARGE_BATT_DISABLE) {
			/* The charger is only powering the phone. Toggle the
			 * enable line periodically to prevent auto shutdown.
			 */
			di->last_charge_seen = di->last_poll;
			pr_info("batt: charging POKE CHARGER\n");
			chg_ctl = DISABLE;
			ds2784_blocking_notify(DS2784_CHARGING_CONTROL,&chg_ctl);
			udelay(10);
			chg_ctl = (source == CHARGE_FAST) ? ENABLE_FAST_CHG : ENABLE_SLOW_CHG;
			ds2784_blocking_notify(DS2784_CHARGING_CONTROL,&chg_ctl);
		} else {
			/* The charger has probably stopped charging. Turn it
			 * off until the next sample period.
			 */
			charge_timeout = true;
			charge_mode = CHARGE_OFF;
		}
	}

	if (source == CHARGE_OFF)
		charge_mode = CHARGE_OFF;

	/* Don't use CHARGE_BATT_DISABLE unless the voltage is high since the
	 * voltage drop over the discharge-path diode can cause a shutdown.
	 */
	if (charge_mode == CHARGE_BATT_DISABLE && volt < CE_DISABLE_MIN_MV)
		charge_mode = CHARGE_OFF;

	if (htc_batt_info.rep.last_charge_mode == charge_mode)
		goto done;

	htc_batt_info.rep.last_charge_mode = charge_mode;
	htc_batt_info.rep.charge_mode = charge_mode;

	switch (charge_mode) {
	case CHARGE_OFF:
		/* CHARGER_EN is active low.  Set to 1 to disable. */
		chg_ctl = DISABLE;
		ds2784_blocking_notify(DS2784_CHARGING_CONTROL,&chg_ctl);
		ds2784_set_cc(di, true);
		if ((temp >= TEMP_CRITICAL) ||
            	(htc_batt_info.rep.OT_critical == 1) ||
            	(htc_batt_info.rep.OT_cold == 1))
			pr_info("batt: charging OFF [OVERTEMP]\n");
		else if (htc_batt_info.rep.cooldown)
			pr_info("batt: charging OFF [COOLDOWN]\n");
		else if (htc_batt_info.rep.battery_full)
			pr_info("batt: charging OFF [FULL]\n");
		else if (charge_timeout)
			pr_info("batt: charging OFF [TIMEOUT]\n");
		else
			pr_info("batt: charging OFF\n");
		break;
	case CHARGE_BATT_DISABLE:
		di->last_charge_seen = di->last_poll;
		ds2784_set_cc(di, false);
		chg_ctl = (source == CHARGE_FAST) ? ENABLE_FAST_CHG : ENABLE_SLOW_CHG;
		ds2784_blocking_notify(DS2784_CHARGING_CONTROL,&chg_ctl);
		if ((temp >= TEMP_CRITICAL) ||
            	(htc_batt_info.rep.OT_critical == 1) ||
            	(htc_batt_info.rep.OT_cold == 1))
			pr_info("batt: charging BATTOFF [OVERTEMP]\n");
		else if (htc_batt_info.rep.cooldown)
			pr_info("batt: charging BATTOFF [COOLDOWN]\n");
		else if (htc_batt_info.rep.battery_full)
			pr_info("batt: charging BATTOFF [FULL]\n");
		else
			pr_info("batt: charging BATTOFF [UNKNOWN]\n");
		break;
	case CHARGE_SLOW:
		di->last_charge_seen = di->last_poll;
		ds2784_set_cc(di, true);
		chg_ctl = ENABLE_SLOW_CHG;
		ds2784_blocking_notify(DS2784_CHARGING_CONTROL,&chg_ctl);
		pr_info("batt: charging SLOW\n");
		break;
	case CHARGE_FAST:
		di->last_charge_seen = di->last_poll;
		ds2784_set_cc(di, true);
		chg_ctl = ENABLE_FAST_CHG;
		ds2784_blocking_notify(DS2784_CHARGING_CONTROL,&chg_ctl);
		pr_info("batt: charging FAST\n");
		break;
	}
	rc = 1;
done:
	mutex_unlock(&charge_state_lock);
	return rc;
}
static void ds2784_program_alarm(struct ds2784_device_info *di, int seconds)
{
	ktime_t low_interval = ktime_set(seconds - 10, 0);
	ktime_t slack = ktime_set(20, 0);
	ktime_t next;

	next = ktime_add(di->last_poll, low_interval);

	alarm_start_range(&di->alarm, next, ktime_add(next, slack));
}

static void ds2784_battery_update_status(struct ds2784_device_info *di)
{
	int last_level;
	last_level = di->percentage;

	ds2784_battery_read_status(di);

	if ((last_level != di->percentage) || di->temp_C > TEMP_WARM)
		ds2784_blocking_notify(DS2784_LEVEL_UPDATE, &htc_batt_info.rep.level);
}

static void ds2784_battery_work(struct work_struct *work)
{
	struct ds2784_device_info *di = container_of(work,
		struct ds2784_device_info, monitor_work);
	unsigned long flags;
	
	ds2784_battery_update_status(di);

	di->last_poll = alarm_get_elapsed_realtime();

	battery_adjust_charge_state(di);

	/* prevent suspend before starting the alarm */
	local_irq_save(flags);

	wake_unlock(&di->work_wake_lock);
	ds2784_program_alarm(di, FAST_POLL);
	local_irq_restore(flags);
}

static void ds2784_battery_alarm(struct alarm *alarm)
{
	struct ds2784_device_info *di =
		container_of(alarm, struct ds2784_device_info, alarm);
	wake_lock(&di->work_wake_lock);
	queue_work(di->monitor_wqueue, &di->monitor_work);
}

static int ds2784_battery_probe(struct platform_device *pdev)
{
	int rc;
	struct ds2784_device_info *di;
	struct ds2784_platform_data *pdata;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	di->update_time = jiffies;
	platform_set_drvdata(pdev, di);

	pdata = pdev->dev.platform_data;
	di->dev		= &pdev->dev;
	di->w1_dev	     = pdev->dev.parent;

	htc_batt_info.rep.last_charge_mode = 0xff;

	INIT_WORK(&di->monitor_work, ds2784_battery_work);
	di->monitor_wqueue = create_freezeable_workqueue(
		dev_name(&pdev->dev));

	/* init to something sane */
	di->last_poll = alarm_get_elapsed_realtime();

	if (!di->monitor_wqueue) {
		rc = -ESRCH;
		goto fail_workqueue;
	}
	wake_lock_init(&di->work_wake_lock, WAKE_LOCK_SUSPEND,
			"ds2784-battery");
	alarm_init(&di->alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
			ds2784_battery_alarm);
	wake_lock(&di->work_wake_lock);
	queue_work(di->monitor_wqueue, &di->monitor_work);
	return 0;

fail_workqueue:
	kfree(di);
	return rc;
}

static int ds2784_battery_remove(struct platform_device *pdev)
{
	struct ds2784_device_info *di = platform_get_drvdata(pdev);

	destroy_workqueue(di->monitor_wqueue);

	return 0;
}

/* FIXME: power down DQ master when not in use. */
static int ds2784_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct ds2784_device_info *di = platform_get_drvdata(pdev);
	unsigned long flags;

	/* If we are on battery, reduce our update rate until
	 * we next resume.
	 */
	if (di->rep.charging_source == SOURCE_NONE) {
		local_irq_save(flags);
		ds2784_program_alarm(di, SLOW_POLL);
		di->slow_poll= 1;
		local_irq_restore(flags);
	}
	gpio_direction_output(87, 0);
	return 0;
}
static int ds2784_resume(struct platform_device *pdev)
{
	struct ds2784_device_info *di = platform_get_drvdata(pdev);
	unsigned long flags;

	/* We might be on a slow sample cycle.  If we're
	 * resuming we should resample the battery state
	 * if it's been over a minute since we last did
	 * so, and move back to sampling every minute until
	 * we suspend again.
	 */
	gpio_direction_output(87, 1);
	ndelay(100 * 1000);

	if (di->slow_poll) {
		local_irq_save(flags);
		ds2784_program_alarm(di, FAST_POLL);
		di->slow_poll = 0;
		local_irq_restore(flags);
	}
	return 0;
}


MODULE_ALIAS("platform:ds2784-battery");
static struct platform_driver ds2784_battery_driver = {
	.driver = {
		.name = "ds2784-battery",
	},
	.suspend = ds2784_suspend,
	.resume = ds2784_resume,
	.probe	  = ds2784_battery_probe,
	.remove   = ds2784_battery_remove,
};

static int __init ds2784_battery_init(void)
{
	wake_lock_init(&vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_present");
	register_notifier_cable_status(&cable_status_handler);
	mutex_init(&htc_batt_info.lock);
	return platform_driver_register(&ds2784_battery_driver);
}

static void __exit ds2784_battery_exit(void)
{
	platform_driver_unregister(&ds2784_battery_driver);
}

module_init(ds2784_battery_init);
module_exit(ds2784_battery_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Justin Lin <Justin_lin@htc.com>");
MODULE_DESCRIPTION("ds2784 battery driver");
