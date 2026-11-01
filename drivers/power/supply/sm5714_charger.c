// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal charger control for the Silicon Mitus SM5714 (Samsung Galaxy M52 5G).
 *
 * Without a driver the chip charges with its reset defaults (500 mA input,
 * 500 mA charge) from any adapter. This driver polls the BC1.2 result of the
 * SM5714 MUIC and the battery temperature and sets only two things, using
 * Samsung's own M52 table (DTBO cable-info):
 *   SDP (PC port)   500 mA input /  500 mA charge
 *   CDP            1500 mA input / 1500 mA charge
 *   DCP (adapter)  1800 mA input / 2100 mA charge
 * and applies Samsung's battery temperature zones (sec_battery "swelling"
 * protection, thresholds/currents from the same DTBO):
 *   < 0 degC     charging off
 *   0 .. 5       450 mA
 *   5 .. 15     1150 mA
 *   15 .. 18    2900 mA (capped by the adapter table)
 *   18 .. 42    adapter table
 *   42 .. 50    1700 mA, float voltage 4.20 V
 *   >= 50       charging off
 * The normal float voltage is whatever the chip was set to at probe.
 *
 * Samsung AFC: a DCP that the MUIC flags as HVDCP is asked for 9 V (Samsung
 * sm5714_muic_voltage_control() sequence); only if the adapter accepts and the
 * measured VBUS really is 7..10 V, the input goes to 1650 mA / charge 2900 mA
 * (Samsung HV table). 9 V is used only in the NORMAL zone -- stricter than
 * stock, whose charger-thermistor limits cannot be read reliably here -- and is
 * dropped back to 5 V on leaving it, on driver removal and at shutdown.
 * QC, USB PD and the PCA9468 direct charger are left untouched.
 *
 * USB OTG: an optional "usb-otg-vbus" child node registers a regulator that
 * puts the chip into Samsung's OP_MODE_USB_OTG (5.1 V boost, 900 mA limit) for
 * the Type-C port controller (sm5714-usbpd) when a USB device is attached.
 * With no adapter VBUS the MUIC never closes its D+/D- switch on its own (it
 * only does so after BC1.2), so while OTG is on the switch is held on USB
 * manually, as Samsung's MANSW_OTG does, and given back to automatic after.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/workqueue.h>

#define SM5714_CHG_REG_STATUS1		0x0d
#define SM5714_CHG_REG_CNTL1		0x13
#define SM5714_CHG_REG_VBUSCNTL		0x15
#define SM5714_CHG_REG_CHGCNTL2		0x18
#define SM5714_CHG_REG_CHGCNTL4		0x1a
#define SM5714_CHG_REG_DEVICEID		0x50

#define SM5714_CNTL1_ENQ4FET		BIT(3)	/* charging enable */
#define SM5714_CHGCNTL4_BATREG_MASK	GENMASK(5, 0)
#define SM5714_BATREG_4200MV		(6 + (4200 - 4050) / 10)

#define SM5714_STATUS1_VBUSPOK		BIT(0)
#define SM5714_VBUSCNTL_LIMIT_MASK	GENMASK(6, 0)

#define SM5714_MUIC_I2C_ADDR		(0x4a >> 1)
#define SM5714_MUIC_REG_INT2		0x02	/* read-clear */
#define SM5714_MUIC_REG_INTMASK2	0x04
#define SM5714_MUIC_REG_MANUAL_SW	0x06	/* [7] manual, [5:3] DM, [2:0] DP */
#define SM5714_MUIC_MANSW_OTG		0x89	/* manual, DM/DP to USB (Samsung MANSW_OTG) */
#define SM5714_MUIC_MANSW_AUTO		0x00
#define SM5714_MUIC_REG_DEVICETYPE1	0x07
#define SM5714_MUIC_REG_DEVICETYPE2	0x08
#define SM5714_MUIC_REG_AFCCNTL		0x09
#define SM5714_MUIC_REG_AFCTXD		0x0a
#define SM5714_MUIC_REG_AFCSTATUS	0x0b	/* read-clear */
#define SM5714_MUIC_REG_VBUS_VOLTAGE	0x0c	/* 100 mV units */
#define SM5714_MUIC_REG_VBUS		0x3b
#define SM5714_MUIC_DEV1_SDP		BIT(1)
#define SM5714_MUIC_DEV1_DCP		BIT(2)
#define SM5714_MUIC_DEV1_CDP		BIT(3)
#define SM5714_MUIC_DEV1_AFC_TA		BIT(5)
#define SM5714_MUIC_DEV2_HVDCP		BIT(6)
#define SM5714_MUIC_INT2_AFC_ACCEPTED	BIT(1)
#define SM5714_MUIC_INT2_VBUS_UPDATE	BIT(2)
#define SM5714_MUIC_INT2_AFC_ERROR	BIT(5)
#define SM5714_MUIC_INT2_AFC_MASK	GENMASK(5, 0)
#define SM5714_MUIC_AFCCNTL_ENAFC	BIT(0)
#define SM5714_MUIC_AFCCNTL_DP_RESET	BIT(1)
#define SM5714_MUIC_AFCCNTL_VBUS_READ	BIT(3)
#define SM5714_MUIC_VBUS_VALID		BIT(2)
/* AFCTXD: high nibble = volts - 5, low nibble = current step */
#define SM5714_AFC_TXD_5V_1950MA	0x08
#define SM5714_AFC_TXD_9V_1650MA	0x46

/* flash LED block of the same chip (Samsung leds-sm5714-fled.c / sm5714_charger_oper.c) */
#define SM5714_CHG_REG_CNTL2		0x14
#define SM5714_CNTL2_OPMODE_MASK	GENMASK(3, 0)
#define SM5714_OPMODE_CHG_ON_VBUS	0x5
#define SM5714_OPMODE_USB_OTG		0x7
#define SM5714_OPMODE_FLASH_BOOST	0x8
#define SM5714_CHG_REG_BSTCNTL1		0x23
#define SM5714_BSTCNTL1_TORCH_MASK	(GENMASK(7, 6) | GENMASK(3, 0))
#define SM5714_BSTCNTL1_TORCH		((1 << 6) | 0x6)	/* OTG 900 mA, boost 5.1 V */
#define SM5714_BSTCNTL1_OTG		SM5714_BSTCNTL1_TORCH	/* same in Samsung's op-mode table */
#define SM5714_CHG_REG_FLEDCNTL1	0x41
#define SM5714_FLEDCNTL1_MODE_MASK	GENMASK(1, 0)
#define SM5714_FLED_MODE_TORCH		0x1
#define SM5714_CHG_REG_FLEDCNTL2	0x42
#define SM5714_FLEDCNTL2_MLED_MASK	GENMASK(6, 4)
#define SM5714_TORCH_MAX_BRIGHTNESS	8		/* MLED current code 0..7, stock torch = 7 */

#define SM5714_AFC_INPUT_MA		1650
#define SM5714_AFC_CHARGE_MA		2900
#define SM5714_AFC_PRE_INPUT_MA		500	/* Samsung pre_afc_input_current */

#define SM5714_CHG_POLL_INTERVAL	(5 * HZ)

/* Samsung M52 zones, battery temperature in 0.1 degC */
enum sm5714_zone {
	SM5714_ZONE_COLD,	/* < 0: off */
	SM5714_ZONE_COOL3,	/* 0 .. 5 */
	SM5714_ZONE_COOL2,	/* 5 .. 15 */
	SM5714_ZONE_COOL1,	/* 15 .. 18 */
	SM5714_ZONE_NORMAL,	/* 18 .. 42 */
	SM5714_ZONE_WARM,	/* 42 .. 50 */
	SM5714_ZONE_OVERHEAT,	/* >= 50: off */
};

static const int sm5714_zone_lower[] = {
	[SM5714_ZONE_COOL3]	= 0,
	[SM5714_ZONE_COOL2]	= 50,
	[SM5714_ZONE_COOL1]	= 150,
	[SM5714_ZONE_NORMAL]	= 180,
	[SM5714_ZONE_WARM]	= 420,
	[SM5714_ZONE_OVERHEAT]	= 500,
};

static const int sm5714_zone_current[] = {
	[SM5714_ZONE_COOL3]	= 450,
	[SM5714_ZONE_COOL2]	= 1150,
	[SM5714_ZONE_COOL1]	= 2900,
	[SM5714_ZONE_NORMAL]	= INT_MAX,
	[SM5714_ZONE_WARM]	= 1700,
};

#define SM5714_ZONE_HYST		19	/* sec_battery THERMAL_HYSTERESIS_2 */

struct sm5714_chg {
	struct i2c_client *client;
	struct i2c_client *muic;
	struct power_supply *psy;
	struct delayed_work work;
	enum power_supply_usb_type usb_type;
	enum sm5714_zone zone;
	int normal_batreg;
	int batreg;
	int enabled;
	bool online;
	int input_ma;
	int charge_ma;
	int afc_mv;		/* 0: adapter at 5 V */
	bool afc_failed;	/* no new 9 V attempt until unplugged */
	struct led_classdev torch;
	struct mutex torch_lock;
	int torch_level;	/* 0: off */
	bool torch_boost;	/* chip in FLASH_BOOST for the torch (no VBUS) */
	int bstcntl1_saved;
	bool otg_on;		/* USB OTG boost requested (VBUS out to a device) */
	int opmode;		/* op-mode last written to CNTL2, -1: not yet */
	struct regulator_dev *otg_rdev;
};

static int sm5714_chg_set_input(struct sm5714_chg *chg, int ma)
{
	int reg;
	u8 val = ma < 100 ? 0 : ((ma - 100) / 25) & SM5714_VBUSCNTL_LIMIT_MASK;

	reg = i2c_smbus_read_byte_data(chg->client, SM5714_CHG_REG_VBUSCNTL);
	if (reg < 0)
		return reg;
	reg = (reg & ~SM5714_VBUSCNTL_LIMIT_MASK) | val;
	return i2c_smbus_write_byte_data(chg->client, SM5714_CHG_REG_VBUSCNTL, reg);
}

static int sm5714_chg_set_charge(struct sm5714_chg *chg, int ma)
{
	int ua = ma * 1000;
	u8 val;

	if (ua < 109375)
		val = 0x07;
	else
		val = min((ua - 109375) / 15625 + 7, 0xdf);
	return i2c_smbus_write_byte_data(chg->client, SM5714_CHG_REG_CHGCNTL2, val);
}

/* battery temperature in 0.1 degC (battery NTC, read by the fuel gauge driver) */
static int sm5714_chg_battery_temp(int *temp)
{
	union power_supply_propval val;
	struct power_supply *bat;
	int ret;

	bat = power_supply_get_by_name("battery");
	if (!bat)
		return -ENODEV;
	ret = power_supply_get_property(bat, POWER_SUPPLY_PROP_TEMP, &val);
	power_supply_put(bat);
	if (!ret)
		*temp = val.intval;

	return ret;
}

static enum sm5714_zone sm5714_chg_zone_for(int temp)
{
	enum sm5714_zone z;

	if (temp < sm5714_zone_lower[SM5714_ZONE_COOL3])
		return SM5714_ZONE_COLD;
	for (z = SM5714_ZONE_OVERHEAT; z > SM5714_ZONE_COOL3; z--)
		if (temp >= sm5714_zone_lower[z])
			return z;
	return SM5714_ZONE_COOL3;
}

static void sm5714_chg_update_zone(struct sm5714_chg *chg)
{
	enum sm5714_zone z, cur = chg->zone;
	int temp;

	if (sm5714_chg_battery_temp(&temp))	/* no gauge yet: keep the last zone */
		return;

	z = sm5714_chg_zone_for(temp);
	/*
	 * A stricter zone applies at once; going back towards NORMAL waits until
	 * the temperature is 1.9 degC past the edge of the current zone.
	 */
	if (cur > SM5714_ZONE_NORMAL && z < cur &&
	    temp >= sm5714_zone_lower[cur] - SM5714_ZONE_HYST)
		z = cur;
	if (cur < SM5714_ZONE_NORMAL && z > cur &&
	    temp < sm5714_zone_lower[cur + 1] + SM5714_ZONE_HYST)
		z = cur;
	if (z != chg->zone) {
		dev_info(&chg->client->dev, "battery %d.%d degC: zone %d -> %d\n",
			 temp / 10, abs(temp % 10), chg->zone, z);
		chg->zone = z;
		power_supply_changed(chg->psy);
	}
}

static int sm5714_chg_set_batreg(struct sm5714_chg *chg, int batreg)
{
	int reg = i2c_smbus_read_byte_data(chg->client, SM5714_CHG_REG_CHGCNTL4);

	if (reg < 0)
		return reg;
	reg = (reg & ~SM5714_CHGCNTL4_BATREG_MASK) | batreg;
	return i2c_smbus_write_byte_data(chg->client, SM5714_CHG_REG_CHGCNTL4, reg);
}

/* Samsung chg_set_enq4fet(): enable with the input limit briefly at 500 mA */
static int sm5714_chg_set_enable(struct sm5714_chg *chg, bool enable)
{
	int reg = i2c_smbus_read_byte_data(chg->client, SM5714_CHG_REG_CNTL1);
	int ret;

	if (reg < 0)
		return reg;
	if (!enable)
		return i2c_smbus_write_byte_data(chg->client, SM5714_CHG_REG_CNTL1,
						 reg & ~SM5714_CNTL1_ENQ4FET);

	if (chg->input_ma > 500)
		sm5714_chg_set_input(chg, 500);
	ret = i2c_smbus_write_byte_data(chg->client, SM5714_CHG_REG_CNTL1,
					reg | SM5714_CNTL1_ENQ4FET);
	if (chg->input_ma > 500) {
		msleep((chg->input_ma - 500) / 250 + 1);
		sm5714_chg_set_input(chg, chg->input_ma);
	}
	return ret;
}

static int sm5714_muic_update(struct sm5714_chg *chg, u8 reg, u8 mask, u8 val)
{
	int old = i2c_smbus_read_byte_data(chg->muic, reg);

	if (old < 0)
		return old;
	if ((old & mask) == val)
		return 0;
	return i2c_smbus_write_byte_data(chg->muic, reg, (old & ~mask) | val);
}

/* Samsung sm5714_muic_get_vbus(): VBUS in mV, 0 when there is none */
static int sm5714_afc_vbus_mv(struct sm5714_chg *chg)
{
	int i, v;

	v = i2c_smbus_read_byte_data(chg->muic, SM5714_MUIC_REG_VBUS);
	if (v <= 0 || !(v & SM5714_MUIC_VBUS_VALID))
		return v;

	sm5714_muic_update(chg, SM5714_MUIC_REG_AFCCNTL, SM5714_MUIC_AFCCNTL_VBUS_READ,
			   SM5714_MUIC_AFCCNTL_VBUS_READ);
	for (i = 0; i < 5; i++) {
		usleep_range(5000, 5100);
		v = i2c_smbus_read_byte_data(chg->muic, SM5714_MUIC_REG_INT2);
		if (v >= 0 && (v & SM5714_MUIC_INT2_VBUS_UPDATE))
			break;
	}
	sm5714_muic_update(chg, SM5714_MUIC_REG_AFCCNTL, SM5714_MUIC_AFCCNTL_VBUS_READ, 0);

	v = i2c_smbus_read_byte_data(chg->muic, SM5714_MUIC_REG_VBUS_VOLTAGE);
	return v < 0 ? v : v * 100;
}

/*
 * Samsung sm5714_muic_voltage_control(): ask the AFC adapter for @mv (5000 or
 * 9000) and check that VBUS really moved there. The MUIC interrupt line is not
 * used, so the AFC interrupts stay masked and INT2 is polled.
 */
static int sm5714_afc_request(struct sm5714_chg *chg, int mv)
{
	u8 txd = mv == 9000 ? SM5714_AFC_TXD_9V_1650MA : SM5714_AFC_TXD_5V_1950MA;
	int mask, irq, i, errors = 0, vbus = 0, ret = -ETIMEDOUT;

	mask = i2c_smbus_read_byte_data(chg->muic, SM5714_MUIC_REG_INTMASK2);
	if (mask < 0)
		return mask;
	i2c_smbus_write_byte_data(chg->muic, SM5714_MUIC_REG_INTMASK2,
				  mask | SM5714_MUIC_INT2_AFC_MASK);
	i2c_smbus_read_byte_data(chg->muic, SM5714_MUIC_REG_INT2);
	i2c_smbus_read_byte_data(chg->muic, SM5714_MUIC_REG_AFCSTATUS);

	i2c_smbus_write_byte_data(chg->muic, SM5714_MUIC_REG_AFCTXD, txd);
	sm5714_muic_update(chg, SM5714_MUIC_REG_AFCCNTL, SM5714_MUIC_AFCCNTL_ENAFC,
			   SM5714_MUIC_AFCCNTL_ENAFC);
	for (i = 0; i < 10; i++) {
		msleep(50);
		irq = i2c_smbus_read_byte_data(chg->muic, SM5714_MUIC_REG_INT2);
		if (irq < 0)
			continue;
		if (irq & SM5714_MUIC_INT2_AFC_ACCEPTED) {
			ret = 0;
			break;
		}
		if (irq & SM5714_MUIC_INT2_AFC_ERROR) {
			if (errors++ >= 2) {
				ret = -EIO;
				break;
			}
			sm5714_muic_update(chg, SM5714_MUIC_REG_AFCCNTL,
					   SM5714_MUIC_AFCCNTL_ENAFC, SM5714_MUIC_AFCCNTL_ENAFC);
		}
	}
	sm5714_muic_update(chg, SM5714_MUIC_REG_AFCCNTL, SM5714_MUIC_AFCCNTL_ENAFC, 0);

	if (!ret) {
		ret = -ERANGE;
		for (i = 0; i < 5; i++) {
			vbus = sm5714_afc_vbus_mv(chg);
			if (vbus >= mv - 2000 && vbus <= mv + 1000) {
				ret = 0;
				break;
			}
			msleep(100);
		}
	}
	i2c_smbus_write_byte_data(chg->muic, SM5714_MUIC_REG_INTMASK2, mask);

	dev_info(&chg->client->dev, "AFC %d mV: %s, VBUS %d mV\n", mv,
		 ret == -ERANGE ? "VBUS out of range" : ret ? "not accepted" : "ok", vbus);
	return ret;
}

static int sm5714_chg_set_input_cached(struct sm5714_chg *chg, int ma)
{
	int ret = sm5714_chg_set_input(chg, ma);

	if (!ret)
		chg->input_ma = ma;
	return ret;
}

/* back to 5 V with the input limited first; DP reset if the adapter ignores it */
static void sm5714_afc_to_5v(struct sm5714_chg *chg)
{
	sm5714_chg_set_input_cached(chg, SM5714_AFC_PRE_INPUT_MA);
	if (sm5714_afc_request(chg, 5000))
		sm5714_muic_update(chg, SM5714_MUIC_REG_AFCCNTL, SM5714_MUIC_AFCCNTL_DP_RESET,
				   SM5714_MUIC_AFCCNTL_DP_RESET);
	chg->afc_mv = 0;
}

static void sm5714_afc_update(struct sm5714_chg *chg, enum power_supply_usb_type prev)
{
	bool hv_allowed = chg->zone == SM5714_ZONE_NORMAL;
	int dev2;

	if (chg->afc_mv) {
		if (!hv_allowed) {
			dev_info(&chg->client->dev, "zone %d: AFC back to 5 V\n", chg->zone);
			sm5714_afc_to_5v(chg);
		}
		return;
	}

	/* one poll after the DCP attach, like Samsung's pre_afc_work_delay */
	if (!hv_allowed || chg->afc_failed || prev != POWER_SUPPLY_USB_TYPE_DCP)
		return;
	dev2 = i2c_smbus_read_byte_data(chg->muic, SM5714_MUIC_REG_DEVICETYPE2);
	if (dev2 < 0 || !(dev2 & SM5714_MUIC_DEV2_HVDCP))
		return;

	sm5714_chg_set_input_cached(chg, SM5714_AFC_PRE_INPUT_MA);
	switch (sm5714_afc_request(chg, 9000)) {
	case 0:
		chg->afc_mv = 9000;
		break;
	case -ERANGE:	/* accepted but VBUS did not follow: make sure it is 5 V */
		sm5714_afc_to_5v(chg);
		fallthrough;
	default:
		chg->afc_failed = true;
	}
}

static int sm5714_chg_update(struct sm5714_chg *chg, u8 reg, u8 mask, u8 val)
{
	int old = i2c_smbus_read_byte_data(chg->client, reg);

	if (old < 0)
		return old;
	if ((old & mask) == val)
		return 0;
	return i2c_smbus_write_byte_data(chg->client, reg, (old & ~mask) | val);
}

/*
 * Charger op-mode as in Samsung's sm5714_charger_oper table: USB_OTG while a
 * device takes VBUS from us, FLASH_BOOST for the torch on battery, otherwise
 * CHG_ON_VBUS. Both boost modes run at 5.1 V / 900 mA (BSTCNTL1); the value
 * the chip had before is put back when no boost is needed any more.
 * Caller holds torch_lock.
 */
static void sm5714_opmode_apply(struct sm5714_chg *chg)
{
	int mode = chg->otg_on ? SM5714_OPMODE_USB_OTG :
		   chg->torch_boost ? SM5714_OPMODE_FLASH_BOOST : SM5714_OPMODE_CHG_ON_VBUS;
	bool was_boost = chg->opmode == SM5714_OPMODE_USB_OTG ||
			 chg->opmode == SM5714_OPMODE_FLASH_BOOST;
	bool boost = mode != SM5714_OPMODE_CHG_ON_VBUS;

	if (mode == chg->opmode)
		return;
	if (boost && !was_boost) {
		chg->bstcntl1_saved = i2c_smbus_read_byte_data(chg->client,
							       SM5714_CHG_REG_BSTCNTL1);
		sm5714_chg_update(chg, SM5714_CHG_REG_BSTCNTL1, SM5714_BSTCNTL1_TORCH_MASK,
				  SM5714_BSTCNTL1_OTG);
	}
	sm5714_chg_update(chg, SM5714_CHG_REG_CNTL2, SM5714_CNTL2_OPMODE_MASK, mode);
	if (!boost && was_boost && chg->bstcntl1_saved >= 0)
		i2c_smbus_write_byte_data(chg->client, SM5714_CHG_REG_BSTCNTL1,
					  chg->bstcntl1_saved);
	if (mode == SM5714_OPMODE_USB_OTG || chg->opmode == SM5714_OPMODE_USB_OTG)
		dev_info(&chg->client->dev, "op-mode 0x%x%s\n", mode,
			 mode == SM5714_OPMODE_USB_OTG ? " (USB OTG, VBUS 5.1 V)" : "");
	chg->opmode = mode;
}

/*
 * Torch as in stock: MLED current in FLEDCNTL2, FLED mode TORCH in FLEDCNTL1.
 * With VBUS the chip stays in CHG_ON_VBUS; on battery it needs FLASH_BOOST
 * (5.1 V, 900 mA) -- Samsung's charger op-mode table. Caller holds torch_lock.
 */
static void sm5714_torch_apply(struct sm5714_chg *chg)
{
	bool boost = chg->torch_level && !chg->online;

	if (!chg->torch_level)
		sm5714_chg_update(chg, SM5714_CHG_REG_FLEDCNTL1, SM5714_FLEDCNTL1_MODE_MASK, 0);
	else
		sm5714_chg_update(chg, SM5714_CHG_REG_FLEDCNTL2, SM5714_FLEDCNTL2_MLED_MASK,
				  (chg->torch_level - 1) << 4);

	chg->torch_boost = boost;
	sm5714_opmode_apply(chg);

	if (chg->torch_level)
		sm5714_chg_update(chg, SM5714_CHG_REG_FLEDCNTL1, SM5714_FLEDCNTL1_MODE_MASK,
				  SM5714_FLED_MODE_TORCH);
}

static int sm5714_torch_set(struct led_classdev *cdev, enum led_brightness brightness)
{
	struct sm5714_chg *chg = container_of(cdev, struct sm5714_chg, torch);

	mutex_lock(&chg->torch_lock);
	chg->torch_level = brightness;
	sm5714_torch_apply(chg);
	mutex_unlock(&chg->torch_lock);

	return 0;
}

static void sm5714_chg_work(struct work_struct *work)
{
	struct sm5714_chg *chg = container_of(work, struct sm5714_chg, work.work);
	enum power_supply_usb_type type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	int st1, dev1, input_ma = 500, charge_ma = 500, batreg;
	int afc_mv = chg->afc_mv;
	bool online, changed, enable;

	st1 = i2c_smbus_read_byte_data(chg->client, SM5714_CHG_REG_STATUS1);
	dev1 = i2c_smbus_read_byte_data(chg->muic, SM5714_MUIC_REG_DEVICETYPE1);
	if (st1 < 0 || dev1 < 0)
		goto out;

	/* with the OTG boost on, VBUSPOK is our own 5 V, not an adapter */
	online = (st1 & SM5714_STATUS1_VBUSPOK) && !READ_ONCE(chg->otg_on);
	if (online) {
		if (dev1 & (SM5714_MUIC_DEV1_DCP | SM5714_MUIC_DEV1_AFC_TA)) {
			type = POWER_SUPPLY_USB_TYPE_DCP;
			input_ma = 1800;
			charge_ma = 2100;
		} else if (dev1 & SM5714_MUIC_DEV1_CDP) {
			type = POWER_SUPPLY_USB_TYPE_CDP;
			input_ma = 1500;
			charge_ma = 1500;
		} else if (dev1 & SM5714_MUIC_DEV1_SDP) {
			type = POWER_SUPPLY_USB_TYPE_SDP;
		}

		sm5714_chg_update_zone(chg);
		if (type == POWER_SUPPLY_USB_TYPE_DCP)
			sm5714_afc_update(chg, chg->usb_type);
		if (chg->afc_mv) {
			input_ma = SM5714_AFC_INPUT_MA;
			charge_ma = SM5714_AFC_CHARGE_MA;
		}
		if (chg->zone != SM5714_ZONE_COLD && chg->zone != SM5714_ZONE_OVERHEAT)
			charge_ma = min(charge_ma, sm5714_zone_current[chg->zone]);
	} else {
		/* the adapter drops back to 5 V by itself when unplugged */
		chg->afc_mv = 0;
		chg->afc_failed = false;
	}

	/* unplugged: fall back to the safe 500/500 for the next attach */
	if (input_ma != chg->input_ma)
		sm5714_chg_set_input_cached(chg, input_ma);
	if (charge_ma != chg->charge_ma && !sm5714_chg_set_charge(chg, charge_ma))
		chg->charge_ma = charge_ma;

	batreg = (online && chg->zone == SM5714_ZONE_WARM) ?
		 min(chg->normal_batreg, SM5714_BATREG_4200MV) : chg->normal_batreg;
	if (batreg != chg->batreg && !sm5714_chg_set_batreg(chg, batreg))
		chg->batreg = batreg;

	enable = !online || (chg->zone != SM5714_ZONE_COLD && chg->zone != SM5714_ZONE_OVERHEAT);
	if (enable != chg->enabled && !sm5714_chg_set_enable(chg, enable))
		chg->enabled = enable;

	changed = online != chg->online || type != chg->usb_type || afc_mv != chg->afc_mv;
	if (online != chg->online) {
		/* the torch needs FLASH_BOOST only while there is no VBUS */
		mutex_lock(&chg->torch_lock);
		chg->online = online;
		if (chg->torch_level || chg->torch_boost)
			sm5714_torch_apply(chg);
		mutex_unlock(&chg->torch_lock);
	}
	chg->usb_type = type;
	if (changed) {
		dev_info(&chg->client->dev, "%s, type %d%s: input %d mA, charge %d mA\n",
			 online ? "online" : "offline", type, chg->afc_mv ? " AFC 9 V" : "",
			 chg->input_ma, chg->charge_ma);
		power_supply_changed(chg->psy);
	}
out:
	schedule_delayed_work(&chg->work, SM5714_CHG_POLL_INTERVAL);
}

static int sm5714_chg_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct sm5714_chg *chg = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chg->online;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = chg->usb_type;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (chg->zone == SM5714_ZONE_OVERHEAT)
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if (chg->zone == SM5714_ZONE_COLD)
			val->intval = POWER_SUPPLY_HEALTH_COLD;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = chg->input_ma * 1000;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		val->intval = chg->charge_ma * 1000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property sm5714_chg_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
};

static const struct power_supply_desc sm5714_chg_desc = {
	.name		= "sm5714-charger",
	.type		= POWER_SUPPLY_TYPE_USB,
	.usb_types	= BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
			  BIT(POWER_SUPPLY_USB_TYPE_SDP) |
			  BIT(POWER_SUPPLY_USB_TYPE_CDP) |
			  BIT(POWER_SUPPLY_USB_TYPE_DCP),
	.properties	= sm5714_chg_props,
	.num_properties	= ARRAY_SIZE(sm5714_chg_props),
	.get_property	= sm5714_chg_get_property,
};

/* stop polling and leave the adapter at 5 V (removal, reboot, power-off) */
static void sm5714_chg_stop(void *data)
{
	struct sm5714_chg *chg = data;

	cancel_delayed_work_sync(&chg->work);
	mutex_lock(&chg->torch_lock);
	chg->torch_level = 0;
	if (chg->otg_on)
		i2c_smbus_write_byte_data(chg->muic, SM5714_MUIC_REG_MANUAL_SW,
					  SM5714_MUIC_MANSW_AUTO);
	WRITE_ONCE(chg->otg_on, false);
	sm5714_torch_apply(chg);
	mutex_unlock(&chg->torch_lock);
	if (chg->afc_mv)
		sm5714_afc_to_5v(chg);
}

static void sm5714_chg_shutdown(struct i2c_client *client)
{
	struct sm5714_chg *chg = i2c_get_clientdata(client);

	if (chg)
		sm5714_chg_stop(chg);
}

static int sm5714_otg_set(struct regulator_dev *rdev, bool on)
{
	struct sm5714_chg *chg = rdev_get_drvdata(rdev);

	mutex_lock(&chg->torch_lock);
	WRITE_ONCE(chg->otg_on, on);
	if (on)
		chg->online = false;	/* no adapter while we source VBUS */
	sm5714_opmode_apply(chg);
	i2c_smbus_write_byte_data(chg->muic, SM5714_MUIC_REG_MANUAL_SW,
				  on ? SM5714_MUIC_MANSW_OTG : SM5714_MUIC_MANSW_AUTO);
	mutex_unlock(&chg->torch_lock);
	/* let the poll see the new state (input limits, torch boost) soon */
	mod_delayed_work(system_wq, &chg->work, HZ / 2);

	return 0;
}

static int sm5714_otg_enable(struct regulator_dev *rdev)
{
	return sm5714_otg_set(rdev, true);
}

static int sm5714_otg_disable(struct regulator_dev *rdev)
{
	return sm5714_otg_set(rdev, false);
}

static int sm5714_otg_is_enabled(struct regulator_dev *rdev)
{
	struct sm5714_chg *chg = rdev_get_drvdata(rdev);

	return READ_ONCE(chg->otg_on);
}

static const struct regulator_ops sm5714_otg_ops = {
	.enable		= sm5714_otg_enable,
	.disable	= sm5714_otg_disable,
	.is_enabled	= sm5714_otg_is_enabled,
};

static const struct regulator_desc sm5714_otg_desc = {
	.name		= "sm5714-otg-vbus",
	.type		= REGULATOR_VOLTAGE,
	.owner		= THIS_MODULE,
	.ops		= &sm5714_otg_ops,
	.fixed_uV	= 5100000,
	.n_voltages	= 1,
};

static int sm5714_otg_register(struct sm5714_chg *chg)
{
	struct device *dev = &chg->client->dev;
	struct regulator_config rcfg = {};
	struct device_node *np;

	np = of_get_child_by_name(dev->of_node, "usb-otg-vbus");
	if (!np)
		return 0;	/* no OTG described: torch/charging only */

	rcfg.dev = dev;
	rcfg.driver_data = chg;
	rcfg.of_node = np;
	rcfg.init_data = of_get_regulator_init_data(dev, np, &sm5714_otg_desc);
	chg->otg_rdev = devm_regulator_register(dev, &sm5714_otg_desc, &rcfg);
	of_node_put(np);
	if (IS_ERR(chg->otg_rdev))
		return dev_err_probe(dev, PTR_ERR(chg->otg_rdev), "failed to register OTG VBUS\n");

	return 0;
}

static int sm5714_chg_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config cfg = {};
	struct sm5714_chg *chg;
	int id, ret;

	id = i2c_smbus_read_byte_data(client, SM5714_CHG_REG_DEVICEID);
	if (id < 0)
		return dev_err_probe(dev, id, "failed to read device id\n");

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->client = client;
	chg->input_ma = -1;
	chg->charge_ma = -1;
	chg->enabled = -1;
	chg->zone = SM5714_ZONE_NORMAL;
	chg->usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;

	ret = i2c_smbus_read_byte_data(client, SM5714_CHG_REG_CHGCNTL4);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to read float voltage\n");
	chg->normal_batreg = ret & SM5714_CHGCNTL4_BATREG_MASK;
	chg->batreg = chg->normal_batreg;

	chg->muic = devm_i2c_new_dummy_device(dev, client->adapter, SM5714_MUIC_I2C_ADDR);
	if (IS_ERR(chg->muic))
		return dev_err_probe(dev, PTR_ERR(chg->muic), "failed to add MUIC client\n");

	cfg.drv_data = chg;
	cfg.fwnode = dev_fwnode(dev);
	chg->psy = devm_power_supply_register(dev, &sm5714_chg_desc, &cfg);
	if (IS_ERR(chg->psy))
		return dev_err_probe(dev, PTR_ERR(chg->psy), "failed to register charger\n");

	mutex_init(&chg->torch_lock);
	chg->bstcntl1_saved = -1;
	chg->opmode = -1;
	chg->torch.name = "white:flash";
	chg->torch.max_brightness = SM5714_TORCH_MAX_BRIGHTNESS;
	chg->torch.brightness_set_blocking = sm5714_torch_set;
	ret = devm_led_classdev_register(dev, &chg->torch);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register torch\n");

	INIT_DELAYED_WORK(&chg->work, sm5714_chg_work);
	ret = devm_add_action_or_reset(dev, sm5714_chg_stop, chg);
	if (ret)
		return ret;

	ret = sm5714_otg_register(chg);
	if (ret)
		return ret;
	i2c_set_clientdata(client, chg);
	schedule_delayed_work(&chg->work, 0);

	dev_info(dev, "SM5714 charger, device id 0x%02x\n", id);

	return 0;
}

static const struct of_device_id sm5714_chg_of_match[] = {
	{ .compatible = "siliconmitus,sm5714-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5714_chg_of_match);

static struct i2c_driver sm5714_chg_driver = {
	.driver = {
		.name = "sm5714-charger",
		.of_match_table = sm5714_chg_of_match,
	},
	.probe = sm5714_chg_probe,
	.shutdown = sm5714_chg_shutdown,
};
module_i2c_driver(sm5714_chg_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5714 charger current control");
MODULE_LICENSE("GPL");
