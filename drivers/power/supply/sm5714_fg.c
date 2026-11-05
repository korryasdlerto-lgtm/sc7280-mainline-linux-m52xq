// SPDX-License-Identifier: GPL-2.0-only
/*
 * Read-only driver for the fuel gauge part of the Silicon Mitus SM5714
 * (Samsung Galaxy M52 5G). Every value is read from the gauge SRAM: the SRAM
 * address is written to RADDR, then the 16-bit word is read from RDATA.
 * Nothing else is ever written -- charging stays under hardware control.
 * Conversions follow Samsung's sm5714_fuelgauge.c.
 *
 * The gauge's own temperature reading jumps by ~3.5 degC with the charge
 * current, so like the stock sec_battery (thermal_source = ADC) the battery
 * temperature comes from the battery NTC on the PMIC ADC when io-channels
 * points at it, converted with Samsung's M52 temp_table_adc/temp_table_data.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iio/consumer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>

#define SM5714_FG_REG_DEVICE_ID		0x00
#define SM5714_FG_REG_SYSTEM_STATUS	0x10
#define SM5714_FG_REG_AUX_STAT		0x22
#define SM5714_FG_REG_SRAM_RADDR	0x8c
#define SM5714_FG_REG_SRAM_RDATA	0x8d

#define SM5714_FG_SRAM_SOC		0x00
#define SM5714_FG_SRAM_OCV		0x01
#define SM5714_FG_SRAM_VBAT		0x03
#define SM5714_FG_SRAM_CURRENT		0x05
#define SM5714_FG_SRAM_TEMPERATURE	0x07
#define SM5714_FG_SRAM_VBAT_AVG		0x08
#define SM5714_FG_SRAM_CURRENT_AVG	0x09

#define SM5714_AUX_STAT_CHG_ON		BIT(5)
#define SM5714_AUX_STAT_CURR_X2_5	BIT(10)

#define SM5714_FG_POLL_INTERVAL		(60 * HZ)

/* PM7325 ADC7 GPIO2_100K_PU: Samsung's "adc-temp" battery NTC */
#define SM5714_FG_NTC_ADC_CHANNEL	0x14b
#define SM5714_FG_NTC_SAMPLES		5	/* sec_battery adc_check_count */

/*
 * Samsung's M52 temp_table_adc/temp_table_data mapped onto the temperature the
 * mainline ADC7 driver reports for the same ADC code (100k NTC, 100k pull-up):
 * { mainline millidegC, Samsung 0.1 degC }.
 */
static const int sm5714_fg_ntc_table[][2] = {
	{ -19638, -200 }, { -14589, -150 }, {  -9587, -100 }, {  -4656,  -50 },
	{    319,    0 }, {   4502,   50 }, {   9403,  100 }, {  14228,  150 },
	{  19132,  200 }, {  23919,  250 }, {  28742,  300 }, {  33449,  350 },
	{  39241,  400 }, {  44659,  450 }, {  49903,  500 }, {  54928,  550 },
	{  59750,  600 }, {  64934,  650 }, {  70376,  700 }, {  75736,  750 },
	{  80928,  800 }, {  86135,  850 }, {  91292,  900 },
};

struct sm5714_fg {
	struct i2c_client *client;
	struct power_supply *psy;
	struct iio_channel *ntc;	/* NULL: use the gauge temperature */
	struct mutex lock;
	struct delayed_work work;
};

static int sm5714_fg_read_sram(struct sm5714_fg *fg, u8 addr)
{
	int ret;

	mutex_lock(&fg->lock);
	ret = i2c_smbus_write_word_data(fg->client, SM5714_FG_REG_SRAM_RADDR, addr);
	if (!ret)
		ret = i2c_smbus_read_word_data(fg->client, SM5714_FG_REG_SRAM_RDATA);
	mutex_unlock(&fg->lock);

	return ret;
}

/* mV */
static int sm5714_fg_raw_to_mv(int raw)
{
	if (raw & 0x8000)
		return 2700 - ((raw & 0x7fff) * 10) / 109;
	return (raw * 10) / 109 + 2700;
}

/* mA, positive while charging */
static int sm5714_fg_raw_to_ma(int raw, int aux)
{
	int ma = ((raw & 0x7fff) * 1000) / 2044;

	if (raw & 0x8000)
		ma = -ma;
	if (aux & SM5714_AUX_STAT_CURR_X2_5)
		ma = ma * 25 / 10;

	return ma;
}

static int sm5714_fg_get_current(struct sm5714_fg *fg, u8 sram, int *ma)
{
	int raw, aux;

	raw = sm5714_fg_read_sram(fg, sram);
	if (raw < 0)
		return raw;
	aux = i2c_smbus_read_word_data(fg->client, SM5714_FG_REG_AUX_STAT);
	if (aux < 0)
		return aux;

	*ma = sm5714_fg_raw_to_ma(raw, aux);
	return 0;
}

static int sm5714_fg_ntc_to_decidegc(int mdegc)
{
	const int (*t)[2] = sm5714_fg_ntc_table;
	int i, n = ARRAY_SIZE(sm5714_fg_ntc_table);

	if (mdegc <= t[0][0])
		return t[0][1];
	for (i = 1; i < n; i++)
		if (mdegc < t[i][0])
			return t[i - 1][1] + (t[i][1] - t[i - 1][1]) *
			       (mdegc - t[i - 1][0]) / (t[i][0] - t[i - 1][0]);
	return t[n - 1][1];
}

/* 0.1 degC, mean of the samples without the lowest and the highest */
static int sm5714_fg_get_ntc_temp(struct sm5714_fg *fg, int *temp)
{
	int i, v, ret, sum = 0, lo = INT_MAX, hi = INT_MIN;

	for (i = 0; i < SM5714_FG_NTC_SAMPLES; i++) {
		ret = iio_read_channel_processed(fg->ntc, &v);
		if (ret < 0)
			return ret;
		sum += v;
		lo = min(lo, v);
		hi = max(hi, v);
	}

	*temp = sm5714_fg_ntc_to_decidegc((sum - lo - hi) / (SM5714_FG_NTC_SAMPLES - 2));
	return 0;
}

/* the charger stops by itself after top-off; the adapter is still there */
static bool sm5714_fg_adapter_online(void)
{
	union power_supply_propval val;
	struct power_supply *chg;
	int ret;

	chg = power_supply_get_by_name("sm5714-charger");
	if (!chg)
		return false;
	ret = power_supply_get_property(chg, POWER_SUPPLY_PROP_ONLINE, &val);
	power_supply_put(chg);

	return !ret && val.intval;
}

static int sm5714_fg_get_status(struct sm5714_fg *fg, int *status)
{
	int soc, aux, ma, ret;

	aux = i2c_smbus_read_word_data(fg->client, SM5714_FG_REG_AUX_STAT);
	if (aux < 0)
		return aux;
	soc = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_SOC);
	if (soc < 0)
		return soc;
	ret = sm5714_fg_get_current(fg, SM5714_FG_SRAM_CURRENT_AVG, &ma);
	if (ret)
		return ret;

	if (!(aux & SM5714_AUX_STAT_CHG_ON) && !sm5714_fg_adapter_online())
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (((soc * 10) >> 8) >= 990)
		*status = POWER_SUPPLY_STATUS_FULL;
	else if (ma > 0)
		*status = POWER_SUPPLY_STATUS_CHARGING;
	else
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	return 0;
}

static int sm5714_fg_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct sm5714_fg *fg = power_supply_get_drvdata(psy);
	int raw, ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = sm5714_fg_get_status(fg, &val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		raw = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_SOC);
		if (raw < 0)
			return raw;
		/* 0.1 % units, rounded */
		val->intval = clamp((((raw * 10) >> 8) + 5) / 10, 0, 100);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		raw = sm5714_fg_read_sram(fg, psp == POWER_SUPPLY_PROP_VOLTAGE_NOW ?
					  SM5714_FG_SRAM_VBAT : SM5714_FG_SRAM_VBAT_AVG);
		if (raw < 0)
			return raw;
		val->intval = sm5714_fg_raw_to_mv(raw) * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		raw = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_OCV);
		if (raw < 0)
			return raw;
		val->intval = ((raw * 1000) >> 11) * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = sm5714_fg_get_current(fg, psp == POWER_SUPPLY_PROP_CURRENT_NOW ?
					    SM5714_FG_SRAM_CURRENT : SM5714_FG_SRAM_CURRENT_AVG,
					    &val->intval);
		val->intval *= 1000;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (fg->ntc)
			return sm5714_fg_get_ntc_temp(fg, &val->intval);
		raw = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_TEMPERATURE);
		if (raw < 0)
			return raw;
		/* 0.1 degC */
		val->intval = (((raw & 0x7fff) * 10) * 2989) >> 19;
		if (raw & 0x8000)
			val->intval = -val->intval;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static enum power_supply_property sm5714_fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TEMP,
};

static const struct power_supply_desc sm5714_fg_desc = {
	.name		= "battery",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.properties	= sm5714_fg_props,
	.num_properties	= ARRAY_SIZE(sm5714_fg_props),
	.get_property	= sm5714_fg_get_property,
};

/* The gauge has no interrupt wired here: notify userspace periodically. */
static void sm5714_fg_work(struct work_struct *work)
{
	struct sm5714_fg *fg = container_of(work, struct sm5714_fg, work.work);

	power_supply_changed(fg->psy);
	schedule_delayed_work(&fg->work, SM5714_FG_POLL_INTERVAL);
}

static void sm5714_fg_cancel_work(void *data)
{
	cancel_delayed_work_sync(data);
}

/*
 * The PMIC ADC node comes from the Samsung DTBO overlay, so io-channels holds
 * a fixed phandle number. Check that it still resolves to the battery NTC
 * channel before using it: a stale phandle must not defer the gauge forever.
 * v253: the ADC node is "vadc" in the downstream DT and "adc@3100" in the
 * upstream-style DT; with only "vadc" accepted the gauge used its own die
 * temperature, which heats up while charging (9 V AFC flapping to 5 V).
 */
static struct iio_channel *sm5714_fg_get_ntc(struct device *dev)
{
	struct of_phandle_args args;
	struct iio_channel *ntc;
	bool ok;

	if (of_parse_phandle_with_args(dev->of_node, "io-channels",
				       "#io-channel-cells", 0, &args))
		return NULL;
	ok = (of_node_name_eq(args.np, "vadc") || of_node_name_eq(args.np, "adc")) &&
	     args.args_count == 1 &&
	     args.args[0] == SM5714_FG_NTC_ADC_CHANNEL;
	of_node_put(args.np);
	if (!ok) {
		dev_warn(dev, "io-channels is not the battery NTC, using gauge temperature\n");
		return NULL;
	}

	ntc = devm_iio_channel_get(dev, NULL);
	if (IS_ERR(ntc) && PTR_ERR(ntc) != -EPROBE_DEFER) {
		dev_warn(dev, "battery NTC unavailable (%pe), using gauge temperature\n", ntc);
		return NULL;
	}

	return ntc;
}

static int sm5714_fg_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config cfg = {};
	struct sm5714_fg *fg;
	int id, ret;

	id = i2c_smbus_read_word_data(client, SM5714_FG_REG_DEVICE_ID);
	if (id < 0)
		return dev_err_probe(dev, id, "failed to read device id\n");

	fg = devm_kzalloc(dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;

	fg->client = client;
	mutex_init(&fg->lock);

	fg->ntc = sm5714_fg_get_ntc(dev);
	if (IS_ERR(fg->ntc))
		return dev_err_probe(dev, PTR_ERR(fg->ntc), "battery NTC not ready\n");

	cfg.drv_data = fg;
	cfg.fwnode = dev_fwnode(dev);
	fg->psy = devm_power_supply_register(dev, &sm5714_fg_desc, &cfg);
	if (IS_ERR(fg->psy))
		return dev_err_probe(dev, PTR_ERR(fg->psy), "failed to register battery\n");

	INIT_DELAYED_WORK(&fg->work, sm5714_fg_work);
	ret = devm_add_action_or_reset(dev, sm5714_fg_cancel_work, &fg->work);
	if (ret)
		return ret;
	schedule_delayed_work(&fg->work, SM5714_FG_POLL_INTERVAL);

	dev_info(dev, "SM5714 fuel gauge, device id 0x%04x, temperature from %s\n",
		 id, fg->ntc ? "battery NTC" : "gauge");

	return 0;
}

static const struct of_device_id sm5714_fg_of_match[] = {
	{ .compatible = "siliconmitus,sm5714-fuelgauge" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5714_fg_of_match);

static struct i2c_driver sm5714_fg_driver = {
	.driver = {
		.name = "sm5714-fuelgauge",
		.of_match_table = sm5714_fg_of_match,
	},
	.probe = sm5714_fg_probe,
};
module_i2c_driver(sm5714_fg_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5714 fuel gauge (read-only)");
MODULE_LICENSE("GPL");
