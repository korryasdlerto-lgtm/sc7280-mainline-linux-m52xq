// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S2MPB03 camera-sensor PMIC: 7 LDOs on I2C (SM-M526B, address 0x56).
 *
 * Register map from the Samsung 5.4 vendor driver (drivers/regulator/s2mpb03):
 * one control register per LDO (0x03..0x09): bits[5:0] voltage selector,
 * bit 7 enable.  LDO1/2/4: 0.7 V + 12.5 mV * sel, LDO3: 0.7 V + 25 mV * sel,
 * LDO5/6/7: 1.8 V + 25 mV * sel.
 */
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define S2MPB03_REG_PMIC_ID	0x00
#define S2MPB03_REG_LDO1_CTRL	0x03
#define S2MPB03_VSEL_MASK	0x3f
#define S2MPB03_ENABLE_MASK	0x80
#define S2MPB03_ENABLE_TIME_US	150
#define S2MPB03_RAMP_DELAY	12000	/* uV/us */

static const struct regulator_ops s2mpb03_ldo_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= regulator_set_voltage_time_sel,
};

#define S2MPB03_LDO(n, _min, _step)				\
	{							\
		.name		= "s2mpb03-ldo" #n,		\
		.of_match	= of_match_ptr("s2mpb03-ldo" #n),\
		.regulators_node = of_match_ptr("regulators"),	\
		.id		= n - 1,			\
		.ops		= &s2mpb03_ldo_ops,		\
		.type		= REGULATOR_VOLTAGE,		\
		.owner		= THIS_MODULE,			\
		.min_uV		= _min,				\
		.uV_step	= _step,			\
		.n_voltages	= S2MPB03_VSEL_MASK + 1,	\
		.vsel_reg	= S2MPB03_REG_LDO1_CTRL + n - 1,\
		.vsel_mask	= S2MPB03_VSEL_MASK,		\
		.enable_reg	= S2MPB03_REG_LDO1_CTRL + n - 1,\
		.enable_mask	= S2MPB03_ENABLE_MASK,		\
		.enable_time	= S2MPB03_ENABLE_TIME_US,	\
	}

static const struct regulator_desc s2mpb03_regulators[] = {
	S2MPB03_LDO(1, 700000, 12500),
	S2MPB03_LDO(2, 700000, 12500),
	S2MPB03_LDO(3, 700000, 25000),
	S2MPB03_LDO(4, 700000, 12500),
	S2MPB03_LDO(5, 1800000, 25000),
	S2MPB03_LDO(6, 1800000, 25000),
	S2MPB03_LDO(7, 1800000, 25000),
};

static const struct regmap_config s2mpb03_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= 0x0b,
};

static int s2mpb03_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct regulator_config config = { };
	struct regmap *regmap;
	unsigned int id;
	int i, ret;

	regmap = devm_regmap_init_i2c(i2c, &s2mpb03_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap), "regmap init failed\n");

	ret = regmap_read(regmap, S2MPB03_REG_PMIC_ID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read the PMIC id\n");
	dev_info(dev, "S2MPB03 camera PMIC, id 0x%02x\n", id);

	config.dev = dev;
	config.regmap = regmap;

	for (i = 0; i < ARRAY_SIZE(s2mpb03_regulators); i++) {
		struct regulator_dev *rdev;

		rdev = devm_regulator_register(dev, &s2mpb03_regulators[i], &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "failed to register %s\n",
					     s2mpb03_regulators[i].name);
	}

	return 0;
}

static const struct of_device_id s2mpb03_of_match[] = {
	{ .compatible = "samsung,s2mpb03pmic" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpb03_of_match);

static const struct i2c_device_id s2mpb03_i2c_id[] = {
	{ "s2mpb03-pmic" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, s2mpb03_i2c_id);

static struct i2c_driver s2mpb03_driver = {
	.driver = {
		.name = "s2mpb03-regulator",
		.of_match_table = s2mpb03_of_match,
	},
	.probe = s2mpb03_probe,
	.id_table = s2mpb03_i2c_id,
};
module_i2c_driver(s2mpb03_driver);

MODULE_DESCRIPTION("Samsung S2MPB03 camera PMIC regulator driver");
MODULE_LICENSE("GPL");
