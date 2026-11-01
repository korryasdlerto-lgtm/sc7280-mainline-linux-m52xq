// SPDX-License-Identifier: GPL-2.0
/*
 * Debug helper for the Galaxy M52 5G (SM-M526B) cameras whose sensor drivers
 * do not exist yet: powers the sensor from the DT description, scans the CCI
 * bus for ACKing addresses and prints the 16-bit chip-id registers 0x0000 and
 * 0x0002 of every address found.  Not meant to be upstreamed.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

static const char * const names[] = { "vio", "vdig", "vana", "vaf" };

static int rd16(struct i2c_client *c, u16 addr, u16 reg, u16 *val)
{
	u8 out[2] = { reg >> 8, reg & 0xff }, in[2];
	struct i2c_msg m[2] = {
		{ .addr = addr, .flags = 0, .len = 2, .buf = out },
		{ .addr = addr, .flags = I2C_M_RD, .len = 2, .buf = in },
	};

	if (i2c_transfer(c->adapter, m, 2) != 2)
		return -EIO;
	*val = in[0] << 8 | in[1];
	return 0;
}

static void scan(struct i2c_client *c, const char *tag)
{
	u8 b;
	int a;

	for (a = 0x08; a < 0x78; a++) {
		struct i2c_msg m = { .addr = a, .flags = I2C_M_RD, .len = 1, .buf = &b };
		u16 v0 = 0, v2 = 0;

		if (a == c->addr)
			continue;
		if (i2c_transfer(c->adapter, &m, 1) != 1)
			continue;
		rd16(c, a, 0x0000, &v0);
		rd16(c, a, 0x0002, &v2);
		dev_info(&c->dev, "%s: ACK at 0x%02x  reg0000=0x%04x reg0002=0x%04x\n",
			 tag, a, v0, v2);
		{
			/* 8-bit register addressing dump of registers 0x00..0x0f (voice-coil drivers) */
			u8 dump[16], r0 = 0;
			struct i2c_msg dm[2] = {
				{ .addr = a, .flags = 0, .len = 1, .buf = &r0 },
				{ .addr = a, .flags = I2C_M_RD, .len = 16, .buf = dump },
			};

			if (i2c_transfer(c->adapter, dm, 2) == 2)
				dev_info(&c->dev, "  8bit regs 00..0f: %*phN\n", 16, dump);
		}
	}
}

static int camscan_probe(struct i2c_client *c)
{
	struct device *dev = &c->dev;
	struct regulator *r[ARRAY_SIZE(names)] = {};
	struct gpio_desc *rst;
	struct clk *clk;
	int i, ret;

	clk = devm_clk_get_optional(dev, "xclk");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "clk\n");
	rst = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "reset\n");
	for (i = 0; i < ARRAY_SIZE(names); i++) {
		r[i] = devm_regulator_get_optional(dev, names[i]);
		if (IS_ERR(r[i])) {
			if (PTR_ERR(r[i]) == -EPROBE_DEFER)
				return -EPROBE_DEFER;
			r[i] = NULL;
		}
	}

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		if (!r[i])
			continue;
		ret = regulator_enable(r[i]);
		dev_info(dev, "supply %s on (%d) %d uV\n", names[i], ret, regulator_get_voltage(r[i]));
		usleep_range(1000, 2000);
	}
	ret = clk_prepare_enable(clk);
	dev_info(dev, "clk %lu Hz on (%d)\n", clk_get_rate(clk), ret);
	usleep_range(2000, 3000);

	scan(c, "reset asserted");
	if (rst) {
		gpiod_set_value_cansleep(rst, 0);
		msleep(20);
		scan(c, "reset released");
		gpiod_set_value_cansleep(rst, 1);
		msleep(5);
		gpiod_set_value_cansleep(rst, 0);
		msleep(50);
		scan(c, "reset pulsed+50ms");
	}

	clk_disable_unprepare(clk);
	for (i = ARRAY_SIZE(names) - 1; i >= 0; i--)
		if (r[i])
			regulator_disable(r[i]);
	if (rst)
		gpiod_set_value_cansleep(rst, 1);
	return 0;
}

static const struct of_device_id camscan_of[] = { { .compatible = "m52xq,camscan" }, { } };
MODULE_DEVICE_TABLE(of, camscan_of);

static struct i2c_driver camscan_driver = {
	.driver = { .name = "m52xq-camscan", .of_match_table = camscan_of },
	.probe = camscan_probe,
};
module_i2c_driver(camscan_driver);
MODULE_LICENSE("GPL");
