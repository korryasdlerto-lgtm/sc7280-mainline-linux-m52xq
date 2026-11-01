// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal Type-C port driver for the Silicon Mitus SM5714 PDIC
 * (Samsung Galaxy M52 5G), CC logic only -- no USB Power Delivery.
 *
 * Register meanings and the init sequence follow Samsung's sm5714_typec.c:
 *   CC_CNTL1 = 0x41   leave "sink only" mode (dual-role toggling)
 *   CC_CNTL3 = 0x80   CC_OP_EN (0x82 from CC_DISABLE / ATTWAIT_SRC with VBUS)
 *   CC_STATUS[2:0]    1 = a source is attached (we are sink, USB device role)
 *                     2 = a sink is attached (we are source: USB host + VBUS)
 *                     3 = audio adapter accessory, 4/5 = debug accessory
 *   CC_STATUS[5]      cable flipped (CC2)
 * INT1..INT5 are read-to-clear, the IRQ line (TLMM 140) is active low.
 *
 * Roles go to the dwc3 role switch found through the connector graph, VBUS
 * for a sink comes from the SM5714 charger's "vbus" regulator (OTG boost).
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/role.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_altmode.h>

#define SM5714_TC_REG_INT1		0x01	/* INT1..INT5: read to clear */
#define SM5714_TC_REG_INT_MASK1		0x06	/* INT_MASK1..5, 1 = masked */
#define SM5714_TC_REG_STATUS1		0x0b
#define SM5714_TC_REG_CC_STATUS		0x28
#define SM5714_TC_REG_CC_CNTL1		0x29
#define SM5714_TC_REG_CC_CNTL3		0x2b
#define SM5714_TC_REG_PD_STATE0		0xd5

#define SM5714_TC_INT1_VBUSPOK		BIT(0)
#define SM5714_TC_INT1_ATTACH		BIT(3)
#define SM5714_TC_INT1_DETACH		BIT(4)

#define SM5714_TC_CC_ATTACH_MASK	GENMASK(2, 0)
#define SM5714_TC_ATTACH_NONE		0
#define SM5714_TC_ATTACH_SOURCE		1	/* partner is a source */
#define SM5714_TC_ATTACH_SINK		2	/* partner is a sink */
#define SM5714_TC_ATTACH_AUDIO		3
#define SM5714_TC_ATTACH_DEBUG_UNORI	4
#define SM5714_TC_ATTACH_DEBUG_ORI	5
#define SM5714_TC_CC_ADV_CURR		GENMASK(4, 3)
#define SM5714_TC_CC_FLIP		BIT(5)

#define SM5714_TC_CC_CNTL1_DRP		0x41
#define SM5714_TC_CC_CNTL3_OP_EN	0x80
#define SM5714_TC_CC_CNTL3_UNATT_SNK	0x82

struct sm5714_tc {
	struct device *dev;
	struct i2c_client *client;
	struct mutex lock;
	struct fwnode_handle *connector;
	struct typec_port *port;
	struct typec_capability cap;
	struct typec_partner *partner;
	struct usb_role_switch *role_sw;
	struct regulator *vbus;
	bool vbus_on;
	int attach;		/* last CC_STATUS attach type, -1: unknown */
	int flip;
};

static const char * const sm5714_tc_attach_name[] = {
	[SM5714_TC_ATTACH_NONE]		= "nothing",
	[SM5714_TC_ATTACH_SOURCE]	= "source (USB device role)",
	[SM5714_TC_ATTACH_SINK]		= "sink (USB host role, VBUS on)",
	[SM5714_TC_ATTACH_AUDIO]	= "audio adapter accessory",
	[SM5714_TC_ATTACH_DEBUG_UNORI]	= "debug accessory",
	[SM5714_TC_ATTACH_DEBUG_ORI]	= "debug accessory (oriented)",
};

static int sm5714_tc_set_vbus(struct sm5714_tc *tc, bool on)
{
	int ret;

	if (!tc->vbus || on == tc->vbus_on)
		return 0;
	ret = on ? regulator_enable(tc->vbus) : regulator_disable(tc->vbus);
	if (ret) {
		dev_err(tc->dev, "failed to switch VBUS %s: %d\n", on ? "on" : "off", ret);
		return ret;
	}
	tc->vbus_on = on;

	return 0;
}

static void sm5714_tc_set_partner(struct sm5714_tc *tc, int attach)
{
	struct typec_partner_desc desc = {};

	if (tc->partner) {
		typec_unregister_partner(tc->partner);
		tc->partner = NULL;
	}
	if (attach == SM5714_TC_ATTACH_NONE)
		return;

	if (attach == SM5714_TC_ATTACH_AUDIO)
		desc.accessory = TYPEC_ACCESSORY_AUDIO;
	else if (attach >= SM5714_TC_ATTACH_DEBUG_UNORI)
		desc.accessory = TYPEC_ACCESSORY_DEBUG;
	tc->partner = typec_register_partner(tc->port, &desc);
	if (IS_ERR(tc->partner)) {
		dev_warn(tc->dev, "failed to register partner: %ld\n", PTR_ERR(tc->partner));
		tc->partner = NULL;
	}
}

/* Caller holds tc->lock. */
static void sm5714_tc_update(struct sm5714_tc *tc)
{
	enum usb_role role = USB_ROLE_DEVICE;
	int cc, attach, flip;
	bool source;

	cc = i2c_smbus_read_byte_data(tc->client, SM5714_TC_REG_CC_STATUS);
	if (cc < 0) {
		dev_err(tc->dev, "failed to read CC_STATUS: %d\n", cc);
		return;
	}
	attach = cc & SM5714_TC_CC_ATTACH_MASK;
	if (attach > SM5714_TC_ATTACH_DEBUG_ORI)
		attach = SM5714_TC_ATTACH_NONE;
	flip = !!(cc & SM5714_TC_CC_FLIP);
	if (attach == tc->attach && flip == tc->flip)
		return;

	dev_info(tc->dev, "CC_STATUS 0x%02x: %s%s\n", cc, sm5714_tc_attach_name[attach],
		 attach != SM5714_TC_ATTACH_NONE && flip ? ", flipped" : "");

	source = attach == SM5714_TC_ATTACH_SINK;
	if (source)
		role = USB_ROLE_HOST;

	/* VBUS off before the controller leaves host mode, on before it enters it */
	if (!source)
		sm5714_tc_set_vbus(tc, false);

	if (attach != tc->attach)
		sm5714_tc_set_partner(tc, attach);

	typec_set_orientation(tc->port, attach == SM5714_TC_ATTACH_NONE ?
			      TYPEC_ORIENTATION_NONE :
			      flip ? TYPEC_ORIENTATION_REVERSE : TYPEC_ORIENTATION_NORMAL);
	typec_set_pwr_role(tc->port, source ? TYPEC_SOURCE : TYPEC_SINK);
	typec_set_data_role(tc->port, source ? TYPEC_HOST : TYPEC_DEVICE);
	typec_set_vconn_role(tc->port, TYPEC_SINK);
	typec_set_mode(tc->port, attach == SM5714_TC_ATTACH_AUDIO ? TYPEC_MODE_AUDIO :
			attach >= SM5714_TC_ATTACH_DEBUG_UNORI ? TYPEC_MODE_DEBUG :
			TYPEC_STATE_USB);

	if (source)
		sm5714_tc_set_vbus(tc, true);

	usb_role_switch_set_role(tc->role_sw, role);

	tc->attach = attach;
	tc->flip = flip;
}

static irqreturn_t sm5714_tc_irq(int irq, void *data)
{
	struct sm5714_tc *tc = data;
	u8 ints[5];
	int ret;

	ret = i2c_smbus_read_i2c_block_data(tc->client, SM5714_TC_REG_INT1,
					    sizeof(ints), ints);
	if (ret < 0) {
		dev_err(tc->dev, "failed to read interrupts: %d\n", ret);
		return IRQ_NONE;
	}
	dev_dbg(tc->dev, "INT %*ph\n", (int)sizeof(ints), ints);

	mutex_lock(&tc->lock);
	sm5714_tc_update(tc);
	mutex_unlock(&tc->lock);

	return IRQ_HANDLED;
}

static void sm5714_tc_hw_init(struct sm5714_tc *tc)
{
	/* only attach/detach/VBUS interrupts; the PD block stays masked */
	u8 mask[5] = {
		(u8)~(SM5714_TC_INT1_VBUSPOK | SM5714_TC_INT1_ATTACH | SM5714_TC_INT1_DETACH),
		0xff, 0xff, 0xff, 0xff,
	};
	int status1, state0;
	u8 ints[5];

	i2c_smbus_write_i2c_block_data(tc->client, SM5714_TC_REG_INT_MASK1,
				       sizeof(mask), mask);
	i2c_smbus_read_i2c_block_data(tc->client, SM5714_TC_REG_INT1, sizeof(ints), ints);

	/* Samsung sm5714_usbpd_reg_init(): release "sink only" (dual role) */
	i2c_smbus_write_byte_data(tc->client, SM5714_TC_REG_CC_CNTL1, SM5714_TC_CC_CNTL1_DRP);

	/* Samsung sm5714_check_cc_state(), only while nothing is attached */
	status1 = i2c_smbus_read_byte_data(tc->client, SM5714_TC_REG_STATUS1);
	state0 = i2c_smbus_read_byte_data(tc->client, SM5714_TC_REG_PD_STATE0);
	if (status1 >= 0 && state0 >= 0 && !(status1 & SM5714_TC_INT1_ATTACH)) {
		if ((state0 & 0x0f) == 0x1 ||
		    ((state0 & 0x0f) == 0xa && (status1 & SM5714_TC_INT1_VBUSPOK)))
			i2c_smbus_write_byte_data(tc->client, SM5714_TC_REG_CC_CNTL3,
						  SM5714_TC_CC_CNTL3_UNATT_SNK);
		else
			i2c_smbus_write_byte_data(tc->client, SM5714_TC_REG_CC_CNTL3,
						  SM5714_TC_CC_CNTL3_OP_EN);
	}
	dev_info(tc->dev, "STATUS1 0x%02x PD_STATE0 0x%02x, dual role enabled\n",
		 status1, state0);
}

static void sm5714_tc_cleanup(void *data)
{
	struct sm5714_tc *tc = data;

	mutex_lock(&tc->lock);
	sm5714_tc_set_vbus(tc, false);
	if (tc->partner)
		typec_unregister_partner(tc->partner);
	tc->partner = NULL;
	mutex_unlock(&tc->lock);
	typec_unregister_port(tc->port);
	usb_role_switch_put(tc->role_sw);
	fwnode_handle_put(tc->connector);
}

static int sm5714_tc_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sm5714_tc *tc;
	int ret;

	if (!client->irq)
		return dev_err_probe(dev, -EINVAL, "no interrupt\n");

	tc = devm_kzalloc(dev, sizeof(*tc), GFP_KERNEL);
	if (!tc)
		return -ENOMEM;
	tc->dev = dev;
	tc->client = client;
	tc->attach = -1;
	tc->flip = -1;
	mutex_init(&tc->lock);

	tc->connector = device_get_named_child_node(dev, "connector");
	if (!tc->connector)
		return dev_err_probe(dev, -ENODEV, "no connector node\n");

	tc->vbus = devm_regulator_get_optional(dev, "vbus");
	if (IS_ERR(tc->vbus)) {
		ret = PTR_ERR(tc->vbus);
		if (ret != -ENODEV) {
			fwnode_handle_put(tc->connector);
			return dev_err_probe(dev, ret, "failed to get VBUS supply\n");
		}
		dev_warn(dev, "no VBUS supply, host mode without power\n");
		tc->vbus = NULL;
	}

	tc->role_sw = fwnode_usb_role_switch_get(tc->connector);
	if (IS_ERR(tc->role_sw)) {
		fwnode_handle_put(tc->connector);
		return dev_err_probe(dev, PTR_ERR(tc->role_sw), "failed to get role switch\n");
	}

	tc->cap.type = TYPEC_PORT_DRP;
	tc->cap.data = TYPEC_PORT_DRD;
	tc->cap.revision = USB_TYPEC_REV_1_2;
	tc->cap.prefer_role = TYPEC_NO_PREFERRED_ROLE;
	tc->cap.accessory[0] = TYPEC_ACCESSORY_AUDIO;
	tc->cap.accessory[1] = TYPEC_ACCESSORY_DEBUG;
	tc->cap.orientation_aware = true;
	tc->cap.fwnode = tc->connector;
	tc->port = typec_register_port(dev, &tc->cap);
	if (IS_ERR(tc->port)) {
		usb_role_switch_put(tc->role_sw);
		fwnode_handle_put(tc->connector);
		return dev_err_probe(dev, PTR_ERR(tc->port), "failed to register port\n");
	}

	ret = devm_add_action_or_reset(dev, sm5714_tc_cleanup, tc);
	if (ret)
		return ret;

	sm5714_tc_hw_init(tc);
	/* give the CC state machine time to settle after leaving sink-only */
	msleep(100);
	mutex_lock(&tc->lock);
	sm5714_tc_update(tc);
	mutex_unlock(&tc->lock);

	ret = devm_request_threaded_irq(dev, client->irq, NULL, sm5714_tc_irq,
					IRQF_ONESHOT, "sm5714-usbpd", tc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request irq\n");
	enable_irq_wake(client->irq);

	return 0;
}

static const struct of_device_id sm5714_tc_of_match[] = {
	{ .compatible = "siliconmitus,sm5714-usbpd" },
	{ .compatible = "sm5714-usbpd" },	/* as named by the Samsung DTBO */
	{ }
};
MODULE_DEVICE_TABLE(of, sm5714_tc_of_match);

static struct i2c_driver sm5714_tc_driver = {
	.driver = {
		.name = "sm5714-usbpd",
		.of_match_table = sm5714_tc_of_match,
	},
	.probe = sm5714_tc_probe,
};
module_i2c_driver(sm5714_tc_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5714 Type-C port (CC only)");
MODULE_LICENSE("GPL");
