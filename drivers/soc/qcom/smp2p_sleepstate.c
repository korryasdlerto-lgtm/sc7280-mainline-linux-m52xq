// SPDX-License-Identifier: GPL-2.0-only
/*
 * M52XQ: tell the remote processors (ADSP) over smp2p that the application processor is suspended.
 *
 * Downstream (Samsung/QC) drivers/soc/qcom/smp2p_sleepstate.c: the "sleepstate" outbound smp2p entry has an
 * "AP is awake" bit (bit 12). It is cleared when the system goes to suspend and set again on resume; as long
 * as it stays set the ADSP never decides that the system sleeps and keeps its own sleep states off.
 * The inbound "sleepstate_see" entry is the ADSP's way to wake the AP.
 */
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/pm_wakeup.h>
#include <linux/suspend.h>
#include <linux/soc/qcom/smem_state.h>

#define SLEEPSTATE_AWAKE_BIT	BIT(12)

struct smp2p_sleepstate {
	struct device *dev;
	struct qcom_smem_state *state;
	struct notifier_block nb;
};

static int smp2p_sleepstate_pm_notify(struct notifier_block *nb, unsigned long event, void *unused)
{
	struct smp2p_sleepstate *ss = container_of(nb, struct smp2p_sleepstate, nb);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		qcom_smem_state_update_bits(ss->state, SLEEPSTATE_AWAKE_BIT, 0);
		break;
	case PM_POST_SUSPEND:
		qcom_smem_state_update_bits(ss->state, SLEEPSTATE_AWAKE_BIT, SLEEPSTATE_AWAKE_BIT);
		break;
	default:
		break;
	}

	return NOTIFY_DONE;
}

static irqreturn_t smp2p_sleepstate_irq(int irq, void *data)
{
	struct smp2p_sleepstate *ss = data;

	pm_wakeup_dev_event(ss->dev, 200, true);

	return IRQ_HANDLED;
}

static int smp2p_sleepstate_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct smp2p_sleepstate *ss;
	unsigned int bit;
	int irq, ret;

	ss = devm_kzalloc(dev, sizeof(*ss), GFP_KERNEL);
	if (!ss)
		return -ENOMEM;

	ss->dev = dev;
	ss->state = devm_qcom_smem_state_get(dev, NULL, &bit);
	if (IS_ERR(ss->state))
		return dev_err_probe(dev, PTR_ERR(ss->state), "failed to get the sleepstate smem state\n");

	device_init_wakeup(dev, true);

	irq = platform_get_irq_byname(pdev, "smp2p-sleepstate-in");
	if (irq < 0)
		return irq;

	ret = devm_request_threaded_irq(dev, irq, NULL, smp2p_sleepstate_irq,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT, "smp2p_sleepstate", ss);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request the sleepstate irq\n");

	ret = dev_pm_set_wake_irq(dev, irq);
	if (ret)
		dev_warn(dev, "failed to set the wake irq: %d\n", ret);

	/* the AP is awake now */
	qcom_smem_state_update_bits(ss->state, SLEEPSTATE_AWAKE_BIT, SLEEPSTATE_AWAKE_BIT);

	ss->nb.notifier_call = smp2p_sleepstate_pm_notify;
	ret = register_pm_notifier(&ss->nb);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, ss);
	dev_info(dev, "smp2p sleepstate ready (awake bit 12)\n");

	return 0;
}

static void smp2p_sleepstate_remove(struct platform_device *pdev)
{
	struct smp2p_sleepstate *ss = platform_get_drvdata(pdev);

	unregister_pm_notifier(&ss->nb);
	dev_pm_clear_wake_irq(&pdev->dev);
}

static const struct of_device_id smp2p_sleepstate_of_match[] = {
	{ .compatible = "qcom,smp2p-sleepstate" },
	{ }
};
MODULE_DEVICE_TABLE(of, smp2p_sleepstate_of_match);

static struct platform_driver smp2p_sleepstate_driver = {
	.probe = smp2p_sleepstate_probe,
	.remove = smp2p_sleepstate_remove,
	.driver = {
		.name = "smp2p-sleepstate",
		.of_match_table = smp2p_sleepstate_of_match,
	},
};
module_platform_driver(smp2p_sleepstate_driver);

MODULE_DESCRIPTION("Qualcomm SMP2P sleepstate notifier (M52XQ)");
MODULE_LICENSE("GPL");
