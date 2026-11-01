// SPDX-License-Identifier: GPL-2.0-only
/*
 * M52XQ (SM7325): keep the phone alive across an ADSP restart (SSR).
 *
 * The LPASS codec macros (rx/tx/va) and the SoundWire masters take their clocks (MCLK, NPL) from
 * the ADSP (q6afe clocks). When the ADSP stops, those clocks disappear but the macro, SoundWire
 * and sound card drivers stay bound; after the restart the clocks are registered again, yet the
 * old consumers keep stale handles ("rx_macro: unable to prepare mclk") and the next register
 * access of the SoundWire IRQ thread or of a rebind hits unclocked hardware -> SoC hang -> watchdog
 * reset (tested 2026-10-17: unbinding card + SoundWire by hand before the restart survived it,
 * rebinding them with the stale macros hung the SoC).
 *
 * So: before the ADSP shuts down, release the drivers of the card, the SoundWire masters and the
 * macros; a few seconds after it is up again (APR services and q6afe clocks registered), attach
 * them back in reverse order.
 */
#include <linux/device.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/remoteproc/qcom_rproc.h>
#include <linux/workqueue.h>

/* release order (attach in reverse) */
static const char * const m52xq_audio_devs[] = {
	"soc:sound",		/* snd-sm8250 machine driver */
	"3210000.soundwire",	/* Rx SoundWire master */
	"3230000.soundwire",	/* Tx SoundWire master */
	"3200000.codec",	/* rx_macro */
	"3220000.codec",	/* tx_macro */
	"3370000.codec",	/* va_macro */
};

static bool released[ARRAY_SIZE(m52xq_audio_devs)];
static void *ssr_cookie;
static struct delayed_work attach_work;
static DEFINE_MUTEX(ssr_lock);

static struct device *m52xq_find(const char *name)
{
	return bus_find_device_by_name(&platform_bus_type, NULL, name);
}

static void m52xq_audio_release(void)
{
	int i;

	cancel_delayed_work_sync(&attach_work);
	mutex_lock(&ssr_lock);
	for (i = 0; i < ARRAY_SIZE(m52xq_audio_devs); i++) {
		struct device *dev = m52xq_find(m52xq_audio_devs[i]);

		if (!dev)
			continue;
		if (dev->driver) {
			device_release_driver(dev);
			released[i] = true;
			pr_info("m52xq-audio-ssr: released %s\n", m52xq_audio_devs[i]);
		}
		put_device(dev);
	}
	mutex_unlock(&ssr_lock);
}

static void m52xq_audio_attach(struct work_struct *work)
{
	int i;

	mutex_lock(&ssr_lock);
	for (i = ARRAY_SIZE(m52xq_audio_devs) - 1; i >= 0; i--) {
		struct device *dev;
		int ret;

		if (!released[i])
			continue;
		dev = m52xq_find(m52xq_audio_devs[i]);
		if (!dev)
			continue;
		ret = device_attach(dev);
		pr_info("m52xq-audio-ssr: attach %s: %d\n", m52xq_audio_devs[i], ret);
		released[i] = false;
		put_device(dev);
	}
	mutex_unlock(&ssr_lock);
}

static int m52xq_audio_ssr_notify(struct notifier_block *nb, unsigned long event, void *data)
{
	switch (event) {
	case QCOM_SSR_BEFORE_SHUTDOWN:
		pr_info("m52xq-audio-ssr: ADSP going down, releasing the audio drivers\n");
		m52xq_audio_release();
		break;
	case QCOM_SSR_AFTER_POWERUP:
		/* nothing to do at the first boot: only attach what we released */
		schedule_delayed_work(&attach_work, msecs_to_jiffies(3000));
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block m52xq_audio_ssr_nb = {
	.notifier_call = m52xq_audio_ssr_notify,
};

static int __init m52xq_audio_ssr_init(void)
{
	INIT_DELAYED_WORK(&attach_work, m52xq_audio_attach);
	ssr_cookie = qcom_register_ssr_notifier("lpass", &m52xq_audio_ssr_nb);
	if (IS_ERR(ssr_cookie))
		return PTR_ERR(ssr_cookie);
	pr_info("m52xq-audio-ssr: watching lpass restarts\n");
	return 0;
}
late_initcall(m52xq_audio_ssr_init);

MODULE_DESCRIPTION("M52XQ: release/attach the LPASS audio drivers around an ADSP restart");
MODULE_LICENSE("GPL");
