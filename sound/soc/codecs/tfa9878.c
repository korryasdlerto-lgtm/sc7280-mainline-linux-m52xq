// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal driver for the NXP TFA9878 mono class-D speaker amplifier
 * (Samsung Galaxy M52 5G, SM-M526B).
 *
 * Register programming is taken from the downstream Samsung tfa98xx driver
 * (tfa9878_specific() "optimal settings" for revision 0x1a78) and from the
 * device/"music" profile bitfields of the M52 tfa98xx.cnt container. The
 * speaker protection algorithm (SB/PowerBass/MBDRC messages) runs on the
 * ADSP downstream and is not used here, so keep the playback level moderate.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define TFA9878_REG_REV		0x03

/* bitfield: address << 8 | pos << 4 | (len - 1), as in the NXP headers */
#define TFA_BF_PWDN		0x0000
#define TFA_BF_I2CR		0x0010
#define TFA_BF_AMPE		0x0030
#define TFA_BF_AMPGAIN		0x5257
#define TFA_AMPGAIN_DEFAULT	0xc3	/* from reg 0x52 = 0x587c (tfa9878_rev1a78_regs) */
#define TFA_BF_DCA		0x0040
#define TFA_BF_MANSCONF		0x0120
#define TFA_BF_AUDFS		0x0203
#define TFA_BF_CLKS		0x1060
#define TFA_BF_TDMNBCK		0x20c3
#define TFA_BF_TDMSLLN		0x2144
#define TFA_BF_TDMSSIZE		0x2224
#define TFA_BF_MTPK		0xa107

struct tfa_bf_val {
	u16 bf;
	u16 val;
};

struct tfa_reg_val {
	u8 reg;
	u16 val;
};

/* tfa9878_specific(), case 0x1a78 ("version 12") */
static const struct tfa_reg_val tfa9878_rev1a78_regs[] = {
	{ 0x01, 0x2e18 }, { 0x02, 0x0628 }, { 0x04, 0x0241 }, { 0x52, 0x587c },
	{ 0x61, 0x0183 }, { 0x63, 0x055a }, { 0x65, 0x0542 }, { 0x70, 0xb7ff },
	{ 0x71, 0x303e }, { 0x83, 0x009a }, { 0x84, 0x0211 }, { 0x8c, 0x0210 },
	{ 0xce, 0x2202 }, { 0xd5, 0x0000 },
};

/* tfa98xx.cnt, device "speaker" list (bitfields only) */
static const struct tfa_bf_val tfa9878_dev_bfs[] = {
	{ 0x20c3, 0x2 }, { 0x2144, 0x1f }, { 0x2224, 0x17 }, { 0x2013, 0x1 },
	{ 0x2330, 0x1 }, { 0x2340, 0x1 }, { 0x2603, 0x0 }, { 0x2643, 0x1 },
	{ 0x2300, 0x1 }, { 0x2310, 0x0 }, { 0x26c3, 0x0 }, { 0x2703, 0x1 },
	{ 0x6802, 0x1 }, { 0x6831, 0x1 }, { 0x6851, 0x2 }, { 0x8790, 0x0 },
	{ 0x70f0, 0x1 }, { 0x0401, 0x1 }, { 0x70e0, 0x0 }, { 0x7003, 0xa },
	{ 0x5f63, 0x4 }, { 0x5f23, 0xf }, { 0x7444, 0x9 }, { 0x7584, 0xd },
	{ 0x7695, 0x3f }, { 0x6a02, 0x7 }, { 0x6305, 0x1a }, { 0x6365, 0x1 },
	{ 0x6505, 0x1 }, { 0x6565, 0x1 }, { 0x6105, 0x3 }, { 0x6165, 0x2c },
};

/* tfa98xx.cnt, profile "music" (bitfields only; REV pseudo-write dropped) */
static const struct tfa_bf_val tfa9878_music_bfs[] = {
	{ 0x0255, 0x31 }, { 0x0040, 0x1 }, { 0x64e1, 0x0 }, { 0x60e1, 0x0 },
	{ 0x62e1, 0x0 }, { 0xb190, 0x0 }, { 0x5f63, 0x4 },
};

struct tfa9878 {
	struct regmap *regmap;
	unsigned int rate;
	unsigned int slot_width;
	u8 gain;		/* AMPGAIN, "Speaker Amp Gain" control (the call volume of the loud speaker) */
	bool on;
};

static int tfa_set_bf(struct regmap *map, u16 bf, u16 val)
{
	unsigned int reg = bf >> 8, pos = (bf >> 4) & 0xf, len = (bf & 0xf) + 1;
	unsigned int mask = GENMASK(pos + len - 1, pos);

	return regmap_update_bits(map, reg, mask, (val << pos) & mask);
}

static int tfa_get_bf(struct regmap *map, u16 bf, unsigned int *val)
{
	unsigned int reg = bf >> 8, pos = (bf >> 4) & 0xf, len = (bf & 0xf) + 1;
	int ret;

	ret = regmap_read(map, reg, val);
	if (ret)
		return ret;
	*val = (*val >> pos) & GENMASK(len - 1, 0);
	return 0;
}

static int tfa9878_rate_code(unsigned int rate)
{
	switch (rate) {
	case 8000:	return 0;
	case 11025:	return 1;
	case 12000:	return 2;
	case 16000:	return 3;
	case 22050:	return 4;
	case 24000:	return 5;
	case 32000:	return 6;
	case 44100:	return 7;
	default:	return 8;	/* 48000 */
	}
}

/*
 * m52xq: the I2C writes of the power-up sequence used to be fire-and-forget. During a voice call the GPI DMA that serves this bus
 * (gpi 900000: "Event without a pending descriptor!") sometimes loses a transfer when the route is switched, the amplifier stayed
 * half configured and the loud speaker was silent (it worked on the 3rd try). Every transfer is now checked, the state is read
 * back and the whole sequence is repeated (at most TFA_POWERUP_TRIES times).
 */
#define TFA_POWERUP_TRIES	4

static int tfa9878_power_up_once(struct tfa9878 *tfa, int *errs)
{
	struct regmap *map = tfa->regmap;
	unsigned int val, clks = 0, pwdn = 1, ampe = 0, mansconf = 0, audfs = 0xff, gain = 0;
	int i, ret, err = 0;

#define TFA_W(reg, v) do { ret = regmap_write(map, (reg), (v)); if (ret) { err = ret; (*errs)++; } } while (0)
#define TFA_B(bf, v) do { ret = tfa_set_bf(map, (bf), (v)); if (ret) { err = ret; (*errs)++; } } while (0)

	/* soft reset of all I2C registers (tfa98xx_init) */
	ret = regmap_write(map, TFA_BF_I2CR >> 8, BIT(1));
	if (ret)
		return ret;
	usleep_range(1000, 2000);

	/* unlock key1 and key2 (tfa9878_specific, tfa98xx_key2) */
	TFA_W(0x0f, 0x5a6b);
	ret = regmap_read(map, 0xfb, &val);
	if (ret) {
		err = ret;
		(*errs)++;
		val = 0;
	}
	TFA_W(0xa0, val ^ 0x005a);
	TFA_W(0x0f, 0x5a6b);
	TFA_B(TFA_BF_MTPK, 0x5a);

	for (i = 0; i < ARRAY_SIZE(tfa9878_rev1a78_regs); i++)
		TFA_W(tfa9878_rev1a78_regs[i].reg, tfa9878_rev1a78_regs[i].val);

	for (i = 0; i < ARRAY_SIZE(tfa9878_dev_bfs); i++)
		TFA_B(tfa9878_dev_bfs[i].bf, tfa9878_dev_bfs[i].val);

	/*
	 * The container is made for 2 x 32-bit slots (64 BCLK/frame, 24-bit
	 * samples). Other slot widths are untested: a 16-bit guess
	 * (NBCK 1, SLLN/SSIZE 15) gave TDMERR/TDMLUTER, so warn about them.
	 */
	if (tfa->slot_width != 32)
		pr_warn("tfa9878: %u-bit slots, container expects 32\n",
			tfa->slot_width);

	TFA_B(TFA_BF_AUDFS, tfa9878_rate_code(tfa->rate));
	for (i = 0; i < ARRAY_SIZE(tfa9878_music_bfs); i++)
		TFA_B(tfa9878_music_bfs[i].bf, tfa9878_music_bfs[i].val);
	TFA_B(TFA_BF_DCA, 0);

	/* TFA_STATE_INIT_CF / TFA_STATE_OPERATING for a probus (DSP-less) part */
	TFA_B(TFA_BF_MANSCONF, 1);
	TFA_B(TFA_BF_PWDN, 0);
	for (i = 0; i < 50; i++) {
		if (!tfa_get_bf(map, TFA_BF_CLKS, &clks) && clks)
			break;
		usleep_range(1000, 2000);
	}
	if (!clks)
		pr_warn("tfa9878: clocks not stable (no BCLK/WS?)\n");

	TFA_B(TFA_BF_AMPGAIN, tfa->gain);
	TFA_B(TFA_BF_AMPE, 1);

	/* read back what decides whether the amplifier plays */
	if (tfa_get_bf(map, TFA_BF_PWDN, &pwdn) || tfa_get_bf(map, TFA_BF_AMPE, &ampe) ||
	    tfa_get_bf(map, TFA_BF_MANSCONF, &mansconf) || tfa_get_bf(map, TFA_BF_AUDFS, &audfs) ||
	    tfa_get_bf(map, TFA_BF_AMPGAIN, &gain)) {
		(*errs)++;
		return err ? err : -EIO;
	}
	/* only PWDN / AMPE / MANSCONF decide: the chip may adjust AUDFS / AMPGAIN itself (they are only printed) */
	if (pwdn != 0 || ampe != 1 || mansconf != 1) {
		pr_warn("tfa9878: read back PWDN %u AMPE %u MANSCONF %u (AUDFS %u AMPGAIN %u), expected 0 1 1\n", pwdn, ampe, mansconf,
			audfs, gain);
		return -EIO;
	}
#undef TFA_W
#undef TFA_B
	return err;
}

static int tfa9878_power_up(struct tfa9878 *tfa)
{
	int attempt, errs = 0, ret = -EIO;

	for (attempt = 1; attempt <= TFA_POWERUP_TRIES; attempt++) {
		int e = 0;

		ret = tfa9878_power_up_once(tfa, &e);
		errs += e;
		if (!ret) {
			if (attempt > 1)
				pr_info("tfa9878: power up ok after %d attempts (%d failed I2C transfers)\n", attempt, errs);
			tfa->on = true;
			return 0;
		}
		pr_warn("tfa9878: power up attempt %d of %d failed: %d (%d I2C errors)\n", attempt, TFA_POWERUP_TRIES, ret, e);
		msleep(20);
	}
	/* leave the amplifier in the state of the last try: AMPE may be set, the gain control keeps working */
	tfa->on = true;
	return ret;
}

static void tfa9878_power_down(struct tfa9878 *tfa)
{
	tfa->on = false;
	tfa_set_bf(tfa->regmap, TFA_BF_AMPE, 0);
	tfa_set_bf(tfa->regmap, TFA_BF_PWDN, 1);
}

static int tfa9878_amp_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct tfa9878 *tfa = snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		return tfa9878_power_up(tfa);
	case SND_SOC_DAPM_PRE_PMD:
		tfa9878_power_down(tfa);
		break;
	}
	return 0;
}

static const struct snd_soc_dapm_widget tfa9878_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("AIF IN", "Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUT_DRV_E("AMP", SND_SOC_NOPM, 0, 0, NULL, 0,
			       tfa9878_amp_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUTPUT("SPK"),
	/* current/voltage sense sent back to the ADSP on DATAO (QUIN_MI2S_TX) */
	SND_SOC_DAPM_SIGGEN("IV"),
	SND_SOC_DAPM_AIF_OUT("AIF OUT", "Capture", 0, SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route tfa9878_dapm_routes[] = {
	{ "AMP", NULL, "AIF IN" },
	{ "SPK", NULL, "AMP" },
	{ "AIF OUT", NULL, "IV" },
};

static int tfa9878_gain_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct tfa9878 *tfa = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = tfa->gain;
	return 0;
}

static int tfa9878_gain_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct tfa9878 *tfa = snd_soc_component_get_drvdata(component);
	u8 val = ucontrol->value.integer.value[0];

	if (val == tfa->gain)
		return 0;
	tfa->gain = val;
	if (tfa->on)
		tfa_set_bf(tfa->regmap, TFA_BF_AMPGAIN, val);
	return 1;
}

static const struct snd_kcontrol_new tfa9878_controls[] = {
	SOC_SINGLE_EXT("Speaker Amp Gain", SND_SOC_NOPM, 0, 255, 0, tfa9878_gain_get, tfa9878_gain_put),
};

static const struct snd_soc_component_driver tfa9878_component = {
	.controls		= tfa9878_controls,
	.num_controls		= ARRAY_SIZE(tfa9878_controls),
	.dapm_widgets		= tfa9878_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(tfa9878_dapm_widgets),
	.dapm_routes		= tfa9878_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(tfa9878_dapm_routes),
	.idle_bias_on		= false,
	.endianness		= 1,
};

static int tfa9878_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct tfa9878 *tfa = snd_soc_component_get_drvdata(dai->component);

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	tfa->rate = params_rate(params);
	tfa->slot_width = params_physical_width(params);
	return 0;
}

static int tfa9878_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_BC_FC)
		return -EINVAL;
	return 0;
}

static const struct snd_soc_dai_ops tfa9878_dai_ops = {
	.hw_params	= tfa9878_hw_params,
	.set_fmt	= tfa9878_set_fmt,
};

static struct snd_soc_dai_driver tfa9878_dai = {
	.name = "tfa9878-hifi",
	.playback = {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_32000 |
				  SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
				  SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture = {
		.stream_name	= "Capture",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_32000 |
				  SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
				  SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &tfa9878_dai_ops,
};

static const struct regmap_config tfa9878_regmap = {
	.reg_bits	= 8,
	.val_bits	= 16,
	.max_register	= 0xff,
	.cache_type	= REGCACHE_NONE,
};

static int tfa9878_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct tfa9878 *tfa;
	unsigned int rev;
	int ret;

	tfa = devm_kzalloc(dev, sizeof(*tfa), GFP_KERNEL);
	if (!tfa)
		return -ENOMEM;

	tfa->rate = 48000;
	tfa->slot_width = 16;
	tfa->gain = TFA_AMPGAIN_DEFAULT;
	tfa->regmap = devm_regmap_init_i2c(i2c, &tfa9878_regmap);
	if (IS_ERR(tfa->regmap))
		return PTR_ERR(tfa->regmap);

	ret = regmap_read(tfa->regmap, TFA9878_REG_REV, &rev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read revision\n");
	if ((rev & 0xff) != 0x78)
		return dev_err_probe(dev, -ENODEV, "not a TFA9878 (rev 0x%04x)\n", rev);
	dev_info(dev, "TFA9878 rev 0x%04x\n", rev);

	i2c_set_clientdata(i2c, tfa);

	return devm_snd_soc_register_component(dev, &tfa9878_component,
					       &tfa9878_dai, 1);
}

static const struct i2c_device_id tfa9878_i2c_id[] = {
	{ "tfa9878" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tfa9878_i2c_id);

static const struct of_device_id tfa9878_of_match[] = {
	{ .compatible = "nxp,tfa9878" },
	{ .compatible = "tfa,tfa98xx" },	/* Samsung downstream DTBO node */
	{ }
};
MODULE_DEVICE_TABLE(of, tfa9878_of_match);

static struct i2c_driver tfa9878_i2c_driver = {
	.driver = {
		.name = "tfa9878",
		.of_match_table = tfa9878_of_match,
	},
	.probe = tfa9878_i2c_probe,
	.id_table = tfa9878_i2c_id,
};
module_i2c_driver(tfa9878_i2c_driver);

MODULE_DESCRIPTION("ASoC NXP TFA9878 minimal speaker amplifier driver");
MODULE_LICENSE("GPL");
