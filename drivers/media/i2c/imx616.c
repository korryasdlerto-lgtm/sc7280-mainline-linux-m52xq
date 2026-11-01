// SPDX-License-Identifier: GPL-2.0
/*
 * Sony IMX616 (64 Mpixel Quad Bayer, 4-lane MIPI CSI-2) sensor driver for the
 * front camera of the Galaxy M52 5G (SM-M526B).
 *
 * The register sequences are the Samsung setB files of the Exynos is-cis-imx616
 * driver (global + 2x2 binning 3264x2448 30 fps).  They assume a 26 MHz EXTCLK;
 * this board supplies 24 MHz, so EXTCLK (0x0136/7) and the PLL multipliers
 * (0x0307 -> 237, 0x030E/F -> 257) are patched, giving 2056 Mbit/s per lane
 * (original 2054).
 *
 * m52xq: second mode, the full 6528x4896 frame (setB "REG_F Full Remosaic", the sensor
 * re-mosaics the quad Bayer array itself and sends a plain Bayer frame, 24 fps nominal).
 * Its PLL is patched the same way for 24 MHz: VT multiplier 0x0306/7 332 -> 360,
 * OP multiplier 0x030E/F 256 -> 277 = 2216 Mbit/s per lane (original 2218.67).
 */
#include <linux/clk.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

#define IMX616_REG_CHIP_ID		CCI_REG16(0x0016)
#define IMX616_REG_STREAM		CCI_REG8(0x0100)
#define IMX616_REG_EXPOSURE		CCI_REG16(0x0202)
#define IMX616_REG_AGAIN		CCI_REG16(0x0204)
#define IMX616_REG_VTS			CCI_REG16(0x0340)

#define IMX616_XCLK_FREQ		24000000
#define IMX616_LINK_FREQ		1028000000LL
#define IMX616_LANES			4
#define IMX616_PIXEL_RATE		(IMX616_LINK_FREQ * 2 * IMX616_LANES / 10)

#define IMX616_WIDTH			3264
#define IMX616_HEIGHT			2448
#define IMX616_HTS			7536
/*
 * m52xq v226: the frame length of the setB table (0x0340/1 = 0x09D6). 3634 came from the nominal PIXEL_RATE (link rate
 * 822.4 Mpixel/s), but the VT clock is 24 MHz / 4 x 237 = 1422 MHz -> 568.8 Mpixel/s: 7536 x 3634 gave 20.8 fps,
 * 7536 x 2518 gives the 30 fps of the setB mode.
 */
#define IMX616_VTS_DEF			2518

#define IMX616_LINK_FREQ_FULL		1108000000LL
#define IMX616_PIXEL_RATE_FULL		(IMX616_LINK_FREQ_FULL * 2 * IMX616_LANES / 10)
#define IMX616_FULL_WIDTH		6528
#define IMX616_FULL_HEIGHT		4896
#define IMX616_FULL_HTS			7192
#define IMX616_FULL_VTS_DEF		5000

/*
 * Quad sensitivity calibration (QSC) of the module for the sensor's own re-mosaic (full
 * frame mode): 1560 bytes from the module EEPROM (M52: CCI1 master 1, 0x51, offset 0x0970),
 * written from 0xC500 before the stream starts (is-cis-imx616.c QuadSensCal_write). It is
 * different on every phone. v235: the driver reads it from the module EEPROM itself (bus 1 of
 * the sensor's own CCI, address 0x51, 16-bit big-endian offset), so no per-phone file has to be
 * shipped; the firmware "imx616-qsc.bin" stays as a fallback when the EEPROM cannot be read.
 * Written while streaming it has no effect; without it the full frame shows colour blotches.
 */
#define IMX616_EEPROM_ADDR		0x51
#define IMX616_EEPROM_QSC_OFFSET	0x0970
#define IMX616_EEPROM_CHUNK		8	/* the CCI reads at most 12 bytes per transfer */
#define IMX616_EEPROM_BUS_REG		1	/* i2c-bus@1 of the CCI (the sensor is on bus@0) */
#define IMX616_QSC_REG			0xC500
#define IMX616_QSC_SIZE			1560
#define IMX616_QSC_FW			"imx616-qsc.bin"
#define IMX616_VTS_MAX			0xffff
#define IMX616_VBLANK_MIN		16
#define IMX616_EXPOSURE_MARGIN		24
#define IMX616_EXPOSURE_MIN		8
#define IMX616_EXPOSURE_DEF		0x9a6

/* gain control is in units of 1/64 (64 = 1x); the register holds 1024 - 1024 / gain */
#define IMX616_GAIN_MIN			72
#define IMX616_GAIN_MAX			4096
#define IMX616_GAIN_DEF			72

static const char * const imx616_supply_names[] = {
	"vio", "vdig", "vana",
};
#define IMX616_NUM_SUPPLIES ARRAY_SIZE(imx616_supply_names)

static const struct cci_reg_sequence imx616_global_regs[] = {
	{ CCI_REG8(0x0136), 0x18 },
	{ CCI_REG8(0x0137), 0x00 },
	{ CCI_REG8(0x3C7E), 0x0D },
	{ CCI_REG8(0x3C7F), 0x14 },
	{ CCI_REG8(0x0111), 0x02 },
	{ CCI_REG8(0x380C), 0x00 },
	{ CCI_REG8(0x3C00), 0x01 },
	{ CCI_REG8(0x3C01), 0x00 },
	{ CCI_REG8(0x3C02), 0x00 },
	{ CCI_REG8(0x3C03), 0x03 },
	{ CCI_REG8(0x3C04), 0xFF },
	{ CCI_REG8(0x3C05), 0x01 },
	{ CCI_REG8(0x3C06), 0x00 },
	{ CCI_REG8(0x3C07), 0x00 },
	{ CCI_REG8(0x3C08), 0x03 },
	{ CCI_REG8(0x3C09), 0xFF },
	{ CCI_REG8(0x3C0A), 0x00 },
	{ CCI_REG8(0x3C0B), 0x00 },
	{ CCI_REG8(0x3C0C), 0x10 },
	{ CCI_REG8(0x3C0D), 0x10 },
	{ CCI_REG8(0x3C0E), 0x10 },
	{ CCI_REG8(0x3C0F), 0x10 },
	{ CCI_REG8(0x3C10), 0x10 },
	{ CCI_REG8(0x3C11), 0x20 },
	{ CCI_REG8(0x3C15), 0x00 },
	{ CCI_REG8(0x3C16), 0x00 },
	{ CCI_REG8(0x3C17), 0x01 },
	{ CCI_REG8(0x3C18), 0x00 },
	{ CCI_REG8(0x3C19), 0x01 },
	{ CCI_REG8(0x3C1A), 0x00 },
	{ CCI_REG8(0x3C1B), 0x01 },
	{ CCI_REG8(0x3C1C), 0x00 },
	{ CCI_REG8(0x3C1D), 0x01 },
	{ CCI_REG8(0x3C1E), 0x00 },
	{ CCI_REG8(0x3C1F), 0x00 },
	{ CCI_REG8(0x3F89), 0x01 },
	{ CCI_REG8(0x3F8F), 0x01 },
	{ CCI_REG8(0x53B9), 0x01 },
	{ CCI_REG8(0x571C), 0x00 },
	{ CCI_REG8(0x571D), 0x72 },
	{ CCI_REG8(0x5732), 0x00 },
	{ CCI_REG8(0x5733), 0xAD },
	{ CCI_REG8(0x5736), 0x00 },
	{ CCI_REG8(0x5737), 0x2F },
	{ CCI_REG8(0x5748), 0x00 },
	{ CCI_REG8(0x5749), 0x72 },
	{ CCI_REG8(0x5754), 0x00 },
	{ CCI_REG8(0x5755), 0xAD },
	{ CCI_REG8(0x5756), 0x00 },
	{ CCI_REG8(0x5757), 0x59 },
	{ CCI_REG8(0x5768), 0x00 },
	{ CCI_REG8(0x5769), 0x8B },
	{ CCI_REG8(0x577A), 0x00 },
	{ CCI_REG8(0x577B), 0x72 },
	{ CCI_REG8(0x577C), 0x00 },
	{ CCI_REG8(0x577D), 0x1D },
	{ CCI_REG8(0x5784), 0x02 },
	{ CCI_REG8(0x5785), 0x4E },
	{ CCI_REG8(0x5786), 0x00 },
	{ CCI_REG8(0x5787), 0xAD },
	{ CCI_REG8(0x5788), 0x00 },
	{ CCI_REG8(0x5789), 0x66 },
	{ CCI_REG8(0x5794), 0x06 },
	{ CCI_REG8(0x5795), 0xB3 },
	{ CCI_REG8(0x579C), 0x00 },
	{ CCI_REG8(0x579D), 0xAD },
	{ CCI_REG8(0x579E), 0x00 },
	{ CCI_REG8(0x579F), 0x48 },
	{ CCI_REG8(0x57A8), 0x02 },
	{ CCI_REG8(0x57A9), 0x4E },
	{ CCI_REG8(0x57AC), 0x00 },
	{ CCI_REG8(0x57AD), 0x4F },
	{ CCI_REG8(0x57B0), 0x00 },
	{ CCI_REG8(0x57B1), 0x98 },
	{ CCI_REG8(0x57B4), 0x06 },
	{ CCI_REG8(0x57B5), 0xB3 },
	{ CCI_REG8(0x57B6), 0x00 },
	{ CCI_REG8(0x57B7), 0x7A },
	{ CCI_REG8(0x57BA), 0x02 },
	{ CCI_REG8(0x57BB), 0x4E },
	{ CCI_REG8(0x57C2), 0x00 },
	{ CCI_REG8(0x57C3), 0x72 },
	{ CCI_REG8(0x57C4), 0x00 },
	{ CCI_REG8(0x57C5), 0x18 },
	{ CCI_REG8(0x57CE), 0x00 },
	{ CCI_REG8(0x57CF), 0xAD },
	{ CCI_REG8(0x57D0), 0x00 },
	{ CCI_REG8(0x57D1), 0x66 },
	{ CCI_REG8(0x57DE), 0x00 },
	{ CCI_REG8(0x57DF), 0x4A },
	{ CCI_REG8(0x57E2), 0x00 },
	{ CCI_REG8(0x57E3), 0x98 },
	{ CCI_REG8(0x57EC), 0x00 },
	{ CCI_REG8(0x57ED), 0x72 },
	{ CCI_REG8(0x57F8), 0x00 },
	{ CCI_REG8(0x57F9), 0xAD },
	{ CCI_REG8(0x57FA), 0x00 },
	{ CCI_REG8(0x57FB), 0x59 },
	{ CCI_REG8(0x580C), 0x00 },
	{ CCI_REG8(0x580D), 0x72 },
	{ CCI_REG8(0x5818), 0x00 },
	{ CCI_REG8(0x5819), 0xAD },
	{ CCI_REG8(0x581A), 0x00 },
	{ CCI_REG8(0x581B), 0x59 },
	{ CCI_REG8(0x582C), 0x00 },
	{ CCI_REG8(0x582D), 0x72 },
	{ CCI_REG8(0x5838), 0x00 },
	{ CCI_REG8(0x5839), 0xAD },
	{ CCI_REG8(0x583A), 0x00 },
	{ CCI_REG8(0x583B), 0x59 },
	{ CCI_REG8(0x6076), 0x01 },
	{ CCI_REG8(0x6077), 0x01 },
	{ CCI_REG8(0x6078), 0x01 },
	{ CCI_REG8(0x6079), 0x01 },
	{ CCI_REG8(0x60D5), 0x0F },
	{ CCI_REG8(0x60D6), 0x0F },
	{ CCI_REG8(0x60D7), 0x0F },
	{ CCI_REG8(0x60D8), 0x0F },
	{ CCI_REG8(0x60D9), 0x0F },
	{ CCI_REG8(0x60DA), 0x0F },
	{ CCI_REG8(0x60DB), 0x0F },
	{ CCI_REG8(0x60DC), 0x0F },
	{ CCI_REG8(0x60DD), 0x0F },
	{ CCI_REG8(0x60DE), 0x0F },
	{ CCI_REG8(0x60DF), 0x0F },
	{ CCI_REG8(0x60E0), 0x0F },
	{ CCI_REG8(0x60E1), 0x0F },
	{ CCI_REG8(0x60E2), 0x0F },
	{ CCI_REG8(0x60E3), 0x0F },
	{ CCI_REG8(0x62A2), 0x08 },
	{ CCI_REG8(0x62A3), 0x08 },
	{ CCI_REG8(0x62A5), 0x08 },
	{ CCI_REG8(0x62A7), 0x08 },
	{ CCI_REG8(0x62A9), 0x08 },
	{ CCI_REG8(0x62AB), 0x08 },
	{ CCI_REG8(0x62AD), 0x08 },
	{ CCI_REG8(0x62B1), 0x08 },
	{ CCI_REG8(0x62B2), 0x08 },
	{ CCI_REG8(0x62B3), 0x08 },
	{ CCI_REG8(0x62B4), 0x08 },
	{ CCI_REG8(0x62B5), 0x08 },
	{ CCI_REG8(0x62B6), 0x08 },
	{ CCI_REG8(0x62B7), 0x10 },
	{ CCI_REG8(0x62B8), 0x10 },
	{ CCI_REG8(0x62B9), 0x10 },
	{ CCI_REG8(0x62BA), 0x10 },
	{ CCI_REG8(0x62BB), 0x10 },
	{ CCI_REG8(0x62BC), 0x10 },
	{ CCI_REG8(0x62BD), 0x10 },
	{ CCI_REG8(0x62BE), 0x10 },
	{ CCI_REG8(0x62BF), 0x10 },
	{ CCI_REG8(0x62C0), 0x10 },
	{ CCI_REG8(0x62C1), 0x10 },
	{ CCI_REG8(0x62C2), 0x10 },
	{ CCI_REG8(0x62C3), 0x10 },
	{ CCI_REG8(0x62C4), 0x04 },
	{ CCI_REG8(0x62C5), 0x04 },
	{ CCI_REG8(0x62C6), 0x04 },
	{ CCI_REG8(0x62C7), 0x04 },
	{ CCI_REG8(0x62C8), 0x04 },
	{ CCI_REG8(0x62C9), 0x04 },
	{ CCI_REG8(0x62CA), 0x04 },
	{ CCI_REG8(0x62CB), 0x04 },
	{ CCI_REG8(0x62CC), 0x04 },
	{ CCI_REG8(0x62CD), 0x04 },
	{ CCI_REG8(0x62CE), 0x04 },
	{ CCI_REG8(0x62CF), 0x04 },
	{ CCI_REG8(0x62D0), 0x04 },
	{ CCI_REG8(0x62D1), 0x04 },
	{ CCI_REG8(0x62D2), 0x04 },
	{ CCI_REG8(0x62D3), 0x04 },
	{ CCI_REG8(0x62D4), 0x04 },
	{ CCI_REG8(0x62D5), 0x04 },
	{ CCI_REG8(0x62D6), 0x04 },
	{ CCI_REG8(0x62D7), 0x04 },
	{ CCI_REG8(0x62D8), 0x04 },
	{ CCI_REG8(0x62D9), 0x04 },
	{ CCI_REG8(0x62DA), 0x04 },
	{ CCI_REG8(0x62DB), 0x04 },
	{ CCI_REG8(0x62DC), 0x04 },
	{ CCI_REG8(0x62DD), 0x04 },
	{ CCI_REG8(0x62DE), 0x04 },
	{ CCI_REG8(0x62DF), 0x04 },
	{ CCI_REG8(0x62E0), 0x04 },
	{ CCI_REG8(0x62E1), 0x04 },
	{ CCI_REG8(0x62E2), 0x04 },
	{ CCI_REG8(0x62E3), 0x04 },
	{ CCI_REG8(0x62E4), 0x04 },
	{ CCI_REG8(0x62E5), 0x04 },
	{ CCI_REG8(0x62E6), 0x05 },
	{ CCI_REG8(0x62E7), 0x05 },
	{ CCI_REG8(0x62E8), 0x05 },
	{ CCI_REG8(0x62E9), 0x05 },
	{ CCI_REG8(0x62EA), 0x05 },
	{ CCI_REG8(0x62EB), 0x05 },
	{ CCI_REG8(0x62EC), 0x05 },
	{ CCI_REG8(0x62ED), 0x05 },
	{ CCI_REG8(0x62EE), 0x05 },
	{ CCI_REG8(0x62EF), 0x05 },
	{ CCI_REG8(0x62F0), 0x05 },
	{ CCI_REG8(0x62F1), 0x05 },
	{ CCI_REG8(0x62F2), 0x05 },
	{ CCI_REG8(0x62F3), 0x01 },
	{ CCI_REG8(0x62F4), 0x01 },
	{ CCI_REG8(0x62F5), 0x01 },
	{ CCI_REG8(0x62F6), 0x01 },
	{ CCI_REG8(0x62F7), 0x01 },
	{ CCI_REG8(0x62F8), 0x01 },
	{ CCI_REG8(0x62F9), 0x01 },
	{ CCI_REG8(0x62FA), 0x01 },
	{ CCI_REG8(0x62FB), 0x01 },
	{ CCI_REG8(0x62FC), 0x01 },
	{ CCI_REG8(0x62FD), 0x01 },
	{ CCI_REG8(0x62FE), 0x01 },
	{ CCI_REG8(0x62FF), 0x01 },
	{ CCI_REG8(0x6300), 0x01 },
	{ CCI_REG8(0x6301), 0x01 },
	{ CCI_REG8(0x6302), 0x01 },
	{ CCI_REG8(0x6303), 0x01 },
	{ CCI_REG8(0x6304), 0x01 },
	{ CCI_REG8(0x6305), 0x01 },
	{ CCI_REG8(0x6306), 0x01 },
	{ CCI_REG8(0x6307), 0x01 },
	{ CCI_REG8(0x6308), 0x01 },
	{ CCI_REG8(0x6309), 0x01 },
	{ CCI_REG8(0x630A), 0x01 },
	{ CCI_REG8(0x630B), 0x01 },
	{ CCI_REG8(0x630C), 0x01 },
	{ CCI_REG8(0x630D), 0x01 },
	{ CCI_REG8(0x630E), 0x01 },
	{ CCI_REG8(0x630F), 0x01 },
	{ CCI_REG8(0x6310), 0x01 },
	{ CCI_REG8(0x6311), 0x01 },
	{ CCI_REG8(0x6312), 0x01 },
	{ CCI_REG8(0x6313), 0x01 },
	{ CCI_REG8(0x6314), 0x01 },
	{ CCI_REG8(0x6315), 0x01 },
	{ CCI_REG8(0x6316), 0x01 },
	{ CCI_REG8(0x6317), 0x01 },
	{ CCI_REG8(0x6318), 0x01 },
	{ CCI_REG8(0x6319), 0x01 },
	{ CCI_REG8(0x631A), 0x01 },
	{ CCI_REG8(0x631B), 0x01 },
	{ CCI_REG8(0x631C), 0x01 },
	{ CCI_REG8(0x631D), 0x01 },
	{ CCI_REG8(0x631E), 0x01 },
	{ CCI_REG8(0x631F), 0x01 },
	{ CCI_REG8(0x6320), 0x01 },
	{ CCI_REG8(0x6321), 0x01 },
	{ CCI_REG8(0x6366), 0x00 },
	{ CCI_REG8(0x6367), 0x55 },
	{ CCI_REG8(0x6368), 0x00 },
	{ CCI_REG8(0x6369), 0x55 },
	{ CCI_REG8(0x636A), 0x00 },
	{ CCI_REG8(0x636B), 0x55 },
	{ CCI_REG8(0x636C), 0x00 },
	{ CCI_REG8(0x636D), 0x55 },
	{ CCI_REG8(0x636E), 0x00 },
	{ CCI_REG8(0x636F), 0x55 },
	{ CCI_REG8(0x6370), 0x00 },
	{ CCI_REG8(0x6371), 0x55 },
	{ CCI_REG8(0x6372), 0x00 },
	{ CCI_REG8(0x6373), 0x55 },
	{ CCI_REG8(0x6374), 0x00 },
	{ CCI_REG8(0x6375), 0x55 },
	{ CCI_REG8(0x6376), 0x00 },
	{ CCI_REG8(0x6377), 0x55 },
	{ CCI_REG8(0x6378), 0x00 },
	{ CCI_REG8(0x6379), 0x55 },
	{ CCI_REG8(0x637A), 0x00 },
	{ CCI_REG8(0x637B), 0x55 },
	{ CCI_REG8(0x637C), 0x00 },
	{ CCI_REG8(0x637D), 0x55 },
	{ CCI_REG8(0x637E), 0x00 },
	{ CCI_REG8(0x637F), 0x55 },
	{ CCI_REG8(0x63C4), 0x00 },
	{ CCI_REG8(0x63C5), 0xBC },
	{ CCI_REG8(0x63C6), 0x00 },
	{ CCI_REG8(0x63C7), 0xBC },
	{ CCI_REG8(0x63C8), 0x00 },
	{ CCI_REG8(0x63C9), 0xBC },
	{ CCI_REG8(0x63CA), 0x00 },
	{ CCI_REG8(0x63CB), 0xBC },
	{ CCI_REG8(0x63CC), 0x00 },
	{ CCI_REG8(0x63CD), 0xBC },
	{ CCI_REG8(0x63CE), 0x00 },
	{ CCI_REG8(0x63CF), 0xBC },
	{ CCI_REG8(0x63D0), 0x00 },
	{ CCI_REG8(0x63D1), 0xBC },
	{ CCI_REG8(0x63D2), 0x00 },
	{ CCI_REG8(0x63D3), 0xBC },
	{ CCI_REG8(0x63D4), 0x00 },
	{ CCI_REG8(0x63D5), 0xBC },
	{ CCI_REG8(0x63D6), 0x00 },
	{ CCI_REG8(0x63D7), 0xBC },
	{ CCI_REG8(0x63D8), 0x00 },
	{ CCI_REG8(0x63D9), 0xBC },
	{ CCI_REG8(0x63DA), 0x00 },
	{ CCI_REG8(0x63DB), 0xBC },
	{ CCI_REG8(0x63DC), 0x00 },
	{ CCI_REG8(0x63DD), 0xBC },
	{ CCI_REG8(0x63EA), 0x00 },
	{ CCI_REG8(0x63EB), 0xBC },
	{ CCI_REG8(0x63EC), 0x00 },
	{ CCI_REG8(0x63ED), 0xBC },
	{ CCI_REG8(0x63EE), 0x19 },
	{ CCI_REG8(0x63EF), 0x19 },
	{ CCI_REG8(0x63F0), 0x19 },
	{ CCI_REG8(0x63F1), 0x19 },
	{ CCI_REG8(0x63F2), 0x19 },
	{ CCI_REG8(0x63F3), 0x19 },
	{ CCI_REG8(0x63F4), 0x19 },
	{ CCI_REG8(0x63F5), 0x19 },
	{ CCI_REG8(0x63F6), 0x19 },
	{ CCI_REG8(0x63F7), 0x19 },
	{ CCI_REG8(0x63F8), 0x19 },
	{ CCI_REG8(0x63F9), 0x19 },
	{ CCI_REG8(0x63FA), 0x19 },
	{ CCI_REG8(0x63FB), 0x19 },
	{ CCI_REG8(0x63FC), 0x19 },
	{ CCI_REG8(0x63FD), 0x19 },
	{ CCI_REG8(0x63FE), 0x19 },
	{ CCI_REG8(0x63FF), 0x19 },
	{ CCI_REG8(0x6400), 0x19 },
	{ CCI_REG8(0x6401), 0x19 },
	{ CCI_REG8(0x6402), 0x19 },
	{ CCI_REG8(0x6403), 0x19 },
	{ CCI_REG8(0x6404), 0x19 },
	{ CCI_REG8(0x6405), 0x19 },
	{ CCI_REG8(0x6406), 0x19 },
	{ CCI_REG8(0x6407), 0x19 },
	{ CCI_REG8(0x6408), 0x19 },
	{ CCI_REG8(0x6409), 0x19 },
	{ CCI_REG8(0x640A), 0x19 },
	{ CCI_REG8(0x640B), 0x19 },
	{ CCI_REG8(0x640C), 0x19 },
	{ CCI_REG8(0x640D), 0x19 },
	{ CCI_REG8(0x640E), 0x19 },
	{ CCI_REG8(0x640F), 0x19 },
	{ CCI_REG8(0x6410), 0x19 },
	{ CCI_REG8(0x6411), 0x19 },
	{ CCI_REG8(0x6412), 0x19 },
	{ CCI_REG8(0x6413), 0x19 },
	{ CCI_REG8(0x6414), 0x19 },
	{ CCI_REG8(0x6415), 0x19 },
	{ CCI_REG8(0x6416), 0x19 },
	{ CCI_REG8(0x6417), 0x19 },
	{ CCI_REG8(0x6418), 0x19 },
	{ CCI_REG8(0x6419), 0x19 },
	{ CCI_REG8(0x641A), 0x19 },
	{ CCI_REG8(0x641B), 0x19 },
	{ CCI_REG8(0x641C), 0x19 },
	{ CCI_REG8(0x644A), 0x00 },
	{ CCI_REG8(0x644B), 0x21 },
	{ CCI_REG8(0x644C), 0x00 },
	{ CCI_REG8(0x644D), 0x1D },
	{ CCI_REG8(0x644E), 0x00 },
	{ CCI_REG8(0x644F), 0x09 },
	{ CCI_REG8(0x6450), 0x00 },
	{ CCI_REG8(0x6451), 0x18 },
	{ CCI_REG8(0x6452), 0x00 },
	{ CCI_REG8(0x6453), 0x06 },
	{ CCI_REG8(0x6454), 0x00 },
	{ CCI_REG8(0x6455), 0x18 },
	{ CCI_REG8(0x6456), 0x00 },
	{ CCI_REG8(0x6457), 0x06 },
	{ CCI_REG8(0x6462), 0x00 },
	{ CCI_REG8(0x6463), 0x21 },
	{ CCI_REG8(0x6464), 0x00 },
	{ CCI_REG8(0x6465), 0x1E },
	{ CCI_REG8(0x6466), 0x00 },
	{ CCI_REG8(0x6467), 0x1C },
	{ CCI_REG8(0x6468), 0x00 },
	{ CCI_REG8(0x6469), 0x18 },
	{ CCI_REG8(0x646A), 0x00 },
	{ CCI_REG8(0x646B), 0x17 },
	{ CCI_REG8(0x646C), 0x00 },
	{ CCI_REG8(0x646D), 0x18 },
	{ CCI_REG8(0x646E), 0x00 },
	{ CCI_REG8(0x646F), 0x17 },
	{ CCI_REG8(0x647A), 0x00 },
	{ CCI_REG8(0x647B), 0x1D },
	{ CCI_REG8(0x647C), 0x00 },
	{ CCI_REG8(0x647D), 0x18 },
	{ CCI_REG8(0x647E), 0x00 },
	{ CCI_REG8(0x647F), 0x18 },
	{ CCI_REG8(0x6486), 0x00 },
	{ CCI_REG8(0x6487), 0x1E },
	{ CCI_REG8(0x6488), 0x00 },
	{ CCI_REG8(0x6489), 0x18 },
	{ CCI_REG8(0x648A), 0x00 },
	{ CCI_REG8(0x648B), 0x18 },
	{ CCI_REG8(0x648C), 0x00 },
	{ CCI_REG8(0x648D), 0x38 },
	{ CCI_REG8(0x648E), 0x00 },
	{ CCI_REG8(0x648F), 0x32 },
	{ CCI_REG8(0x6490), 0x00 },
	{ CCI_REG8(0x6491), 0x31 },
	{ CCI_REG8(0x6492), 0x00 },
	{ CCI_REG8(0x6493), 0x29 },
	{ CCI_REG8(0x6494), 0x00 },
	{ CCI_REG8(0x6495), 0x29 },
	{ CCI_REG8(0x6496), 0x00 },
	{ CCI_REG8(0x6497), 0x2A },
	{ CCI_REG8(0x6498), 0x00 },
	{ CCI_REG8(0x6499), 0x29 },
	{ CCI_REG8(0x64A4), 0x08 },
	{ CCI_REG8(0x64A5), 0x08 },
	{ CCI_REG8(0x64A6), 0x01 },
	{ CCI_REG8(0x64A7), 0x07 },
	{ CCI_REG8(0x64A8), 0x00 },
	{ CCI_REG8(0x64A9), 0x05 },
	{ CCI_REG8(0x64AA), 0x00 },
	{ CCI_REG8(0x64B0), 0x08 },
	{ CCI_REG8(0x64B1), 0x07 },
	{ CCI_REG8(0x64B2), 0x07 },
	{ CCI_REG8(0x64B3), 0x07 },
	{ CCI_REG8(0x64B4), 0x07 },
	{ CCI_REG8(0x64B5), 0x05 },
	{ CCI_REG8(0x64B6), 0x05 },
	{ CCI_REG8(0x64BC), 0x07 },
	{ CCI_REG8(0x64BD), 0x07 },
	{ CCI_REG8(0x64BE), 0x05 },
	{ CCI_REG8(0x64C2), 0x07 },
	{ CCI_REG8(0x64C3), 0x07 },
	{ CCI_REG8(0x64C4), 0x05 },
	{ CCI_REG8(0x64C5), 0x0D },
	{ CCI_REG8(0x64C6), 0x0C },
	{ CCI_REG8(0x64C7), 0x0C },
	{ CCI_REG8(0x64C8), 0x0C },
	{ CCI_REG8(0x64C9), 0x0C },
	{ CCI_REG8(0x64CA), 0x08 },
	{ CCI_REG8(0x64CB), 0x08 },
	{ CCI_REG8(0x6558), 0x01 },
	{ CCI_REG8(0x6559), 0x01 },
	{ CCI_REG8(0x655A), 0x01 },
	{ CCI_REG8(0x655B), 0x01 },
	{ CCI_REG8(0x655C), 0x01 },
	{ CCI_REG8(0x655D), 0x01 },
	{ CCI_REG8(0x655E), 0x01 },
	{ CCI_REG8(0x655F), 0x01 },
	{ CCI_REG8(0x6560), 0x01 },
	{ CCI_REG8(0x6561), 0x01 },
	{ CCI_REG8(0x6562), 0x01 },
	{ CCI_REG8(0x6563), 0x01 },
	{ CCI_REG8(0x6564), 0x01 },
	{ CCI_REG8(0x6565), 0x01 },
	{ CCI_REG8(0x6566), 0x01 },
	{ CCI_REG8(0x6567), 0x01 },
	{ CCI_REG8(0x6568), 0x01 },
	{ CCI_REG8(0x6569), 0x01 },
	{ CCI_REG8(0x656A), 0x01 },
	{ CCI_REG8(0x656B), 0x01 },
	{ CCI_REG8(0x656C), 0x01 },
	{ CCI_REG8(0x656D), 0x01 },
	{ CCI_REG8(0x656E), 0x01 },
	{ CCI_REG8(0x656F), 0x01 },
	{ CCI_REG8(0x6570), 0x01 },
	{ CCI_REG8(0x6571), 0x01 },
	{ CCI_REG8(0x6572), 0x01 },
	{ CCI_REG8(0x6573), 0x01 },
	{ CCI_REG8(0x6574), 0x01 },
	{ CCI_REG8(0x6575), 0x01 },
	{ CCI_REG8(0x6576), 0x01 },
	{ CCI_REG8(0x6577), 0x01 },
	{ CCI_REG8(0x6578), 0x01 },
	{ CCI_REG8(0x6579), 0x01 },
	{ CCI_REG8(0x657A), 0x01 },
	{ CCI_REG8(0x657B), 0x01 },
	{ CCI_REG8(0x657C), 0x01 },
	{ CCI_REG8(0x657D), 0x01 },
	{ CCI_REG8(0x657E), 0x01 },
	{ CCI_REG8(0x657F), 0x01 },
	{ CCI_REG8(0x658F), 0x07 },
	{ CCI_REG8(0x6590), 0x05 },
	{ CCI_REG8(0x6591), 0x07 },
	{ CCI_REG8(0x6592), 0x05 },
	{ CCI_REG8(0x6593), 0x07 },
	{ CCI_REG8(0x6594), 0x05 },
	{ CCI_REG8(0x6595), 0x07 },
	{ CCI_REG8(0x6596), 0x05 },
	{ CCI_REG8(0x6597), 0x05 },
	{ CCI_REG8(0x6598), 0x05 },
	{ CCI_REG8(0x6599), 0x05 },
	{ CCI_REG8(0x659A), 0x05 },
	{ CCI_REG8(0x659B), 0x05 },
	{ CCI_REG8(0x659C), 0x05 },
	{ CCI_REG8(0x659D), 0x05 },
	{ CCI_REG8(0x659E), 0x07 },
	{ CCI_REG8(0x659F), 0x05 },
	{ CCI_REG8(0x65A0), 0x07 },
	{ CCI_REG8(0x65A1), 0x05 },
	{ CCI_REG8(0x65A2), 0x07 },
	{ CCI_REG8(0x65A3), 0x05 },
	{ CCI_REG8(0x65A4), 0x07 },
	{ CCI_REG8(0x65A5), 0x05 },
	{ CCI_REG8(0x65A6), 0x05 },
	{ CCI_REG8(0x65A7), 0x05 },
	{ CCI_REG8(0x65A8), 0x05 },
	{ CCI_REG8(0x65A9), 0x05 },
	{ CCI_REG8(0x65AA), 0x05 },
	{ CCI_REG8(0x65AB), 0x05 },
	{ CCI_REG8(0x65AC), 0x05 },
	{ CCI_REG8(0x65AD), 0x07 },
	{ CCI_REG8(0x65AE), 0x07 },
	{ CCI_REG8(0x65AF), 0x07 },
	{ CCI_REG8(0x65B0), 0x05 },
	{ CCI_REG8(0x65B1), 0x05 },
	{ CCI_REG8(0x65B2), 0x05 },
	{ CCI_REG8(0x65B3), 0x05 },
	{ CCI_REG8(0x65B4), 0x07 },
	{ CCI_REG8(0x65B5), 0x07 },
	{ CCI_REG8(0x65B6), 0x07 },
	{ CCI_REG8(0x65B7), 0x07 },
	{ CCI_REG8(0x65B8), 0x05 },
	{ CCI_REG8(0x65B9), 0x05 },
	{ CCI_REG8(0x65BA), 0x05 },
	{ CCI_REG8(0x65BB), 0x05 },
	{ CCI_REG8(0x65BC), 0x05 },
	{ CCI_REG8(0x65BD), 0x05 },
	{ CCI_REG8(0x65BE), 0x05 },
	{ CCI_REG8(0x65BF), 0x05 },
	{ CCI_REG8(0x65C0), 0x05 },
	{ CCI_REG8(0x65C1), 0x05 },
	{ CCI_REG8(0x65C2), 0x05 },
	{ CCI_REG8(0x65C3), 0x05 },
	{ CCI_REG8(0x65C4), 0x05 },
	{ CCI_REG8(0x65C5), 0x05 },
	{ CCI_REG8(0x65C6), 0x00 },
	{ CCI_REG8(0x65C7), 0x34 },
	{ CCI_REG8(0x65C8), 0x1E },
	{ CCI_REG8(0x65C9), 0x28 },
	{ CCI_REG8(0x65CA), 0x14 },
	{ CCI_REG8(0x65CB), 0x18 },
	{ CCI_REG8(0x65CC), 0x0F },
	{ CCI_REG8(0x65CE), 0x2C },
	{ CCI_REG8(0x65D0), 0x1A },
	{ CCI_REG8(0x65D2), 0x30 },
	{ CCI_REG8(0x65D4), 0x34 },
	{ CCI_REG8(0x65D5), 0x00 },
	{ CCI_REG8(0x65D6), 0x46 },
	{ CCI_REG8(0x65D7), 0x37 },
	{ CCI_REG8(0x65D8), 0x3C },
	{ CCI_REG8(0x65D9), 0x28 },
	{ CCI_REG8(0x65DA), 0x28 },
	{ CCI_REG8(0x65DB), 0x23 },
	{ CCI_REG8(0x65DD), 0x41 },
	{ CCI_REG8(0x65DF), 0x2D },
	{ CCI_REG8(0x65E1), 0x46 },
	{ CCI_REG8(0x65E3), 0x4B },
	{ CCI_REG8(0x65E4), 0x00 },
	{ CCI_REG8(0x65E5), 0x4B },
	{ CCI_REG8(0x65E6), 0x4B },
	{ CCI_REG8(0x65E7), 0x41 },
	{ CCI_REG8(0x65E8), 0x41 },
	{ CCI_REG8(0x65E9), 0x2D },
	{ CCI_REG8(0x65EA), 0x2D },
	{ CCI_REG8(0x65EC), 0x3C },
	{ CCI_REG8(0x65EE), 0x28 },
	{ CCI_REG8(0x65F0), 0x41 },
	{ CCI_REG8(0x65F2), 0x46 },
	{ CCI_REG8(0x65F3), 0x00 },
	{ CCI_REG8(0x65F4), 0x00 },
	{ CCI_REG8(0x65F5), 0x00 },
	{ CCI_REG8(0x65F6), 0x00 },
	{ CCI_REG8(0x65F7), 0x00 },
	{ CCI_REG8(0x65F8), 0x00 },
	{ CCI_REG8(0x65F9), 0x00 },
	{ CCI_REG8(0x65FB), 0x00 },
	{ CCI_REG8(0x65FD), 0x00 },
	{ CCI_REG8(0x65FF), 0x00 },
	{ CCI_REG8(0x6601), 0x00 },
	{ CCI_REG8(0x6E1C), 0x00 },
	{ CCI_REG8(0x6E1D), 0x00 },
	{ CCI_REG8(0x6E25), 0x00 },
	{ CCI_REG8(0x6E38), 0x03 },
	{ CCI_REG8(0x7529), 0x07 },
	{ CCI_REG8(0x752A), 0x08 },
	{ CCI_REG8(0x895C), 0x01 },
	{ CCI_REG8(0x895D), 0x00 },
	{ CCI_REG8(0x8966), 0x00 },
	{ CCI_REG8(0x8967), 0x4E },
	{ CCI_REG8(0x896A), 0x00 },
	{ CCI_REG8(0x896B), 0x24 },
	{ CCI_REG8(0x896F), 0x34 },
	{ CCI_REG8(0x8976), 0x00 },
	{ CCI_REG8(0x8977), 0x00 },
	{ CCI_REG8(0x9004), 0x1F },
	{ CCI_REG8(0x9200), 0xB7 },
	{ CCI_REG8(0x9201), 0x34 },
	{ CCI_REG8(0x9202), 0xB7 },
	{ CCI_REG8(0x9203), 0x36 },
	{ CCI_REG8(0x9204), 0xB7 },
	{ CCI_REG8(0x9205), 0x37 },
	{ CCI_REG8(0x9206), 0xB7 },
	{ CCI_REG8(0x9207), 0x38 },
	{ CCI_REG8(0x9208), 0xB7 },
	{ CCI_REG8(0x9209), 0x39 },
	{ CCI_REG8(0x920A), 0xB7 },
	{ CCI_REG8(0x920B), 0x3A },
	{ CCI_REG8(0x920C), 0xB7 },
	{ CCI_REG8(0x920D), 0x3C },
	{ CCI_REG8(0x920E), 0xB7 },
	{ CCI_REG8(0x920F), 0x3D },
	{ CCI_REG8(0x9210), 0xB7 },
	{ CCI_REG8(0x9211), 0x3E },
	{ CCI_REG8(0x9212), 0xB7 },
	{ CCI_REG8(0x9213), 0x3F },
	{ CCI_REG8(0x9214), 0xF6 },
	{ CCI_REG8(0x9215), 0x13 },
	{ CCI_REG8(0x9216), 0xF6 },
	{ CCI_REG8(0x9217), 0x34 },
	{ CCI_REG8(0x9218), 0xF4 },
	{ CCI_REG8(0x9219), 0xA7 },
	{ CCI_REG8(0x921A), 0xF4 },
	{ CCI_REG8(0x921B), 0xAA },
	{ CCI_REG8(0x921C), 0xF4 },
	{ CCI_REG8(0x921D), 0xAD },
	{ CCI_REG8(0x921E), 0xF4 },
	{ CCI_REG8(0x921F), 0xB0 },
	{ CCI_REG8(0x9220), 0xF4 },
	{ CCI_REG8(0x9221), 0xB3 },
	{ CCI_REG8(0x9222), 0x85 },
	{ CCI_REG8(0x9223), 0x77 },
	{ CCI_REG8(0x9224), 0xC4 },
	{ CCI_REG8(0x9225), 0x4B },
	{ CCI_REG8(0x9226), 0xC4 },
	{ CCI_REG8(0x9227), 0x4C },
	{ CCI_REG8(0x9228), 0xC4 },
	{ CCI_REG8(0x9229), 0x4D },
	{ CCI_REG8(0x922A), 0xF5 },
	{ CCI_REG8(0x922B), 0x5E },
	{ CCI_REG8(0x922C), 0xF5 },
	{ CCI_REG8(0x922D), 0x5F },
	{ CCI_REG8(0x922E), 0xF5 },
	{ CCI_REG8(0x922F), 0x64 },
	{ CCI_REG8(0x9230), 0xF5 },
	{ CCI_REG8(0x9231), 0x65 },
	{ CCI_REG8(0x9232), 0xF5 },
	{ CCI_REG8(0x9233), 0x6A },
	{ CCI_REG8(0x9234), 0xF5 },
	{ CCI_REG8(0x9235), 0x6B },
	{ CCI_REG8(0x9236), 0xF5 },
	{ CCI_REG8(0x9237), 0x70 },
	{ CCI_REG8(0x9238), 0xF5 },
	{ CCI_REG8(0x9239), 0x71 },
	{ CCI_REG8(0x923A), 0xF5 },
	{ CCI_REG8(0x923B), 0x76 },
	{ CCI_REG8(0x923C), 0xF5 },
	{ CCI_REG8(0x923D), 0x77 },
	{ CCI_REG8(0x9360), 0x00 },
	{ CCI_REG8(0x9361), 0x00 },
	{ CCI_REG8(0x9362), 0xD8 },
	{ CCI_REG8(0x9363), 0x00 },
	{ CCI_REG8(0x9810), 0x14 },
	{ CCI_REG8(0x9814), 0x14 },
	{ CCI_REG8(0xB074), 0xF0 },
	{ CCI_REG8(0xBC7C), 0x11 },
	{ CCI_REG8(0xBC7D), 0x04 },
	{ CCI_REG8(0xBC7E), 0x07 },
	{ CCI_REG8(0xBC7F), 0xB8 },
	{ CCI_REG8(0xC020), 0x00 },
	{ CCI_REG8(0xC026), 0x00 },
	{ CCI_REG8(0xC027), 0x00 },
	{ CCI_REG8(0xC448), 0x01 },
	{ CCI_REG8(0xC44F), 0x01 },
	{ CCI_REG8(0xC450), 0x00 },
	{ CCI_REG8(0xC451), 0x00 },
	{ CCI_REG8(0xC452), 0x01 },
	{ CCI_REG8(0xC455), 0x00 },
	{ CCI_REG8(0xE186), 0x2B },
	{ CCI_REG8(0xE206), 0x35 },
	{ CCI_REG8(0xE226), 0x32 },
	{ CCI_REG8(0xE266), 0x32 },
	{ CCI_REG8(0xE2A6), 0x32 },
	{ CCI_REG8(0xE2C6), 0x31 },
	{ CCI_REG8(0xE2E6), 0x31 },
	{ CCI_REG8(0xEC00), 0x01 },
	{ CCI_REG8(0x3E14), 0x01 },
	{ CCI_REG8(0xE013), 0x01 },
	{ CCI_REG8(0x88D6), 0x60 },
	{ CCI_REG8(0x9852), 0x00 },
	{ CCI_REG8(0xA569), 0x06 },
	{ CCI_REG8(0xA56A), 0x13 },
	{ CCI_REG8(0xA56B), 0x13 },
	{ CCI_REG8(0xA56C), 0x01 },
	{ CCI_REG8(0xA678), 0x00 },
	{ CCI_REG8(0xA679), 0x20 },
	{ CCI_REG8(0xA812), 0x00 },
	{ CCI_REG8(0xA813), 0x3F },
	{ CCI_REG8(0xA814), 0x3F },
	{ CCI_REG8(0xA830), 0x68 },
	{ CCI_REG8(0xA831), 0x56 },
	{ CCI_REG8(0xA832), 0x2B },
	{ CCI_REG8(0xA833), 0x55 },
	{ CCI_REG8(0xA834), 0x55 },
	{ CCI_REG8(0xA835), 0x16 },
	{ CCI_REG8(0xA837), 0x51 },
	{ CCI_REG8(0xA838), 0x34 },
	{ CCI_REG8(0xA854), 0x4F },
	{ CCI_REG8(0xA855), 0x48 },
	{ CCI_REG8(0xA856), 0x45 },
	{ CCI_REG8(0xA857), 0x02 },
	{ CCI_REG8(0xA85A), 0x23 },
	{ CCI_REG8(0xA85B), 0x16 },
	{ CCI_REG8(0xA85C), 0x12 },
	{ CCI_REG8(0xA85D), 0x02 },
	{ CCI_REG8(0xAA55), 0x00 },
	{ CCI_REG8(0xAA56), 0x01 },
	{ CCI_REG8(0xAA57), 0x30 },
	{ CCI_REG8(0xAA58), 0x01 },
	{ CCI_REG8(0xAA59), 0x30 },
	{ CCI_REG8(0xAC72), 0x01 },
	{ CCI_REG8(0xAC73), 0x26 },
	{ CCI_REG8(0xAC74), 0x01 },
	{ CCI_REG8(0xAC75), 0x26 },
	{ CCI_REG8(0xAC76), 0x00 },
	{ CCI_REG8(0xAC77), 0xC4 },
	{ CCI_REG8(0xAE09), 0xFF },
	{ CCI_REG8(0xAE0A), 0xFF },
	{ CCI_REG8(0xAE12), 0x58 },
	{ CCI_REG8(0xAE13), 0x58 },
	{ CCI_REG8(0xAE15), 0x10 },
	{ CCI_REG8(0xAE16), 0x10 },
	{ CCI_REG8(0xAF05), 0x48 },
	{ CCI_REG8(0xB069), 0x02 },
	{ CCI_REG8(0xEA4B), 0x00 },
	{ CCI_REG8(0xEA4C), 0x00 },
	{ CCI_REG8(0xEA4D), 0x00 },
	{ CCI_REG8(0xEA4E), 0x00 },
};

static const struct cci_reg_sequence imx616_mode_regs[] = {
	{ CCI_REG8(0x0112), 0x0A },
	{ CCI_REG8(0x0113), 0x0A },
	{ CCI_REG8(0x0114), 0x03 },
	{ CCI_REG8(0x0342), 0x1D },
	{ CCI_REG8(0x0343), 0x70 },
	{ CCI_REG8(0x0340), 0x09 },
	{ CCI_REG8(0x0341), 0xD6 },
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x00 },
	{ CCI_REG8(0x0348), 0x19 },
	{ CCI_REG8(0x0349), 0x9F },
	{ CCI_REG8(0x034A), 0x13 },
	{ CCI_REG8(0x034B), 0x3F },
	{ CCI_REG8(0x0220), 0x62 },
	{ CCI_REG8(0x0222), 0x01 },
	{ CCI_REG8(0x0900), 0x01 },
	{ CCI_REG8(0x0901), 0x22 },
	{ CCI_REG8(0x0902), 0x08 },
	{ CCI_REG8(0x3140), 0x00 },
	{ CCI_REG8(0x3246), 0x81 },
	{ CCI_REG8(0x3247), 0x81 },
	{ CCI_REG8(0x0401), 0x00 },
	{ CCI_REG8(0x0404), 0x00 },
	{ CCI_REG8(0x0405), 0x10 },
	{ CCI_REG8(0x0408), 0x00 },
	{ CCI_REG8(0x0409), 0x08 },
	{ CCI_REG8(0x040A), 0x00 },
	{ CCI_REG8(0x040B), 0x08 },
	{ CCI_REG8(0x040C), 0x0C },
	{ CCI_REG8(0x040D), 0xC0 },
	{ CCI_REG8(0x040E), 0x09 },
	{ CCI_REG8(0x040F), 0x90 },
	{ CCI_REG8(0x034C), 0x0C },
	{ CCI_REG8(0x034D), 0xC0 },
	{ CCI_REG8(0x034E), 0x09 },
	{ CCI_REG8(0x034F), 0x90 },
	{ CCI_REG8(0x0301), 0x05 },
	{ CCI_REG8(0x0303), 0x02 },
	{ CCI_REG8(0x0305), 0x04 },
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0307), 0xED },
	{ CCI_REG8(0x030B), 0x01 },
	{ CCI_REG8(0x030D), 0x03 },
	{ CCI_REG8(0x030E), 0x01 },
	{ CCI_REG8(0x030F), 0x01 },
	{ CCI_REG8(0x0310), 0x01 },
	{ CCI_REG8(0x3620), 0x00 },
	{ CCI_REG8(0x3621), 0x00 },
	{ CCI_REG8(0x3C12), 0x56 },
	{ CCI_REG8(0x3C13), 0x52 },
	{ CCI_REG8(0x3C14), 0x3E },
	{ CCI_REG8(0x3F0C), 0x00 },
	{ CCI_REG8(0x3F14), 0x01 },
	{ CCI_REG8(0x3F80), 0x04 },
	{ CCI_REG8(0x3F81), 0x73 },
	{ CCI_REG8(0x3F8C), 0x00 },
	{ CCI_REG8(0x3F8D), 0x00 },
	{ CCI_REG8(0x3FFC), 0x00 },
	{ CCI_REG8(0x3FFD), 0x21 },
	{ CCI_REG8(0x3FFE), 0x00 },
	{ CCI_REG8(0x3FFF), 0x4C },
	{ CCI_REG8(0xBCF1), 0x00 },
	{ CCI_REG8(0x0202), 0x09 },
	{ CCI_REG8(0x0203), 0xA6 },
	{ CCI_REG8(0x0224), 0x01 },
	{ CCI_REG8(0x0225), 0xF4 },
	{ CCI_REG8(0x3FE0), 0x01 },
	{ CCI_REG8(0x3FE1), 0xF4 },
	{ CCI_REG8(0x0204), 0x00 },
	{ CCI_REG8(0x0205), 0x70 },
	{ CCI_REG8(0x0216), 0x00 },
	{ CCI_REG8(0x0217), 0x70 },
	{ CCI_REG8(0x0218), 0x01 },
	{ CCI_REG8(0x0219), 0x00 },
	{ CCI_REG8(0x020E), 0x01 },
	{ CCI_REG8(0x020F), 0x00 },
	{ CCI_REG8(0x0210), 0x01 },
	{ CCI_REG8(0x0211), 0x00 },
	{ CCI_REG8(0x0212), 0x01 },
	{ CCI_REG8(0x0213), 0x00 },
	{ CCI_REG8(0x0214), 0x01 },
	{ CCI_REG8(0x0215), 0x00 },
	{ CCI_REG8(0x3FE2), 0x00 },
	{ CCI_REG8(0x3FE3), 0x70 },
	{ CCI_REG8(0x3FE4), 0x01 },
	{ CCI_REG8(0x3FE5), 0x00 },
	{ CCI_REG8(0x0B00), 0x00 },
	{ CCI_REG8(0x323B), 0x00 },
	{ CCI_REG8(0x323D), 0x00 },
	{ CCI_REG8(0x323C), 0x00 },
	{ CCI_REG8(0x323E), 0x00 },
	{ CCI_REG8(0x323F), 0x00 },
	{ CCI_REG8(0x0B06), 0x00 },
};

/* setB REG_F: full remosaic 6528x4896, PLL patched for the 24 MHz EXTCLK (see the top) */
static const struct cci_reg_sequence imx616_full_regs[] = {
	{ CCI_REG8(0x0112), 0x0A },
	{ CCI_REG8(0x0113), 0x0A },
	{ CCI_REG8(0x0114), 0x03 },
	{ CCI_REG8(0x0342), 0x1C },
	{ CCI_REG8(0x0343), 0x18 },
	{ CCI_REG8(0x0340), 0x13 },
	{ CCI_REG8(0x0341), 0x88 },
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x00 },
	{ CCI_REG8(0x0348), 0x19 },
	{ CCI_REG8(0x0349), 0x9F },
	{ CCI_REG8(0x034A), 0x13 },
	{ CCI_REG8(0x034B), 0x3F },
	{ CCI_REG8(0x0220), 0x62 },
	{ CCI_REG8(0x0222), 0x01 },
	{ CCI_REG8(0x0900), 0x00 },
	{ CCI_REG8(0x0901), 0x11 },
	{ CCI_REG8(0x0902), 0x0A },
	{ CCI_REG8(0x3140), 0x00 },
	{ CCI_REG8(0x3246), 0x01 },
	{ CCI_REG8(0x3247), 0x01 },
	{ CCI_REG8(0x0401), 0x00 },
	{ CCI_REG8(0x0404), 0x00 },
	{ CCI_REG8(0x0405), 0x10 },
	{ CCI_REG8(0x0408), 0x00 },
	{ CCI_REG8(0x0409), 0x10 },
	{ CCI_REG8(0x040A), 0x00 },
	{ CCI_REG8(0x040B), 0x10 },
	{ CCI_REG8(0x040C), 0x19 },
	{ CCI_REG8(0x040D), 0x80 },
	{ CCI_REG8(0x040E), 0x13 },
	{ CCI_REG8(0x040F), 0x20 },
	{ CCI_REG8(0x034C), 0x19 },
	{ CCI_REG8(0x034D), 0x80 },
	{ CCI_REG8(0x034E), 0x13 },
	{ CCI_REG8(0x034F), 0x20 },
	{ CCI_REG8(0x0301), 0x05 },
	{ CCI_REG8(0x0303), 0x02 },
	{ CCI_REG8(0x0305), 0x04 },
	{ CCI_REG8(0x0306), 0x01 },
	{ CCI_REG8(0x0307), 0x68 },
	{ CCI_REG8(0x030B), 0x01 },
	{ CCI_REG8(0x030D), 0x03 },
	{ CCI_REG8(0x030E), 0x01 },
	{ CCI_REG8(0x030F), 0x15 },
	{ CCI_REG8(0x0310), 0x01 },
	{ CCI_REG8(0x3620), 0x01 },
	{ CCI_REG8(0x3621), 0x01 },
	{ CCI_REG8(0x3C12), 0x62 },
	{ CCI_REG8(0x3C13), 0x32 },
	{ CCI_REG8(0x3C14), 0x20 },
	{ CCI_REG8(0x3F0C), 0x00 },
	{ CCI_REG8(0x3F14), 0x01 },
	{ CCI_REG8(0x3F80), 0x04 },
	{ CCI_REG8(0x3F81), 0x58 },
	{ CCI_REG8(0x3F8C), 0x00 },
	{ CCI_REG8(0x3F8D), 0x00 },
	{ CCI_REG8(0x3FFC), 0x00 },
	{ CCI_REG8(0x3FFD), 0x1C },
	{ CCI_REG8(0x3FFE), 0x00 },
	{ CCI_REG8(0x3FFF), 0x0A },
	{ CCI_REG8(0xBCF1), 0x00 },
	{ CCI_REG8(0x0202), 0x13 },
	{ CCI_REG8(0x0203), 0x58 },
	{ CCI_REG8(0x0224), 0x01 },
	{ CCI_REG8(0x0225), 0xF4 },
	{ CCI_REG8(0x3FE0), 0x01 },
	{ CCI_REG8(0x3FE1), 0xF4 },
	{ CCI_REG8(0x0204), 0x00 },
	{ CCI_REG8(0x0205), 0x70 },
	{ CCI_REG8(0x0216), 0x00 },
	{ CCI_REG8(0x0217), 0x70 },
	{ CCI_REG8(0x0218), 0x01 },
	{ CCI_REG8(0x0219), 0x00 },
	{ CCI_REG8(0x020E), 0x01 },
	{ CCI_REG8(0x020F), 0x00 },
	{ CCI_REG8(0x0210), 0x01 },
	{ CCI_REG8(0x0211), 0x00 },
	{ CCI_REG8(0x0212), 0x01 },
	{ CCI_REG8(0x0213), 0x00 },
	{ CCI_REG8(0x0214), 0x01 },
	{ CCI_REG8(0x0215), 0x00 },
	{ CCI_REG8(0x3FE2), 0x00 },
	{ CCI_REG8(0x3FE3), 0x70 },
	{ CCI_REG8(0x3FE4), 0x01 },
	{ CCI_REG8(0x3FE5), 0x00 },
	{ CCI_REG8(0x0B00), 0x00 },
	{ CCI_REG8(0x323B), 0x00 },
	{ CCI_REG8(0x323D), 0x00 },
	{ CCI_REG8(0x323C), 0x00 },
	{ CCI_REG8(0x323E), 0x00 },
	{ CCI_REG8(0x323F), 0x00 },
	{ CCI_REG8(0x0B06), 0x01 },
};

static const s64 imx616_link_freq_menu[] = { IMX616_LINK_FREQ, IMX616_LINK_FREQ_FULL };

struct imx616_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts_def;
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
	unsigned int link_freq_index;
	s64 pixel_rate;
};

static const struct imx616_mode imx616_modes[] = {
	{
		.width = IMX616_WIDTH, .height = IMX616_HEIGHT,
		.hts = IMX616_HTS, .vts_def = IMX616_VTS_DEF,
		.regs = imx616_mode_regs, .num_regs = ARRAY_SIZE(imx616_mode_regs),
		.link_freq_index = 0, .pixel_rate = IMX616_PIXEL_RATE,
	},
	{
		.width = IMX616_FULL_WIDTH, .height = IMX616_FULL_HEIGHT,
		.hts = IMX616_FULL_HTS, .vts_def = IMX616_FULL_VTS_DEF,
		.regs = imx616_full_regs, .num_regs = ARRAY_SIZE(imx616_full_regs),
		.link_freq_index = 1, .pixel_rate = IMX616_PIXEL_RATE_FULL,
	},
};

struct imx616 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct device *dev;
	struct regmap *regmap;
	struct clk *xclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[IMX616_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *hblank;
	const struct imx616_mode *mode;
	u8 *qsc;		/* m52xq: cached QSC data, NULL until loaded */
	bool qsc_missing;	/* request_firmware failed once: do not retry every stream */
};

static inline struct imx616 *to_imx616(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx616, sd);
}

static int imx616_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx616 *gw = container_of(ctrl->handler, struct imx616, ctrls);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK)
		__v4l2_ctrl_modify_range(gw->exposure, IMX616_EXPOSURE_MIN,
					 gw->mode->height + ctrl->val - IMX616_EXPOSURE_MARGIN,
					 1, IMX616_EXPOSURE_DEF);

	if (!pm_runtime_get_if_in_use(gw->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(gw->regmap, IMX616_REG_EXPOSURE, ctrl->val, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(gw->regmap, IMX616_REG_AGAIN, 1024 - 65536 / ctrl->val, &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(gw->regmap, IMX616_REG_VTS, gw->mode->height + ctrl->val, &ret);
		break;
	default:
		ret = -EINVAL;
	}

	pm_runtime_put(gw->dev);
	return ret;
}

static const struct v4l2_ctrl_ops imx616_ctrl_ops = {
	.s_ctrl = imx616_s_ctrl,
};

static int imx616_init_controls(struct imx616 *gw)
{
	struct v4l2_ctrl_handler *hdl = &gw->ctrls;
	struct v4l2_fwnode_device_properties props;
	int ret;

	v4l2_ctrl_handler_init(hdl, 9);

	gw->pixel_rate = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE, IMX616_PIXEL_RATE,
					   IMX616_PIXEL_RATE_FULL, 1, IMX616_PIXEL_RATE);
	if (gw->pixel_rate)
		gw->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	gw->link_freq = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ,
					       ARRAY_SIZE(imx616_link_freq_menu) - 1, 0,
					       imx616_link_freq_menu);
	if (gw->link_freq)
		gw->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	gw->hblank = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_HBLANK, IMX616_HTS - IMX616_WIDTH,
				       IMX616_HTS - IMX616_WIDTH, 1, IMX616_HTS - IMX616_WIDTH);
	if (gw->hblank)
		gw->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	gw->vblank = v4l2_ctrl_new_std(hdl, &imx616_ctrl_ops, V4L2_CID_VBLANK,
				       IMX616_VBLANK_MIN, IMX616_VTS_MAX - IMX616_HEIGHT, 1,
				       IMX616_VTS_DEF - IMX616_HEIGHT);

	gw->exposure = v4l2_ctrl_new_std(hdl, &imx616_ctrl_ops, V4L2_CID_EXPOSURE,
					 IMX616_EXPOSURE_MIN,
					 IMX616_VTS_DEF - IMX616_EXPOSURE_MARGIN, 1,
					 IMX616_EXPOSURE_DEF);

	v4l2_ctrl_new_std(hdl, &imx616_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX616_GAIN_MIN, IMX616_GAIN_MAX, 1, IMX616_GAIN_DEF);

	if (hdl->error) {
		ret = hdl->error;
		goto err;
	}

	ret = v4l2_fwnode_device_parse(gw->dev, &props);
	if (ret)
		goto err;

	ret = v4l2_ctrl_new_fwnode_properties(hdl, &imx616_ctrl_ops, &props);
	if (ret)
		goto err;

	gw->sd.ctrl_handler = hdl;
	return 0;
err:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static void imx616_fill_format(struct v4l2_mbus_framefmt *fmt, const struct imx616_mode *mode)
{
	fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int imx616_init_state(struct v4l2_subdev *sd, struct v4l2_subdev_state *state)
{
	imx616_fill_format(v4l2_subdev_state_get_format(state, 0), &imx616_modes[0]);
	*v4l2_subdev_state_get_crop(state, 0) = (struct v4l2_rect){
		.width = IMX616_WIDTH, .height = IMX616_HEIGHT,
	};
	return 0;
}

static int imx616_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct imx616 *gw = to_imx616(sd);
	const struct imx616_mode *mode;

	mode = v4l2_find_nearest_size(imx616_modes, ARRAY_SIZE(imx616_modes), width, height,
				      fmt->format.width, fmt->format.height);
	imx616_fill_format(&fmt->format, mode);
	*v4l2_subdev_state_get_format(state, fmt->pad) = fmt->format;
	*v4l2_subdev_state_get_crop(state, 0) = (struct v4l2_rect){
		.width = mode->width, .height = mode->height,
	};

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE && gw->mode != mode) {
		u32 hblank = mode->hts - mode->width;

		gw->mode = mode;
		/* the control handler lock is the state lock, held here */
		__v4l2_ctrl_s_ctrl_int64(gw->pixel_rate, mode->pixel_rate);
		__v4l2_ctrl_s_ctrl(gw->link_freq, mode->link_freq_index);
		__v4l2_ctrl_modify_range(gw->hblank, hblank, hblank, 1, hblank);
		__v4l2_ctrl_modify_range(gw->vblank, IMX616_VBLANK_MIN,
					 IMX616_VTS_MAX - mode->height, 1,
					 mode->vts_def - mode->height);
		__v4l2_ctrl_s_ctrl(gw->vblank, mode->vts_def - mode->height);
		/*
		 * the s_ctrl above is a no-op when the value does not change (see the
		 * imx258 fix): set the exposure range for the new frame height explicitly
		 */
		__v4l2_ctrl_modify_range(gw->exposure, IMX616_EXPOSURE_MIN,
					 mode->vts_def - IMX616_EXPOSURE_MARGIN, 1,
					 min_t(s32, gw->exposure->val,
					       mode->vts_def - IMX616_EXPOSURE_MARGIN));
	}
	return 0;
}

static int imx616_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	return 0;
}

static int imx616_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(imx616_modes) || fse->code != MEDIA_BUS_FMT_SRGGB10_1X10)
		return -EINVAL;
	fse->min_width = fse->max_width = imx616_modes[fse->index].width;
	fse->min_height = fse->max_height = imx616_modes[fse->index].height;
	return 0;
}

static int imx616_get_selection(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		return 0;
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r = (struct v4l2_rect){ .width = IMX616_FULL_WIDTH, .height = IMX616_FULL_HEIGHT };
		return 0;
	}
	return -EINVAL;
}

struct imx616_adap_search {
	struct i2c_adapter *own;
	struct i2c_adapter *found;
};

/* the other I2C master (reg = 1) of the CCI the sensor sits on: the module EEPROM is on it */
static int imx616_eeprom_adapter_cb(struct device *dev, void *data)
{
	struct imx616_adap_search *search = data;
	struct i2c_adapter *adap = i2c_verify_adapter(dev);
	u32 reg;

	if (!adap || adap == search->own || !dev->of_node)
		return 0;
	if (of_property_read_u32(dev->of_node, "reg", &reg) || reg != IMX616_EEPROM_BUS_REG)
		return 0;
	search->found = i2c_get_adapter(adap->nr);
	return search->found ? 1 : 0;
}

/* m52xq: read the QSC straight from the module EEPROM (the sensor is powered: stream start) */
static u8 *imx616_read_qsc_eeprom(struct imx616 *gw)
{
	struct i2c_client *client = to_i2c_client(gw->dev);
	struct imx616_adap_search search = { .own = client->adapter };
	unsigned int off;
	u8 *buf;
	int ret = 0, tries;

	if (!client->adapter->dev.parent)
		return NULL;
	device_for_each_child(client->adapter->dev.parent, &search, imx616_eeprom_adapter_cb);
	if (!search.found)
		return NULL;

	buf = devm_kzalloc(gw->dev, IMX616_QSC_SIZE, GFP_KERNEL);
	if (!buf) {
		i2c_put_adapter(search.found);
		return NULL;
	}

	for (off = 0; off < IMX616_QSC_SIZE; off += IMX616_EEPROM_CHUNK) {
		unsigned int len = min_t(unsigned int, IMX616_EEPROM_CHUNK, IMX616_QSC_SIZE - off);
		u16 addr = IMX616_EEPROM_QSC_OFFSET + off;
		u8 a[2] = { addr >> 8, addr & 0xff };
		struct i2c_msg msgs[2] = {
			{ .addr = IMX616_EEPROM_ADDR, .flags = 0, .len = 2, .buf = a },
			{ .addr = IMX616_EEPROM_ADDR, .flags = I2C_M_RD, .len = len, .buf = buf + off },
		};

		for (tries = 0; tries < 3; tries++) {
			ret = i2c_transfer(search.found, msgs, 2);
			if (ret == 2)
				break;
			usleep_range(1000, 2000);
		}
		if (ret != 2) {
			dev_warn(gw->dev, "module EEPROM read failed at 0x%04x (%d)\n",
				 IMX616_EEPROM_QSC_OFFSET + off, ret);
			goto fail;
		}
	}
	i2c_put_adapter(search.found);

	/* an unpowered or empty EEPROM reads back all 0x00 or all 0xff */
	if (!memchr_inv(buf, 0x00, IMX616_QSC_SIZE) || !memchr_inv(buf, 0xff, IMX616_QSC_SIZE)) {
		dev_warn(gw->dev, "module EEPROM: QSC area is empty\n");
		devm_kfree(gw->dev, buf);
		return NULL;
	}
	dev_info(gw->dev, "QSC read from the module EEPROM (%d bytes, crc32 %08x)\n",
		 IMX616_QSC_SIZE, ~crc32_le(~0, buf, IMX616_QSC_SIZE));
	return buf;

fail:
	i2c_put_adapter(search.found);
	devm_kfree(gw->dev, buf);
	return NULL;
}

static int imx616_load_modes(struct imx616 *gw)
{
	int ret;

	ret = cci_multi_reg_write(gw->regmap, imx616_global_regs,
				  ARRAY_SIZE(imx616_global_regs), NULL);
	if (ret)
		return ret;
	ret = cci_multi_reg_write(gw->regmap, gw->mode->regs, gw->mode->num_regs, NULL);
	if (ret || gw->mode->width != IMX616_FULL_WIDTH)
		return ret;

	/* m52xq: the full frame (sensor re-mosaic) needs the QSC of the module before the stream */
	if (!gw->qsc && !gw->qsc_missing)
		gw->qsc = imx616_read_qsc_eeprom(gw);
	if (!gw->qsc && !gw->qsc_missing) {
		const struct firmware *fw;

		if (request_firmware(&fw, IMX616_QSC_FW, gw->dev)) {
			dev_warn(gw->dev, "no %s: full frame without QSC (colour blotches)\n", IMX616_QSC_FW);
			gw->qsc_missing = true;
		} else if (fw->size != IMX616_QSC_SIZE) {
			dev_warn(gw->dev, "%s: %zu bytes, expected %d\n", IMX616_QSC_FW, fw->size, IMX616_QSC_SIZE);
			gw->qsc_missing = true;
			release_firmware(fw);
		} else {
			gw->qsc = devm_kmemdup(gw->dev, fw->data, fw->size, GFP_KERNEL);
			release_firmware(fw);
			dev_info(gw->dev, "QSC loaded (%d bytes)\n", IMX616_QSC_SIZE);
		}
	}
	if (gw->qsc) {
		unsigned int i;

		/* the CCI takes a few data bytes per transfer: write it in 8-byte bursts */
		for (i = 0; i < IMX616_QSC_SIZE && !ret; i += 8)
			ret = regmap_bulk_write(gw->regmap, IMX616_QSC_REG + i, gw->qsc + i,
						min_t(unsigned int, 8, IMX616_QSC_SIZE - i));
		if (ret)
			dev_err(gw->dev, "QSC write failed (%d)\n", ret);
	}
	return ret;
}

static int imx616_enable_streams(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				 u32 pad, u64 streams_mask)
{
	struct imx616 *gw = to_imx616(sd);
	int ret;

	ret = pm_runtime_resume_and_get(gw->dev);
	if (ret < 0)
		return ret;

	ret = imx616_load_modes(gw);
	if (ret) {
		dev_err(gw->dev, "failed to write the init sequence (%d)\n", ret);
		goto err_put;
	}

	ret = __v4l2_ctrl_handler_setup(&gw->ctrls);
	if (ret)
		goto err_put;

	cci_write(gw->regmap, IMX616_REG_STREAM, 1, &ret);
	if (ret)
		goto err_put;
	return 0;

err_put:
	pm_runtime_put_autosuspend(gw->dev);
	return ret;
}

static int imx616_disable_streams(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				  u32 pad, u64 streams_mask)
{
	struct imx616 *gw = to_imx616(sd);
	int ret = 0;

	cci_write(gw->regmap, IMX616_REG_STREAM, 0, &ret);
	pm_runtime_put_autosuspend(gw->dev);
	return ret;
}

static const struct v4l2_subdev_video_ops imx616_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops imx616_pad_ops = {
	.enum_mbus_code = imx616_enum_mbus_code,
	.enum_frame_size = imx616_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx616_set_fmt,
	.get_selection = imx616_get_selection,
	.enable_streams = imx616_enable_streams,
	.disable_streams = imx616_disable_streams,
};

static const struct v4l2_subdev_ops imx616_subdev_ops = {
	.video = &imx616_video_ops,
	.pad = &imx616_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx616_internal_ops = {
	.init_state = imx616_init_state,
};

static int imx616_power_on(struct device *dev)
{
	struct imx616 *gw = to_imx616(dev_get_drvdata(dev));
	int i, ret;

	gpiod_set_value_cansleep(gw->reset_gpio, 1);

	/* sequential, as on the stock board: vio, vdig, vana */
	for (i = 0; i < IMX616_NUM_SUPPLIES; i++) {
		ret = regulator_enable(gw->supplies[i].consumer);
		if (ret) {
			dev_err(dev, "failed to enable %s\n", gw->supplies[i].supply);
			goto err_supplies;
		}
		usleep_range(1000, 2000);
	}

	ret = clk_prepare_enable(gw->xclk);
	if (ret) {
		dev_err(dev, "failed to enable the clock\n");
		goto err_supplies;
	}

	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(gw->reset_gpio, 0);
	msleep(20);
	return 0;

err_supplies:
	while (--i >= 0)
		regulator_disable(gw->supplies[i].consumer);
	return ret;
}

static int imx616_power_off(struct device *dev)
{
	struct imx616 *gw = to_imx616(dev_get_drvdata(dev));
	int i;

	gpiod_set_value_cansleep(gw->reset_gpio, 1);
	clk_disable_unprepare(gw->xclk);
	for (i = IMX616_NUM_SUPPLIES - 1; i >= 0; i--)
		regulator_disable(gw->supplies[i].consumer);
	return 0;
}

static int imx616_check_hwcfg(struct device *dev)
{
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct fwnode_handle *endpoint;
	int ret;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint)
		return dev_err_probe(dev, -EINVAL, "endpoint node not found\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (ret)
		return ret;

	if (ep.bus.mipi_csi2.num_data_lanes != IMX616_LANES) {
		dev_err(dev, "only %d data lanes are supported\n", IMX616_LANES);
		ret = -EINVAL;
	} else if (ep.nr_of_link_frequencies < 1 ||
		   ep.link_frequencies[0] != IMX616_LINK_FREQ) {
		dev_err(dev, "link-frequencies must start with %lld\n", IMX616_LINK_FREQ);
		ret = -EINVAL;
	} else if (ep.nr_of_link_frequencies < 2 ||
		   ep.link_frequencies[1] != IMX616_LINK_FREQ_FULL) {
		/* m52xq: the full frame mode runs at its own rate; the DT lists only the binning one */
		dev_info(dev, "full frame mode uses link frequency %lld (not in the DT)\n",
			 IMX616_LINK_FREQ_FULL);
	}

	v4l2_fwnode_endpoint_free(&ep);
	return ret;
}

static int imx616_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx616 *gw;
	u64 id;
	int ret, i;

	gw = devm_kzalloc(dev, sizeof(*gw), GFP_KERNEL);
	if (!gw)
		return -ENOMEM;
	gw->dev = dev;
	gw->mode = &imx616_modes[0];

	ret = imx616_check_hwcfg(dev);
	if (ret)
		return ret;

	gw->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(gw->regmap))
		return dev_err_probe(dev, PTR_ERR(gw->regmap), "cci regmap\n");

	gw->xclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(gw->xclk))
		return dev_err_probe(dev, PTR_ERR(gw->xclk), "failed to get the clock\n");
	if (clk_get_rate(gw->xclk) != IMX616_XCLK_FREQ)
		return dev_err_probe(dev, -EINVAL, "unsupported clock rate %lu\n",
				     clk_get_rate(gw->xclk));

	for (i = 0; i < IMX616_NUM_SUPPLIES; i++)
		gw->supplies[i].supply = imx616_supply_names[i];
	ret = devm_regulator_bulk_get(dev, IMX616_NUM_SUPPLIES, gw->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get the supplies\n");

	gw->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(gw->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(gw->reset_gpio), "reset gpio\n");

	v4l2_i2c_subdev_init(&gw->sd, client, &imx616_subdev_ops);
	gw->sd.internal_ops = &imx616_internal_ops;

	ret = imx616_power_on(dev);
	if (ret)
		return ret;

	ret = cci_read(gw->regmap, IMX616_REG_CHIP_ID, &id, NULL);
	if (ret) {
		dev_err(dev, "failed to read the chip id (%d)\n", ret);
		goto err_power_off;
	}
	dev_info(dev, "detected IMX616 (model id 0x%04llx)\n", id);

	ret = imx616_init_controls(gw);
	if (ret)
		goto err_power_off;

	gw->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	gw->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	gw->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&gw->sd.entity, 1, &gw->pad);
	if (ret)
		goto err_ctrls;

	gw->sd.state_lock = gw->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&gw->sd);
	if (ret)
		goto err_entity;

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_put_autosuspend(dev);

	ret = v4l2_async_register_subdev_sensor(&gw->sd);
	if (ret) {
		dev_err(dev, "failed to register the subdev (%d)\n", ret);
		goto err_cleanup;
	}
	return 0;

err_cleanup:
	v4l2_subdev_cleanup(&gw->sd);
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
err_entity:
	media_entity_cleanup(&gw->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&gw->ctrls);
err_power_off:
	imx616_power_off(dev);
	return ret;
}

static void imx616_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx616 *gw = to_imx616(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&gw->ctrls);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx616_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx616_pm_ops, imx616_power_off, imx616_power_on, NULL);

static const struct of_device_id imx616_of_match[] = {
	{ .compatible = "sony,imx616" },
	{ }
};
MODULE_DEVICE_TABLE(of, imx616_of_match);

static struct i2c_driver imx616_driver = {
	.driver = {
		.name = "imx616",
		.of_match_table = imx616_of_match,
		.pm = pm_ptr(&imx616_pm_ops),
	},
	.probe = imx616_probe,
	.remove = imx616_remove,
};
module_i2c_driver(imx616_driver);

MODULE_DESCRIPTION("Sony IMX616 sensor driver");
MODULE_LICENSE("GPL");
