/*
 * Freescale i.MX23/i.MX28 LRADC driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __MXS_LRADC_H
#define __MXS_LRADC_H

#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/sysfs.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/stmp_device.h>
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/clk.h>

#define LRADC_NAME_ADC	"mxs-lradc-adc"
#define LRADC_NAME_TS	"mxs-lradc-ts"

#define LRADC_MAX_DELAY_CHANS	4
#define LRADC_MAX_MAPPED_CHANS	8
#define LRADC_MAX_TOTAL_CHANS	16

#define LRADC_DELAY_TIMER_HZ	2000

#define	LRADC_CTRL0				0x00
# define LRADC_CTRL0_MX28_TOUCH_DETECT_ENABLE	(1 << 23)
# define LRADC_CTRL0_MX28_TOUCH_SCREEN_TYPE	(1 << 22)
# define LRADC_CTRL0_MX28_YNNSW	/* YM */	(1 << 21)
# define LRADC_CTRL0_MX28_YPNSW	/* YP */	(1 << 20)
# define LRADC_CTRL0_MX28_YPPSW	/* YP */	(1 << 19)
# define LRADC_CTRL0_MX28_XNNSW	/* XM */	(1 << 18)
# define LRADC_CTRL0_MX28_XNPSW	/* XM */	(1 << 17)
# define LRADC_CTRL0_MX28_XPPSW	/* XP */	(1 << 16)

# define LRADC_CTRL0_MX23_TOUCH_DETECT_ENABLE	(1 << 20)
# define LRADC_CTRL0_MX23_YM			(1 << 19)
# define LRADC_CTRL0_MX23_XM			(1 << 18)
# define LRADC_CTRL0_MX23_YP			(1 << 17)
# define LRADC_CTRL0_MX23_XP			(1 << 16)

# define LRADC_CTRL0_MX28_PLATE_MASK \
		(LRADC_CTRL0_MX28_TOUCH_DETECT_ENABLE | \
		LRADC_CTRL0_MX28_YNNSW | LRADC_CTRL0_MX28_YPNSW | \
		LRADC_CTRL0_MX28_YPPSW | LRADC_CTRL0_MX28_XNNSW | \
		LRADC_CTRL0_MX28_XNPSW | LRADC_CTRL0_MX28_XPPSW)

# define LRADC_CTRL0_MX23_PLATE_MASK \
		(LRADC_CTRL0_MX23_TOUCH_DETECT_ENABLE | \
		LRADC_CTRL0_MX23_YM | LRADC_CTRL0_MX23_XM | \
		LRADC_CTRL0_MX23_YP | LRADC_CTRL0_MX23_XP)

#define	LRADC_CTRL1				0x10
#define	LRADC_CTRL1_TOUCH_DETECT_IRQ_EN		(1 << 24)
#define	LRADC_CTRL1_LRADC_IRQ_EN(n)		(1 << ((n) + 16))
#define	LRADC_CTRL1_MX28_LRADC_IRQ_EN_MASK	(0x1fff << 16)
#define	LRADC_CTRL1_MX23_LRADC_IRQ_EN_MASK	(0x01ff << 16)
#define	LRADC_CTRL1_LRADC_IRQ_EN_OFFSET		16
#define	LRADC_CTRL1_TOUCH_DETECT_IRQ		(1 << 8)
#define	LRADC_CTRL1_LRADC_IRQ(n)		(1 << (n))
#define	LRADC_CTRL1_MX28_LRADC_IRQ_MASK		0x1fff
#define	LRADC_CTRL1_MX23_LRADC_IRQ_MASK		0x01ff
#define	LRADC_CTRL1_LRADC_IRQ_OFFSET		0

#define	LRADC_CTRL2				0x20
#define	LRADC_CTRL2_DIVIDE_BY_TWO_OFFSET	24
#define	LRADC_CTRL2_TEMPSENSE_PWD		(1 << 15)

#define	LRADC_STATUS				0x40
#define	LRADC_STATUS_TOUCH_DETECT_RAW		(1 << 0)

#define	LRADC_CH(n)				(0x50 + (0x10 * (n)))
#define	LRADC_CH_ACCUMULATE			(1 << 29)
#define	LRADC_CH_NUM_SAMPLES_MASK		(0x1f << 24)
#define	LRADC_CH_NUM_SAMPLES_OFFSET		24
#define	LRADC_CH_NUM_SAMPLES(x) \
				((x) << LRADC_CH_NUM_SAMPLES_OFFSET)
#define	LRADC_CH_VALUE_MASK			0x3ffff
#define	LRADC_CH_VALUE_OFFSET			0

#define	LRADC_DELAY(n)				(0xd0 + (0x10 * (n)))
#define	LRADC_DELAY_TRIGGER_LRADCS_MASK		(0xff << 24)
#define	LRADC_DELAY_TRIGGER_LRADCS_OFFSET	24
#define	LRADC_DELAY_TRIGGER(x) \
				(((x) << LRADC_DELAY_TRIGGER_LRADCS_OFFSET) & \
				LRADC_DELAY_TRIGGER_LRADCS_MASK)
#define	LRADC_DELAY_KICK			(1 << 20)
#define	LRADC_DELAY_TRIGGER_DELAYS_MASK		(0xf << 16)
#define	LRADC_DELAY_TRIGGER_DELAYS_OFFSET	16
#define	LRADC_DELAY_TRIGGER_DELAYS(x) \
				(((x) << LRADC_DELAY_TRIGGER_DELAYS_OFFSET) & \
				LRADC_DELAY_TRIGGER_DELAYS_MASK)
#define	LRADC_DELAY_LOOP_COUNT_MASK		(0x1f << 11)
#define	LRADC_DELAY_LOOP_COUNT_OFFSET		11
#define	LRADC_DELAY_LOOP(x) \
				(((x) << LRADC_DELAY_LOOP_COUNT_OFFSET) & \
				LRADC_DELAY_LOOP_COUNT_MASK)
#define	LRADC_DELAY_DELAY_MASK			0x7ff
#define	LRADC_DELAY_DELAY_OFFSET		0
#define	LRADC_DELAY_DELAY(x) \
				(((x) << LRADC_DELAY_DELAY_OFFSET) & \
				LRADC_DELAY_DELAY_MASK)

#define	LRADC_CTRL4				0x140
#define	LRADC_CTRL4_LRADCSELECT_MASK(n)		(0xf << ((n) * 4))
#define	LRADC_CTRL4_LRADCSELECT_OFFSET(n)	((n) * 4)

#define LRADC_RESOLUTION			12
#define LRADC_SINGLE_SAMPLE_MASK		((1 << LRADC_RESOLUTION) - 1)

enum mxs_lradc_id {
	IMX23_LRADC,
	IMX28_LRADC,
};

enum mxs_lradc_ts_wires {
	MXS_LRADC_TOUCHSCREEN_NONE = 0,
	MXS_LRADC_TOUCHSCREEN_4WIRE,
	MXS_LRADC_TOUCHSCREEN_5WIRE,
};

struct mxs_lradc {
	enum mxs_lradc_id	soc;

	void __iomem		*base;
	struct clk		*clk;

	int			irq[13];
	const char * const	*irq_name;
	int			irq_count;

	/*
	 * Touchscreen LRADC channels receives a private slot in the CTRL4
	 * register, the slot #7. Therefore only 7 slots instead of 8 in the
	 * CTRL4 register can be mapped to LRADC channels when using the
	 * touchscreen.
	 *
	 * Furthermore, certain LRADC channels are shared between touchscreen
	 * and/or touch-buttons and generic LRADC block. Therefore when using
	 * either of these, these channels are not available for the regular
	 * sampling. The shared channels are as follows:
	 *
	 * CH0 -- Touch button #0
	 * CH1 -- Touch button #1
	 * CH2 -- Touch screen XPUL
	 * CH3 -- Touch screen YPLL
	 * CH4 -- Touch screen XNUL
	 * CH5 -- Touch screen YNLR
	 * CH6 -- Touch screen WIPER (5-wire only)
	 *
	 * The bitfields below represents which parts of the LRADC block are
	 * switched into special mode of operation. These channels can not
	 * be sampled as regular LRADC channels. The driver will refuse any
	 * attempt to sample these channels.
	 */
#define CHAN_MASK_TOUCHBUTTON		(0x3 << 0)
#define CHAN_MASK_TOUCHSCREEN_4WIRE	(0xf << 2)
#define CHAN_MASK_TOUCHSCREEN_5WIRE	(0x1f << 2)
	enum mxs_lradc_ts_wires	use_touchscreen;
	bool			use_touchbutton;
};

static inline void mxs_lradc_reg_set(struct mxs_lradc *lradc, u32 val, u32 reg)
{
	writel(val, lradc->base + reg + STMP_OFFSET_REG_SET);
}

static inline void mxs_lradc_reg_clear(struct mxs_lradc *lradc,
				       u32 val, u32 reg)
{
	writel(val, lradc->base + reg + STMP_OFFSET_REG_CLR);
}

static inline void mxs_lradc_reg_wrt(struct mxs_lradc *lradc, u32 val, u32 reg)
{
	writel(val, lradc->base + reg);
}

static inline u32 mxs_lradc_irq_en_mask(struct mxs_lradc *lradc)
{
	if (lradc->soc == IMX23_LRADC)
		return LRADC_CTRL1_MX23_LRADC_IRQ_EN_MASK;
	return LRADC_CTRL1_MX28_LRADC_IRQ_EN_MASK;
}

static inline u32 mxs_lradc_irq_mask(struct mxs_lradc *lradc)
{
	if (lradc->soc == IMX23_LRADC)
		return LRADC_CTRL1_MX23_LRADC_IRQ_MASK;
	return LRADC_CTRL1_MX28_LRADC_IRQ_MASK;
}

#endif /* __MXS_LRADC_H */
