/*
 * Freescale i.MX23/i.MX28 LRADC general-purpose ADC driver
 *
 * Copyright (c) 2012 DENX Software Engineering, GmbH.
 * Marek Vasut <marex@denx.de>
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

#include <linux/mfd/mxs-lradc.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

/*
 * Make this runtime configurable if necessary. Currently, if the buffered mode
 * is enabled, the LRADC takes LRADC_DELAY_TIMER_LOOP samples of data before
 * triggering IRQ. The sampling happens every (LRADC_DELAY_TIMER_PER / 2000)
 * seconds. The result is that the samples arrive every 500mS.
 */
#define LRADC_DELAY_TIMER_PER	200
#define LRADC_DELAY_TIMER_LOOP	5

#define VREF_MV_BASE 1850

static const uint32_t lradc_adc_vref_mv[][LRADC_MAX_TOTAL_CHANS] = {
	[IMX23_LRADC] = {
		VREF_MV_BASE,		/* CH0 */
		VREF_MV_BASE,		/* CH1 */
		VREF_MV_BASE,		/* CH2 */
		VREF_MV_BASE,		/* CH3 */
		VREF_MV_BASE,		/* CH4 */
		VREF_MV_BASE,		/* CH5 */
		VREF_MV_BASE * 2,	/* CH6 VDDIO */
		VREF_MV_BASE * 4,	/* CH7 VBATT */
		VREF_MV_BASE,		/* CH8 Temp sense 0 */
		VREF_MV_BASE,		/* CH9 Temp sense 1 */
		VREF_MV_BASE,		/* CH10 */
		VREF_MV_BASE,		/* CH11 */
		VREF_MV_BASE,		/* CH12 USB_DP */
		VREF_MV_BASE,		/* CH13 USB_DN */
		VREF_MV_BASE,		/* CH14 VBG */
		VREF_MV_BASE * 4,	/* CH15 VDD5V */
	},
	[IMX28_LRADC] = {
		VREF_MV_BASE,		/* CH0 */
		VREF_MV_BASE,		/* CH1 */
		VREF_MV_BASE,		/* CH2 */
		VREF_MV_BASE,		/* CH3 */
		VREF_MV_BASE,		/* CH4 */
		VREF_MV_BASE,		/* CH5 */
		VREF_MV_BASE,		/* CH6 */
		VREF_MV_BASE * 4,	/* CH7 VBATT */
		VREF_MV_BASE,		/* CH8 Temp sense 0 */
		VREF_MV_BASE,		/* CH9 Temp sense 1 */
		VREF_MV_BASE * 2,	/* CH10 VDDIO */
		VREF_MV_BASE,		/* CH11 VTH */
		VREF_MV_BASE * 2,	/* CH12 VDDA */
		VREF_MV_BASE,		/* CH13 VDDD */
		VREF_MV_BASE,		/* CH14 VBG */
		VREF_MV_BASE * 4,	/* CH15 VDD5V */
	},
};

enum mxs_lradc_divbytwo {
	MXS_LRADC_DIV_DISABLED = 0,
	MXS_LRADC_DIV_ENABLED,
};

struct mxs_lradc_scale {
	unsigned int		integer;
	unsigned int		nano;
};

struct mxs_lradc_adc {
	struct mxs_lradc	*lradc;
	struct device		*dev;

	uint32_t		*buffer;
	struct iio_trigger	*trig;
	struct mutex		lock;
	struct completion	completion;

	const uint32_t		*vref_mv;
	struct mxs_lradc_scale	scale_avail[LRADC_MAX_TOTAL_CHANS][2];
	unsigned long		is_divided;
};

/*
 * Raw I/O operations
 */
static int lradc_adc_read_single(struct iio_dev *iio_dev, int chan, int *val)
{
	struct mxs_lradc_adc *adc = iio_priv(iio_dev);
	struct mxs_lradc *lradc = adc->lradc;
	int ret;

	/*
	 * See if there is no buffered operation in progess. If there is, simply
	 * bail out. This can be improved to support both buffered and raw IO at
	 * the same time, yet the code becomes horribly complicated. Therefore I
	 * applied KISS principle here.
	 */
	ret = mutex_trylock(&adc->lock);
	if (!ret)
		return -EBUSY;

	reinit_completion(&adc->completion);

	/*
	 * No buffered operation in progress, map the channel and trigger it.
	 * Virtual channel 0 is always used here as the others are always not
	 * used if doing raw sampling.
	 */
	if (lradc->soc == IMX28_LRADC)
		mxs_lradc_reg_clear(lradc, LRADC_CTRL1_MX28_LRADC_IRQ_EN_MASK,
			LRADC_CTRL1);
	mxs_lradc_reg_clear(lradc, 0xff, LRADC_CTRL0);

	/* Enable / disable the divider per requirement */
	if (test_bit(chan, &adc->is_divided))
		mxs_lradc_reg_set(lradc, 1 << LRADC_CTRL2_DIVIDE_BY_TWO_OFFSET,
			LRADC_CTRL2);
	else
		mxs_lradc_reg_clear(lradc,
			1 << LRADC_CTRL2_DIVIDE_BY_TWO_OFFSET, LRADC_CTRL2);

	/* Clean the slot's previous content, then set new one. */
	mxs_lradc_reg_clear(lradc, LRADC_CTRL4_LRADCSELECT_MASK(0),
			LRADC_CTRL4);
	mxs_lradc_reg_set(lradc, chan, LRADC_CTRL4);

	mxs_lradc_reg_wrt(lradc, 0, LRADC_CH(0));

	/* Enable the IRQ and start sampling the channel. */
	mxs_lradc_reg_set(lradc, LRADC_CTRL1_LRADC_IRQ_EN(0), LRADC_CTRL1);
	mxs_lradc_reg_set(lradc, 1 << 0, LRADC_CTRL0);

	/* Wait for completion on the channel, 1 second max. */
	ret = wait_for_completion_killable_timeout(&adc->completion, HZ);
	if (!ret)
		ret = -ETIMEDOUT;
	if (ret < 0)
		goto err;

	/* Read the data. */
	*val = readl(lradc->base + LRADC_CH(0)) & LRADC_CH_VALUE_MASK;
	ret = IIO_VAL_INT;

err:
	mxs_lradc_reg_clear(lradc, LRADC_CTRL1_LRADC_IRQ_EN(0), LRADC_CTRL1);

	mutex_unlock(&adc->lock);

	return ret;
}

static int lradc_adc_read_temp(struct iio_dev *iio_dev, int *val)
{
	int ret, min, max;

	ret = lradc_adc_read_single(iio_dev, 8, &min);
	if (ret != IIO_VAL_INT)
		return ret;

	ret = lradc_adc_read_single(iio_dev, 9, &max);
	if (ret != IIO_VAL_INT)
		return ret;

	*val = max - min;

	return IIO_VAL_INT;
}

static int lradc_adc_read_raw(struct iio_dev *iio_dev,
			const struct iio_chan_spec *chan,
			int *val, int *val2, long m)
{
	struct mxs_lradc_adc *adc = iio_priv(iio_dev);

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		if (chan->type == IIO_TEMP)
			return lradc_adc_read_temp(iio_dev, val);

		return lradc_adc_read_single(iio_dev, chan->channel, val);

	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_TEMP) {
			/* From the datasheet, we have to multiply by 1.012 and
			 * divide by 4
			 */
			*val = 0;
			*val2 = 253000;
			return IIO_VAL_INT_PLUS_MICRO;
		}

		*val = adc->vref_mv[chan->channel];
		*val2 = chan->scan_type.realbits -
			test_bit(chan->channel, &adc->is_divided);
		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_OFFSET:
		if (chan->type == IIO_TEMP) {
			/* The calculated value from the ADC is in Kelvin, we
			 * want Celsius for hwmon so the offset is
			 * -272.15 * scale
			 */
			*val = -1075;
			*val2 = 691699;

			return IIO_VAL_INT_PLUS_MICRO;
		}

		return -EINVAL;

	default:
		break;
	}

	return -EINVAL;
}

static int lradc_adc_write_raw(struct iio_dev *iio_dev,
			       const struct iio_chan_spec *chan,
			       int val, int val2, long m)
{
	struct mxs_lradc_adc *adc = iio_priv(iio_dev);
	struct mxs_lradc_scale *scale_avail = adc->scale_avail[chan->channel];
	int ret;

	ret = mutex_trylock(&adc->lock);
	if (!ret)
		return -EBUSY;

	switch (m) {
	case IIO_CHAN_INFO_SCALE:
		ret = -EINVAL;
		if (val == scale_avail[MXS_LRADC_DIV_DISABLED].integer &&
		    val2 == scale_avail[MXS_LRADC_DIV_DISABLED].nano) {
			/* divider by two disabled */
			clear_bit(chan->channel, &adc->is_divided);
			ret = 0;
		} else if (val == scale_avail[MXS_LRADC_DIV_ENABLED].integer &&
			   val2 == scale_avail[MXS_LRADC_DIV_ENABLED].nano) {
			/* divider by two enabled */
			set_bit(chan->channel, &adc->is_divided);
			ret = 0;
		}

		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&adc->lock);

	return ret;
}

static int lradc_adc_write_raw_get_fmt(struct iio_dev *iio_dev,
				       const struct iio_chan_spec *chan,
				       long m)
{
	return IIO_VAL_INT_PLUS_NANO;
}

static ssize_t lradc_adc_show_scale_available_ch(struct device *dev,
		struct device_attribute *attr,
		char *buf,
		int ch)
{
	struct iio_dev *iio = dev_to_iio_dev(dev);
	struct mxs_lradc_adc *adc = iio_priv(iio);
	int i, len = 0;

	for (i = 0; i < ARRAY_SIZE(adc->scale_avail[ch]); i++)
		len += sprintf(buf + len, "%d.%09u ",
			       adc->scale_avail[ch][i].integer,
			       adc->scale_avail[ch][i].nano);

	len += sprintf(buf + len, "\n");

	return len;
}

static ssize_t lradc_adc_show_scale_available(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev_attr *iio_attr = to_iio_dev_attr(attr);

	return lradc_adc_show_scale_available_ch(dev, attr, buf,
						 iio_attr->address);
}

#define SHOW_SCALE_AVAILABLE_ATTR(ch)					\
static IIO_DEVICE_ATTR(in_voltage##ch##_scale_available, S_IRUGO,	\
		       lradc_adc_show_scale_available, NULL, ch)

SHOW_SCALE_AVAILABLE_ATTR(0);
SHOW_SCALE_AVAILABLE_ATTR(1);
SHOW_SCALE_AVAILABLE_ATTR(2);
SHOW_SCALE_AVAILABLE_ATTR(3);
SHOW_SCALE_AVAILABLE_ATTR(4);
SHOW_SCALE_AVAILABLE_ATTR(5);
SHOW_SCALE_AVAILABLE_ATTR(6);
SHOW_SCALE_AVAILABLE_ATTR(7);
SHOW_SCALE_AVAILABLE_ATTR(10);
SHOW_SCALE_AVAILABLE_ATTR(11);
SHOW_SCALE_AVAILABLE_ATTR(12);
SHOW_SCALE_AVAILABLE_ATTR(13);
SHOW_SCALE_AVAILABLE_ATTR(14);
SHOW_SCALE_AVAILABLE_ATTR(15);

static struct attribute *lradc_adc_attributes[] = {
	&iio_dev_attr_in_voltage0_scale_available.dev_attr.attr,
	&iio_dev_attr_in_voltage1_scale_available.dev_attr.attr,
	&iio_dev_attr_in_voltage2_scale_available.dev_attr.attr,
	&iio_dev_attr_in_voltage3_scale_available.dev_attr.attr,
	&iio_dev_attr_in_voltage4_scale_available.dev_attr.attr,
	&iio_dev_attr_in_voltage5_scale_available.dev_attr.attr,
	&iio_dev_attr_in_voltage6_scale_available.dev_attr.attr,
	&iio_dev_attr_in_voltage7_scale_available.dev_attr.attr,
	&iio_dev_attr_in_voltage10_scale_available.dev_attr.attr,
	&iio_dev_attr_in_voltage11_scale_available.dev_attr.attr,
	&iio_dev_attr_in_voltage12_scale_available.dev_attr.attr,
	&iio_dev_attr_in_voltage13_scale_available.dev_attr.attr,
	&iio_dev_attr_in_voltage14_scale_available.dev_attr.attr,
	&iio_dev_attr_in_voltage15_scale_available.dev_attr.attr,
	NULL
};

static const struct attribute_group lradc_adc_attribute_group = {
	.attrs = lradc_adc_attributes,
};

static const struct iio_info lradc_adc_iio_info = {
	.driver_module		= THIS_MODULE,
	.read_raw		= lradc_adc_read_raw,
	.write_raw		= lradc_adc_write_raw,
	.write_raw_get_fmt	= lradc_adc_write_raw_get_fmt,
	.attrs			= &lradc_adc_attribute_group,
};

/*
 * IRQ Handling
 */
static irqreturn_t lradc_adc_handle_irq(int irq, void *data)
{
	struct iio_dev *iio = data;
	struct mxs_lradc_adc *adc = iio_priv(iio);
	struct mxs_lradc *lradc = adc->lradc;
	unsigned long reg = readl(lradc->base + LRADC_CTRL1);

	if (!(reg & mxs_lradc_irq_mask(lradc)))
		return IRQ_NONE;

	if (iio_buffer_enabled(iio))
		iio_trigger_poll(iio->trig);
	else if (reg & LRADC_CTRL1_LRADC_IRQ(0))
		complete(&adc->completion);

	mxs_lradc_reg_clear(lradc, reg & mxs_lradc_irq_mask(lradc),
			LRADC_CTRL1);

	return IRQ_HANDLED;
}

/*
 * Trigger handling
 */
static irqreturn_t lradc_adc_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *iio = pf->indio_dev;
	struct mxs_lradc_adc *adc = iio_priv(iio);
	const uint32_t chan_value = LRADC_CH_ACCUMULATE |
		((LRADC_DELAY_TIMER_LOOP - 1) << LRADC_CH_NUM_SAMPLES_OFFSET);
	unsigned int i, j = 0;

	for_each_set_bit(i, iio->active_scan_mask, LRADC_MAX_TOTAL_CHANS) {
		adc->buffer[j] = readl(adc->lradc->base + LRADC_CH(j));
		mxs_lradc_reg_wrt(adc->lradc, chan_value, LRADC_CH(j));
		adc->buffer[j] &= LRADC_CH_VALUE_MASK;
		adc->buffer[j] /= LRADC_DELAY_TIMER_LOOP;
		j++;
	}

	iio_push_to_buffers_with_timestamp(iio, adc->buffer, pf->timestamp);

	iio_trigger_notify_done(iio->trig);

	return IRQ_HANDLED;
}

static int lradc_adc_configure_trigger(struct iio_trigger *trig, bool state)
{
	struct iio_dev *iio = iio_trigger_get_drvdata(trig);
	struct mxs_lradc_adc *adc = iio_priv(iio);
	const uint32_t st = state ? STMP_OFFSET_REG_SET : STMP_OFFSET_REG_CLR;

	mxs_lradc_reg_wrt(adc->lradc, LRADC_DELAY_KICK, LRADC_DELAY(0) + st);

	return 0;
}

static const struct iio_trigger_ops lradc_adc_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &lradc_adc_configure_trigger,
};

static int lradc_adc_trigger_init(struct iio_dev *iio)
{
	int ret;
	struct iio_trigger *trig;
	struct mxs_lradc_adc *adc = iio_priv(iio);

	trig = iio_trigger_alloc("%s-dev%i", iio->name, iio->id);
	if (trig == NULL)
		return -ENOMEM;

	trig->dev.parent = adc->dev;
	iio_trigger_set_drvdata(trig, iio);
	trig->ops = &lradc_adc_trigger_ops;

	ret = iio_trigger_register(trig);
	if (ret) {
		iio_trigger_free(trig);
		return ret;
	}

	adc->trig = trig;

	return 0;
}

static void lradc_adc_trigger_remove(struct iio_dev *iio)
{
	struct mxs_lradc_adc *adc = iio_priv(iio);

	iio_trigger_unregister(adc->trig);
	iio_trigger_free(adc->trig);
}

static int lradc_adc_buffer_preenable(struct iio_dev *iio)
{
	struct mxs_lradc_adc *adc = iio_priv(iio);
	struct mxs_lradc *lradc = adc->lradc;
	int ret = 0, chan, ofs = 0;
	unsigned long enable = 0;
	uint32_t ctrl4_set = 0;
	uint32_t ctrl4_clr = 0;
	uint32_t ctrl1_irq = 0;
	const uint32_t chan_value = LRADC_CH_ACCUMULATE |
		((LRADC_DELAY_TIMER_LOOP - 1) << LRADC_CH_NUM_SAMPLES_OFFSET);
	const int len = bitmap_weight(iio->active_scan_mask,
			LRADC_MAX_TOTAL_CHANS);

	if (!len)
		return -EINVAL;

	/*
	 * Lock the driver so raw access can not be done during buffered
	 * operation. This simplifies the code a lot.
	 */
	ret = mutex_trylock(&adc->lock);
	if (!ret)
		return -EBUSY;

	adc->buffer = kmalloc_array(len, sizeof(*adc->buffer), GFP_KERNEL);
	if (!adc->buffer) {
		ret = -ENOMEM;
		goto err_mem;
	}

	if (lradc->soc == IMX28_LRADC)
		mxs_lradc_reg_clear(lradc, LRADC_CTRL1_MX28_LRADC_IRQ_EN_MASK,
							LRADC_CTRL1);
	mxs_lradc_reg_clear(lradc, 0xff, LRADC_CTRL0);

	for_each_set_bit(chan, iio->active_scan_mask, LRADC_MAX_TOTAL_CHANS) {
		ctrl4_set |= chan << LRADC_CTRL4_LRADCSELECT_OFFSET(ofs);
		ctrl4_clr |= LRADC_CTRL4_LRADCSELECT_MASK(ofs);
		ctrl1_irq |= LRADC_CTRL1_LRADC_IRQ_EN(ofs);
		mxs_lradc_reg_wrt(lradc, chan_value, LRADC_CH(ofs));
		bitmap_set(&enable, ofs, 1);
		ofs++;
	}

	mxs_lradc_reg_clear(lradc, LRADC_DELAY_TRIGGER_LRADCS_MASK |
					LRADC_DELAY_KICK, LRADC_DELAY(0));
	mxs_lradc_reg_clear(lradc, ctrl4_clr, LRADC_CTRL4);
	mxs_lradc_reg_set(lradc, ctrl4_set, LRADC_CTRL4);
	mxs_lradc_reg_set(lradc, ctrl1_irq, LRADC_CTRL1);
	mxs_lradc_reg_set(lradc, enable << LRADC_DELAY_TRIGGER_LRADCS_OFFSET,
					LRADC_DELAY(0));

	return 0;

err_mem:
	mutex_unlock(&adc->lock);
	return ret;
}

static int lradc_adc_buffer_postdisable(struct iio_dev *iio)
{
	struct mxs_lradc_adc *adc = iio_priv(iio);
	struct mxs_lradc *lradc = adc->lradc;

	mxs_lradc_reg_clear(lradc, LRADC_DELAY_TRIGGER_LRADCS_MASK |
					LRADC_DELAY_KICK, LRADC_DELAY(0));

	mxs_lradc_reg_clear(lradc, 0xff, LRADC_CTRL0);
	if (lradc->soc == IMX28_LRADC)
		mxs_lradc_reg_clear(lradc, LRADC_CTRL1_MX28_LRADC_IRQ_EN_MASK,
					LRADC_CTRL1);

	kfree(adc->buffer);
	mutex_unlock(&adc->lock);

	return 0;
}

static bool lradc_adc_validate_scan_mask(struct iio_dev *iio,
					const unsigned long *mask)
{
	struct mxs_lradc_adc *adc = iio_priv(iio);
	struct mxs_lradc *lradc = adc->lradc;
	const int map_chans = bitmap_weight(mask, LRADC_MAX_TOTAL_CHANS);
	int rsvd_chans = 0;
	unsigned long rsvd_mask = 0;

	if (lradc->use_touchbutton)
		rsvd_mask |= CHAN_MASK_TOUCHBUTTON;
	if (lradc->use_touchscreen == MXS_LRADC_TOUCHSCREEN_4WIRE)
		rsvd_mask |= CHAN_MASK_TOUCHSCREEN_4WIRE;
	if (lradc->use_touchscreen == MXS_LRADC_TOUCHSCREEN_5WIRE)
		rsvd_mask |= CHAN_MASK_TOUCHSCREEN_5WIRE;

	if (lradc->use_touchbutton)
		rsvd_chans++;
	if (lradc->use_touchscreen)
		rsvd_chans++;

	/* Test for attempts to map channels with special mode of operation. */
	if (bitmap_intersects(mask, &rsvd_mask, LRADC_MAX_TOTAL_CHANS))
		return false;

	/* Test for attempts to map more channels then available slots. */
	if (map_chans + rsvd_chans > LRADC_MAX_MAPPED_CHANS)
		return false;

	return true;
}

static const struct iio_buffer_setup_ops lradc_adc_buffer_ops = {
	.preenable = &lradc_adc_buffer_preenable,
	.postenable = &iio_triggered_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
	.postdisable = &lradc_adc_buffer_postdisable,
	.validate_scan_mask = &lradc_adc_validate_scan_mask,
};

/*
 * Driver initialization
 */

#define MXS_ADC_CHAN(idx, chan_type) {				\
	.type = (chan_type),					\
	.indexed = 1,						\
	.scan_index = (idx),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			      BIT(IIO_CHAN_INFO_SCALE),		\
	.channel = (idx),					\
	.address = (idx),					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = LRADC_RESOLUTION,			\
		.storagebits = 32,				\
	},							\
}

static const struct iio_chan_spec lradc_adc_chan_spec[] = {
	MXS_ADC_CHAN(0, IIO_VOLTAGE),
	MXS_ADC_CHAN(1, IIO_VOLTAGE),
	MXS_ADC_CHAN(2, IIO_VOLTAGE),
	MXS_ADC_CHAN(3, IIO_VOLTAGE),
	MXS_ADC_CHAN(4, IIO_VOLTAGE),
	MXS_ADC_CHAN(5, IIO_VOLTAGE),
	MXS_ADC_CHAN(6, IIO_VOLTAGE),
	MXS_ADC_CHAN(7, IIO_VOLTAGE),	/* VBATT */
	/* Combined Temperature sensors */
	{
		.type = IIO_TEMP,
		.indexed = 1,
		.scan_index = 8,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_OFFSET) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.channel = 8,
		.scan_type = {.sign = 'u', .realbits = 18, .storagebits = 32,},
	},
	MXS_ADC_CHAN(10, IIO_VOLTAGE),	/* VDDIO */
	MXS_ADC_CHAN(11, IIO_VOLTAGE),	/* VTH */
	MXS_ADC_CHAN(12, IIO_VOLTAGE),	/* VDDA */
	MXS_ADC_CHAN(13, IIO_VOLTAGE),	/* VDDD */
	MXS_ADC_CHAN(14, IIO_VOLTAGE),	/* VBG */
	MXS_ADC_CHAN(15, IIO_VOLTAGE),	/* VDD5V */
};

static void lradc_adc_hw_init(struct mxs_lradc_adc *adc)
{
	/* The ADC always uses DELAY CHANNEL 0. */
	const uint32_t adc_cfg =
		(1 << (LRADC_DELAY_TRIGGER_DELAYS_OFFSET + 0)) |
		(LRADC_DELAY_TIMER_PER << LRADC_DELAY_DELAY_OFFSET);

	/* Configure DELAY CHANNEL 0 for generic ADC sampling. */
	mxs_lradc_reg_wrt(adc->lradc, adc_cfg, LRADC_DELAY(0));

	/* Start internal temperature sensing. */
	mxs_lradc_reg_wrt(adc->lradc, 0, LRADC_CTRL2);
}

static void lradc_adc_hw_stop(struct mxs_lradc_adc *adc)
{
	mxs_lradc_reg_wrt(adc->lradc, 0, LRADC_DELAY(0));
}

static int lradc_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mxs_lradc *lradc = dev_get_platdata(dev);
	struct iio_dev *iio;
	struct mxs_lradc_adc *adc;
	int ret, i, s;
	uint64_t scale_uv;

	/* Allocate the IIO device. */
	iio = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!iio) {
		dev_err(dev, "Failed to allocate IIO device\n");
		return -ENOMEM;
	}

	adc = iio_priv(iio);
	adc->lradc = lradc;
	adc->dev = dev;

	init_completion(&adc->completion);
	mutex_init(&adc->lock);

	for (i = 0; i < lradc->irq_count; i++) {
		ret = devm_request_irq(dev, lradc->irq[i], lradc_adc_handle_irq,
				       IRQF_SHARED, lradc->irq_name[i], iio);
		if (ret)
			return ret;
	}

	platform_set_drvdata(pdev, iio);

	iio->name = pdev->name;
	iio->dev.parent = dev;
	iio->dev.of_node = dev->parent->of_node;
	iio->info = &lradc_adc_iio_info;
	iio->modes = INDIO_DIRECT_MODE;
	iio->channels = lradc_adc_chan_spec;
	iio->num_channels = ARRAY_SIZE(lradc_adc_chan_spec);
	iio->masklength = LRADC_MAX_TOTAL_CHANS;

	ret = iio_triggered_buffer_setup(iio, &iio_pollfunc_store_time,
				&lradc_adc_trigger_handler,
				&lradc_adc_buffer_ops);
	if (ret)
		return ret;

	ret = lradc_adc_trigger_init(iio);
	if (ret)
		goto err_trig;

	adc->vref_mv = lradc_adc_vref_mv[lradc->soc];

	/* Populate available ADC input ranges */
	for (i = 0; i < LRADC_MAX_TOTAL_CHANS; i++) {
		for (s = 0; s < ARRAY_SIZE(adc->scale_avail[i]); s++) {
			/*
			 * [s=0] = optional divider by two disabled (default)
			 * [s=1] = optional divider by two enabled
			 *
			 * The scale is calculated by doing:
			 *   Vref >> (realbits - s)
			 * which multiplies by two on the second component
			 * of the array.
			 */
			scale_uv = ((u64)adc->vref_mv[i] * 100000000) >>
				   (LRADC_RESOLUTION - s);
			adc->scale_avail[i][s].nano =
					do_div(scale_uv, 100000000) * 10;
			adc->scale_avail[i][s].integer = scale_uv;
		}
	}

	/* Configure the hardware. */
	lradc_adc_hw_init(adc);

	/* Register IIO device. */
	ret = iio_device_register(iio);
	if (ret) {
		dev_err(dev, "Failed to register IIO device\n");
		goto err_dev;
	}

	return 0;

err_dev:
	lradc_adc_hw_stop(adc);
	lradc_adc_trigger_remove(iio);
err_trig:
	iio_triggered_buffer_cleanup(iio);
	return ret;
}

static int lradc_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *iio = platform_get_drvdata(pdev);
	struct mxs_lradc_adc *adc = iio_priv(iio);

	iio_device_unregister(iio);
	lradc_adc_hw_stop(adc);
	lradc_adc_trigger_remove(iio);
	iio_triggered_buffer_cleanup(iio);

	return 0;
}

static struct platform_driver lradc_adc_driver = {
	.driver	= {
		.name = LRADC_NAME_ADC,
	},
	.probe	= lradc_adc_probe,
	.remove	= lradc_adc_remove,
};

module_platform_driver(lradc_adc_driver);

MODULE_AUTHOR("Marek Vasut <marex@denx.de>");
MODULE_DESCRIPTION("Freescale i.MX23/i.MX28 LRADC general-purpose ADC driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" LRADC_NAME_ADC);
