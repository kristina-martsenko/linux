#include <linux/module.h>
#include <linux/mfd/mxs-lradc.h>
#include <linux/platform_device.h>

enum lradc_ts_plate {
	LRADC_TOUCH = 0,
	LRADC_SAMPLE_X,
	LRADC_SAMPLE_Y,
	LRADC_SAMPLE_PRESSURE,
	LRADC_SAMPLE_VALID,
};

struct mxs_lradc_ts {
	struct mxs_lradc	*lradc;
	struct device		*dev;
	struct input_dev	*input;

	enum lradc_ts_plate	cur_plate; /* statemachine */
	bool			ts_valid;
	unsigned		ts_x_pos;
	unsigned		ts_y_pos;
	unsigned		ts_pressure;

	/* handle touchscreen's physical behaviour */
	/* samples per coordinate */
	unsigned		over_sample_cnt;
	/* time clocks between samples */
	unsigned		over_sample_delay;
	/* time in clocks to wait after the plates where switched */
	unsigned		settling_delay;
};

static u32 mxs_lradc_plate_mask(struct mxs_lradc *lradc)
{
	if (lradc->soc == IMX23_LRADC)
		return LRADC_CTRL0_MX23_PLATE_MASK;
	return LRADC_CTRL0_MX28_PLATE_MASK;
}

static u32 mxs_lradc_touch_detect_bit(struct mxs_lradc *lradc)
{
	if (lradc->soc == IMX23_LRADC)
		return LRADC_CTRL0_MX23_TOUCH_DETECT_ENABLE;
	return LRADC_CTRL0_MX28_TOUCH_DETECT_ENABLE;
}

static u32 mxs_lradc_drive_x_plate(struct mxs_lradc *lradc)
{
	if (lradc->soc == IMX23_LRADC)
		return LRADC_CTRL0_MX23_XP | LRADC_CTRL0_MX23_XM;
	return LRADC_CTRL0_MX28_XPPSW | LRADC_CTRL0_MX28_XNNSW;
}

static u32 mxs_lradc_drive_y_plate(struct mxs_lradc *lradc)
{
	if (lradc->soc == IMX23_LRADC)
		return LRADC_CTRL0_MX23_YP | LRADC_CTRL0_MX23_YM;
	return LRADC_CTRL0_MX28_YPPSW | LRADC_CTRL0_MX28_YNNSW;
}

static u32 mxs_lradc_drive_pressure(struct mxs_lradc *lradc)
{
	if (lradc->soc == IMX23_LRADC)
		return LRADC_CTRL0_MX23_YP | LRADC_CTRL0_MX23_XM;
	return LRADC_CTRL0_MX28_YPPSW | LRADC_CTRL0_MX28_XNNSW;
}

static bool mxs_lradc_check_touch_event(struct mxs_lradc *lradc)
{
	return !!(readl(lradc->base + LRADC_STATUS) &
					LRADC_STATUS_TOUCH_DETECT_RAW);
}

static void mxs_lradc_setup_ts_channel(struct mxs_lradc_ts *ts, unsigned ch)
{
	struct mxs_lradc *lradc = ts->lradc;

	/*
	 * prepare for oversampling conversion
	 *
	 * from the datasheet:
	 * "The ACCUMULATE bit in the appropriate channel register
	 * HW_LRADC_CHn must be set to 1 if NUM_SAMPLES is greater then 0;
	 * otherwise, the IRQs will not fire."
	 */
	mxs_lradc_reg_wrt(lradc, LRADC_CH_ACCUMULATE |
			LRADC_CH_NUM_SAMPLES(ts->over_sample_cnt - 1),
			LRADC_CH(ch));

	/* from the datasheet:
	 * "Software must clear this register in preparation for a
	 * multi-cycle accumulation.
	 */
	mxs_lradc_reg_clear(lradc, LRADC_CH_VALUE_MASK, LRADC_CH(ch));

	/* prepare the delay/loop unit according to the oversampling count */
	mxs_lradc_reg_wrt(lradc, LRADC_DELAY_TRIGGER(1 << ch) |
		LRADC_DELAY_TRIGGER_DELAYS(0) |
		LRADC_DELAY_LOOP(ts->over_sample_cnt - 1) |
		LRADC_DELAY_DELAY(ts->over_sample_delay - 1),
			LRADC_DELAY(3));

	mxs_lradc_reg_clear(lradc, LRADC_CTRL1_LRADC_IRQ(2) |
			LRADC_CTRL1_LRADC_IRQ(3) | LRADC_CTRL1_LRADC_IRQ(4) |
			LRADC_CTRL1_LRADC_IRQ(5), LRADC_CTRL1);

	/* wake us again, when the complete conversion is done */
	mxs_lradc_reg_set(lradc, LRADC_CTRL1_LRADC_IRQ_EN(ch), LRADC_CTRL1);
	/*
	 * after changing the touchscreen plates setting
	 * the signals need some initial time to settle. Start the
	 * SoC's delay unit and start the conversion later
	 * and automatically.
	 */
	mxs_lradc_reg_wrt(lradc, LRADC_DELAY_TRIGGER(0) | /* don't trigger ADC */
		LRADC_DELAY_TRIGGER_DELAYS(1 << 3) | /* trigger DELAY unit#3 */
		LRADC_DELAY_KICK |
		LRADC_DELAY_DELAY(ts->settling_delay),
			LRADC_DELAY(2));
}

/*
 * Pressure detection is special:
 * We want to do both required measurements for the pressure detection in
 * one turn. Use the hardware features to chain both conversions and let the
 * hardware report one interrupt if both conversions are done
 */
static void mxs_lradc_setup_ts_pressure(struct mxs_lradc_ts *ts, unsigned ch1,
							unsigned ch2)
{
	struct mxs_lradc *lradc = ts->lradc;
	u32 reg;

	/*
	 * prepare for oversampling conversion
	 *
	 * from the datasheet:
	 * "The ACCUMULATE bit in the appropriate channel register
	 * HW_LRADC_CHn must be set to 1 if NUM_SAMPLES is greater then 0;
	 * otherwise, the IRQs will not fire."
	 */
	reg = LRADC_CH_ACCUMULATE |
		LRADC_CH_NUM_SAMPLES(ts->over_sample_cnt - 1);
	mxs_lradc_reg_wrt(lradc, reg, LRADC_CH(ch1));
	mxs_lradc_reg_wrt(lradc, reg, LRADC_CH(ch2));

	/* from the datasheet:
	 * "Software must clear this register in preparation for a
	 * multi-cycle accumulation.
	 */
	mxs_lradc_reg_clear(lradc, LRADC_CH_VALUE_MASK, LRADC_CH(ch1));
	mxs_lradc_reg_clear(lradc, LRADC_CH_VALUE_MASK, LRADC_CH(ch2));

	/* prepare the delay/loop unit according to the oversampling count */
	mxs_lradc_reg_wrt(lradc, LRADC_DELAY_TRIGGER(1 << ch1) |
		LRADC_DELAY_TRIGGER(1 << ch2) | /* start both channels */
		LRADC_DELAY_TRIGGER_DELAYS(0) |
		LRADC_DELAY_LOOP(ts->over_sample_cnt - 1) |
		LRADC_DELAY_DELAY(ts->over_sample_delay - 1),
					LRADC_DELAY(3));

	mxs_lradc_reg_clear(lradc, LRADC_CTRL1_LRADC_IRQ(2) |
			LRADC_CTRL1_LRADC_IRQ(3) | LRADC_CTRL1_LRADC_IRQ(4) |
			LRADC_CTRL1_LRADC_IRQ(5), LRADC_CTRL1);

	/* wake us again, when the conversions are done */
	mxs_lradc_reg_set(lradc, LRADC_CTRL1_LRADC_IRQ_EN(ch2), LRADC_CTRL1);
	/*
	 * after changing the touchscreen plates setting
	 * the signals need some initial time to settle. Start the
	 * SoC's delay unit and start the conversion later
	 * and automatically.
	 */
	mxs_lradc_reg_wrt(lradc, LRADC_DELAY_TRIGGER(0) | /* don't trigger ADC */
		LRADC_DELAY_TRIGGER_DELAYS(1 << 3) | /* trigger DELAY unit#3 */
		LRADC_DELAY_KICK |
		LRADC_DELAY_DELAY(ts->settling_delay), LRADC_DELAY(2));
}

static unsigned mxs_lradc_read_raw_channel(struct mxs_lradc_ts *ts,
							unsigned channel)
{
	u32 reg;
	unsigned num_samples, val;

	reg = readl(ts->lradc->base + LRADC_CH(channel));
	if (reg & LRADC_CH_ACCUMULATE)
		num_samples = ts->over_sample_cnt;
	else
		num_samples = 1;

	val = (reg & LRADC_CH_VALUE_MASK) >> LRADC_CH_VALUE_OFFSET;
	return val / num_samples;
}

static unsigned mxs_lradc_read_ts_pressure(struct mxs_lradc_ts *ts,
						unsigned ch1, unsigned ch2)
{
	u32 reg, mask;
	unsigned pressure, m1, m2;

	mask = LRADC_CTRL1_LRADC_IRQ(ch1) | LRADC_CTRL1_LRADC_IRQ(ch2);
	reg = readl(ts->lradc->base + LRADC_CTRL1) & mask;

	while (reg != mask) {
		reg = readl(ts->lradc->base + LRADC_CTRL1) & mask;
		dev_dbg(ts->dev, "One channel is still busy: %X\n", reg);
	}

	m1 = mxs_lradc_read_raw_channel(ts, ch1);
	m2 = mxs_lradc_read_raw_channel(ts, ch2);

	if (m2 == 0) {
		dev_warn(ts->dev, "Cannot calculate pressure\n");
		return 1 << (LRADC_RESOLUTION - 1);
	}

	/* simply scale the value from 0 ... max ADC resolution */
	pressure = m1;
	pressure *= (1 << LRADC_RESOLUTION);
	pressure /= m2;

	dev_dbg(ts->dev, "Pressure = %u\n", pressure);
	return pressure;
}

#define TS_CH_XP 2
#define TS_CH_YP 3
#define TS_CH_XM 4
#define TS_CH_YM 5

static int mxs_lradc_read_ts_channel(struct mxs_lradc_ts *ts)
{
	struct mxs_lradc *lradc = ts->lradc;
	u32 reg;
	int val;

	reg = readl(lradc->base + LRADC_CTRL1);

	/* only channels 3 to 5 are of interest here */
	if (reg & LRADC_CTRL1_LRADC_IRQ(TS_CH_YP)) {
		mxs_lradc_reg_clear(lradc, LRADC_CTRL1_LRADC_IRQ_EN(TS_CH_YP) |
			LRADC_CTRL1_LRADC_IRQ(TS_CH_YP), LRADC_CTRL1);
		val = mxs_lradc_read_raw_channel(ts, TS_CH_YP);
	} else if (reg & LRADC_CTRL1_LRADC_IRQ(TS_CH_XM)) {
		mxs_lradc_reg_clear(lradc, LRADC_CTRL1_LRADC_IRQ_EN(TS_CH_XM) |
			LRADC_CTRL1_LRADC_IRQ(TS_CH_XM), LRADC_CTRL1);
		val = mxs_lradc_read_raw_channel(ts, TS_CH_XM);
	} else if (reg & LRADC_CTRL1_LRADC_IRQ(TS_CH_YM)) {
		mxs_lradc_reg_clear(lradc, LRADC_CTRL1_LRADC_IRQ_EN(TS_CH_YM) |
			LRADC_CTRL1_LRADC_IRQ(TS_CH_YM), LRADC_CTRL1);
		val = mxs_lradc_read_raw_channel(ts, TS_CH_YM);
	} else {
		return -EIO;
	}

	mxs_lradc_reg_wrt(lradc, 0, LRADC_DELAY(2));
	mxs_lradc_reg_wrt(lradc, 0, LRADC_DELAY(3));

	return val;
}

/*
 * YP(open)--+-------------+
 *           |             |--+
 *           |             |  |
 *    YM(-)--+-------------+  |
 *             +--------------+
 *             |              |
 *         XP(weak+)        XM(open)
 *
 * "weak+" means 200k Ohm VDDIO
 * (-) means GND
 */
static void mxs_lradc_setup_touch_detection(struct mxs_lradc_ts *ts)
{
	struct mxs_lradc *lradc = ts->lradc;

	/*
	 * In order to detect a touch event the 'touch detect enable' bit
	 * enables:
	 *  - a weak pullup to the X+ connector
	 *  - a strong ground at the Y- connector
	 */
	mxs_lradc_reg_clear(lradc, mxs_lradc_plate_mask(lradc), LRADC_CTRL0);
	mxs_lradc_reg_set(lradc, mxs_lradc_touch_detect_bit(lradc),
				LRADC_CTRL0);
}

/*
 * YP(meas)--+-------------+
 *           |             |--+
 *           |             |  |
 * YM(open)--+-------------+  |
 *             +--------------+
 *             |              |
 *           XP(+)          XM(-)
 *
 * (+) means here 1.85 V
 * (-) means here GND
 */
static void mxs_lradc_prepare_x_pos(struct mxs_lradc_ts *ts)
{
	struct mxs_lradc *lradc = ts->lradc;

	mxs_lradc_reg_clear(lradc, mxs_lradc_plate_mask(lradc), LRADC_CTRL0);
	mxs_lradc_reg_set(lradc, mxs_lradc_drive_x_plate(lradc), LRADC_CTRL0);

	ts->cur_plate = LRADC_SAMPLE_X;
	mxs_lradc_setup_ts_channel(ts, TS_CH_YP);
}

/*
 *   YP(+)--+-------------+
 *          |             |--+
 *          |             |  |
 *   YM(-)--+-------------+  |
 *            +--------------+
 *            |              |
 *         XP(open)        XM(meas)
 *
 * (+) means here 1.85 V
 * (-) means here GND
 */
static void mxs_lradc_prepare_y_pos(struct mxs_lradc_ts *ts)
{
	struct mxs_lradc *lradc = ts->lradc;

	mxs_lradc_reg_clear(lradc, mxs_lradc_plate_mask(lradc), LRADC_CTRL0);
	mxs_lradc_reg_set(lradc, mxs_lradc_drive_y_plate(lradc), LRADC_CTRL0);

	ts->cur_plate = LRADC_SAMPLE_Y;
	mxs_lradc_setup_ts_channel(ts, TS_CH_XM);
}

/*
 *    YP(+)--+-------------+
 *           |             |--+
 *           |             |  |
 * YM(meas)--+-------------+  |
 *             +--------------+
 *             |              |
 *          XP(meas)        XM(-)
 *
 * (+) means here 1.85 V
 * (-) means here GND
 */
static void mxs_lradc_prepare_pressure(struct mxs_lradc_ts *ts)
{
	struct mxs_lradc *lradc = ts->lradc;

	mxs_lradc_reg_clear(lradc, mxs_lradc_plate_mask(lradc), LRADC_CTRL0);
	mxs_lradc_reg_set(lradc, mxs_lradc_drive_pressure(lradc), LRADC_CTRL0);

	ts->cur_plate = LRADC_SAMPLE_PRESSURE;
	mxs_lradc_setup_ts_pressure(ts, TS_CH_XP, TS_CH_YM);
}

static void mxs_lradc_enable_touch_detection(struct mxs_lradc_ts *ts)
{
	mxs_lradc_setup_touch_detection(ts);

	ts->cur_plate = LRADC_TOUCH;
	mxs_lradc_reg_clear(ts->lradc, LRADC_CTRL1_TOUCH_DETECT_IRQ |
				LRADC_CTRL1_TOUCH_DETECT_IRQ_EN, LRADC_CTRL1);
	mxs_lradc_reg_set(ts->lradc, LRADC_CTRL1_TOUCH_DETECT_IRQ_EN,
			  LRADC_CTRL1);
}

static void mxs_lradc_report_ts_event(struct mxs_lradc_ts *ts)
{
	input_report_abs(ts->input, ABS_X, ts->ts_x_pos);
	input_report_abs(ts->input, ABS_Y, ts->ts_y_pos);
	input_report_abs(ts->input, ABS_PRESSURE, ts->ts_pressure);
	input_report_key(ts->input, BTN_TOUCH, 1);
	input_sync(ts->input);
}

static void mxs_lradc_complete_touch_event(struct mxs_lradc_ts *ts)
{
	struct mxs_lradc *lradc = ts->lradc;

	mxs_lradc_setup_touch_detection(ts);
	ts->cur_plate = LRADC_SAMPLE_VALID;
	/*
	 * start a dummy conversion to burn time to settle the signals
	 * note: we are not interested in the conversion's value
	 */
	mxs_lradc_reg_wrt(lradc, 0, LRADC_CH(5));
	mxs_lradc_reg_clear(lradc, LRADC_CTRL1_LRADC_IRQ(5), LRADC_CTRL1);
	mxs_lradc_reg_set(lradc, LRADC_CTRL1_LRADC_IRQ_EN(5), LRADC_CTRL1);
	mxs_lradc_reg_wrt(lradc, LRADC_DELAY_TRIGGER(1 << 5) |
		LRADC_DELAY_KICK | LRADC_DELAY_DELAY(10), /* waste 5 ms */
			LRADC_DELAY(2));
}

/*
 * in order to avoid false measurements, report only samples where
 * the surface is still touched after the position measurement
 */
static void mxs_lradc_finish_touch_event(struct mxs_lradc_ts *ts, bool valid)
{
	struct mxs_lradc *lradc = ts->lradc;

	/* if it is still touched, report the sample */
	if (valid && mxs_lradc_check_touch_event(lradc)) {
		ts->ts_valid = true;
		mxs_lradc_report_ts_event(ts);
	}

	/* if it is even still touched, continue with the next measurement */
	if (mxs_lradc_check_touch_event(lradc)) {
		mxs_lradc_prepare_y_pos(ts);
		return;
	}

	if (ts->ts_valid) {
		/* signal the release */
		ts->ts_valid = false;
		input_report_key(ts->input, BTN_TOUCH, 0);
		input_sync(ts->input);
	}

	/* if it is released, wait for the next touch via IRQ */
	ts->cur_plate = LRADC_TOUCH;
	mxs_lradc_reg_clear(lradc, LRADC_CTRL1_TOUCH_DETECT_IRQ, LRADC_CTRL1);
	mxs_lradc_reg_set(lradc, LRADC_CTRL1_TOUCH_DETECT_IRQ_EN, LRADC_CTRL1);
}

/* touchscreen's state machine */
static void mxs_lradc_handle_touch(struct mxs_lradc_ts *ts)
{
	struct mxs_lradc *lradc = ts->lradc;
	int val;

	switch (ts->cur_plate) {
	case LRADC_TOUCH:
		/*
		 * start with the Y-pos, because it uses nearly the same plate
		 * settings like the touch detection
		 */
		if (mxs_lradc_check_touch_event(lradc)) {
			mxs_lradc_reg_clear(lradc,
					LRADC_CTRL1_TOUCH_DETECT_IRQ_EN,
					LRADC_CTRL1);
			mxs_lradc_prepare_y_pos(ts);
		}
		mxs_lradc_reg_clear(lradc, LRADC_CTRL1_TOUCH_DETECT_IRQ,
					LRADC_CTRL1);
		return;

	case LRADC_SAMPLE_Y:
		val = mxs_lradc_read_ts_channel(ts);
		if (val < 0) {
			mxs_lradc_enable_touch_detection(ts); /* re-start */
			return;
		}
		ts->ts_y_pos = val;
		mxs_lradc_prepare_x_pos(ts);
		return;

	case LRADC_SAMPLE_X:
		val = mxs_lradc_read_ts_channel(ts);
		if (val < 0) {
			mxs_lradc_enable_touch_detection(ts); /* re-start */
			return;
		}
		ts->ts_x_pos = val;
		mxs_lradc_prepare_pressure(ts);
		return;

	case LRADC_SAMPLE_PRESSURE:
		ts->ts_pressure =
			mxs_lradc_read_ts_pressure(ts, TS_CH_XP, TS_CH_YM);
		mxs_lradc_complete_touch_event(ts);
		return;

	case LRADC_SAMPLE_VALID:
		val = mxs_lradc_read_ts_channel(ts); /* ignore the value */
		mxs_lradc_finish_touch_event(ts, 1);
		break;
	}
}

static irqreturn_t lradc_ts_handle_irq(int irq, void *data)
{
	struct mxs_lradc_ts *ts = data;
	struct mxs_lradc *lradc = ts->lradc;
	unsigned long reg = readl(lradc->base + LRADC_CTRL1);
	const uint32_t ts_irq_mask =
		LRADC_CTRL1_TOUCH_DETECT_IRQ |
		LRADC_CTRL1_LRADC_IRQ(2) |
		LRADC_CTRL1_LRADC_IRQ(3) |
		LRADC_CTRL1_LRADC_IRQ(4) |
		LRADC_CTRL1_LRADC_IRQ(5);

	if (!(reg & ts_irq_mask))
		return IRQ_NONE;

	mxs_lradc_handle_touch(ts);

	mxs_lradc_reg_clear(lradc, reg & mxs_lradc_irq_mask(lradc),
			LRADC_CTRL1);

	return IRQ_HANDLED;
}

static int lradc_ts_open(struct input_dev *dev)
{
	struct mxs_lradc_ts *ts = input_get_drvdata(dev);

	/* Enable the touch-detect circuitry. */
	mxs_lradc_enable_touch_detection(ts);

	return 0;
}

static void lradc_ts_disable(struct mxs_lradc_ts *ts)
{
	struct mxs_lradc *lradc = ts->lradc;

	/* stop all interrupts from firing */
	mxs_lradc_reg_clear(lradc, LRADC_CTRL1_TOUCH_DETECT_IRQ_EN |
		LRADC_CTRL1_LRADC_IRQ_EN(2) | LRADC_CTRL1_LRADC_IRQ_EN(3) |
		LRADC_CTRL1_LRADC_IRQ_EN(4) | LRADC_CTRL1_LRADC_IRQ_EN(5),
		LRADC_CTRL1);

	/* Power-down touchscreen touch-detect circuitry. */
	mxs_lradc_reg_clear(lradc, mxs_lradc_plate_mask(lradc), LRADC_CTRL0);
}

static void lradc_ts_close(struct input_dev *dev)
{
	struct mxs_lradc_ts *ts = input_get_drvdata(dev);

	lradc_ts_disable(ts);
}

static void lradc_ts_hw_init(struct mxs_lradc_ts *ts)
{
	struct mxs_lradc *lradc = ts->lradc;

	/* Configure the touchscreen type */
	if (lradc->soc == IMX28_LRADC) {
		mxs_lradc_reg_clear(lradc, LRADC_CTRL0_MX28_TOUCH_SCREEN_TYPE,
							LRADC_CTRL0);
	if (lradc->use_touchscreen == MXS_LRADC_TOUCHSCREEN_5WIRE)
		mxs_lradc_reg_set(lradc, LRADC_CTRL0_MX28_TOUCH_SCREEN_TYPE,
				LRADC_CTRL0);
	}
}

static int lradc_ts_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *lradc_node = dev->parent->of_node;
	struct mxs_lradc *lradc = dev_get_platdata(dev);
	struct mxs_lradc_ts *ts;
	struct input_dev *input;
	int ret, i;
	u32 adapt;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	input = devm_input_allocate_device(dev);
	if (!ts || !input)
		return -ENOMEM;

	ts->lradc = lradc;
	ts->dev = dev;
	ts->input = input;
	platform_set_drvdata(pdev, ts);
	input_set_drvdata(input, ts);

	ts->over_sample_cnt = 4;
	ret = of_property_read_u32(lradc_node, "fsl,ave-ctrl", &adapt);
	if (ret == 0)
		ts->over_sample_cnt = adapt;

	ts->over_sample_delay = 2;
	ret = of_property_read_u32(lradc_node, "fsl,ave-delay", &adapt);
	if (ret == 0)
		ts->over_sample_delay = adapt;

	ts->settling_delay = 10;
	ret = of_property_read_u32(lradc_node, "fsl,settling", &adapt);
	if (ret == 0)
		ts->settling_delay = adapt;

	input->name = LRADC_NAME_TS;
	input->id.bustype = BUS_HOST;
	input->open = lradc_ts_open;
	input->close = lradc_ts_close;

	__set_bit(EV_ABS, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);
	input_set_abs_params(input, ABS_X, 0, LRADC_SINGLE_SAMPLE_MASK, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, LRADC_SINGLE_SAMPLE_MASK, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, LRADC_SINGLE_SAMPLE_MASK,
			     0, 0);

	lradc_ts_hw_init(ts);

	for (i = 0; i < lradc->irq_count; i++) {
		ret = request_threaded_irq(lradc->irq[i], lradc_ts_handle_irq,
					   NULL, IRQF_SHARED,
					   lradc->irq_name[i], ts);
		if (ret)
			goto err_irq;
	}

	ret = input_register_device(input);
	if (ret)
		goto err_irq;

	return 0;

err_irq:
	while (--i >= 0)
		free_irq(lradc->irq[i], ts);
	return ret;
}

static int lradc_ts_remove(struct platform_device *pdev)
{
	struct mxs_lradc_ts *ts = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < ts->lradc->irq_count; i++)
		free_irq(ts->lradc->irq[i], ts);

	lradc_ts_disable(ts);

	return 0;
}

static struct platform_driver lradc_ts_driver = {
	.driver	= {
		.name = LRADC_NAME_TS,
	},
	.probe	= lradc_ts_probe,
	.remove	= lradc_ts_remove,
};
module_platform_driver(lradc_ts_driver);

MODULE_LICENSE("GPL v2");
