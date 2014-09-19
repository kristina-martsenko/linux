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

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/mfd/core.h>
#include <linux/mfd/mxs-lradc.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

static struct mfd_cell lradc_adc_dev = {
	.name = LRADC_NAME_ADC,
};

static struct mfd_cell lradc_ts_dev = {
	.name = LRADC_NAME_TS,
};

static const char * const mx23_lradc_irq_names[] = {
	"mxs-lradc-touchscreen",
	"mxs-lradc-channel0",
	"mxs-lradc-channel1",
	"mxs-lradc-channel2",
	"mxs-lradc-channel3",
	"mxs-lradc-channel4",
	"mxs-lradc-channel5",
	"mxs-lradc-channel6",
	"mxs-lradc-channel7",
};

static const char * const mx28_lradc_irq_names[] = {
	"mxs-lradc-touchscreen",
	"mxs-lradc-thresh0",
	"mxs-lradc-thresh1",
	"mxs-lradc-channel0",
	"mxs-lradc-channel1",
	"mxs-lradc-channel2",
	"mxs-lradc-channel3",
	"mxs-lradc-channel4",
	"mxs-lradc-channel5",
	"mxs-lradc-channel6",
	"mxs-lradc-channel7",
	"mxs-lradc-button0",
	"mxs-lradc-button1",
};

struct mxs_lradc_of_config {
	const int		irq_count;
	const char * const	*irq_name;
};

static const struct mxs_lradc_of_config mxs_lradc_of_config[] = {
	[IMX23_LRADC] = {
		.irq_count	= ARRAY_SIZE(mx23_lradc_irq_names),
		.irq_name	= mx23_lradc_irq_names,
	},
	[IMX28_LRADC] = {
		.irq_count	= ARRAY_SIZE(mx28_lradc_irq_names),
		.irq_name	= mx28_lradc_irq_names,
	},
};

static const struct of_device_id mxs_lradc_dt_ids[] = {
	{ .compatible = "fsl,imx23-lradc", .data = (void *)IMX23_LRADC, },
	{ .compatible = "fsl,imx28-lradc", .data = (void *)IMX28_LRADC, },
	{ },
};
MODULE_DEVICE_TABLE(of, mxs_lradc_dt_ids);

static int mxs_lradc_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
		of_match_device(mxs_lradc_dt_ids, &pdev->dev);
	const struct mxs_lradc_of_config *of_cfg =
		&mxs_lradc_of_config[(enum mxs_lradc_id)of_id->data];
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct mxs_lradc *lradc;
	struct resource *iores;
	u32 ts_wires = 0;
	int ret = 0;
	int i;

	lradc = devm_kzalloc(&pdev->dev, sizeof(*lradc), GFP_KERNEL);
	if (!lradc)
		return -ENOMEM;

	lradc->soc = (enum mxs_lradc_id)of_id->data;

	/* Grab the memory area */
	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	lradc->base = devm_ioremap_resource(dev, iores);
	if (IS_ERR(lradc->base))
		return PTR_ERR(lradc->base);

	lradc->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(lradc->clk)) {
		dev_err(dev, "Failed to get the delay unit clock\n");
		return PTR_ERR(lradc->clk);
	}
	ret = clk_prepare_enable(lradc->clk);
	if (ret != 0) {
		dev_err(dev, "Failed to enable the delay unit clock\n");
		return ret;
	}

	lradc->irq_count = of_cfg->irq_count;
	lradc->irq_name = of_cfg->irq_name;
	for (i = 0; i < lradc->irq_count; i++) {
		lradc->irq[i] = platform_get_irq(pdev, i);
		if (lradc->irq[i] < 0)
			return lradc->irq[i];
	}

	platform_set_drvdata(pdev, lradc);

	ret = stmp_reset_block(lradc->base);
	if (ret)
		return ret;

	lradc_adc_dev.platform_data = lradc;
	lradc_adc_dev.pdata_size = sizeof(*lradc);

	ret = mfd_add_devices(&pdev->dev, -1, &lradc_adc_dev, 1, NULL, 0, NULL);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add the ADC subdevice\n");
		return ret;
	}

	ret = of_property_read_u32(node, "fsl,lradc-touchscreen-wires",
				   &ts_wires);
	if (ret)
		return 0; /* touchscreen feature disabled */

	lradc_ts_dev.platform_data = lradc;
	lradc_ts_dev.pdata_size = sizeof(*lradc);

	switch (ts_wires) {
	case 4:
		lradc->use_touchscreen = MXS_LRADC_TOUCHSCREEN_4WIRE;
		break;
	case 5:
		if (lradc->soc == IMX28_LRADC) {
			lradc->use_touchscreen = MXS_LRADC_TOUCHSCREEN_5WIRE;
			break;
		}
		/* fall through an error message for i.MX23 */
	default:
		dev_err(&pdev->dev,
			"Unsupported number of touchscreen wires (%d)\n",
			ts_wires);
		return 0;
	}

	ret = mfd_add_devices(&pdev->dev, -1, &lradc_ts_dev, 1, NULL, 0, NULL);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to add the touchscreen subdevice\n");
		goto err_remove_adc;
	}

	return 0;

err_remove_adc:
	mfd_remove_devices(&pdev->dev);
	return ret;
}

static int mxs_lradc_remove(struct platform_device *pdev)
{
	struct mxs_lradc *lradc = platform_get_drvdata(pdev);

	mfd_remove_devices(&pdev->dev);
	clk_disable_unprepare(lradc->clk);
	return 0;
}

static struct platform_driver mxs_lradc_driver = {
	.driver = {
		.name = "mxs-lradc",
		.of_match_table = mxs_lradc_dt_ids,
	},
	.probe = mxs_lradc_probe,
	.remove = mxs_lradc_remove,
};

module_platform_driver(mxs_lradc_driver);

MODULE_DESCRIPTION("Freescale i.MX23/i.MX28 LRADC driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mxs-lradc");
