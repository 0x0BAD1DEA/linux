/*
 * Functions to access SY3686A power management chip voltages
 *
 * Copyright (C) 2019 reMarkable AS - http://www.remarkable.com/
 *
 * Author: Lars Ivar Miljeteig <lars.ivar.miljeteig@remarkable.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <linux/mfd/sy7636a.h>

static int sy7636a_regulator_get_voltage(struct regulator_dev *rdev)
{
	int ret, vcom;

	ret = sy7636a_get_vcom_voltage(rdev->regmap, &vcom);
	if (ret)
		return ret;

	return -vcom;
}

static int sy7636a_regulator_disable(struct regulator_dev *rdev)
{
	int ret;

	ret = regulator_disable_regmap(rdev);
	usleep_range(30000, 35000);

	return ret;
}

static int sy7636a_regulator_enable(struct regulator_dev *rdev)
{
	struct sy7636a *sy7636a = dev_get_drvdata(rdev->dev.parent);

	ret = regulator_enable_regmap(rdev);
	if (ret)
		return ret;

	ret = sy7636a_get_powergood(rdev->dev.parent);
	if (ret)
		return ret;

	return 0;
}

static int sy7636a_regulator_init(struct device *dev)
{
	struct sy7636a *sy7636a = dev_get_drvdata(dev->parent);

	return regmap_write(sy7636a->regmap, SY7636A_REG_POWER_ON_DELAY_TIME, 0x00);
}

static int sy7636a_regulator_suspend(struct device *dev)
{
	return sy7636a_vcom_suspend(dev);
}

static int sy7636a_regulator_resume(struct device *dev)
{
	int ret;

	ret = sy7636a_vcom_resume(dev);
	if (ret)
		return ret;

	return sy7636a_regulator_init(dev);
}

static int sy7636a_regulator_probe(struct platform_device *pdev)
{
	int ret;
	struct sy7636a *sy7636a = dev_get_drvdata(pdev->dev.parent);
	struct regulator_config config = { };
	struct regulator_dev *rdev;

	if (!sy7636a)
		return -EPROBE_DEFER;

	platform_set_drvdata(pdev, sy7636a);

	ret = sy7636a_regulator_init(sy7636a);
	if (ret) {
		dev_err(sy7636a->dev, "Failed to initialize regulator: %d\n", ret);
		return ret;
	}

	config.dev = &pdev->dev;
	config.dev->of_node = sy7636a->dev->of_node;
	config.driver_data = sy7636a;
	config.regmap = sy7636a->regmap;

	rdev = devm_regulator_register(&pdev->dev, &desc, &config);
	if (IS_ERR(rdev)) {
		dev_err(sy7636a->dev, "Failed to register %s regulator\n", pdev->name);
		return PTR_ERR(rdev);
	}

	return 0;
}

static const struct regulator_ops sy7636a_vcom_volt_ops = {
	.get_voltage = sy7636a_regulator_get_voltage,
	.enable = sy7636a_regulator_enable,
	.disable = sy7636a_regulator_disable,
	.is_enabled = regulator_is_enabled_regmap,
};

struct regulator_desc desc = {
	.name = "vcom",
	.id = 0,
	.ops = &sy7636a_vcom_volt_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.enable_reg = SY7636A_REG_OPERATION_MODE_CRL,
	.enable_mask = SY7636A_OPERATION_MODE_CRL_ONOFF,
	.regulators_node = of_match_ptr("regulators"),
	.of_match = of_match_ptr("vcom"),
};

static const struct platform_device_id sy7636a_regulator_id_table[] = {
	{ "sy7636a-regulator", },
};
MODULE_DEVICE_TABLE(platform, sy7636a_regulator_id_table);

static const struct dev_pm_ops sy7636a_pm_ops = {
	.suspend = sy7636a_regulator_suspend,
	.resume = sy7636a_regulator_resume,
};

static struct platform_driver sy7636a_regulator_driver = {
	.driver = {
		.name = "sy7636a-regulator",
		.pm = &sy7636a_pm_ops,
	},
	.probe = sy7636a_regulator_probe,
	.id_table = sy7636a_regulator_id_table,
};
module_platform_driver(sy7636a_regulator_driver);

MODULE_AUTHOR("Lars Ivar Miljeteig <lars.ivar.miljeteig@remarkable.com>");
MODULE_DESCRIPTION("SY7636A voltage regulator driver");
MODULE_LICENSE("GPL v2");
