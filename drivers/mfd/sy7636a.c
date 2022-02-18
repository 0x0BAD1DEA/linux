/*
 * MFD driver for SY7636A chip
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
 *
 * Based on the lp87565 driver by Keerthy <j-keerthy@ti.com>
 */

#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>
#include <linux/gpio/consumer.h>

#include <linux/mfd/sy7636a.h>

static const struct regmap_config sy7636a_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static const struct mfd_cell sy7636a_cells[] = {
	{ .name = "sy7636a-regulator", },
	{ .name = "sy7636a-temperature", },
	{ .name = "sy7636a-thermal", },
};

static const struct of_device_id of_sy7636a_match_table[] = {
	{ .compatible = "silergy,sy7636a", },
	{}
};
MODULE_DEVICE_TABLE(of, of_sy7636a_match_table);

static const char *pgood[] = {
	"REG=OFF, GPIO=OFF",
	"REG=ON, GPIO=OFF",
	"REG=OFF, GPIO=ON",
	"REG=ON, GPIO=ON"
};

static const char *states[] = {
	"no fault event",
	"UVP at VP rail",
	"UVP at VN rail",
	"UVP at VPOS rail",
	"UVP at VNEG rail",
	"UVP at VDDH rail",
	"UVP at VEE rail",
	"SCP at VP rail",
	"SCP at VN rail",
	"SCP at VPOS rail",
	"SCP at VNEG rail",
	"SCP at VDDH rail",
	"SCP at VEE rail",
	"SCP at V COM rail",
	"UVLO",
	"Thermal shutdown",
};


// Start of VCOM functions
int sy7636a_vcom_init(struct sy7636a *sy7636a)
{
	return regmap_write(sy7636a->regmap, SY7636A_REG_POWER_ON_DELAY_TIME, 0x00);
}
int sy7636a_vcom_get_voltage(struct device *dev, int *value)
{
	int ret;
	unsigned int val, val_h;
	int vcom;
	struct sy7636a *sy7636a = dev_get_drvdata(dev);

	ret = regmap_read(sy7636a->regmap, SY7636A_REG_VCOM_ADJUST_CTRL_L, &val);
	if (ret)
		return ret;
	ret = regmap_read(sy7636a->regmap, SY7636A_REG_VCOM_ADJUST_CTRL_H, &val_h);
	if (ret)
		return ret;
	val |= (val_h << 8);

	vcom = -(val & SY7636A_REG_VCOM_ADJUST_CTRL_MASK) * SY7636A_REG_VCOM_ADJUST_CTRL_SCAL;
	dev_info(sy7636a->dev, "SY7636a: vcom_get: Raw vcom is %d\n", vcom);

	// If this is a new value, apply a shift described by vcom_adj.
	if (sy7636a->vcom != vcom && sy7636a->suspended != 1) {
		if (sy7636a_vcom_set_voltage(dev, vcom) != 0) {
			sy7636a->vcom = vcom;
		}
		sy7636a->vcom = vcom;
	}

	if (value)
		*value = vcom;

	return 0;
}
int sy7636a_vcom_set_voltage(struct device *dev, int value)
{
	int ret;
	unsigned int rval;
	struct sy7636a *sy7636a = dev_get_drvdata(dev);

	dev_info(sy7636a->dev, "SY7636a: vcom_set: Trying to set %d\n", value);

	if (sy7636a->vcom != value && sy7636a->suspended != 1) {
		value = value + sy7636a->vadj;
		dev_info(sy7636a->dev, "SY7636a: vcom_set: Adjusting by %d. Now setting %d\n", sy7636a->vadj, value);
	}

	if (value < SY7636A_REG_VCOM_MIN || value > SY7636A_REG_VCOM_MAX) {
		dev_info(sy7636a->dev, "SY7636a: vcom_set: Voltage error!\n");
		return -EINVAL;
    }

	sy7636a->vcom = value;

	rval = (unsigned int)((-value) / SY7636A_REG_VCOM_ADJUST_CTRL_SCAL) & SY7636A_REG_VCOM_ADJUST_CTRL_MASK;

	ret = regmap_write(sy7636a->regmap, SY7636A_REG_VCOM_ADJUST_CTRL_L, rval & 0xFF);
	if (ret)
		return ret;
	ret = regmap_write(sy7636a->regmap, SY7636A_REG_VCOM_ADJUST_CTRL_H, (rval >> 8) & 0x01);
	if (ret)
		return ret;

	return 0;
}
int sy7636a_vcom_get_pgood(struct device *dev)
{
	struct sy7636a *sy7636a = dev_get_drvdata(dev);
	int pwr_good = 0;
	unsigned long t0, t1;
	const unsigned int wait_time = 500;
	unsigned int wait_cnt;

	if (!sy7636a->pgood_gpio)
		return -EINVAL;
    
	t0 = jiffies;
	for (wait_cnt = 0; wait_cnt < wait_time; wait_cnt++) {
		pwr_good = gpiod_get_value_cansleep(sy7636a->pgood_gpio);
		if (pwr_good < 0) {
			dev_err(sy7636a->dev, "Failed to read pgood gpio: %d\n", pwr_good);
			return pwr_good;
		} else if (pwr_good) {
			break;
		}
		usleep_range(1000, 1500);
	}
	t1 = jiffies;

	if (!pwr_good) {
		dev_err(sy7636a->dev, "Power good signal timeout after %u ms\n", jiffies_to_msecs(t1 - t0));
		return -ETIME;
	}
	dev_dbg(sy7636a->dev, "Power good OK (took %u ms, %u waits)\n", jiffies_to_msecs(t1 - t0), wait_cnt);

	return 0;
}
int sy7636a_vcom_suspend(struct device *dev) {
	int ret;
	struct sy7636a *sy7636a = dev_get_drvdata(dev);

	ret = sy7636a_vcom_get_voltage(dev, NULL);
	if (ret)
		return ret;

	sy7636a->suspended = 1;

	ret = sy7636a_vcom_set_voltage(dev, 0x00); // FIXME: Get the datasheet to find out how to properly suspend the entire device.
	if (ret)
		return ret;

	return 0;
}
int sy7636a_vcom_resume(struct device *dev) {
	int ret;
	struct sy7636a *sy7636a = dev_get_drvdata(dev);
    
	ret = sy7636a_vcom_set_voltage(dev, sy7636a->vcom);
	if (ret)
		return ret;

	sy7636a->suspended = 0;

	ret = sy7636a_vcom_init(sy7636a);
	if (ret)
		return ret;

	return 0;
}
// End of VCOM functions

// Start of ATTRS
static ssize_t state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	unsigned int val;
	struct sy7636a *sy7636a = dev_get_drvdata(dev);

	ret = regmap_read(sy7636a->regmap, SY7636A_REG_FAULT_FLAG, &val);
	if (ret) {
		dev_err(sy7636a->dev, "Failed to read from device\n");
		return ret;
	}

	val = val >> 1;

	if (val >= ARRAY_SIZE(states)) {
		dev_err(sy7636a->dev, "Unexpected value read from device: %u\n", val);
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", states[val]);
}
static DEVICE_ATTR(state, S_IRUGO, state_show, NULL);

static ssize_t powergood_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	unsigned int val;
	struct sy7636a *sy7636a = dev_get_drvdata(dev);

	ret = regmap_read(sy7636a->regmap, SY7636A_REG_FAULT_FLAG, &val);
	if (ret) {
		dev_err(sy7636a->dev, "Failed to read from device\n");
		return ret;
	}

	val &= 0x01;
	val |= gpiod_get_value(sy7636a->pgood_gpio) ? 0x02 : 0x00;

	return snprintf(buf, PAGE_SIZE, "%s\n", pgood[val]);
}
static DEVICE_ATTR(power_good, S_IRUGO, powergood_show, NULL);

static ssize_t vrcom_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret, vcom;
	struct sy7636a *sy7636a = dev_get_drvdata(dev);

	ret = sy7636a_vcom_get_voltage(dev, &vcom);
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", vcom - sy7636a->vadj);
}
static ssize_t vrcom_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret, vcom;

	ret = kstrtoint(buf, 0, &vcom);
	if (ret)
		return ret;

	ret = sy7636a_vcom_set_voltage(dev, vcom);
	if (ret)
		return ret;

	return count;
}
static DEVICE_ATTR(vrcom, S_IRUGO | S_IWUSR, vrcom_show, vrcom_store);

static ssize_t vcom_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret, vcom;

	ret = sy7636a_vcom_get_voltage(dev, &vcom);
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", vcom);
}
static DEVICE_ATTR(vcom, S_IRUGO, vcom_show, NULL);

static ssize_t vadj_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sy7636a *sy7636a = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", sy7636a->vadj);
}
static ssize_t vadj_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret, vadj, vcom;
	struct sy7636a *sy7636a = dev_get_drvdata(dev);

	ret = kstrtoint(buf, 0, &vadj);
	if (ret)
		return ret;

	vcom = sy7636a->vcom;
	sy7636a->vadj = vadj;
	sy7636a->vcom = 0;

	ret = sy7636a_vcom_set_voltage(dev, vcom);
	if (ret) {
		sy7636a->vcom = vcom;
		return ret;
	}

	return count;
}
static DEVICE_ATTR(vadj, S_IRUGO | S_IWUSR, vadj_show, vadj_store);

static struct attribute *sy7636a_sysfs_attrs[] = {
	&dev_attr_state.attr,
	&dev_attr_power_good.attr,
	&dev_attr_vrcom.attr,
	&dev_attr_vcom.attr,
	&dev_attr_vadj.attr,
	NULL,
};
static const struct attribute_group sy7636a_sysfs_attr_group = {
	.attrs = sy7636a_sysfs_attrs,
};
// End of ATTRS

static int sy7636a_probe(struct i2c_client *client, const struct i2c_device_id *ids)
{
	struct sy7636a *sy7636a;
	int ret;

	sy7636a = devm_kzalloc(&client->dev, sizeof(struct sy7636a), GFP_KERNEL);
	if (sy7636a == NULL)
		return -ENOMEM;

	sy7636a->dev = &client->dev;
	sy7636a->suspended = 0;
	sy7636a->vcom = 0;
	sy7636a->vadj = 0;
    sy7636a->pgood_gpio = NULL;

	sy7636a->regmap = devm_regmap_init_i2c(client, &sy7636a_regmap_config);
	if (IS_ERR(sy7636a->regmap)) {
		ret = PTR_ERR(sy7636a->regmap);
		dev_err(sy7636a->dev, "Failed to initialize register map: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(client, sy7636a);

	ret = sysfs_create_group(&client->dev.kobj, &sy7636a_sysfs_attr_group);
	if (ret) {
		dev_err(sy7636a->dev, "Failed to create sysfs attributes\n");
		return ret;
	}

	ret = sy7636a_vcom_init(sy7636a);
	if (ret) {
		dev_err(sy7636a->dev, "Failed to initialize regulator: %d\n", ret);
		return ret;
	}

	ret = devm_mfd_add_devices(sy7636a->dev, PLATFORM_DEVID_AUTO, sy7636a_cells, ARRAY_SIZE(sy7636a_cells), NULL, 0, NULL);
	if (ret) {
		dev_err(sy7636a->dev, "Failed to add mfd devices\n");
		sysfs_remove_group(&client->dev.kobj, &sy7636a_sysfs_attr_group);
		return ret;
	}

	return 0;
}

static const struct i2c_device_id sy7636a_id_table[] = {
	{ "sy7636a", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, sy7636a_id_table);

static struct i2c_driver sy7636a_driver = {
	.driver	= {
		.name	= "sy7636a",
		.of_match_table = of_sy7636a_match_table,
	},
	.probe = sy7636a_probe,
	.id_table = sy7636a_id_table,
};
module_i2c_driver(sy7636a_driver);

MODULE_AUTHOR("Lars Ivar Miljeteig <lars.ivar.miljeteig@remarkable.com>");
MODULE_DESCRIPTION("Silergy SY7636A Multi-Function Device Driver");
MODULE_LICENSE("GPL v2");
