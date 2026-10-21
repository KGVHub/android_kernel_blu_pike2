/*
 * Silan SC7A20 3-axis accelerometer driver for Spreadtrum SC7731E boards
 *
 * Written for the iBall Slide Skye 03 (sp7731e_1h10), whose stock kernel
 * has an unpublished "sc7a20_acc" driver. The SC7A20 is register
 * compatible with ST's LIS3DH; this driver programs it in high-resolution
 * +-2g mode (1 mg/digit) and polls it.
 *
 * It provides exactly what the stock sensors HAL (sensors.sp7731e.so) uses:
 *  - a virtual input device named "accelerometer" (no parent, so it shows up
 *    as /sys/devices/virtual/input/inputN, where ueventd.rc grants the HAL
 *    access to "enable" and "delay")
 *  - "enable" (0/1) and "delay" (ms) attributes on that input device
 *  - ABS_X/ABS_Y/ABS_Z in units of 1/2^20 g (the HAL multiplies by
 *    9.80665 / 2^20 to get m/s^2)
 *
 * Device tree (stock binding):
 *	sc7a20_acc@18 {
 *		compatible = "SC,sc7a20_acc";
 *		reg = <0x18>;
 *		poll_interval = <10>;
 *		min_interval = <4>;
 *		g_range = <0>;
 *		axis_map_x = <1>; axis_map_y = <0>; axis_map_z = <2>;
 *		negate_x = <0>; negate_y = <1>; negate_z = <0>;
 *	};
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define SC7A20_NAME		"accelerometer"

#define SC7A20_WHO_AM_I		0x0f
#define SC7A20_CHIP_ID		0x11
#define LIS3DH_CHIP_ID		0x33
#define SC7A20_CTRL_REG1	0x20	/* ODR[7:4] LPen Zen Yen Xen */
#define SC7A20_CTRL_REG4	0x23	/* BDU BLE FS[5:4] HR ST SIM */
#define SC7A20_OUT_X_L		0x28
#define SC7A20_AUTO_INC		0x80

#define SC7A20_AXES_ENABLE	0x07
#define SC7A20_CTRL4_2G_HR	0x88	/* block data update, +-2g, high resolution */

/* the HAL expects 2^20 counts per g; high-resolution +-2g gives 1 mg/digit */
#define SC7A20_COUNTS_PER_G	(1 << 20)
#define SC7A20_MG_PER_G		1000
#define SC7A20_RANGE_G		2

/*
 * The stock device tree's axis_map/negate values follow the LIS3DH axis
 * convention, but the SC7A20 reports Y and Z with the opposite sign. Applying
 * the stock DT values as-is gave, on the Slide Skye 03, z = -9.8 lying face up
 * and y = -9.8 held upright (Android expects +9.8 for both), with x correct.
 * Flip Y and Z after the DT mapping, like the stock driver must have done.
 */
static const bool sc7a20_flip[3] = { false, true, true };

struct sc7a20_odr {
	unsigned int max_ms;	/* use this ODR for polling delays >= max_ms */
	u8 bits;
};

/* ODR codes for CTRL_REG1[7:4], fastest first */
static const struct sc7a20_odr sc7a20_odr_table[] = {
	{ 3, 0x70 },	/* 400 Hz */
	{ 5, 0x60 },	/* 200 Hz */
	{ 10, 0x50 },	/* 100 Hz */
	{ 20, 0x40 },	/* 50 Hz */
	{ 40, 0x30 },	/* 25 Hz */
	{ 100, 0x20 },	/* 10 Hz */
	{ 1000, 0x10 },	/* 1 Hz */
};

struct sc7a20 {
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;
	struct mutex lock;	/* protects enabled, delay_ms and the chip */
	bool enabled;
	unsigned int delay_ms;
	unsigned int min_delay_ms;
	u32 axis_map[3];
	u32 negate[3];
};

static int sc7a20_write(struct sc7a20 *acc, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(acc->client, reg, val);
}

static u8 sc7a20_odr_bits(unsigned int delay_ms)
{
	int i;

	for (i = ARRAY_SIZE(sc7a20_odr_table) - 1; i > 0; i--)
		if (delay_ms >= sc7a20_odr_table[i].max_ms)
			return sc7a20_odr_table[i].bits;
	return sc7a20_odr_table[0].bits;
}

static int sc7a20_power(struct sc7a20 *acc, bool on)
{
	u8 reg1 = on ? sc7a20_odr_bits(acc->delay_ms) | SC7A20_AXES_ENABLE : 0;

	return sc7a20_write(acc, SC7A20_CTRL_REG1, reg1);
}

static void sc7a20_work(struct work_struct *work)
{
	struct sc7a20 *acc = container_of(work, struct sc7a20, work.work);
	u8 buf[6];
	int raw[3], i, ret;

	ret = i2c_smbus_read_i2c_block_data(acc->client,
					    SC7A20_OUT_X_L | SC7A20_AUTO_INC,
					    sizeof(buf), buf);
	if (ret == sizeof(buf)) {
		/* 12-bit left-justified samples, 1 mg/digit */
		for (i = 0; i < 3; i++)
			raw[i] = (s16)(buf[2 * i] | (buf[2 * i + 1] << 8)) >> 4;

		for (i = 0; i < 3; i++) {
			s64 v = (s64)raw[acc->axis_map[i]] * SC7A20_COUNTS_PER_G;

			v = div_s64(v, SC7A20_MG_PER_G);
			if (acc->negate[i] != sc7a20_flip[i])
				v = -v;
			input_report_abs(acc->input, ABS_X + i, (int)v);
		}
		input_sync(acc->input);
	} else {
		dev_err_ratelimited(&acc->client->dev, "read data failed: %d\n", ret);
	}

	mutex_lock(&acc->lock);
	if (acc->enabled)
		schedule_delayed_work(&acc->work, msecs_to_jiffies(acc->delay_ms));
	mutex_unlock(&acc->lock);
}

static int sc7a20_set_enable(struct sc7a20 *acc, bool enable)
{
	int ret = 0;

	mutex_lock(&acc->lock);
	if (enable != acc->enabled) {
		ret = sc7a20_power(acc, enable);
		if (!ret) {
			acc->enabled = enable;
			if (enable)
				schedule_delayed_work(&acc->work,
						      msecs_to_jiffies(acc->delay_ms));
		}
	}
	mutex_unlock(&acc->lock);

	if (!ret && !enable)
		cancel_delayed_work_sync(&acc->work);
	return ret;
}

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct sc7a20 *acc = input_get_drvdata(to_input_dev(dev));

	return sprintf(buf, "%d\n", acc->enabled);
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct sc7a20 *acc = input_get_drvdata(to_input_dev(dev));
	unsigned long val;
	int ret;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	ret = sc7a20_set_enable(acc, !!val);
	return ret ? ret : count;
}

static ssize_t delay_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct sc7a20 *acc = input_get_drvdata(to_input_dev(dev));

	return sprintf(buf, "%u\n", acc->delay_ms);
}

static ssize_t delay_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct sc7a20 *acc = input_get_drvdata(to_input_dev(dev));
	unsigned long val;
	int ret = 0;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&acc->lock);
	acc->delay_ms = clamp_t(unsigned long, val, acc->min_delay_ms, 1000);
	if (acc->enabled) {
		ret = sc7a20_power(acc, true);	/* pick a matching ODR */
		mod_delayed_work(system_wq, &acc->work,
				 msecs_to_jiffies(acc->delay_ms));
	}
	mutex_unlock(&acc->lock);
	return ret ? ret : count;
}

static DEVICE_ATTR(enable, 0664, enable_show, enable_store);
static DEVICE_ATTR(delay, 0664, delay_show, delay_store);

static struct attribute *sc7a20_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	NULL
};

static const struct attribute_group sc7a20_attr_group = {
	.attrs = sc7a20_attrs,
};

static void sc7a20_parse_dt(struct sc7a20 *acc)
{
	struct device_node *np = acc->client->dev.of_node;
	static const char * const map_props[3] = {
		"axis_map_x", "axis_map_y", "axis_map_z" };
	static const char * const neg_props[3] = {
		"negate_x", "negate_y", "negate_z" };
	u32 poll = 10, min = 4;
	int i;

	of_property_read_u32(np, "poll_interval", &poll);
	of_property_read_u32(np, "min_interval", &min);
	acc->min_delay_ms = max_t(u32, min, 1);
	acc->delay_ms = max_t(u32, poll, acc->min_delay_ms);

	for (i = 0; i < 3; i++) {
		acc->axis_map[i] = i;
		acc->negate[i] = 0;
		of_property_read_u32(np, map_props[i], &acc->axis_map[i]);
		of_property_read_u32(np, neg_props[i], &acc->negate[i]);
		if (acc->axis_map[i] > 2) {
			dev_warn(&acc->client->dev, "invalid %s, using %d\n",
				 map_props[i], i);
			acc->axis_map[i] = i;
		}
	}
}

static int sc7a20_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct sc7a20 *acc;
	struct input_dev *input;
	int range = SC7A20_RANGE_G * SC7A20_COUNTS_PER_G;
	int ret, i;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_I2C_BLOCK))
		return -ENODEV;

	ret = i2c_smbus_read_byte_data(client, SC7A20_WHO_AM_I);
	if (ret < 0) {
		dev_info(dev, "no accelerometer at 0x%02x (%d)\n", client->addr, ret);
		return -ENODEV;
	}
	if (ret != SC7A20_CHIP_ID && ret != LIS3DH_CHIP_ID) {
		dev_err(dev, "unexpected WHO_AM_I 0x%02x\n", ret);
		return -ENODEV;
	}
	dev_info(dev, "chip id 0x%02x\n", ret);

	acc = devm_kzalloc(dev, sizeof(*acc), GFP_KERNEL);
	if (!acc)
		return -ENOMEM;
	acc->client = client;
	mutex_init(&acc->lock);
	INIT_DELAYED_WORK(&acc->work, sc7a20_work);
	i2c_set_clientdata(client, acc);
	sc7a20_parse_dt(acc);

	ret = sc7a20_write(acc, SC7A20_CTRL_REG4, SC7A20_CTRL4_2G_HR);
	if (!ret)
		ret = sc7a20_power(acc, false);
	if (ret) {
		dev_err(dev, "chip init failed: %d\n", ret);
		return ret;
	}

	/*
	 * No parent: the HAL looks for /sys/devices/virtual/input/inputN.
	 * (devm_input_allocate_device() would set the parent, so manage it.)
	 */
	input = input_allocate_device();
	if (!input)
		return -ENOMEM;
	input->name = SC7A20_NAME;
	input->id.bustype = BUS_I2C;
	__set_bit(EV_ABS, input->evbit);
	for (i = 0; i < 3; i++)
		input_set_abs_params(input, ABS_X + i, -range, range, 0, 0);
	input_set_drvdata(input, acc);
	acc->input = input;

	ret = input_register_device(input);
	if (ret) {
		input_free_device(input);
		return ret;
	}

	ret = sysfs_create_group(&input->dev.kobj, &sc7a20_attr_group);
	if (ret) {
		input_unregister_device(input);
		return ret;
	}

	dev_info(dev, "SC7A20 accelerometer, poll %u ms\n", acc->delay_ms);
	return 0;
}

static int sc7a20_remove(struct i2c_client *client)
{
	struct sc7a20 *acc = i2c_get_clientdata(client);

	sc7a20_set_enable(acc, false);
	sysfs_remove_group(&acc->input->dev.kobj, &sc7a20_attr_group);
	input_unregister_device(acc->input);
	return 0;
}

static int __maybe_unused sc7a20_suspend(struct device *dev)
{
	struct sc7a20 *acc = i2c_get_clientdata(to_i2c_client(dev));

	cancel_delayed_work_sync(&acc->work);
	return sc7a20_power(acc, false);
}

static int __maybe_unused sc7a20_resume(struct device *dev)
{
	struct sc7a20 *acc = i2c_get_clientdata(to_i2c_client(dev));
	int ret = 0;

	mutex_lock(&acc->lock);
	if (acc->enabled) {
		ret = sc7a20_power(acc, true);
		schedule_delayed_work(&acc->work, msecs_to_jiffies(acc->delay_ms));
	}
	mutex_unlock(&acc->lock);
	return ret;
}

static SIMPLE_DEV_PM_OPS(sc7a20_pm_ops, sc7a20_suspend, sc7a20_resume);

static const struct i2c_device_id sc7a20_id[] = {
	{ "sc7a20_acc", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sc7a20_id);

static const struct of_device_id sc7a20_of_match[] = {
	{ .compatible = "SC,sc7a20_acc" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc7a20_of_match);

static struct i2c_driver sc7a20_driver = {
	.driver = {
		.name = "sc7a20_acc",
		.of_match_table = sc7a20_of_match,
		.pm = &sc7a20_pm_ops,
	},
	.probe = sc7a20_probe,
	.remove = sc7a20_remove,
	.id_table = sc7a20_id,
};
module_i2c_driver(sc7a20_driver);

MODULE_DESCRIPTION("Silan SC7A20 accelerometer driver for Spreadtrum boards");
MODULE_LICENSE("GPL v2");
