/*
 * Hynitron CST2xx (CST21680) touchscreen driver for Spreadtrum SC7731E boards
 *
 * Written for the iBall Slide Skye 03 (sp7731e_1h10), whose stock kernel has
 * a built-in "cst2xx" driver that was never published. The touch report
 * protocol follows Hynitron's hyn_cst2xx driver (as found in
 * rockchip-linux/kernel); the device tree binding is the one used by the
 * stock firmware:
 *
 *	cst2xx@1A {
 *		compatible = "cst2xx,cst2xx_ts";
 *		reg = <0x1a>;
 *		gpios = <&ap_gpio 63 0		reset
 *			 &ap_gpio 64 0>;	interrupt
 *		vdd_name = "vdd28";
 *		TP_MAX_X = <600>;
 *		TP_MAX_Y = <1024>;
 *	};
 *
 * This driver never writes the controller's firmware: the firmware update
 * paths of the Hynitron driver (boot-time update and the /proc tool
 * interface) are intentionally not included.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#define CST2XX_NAME		"cst2xx"
#define CST2XX_MAX_FINGERS	5
#define CST2XX_MAX_SLOTS	10
#define CST2XX_PRESS_MAX	255
#define CST2XX_WIDTH_MAX	200
#define CST2XX_REPORT_END	0xAB
#define CST2XX_I2C_CHUNK	6	/* Hynitron's TRANSACTION_LENGTH_LIMITED */
/*
 * The controller reports both axes on a 0..1024 grid regardless of the
 * panel; scale them to TP_MAX_X/TP_MAX_Y (measured on the Slide Skye 03:
 * full width 0..~1007, screen centre ~(520, 505)).
 */
#define CST2XX_RAW_MAX		1024

struct cst2xx_ts {
	struct i2c_client *client;
	struct input_dev *input;
	struct regulator *vdd;
	int rst_gpio;
	int irq_gpio;
	u32 max_x;
	u32 max_y;
	unsigned long active;	/* bitmap of slots currently down */
};

static int cst2xx_write(struct i2c_client *client, const u8 *buf, int len)
{
	int ret = i2c_master_send(client, buf, len);

	return ret == len ? 0 : (ret < 0 ? ret : -EIO);
}

/* Read @len bytes from the 16-bit register @reg into @buf */
static int cst2xx_read_reg(struct i2c_client *client, u16 reg, u8 *buf, int len)
{
	u8 addr[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msgs[2] = {
		{ .addr = client->addr, .flags = 0, .len = 2, .buf = addr },
		{ .addr = client->addr, .flags = I2C_M_RD, .len = len, .buf = buf },
	};
	int ret = i2c_transfer(client->adapter, msgs, 2);

	return ret == 2 ? 0 : (ret < 0 ? ret : -EIO);
}

static void cst2xx_hw_reset(struct cst2xx_ts *ts)
{
	gpio_direction_output(ts->rst_gpio, 0);
	msleep(10);
	gpio_direction_output(ts->rst_gpio, 1);
	msleep(60);
}

static int cst2xx_test_i2c(struct i2c_client *client)
{
	static const u8 cmd[2] = { 0xD0, 0x00 };
	int retry, ret = -EIO;

	for (retry = 0; retry < 5; retry++) {
		ret = cst2xx_write(client, cmd, sizeof(cmd));
		if (!ret)
			return 0;
		msleep(2);
	}
	return ret;
}

static void cst2xx_release_all(struct cst2xx_ts *ts)
{
	int slot;

	for_each_set_bit(slot, &ts->active, CST2XX_MAX_SLOTS) {
		input_mt_slot(ts->input, slot);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
	}
	ts->active = 0;
	input_mt_report_pointer_emulation(ts->input, false);
	input_sync(ts->input);
}

/*
 * Report layout (register 0xD000):
 *   bytes 0-4	first point: [id:4 | state:3 | 0], x[11:4], y[11:4],
 *		[x[3:0] | y[3:0]], width
 *   byte 5	[key flag:1 | point count:7]
 *   byte 6	0xAB when only one point is present
 * Further points (5 bytes each, plus 2 key bytes when the key flag is set)
 * follow at 0xD007 and are terminated by 0xAB. Writing D0 00 AB
 * acknowledges the report.
 */
static irqreturn_t cst2xx_irq(int irq, void *dev_id)
{
	static const u8 ack[3] = { 0xD0, 0x00, CST2XX_REPORT_END };
	struct cst2xx_ts *ts = dev_id;
	struct i2c_client *client = ts->client;
	u8 buf[64];
	unsigned long seen = 0;
	int cnt, len, idx, i, slot, ret;

	ret = cst2xx_read_reg(client, 0xD000, buf, 7);
	if (ret) {
		dev_err_ratelimited(&client->dev, "read touch data failed: %d\n", ret);
		return IRQ_HANDLED;
	}

	if (buf[6] != CST2XX_REPORT_END)
		return IRQ_HANDLED;

	cnt = buf[5] & 0x7f;
	if (cnt > CST2XX_MAX_FINGERS)
		return IRQ_HANDLED;

	if (cnt > 1) {
		len = (cnt - 1) * 5 + ((buf[5] & 0x80) ? 3 : 1);
		for (idx = 0; idx < len; idx += CST2XX_I2C_CHUNK) {
			int n = min(len - idx, CST2XX_I2C_CHUNK);

			ret = cst2xx_read_reg(client, 0xD007 + idx, &buf[5 + idx], n);
			if (ret)
				goto ack;
		}
		if (buf[5 + len - 1] != CST2XX_REPORT_END)
			goto ack;
	}

	for (i = 0, idx = 0; i < cnt; i++, idx += 5) {
		int id = (buf[idx] >> 4) & 0x0f;
		int state = (buf[idx] & 0x0f) >> 1;
		int x = (buf[idx + 1] << 4) | ((buf[idx + 3] >> 4) & 0x0f);
		int y = (buf[idx + 2] << 4) | (buf[idx + 3] & 0x0f);
		int w = buf[idx + 4];

		x = x * ts->max_x / CST2XX_RAW_MAX;
		y = y * ts->max_y / CST2XX_RAW_MAX;

		if (id >= CST2XX_MAX_SLOTS || state != 0x03)
			continue;

		input_mt_slot(ts->input, id);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		input_report_abs(ts->input, ABS_MT_POSITION_X, min_t(int, x, ts->max_x));
		input_report_abs(ts->input, ABS_MT_POSITION_Y, min_t(int, y, ts->max_y));
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, w);
		input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR, w);
		__set_bit(id, &seen);
	}

	/* lift every finger that is no longer reported as down */
	for_each_set_bit(slot, &ts->active, CST2XX_MAX_SLOTS) {
		if (test_bit(slot, &seen))
			continue;
		input_mt_slot(ts->input, slot);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
	}
	ts->active = seen;
	input_mt_report_pointer_emulation(ts->input, false);
	input_sync(ts->input);

ack:
	if (cst2xx_write(client, ack, sizeof(ack)))
		dev_err_ratelimited(&client->dev, "ack touch report failed\n");
	return IRQ_HANDLED;
}

static int cst2xx_parse_dt(struct cst2xx_ts *ts)
{
	struct device *dev = &ts->client->dev;
	struct device_node *np = dev->of_node;
	const char *vdd_name = NULL;

	ts->rst_gpio = of_get_gpio(np, 0);
	ts->irq_gpio = of_get_gpio(np, 1);
	if (!gpio_is_valid(ts->rst_gpio) || !gpio_is_valid(ts->irq_gpio)) {
		dev_err(dev, "invalid reset/irq gpios\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "TP_MAX_X", &ts->max_x))
		ts->max_x = 600;
	if (of_property_read_u32(np, "TP_MAX_Y", &ts->max_y))
		ts->max_y = 1024;

	if (!of_property_read_string(np, "vdd_name", &vdd_name)) {
		ts->vdd = regulator_get(NULL, vdd_name);
		if (IS_ERR(ts->vdd)) {
			dev_warn(dev, "no regulator %s: %ld\n", vdd_name, PTR_ERR(ts->vdd));
			ts->vdd = NULL;
		}
	}
	return 0;
}

static int cst2xx_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct cst2xx_ts *ts;
	struct input_dev *input;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	ts->client = client;
	i2c_set_clientdata(client, ts);

	ret = cst2xx_parse_dt(ts);
	if (ret)
		return ret;

	ret = devm_gpio_request(dev, ts->rst_gpio, "cst2xx reset");
	if (ret)
		goto err_put_vdd;
	ret = devm_gpio_request_one(dev, ts->irq_gpio, GPIOF_IN, "cst2xx irq");
	if (ret)
		goto err_put_vdd;

	if (ts->vdd) {
		ret = regulator_enable(ts->vdd);
		if (ret) {
			dev_err(dev, "enable vdd failed: %d\n", ret);
			goto err_put_vdd;
		}
	}
	cst2xx_hw_reset(ts);

	ret = cst2xx_test_i2c(client);
	if (ret) {
		dev_info(dev, "no cst2xx controller at 0x%02x (%d)\n", client->addr, ret);
		ret = -ENODEV;
		goto err_vdd_off;
	}

	input = devm_input_allocate_device(dev);
	if (!input) {
		ret = -ENOMEM;
		goto err_vdd_off;
	}
	ts->input = input;
	input->name = CST2XX_NAME;
	input->phys = "input/ts";
	input->id.bustype = BUS_I2C;

	__set_bit(INPUT_PROP_DIRECT, input->propbit);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, ts->max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, ts->max_y, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, CST2XX_PRESS_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, CST2XX_WIDTH_MAX, 0, 0);
	ret = input_mt_init_slots(input, CST2XX_MAX_SLOTS, INPUT_MT_DIRECT);
	if (ret)
		goto err_vdd_off;

	ret = input_register_device(input);
	if (ret)
		goto err_vdd_off;

	client->irq = gpio_to_irq(ts->irq_gpio);
	ret = devm_request_threaded_irq(dev, client->irq, NULL, cst2xx_irq,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"cst2xx_ts", ts);
	if (ret) {
		dev_err(dev, "request irq %d failed: %d\n", client->irq, ret);
		goto err_vdd_off;
	}

	dev_info(dev, "Hynitron cst2xx touchscreen, %ux%u, irq %d\n",
		 ts->max_x, ts->max_y, client->irq);
	return 0;

err_vdd_off:
	if (ts->vdd)
		regulator_disable(ts->vdd);
err_put_vdd:
	if (ts->vdd)
		regulator_put(ts->vdd);
	return ret;
}

static int cst2xx_remove(struct i2c_client *client)
{
	struct cst2xx_ts *ts = i2c_get_clientdata(client);

	devm_free_irq(&client->dev, client->irq, ts);
	cst2xx_release_all(ts);
	if (ts->vdd) {
		regulator_disable(ts->vdd);
		regulator_put(ts->vdd);
	}
	return 0;
}

static const struct i2c_device_id cst2xx_id[] = {
	{ CST2XX_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cst2xx_id);

static const struct of_device_id cst2xx_of_match[] = {
	{ .compatible = "cst2xx,cst2xx_ts" },
	{ }
};
MODULE_DEVICE_TABLE(of, cst2xx_of_match);

static struct i2c_driver cst2xx_driver = {
	.driver = {
		.name = CST2XX_NAME,
		.of_match_table = cst2xx_of_match,
	},
	.probe = cst2xx_probe,
	.remove = cst2xx_remove,
	.id_table = cst2xx_id,
};
module_i2c_driver(cst2xx_driver);

MODULE_DESCRIPTION("Hynitron CST2xx touchscreen driver for Spreadtrum boards");
MODULE_LICENSE("GPL v2");
