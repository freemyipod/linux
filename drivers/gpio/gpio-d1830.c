// SPDX-License-Identifier: GPL-2.0-only
/*
 * GPIO Driver for Dialog Semiconductor D1830 PMIC
 *
 * This driver exposes specific bits of PMIC registers as GPIO lines.
 * It is read-only and intended for button polling via gpio-keys-polled.
 *
 * Copyright (C) 2026 Vencislav Atanasov <user890104@freemyipod.org>
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio/driver.h>
#include <linux/of.h>

struct d1830_gpio_map {
	u8 reg;
	u8 bit;
};

struct d1830_gpio {
	struct i2c_client *client;
	struct gpio_chip gpio_chip;
	struct d1830_gpio_map *map;
	int num_gpios;
};

static int d1830_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	return GPIO_LINE_DIRECTION_IN;
}

static int d1830_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct d1830_gpio *gpio_dev = gpiochip_get_data(chip);
	struct d1830_gpio_map *entry;
	int ret;

	if (offset >= gpio_dev->num_gpios) {
		return -EINVAL;
	}

	entry = &gpio_dev->map[offset];

	ret = i2c_smbus_read_byte_data(gpio_dev->client, entry->reg);
	if (ret < 0) {
		dev_err(&gpio_dev->client->dev,
			"Failed to read reg 0x%02x: %d\n", entry->reg, ret);
		return 0;
	}

	return !!(ret & BIT(entry->bit));
}

static int d1830_gpio_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	return 0;
}

static int d1830_gpio_direction_output(struct gpio_chip *chip, unsigned int offset, int value)
{
	return -ENOTSUPP;
}

static void d1830_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	// read-only
}

static int d1830_gpio_parse_dt(struct d1830_gpio *gpio_dev)
{
	struct device *dev = &gpio_dev->client->dev;
	struct device_node *np = dev->of_node;
	int size, i;

	if (!np)
		return -ENODEV;

	size = of_property_count_u32_elems(np, "dlg,gpio-map");
	if (size <= 0 || size % 2) {
		dev_err(dev, "Invalid or missing 'dlg,gpio-map' property "
			"(size=%d)\n", size);
		return -EINVAL;
	}

	gpio_dev->num_gpios = size / 2;

	gpio_dev->map = devm_kcalloc(dev, gpio_dev->num_gpios,
				  sizeof(*gpio_dev->map), GFP_KERNEL);
	if (!gpio_dev->map)
		return -ENOMEM;

	for (i = 0; i < gpio_dev->num_gpios; i++) {
		u32 reg, bit;

		of_property_read_u32_index(np, "dlg,gpio-map",
					   i * 2, &reg);
		of_property_read_u32_index(np, "dlg,gpio-map",
					   i * 2 + 1, &bit);

		if (reg > 0xff || bit > 7) {
			dev_err(dev, "GPIO %d: reg=0x%x bit=%u out of range\n",
				i, reg, bit);
			return -EINVAL;
		}

		gpio_dev->map[i].reg = (u8)reg;
		gpio_dev->map[i].bit = (u8)bit;

		dev_dbg(dev, "GPIO %d -> reg 0x%02x bit %u\n", i, reg, bit);
	}

	return 0;
}

static int d1830_gpio_probe(struct i2c_client *client)
{
	struct d1830_gpio *gpio_dev;
	struct device *dev = &client->dev;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "Adapter does not support SMBUS_BYTE_DATA\n");
		return -ENODEV;
	}

	gpio_dev = devm_kzalloc(dev, sizeof(*gpio_dev), GFP_KERNEL);
	if (!gpio_dev)
		return -ENOMEM;

	gpio_dev->client = client;
	i2c_set_clientdata(client, gpio_dev);

	ret = d1830_gpio_parse_dt(gpio_dev);
	if (ret)
		return ret;

	gpio_dev->gpio_chip.label = dev_name(dev);
	gpio_dev->gpio_chip.parent = dev;
	gpio_dev->gpio_chip.owner = THIS_MODULE;
	gpio_dev->gpio_chip.base = -1;
	gpio_dev->gpio_chip.ngpio = gpio_dev->num_gpios;
	gpio_dev->gpio_chip.can_sleep = true;
	gpio_dev->gpio_chip.get_direction = d1830_gpio_get_direction;
	gpio_dev->gpio_chip.direction_input = d1830_gpio_direction_input;
	gpio_dev->gpio_chip.direction_output = d1830_gpio_direction_output;
	gpio_dev->gpio_chip.get = d1830_gpio_get;
	gpio_dev->gpio_chip.set = d1830_gpio_set;

	ret = devm_gpiochip_add_data(dev, &gpio_dev->gpio_chip, gpio_dev);
	if (ret) {
		dev_err(dev, "Failed to add GPIO chip: %d\n", ret);
		return ret;
	}

	dev_info(dev, "Registered %u read-only GPIOs using Dialog D1830 driver\n",
		gpio_dev->num_gpios);
	return 0;
}

static const struct of_device_id d1830_gpio_of_match[] = {
	{ .compatible = "dlg,d1830-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, d1830_gpio_of_match);

static const struct i2c_device_id d1830_gpio_id[] = {
	{ "d1830-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, d1830_gpio_id);

static struct i2c_driver d1830_gpio_driver = {
	.driver = {
		.name = "gpio-d1830",
		.of_match_table = d1830_gpio_of_match,
	},
	.probe = d1830_gpio_probe,
	.id_table = d1830_gpio_id,
};

module_i2c_driver(d1830_gpio_driver);

MODULE_AUTHOR("Vencislav Atanasov <user890104@freemyipod.org>");
MODULE_DESCRIPTION("Dialog Semiconductor D1830 PMIC read-only GPIO driver");
MODULE_LICENSE("GPL v2");
