// SPDX-License-Identifier: GPL-2.0+
/*
 * Apple Lightning Tristar mux — NXP CBTL1609A1 (iPod nano 7G / N31)
 *
 * Public “0x34 write / 0x35 read” is 8-bit; Linux 7-bit address is 0x1a.
 * THS7383 Dx/ACCx pin tables are public (nyansatan); CBTL1609 I2C indices
 * that program them are still OPEN in public docs. RetailOS RE shows
 * **zero** Dx/mux register writes — dump is flat until accessory attaches.
 * Only apple,init-sequence from DT may write. UDC soft reconnect is done
 * from initramfs via sysfs udc soft_connect.
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/sysfs.h>

#define TRISTAR_DUMP_LEN	0x40

struct apple_tristar {
	struct i2c_client *client;
	u8 last_dump[TRISTAR_DUMP_LEN];
	u8 read_reg;
	u8 read_val;
	bool dump_ok;
	bool dump_flat;
};

static int tristar_read_reg(struct apple_tristar *ts, u8 reg, u8 *val)
{
	int ret = i2c_smbus_read_byte_data(ts->client, reg);

	if (ret < 0)
		return ret;
	*val = (u8)ret;
	return 0;
}

static int tristar_write_reg(struct apple_tristar *ts, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(ts->client, reg, val);
}

static bool tristar_dump_is_flat(const u8 *dump, size_t len)
{
	size_t i;

	for (i = 1; i < len; i++) {
		if (dump[i] != dump[0])
			return false;
	}
	return true;
}

static int tristar_dump(struct apple_tristar *ts)
{
	int i, ret;
	u8 v;

	for (i = 0; i < TRISTAR_DUMP_LEN; i++) {
		ret = tristar_read_reg(ts, i, &v);
		if (ret) {
			dev_warn(&ts->client->dev,
				 "read 0x%02x failed: %d\n", i, ret);
			ts->dump_ok = false;
			return ret;
		}
		ts->last_dump[i] = v;
	}

	ts->dump_ok = true;
	ts->dump_flat = tristar_dump_is_flat(ts->last_dump, TRISTAR_DUMP_LEN);

	dev_dbg(&ts->client->dev,
		 "CBTL1609 dump[0..0x3f] on %s:\n",
		 ts->client->adapter->name);
	dev_dbg(&ts->client->dev, "  %*ph\n", 16, ts->last_dump);
	dev_dbg(&ts->client->dev, "  %*ph\n", 16, ts->last_dump + 16);
	dev_dbg(&ts->client->dev, "  %*ph\n", 16, ts->last_dump + 32);
	dev_dbg(&ts->client->dev, "  %*ph\n", 16, ts->last_dump + 48);
	return 0;
}

static int tristar_apply_init_sequence(struct apple_tristar *ts)
{
	struct device *dev = &ts->client->dev;
	struct device_node *np = dev->of_node;
	int n, i, ret;
	u32 reg, val;

	if (!np)
		return 0;

	n = of_property_count_u32_elems(np, "apple,init-sequence");
	if (n <= 0)
		return 0;
	if (n % 2) {
		dev_err(dev, "apple,init-sequence must be reg,val pairs\n");
		return -EINVAL;
	}

	for (i = 0; i < n; i += 2) {
		of_property_read_u32_index(np, "apple,init-sequence", i, &reg);
		of_property_read_u32_index(np, "apple,init-sequence", i + 1, &val);
		ret = tristar_write_reg(ts, (u8)reg, (u8)val);
		if (ret) {
			dev_err(dev, "init write 0x%02x=0x%02x failed: %d\n",
				reg, val, ret);
			return ret;
		}
		dev_dbg(dev, "init 0x%02x <= 0x%02x\n", reg, val);
		udelay(100);
	}
	return 0;
}

/*
 * Mode heuristic from dump only — no invented mux map.
 * Flat dump → unknown; non-flat → "active" (register diversity seen).
 */
static const char *tristar_mode_name(struct apple_tristar *ts)
{
	if (!ts->dump_ok)
		return "unknown";
	if (ts->dump_flat)
		return "unknown";
	return "active";
}

static ssize_t dump_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct apple_tristar *ts = i2c_get_clientdata(to_i2c_client(dev));
	int i, n = 0;

	if (tristar_dump(ts))
		return -EIO;
	for (i = 0; i < TRISTAR_DUMP_LEN; i++)
		n += scnprintf(buf + n, PAGE_SIZE - n, "%02x%s",
			       ts->last_dump[i],
			       (i + 1) % 16 ? " " : "\n");
	return n;
}
static DEVICE_ATTR_RO(dump);

static ssize_t poke_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct apple_tristar *ts = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int reg, val;
	int ret;

	if (sscanf(buf, "%x %x", &reg, &val) != 2)
		return -EINVAL;
	if (reg > 0xff || val > 0xff)
		return -EINVAL;
	ret = tristar_write_reg(ts, reg, val);
	if (ret)
		return ret;
	dev_info(dev, "poke 0x%02x <= 0x%02x\n", reg, val);
	return count;
}
static DEVICE_ATTR_WO(poke);

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct apple_tristar *ts = i2c_get_clientdata(to_i2c_client(dev));

	if (tristar_dump(ts))
		return sysfs_emit(buf, "unknown\n");
	return sysfs_emit(buf, "%s\n", tristar_mode_name(ts));
}
static DEVICE_ATTR_RO(mode);

static ssize_t read_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct apple_tristar *ts = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int reg;
	u8 val;
	int ret;

	if (kstrtouint(buf, 0, &reg) || reg > 0xff)
		return -EINVAL;
	ret = tristar_read_reg(ts, (u8)reg, &val);
	if (ret)
		return ret;
	ts->read_reg = (u8)reg;
	ts->read_val = val;
	return count;
}

static ssize_t read_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct apple_tristar *ts = i2c_get_clientdata(to_i2c_client(dev));

	return sysfs_emit(buf, "0x%02x\n", ts->read_val);
}
static DEVICE_ATTR_RW(read);

static ssize_t value_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct apple_tristar *ts = i2c_get_clientdata(to_i2c_client(dev));

	return sysfs_emit(buf, "0x%02x (reg 0x%02x)\n",
			  ts->read_val, ts->read_reg);
}
static DEVICE_ATTR_RO(value);

static ssize_t verify_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct apple_tristar *ts = i2c_get_clientdata(to_i2c_client(dev));

	if (tristar_dump(ts))
		return sysfs_emit(buf, "FAIL read\n");
	if (ts->dump_flat)
		return sysfs_emit(buf, "FAIL flat 0x%02x\n", ts->last_dump[0]);
	return sysfs_emit(buf, "STATUS_OK non-flat\n");
}
static DEVICE_ATTR_RO(verify);

static ssize_t poll_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct apple_tristar *ts = i2c_get_clientdata(to_i2c_client(dev));
	u8 prior[TRISTAR_DUMP_LEN];
	unsigned int i, deltas = 0;
	int ret;

	memcpy(prior, ts->last_dump, sizeof(prior));
	ret = tristar_dump(ts);
	if (ret)
		return ret;

	for (i = 0; i < TRISTAR_DUMP_LEN; i++) {
		if (prior[i] != ts->last_dump[i])
			deltas++;
	}

	dev_info(dev, "Tristar poll: flat=%d deltas=%u mode=%s (mux map OPEN)\n",
		 ts->dump_flat, deltas, tristar_mode_name(ts));
	return count;
}

static ssize_t poll_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct apple_tristar *ts = i2c_get_clientdata(to_i2c_client(dev));

	return sysfs_emit(buf,
			  "flat=%d last_ok=%d — echo 1 > poll to re-dump\n",
			  ts->dump_flat, ts->dump_ok);
}
static DEVICE_ATTR_RW(poll);

static struct attribute *tristar_attrs[] = {
	&dev_attr_dump.attr,
	&dev_attr_poke.attr,
	&dev_attr_mode.attr,
	&dev_attr_read.attr,
	&dev_attr_value.attr,
	&dev_attr_verify.attr,
	&dev_attr_poll.attr,
	NULL,
};
ATTRIBUTE_GROUPS(tristar);

static int apple_tristar_probe(struct i2c_client *client)
{
	struct apple_tristar *ts;
	struct device *dev = &client->dev;
	int ret;
	u8 id0 = 0xff;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	ts->client = client;
	i2c_set_clientdata(client, ts);

	/*
	 * I2C RX still returns the address byte (DS=0x31/0xe1). Do not
	 * require a read ACK at probe — that either fails the bind or
	 * storms IIC0. U-Boot DFU already routed Lightning USB. Only
	 * apple,init-sequence may write; RetailOS has no Dx mux map.
	 */
	if (of_property_read_bool(dev->of_node, "apple,require-ack")) {
		ret = tristar_read_reg(ts, 0x00, &id0);
		if (ret) {
			dev_err(dev,
				"Tristar no ACK at 7-bit 0x%02x on %s (err=%d)\n",
				client->addr, client->adapter->name, ret);
			return -ENODEV;
		}
		dev_info(dev,
			 "Lightning Tristar ACK @7bit=0x%02x on %s reg0=0x%02x\n",
			 client->addr, client->adapter->name, id0);
	} else {
		dev_info(dev,
			 "Tristar bound @7bit=0x%02x on %s (skip ACK)\n",
			 client->addr, client->adapter->name);
	}

	/* Full 64-byte dump deferred to sysfs (poll/dump) — avoid boot I2C storm */

	/* DT-only init — never invent mux register writes in driver */
	tristar_apply_init_sequence(ts);

	ret = sysfs_create_groups(&dev->kobj, tristar_groups);
	if (ret)
		dev_warn(dev, "sysfs groups failed: %d\n", ret);

	return 0;
}

static void apple_tristar_remove(struct i2c_client *client)
{
	sysfs_remove_groups(&client->dev.kobj, tristar_groups);
}

static const struct of_device_id apple_tristar_of_match[] = {
	{ .compatible = "apple,tristar-cbtl1609" },
	{ .compatible = "nxp,cbtl1609a1" },
	{ },
};
MODULE_DEVICE_TABLE(of, apple_tristar_of_match);

static const struct i2c_device_id apple_tristar_id[] = {
	{ "tristar-cbtl1609" },
	{ },
};
MODULE_DEVICE_TABLE(i2c, apple_tristar_id);

static struct i2c_driver apple_tristar_driver = {
	.driver = {
		.name = "apple-tristar",
		.of_match_table = apple_tristar_of_match,
	},
	.probe = apple_tristar_probe,
	.remove = apple_tristar_remove,
	.id_table = apple_tristar_id,
};
module_i2c_driver(apple_tristar_driver);

MODULE_DESCRIPTION("Apple Lightning Tristar / NXP CBTL1609A1 mux");
MODULE_AUTHOR("Hydrogenuine / FreeMyiPod N31 bring-up");
MODULE_LICENSE("GPL");
