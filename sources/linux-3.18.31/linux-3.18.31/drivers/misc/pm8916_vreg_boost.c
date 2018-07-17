/*
 * Copyright © 2017 Parrot S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/printk.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/spmi.h>
#include <linux/qpnp/qpnp-revid.h>

#define PMIC_PERIPHERAL_TYPE		0x51

#define BOOST_VOLTAGE_5V			0x14
/* current values in mV */
#define BOOST_MAX_VOLTAGE			5550
#define BOOST_MIN_VOLTAGE			4000

#define BOOST_CURRENT_500MA			0x80
/* current values in mA */
#define BOOST_MAX_CURRENT			4000
#define BOOST_MIN_CURRENT			500

#define TOMBAK_CORE_0_SPMI_ADDR			0xf000

#define PMIC_A_DIGITAL_REVISION1		(0x000)
#define PMIC_A_DIGITAL_CDC_RST_CTL		(0x046)
#define PMIC_A_DIGITAL_CDC_TOP_CLK_CTL		(0x048)
#define PMIC_A_DIGITAL_CDC_DIG_CLK_CTL		(0x04a)
#define PMIC_A_ANALOG_MASTER_BIAS_CTL		(0x146)
#define PMIC_A_ANALOG_SPKR_DRV_CTL		(0x1b2)
#define PMIC_A_ANALOG_CURRENT_LIMIT		(0x1c0)
#define PMIC_A_ANALOG_OUTPUT_VOLTAGE		(0x1c1)
#define PMIC_A_ANALOG_BYPASS_MODE		(0x1c2)
#define PMIC_A_ANALOG_BOOST_EN_CTL		(0x1c3)

#define PMIC_A_BOOST_FREQ_CLK_ENABLE		(0x246)
#define PMIC_A_NCP_FREQ_CLK_ENABLE		(0x346)

#define PMIC_SLAVE_ID_0		0
#define PMIC_SLAVE_ID_1		1

#define PMIC_MBG_OK		0x2C08
#define PMIC_LDO7_EN_CTL	0x4646
#define MASK_MSB_BIT		0x80

struct pm8916_vreg_boost {
	struct device *dev;
	struct spmi_device *spmi;
	int base;
	bool enable;
	bool fs;
	uint8_t vb_voltage;
	uint8_t vb_current;
	spinlock_t splock;
};

static int pmic_spmi_write_device(struct pm8916_vreg_boost *pmvb,
			uint16_t reg, uint8_t *value, uint32_t bytes)
{

	int ret;
	struct spmi_device *spmi = NULL;

	if (pmvb == NULL)
		return -EINVAL;

	spmi = pmvb->spmi;

	ret = spmi_ext_register_writel(spmi->ctrl, spmi->sid,
					pmvb->base + reg, value, bytes);
	if (ret)
		dev_err_ratelimited(&spmi->dev,
				"Unable to write to addr=%x, ret(%d)\n",
				pmvb->base + reg, ret);

	/* Try again if the write fails */
	if (ret != 0) {
		usleep_range(10, 11);
		ret = spmi_ext_register_writel(spmi->ctrl, spmi->sid,
						pmvb->base + reg, value, 1);
		if (ret != 0) {
			dev_err_ratelimited(&spmi->dev,
				"failed to write the device\n");
			return ret;
		}
	}

	dev_dbg(&spmi->dev, "%s: reg 0x%x = 0x%x\n", __func__,
					pmvb->base + reg, *value);
	return 0;
}

static int pmic_spmi_read_device(struct pm8916_vreg_boost *pmvb,
				uint16_t reg, uint8_t *dest, uint32_t bytes)
{
	int ret;
	struct spmi_device *spmi = NULL;

	if (pmvb == NULL)
		return -EINVAL;

	spmi = pmvb->spmi;

	ret = spmi_ext_register_readl(spmi->ctrl, spmi->sid,
					pmvb->base + reg, dest, bytes);
	if (ret != 0) {
		dev_err(&spmi->dev, "failed to read the device\n");
		return ret;
	}
	dev_dbg(&spmi->dev, "%s: reg 0x%x = 0x%x\n", __func__,
					pmvb->base + reg, *dest);
	return ret;
}

static int configure_vreg_boost(struct pm8916_vreg_boost *pmvb)
{
	int ret = -EIO;
	uint8_t val = 0;

	if ((pmvb == NULL) || (pmvb->spmi == NULL))
		goto configure_failed;

	ret = spmi_ext_register_readl(pmvb->spmi->ctrl, PMIC_SLAVE_ID_1,
					PMIC_LDO7_EN_CTL, &val, 1);
	if (ret)
		goto configure_failed;

	dev_err(&pmvb->spmi->dev,
		"%s: LDO state: 0x%x\n", __func__, val);

	if ((val & MASK_MSB_BIT) == 0) {
		ret = -EAGAIN;

		dev_err(&pmvb->spmi->dev, "LDO7 not enabled return!\n");
		goto configure_failed;
	}
	ret = spmi_ext_register_readl(pmvb->spmi->ctrl, PMIC_SLAVE_ID_0,
						PMIC_MBG_OK, &val, 1);
	if (ret)
		goto configure_failed;

	dev_err(&pmvb->spmi->dev, "%s: PMIC BG state: 0x%x\n", __func__, val);

	if ((val & MASK_MSB_BIT) == 0) {
		dev_err(&pmvb->spmi->dev,
			"PMIC MBG not ON, enable codec hw_en MB bit again\n");

		val = 0x30;
		ret = pmic_spmi_write_device(pmvb,
			PMIC_A_ANALOG_MASTER_BIAS_CTL, &val, 1);

		/* Allow 1ms for PMIC MBG state to be updated */
		if (pmvb->fs)
			mdelay(1);
		else
			usleep_range(1000, 1100);

		ret = spmi_ext_register_readl(pmvb->spmi->ctrl, PMIC_SLAVE_ID_0,
						PMIC_MBG_OK, &val, 1);
		if (ret) {
			dev_err(&pmvb->spmi->dev,
				"%s: failed to read the device:%d\n",
				__func__, ret);

			ret = -EIO;
			goto configure_failed;
		}

		if ((val & MASK_MSB_BIT) == 0) {
			dev_err(&pmvb->spmi->dev,
				"PMIC MBG still not ON after retry return!\n");

			ret = -EAGAIN;
			goto configure_failed;
		}
	}

	dev_info(&pmvb->spmi->dev, "Setting Analog bias\n");
	val = 0x30;

	ret = pmic_spmi_write_device(pmvb,
			PMIC_A_ANALOG_MASTER_BIAS_CTL, &val, 1);
	if (ret)
		goto configure_failed;

	/* set master clock */
	val = 0x4;
	ret = pmic_spmi_write_device(pmvb,
		PMIC_A_DIGITAL_CDC_TOP_CLK_CTL, &val, 1);

	if (ret)
		goto configure_failed;

	/* set voltage */
	ret = pmic_spmi_write_device(pmvb,
			PMIC_A_ANALOG_OUTPUT_VOLTAGE, &pmvb->vb_voltage, 1);
	if (ret)
		goto configure_failed;

	/* set current */
	val = pmvb->vb_current | 0x80;
	ret = pmic_spmi_write_device(pmvb,
		PMIC_A_ANALOG_CURRENT_LIMIT, &val, 1);
	if (ret)
		goto configure_failed;

	/* BOOST_SET */
	ret = pmic_spmi_read_device(pmvb,
		PMIC_A_ANALOG_SPKR_DRV_CTL, &val, 1);

	if (ret)
		goto configure_failed;

	val = val | 0x4;
	ret = pmic_spmi_write_device(pmvb,
		PMIC_A_ANALOG_SPKR_DRV_CTL, &val, 1);
	if (ret)
		goto configure_failed;

	/* DIG_SW_RST_N */
	ret = pmic_spmi_read_device(pmvb,
		PMIC_A_DIGITAL_CDC_RST_CTL, &val, 1);

	if (ret)
		goto configure_failed;

	val = val | 0x80;
	ret = pmic_spmi_write_device(pmvb,
		PMIC_A_DIGITAL_CDC_RST_CTL, &val, 1);
	if (ret)
		goto configure_failed;

configure_failed:
	return ret;
}

static int enable_vreg_boost(struct pm8916_vreg_boost *pmvb, bool enable)
{
	int ret = -EINVAL;
	uint8_t val = 0;

	if ((pmvb == NULL) || (pmvb->spmi == NULL))
		goto enable_failed;

	if (enable) {
		/* enable clock (BOOST_CLK_EN) */
		ret = pmic_spmi_read_device(pmvb,
			PMIC_A_DIGITAL_CDC_DIG_CLK_CTL, &val, 1);

		if (ret)
			goto enable_failed;

		val = val | 0x20;
		ret = pmic_spmi_write_device(pmvb,
			PMIC_A_DIGITAL_CDC_DIG_CLK_CTL, &val, 1);

		if (ret)
			goto enable_failed;

		if (pmvb->fs)
			mdelay(1);
		else
			usleep_range(1000, 1100);

		/* enable vreg boost */
		ret = pmic_spmi_read_device(pmvb,
			PMIC_A_ANALOG_BOOST_EN_CTL, &val, 1);

		if (ret)
			goto enable_failed;

		val = val | 0x80;
		ret = pmic_spmi_write_device(pmvb,
			PMIC_A_ANALOG_BOOST_EN_CTL, &val, 1);
	} else {
		/* disable vreg boost */
		ret = pmic_spmi_read_device(pmvb,
			PMIC_A_ANALOG_BOOST_EN_CTL, &val, 1);

		if (ret)
			goto enable_failed;

		val = val & ~0x80;
		ret = pmic_spmi_write_device(pmvb,
			PMIC_A_ANALOG_BOOST_EN_CTL, &val, 1);

		if (ret)
			goto enable_failed;

		/* disable clock (BOOST_CLK_EN) */
		ret = pmic_spmi_read_device(pmvb,
			PMIC_A_DIGITAL_CDC_DIG_CLK_CTL, &val, 1);

		if (ret)
			goto enable_failed;

		val = val & ~0x20;
		ret = pmic_spmi_write_device(pmvb,
			PMIC_A_DIGITAL_CDC_DIG_CLK_CTL, &val, 1);

		/* disable master clock */
		val = 0x0;
		ret = pmic_spmi_write_device(pmvb,
			PMIC_A_DIGITAL_CDC_TOP_CLK_CTL, &val, 1);

		if (ret)
			goto enable_failed;
	}

enable_failed:
	return ret;
}

static ssize_t show_enable(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;
	struct pm8916_vreg_boost *pmvb = dev_get_drvdata(dev);

	spin_lock(&pmvb->splock);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", pmvb->enable);
	spin_unlock(&pmvb->splock);
	return ret;
}

static ssize_t set_enable(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int ret;
	bool enable = false;
	struct pm8916_vreg_boost *pmvb = dev_get_drvdata(dev);

	ret = strtobool(buf, &enable);
	if (ret)
		return ret;

	dev_dbg(&pmvb->spmi->dev,
		"%sabling vreg_boost\n", enable ? "en":"dis");

	spin_lock(&pmvb->splock);
	pmvb->fs = 1;
	ret = enable_vreg_boost(pmvb, enable);
	if (ret) {
		spin_unlock(&pmvb->splock);
		return ret;
	}

	pmvb->fs = 0;
	if (enable)
		mdelay(1);

	pmvb->enable = enable;
	spin_unlock(&pmvb->splock);

	return count;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR,
		show_enable, set_enable);

static ssize_t show_vreg_boost_voltage(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct pm8916_vreg_boost *pmvb = dev_get_drvdata(dev);
	uint32_t vb_voltage;

	spin_lock(&pmvb->splock);
	vb_voltage = (pmvb->vb_voltage * 50) + BOOST_MIN_VOLTAGE;
	spin_unlock(&pmvb->splock);

	return snprintf(buf, PAGE_SIZE, "%d\n", vb_voltage);
}

static ssize_t set_vreg_boost_voltage(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int ret = -EBUSY;
	uint32_t vb_voltage = 0;
	struct pm8916_vreg_boost *pmvb = dev_get_drvdata(dev);

	spin_lock(&pmvb->splock);
	if (pmvb->enable)
		goto set_vreg_boost_voltage_failure;

	ret = kstrtou32(buf, 10, &vb_voltage);
	if (ret)
		goto set_vreg_boost_voltage_failure;

	if ((vb_voltage > BOOST_MAX_VOLTAGE) ||
			(vb_voltage < BOOST_MIN_VOLTAGE)) {
		ret = -EINVAL;
		goto set_vreg_boost_voltage_failure;
}

	pmvb->vb_voltage = (uint8_t) ((vb_voltage - BOOST_MIN_VOLTAGE) / 50);

	ret = configure_vreg_boost(pmvb);
	if (ret)
		goto set_vreg_boost_voltage_failure;

	ret = count;
set_vreg_boost_voltage_failure:
	spin_unlock(&pmvb->splock);
	return count;
}

static DEVICE_ATTR(vb_voltage, S_IRUGO | S_IWUSR,
		show_vreg_boost_voltage, set_vreg_boost_voltage);

static ssize_t show_vreg_boost_current(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct pm8916_vreg_boost *pmvb = dev_get_drvdata(dev);
	uint32_t vb_current;

	spin_lock(&pmvb->splock);
	vb_current = (pmvb->vb_current * 500) + BOOST_MIN_CURRENT;
	spin_unlock(&pmvb->splock);

	return snprintf(buf, PAGE_SIZE, "%d\n", vb_current);
}

static ssize_t set_vreg_boost_current(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int ret = -EBUSY;
	uint32_t vb_current = 0;
	struct pm8916_vreg_boost *pmvb = dev_get_drvdata(dev);

	spin_lock(&pmvb->splock);
	if (pmvb->enable)
		goto set_current_failure;

	ret = kstrtou32(buf, 10, &vb_current);
	if (ret)
		goto set_current_failure;

	if ((vb_current > BOOST_MAX_CURRENT) ||
			(vb_current < BOOST_MIN_CURRENT)) {
		ret = -EINVAL;
		goto set_current_failure;
	}

	pmvb->vb_current = (uint8_t) ((vb_current - BOOST_MIN_CURRENT) / 500);

	ret = configure_vreg_boost(pmvb);
	if (ret)
		goto set_current_failure;

	ret = count;

set_current_failure:
	spin_unlock(&pmvb->splock);
	return count;
}

static DEVICE_ATTR(vb_current, S_IRUGO | S_IWUSR,
		show_vreg_boost_current, set_vreg_boost_current);

static struct attribute *pvboost_attributes[4] = {
	&dev_attr_enable.attr,
	NULL,
	NULL,
	NULL,
};

static struct attribute_group pvboost_attribute_grp = {
	.attrs = pvboost_attributes,
};

static int pm8916_spmi_probe(struct spmi_device *spmi)
{
	int ret = -EINVAL;
	struct pm8916_vreg_boost *pmvb = NULL;
	struct resource *spmi_resource;
	bool enable_at_boot;
	uint32_t vb_voltage, vb_current;
	bool allow_update;
	uint8_t pmic_rev = 0;

	ret = -ENXIO;
	spmi_resource = spmi_get_resource(spmi, NULL, IORESOURCE_MEM, 0);
	if (!spmi_resource) {
		dev_err(&spmi->dev, "Unable to get Tombak base address\n");
		goto exit_failure;
	}

	dev_dbg(&spmi->dev, "spmi %p slave ID 0x%08x base %08x resource %08x\n",
		spmi, spmi->sid,
		(spmi->sid << 16) + spmi_resource->start,
		spmi_resource->start);

	ret = -EINVAL;
	if (spmi_resource->start != TOMBAK_CORE_0_SPMI_ADDR)
		goto exit_failure;

	ret = -ENOMEM;
	pmvb = devm_kzalloc(&spmi->dev, sizeof(*pmvb), GFP_KERNEL);
	if (!pmvb)
		goto exit_failure;

	pmvb->spmi = spmi;
	pmvb->dev = &spmi->dev;
	pmvb->base = (spmi->sid << 16) + spmi_resource->start;

	spin_lock_init(&pmvb->splock);
	dev_set_drvdata(&spmi->dev, pmvb);

	/* check codec revision */
	ret = pmic_spmi_read_device(pmvb,
			PMIC_A_DIGITAL_REVISION1, &pmic_rev, 1);
	if (ret) {
		dev_err(&spmi->dev, "Fail to read revision\n");
		goto exit_failure;
	}

	ret = -EINVAL;
	if (pmic_rev != 0x1) { /* TOMBAK_2_0 */
		dev_err(&spmi->dev,
			"Bad pmic revision (%d)\n", pmic_rev);
		goto exit_failure;
	}

	/* optional */
	enable_at_boot = of_property_read_bool(spmi->dev.of_node,
			"vreg-boost-enable");

	allow_update = of_property_read_bool(spmi->dev.of_node,
			"vreg-boost-allow-update");

	/* voltage */
	ret = of_property_read_u32(spmi->dev.of_node,
			"vreg-boost-voltage", &vb_voltage);
	if (ret) {
		dev_err(&spmi->dev,
			"missing 'vreg-boost-voltage' property\n");
		goto exit_failure;
	}

	if ((vb_voltage > BOOST_MAX_VOLTAGE) ||
			(vb_voltage < BOOST_MIN_VOLTAGE)) {
		ret = -EINVAL;
		dev_err(&spmi->dev,
			"bad value for 'vreg-boost-voltage' property\n");
		goto exit_failure;
	}

	pmvb->vb_voltage = (uint8_t) ((vb_voltage - BOOST_MIN_VOLTAGE) / 50);

	/* current */
	ret = of_property_read_u32(spmi->dev.of_node,
			"vreg-boost-current", &vb_current);
	if (ret) {
		dev_err(&spmi->dev,
			"missing 'vreg-boost-current' properties\n");
		goto exit_failure;
	}

	if ((vb_current > BOOST_MAX_CURRENT) ||
			(vb_current < BOOST_MIN_CURRENT)) {
		ret = -EINVAL;
		dev_err(&spmi->dev,
			"bad value for 'vreg-boost-current' property\n");
		goto exit_failure;
	}

	pmvb->vb_current = (uint8_t) ((vb_current - BOOST_MIN_CURRENT) / 500);

	dev_dbg(&spmi->dev, "voltage 0x%02x\tcurrent 0x%02x\n",
				pmvb->vb_voltage, pmvb->vb_current);

	dev_dbg(&spmi->dev, "configure vreg boost\n");
	ret = configure_vreg_boost(pmvb);
	if (ret) {
		dev_err(&spmi->dev, "Fail to configure vreg_boost\n");
		goto exit_failure;
	}

	dev_dbg(&spmi->dev, "enable vreg boost\n");
	ret = enable_vreg_boost(pmvb, enable_at_boot);
	if (ret) {
		dev_err(&spmi->dev, "Fail to %sable vreg_boost\n",
			enable_at_boot ? "en":"dis");
		goto exit_failure;
	}

	usleep_range(1000, 1100);

	dev_err(&spmi->dev, "vreg boost %sabled in dt\n",
			enable_at_boot ? "en":"dis");

	pmvb->enable = enable_at_boot;

	/* add sysfs config file for current */
	if (allow_update) {
		pvboost_attributes[1] = &dev_attr_vb_voltage.attr;
		pvboost_attributes[2] = &dev_attr_vb_current.attr;
	}

	ret = sysfs_create_group(&spmi->dev.kobj, &pvboost_attribute_grp);
	if (ret < 0)
		dev_err(&spmi->dev, "Unable to create 'enable' attribute\n");

exit_failure:
	return ret;
}

static int pm8916_spmi_remove(struct spmi_device *spmi)
{
	struct pm8916_vreg_boost *pmvb = dev_get_drvdata(&spmi->dev);

	/* disable vreg boost */
	sysfs_remove_group(&spmi->dev.kobj, &pvboost_attribute_grp);

	enable_vreg_boost(pmvb, false);
	dev_info(&spmi->dev, "vreg boost disabled\n");

	dev_set_drvdata(&spmi->dev, NULL);
	return 0;
}

static struct spmi_device_id pm8916_spmi_id_table[] = {
	{"parrot,pm8916_vreg_boost", 0},
	{}
};

static struct of_device_id pm8916_vreg_boost_spmi_of_match[] = {
	{ .compatible = "parrot,pm8916_vreg_boost",},
	{ },
};

static struct spmi_driver wcd_spmi_driver = {
	.driver                 = {
		.owner          = THIS_MODULE,
		.name           = "pm8916_vreg_boost",
		.of_match_table = pm8916_vreg_boost_spmi_of_match
	},

	.id_table               = pm8916_spmi_id_table,
	.probe                  = pm8916_spmi_probe,
	.remove                 = pm8916_spmi_remove,
};

static int __init pm8916_vreg_boost_init(void)
{
	spmi_driver_register(&wcd_spmi_driver);
	return 0;
}
late_initcall(pm8916_vreg_boost_init);

static void __exit pm8916_vreg_boost_exit(void)
{
	spmi_driver_unregister(&wcd_spmi_driver);
}
module_exit(pm8916_vreg_boost_exit);

MODULE_DESCRIPTION("Qualcomm PMIC8916 vreg boost driver");
MODULE_LICENSE("GPL v2");
MODULE_DEVICE_TABLE(of, pm8916_spmi_id_table);
