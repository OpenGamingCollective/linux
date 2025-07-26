// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Platform driver for AYN x86 Handhelds.
 *
 * Implements multiple attributes provided by the EC. Fan reading and control,
 * as well as temperature sensor readings are exposed via hwmon sysfs. EC RGB
 * control is exposed via an led-class-multicolor interface.
 *
 * Fan control is provided via a pwm interface in the range [0-255]. AYN use
 * [0-128] as the range in the EC, the written value is scaled to accommodate.
 * The EC also provides a configurable fan curve with five set points that
 * associate a temperature in Celcius [0-100] with a fan speed [0-128]. The
 * auto_point fan speeds are also scaled from the range [0-255]. Temperature
 * readings are scaled from degrees to millidegrees when read.
 *
 * RGB control is provided using 4 registers. One each for the colors red,
 * green, and blue are [0-255]. There is also a effect register that takes
 * switches between an EC controlled breathing that cycles through all colors
 * and fades in/out, and manual, which enables setting a user defined color.
 *
 * Copyright (C) 2025 Derek J. Clark <derekjohn.clark@gmail.com>
 */

#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/dmi.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/types.h>

/* Fan speed and PWM registers */
#define AYN_SENSOR_PWM_FAN_ENABLE_REG	0x10 /* PWM operating mode */
#define AYN_SENSOR_PWM_FAN_SET_REG	0x11 /* PWM duty cycle */
#define AYN_SENSOR_PWM_FAN_SPEED_REG	0x20 /* Fan speed */

/* EC controlled fan curve registers */
#define AYN_SENSOR_PWM_FAN_SPEED_1_REG	0x12
#define AYN_SENSOR_PWM_FAN_SPEED_2_REG	0x14
#define AYN_SENSOR_PWM_FAN_SPEED_3_REG	0x16
#define AYN_SENSOR_PWM_FAN_SPEED_4_REG	0x18
#define AYN_SENSOR_PWM_FAN_SPEED_5_REG	0x1A
#define AYN_SENSOR_PWM_FAN_TEMP_1_REG	0x13
#define AYN_SENSOR_PWM_FAN_TEMP_2_REG	0x15
#define AYN_SENSOR_PWM_FAN_TEMP_3_REG	0x17
#define AYN_SENSOR_PWM_FAN_TEMP_4_REG	0x19
#define AYN_SENSOR_PWM_FAN_TEMP_5_REG	0x1B

/* AYN EC PWM Fan modes */
#define AYN_PWM_FAN_MODE_MANUAL	0x00
#define AYN_PWM_FAN_MODE_AUTO		0x01
#define AYN_PWM_FAN_MODE_EC_CURVE	0x02

/* hwmon fan modes */
#define HWMON_PWM_FAN_MODE_FULL	0x00
#define HWMON_PWM_FAN_MODE_MANUAL	0x01
#define HWMON_PWM_FAN_MODE_AUTO	0x02
#define HWMON_PWM_FAN_MODE_EC_CURVE	0x03

/* Handle ACPI lock mechanism */
#define ACPI_LOCK_DELAY_MS 500

int ayn_pwm_curve_registers[10] = {
	AYN_SENSOR_PWM_FAN_SPEED_1_REG,
	AYN_SENSOR_PWM_FAN_SPEED_2_REG,
	AYN_SENSOR_PWM_FAN_SPEED_3_REG,
	AYN_SENSOR_PWM_FAN_SPEED_4_REG,
	AYN_SENSOR_PWM_FAN_SPEED_5_REG,
	AYN_SENSOR_PWM_FAN_TEMP_1_REG,
	AYN_SENSOR_PWM_FAN_TEMP_2_REG,
	AYN_SENSOR_PWM_FAN_TEMP_3_REG,
	AYN_SENSOR_PWM_FAN_TEMP_4_REG,
	AYN_SENSOR_PWM_FAN_TEMP_5_REG,
};

struct ayn_device {
	u32 ayn_lock; /* ACPI EC Lock */
} drvdata;

/* Handle ACPI lock mechanism */
#define ACPI_LOCK_DELAY_MS 500

static bool lock_global_acpi_lock(void)
{
	return ACPI_SUCCESS(acpi_acquire_global_lock(ACPI_LOCK_DELAY_MS,
						     &drvdata.ayn_lock));
}

static bool unlock_global_acpi_lock(void)
{
	return ACPI_SUCCESS(acpi_release_global_lock(drvdata.ayn_lock));
}

/**
 * read_from_ec() - Reads a value from the embedded controller.
 *
 * @reg: The register to start the read from.
 * @size: The number of sequential registers the data is contained in.
 * @val: Pointer to return the data with.
 *
 * Return: 0, or an error.
 */
static int read_from_ec(u8 reg, int size, long *val)
{
	int ret, i;
	u8 buf;

	if (!lock_global_acpi_lock())
		return -EBUSY;

	*val = 0;
	for (i = 0; i < size; i++) {
		ret = ec_read(reg + i, &buf);
		if (ret)
			return ret;
		*val <<= i * 8;
		*val += buf;
	}

	if (!unlock_global_acpi_lock())
		return -EBUSY;

	return 0;
}

/**
 * write_to_ec() - Writes a value to the embedded controller.
 *
 * @reg: The register to write to.
 * @val: Value to write
 *
 * Return: 0, or an error.
 */
static int write_to_ec(u8 reg, u8 val)
{
	int ret;

	if (!lock_global_acpi_lock())
		return -EBUSY;

	pr_info("Writing EC value %d to register %u\n", val, reg);
	ret = ec_write(reg, val);

	if (!unlock_global_acpi_lock())
		return -EBUSY;

	return ret;
}

/**
 * ayn_pwm_manual() - Enable manual control of the fan.
 */
static int ayn_pwm_manual(void)
{
	return write_to_ec(AYN_SENSOR_PWM_FAN_ENABLE_REG, 0x00);
}

/**
 * ayn_pwm_full() - Set fan to 100% speed.
 */
static int ayn_pwm_full(void)
{
	int ret;

	ret = write_to_ec(AYN_SENSOR_PWM_FAN_ENABLE_REG, 0x00);
	if (ret)
		return ret;

	return write_to_ec(AYN_SENSOR_PWM_FAN_SET_REG, 128);
}

/**
 * ayn_pwm_auto() - Enable automatic EC control of the fan.
 */
static int ayn_pwm_auto(void)
{
	return write_to_ec(AYN_SENSOR_PWM_FAN_ENABLE_REG, 0x01);
}

/**
 * ayn_pwm_ec_curve() - Enable manually setting the fan curve for automatic
 * EC control of the fan.
 */
static int ayn_pwm_ec_curve(void)
{
	return write_to_ec(AYN_SENSOR_PWM_FAN_ENABLE_REG, 0x02);
}

/**
 * ayn_ec_hwmon_is_visible() - Determines RO or RW for hwmon attribute sysfs.
 *
 * @drvdata: Unused void pointer to context data.
 * @type: The hwmon_sensor_types type.
 * @attr: The attribute to set RO/RW on.
 * @channel: HWMON subsystem usage flags for the attribute.
 *
 * Return: Permission level.
 */
static umode_t ayn_ec_hwmon_is_visible(const void *drvdata,
				       enum hwmon_sensor_types type, u32 attr,
				       int channel)
{
	switch (type) {
	case hwmon_fan:
		return 0444;
	case hwmon_pwm:
		return 0644;
	default:
		return 0;
	}
}

/**
 * ayn_pwm_fan_read() - Read from a hwmon pwm or fan attribute.
 *
 * @dev: parent device of the given attribute.
 * @type: The hwmon_sensor_types type.
 * @attr: The attribute to read from.
 * @channel: HWMON subsystem usage flags for the attribute.
 * @val: Pointer to return the read value from.
 *
 * Return: 0, or an error.
 */
static int ayn_pwm_fan_read(struct device *dev, enum hwmon_sensor_types type,
			    u32 attr, int channel, long *val)
{
	int ret;

	switch (type) {
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_input:
			return read_from_ec(AYN_SENSOR_PWM_FAN_SPEED_REG, 2,
					    val);
		default:
			break;
		}
		break;
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_enable:
			ret = read_from_ec(AYN_SENSOR_PWM_FAN_ENABLE_REG, 1,
					   val);
			if (ret)
				return ret;

			/* EC uses 0 for manual, 1 for automatic, 2 for user
			 * fan curve. Reflect hwmon usage instead.
			 */
			if (*val == 1) {
				*val = 2;
				return 0;
			}

			if (*val == 2) {
				*val = 3;
				return 0;
			}

			/* Return 0 when fan at max, otherwise 1 for manual. */
			ret = read_from_ec(AYN_SENSOR_PWM_FAN_SET_REG, 1, val);
			if (ret)
				return ret;

			if (*val == 128)
				*val = 0;
			else
				*val = 1;

			return ret;
		case hwmon_pwm_input:
			ret = read_from_ec(AYN_SENSOR_PWM_FAN_SET_REG, 1, val);
			if (ret)
				return ret;

			*val = *val << 1; /* Max value is 128, scale to 255 */

			return 0;
		default:
			break;
		}
		break;
	default:
		break;
	}
	return -EOPNOTSUPP;
}

/**
 * ayn_pwm_fan_write() - Write to a hwmon pwm attribute.
 *
 * @dev: parent device of the given attribute.
 * @type: The hwmon_sensor_types type.
 * @attr: The attribute to write to.
 * @channel: HWMON subsystem usage flags for the attribute.
 * @val: Value to write.
 *
 * Return: 0, or an error.
 */
static int ayn_pwm_fan_write(struct device *dev, enum hwmon_sensor_types type,
			     u32 attr, int channel, long val)
{
	if (type != hwmon_pwm)
		return -EOPNOTSUPP;
	switch (attr) {
	case hwmon_pwm_enable:
		switch (val) {
		case HWMON_PWM_FAN_MODE_FULL:
			return ayn_pwm_full();
		case HWMON_PWM_FAN_MODE_MANUAL:
			return ayn_pwm_manual();
		case HWMON_PWM_FAN_MODE_AUTO:
			return ayn_pwm_auto();
		case HWMON_PWM_FAN_MODE_EC_CURVE:
			return ayn_pwm_ec_curve();
		default:
			return -EINVAL;
		}
	case hwmon_pwm_input:
		if (val < 0 || val > 255)
			return -EINVAL;

		val = val >> 1; /* Max value is 128, scale from 255 */

		return write_to_ec(AYN_SENSOR_PWM_FAN_SET_REG, val);
	default:
		return -EOPNOTSUPP;
	}
}

static const struct hwmon_channel_info *ayn_ec_sensors[] = {
	HWMON_CHANNEL_INFO(fan, HWMON_F_INPUT),
	HWMON_CHANNEL_INFO(pwm, HWMON_PWM_INPUT | HWMON_PWM_ENABLE),
	NULL,
};

static const struct hwmon_ops ayn_ec_hwmon_ops = {
	.is_visible = ayn_ec_hwmon_is_visible,
	.read = ayn_pwm_fan_read,
	.write = ayn_pwm_fan_write,
};

static const struct hwmon_chip_info ayn_ec_chip_info = {
	.ops = &ayn_ec_hwmon_ops,
	.info = ayn_ec_sensors,
};

/**
 * pwm_curve_store() - Write a fan curve speed or temperature value.
 *
 * @dev: The attribute's parent device.
 * @attr: The attribute to read.
 * @buf: Input value string from sysfs write.
 *
 * Return: Number of bytes read, or an error.
 */
static ssize_t pwm_curve_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int i = to_sensor_dev_attr(attr)->index;
	int ret, val;
	u8 reg;

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if (i < 5) {
		if (val < 0 || val > 255)
			return -EINVAL;
		val = val >> 1; /* Max EC value is 128, scale from 255 */
	} else
		if (val < 0 || val > 100)
			return -EINVAL;

	reg = ayn_pwm_curve_registers[i];

	ret = write_to_ec(reg, val);
	if (ret)
		return ret;

	return count;
}

/**
 * pwm_curve_show() - Read a fan curve speed or temperature value.
 *
 * @dev: The attribute's parent device.
 * @attr: The attribute to read.
 * @buf: Output buffer.
 *
 * Return: Number of bytes read, or an error.
 */
static ssize_t pwm_curve_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	int i = to_sensor_dev_attr(attr)->index;
	long val;
	int ret;
	u8 reg;

	reg = ayn_pwm_curve_registers[i];

	ret = read_from_ec(reg, 1, &val);
	if (ret)
		return ret;

	if (i < 5)
		val = val << 1; /* Max EC value is 128, scale to 255 */

	return sysfs_emit(buf, "%ld\n", val);
}

/* Fan curve attributes */
static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point1_pwm, pwm_curve, 0);
static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point2_pwm, pwm_curve, 1);
static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point3_pwm, pwm_curve, 2);
static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point4_pwm, pwm_curve, 3);
static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point5_pwm, pwm_curve, 4);
static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point1_temp, pwm_curve, 5);
static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point2_temp, pwm_curve, 6);
static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point3_temp, pwm_curve, 7);
static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point4_temp, pwm_curve, 8);
static SENSOR_DEVICE_ATTR_RW(pwm1_auto_point5_temp, pwm_curve, 9);

static struct attribute *ayn_sensors_attrs[] = {
	&sensor_dev_attr_pwm1_auto_point1_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point1_temp.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point2_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point2_temp.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point3_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point3_temp.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point4_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point4_temp.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point5_pwm.dev_attr.attr,
	&sensor_dev_attr_pwm1_auto_point5_temp.dev_attr.attr,
	NULL,
};

ATTRIBUTE_GROUPS(ayn_sensors);

static int ayn_ec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device *hwdev;

	hwdev = devm_hwmon_device_register_with_info(dev, "aynec", NULL,
						     &ayn_ec_chip_info,
						     ayn_sensors_groups);
	return PTR_ERR_OR_ZERO(hwdev);
}

static struct platform_driver ayn_ec_driver = {
	.driver = {
		.name = "ayn-ec",
	},
	.probe = ayn_ec_probe,
};

static struct platform_device *ayn_ec_device;

static int __init ayn_ec_init(void)
{
	ayn_ec_device = platform_create_bundle(&ayn_ec_driver, ayn_ec_probe,
					       NULL, 0, NULL, 0);

	return PTR_ERR_OR_ZERO(ayn_ec_device);
}

static void __exit ayn_ec_exit(void)
{
	platform_device_unregister(ayn_ec_device);
	platform_driver_unregister(&ayn_ec_driver);
}

static const struct dmi_system_id ayn_dmi_table[] = {
	{
		.ident = "AYN Loki Max",
		.matches = {
			DMI_EXACT_MATCH(DMI_BOARD_VENDOR, "ayn"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "Loki Max"),
		},
	},
	{
		.ident = "AYN Loki MiniPro",
		.matches = {
			DMI_EXACT_MATCH(DMI_BOARD_VENDOR, "ayn"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "Loki MiniPro"),
		},
	},
	{
		.ident = "AYN Loki Zero",
		.matches = {
			DMI_EXACT_MATCH(DMI_BOARD_VENDOR, "ayn"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "Loki Zero"),
		},
	},
	{
		.ident = "Tectoy Zeenix Lite",
		.matches = {
			DMI_EXACT_MATCH(DMI_BOARD_VENDOR, "Tectoy"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "Zeenix Lite"),
		},
	},
	{},
};

MODULE_DEVICE_TABLE(dmi, ayn_dmi_table);

module_init(ayn_ec_init);
module_exit(ayn_ec_exit);

MODULE_AUTHOR("Derek J. Clark <derekjohn.clark@gmail.com>");
MODULE_DESCRIPTION("Platform driver that handles EC sensors of AYN x86 devices");
MODULE_LICENSE("GPL");
