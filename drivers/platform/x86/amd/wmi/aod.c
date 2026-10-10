// SPDX-License-Identifier: GPL-2.0
/*
 * AMD Overclocking (AOD) WMI driver
 *
 * Copyright (c) 2026, Advanced Micro Devices, Inc.
 * All Rights Reserved.
 *
 * Author: Shyam Sundar S K <Shyam-sundar.S-k@amd.com>
 */

#include <linux/cleanup.h>
#include <linux/compiler_attributes.h>
#include <linux/container_of.h>
#include <linux/dev_printk.h>
#include <linux/device.h>
#include <linux/device/devres.h>
#include <linux/errno.h>
#include <linux/gfp_types.h>
#include <linux/kobject.h>
#include <linux/kstrtox.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/units.h>
#include <linux/wmi.h>

#include <linux/byteorder/generic.h>

#define DRIVER_NAME			"amd-wmi-aod"
#define AMD_AOD_WMI_GUID		"95F20B61-3D44-41F8-87E1-03CBF94C4E8F"

/* WMI method IDs */
#define AOD_QUERY_VERSION		1
#define AOD_GET_OBJECT_ID		3
#define AOD_GET_DVALUES			4
#define AOD_RUN_COMMAND			5
#define AOD_GET_MINMAX_STEPPING		6
#define AOD_GET_OBJECT_ID2		8

/* Command IDs */
#define AOD_CMD_MEM_CLOCK		0x00020001
#define AOD_CMD_SMT_ENABLE		0x00020026
#define AOD_CMD_PPT_LIMIT		0x00050001
#define AOD_CMD_TDC_LIMIT		0x00050002
#define AOD_CMD_EDC_LIMIT		0x00050003

#define AOD_MAX_OID_COUNT		64
#define AOD_DVALUES_COUNT		100

/* Property flag of a command: discrete values or a linear range */
#define AOD_PROPERTY_LINEAR		1

#define to_amd_aod_attr(_attr)	container_of(_attr, struct amd_aod_attr, dev_attr)

#define AMD_AOD_ATTR(_name, _mode, _show, _store, _setting, _field, _scale)	\
	static struct amd_aod_attr amd_aod_attr_##_name = {			\
		.dev_attr = __ATTR(_name, _mode, _show, _store),		\
		.setting = _setting,						\
		.field = _field,						\
		.scale = _scale,						\
	}

/*
 * GetObjectID/GetObjectID2 result: the length, followed by the ID and the
 * property arrays, followed by AOD_MAX_OID_COUNT WMI strings.
 *
 * The MOF defines fixed-size arrays for all results, so shorter buffers
 * are rejected.
 */
struct amd_aod_oid_result {
	__le32 length;
	__le32 id[AOD_MAX_OID_COUNT];
	__le32 property[AOD_MAX_OID_COUNT];
} __packed;

struct amd_aod_dvalues_result {
	__le32 id;
	__le32 current_value;
	__le32 length;
	__le32 dvalues[AOD_DVALUES_COUNT];
} __packed;

struct amd_aod_minmax_result {
	__le32 id;
	__le32 current_value;
	__le32 min;
	__le32 max;
	__le32 stepping;
} __packed;

/* Start of the RunCommand result buffer */
struct amd_aod_run_command_result {
	__le16 smi_cmd;
	__le32 smi_status;
} __packed;

/* Command list returned by GetObjectID/GetObjectID2 */
struct amd_aod_oid_package {
	u32 length;
	u32 id[AOD_MAX_OID_COUNT];
	u32 property[AOD_MAX_OID_COUNT];
} __packed;

enum amd_aod_setting {
	AOD_SETTING_PPT,
	AOD_SETTING_TDC,
	AOD_SETTING_EDC,
	AOD_SETTING_MEM_CLOCK,
	AOD_SETTING_SMT,
	AOD_SETTING_MAX,
};

enum amd_aod_field {
	AOD_FIELD_CURRENT,
	AOD_FIELD_MIN,
	AOD_FIELD_MAX,
	AOD_FIELD_STEP,
};

struct amd_aod_data {
	struct wmi_device *wdev;
	struct mutex lock;	/* Serializes WMI calls and protects the fields below */
	bool supported[AOD_SETTING_MAX];
	bool linear[AOD_SETTING_MAX];
};

struct amd_aod_attr {
	struct device_attribute dev_attr;
	enum amd_aod_setting setting;
	enum amd_aod_field field;
	u32 scale;	/* sysfs units per firmware unit */
};

static const u32 amd_aod_setting_cmd[AOD_SETTING_MAX] = {
	[AOD_SETTING_PPT] = AOD_CMD_PPT_LIMIT,
	[AOD_SETTING_TDC] = AOD_CMD_TDC_LIMIT,
	[AOD_SETTING_EDC] = AOD_CMD_EDC_LIMIT,
	[AOD_SETTING_MEM_CLOCK] = AOD_CMD_MEM_CLOCK,
	[AOD_SETTING_SMT] = AOD_CMD_SMT_ENABLE,
};

static bool amd_aod_is_limit(enum amd_aod_setting setting)
{
	switch (setting) {
	case AOD_SETTING_PPT:
	case AOD_SETTING_TDC:
	case AOD_SETTING_EDC:
		return true;
	default:
		return false;
	}
}

/*
 * The firmware reads the input buffer even for methods that take no
 * arguments, so a dummy 32-bit argument is passed to those methods.
 */
static int amd_aod_call(struct amd_aod_data *data, u32 method_id, u32 arg,
			struct wmi_buffer *out, size_t min_size)
{
	__le32 in_arg = cpu_to_le32(arg);
	const struct wmi_buffer in = {
		.length = sizeof(in_arg),
		.data = &in_arg,
	};

	return wmidev_invoke_method(data->wdev, 0, method_id, &in, out, min_size);
}

static int amd_aod_query_version(struct amd_aod_data *data, u32 *version)
{
	struct wmi_buffer out;
	int ret;

	ret = amd_aod_call(data, AOD_QUERY_VERSION, 0, &out, sizeof(__le32));
	if (ret)
		return ret;

	*version = le32_to_cpup(out.data);
	kfree(out.data);

	return 0;
}

static void amd_aod_parse_object_id(const struct wmi_buffer *out,
				    struct amd_aod_oid_package *pkg)
{
	const struct amd_aod_oid_result *res = out->data;
	int i;

	pkg->length = min(le32_to_cpu(res->length), AOD_MAX_OID_COUNT);
	for (i = 0; i < AOD_MAX_OID_COUNT; i++) {
		pkg->id[i] = le32_to_cpu(res->id[i]);
		pkg->property[i] = le32_to_cpu(res->property[i]);
	}
}

static int amd_aod_get_object_id(struct amd_aod_data *data, u32 method_id,
				 struct amd_aod_oid_package *pkg)
{
	struct wmi_buffer out;
	int ret;

	ret = amd_aod_call(data, method_id, 0, &out, sizeof(struct amd_aod_oid_result));
	if (ret)
		return ret;

	amd_aod_parse_object_id(&out, pkg);
	kfree(out.data);

	return 0;
}

static int amd_aod_get_dvalues(struct amd_aod_data *data, u32 cmd_id,
			       struct amd_aod_dvalues_result *res)
{
	struct wmi_buffer out;
	int ret;

	ret = amd_aod_call(data, AOD_GET_DVALUES, cmd_id, &out, sizeof(*res));
	if (ret)
		return ret;

	memcpy(res, out.data, sizeof(*res));
	kfree(out.data);

	return 0;
}

static int amd_aod_get_minmax_stepping(struct amd_aod_data *data, u32 cmd_id,
				       struct amd_aod_minmax_result *res)
{
	struct wmi_buffer out;
	int ret;

	ret = amd_aod_call(data, AOD_GET_MINMAX_STEPPING, cmd_id, &out, sizeof(*res));
	if (ret)
		return ret;

	memcpy(res, out.data, sizeof(*res));
	kfree(out.data);

	return 0;
}

static int amd_aod_run_command(struct amd_aod_data *data, u32 cmd_id, u32 value, u32 *status)
{
	__le32 args[2] = { cpu_to_le32(cmd_id), cpu_to_le32(value) };
	const struct wmi_buffer in = {
		.length = sizeof(args),
		.data = args,
	};
	const struct amd_aod_run_command_result *res;
	struct wmi_buffer out;
	int ret;

	ret = wmidev_invoke_method(data->wdev, 0, AOD_RUN_COMMAND, &in, &out, sizeof(*res));
	if (ret)
		return ret;

	res = out.data;
	*status = le32_to_cpu(res->smi_status);
	kfree(out.data);

	return 0;
}

static int amd_aod_set_value(struct amd_aod_data *data, u32 cmd_id, u32 value)
{
	u32 status;
	int ret;

	ret = amd_aod_run_command(data, cmd_id, value, &status);
	if (ret)
		return ret;

	return status ? -EIO : 0;
}

/* Read the current value, using the method that matches the command property */
static int amd_aod_get_current(struct amd_aod_data *data, enum amd_aod_setting setting,
			       u32 *value)
{
	u32 cmd_id = amd_aod_setting_cmd[setting];
	struct amd_aod_dvalues_result dres;
	struct amd_aod_minmax_result mres;
	int ret;

	if (data->linear[setting]) {
		ret = amd_aod_get_minmax_stepping(data, cmd_id, &mres);
		if (ret)
			return ret;

		if (le32_to_cpu(mres.id) != cmd_id)
			return -EPROTO;

		*value = le32_to_cpu(mres.current_value);
		return 0;
	}

	ret = amd_aod_get_dvalues(data, cmd_id, &dres);
	if (ret)
		return ret;

	if (le32_to_cpu(dres.id) != cmd_id)
		return -EPROTO;

	*value = le32_to_cpu(dres.current_value);

	return 0;
}

/*
 * Linear commands accept values within the min/max range in multiples of
 * the step size, discrete commands only accept the values listed by the
 * firmware.
 */
static int amd_aod_check_value(struct amd_aod_data *data, enum amd_aod_setting setting,
			       u32 value)
{
	u32 cmd_id = amd_aod_setting_cmd[setting];
	struct amd_aod_dvalues_result dres;
	struct amd_aod_minmax_result mres;
	u32 min, max, step, count, i;
	int ret;

	if (data->linear[setting]) {
		ret = amd_aod_get_minmax_stepping(data, cmd_id, &mres);
		if (ret)
			return ret;

		min = le32_to_cpu(mres.min);
		max = le32_to_cpu(mres.max);
		step = le32_to_cpu(mres.stepping);

		if (value < min || value > max)
			return -EINVAL;

		if (step && (value - min) % step)
			return -EINVAL;

		return 0;
	}

	ret = amd_aod_get_dvalues(data, cmd_id, &dres);
	if (ret)
		return ret;

	count = min(le32_to_cpu(dres.length), AOD_DVALUES_COUNT);
	for (i = 0; i < count; i++) {
		if (le32_to_cpu(dres.dvalues[i]) == value)
			return 0;
	}

	return -EINVAL;
}

static ssize_t amd_aod_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct amd_aod_attr *aod_attr = to_amd_aod_attr(attr);
	struct amd_aod_data *data = dev_get_drvdata(dev);
	u32 value;
	int ret;

	guard(mutex)(&data->lock);

	ret = amd_aod_get_current(data, aod_attr->setting, &value);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%llu\n", (u64)value * aod_attr->scale);
}

static ssize_t amd_aod_range_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct amd_aod_attr *aod_attr = to_amd_aod_attr(attr);
	struct amd_aod_data *data = dev_get_drvdata(dev);
	u32 cmd_id = amd_aod_setting_cmd[aod_attr->setting];
	struct amd_aod_minmax_result res;
	u32 value;
	int ret;

	guard(mutex)(&data->lock);

	ret = amd_aod_get_minmax_stepping(data, cmd_id, &res);
	if (ret)
		return ret;

	switch (aod_attr->field) {
	case AOD_FIELD_MIN:
		value = le32_to_cpu(res.min);
		break;
	case AOD_FIELD_MAX:
		value = le32_to_cpu(res.max);
		break;
	default:
		value = le32_to_cpu(res.stepping);
		break;
	}

	return sysfs_emit(buf, "%llu\n", (u64)value * aod_attr->scale);
}

static ssize_t amd_aod_value_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct amd_aod_attr *aod_attr = to_amd_aod_attr(attr);
	struct amd_aod_data *data = dev_get_drvdata(dev);
	u32 value;
	int ret;

	ret = kstrtou32(buf, 10, &value);
	if (ret)
		return ret;

	if (value % aod_attr->scale)
		return -EINVAL;

	/* smt_enable only accepts 0 and 1 */
	if (aod_attr->setting == AOD_SETTING_SMT && value > 1)
		return -EINVAL;

	value /= aod_attr->scale;

	guard(mutex)(&data->lock);

	ret = amd_aod_check_value(data, aod_attr->setting, value);
	if (ret)
		return ret;

	ret = amd_aod_set_value(data, amd_aod_setting_cmd[aod_attr->setting], value);
	if (ret)
		return ret;

	return count;
}

AMD_AOD_ATTR(ppt_limit, 0200, NULL, amd_aod_value_store,
	     AOD_SETTING_PPT, AOD_FIELD_CURRENT, MICROWATT_PER_MILLIWATT);
AMD_AOD_ATTR(ppt_limit_min, 0400, amd_aod_range_show, NULL,
	     AOD_SETTING_PPT, AOD_FIELD_MIN, MICROWATT_PER_MILLIWATT);
AMD_AOD_ATTR(ppt_limit_max, 0400, amd_aod_range_show, NULL,
	     AOD_SETTING_PPT, AOD_FIELD_MAX, MICROWATT_PER_MILLIWATT);
AMD_AOD_ATTR(ppt_limit_step, 0400, amd_aod_range_show, NULL,
	     AOD_SETTING_PPT, AOD_FIELD_STEP, MICROWATT_PER_MILLIWATT);
AMD_AOD_ATTR(tdc_limit, 0200, NULL, amd_aod_value_store,
	     AOD_SETTING_TDC, AOD_FIELD_CURRENT, 1);
AMD_AOD_ATTR(tdc_limit_min, 0400, amd_aod_range_show, NULL,
	     AOD_SETTING_TDC, AOD_FIELD_MIN, 1);
AMD_AOD_ATTR(tdc_limit_max, 0400, amd_aod_range_show, NULL,
	     AOD_SETTING_TDC, AOD_FIELD_MAX, 1);
AMD_AOD_ATTR(tdc_limit_step, 0400, amd_aod_range_show, NULL,
	     AOD_SETTING_TDC, AOD_FIELD_STEP, 1);
AMD_AOD_ATTR(edc_limit, 0200, NULL, amd_aod_value_store,
	     AOD_SETTING_EDC, AOD_FIELD_CURRENT, 1);
AMD_AOD_ATTR(edc_limit_min, 0400, amd_aod_range_show, NULL,
	     AOD_SETTING_EDC, AOD_FIELD_MIN, 1);
AMD_AOD_ATTR(edc_limit_max, 0400, amd_aod_range_show, NULL,
	     AOD_SETTING_EDC, AOD_FIELD_MAX, 1);
AMD_AOD_ATTR(edc_limit_step, 0400, amd_aod_range_show, NULL,
	     AOD_SETTING_EDC, AOD_FIELD_STEP, 1);
AMD_AOD_ATTR(mem_clock, 0400, amd_aod_value_show, NULL,
	     AOD_SETTING_MEM_CLOCK, AOD_FIELD_CURRENT, 1);
AMD_AOD_ATTR(smt_enable, 0600, amd_aod_value_show, amd_aod_value_store,
	     AOD_SETTING_SMT, AOD_FIELD_CURRENT, 1);

static struct attribute *amd_aod_attrs[] = {
	&amd_aod_attr_ppt_limit.dev_attr.attr,
	&amd_aod_attr_ppt_limit_min.dev_attr.attr,
	&amd_aod_attr_ppt_limit_max.dev_attr.attr,
	&amd_aod_attr_ppt_limit_step.dev_attr.attr,
	&amd_aod_attr_tdc_limit.dev_attr.attr,
	&amd_aod_attr_tdc_limit_min.dev_attr.attr,
	&amd_aod_attr_tdc_limit_max.dev_attr.attr,
	&amd_aod_attr_tdc_limit_step.dev_attr.attr,
	&amd_aod_attr_edc_limit.dev_attr.attr,
	&amd_aod_attr_edc_limit_min.dev_attr.attr,
	&amd_aod_attr_edc_limit_max.dev_attr.attr,
	&amd_aod_attr_edc_limit_step.dev_attr.attr,
	&amd_aod_attr_mem_clock.dev_attr.attr,
	&amd_aod_attr_smt_enable.dev_attr.attr,
	NULL
};

static umode_t amd_aod_attr_is_visible(struct kobject *kobj, struct attribute *attr, int n)
{
	struct amd_aod_data *data = dev_get_drvdata(kobj_to_dev(kobj));
	struct device_attribute *dev_attr = container_of(attr, struct device_attribute, attr);
	enum amd_aod_setting setting = to_amd_aod_attr(dev_attr)->setting;

	if (!data->supported[setting])
		return 0;

	/* The limits and their range are only supported for linear commands */
	if (amd_aod_is_limit(setting) && !data->linear[setting])
		return 0;

	return attr->mode;
}

static const struct attribute_group amd_aod_group = {
	.attrs = amd_aod_attrs,
	.is_visible = amd_aod_attr_is_visible,
};
__ATTRIBUTE_GROUPS(amd_aod);

static void amd_aod_scan_settings(struct amd_aod_data *data,
				  const struct amd_aod_oid_package *pkg)
{
	int i, j;

	for (i = 0; i < pkg->length; i++) {
		for (j = 0; j < AOD_SETTING_MAX; j++) {
			if (pkg->id[i] != amd_aod_setting_cmd[j])
				continue;

			data->supported[j] = true;
			data->linear[j] = pkg->property[i] == AOD_PROPERTY_LINEAR;
		}
	}
}

static int amd_aod_detect_settings(struct amd_aod_data *data)
{
	struct amd_aod_oid_package *pkg;
	int ret;

	pkg = kzalloc_obj(*pkg);
	if (!pkg)
		return -ENOMEM;

	ret = amd_aod_get_object_id(data, AOD_GET_OBJECT_ID, pkg);
	if (ret)
		goto out;

	amd_aod_scan_settings(data, pkg);

	/* GetObjectID2 might not be implemented by all firmware versions */
	if (amd_aod_get_object_id(data, AOD_GET_OBJECT_ID2, pkg))
		dev_dbg(&data->wdev->dev, "GetObjectID2 not available\n");
	else
		amd_aod_scan_settings(data, pkg);

out:
	kfree(pkg);

	return ret;
}

static int amd_aod_probe(struct wmi_device *wdev, const void *context)
{
	struct device *dev = &wdev->dev;
	struct amd_aod_data *data;
	u32 version;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->wdev = wdev;

	ret = devm_mutex_init(dev, &data->lock);
	if (ret)
		return ret;

	dev_set_drvdata(dev, data);

	/* Make sure that the firmware interface is functional */
	ret = amd_aod_query_version(data, &version);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to query AOD interface version\n");

	ret = amd_aod_detect_settings(data);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get the list of supported commands\n");

	return 0;
}

static const struct wmi_device_id amd_aod_id_table[] = {
	{ AMD_AOD_WMI_GUID, NULL },
	{ }
};
MODULE_DEVICE_TABLE(wmi, amd_aod_id_table);

static struct wmi_driver amd_aod_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.dev_groups = amd_aod_groups,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.id_table = amd_aod_id_table,
	.probe = amd_aod_probe,
	.no_singleton = true,
};
module_wmi_driver(amd_aod_driver);

MODULE_DESCRIPTION("AMD Overclocking (AOD) WMI driver");
MODULE_LICENSE("GPL");
