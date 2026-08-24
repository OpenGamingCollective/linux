// SPDX-License-Identifier: GPL-2.0+
/*
 * HID driver for the AYANEO 3 detachable controller ("Magic Modules").
 *
 * The AYANEO 3 controller exposes three USB HID interfaces behind
 * VID 0x1c4f PID 0x0002 (a generic SigmaMicro ID, hence the DMI gate):
 * a gamepad, a keyboard for the extra buttons, and a vendor interface
 * (application usage 0xff000001) accepting 65-byte commands.
 *
 * This driver binds the vendor interface and provides:
 *  - module identification (which module type is inserted on each side)
 *  - software eject of the left/right modules
 *  - RGB control of the joystick rings as a multicolor LED class device
 *
 * It complements the ayaneo-ec platform driver, which exposes module
 * attach state and controller power. A full eject is: write to this
 * driver's "eject" attribute, then power the controller off through
 * ayaneo-ec's controller_power once the eject completes.
 *
 * The protocol was reverse engineered in the Handheld Daemon project by
 * Antheas Kapenekakis.
 *
 * Command format (65 bytes, unnumbered report):
 *   [0]   report id (0)
 *   [1:3] little-endian sum of bytes 7..64
 *   [3]   command
 *   [4]   subcommand
 *   [5:]  payload
 * The device replies with a 64-byte report echoing the subcommand at
 * byte 3.
 *
 * Copyright (C) 2026 Matías Martínez <hello@matias.me>
 */

#include <linux/delay.h>
#include <linux/dmi.h>
#include <linux/hid.h>
#include <linux/led-class-multicolor.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/unaligned.h>

#define AYA3_REPORT_SIZE	65
#define AYA3_RESP_SIZE		64
#define AYA3_CMD_TIMEOUT_MS	300
#define AYA3_CMD_ATTEMPTS	3

/* Subcommands (byte 4); byte 3 is 0x00 except for the config command */
#define AYA3_SUBCMD_CHECK	0x08
#define AYA3_CMD_CONFIG		0x21
#define AYA3_SUBCMD_CONFIG	0x09

/* CHECK response fields */
#define AYA3_RESP_CMD		3
#define AYA3_RESP_EJECT_STATUS	19
#define AYA3_RESP_MODULE_LEFT	32
#define AYA3_RESP_MODULE_RIGHT	33
/* Bits that stay set in the eject status byte after an eject completes */
#define AYA3_EJECT_DONE_MASK	0x11

/* Config command eject/reset field */
#define AYA3_EJECT_LEFT		0x07
#define AYA3_EJECT_RIGHT	0x70
#define AYA3_RESET		0x88

/* Config command RGB modes */
#define AYA3_RGB_SOLID		0x01
#define AYA3_RGB_OFF		0xff

#define AYA3_VIBRATION_DEFAULT	0x02	/* medium */

struct aya3 {
	struct hid_device *hdev;
	/* DMA-safe command buffer; guarded by lock */
	u8 *xfer;
	/* Serializes commands and cached-config access */
	struct mutex lock;
	struct completion resp_done;
	u8 resp[AYA3_RESP_SIZE];
	u8 resp_expect;
	bool resp_pending;

	u8 rgb[3];
	u8 vibration;

	struct led_classdev_mc mcled;
	struct mc_subled subleds[3];
};

static int aya3_send(struct aya3 *aya)
{
	int ret;

	ret = hid_hw_output_report(aya->hdev, aya->xfer, AYA3_REPORT_SIZE);
	if (ret == -ENOSYS)
		ret = hid_hw_raw_request(aya->hdev, aya->xfer[0], aya->xfer,
					 AYA3_REPORT_SIZE, HID_OUTPUT_REPORT,
					 HID_REQ_SET_REPORT);
	if (ret < 0)
		return ret;
	return 0;
}

/* Send the command in aya->xfer and wait for the echoing reply. */
static int aya3_cmd(struct aya3 *aya, u8 *resp)
{
	int attempt, ret;

	lockdep_assert_held(&aya->lock);

	for (attempt = 0; attempt < AYA3_CMD_ATTEMPTS; attempt++) {
		reinit_completion(&aya->resp_done);
		aya->resp_expect = aya->xfer[4];
		WRITE_ONCE(aya->resp_pending, true);

		ret = aya3_send(aya);
		if (ret) {
			WRITE_ONCE(aya->resp_pending, false);
			return ret;
		}

		if (wait_for_completion_timeout(&aya->resp_done,
						msecs_to_jiffies(AYA3_CMD_TIMEOUT_MS))) {
			if (resp)
				memcpy(resp, aya->resp, AYA3_RESP_SIZE);
			return 0;
		}
	}
	WRITE_ONCE(aya->resp_pending, false);
	return -ETIMEDOUT;
}

static void aya3_checksum(u8 *buf)
{
	unsigned int sum = 0;
	int i;

	for (i = 7; i < AYA3_REPORT_SIZE; i++)
		sum += buf[i];
	put_unaligned_le16(sum, buf + 1);
}

static int aya3_check(struct aya3 *aya, u8 *resp)
{
	memset(aya->xfer, 0, AYA3_REPORT_SIZE);
	aya->xfer[4] = AYA3_SUBCMD_CHECK;
	return aya3_cmd(aya, resp);
}

/*
 * The config command sets everything at once: RGB for both rings,
 * vibration strength, joystick sensitivity, and the eject/reset field.
 */
static int aya3_send_config(struct aya3 *aya, u8 eject)
{
	u8 *buf = aya->xfer;
	u8 mode = AYA3_RGB_SOLID;

	if (!aya->rgb[0] && !aya->rgb[1] && !aya->rgb[2])
		mode = AYA3_RGB_OFF;

	memset(buf, 0, AYA3_REPORT_SIZE);
	buf[3] = AYA3_CMD_CONFIG;
	buf[4] = AYA3_SUBCMD_CONFIG;
	/* Right ring, then left ring: mode, R, G, B */
	buf[8] = mode;
	memcpy(buf + 9, aya->rgb, 3);
	buf[12] = mode;
	memcpy(buf + 13, aya->rgb, 3);
	buf[20] = eject;
	buf[22] = 0x33;
	buf[23] = 0x22;	/* joystick sensitivity 100%/100% */
	buf[24] = aya->vibration << 4;
	buf[32] = 0x01;
	buf[37] = 0x64;
	buf[38] = 0x64;
	aya3_checksum(buf);

	return aya3_cmd(aya, NULL);
}

static int aya3_raw_event(struct hid_device *hdev, struct hid_report *report,
			  u8 *data, int size)
{
	struct aya3 *aya = hid_get_drvdata(hdev);

	if (!READ_ONCE(aya->resp_pending) || size < AYA3_RESP_SIZE)
		return 0;
	if (data[AYA3_RESP_CMD] != aya->resp_expect)
		return 0;

	memcpy(aya->resp, data, AYA3_RESP_SIZE);
	WRITE_ONCE(aya->resp_pending, false);
	complete(&aya->resp_done);
	return 0;
}

static ssize_t aya3_module_show(struct device *dev, char *buf, int offset)
{
	struct aya3 *aya = dev_get_drvdata(dev);
	u8 resp[AYA3_RESP_SIZE];
	int ret;

	ret = mutex_lock_interruptible(&aya->lock);
	if (ret)
		return ret;
	ret = aya3_check(aya, resp);
	mutex_unlock(&aya->lock);
	if (ret)
		return ret;

	return sysfs_emit(buf, "0x%02x\n", resp[offset]);
}

static ssize_t module_left_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return aya3_module_show(dev, buf, AYA3_RESP_MODULE_LEFT);
}
static DEVICE_ATTR_RO(module_left);

static ssize_t module_right_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return aya3_module_show(dev, buf, AYA3_RESP_MODULE_RIGHT);
}
static DEVICE_ATTR_RO(module_right);

static ssize_t eject_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct aya3 *aya = dev_get_drvdata(dev);
	u8 resp[AYA3_RESP_SIZE];
	u8 eject;
	int ret, i;

	if (sysfs_streq(buf, "left"))
		eject = AYA3_EJECT_LEFT;
	else if (sysfs_streq(buf, "right"))
		eject = AYA3_EJECT_RIGHT;
	else if (sysfs_streq(buf, "both"))
		eject = AYA3_EJECT_LEFT | AYA3_EJECT_RIGHT;
	else
		return -EINVAL;

	ret = mutex_lock_interruptible(&aya->lock);
	if (ret)
		return ret;

	ret = aya3_send_config(aya, eject);
	if (ret)
		goto out;

	/*
	 * Wait for the firmware to report the eject as done. Userspace
	 * must then cut power through ayaneo-ec's controller_power for
	 * the module to be physically released.
	 */
	ret = -ETIMEDOUT;
	for (i = 0; i < 20; i++) {
		msleep(400);
		if (aya3_check(aya, resp))
			continue;
		if (!(resp[AYA3_RESP_EJECT_STATUS] & ~AYA3_EJECT_DONE_MASK)) {
			ret = 0;
			break;
		}
	}
out:
	mutex_unlock(&aya->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(eject);

static ssize_t reset_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct aya3 *aya = dev_get_drvdata(dev);
	bool value;
	int ret;

	ret = kstrtobool(buf, &value);
	if (ret)
		return ret;
	if (!value)
		return count;

	ret = mutex_lock_interruptible(&aya->lock);
	if (ret)
		return ret;
	ret = aya3_send_config(aya, AYA3_RESET);
	if (!ret) {
		msleep(500);
		ret = aya3_send_config(aya, 0);
	}
	mutex_unlock(&aya->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(reset);

static struct attribute *aya3_attrs[] = {
	&dev_attr_module_left.attr,
	&dev_attr_module_right.attr,
	&dev_attr_eject.attr,
	&dev_attr_reset.attr,
	NULL
};
ATTRIBUTE_GROUPS(aya3);

static int aya3_led_set(struct led_classdev *cdev, enum led_brightness value)
{
	struct led_classdev_mc *mc = lcdev_to_mccdev(cdev);
	struct aya3 *aya = container_of(mc, struct aya3, mcled);
	int ret, i;

	ret = mutex_lock_interruptible(&aya->lock);
	if (ret)
		return ret;

	led_mc_calc_color_components(mc, value);
	for (i = 0; i < 3; i++)
		aya->rgb[i] = min_t(unsigned int, aya->subleds[i].brightness, 255);

	ret = aya3_send_config(aya, 0);
	mutex_unlock(&aya->lock);
	return ret;
}

static int aya3_register_led(struct aya3 *aya)
{
	struct led_classdev *cdev = &aya->mcled.led_cdev;

	aya->subleds[0].color_index = LED_COLOR_ID_RED;
	aya->subleds[1].color_index = LED_COLOR_ID_GREEN;
	aya->subleds[2].color_index = LED_COLOR_ID_BLUE;
	aya->mcled.subled_info = aya->subleds;
	aya->mcled.num_colors = 3;

	cdev->name = "ayaneo:rgb:joystick_rings";
	cdev->brightness = 0;
	cdev->max_brightness = 255;
	cdev->brightness_set_blocking = aya3_led_set;

	return devm_led_classdev_multicolor_register(&aya->hdev->dev,
						     &aya->mcled);
}

static const struct dmi_system_id aya3_dmi_table[] = {
	{
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "AYANEO"),
			DMI_MATCH(DMI_BOARD_NAME, "AYANEO 3"),
		},
	},
	{}
};

static int aya3_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct aya3 *aya;
	u8 resp[AYA3_RESP_SIZE];
	int ret;

	/* The VID/PID is a generic SigmaMicro ID; bind on AYANEO 3 only */
	if (!dmi_check_system(aya3_dmi_table))
		return -ENODEV;

	ret = hid_parse(hdev);
	if (ret)
		return ret;

	/* Bind only the vendor interface, not the gamepad/keyboard ones */
	if (!hid_is_usb(hdev) ||
	    hdev->collection->usage != (HID_UP_MSVENDOR | 0x0001))
		return -ENODEV;

	aya = devm_kzalloc(&hdev->dev, sizeof(*aya), GFP_KERNEL);
	if (!aya)
		return -ENOMEM;

	aya->xfer = devm_kzalloc(&hdev->dev, AYA3_REPORT_SIZE, GFP_KERNEL);
	if (!aya->xfer)
		return -ENOMEM;

	aya->hdev = hdev;
	aya->vibration = AYA3_VIBRATION_DEFAULT;
	init_completion(&aya->resp_done);
	ret = devm_mutex_init(&hdev->dev, &aya->lock);
	if (ret)
		return ret;
	hid_set_drvdata(hdev, aya);

	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (ret)
		return ret;

	ret = hid_hw_open(hdev);
	if (ret)
		goto err_stop;

	/* Input reports are not delivered during probe by default */
	hid_device_io_start(hdev);

	mutex_lock(&aya->lock);
	ret = aya3_check(aya, resp);
	mutex_unlock(&aya->lock);
	if (ret)
		hid_warn(hdev, "controller did not answer status check: %d\n",
			 ret);
	else
		hid_info(hdev, "modules: left 0x%02x right 0x%02x\n",
			 resp[AYA3_RESP_MODULE_LEFT],
			 resp[AYA3_RESP_MODULE_RIGHT]);

	ret = aya3_register_led(aya);
	if (ret)
		goto err_close;

	return 0;

err_close:
	hid_hw_close(hdev);
err_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void aya3_remove(struct hid_device *hdev)
{
	hid_hw_close(hdev);
	hid_hw_stop(hdev);
}

static const struct hid_device_id aya3_devices[] = {
	{ HID_USB_DEVICE(0x1c4f, 0x0002) },
	{}
};
MODULE_DEVICE_TABLE(hid, aya3_devices);

static struct hid_driver aya3_driver = {
	.name = "hid-ayaneo",
	.id_table = aya3_devices,
	.probe = aya3_probe,
	.remove = aya3_remove,
	.raw_event = aya3_raw_event,
	.driver = {
		.dev_groups = aya3_groups,
	},
};
module_hid_driver(aya3_driver);

MODULE_AUTHOR("Matías Martínez <hello@matias.me>");
MODULE_DESCRIPTION("AYANEO 3 detachable controller driver");
MODULE_LICENSE("GPL");
