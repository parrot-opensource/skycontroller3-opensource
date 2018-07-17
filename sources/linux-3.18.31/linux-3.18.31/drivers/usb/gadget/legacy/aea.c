/*
 * eae.c -- Apple External Accessory driver
 *
 * Copyright (C) 2008 David Brownell
 * Copyright (C) 2008 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/module.h>
#include <linux/usb/composite.h>

#include "u_serial.h"

#define DRIVER_DESC		"Apple External Accessory iAP2"

/*-------------------------------------------------------------------------*/

/* DO NOT REUSE THESE IDs with a protocol-incompatible driver!!  Ever!!
 * Instead:  allocate your own, using normal USB-IF procedures.
 */

/* Thanks to NetChip Technologies for donating this product ID.
 * It's for devices with only this composite CDC configuration.
 */
#define CDC_VENDOR_NUM		0x19cf	/* Parrot SA */
#define CDC_PRODUCT_NUM		0xa4aa	/* CDC Composite: ECM + ACM */

/*-------------------------------------------------------------------------*/

#define USE_SERIAL_IAP	1 // define for f_serial.c

#define IAP_INTERFACE	"iAP Interface"

/*-------------------------------------------------------------------------*/

#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>


/*-------------------------------------------------------------------------*/

enum {
	TTY_PORT_GSER,
	TTY_PORT_AEA,
	TTY_PORTS_MAX,
};

struct aea_serial_function_config {
	struct usb_function *f_usbgfunc[TTY_PORTS_MAX];
	struct usb_function_instance *f_usbginst[TTY_PORTS_MAX];
} aea_config;


USB_GADGET_COMPOSITE_OPTIONS();

static struct usb_device_descriptor device_desc = {
	.bLength =		sizeof device_desc,
	.bDescriptorType =	USB_DT_DEVICE,
	.bcdUSB =		cpu_to_le16(0x0200),
	.bDeviceClass =		0x00,
	.bDeviceSubClass =	0x00,
	.bDeviceProtocol =	0,
	/* .bMaxPacketSize0 = f(hardware) */

	/* Vendor and product id can be overridden by module parameters.  */
	.idVendor =		cpu_to_le16(CDC_VENDOR_NUM),
	.idProduct =		cpu_to_le16(CDC_PRODUCT_NUM),
	/* .bcdDevice = f(hardware) */
	/* .iManufacturer = DYNAMIC */
	/* .iProduct = DYNAMIC */
	/* NO SERIAL NUMBER */
	.bNumConfigurations =	1,
};

static struct usb_otg_descriptor otg_descriptor = {
	.bLength =		sizeof otg_descriptor,
	.bDescriptorType =	USB_DT_OTG,

	/* REVISIT SRP-only hardware is possible, although
	 * it would not be called "OTG" ...
	 */
	.bmAttributes =		USB_OTG_SRP | USB_OTG_HNP,
};

static const struct usb_descriptor_header *otg_desc[] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	NULL,
};

/* string IDs are assigned dynamically */
static struct usb_string strings_dev[] = {
	[USB_GADGET_MANUFACTURER_IDX].s = "Parrot",
	[USB_GADGET_PRODUCT_IDX].s = "Generic",
	[USB_GADGET_SERIAL_IDX].s = "",
	{  } /* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_configuration aea_config_driver = {
	.label			= "Apple External Accessory",
	.bConfigurationValue	= 1,
	/* .iConfiguration = DYNAMIC */
	.bmAttributes		= USB_CONFIG_ATT_SELFPOWER,
};

static int __init aea_bind(struct usb_composite_dev *cdev)
{
	struct usb_gadget *gadget = cdev->gadget;
	int status, i;
	char *functions[TTY_PORTS_MAX] = { "gser", "gser_aea"};

	/* update gser descriptor and string */
	gser_interface_desc.bInterfaceSubClass = 0xf0;
	gser_string_defs[0].s = IAP_INTERFACE;

	device_desc.bcdDevice = cpu_to_le16(0x0300);

	status = usb_string_id(cdev);
	if (status < 0)
		goto fail;

	strings_dev[USB_GADGET_MANUFACTURER_IDX].id = status;
	device_desc.iManufacturer = status;

	status = usb_string_id(cdev);
	if (status < 0)
		goto fail;

	strings_dev[USB_GADGET_PRODUCT_IDX].id = status;
	device_desc.iProduct = status;

	status = usb_string_id(cdev);
	if (status < 0)
		goto fail;

	strings_dev[USB_GADGET_SERIAL_IDX].id = status;

	status = usb_add_config_only(cdev, &aea_config_driver);
	if (status)
		goto fail;

	if (gadget_is_otg(cdev->gadget)) {
		aea_config_driver.descriptors = otg_desc;
		aea_config_driver.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}

	/**
	 * set up serial link layer
	 * 2 tty must be created: one for iap communication,
	 * another for app communication.
	 */
	for (i = 0; i < ARRAY_SIZE(functions); i++) {

		dev_dbg(&gadget->dev, "usbfunc:%s\n", functions[i]);

		aea_config.f_usbginst[i] =
			usb_get_function_instance(functions[i]);
		if (IS_ERR(aea_config.f_usbginst[i])) {
			status = PTR_ERR(aea_config.f_usbginst[i]);
			goto fail_usb_functions;
		}

		aea_config.f_usbgfunc[i] =
			usb_get_function(aea_config.f_usbginst[i]);

		if (IS_ERR(aea_config.f_usbgfunc[i])) {
			status = PTR_ERR(aea_config.f_usbgfunc[i]);
			goto fail_usb_get_function;
		}

		status = usb_add_function(&aea_config_driver,
				aea_config.f_usbgfunc[i]);
		if (status)
			goto fail_usb_add_function;
	}

	usb_composite_overwrite_options(cdev, &coverwrite);
	dev_info(&gadget->dev, "%s\n", DRIVER_DESC);
	return 0;

fail_usb_add_function:
	usb_put_function(aea_config.f_usbgfunc[i]);
fail_usb_get_function:
	usb_put_function_instance(aea_config.f_usbginst[i]);
fail_usb_functions:
	i--;
	while(i >= 0) {
		usb_remove_function(&aea_config_driver,
					aea_config.f_usbgfunc[i]);
		usb_put_function(aea_config.f_usbgfunc[i]);
		usb_put_function_instance(aea_config.f_usbginst[i--]);
	}
fail:
	return status;
}

static int aea_unbind(struct usb_composite_dev *cdev)
{
	int i;

	for (i = 0; i < TTY_PORTS_MAX; i++) {
		usb_put_function(aea_config.f_usbgfunc[i]);
		usb_put_function_instance(aea_config.f_usbginst[i]);
	}
	return 0;
}

static __refdata struct usb_composite_driver aea_driver = {
	.name		= "g_aea",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.max_speed	= USB_SPEED_HIGH,
	.bind		= aea_bind,
	.unbind		= aea_unbind,
};

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("David Brownell");
MODULE_LICENSE("GPL");

/*-------------------------------------------------------------------------*/
static int __init init(void)
{
	return usb_composite_probe(&aea_driver);
}
module_init(init);

static void __exit cleanup(void)
{
	usb_composite_unregister(&aea_driver);
}
module_exit(cleanup);
