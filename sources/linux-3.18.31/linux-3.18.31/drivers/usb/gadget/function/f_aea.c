/*
 * aea_serial.c
 * based on f_serial.c - generic USB serial function driver
 *
 * Copyright (C) 2003 Al Borchers (alborchers@steinerpoint.com)
 * Copyright (C) 2008 by David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 *
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * either version 2 of that License or (at your option) any later version.
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>

#include "u_serial.h"
#include "gadget_chips.h"

/*
 * This function packages a simple "generic serial" port with no real
 * control mechanisms, just raw data transfer over two bulk endpoints.
 *
 * Because it's not standardized, this isn't as interoperable as the
 * CDC ACM driver.  However, for many purposes it's just as functional
 * if you can arrange appropriate host side drivers.
 */
#define MAX_PROTOCOL_LENGTH	60
static char protocol[MAX_PROTOCOL_LENGTH] = "com.unknown.protocol";
module_param_string(app_protocol, protocol,
			MAX_PROTOCOL_LENGTH, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(app_protocol, "Application protocol");

struct f_gser_aea {
	struct gserial port;
	struct usb_composite_dev *cdev;
	u8 data_id;
	u8 port_num;
};

static inline struct f_gser_aea *func_to_gser_aea(struct usb_function *f)
{
	return container_of(f, struct f_gser_aea, port.func);
}

/*-------------------------------------------------------------------------*/

/* interface descriptor: */

static struct usb_interface_descriptor gser_aea_interface_desc  = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass =	0xf0,
	.bInterfaceProtocol =	1,
	.bAlternateSetting =	1,
	/* .iInterface = DYNAMIC */
};

static struct usb_interface_descriptor gser_aea_interface_nop_desc  = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints =	0,
	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass =	0xf0,
	.bInterfaceProtocol =	1,
	.bAlternateSetting =	0,
	/* .iInterface = DYNAMIC */
};

/* full speed support: */

static struct usb_endpoint_descriptor gser_aea_fs_in_desc  = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor gser_aea_fs_out_desc  = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *gser_aea_fs_function[]  = {
	(struct usb_descriptor_header *) &gser_aea_interface_desc,
	(struct usb_descriptor_header *) &gser_aea_fs_in_desc,
	(struct usb_descriptor_header *) &gser_aea_fs_out_desc,
	(struct usb_descriptor_header *) &gser_aea_interface_nop_desc,
	NULL,
};

/* high speed support: */

static struct usb_endpoint_descriptor gser_aea_hs_in_desc  = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_endpoint_descriptor gser_aea_hs_out_desc  = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_descriptor_header *gser_aea_hs_function[]  = {
	(struct usb_descriptor_header *) &gser_aea_interface_desc,
	(struct usb_descriptor_header *) &gser_aea_hs_in_desc,
	(struct usb_descriptor_header *) &gser_aea_hs_out_desc,
	(struct usb_descriptor_header *) &gser_aea_interface_nop_desc,
	NULL,
};

static struct usb_endpoint_descriptor gser_aea_ss_in_desc  = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_endpoint_descriptor gser_aea_ss_out_desc  = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor gser_aea_ss_bulk_comp_desc  = {
	.bLength =              sizeof gser_aea_ss_bulk_comp_desc,
	.bDescriptorType =      USB_DT_SS_ENDPOINT_COMP,
};

static struct usb_descriptor_header *gser_aea_ss_function[]  = {
	(struct usb_descriptor_header *) &gser_aea_interface_desc,
	(struct usb_descriptor_header *) &gser_aea_ss_in_desc,
	(struct usb_descriptor_header *) &gser_aea_ss_bulk_comp_desc,
	(struct usb_descriptor_header *) &gser_aea_ss_out_desc,
	(struct usb_descriptor_header *) &gser_aea_ss_bulk_comp_desc,
	NULL,
};

/* string descriptors: */
static struct usb_string gser_aea_string_defs[] = {
	[0].s = NULL,
	[1].s = NULL,
	{  } /* end of list */
};

static struct usb_gadget_strings gser_aea_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		gser_aea_string_defs,
};

static struct usb_gadget_strings *gser_aea_strings[] = {
	&gser_aea_string_table,
	NULL,
};

/*-------------------------------------------------------------------------*/
static int config_tty_device_ep(struct usb_function *usb_function,
				struct usb_composite_dev *cdev)
{
	struct f_gser_aea *gser_aea = func_to_gser_aea(usb_function);
	struct usb_ep *ep_in = gser_aea->port.in;
	struct usb_ep *ep_out = gser_aea->port.out;
	u8 tty_index = gser_aea->port_num;

	/* USB EndPoints already configured, nothing to do */
	if (ep_in->desc && ep_out->desc)
		return 0;

	DBG(cdev, "configure usb EndPoints for ttyGS%d\n", tty_index);

	if (config_ep_by_speed(cdev->gadget, usb_function, ep_in) ||
	    config_ep_by_speed(cdev->gadget, usb_function, ep_out)) {
		DBG(cdev, "Cannot configure usb EndPoints [ttyGS%d]\n",
		    tty_index);

		ep_in->desc = NULL;
		ep_out->desc = NULL;

		return -EINVAL;
	}

	return 0;
}

static int gser_aea_set_alt(struct usb_function *usb_function,
				 unsigned interface, unsigned alt)
{
	struct f_gser_aea *gser_aea = func_to_gser_aea(usb_function);
	struct usb_composite_dev *cdev = usb_function->config->cdev;

	DBG(cdev, "gser_aea_set_alt [interface=%u] [alt=%u]\n",
	    interface, alt);

	/* if alt == 0: apple device uses the dummy interface
	 *    so we disconnect ttyGS1
	 * if alt == 1: apple device uses the data interface
	 *    so we configure usb EP and connect ttyGS1 */
	if ((gser_aea->port.in->driver_data)
	   || (gser_aea->port.out->driver_data))
		gserial_disconnect(&gser_aea->port);

	if (alt == 1) {
		if (config_tty_device_ep(usb_function, cdev) != 0)
			return -EINVAL;

		gser_aea->port.in->driver_data = cdev;
		gser_aea->port.out->driver_data = cdev;

		gserial_connect(&gser_aea->port, gser_aea->port_num);
	}

	return 0;
}

static void gser_aea_disable(struct usb_function *usb_function)
{
	struct f_gser_aea *gser_aea = func_to_gser_aea(usb_function);
	pr_debug("%s()\n", __func__);

	if ((gser_aea->port.in->driver_data)
	   || (gser_aea->port.out->driver_data))
		gserial_disconnect(&gser_aea->port);
}

/*-------------------------------------------------------------------------*/

/* serial function driver setup/binding */

static int
gser_aea_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_gser_aea	*gser_aea = func_to_gser_aea(f);
	int			status;
	struct usb_ep		*ep;

	/* allocate instance-specific interface IDs */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	gser_aea->data_id = status;
	gser_aea_interface_desc.bInterfaceNumber = status;
	gser_aea_interface_nop_desc.bInterfaceNumber = status;

	status = -ENODEV;

	/* data interface label */
	gser_aea_string_defs[0].s = protocol;
	gser_aea_string_defs[1].s = protocol;

	/* same as f_serial.c */
	if (gser_aea_string_defs[0].id == 0) {
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		gser_aea_string_defs[0].id = status;
		gser_aea_string_defs[1].id = status;
	}

	gser_aea_interface_desc.iInterface = gser_aea_string_defs[0].id;
	gser_aea_interface_nop_desc.iInterface = gser_aea_string_defs[1].id;

	/* allocate instance-specific endpoints */
	ep = usb_ep_autoconfig(cdev->gadget, &gser_aea_fs_in_desc);
	if (!ep)
		goto fail;
	gser_aea->port.in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, &gser_aea_fs_out_desc);
	if (!ep)
		goto fail;
	gser_aea->port.out = ep;
	gser_aea->cdev = cdev;

	/* copy descriptors, and track endpoint copies */
	status = -ENOMEM;

	/* support all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		gser_aea_hs_in_desc.bEndpointAddress =
				gser_aea_fs_in_desc.bEndpointAddress;
		gser_aea_hs_out_desc.bEndpointAddress =
				gser_aea_fs_out_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->hs_descriptors = usb_copy_descriptors(gser_aea_hs_function);
		if (!f->hs_descriptors)
			goto fail;
	} else if (gadget_is_superspeed(c->cdev->gadget)) {
		gser_aea_ss_in_desc.bEndpointAddress =
			gser_aea_fs_in_desc.bEndpointAddress;
		gser_aea_ss_out_desc.bEndpointAddress =
			gser_aea_fs_out_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->ss_descriptors = usb_copy_descriptors(gser_aea_ss_function);
		if (!f->ss_descriptors)
			goto fail;
	} else {
		f->fs_descriptors = usb_copy_descriptors(gser_aea_fs_function);
		if (!f->fs_descriptors)
			goto fail;
	}

	status = 0;
fail:
	if (!status)
		INFO(cdev, "Apple External Accessory ttyGS%d: %s speed IN/%s OUT/%s\n",
			gser_aea->port_num,
			gadget_is_superspeed(cdev->gadget) ? "super" :
			gadget_is_dualspeed(cdev->gadget) ? "dual" : "full",
			gser_aea->port.in->name, gser_aea->port.out->name);
	else {
		if (gadget_is_dualspeed(c->cdev->gadget))
			usb_free_descriptors(f->hs_descriptors);
		if (gadget_is_superspeed(c->cdev->gadget))
			usb_free_descriptors(f->ss_descriptors);
		if (f->fs_descriptors)
			usb_free_descriptors(f->fs_descriptors);

		ERROR(cdev, "%s: can't bind, err %d\n", f->name, status);
	}
	return status;
}

static void
gser_aea_unbind(struct usb_configuration *c, struct usb_function *f)
{
	pr_debug("%s()\n", __func__);

	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	if (gadget_is_superspeed(c->cdev->gadget))
		usb_free_descriptors(f->ss_descriptors);
	if (f->fs_descriptors)
		usb_free_descriptors(f->fs_descriptors);
}

static void gser_aea_free_func(struct usb_function *f)
{
	struct f_gser_aea	*gser_aea;
	pr_debug("%s()\n", __func__);

	gser_aea = container_of(f, struct f_gser_aea, port.func);
	if (!gser_aea)
		return;

	kfree(gser_aea);
}

static inline struct f_serial_opts *to_f_serial_opts(struct config_item *item)
{
	return container_of(to_config_group(item), struct f_serial_opts,
			    func_inst.group);
}

CONFIGFS_ATTR_STRUCT(f_serial_opts);
CONFIGFS_ATTR_OPS(f_serial_opts);

static void serial_attr_release(struct config_item *item)
{
	struct f_serial_opts *opts = to_f_serial_opts(item);

	usb_put_function_instance(&opts->func_inst);
}

static struct configfs_item_operations serial_item_ops = {
	.release	 = serial_attr_release,
	.show_attribute  = f_serial_opts_attr_show,
	.store_attribute = f_serial_opts_attr_store,
};

static ssize_t store_app_protocol(struct f_serial_opts *opts,
					const char *buf, size_t count)
{
	size_t len;

	strlcpy(protocol, buf, MAX_PROTOCOL_LENGTH);
	len = strnlen(protocol, MAX_PROTOCOL_LENGTH);

	/* Remove trailing newline */
	if (protocol[len - 1] == '\n')
		protocol[len - 1] = '\0';

	return strnlen(buf, count);
}

static ssize_t show_app_protocol(struct f_serial_opts *opts, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", protocol);
}

static struct f_serial_opts_attribute f_aea_app_protocol =
	__CONFIGFS_ATTR(app_protocol, S_IRUGO | S_IWUSR,
			show_app_protocol, store_app_protocol);

static ssize_t show_port_num(struct f_serial_opts *opts, char *page)
{
	if (!opts)
		return -EINVAL;
	return sprintf(page, "%u\n", opts->port_num);
}

static struct f_serial_opts_attribute f_serial_port_num =
	__CONFIGFS_ATTR_RO(port_num, show_port_num);

static struct configfs_attribute *aea_attrs[] = {
	&f_serial_port_num.attr,
	&f_aea_app_protocol.attr,
	NULL,
};

static struct config_item_type serial_func_type = {
	.ct_item_ops	= &serial_item_ops,
	.ct_attrs	= aea_attrs,
	.ct_owner	= THIS_MODULE,
};

static void gser_aea_free_instance(struct usb_function_instance *fi)
{
	struct f_serial_opts *opts;

	pr_debug("%s()\n", __func__);
	opts = container_of(fi, struct f_serial_opts, func_inst);

	if (!opts)
		return;

	gserial_free_line(opts->port_num);
	kfree(opts);
}

static struct usb_function_instance *gser_aea_alloc_inst(void)
{
	struct f_serial_opts *opts;
	int ret;

	pr_debug("%s()\n", __func__);
	opts = kzalloc(sizeof(*opts), GFP_KERNEL);
	if (!opts)
		return ERR_PTR(-ENOMEM);
	opts->func_inst.free_func_inst = gser_aea_free_instance;
	ret = gserial_alloc_line(&opts->port_num);
	if (ret) {
		kfree(opts);
		return ERR_PTR(ret);
	}

	config_group_init_type_name(&opts->func_inst.group, "",
				    &serial_func_type);

	return &opts->func_inst;
}

static struct usb_function *
		gser_aea_alloc_func(struct usb_function_instance *fi)
{
	struct f_serial_opts	*opts;
	struct f_gser_aea	*gser_aea;

	/* allocate and initialize one new instance */
	gser_aea = kzalloc(sizeof (*gser_aea), GFP_KERNEL);
	if (!gser_aea)
		return ERR_PTR(-ENOMEM);

	opts = container_of(fi, struct f_serial_opts, func_inst);
	if (!opts)
		return ERR_PTR(-EINVAL);

	gser_aea->port_num = opts->port_num;
	gser_aea->port.func.name = "gser_aea";
	gser_aea->port.func.strings = gser_aea_strings;
	gser_aea->port.func.bind = gser_aea_bind;
	gser_aea->port.func.unbind = gser_aea_unbind;
	gser_aea->port.func.set_alt = gser_aea_set_alt;
	gser_aea->port.func.disable = gser_aea_disable;
	gser_aea->port.func.free_func = gser_aea_free_func;

	pr_debug("app protocol:'%s'\n", protocol);
	return &gser_aea->port.func;
}

DECLARE_USB_FUNCTION_INIT(gser_aea, gser_aea_alloc_inst, gser_aea_alloc_func);
MODULE_LICENSE("GPL");
