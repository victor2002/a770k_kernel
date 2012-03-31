/* drivers/usb/function/whcm.c
 *
 * SKT OEM(DEVGURU) Function Driver
 * 2009.08.14
 * kim.sanghoun@pantech.com
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/device.h>

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>

struct whcm_cch_func_descriptor {
	__u8 	bLength;
	__u8	bDescriptorType;
	__u8	bDescriptorSubType;
	__u16	bcdCDC;
} __attribute__ ((packed));

struct whcm_func_descriptor {
	__u8 	bLength;
	__u8	bDescriptorType;
	__u8	bDescriptorSubType;
	__u16	bcdVersion;
} __attribute__ ((packed));

struct whcm_ccu_func_descriptor {
	__u8 	bLength;
	__u8	bDescriptorType;
	__u8	bDescriptorSubType;
	__u8	bControlInterface;
	__u8	bSubordinateInterface0;
	__u8	bSubordinateInterface1;
	__u8	bSubordinateInterface2;
} __attribute__ ((packed));

static struct usb_interface_descriptor whcm_intf_desc = {
	.bLength            =	0x09,
	.bDescriptorType    =	USB_DT_INTERFACE,
	.bInterfaceNumber		= 0x00,
	.bAlternateSetting	= 0x00,
	.bNumEndpoints      =	0,
	.bInterfaceClass    =	0x02,
	.bInterfaceSubClass =	0x08,
	.bInterfaceProtocol =	0x00,
	.iInterface					= 0x00,//0x04
};

#ifdef DONT_ENUMERATE 
static struct whcm_cch_func_descriptor whcm_cch_func_desc = {
	.bLength 						= 0x05,
	.bDescriptorType 		= 0x24,
	.bDescriptorSubType	= 0x00,
	.bcdCDC							= 0x0110,
}; 

static struct whcm_func_descriptor whcm_func_desc = {
	.bLength						= 0x05,
	.bDescriptorType		= 0x24,
	.bDescriptorSubType	= 0x11,
	.bcdVersion					= 0x0110,
};

static struct whcm_ccu_func_descriptor whcm_ccu_func_desc = {
	.bLength								= 0x07,
	.bDescriptorType				= 0x24,
	.bDescriptorSubType			= 0x06,
	.bControlInterface			= 0x00,
	.bSubordinateInterface0	= 0x00,
	.bSubordinateInterface1	= 0x02,
	.bSubordinateInterface2	= 0x08,
};
#endif

struct whcm_dev {
	struct usb_function function;
 	struct usb_composite_dev *cdev;

	unsigned bound;
};

static struct whcm_dev *_whcm_dev;

static struct usb_descriptor_header *fs_whcm_descs[] = {
  (struct usb_descriptor_header *) &whcm_intf_desc,
#ifdef DONT_ENUMERATE 
  (struct usb_descriptor_header *) &whcm_cch_func_desc,
  (struct usb_descriptor_header *) &whcm_func_desc,
  (struct usb_descriptor_header *) &whcm_ccu_func_desc,
#endif
  NULL,
};

static struct usb_descriptor_header *hs_whcm_descs[] = {
  (struct usb_descriptor_header *) &whcm_intf_desc,
#ifdef DONT_ENUMERATE 
  (struct usb_descriptor_header *) &whcm_cch_func_desc,
  (struct usb_descriptor_header *) &whcm_func_desc,
  (struct usb_descriptor_header *) &whcm_ccu_func_desc,
#endif
  NULL,
};

static inline struct whcm_dev *func_to_dev(struct usb_function *f)
{
  return container_of(f, struct whcm_dev, function); 
} 

static int whcm_function_bind(struct usb_configuration *c, struct usb_function *f)
{
  struct usb_composite_dev *cdev = c->cdev;
  struct whcm_dev  *dev = func_to_dev(f);
  int     id;

  dev->cdev = cdev;

  /* allocate interface ID(s) */
  id = usb_interface_id(c, f);
  if (id < 0)
    return id;
  whcm_intf_desc.bInterfaceNumber = id;

	dev->bound = 1;
  return 0;

}


static void whcm_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
  struct whcm_dev  *dev = func_to_dev(f);

  kfree(_whcm_dev);
  _whcm_dev = NULL;

	dev->bound = 0;

	return;
}

static int whcm_function_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	//no action
  return 0;
}

static void whcm_function_disable(struct usb_function *f)
{
	//no action
}


int pantech_whcm_function_add(struct usb_composite_dev *cdev, struct usb_configuration *c)
{
  struct whcm_dev *dev;
  int ret;

  printk(KERN_ERR "whcm_function_add\n");

  dev = kzalloc(sizeof(*dev), GFP_KERNEL);
  if (!dev)
    return -ENOMEM;

	dev->cdev = cdev;
	dev->function.name = "whcm";
	dev->function.descriptors = fs_whcm_descs;
	dev->function.hs_descriptors = hs_whcm_descs;
	dev->function.bind = whcm_function_bind;
	dev->function.unbind = whcm_function_unbind;
	dev->function.set_alt = whcm_function_set_alt;
  dev->function.disable = whcm_function_disable;

	_whcm_dev = dev;
	
	ret = usb_add_function(c, &dev->function);
	if(ret) {
		kfree(dev);
		printk(KERN_ERR "whcm gadget function failed to initialize\n");
	}
	
	return ret;
}

