/*
 * PANTECH Switch Class Core
 *
 * Copyright (C) 2009 Jinwoo Cha <jwcha@pantech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/pansw.h>

static struct class *pansw_class;

static void pansw_set_enable(struct pansw_classdev *pansw_cdev, int value)
{
    printk(KERN_ERR "%s: value=%d\n", __func__, value);

    pansw_cdev->enable = value;

    pansw_cdev->enable_set(pansw_cdev, value);
}

static ssize_t pansw_enable_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct pansw_classdev *pansw_cdev = dev_get_drvdata(dev);

    /* no lock needed for this */
    return sprintf(buf, "%u\n", pansw_cdev->enable);
}

static ssize_t pansw_enable_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    struct pansw_classdev *pansw_cdev = dev_get_drvdata(dev);
    ssize_t ret = -EINVAL;
    char *after;
    unsigned long state = simple_strtoul(buf, &after, 10);
    size_t count = after - buf;

    if (*after && isspace(*after))
        count++;

    if (count == size) {
        printk(KERN_ERR "%s: state=%d, size=%d\n", __func__, state, size);
        ret = count;
        pansw_set_enable(pansw_cdev, state);
    }

    return  ret;
}

/* CTS - FAIL */
static DEVICE_ATTR(enable, 0660, pansw_enable_show, pansw_enable_store);

/**
 * pansw_classdev_suspend - suspend an pansw_classdev.
 * @pansw_cdev: the pansw_classdev to suspend.
 */
void pansw_classdev_suspend(struct pansw_classdev *pansw_cdev)
{
}
EXPORT_SYMBOL_GPL(pansw_classdev_suspend);

/**
 * pansw_classdev_resume - resume an pansw_classdev.
 * @pansw_cdev: the pansw_classdev to resume.
 */
void pansw_classdev_resume(struct pansw_classdev *pansw_cdev)
{
}
EXPORT_SYMBOL_GPL(pansw_classdev_resume);

/**
 * pansw_classdev_register - register a new object of pansw_classdev class.
 * @dev: The device to register.
 * @pansw_cdev: the pansw_classdev structure for this device.
 */
int pansw_classdev_register(struct device *parent, struct pansw_classdev *pansw_cdev)
{
    int rc;

    pansw_cdev->dev = device_create(pansw_class, parent, 0, "%s",
                        pansw_cdev->name);
    if (unlikely(IS_ERR(pansw_cdev->dev)))
        return PTR_ERR(pansw_cdev->dev);

    dev_set_drvdata(pansw_cdev->dev, pansw_cdev);

    /* register the attributes */
    rc = device_create_file(pansw_cdev->dev, &dev_attr_enable);
    if (rc)
        goto err_out;

    printk(KERN_INFO "Registered pansw device: %s\n",
            pansw_cdev->name);

    return 0;

err_out2:
    device_remove_file(pansw_cdev->dev, &dev_attr_enable);
err_out:
    device_unregister(pansw_cdev->dev);
    return rc;
}
EXPORT_SYMBOL_GPL(pansw_classdev_register);

/**
 * __pansw_classdev_unregister - unregisters a object of pansw_properties class.
 * @pansw_cdev: the pansw device to unregister
 * @suspended: indicates whether system-wide suspend or resume is in progress
 *
 * Unregisters a previously registered via pansw_classdev_register object.
 */
void __pansw_classdev_unregister(struct pansw_classdev *pansw_cdev,
                      bool suspended)
{
    device_remove_file(pansw_cdev->dev, &dev_attr_enable);

    device_unregister(pansw_cdev->dev);
}
EXPORT_SYMBOL_GPL(__pansw_classdev_unregister);

static int __init pansw_init(void)
{
    pansw_class = class_create(THIS_MODULE, "pan-switch");
    if (IS_ERR(pansw_class))
        return PTR_ERR(pansw_class);
    return 0;
}

static void __exit pansw_exit(void)
{
    class_destroy(pansw_class);
}

subsys_initcall(pansw_init);
module_exit(pansw_exit);

MODULE_AUTHOR("Jinwoo Cha <jwcha@pantech.com>");
MODULE_DESCRIPTION("PANTECH Switch Class Interface");
MODULE_LICENSE("GPL");


