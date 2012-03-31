/*
 * Driver model for pansws and pansw triggers
 *
 * Copyright (C) 2009 Jinwoo Cha <jwcha@pantech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __LINUX_PANSW_H_
#define __LINUX_PANSW_H_

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/rwsem.h>
#include <linux/hrtimer.h>

struct device;
/*
 * PANTECH Switch Core
 */

#define PANSW_ON    1
#define PANSW_OFF   0

struct pansw_classdev {
    const char      *name;
    int         enable;
    int         flags;

#define PANSW_SUSPENDED (1 << 0)

    /* Set LED brightness level */
    void        (*enable_set)(struct pansw_classdev *pansw_cdev,
                      int enable);

    struct device       *dev;
    struct list_head     node;          /* Pantech Switch Device list */
};

extern int pansw_classdev_register(struct device *parent,
                 struct pansw_classdev *pansw_cdev);
extern void __pansw_classdev_unregister(struct pansw_classdev *pansw_cdev, bool sus);
static inline void pansw_classdev_unregister(struct pansw_classdev *pansw)
{
    __pansw_classdev_unregister(pansw, false);
}
static inline void pansw_classdev_unregister_suspended(struct pansw_classdev *pansw)
{
    __pansw_classdev_unregister(pansw, true);
}

#endif      /* __LINUX_PANSW_H_ */

