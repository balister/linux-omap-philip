/*
 * Driver for eding/writing to FPGA via SPI for SDR work.
 *
 * Copyright (C) Philip Balister <philip@opensdr.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "linux/fs.h"
#include "linux/module.h"
#include "linux/cdev.h"
#include "linux/device.h"

struct spisdr_dev {
	struct cdev cdev;
} *spisdr_devp;


static dev_t spisdr_dev_number;
static struct class *spisdr_class;

#define DEVICE_NAME	"spisdr"

static struct file_operations spisdr_fops;

static int  __init
spisdr_init(void)
{
	int ret;

	printk("spisdr Start Driver Initialized.\n");

	if (alloc_chrdev_region(&spisdr_dev_number, 0, 1, DEVICE_NAME) < 0) {
		printk(KERN_DEBUG "Can't register device\n");
		return -1;
	}

	spisdr_class = class_create(THIS_MODULE, DEVICE_NAME);

	spisdr_devp = kmalloc(sizeof(struct spisdr_dev), GFP_KERNEL);
	if (!spisdr_devp) {
		printk("Bad kmalloc\n");
		return -ENOMEM;
	}

	cdev_init(&spisdr_devp->cdev, &spisdr_fops);
	spisdr_devp->cdev.owner = THIS_MODULE;

	ret = cdev_add(&spisdr_devp->cdev,
		       MKDEV(MAJOR(spisdr_dev_number), 0), 1);
	if (ret) {
		printk("Bad cdev\n");
		return ret;
	}

	device_create(spisdr_class, NULL, MKDEV(MAJOR(spisdr_dev_number),0),
		      "spisdr0", 0);

	printk("spisdr Driver Initialized.\n");

	return 0;
}

static void __exit
spisdr_cleanup(void)
{

}

static int
spisdr_open(struct inode *inode, struct file *file)
{

	return 0;
}

static int
spisdr_release(struct inode *inode, struct file *file)
{

	return 0;
}

static ssize_t
spisdr_read(struct file *file, char *buf, size_t count, loff_t **ppos)
{

	return 0;
}

static ssize_t
spisdr_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{

	return 0;
}

static loff_t
spisdr_llseek(struct file *file, loff_t offest, int orig)
{

	return 0;
}

static int spisdr_ioctl(struct inode *inode, struct file *file,
                        unsigned int cmd, unsigned long arg)
{

	return 0;
}

static struct file_operations spisdr_fops = {
	.owner		=	THIS_MODULE,
	.open		=	spisdr_open,
	.release 	=	spisdr_release,
	.read		=	spisdr_read,
	.write		=	spisdr_write,
	.llseek		=	spisdr_llseek,
	.ioctl		=	spisdr_ioctl,
};

MODULE_VERSION("0.1");
MODULE_ALIAS(DEVICE_NAME);
MODULE_DESCRIPTION(DEVICE_NAME);
MODULE_AUTHOR("Philip Balister <philip@opensdr.com>");
MODULE_LICENSE("GPL v2");

module_init(spisdr_init);
module_exit(spisdr_cleanup);
