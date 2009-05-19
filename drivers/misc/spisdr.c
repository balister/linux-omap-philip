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
#include "linux/spi/spi.h"

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

	printk("spisdr major number : %d\n", MAJOR(spisdr_dev_number));
	device_create(spisdr_class, NULL, MKDEV(MAJOR(spisdr_dev_number),0),
		      NULL, "spisdr%d", 0);

	printk("spisdr Driver Initialized.\n");

	return 0;
}

static void __exit
spisdr_cleanup(void)
{

	unregister_chrdev_region(spisdr_dev_number, 1);

	device_destroy(spisdr_class, MKDEV(MAJOR(spisdr_dev_number), 0));
	cdev_del(&spisdr_devp->cdev);
	kfree(spisdr_devp);

	class_destroy(spisdr_class);
}


static int
spisdr_open(struct inode *inode, struct file *file)
{
	struct spisdr_dev *spisdr_devp;

	printk("spisdr open called\n");

	spisdr_devp = container_of(inode->i_cdev, struct spisdr_dev, cdev);

	file->private_data = spisdr_devp;

	return 0;
}

static int
spisdr_release(struct inode *inode, struct file *file)
{
	struct spisdr_dev *spisdr_devp = file->private_data;

	printk("spisdr release called\n");

	spisdr_devp = 0;

	return 0;
}

ssize_t
spisdr_read(struct file *file, char *buf, size_t count, loff_t **ppos)
{
	printk("spisdr read called\n");

	return 0;
}

static ssize_t
spisdr_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	printk("spisdr write called\n");

	return 0;
}

static loff_t
spisdr_llseek(struct file *file, loff_t offest, int orig)
{
	printk("spisdr llseek called\n");

	return 0;
}

static int spisdr_ioctl(struct inode *inode, struct file *file,
                        unsigned int cmd, unsigned long arg)
{
	printk("spisdr ioctl called\n");

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

static int spisdr_probe(struct spi_device *spi)
{

}

static spisdr_remove(struct spi_device *spi)
{

}


static struct spi_driver spisdr_spi = {
	.driver = {
		.name =	"spisdr",
		.owner = THIS_MODULE,
	},
	.probe = spisdr_probe,
	.remove = __devexit_p(spisdr_remove),
};

MODULE_VERSION("0.1");
MODULE_ALIAS(DEVICE_NAME);
MODULE_DESCRIPTION(DEVICE_NAME);
MODULE_AUTHOR("Philip Balister <philip@opensdr.com>");
MODULE_LICENSE("GPL v2");

module_init(spisdr_init);
module_exit(spisdr_cleanup);
