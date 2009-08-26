/*
 * Driver for eding/writing to FPGA via SPI for SDR work.
 *
 * Copyright (C) Philip Balister <philip@opensdr.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/wait.h>

#include <linux/spi/spi.h>

#include <asm/uaccess.h>

#include <mach/gpio.h>

struct spisdr_data {
	struct cdev cdev;

	struct mutex	buf_lock;
	unsigned	users;
	u8		*buffer;
} *spisdr_datap;

static dev_t spisdr_dev_number;
static struct class *spisdr_class;

#define DEVICE_NAME	"spisdr"

static struct file_operations	spisdr_fops;
static struct spi_driver	spisdr_spi;

static struct spi_device	*spi_rx;
static struct spi_device	*spi_tx;
static spinlock_t		spi_lock_rx;
static spinlock_t		spi_lock_tx;

static DECLARE_WAIT_QUEUE_HEAD(data_ready_queue);

static unsigned bufsiz = 4096;
static int	in_use = 0;

static irqreturn_t spisdr_irqhandler(int irq, void *dev_id)
{
	int serviced = IRQ_NONE;

/*
	printk("Received spisdr interrupt\n");
*/

	wake_up_interruptible(&data_ready_queue);

	serviced = IRQ_HANDLED;

	return serviced;
}

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

	spisdr_datap = kzalloc(sizeof(struct spisdr_data), GFP_KERNEL);
	if (!spisdr_datap) {
		printk("Bad kmalloc\n");
		return -ENOMEM;
	}

	cdev_init(&spisdr_datap->cdev, &spisdr_fops);
	spisdr_datap->cdev.owner = THIS_MODULE;

	ret = cdev_add(&spisdr_datap->cdev,
		       MKDEV(MAJOR(spisdr_dev_number), 0), 1);
	if (ret) {
		printk("Bad cdev\n");
		return ret;
	}

	printk("spisdr major number : %d\n", MAJOR(spisdr_dev_number));
	device_create(spisdr_class, NULL, MKDEV(MAJOR(spisdr_dev_number),0),
		      NULL, "spisdr%d", 0);

	ret = spi_register_driver(&spisdr_spi);
	if (ret < 0) {
		device_destroy(spisdr_class, MKDEV(MAJOR(spisdr_dev_number)
			, 0));
		cdev_del(&spisdr_datap->cdev);
		kfree(spisdr_datap);
		class_destroy(spisdr_class);
	}

	if ((gpio_request(133, "SPIDSDR_IRQ") == 0) &&
            (gpio_direction_input(133) == 0)){
		gpio_export(133, 0);
	} else {
		printk(KERN_ERR "could not obtain gpio for SPISDR IRQ\n");
		return -1;
	}

	set_irq_type(gpio_to_irq(133), IRQ_TYPE_EDGE_RISING);
	ret = request_irq(gpio_to_irq(133), spisdr_irqhandler, IRQF_DISABLED,
                             "spisdr", &spisdr_datap->cdev);

	printk("spisdr Driver Initialized.\n");

	return 0;
}

static void __exit
spisdr_cleanup(void)
{
	unregister_chrdev_region(spisdr_dev_number, 1);

	device_destroy(spisdr_class, MKDEV(MAJOR(spisdr_dev_number), 0));
	cdev_del(&spisdr_datap->cdev);
	kfree(spisdr_datap);

	class_destroy(spisdr_class);
}


static int
spisdr_open(struct inode *inode, struct file *file)
{
	struct spisdr_data *spisdr_datap;
	int status;

	printk("spisdr open called\n");

	if (in_use)
		return -1;
	in_use = 1;

	spisdr_datap = container_of(inode->i_cdev, struct spisdr_data, cdev);

	file->private_data = spisdr_datap;

	if (!spisdr_datap->buffer) {
		spisdr_datap->buffer = kmalloc(bufsiz, GFP_KERNEL);
		if (!spisdr_datap->buffer) {
			dev_dbg(&spi_rx->dev, "open/ENOMEM\n");
			status = -ENOMEM;
		}
	}

	spi_rx->bits_per_word = 32;
	spi_rx->max_speed_hz = 48000000;
	spi_setup(spi_rx);

	return 0;
}

static int
spisdr_release(struct inode *inode, struct file *file)
{
	struct spisdr_data *spisdr_datap = file->private_data;
	int dofree;

	printk("spisdr release called\n");

	in_use = 0;

	kfree(spisdr_datap->buffer);
	spisdr_datap->buffer = NULL;

	spin_lock_irq(&spi_lock_rx);
	dofree = (spi_rx == NULL);
	spin_unlock_irq(&spi_lock_rx);

	if (dofree)
		kfree(spisdr_datap);

	spisdr_datap = 0;

	return 0;
}

static void spisdr_complete(void *arg)
{
	complete(arg);
}

static ssize_t
spisdr_spi_sync(struct spisdr_data *spisdr, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	// printk("spisdr spi_sync called\n");

	message->complete = spisdr_complete;
	message->context = &done;

	spin_lock_irq(&spi_lock_rx);
	if (spi_rx == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(spi_rx, message);
	spin_unlock(&spi_lock_rx);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
	return status;
}

static inline ssize_t
spisdr_read_from_spi(struct spisdr_data *spisdr, size_t len)
{
	struct spi_transfer	t = {
		.rx_buf		= spisdr->buffer,
		.len		= len,
	};
	struct spi_message	m;

	// printk("spisdr read_from_spi called\n");

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spisdr_spi_sync(spisdr, &m);
}

ssize_t
spisdr_read(struct file *file, char *buf, size_t count, loff_t **ppos)
{
	struct spisdr_data	*spisdr;
	ssize_t			status = 0;
	DECLARE_WAITQUEUE(wait, current);

	// printk("spisdr read called\n");

	if (count > bufsiz)
		return -EMSGSIZE;

	spisdr = file->private_data;

	add_wait_queue(&data_ready_queue, &wait);
	while  (gpio_get_value(133) == 0) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (signal_pending(current)) {
			printk("Signal received\n");
			set_current_state(TASK_RUNNING);
			remove_wait_queue(&data_ready_queue, &wait);
			return -EINTR;
		}
		schedule();
	}
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&data_ready_queue, &wait);

	mutex_lock(&spisdr->buf_lock);
	status = spisdr_read_from_spi(spisdr, count);
	if (status > 0) {
		unsigned long	missing;

		missing = copy_to_user(buf, spisdr->buffer, status);
		if (missing == status)
			status = -EFAULT;
		else
			status = status - missing;
	}
	mutex_unlock(&spisdr->buf_lock);

	return status;
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
	printk("In spisdr_probe: bus_num = %d, CS = %d\n", spi->master->bus_num, spi->chip_select);

	if (spi->chip_select == 0) {
		spi_rx = spi;
		spin_lock_init(&spi_lock_rx);
	}


	if (spi->chip_select == 1) {
		spi_tx = spi;
		spin_lock_init(&spi_lock_tx);
	}

	mutex_init(&spisdr_datap->buf_lock);

	return 0;
}

static int spisdr_remove(struct spi_device *spi)
{

	printk("In spisdr_remove\n");

	return 0;
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
