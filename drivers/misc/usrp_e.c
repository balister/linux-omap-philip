/*
 * -*- linux-c -*-
 * Interface for USRP Embedded from Ettus Research, LLC.
 * This driver uses the GPMC interface on the OMAP3 to pass data
 * to/from a Spartan 3 FPGA.
 *
 * Copyright (C) Ettus Research, LLC
 *
 * Written by Philip Balister <philip@opensdr.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "linux/fs.h"
#include "linux/module.h"
#include "linux/cdev.h"
#include "linux/device.h"
#include "linux/spinlock.h"
#include "linux/errno.h"
#include "linux/irq.h"
#include "linux/interrupt.h"
#include "linux/wait.h"
#include "linux/sched.h"
#include "linux/dma-mapping.h"
#include "linux/semaphore.h"
#include "linux/kthread.h"
#include "linux/poll.h"
#include "linux/slab.h"
#include "linux/delay.h"

#include "plat/gpmc.h"
#include "plat/gpio.h"
#include "plat/dma.h"

#include "asm/uaccess.h"
#include "asm/io.h"
#include "asm/atomic.h"

#include "linux/usrp_e.h"

#define TX_SPACE_AVAILABLE_GPIO	144
#define RX_DATA_READY_GPIO	146

static atomic_t use_count = ATOMIC_INIT(0);
static atomic_t mapped = ATOMIC_INIT(0);
static int shutting_down;

struct usrp_e_dev {
	struct cdev cdev;
	unsigned long mem_base;
	unsigned long control_mem_base;
	u32 *ioaddr;
	u8 *ctl_addr;
	spinlock_t fpga_lock;

	atomic_t n_overruns;
	atomic_t n_underruns;

} *usrp_e_devp;

struct dma_data {
	int ch;
	struct omap_dma_channel_params params;

	unsigned long virt_from;
	unsigned long virt_to;
	unsigned long phys_from;
	unsigned long phys_to;
};

static struct dma_data *rx_dma;
static struct dma_data *tx_dma;

struct ring_buffer_entry {
	unsigned long dma_addr;
	__u8 *frame_addr;
};

struct ring_buffer {
	struct ring_buffer_info (*rbi)[];
	struct ring_buffer_entry (*rbe)[];
	int num_pages;
	unsigned long (*pages)[];
};

static struct ring_buffer tx_rb;
static struct ring_buffer rx_rb;

static struct usrp_e_ring_buffer_size_t rb_size;

#define NUM_PAGES_RX_FLAGS 1
#define NUM_RX_FRAMES 512
#define NUM_PAGES_TX_FLAGS 1
#define NUM_TX_FRAMES 512

static int tx_rb_read;
static int rx_rb_write;

static DEFINE_SPINLOCK(tx_rb_read_lock);
static DEFINE_SPINLOCK(rx_rb_write_lock);

static int alloc_ring_buffer(struct ring_buffer *rb,
			unsigned int num_bufs, enum dma_data_direction direction);
static void delete_ring_buffer(struct ring_buffer *rb,
			unsigned int num_bufs, enum dma_data_direction direction);
static int alloc_ring_buffers(void);
static void init_ring_buffer(struct ring_buffer *rb, int num_bufs,
			int init_flags, enum dma_data_direction direction);

static dev_t usrp_e_dev_number;
static struct class *usrp_e_class;

#define DEVICE_NAME	"usrp_e"

static const struct file_operations usrp_e_fops;

static irqreturn_t space_available_irqhandler(int irq, void *dev_id);
static irqreturn_t data_ready_irqhandler(int irq, void *dev_id);
static void usrp_rx_dma_irq(int ch, u16 stat, void *data);
static void usrp_tx_dma_irq(int ch, u16 stat, void *data);

static DECLARE_WAIT_QUEUE_HEAD(data_received_queue);
static DECLARE_WAIT_QUEUE_HEAD(space_available_queue);
static DECLARE_WAIT_QUEUE_HEAD(received_data_from_user);
static DECLARE_WAIT_QUEUE_HEAD(tx_rb_space_available);

static int init_dma_controller(void);
static void release_dma_controller(void);
static int get_frame_from_fpga_start(void);
static int get_frame_from_fpga_finish(void);
static int send_frame_to_fpga_start(void);
static int send_frame_to_fpga_finish(void);

static int rx_dma_active;
static int tx_dma_active;

static int  __init
usrp_e_init(void)
{
	int ret;
	struct usrp_e_dev *p;

	printk(KERN_DEBUG "usrp_e entering driver initialization\n");

	if (alloc_chrdev_region(&usrp_e_dev_number, 0, 1, DEVICE_NAME) < 0) {
		printk(KERN_DEBUG "Can't register device\n");
		return -1;
	}

	usrp_e_class = class_create(THIS_MODULE, DEVICE_NAME);

	usrp_e_devp = kzalloc(sizeof(struct usrp_e_dev), GFP_KERNEL);
	if (!usrp_e_devp) {
		printk(KERN_ERR "Bad kmalloc\n");
		return -ENOMEM;
	}

	p = usrp_e_devp; /* Shorten var name so I stay sane. */

	printk(KERN_DEBUG "usrp_e data struct malloc'd.\n");

	atomic_set(&p->n_underruns, 0);
	atomic_set(&p->n_overruns, 0);

	printk(KERN_DEBUG "usrp_e Data initialized..\n");

	cdev_init(&p->cdev, &usrp_e_fops);
	p->cdev.owner = THIS_MODULE;

	ret = cdev_add(&p->cdev, MKDEV(MAJOR(usrp_e_dev_number), 0), 1);
	if (ret) {
		printk(KERN_ERR "Bad cdev\n");
		return ret;
	}

	printk(KERN_DEBUG "usrp_e major number : %d\n",
		MAJOR(usrp_e_dev_number));
	device_create(usrp_e_class, NULL, MKDEV(MAJOR(usrp_e_dev_number), 0),
		      NULL, "usrp_e%d", 0);

	printk(KERN_DEBUG "Getting Chip Select\n");

	if (gpmc_cs_request(4, SZ_2K, &p->mem_base) < 0) {
		printk(KERN_ERR "Failed request for GPMC mem for usrp_e\n");
		return -1;
	}
	printk(KERN_DEBUG "Got CS4, address = %lx\n", p->mem_base);

	if (!request_mem_region(p->mem_base, SZ_2K, "usrp_e")) {
		printk(KERN_ERR "Request_mem_region failed.\n");
		gpmc_cs_free(4);
		return -1;
	}

	p->ioaddr = ioremap(p->mem_base, SZ_2K);
	spin_lock_init(&p->fpga_lock);

	if (gpmc_cs_request(6, SZ_2K, &p->control_mem_base) < 0) {
		printk(KERN_ERR "Failed request for GPMC control mem for usrp_e\n");
		return -1;
	}
	printk(KERN_DEBUG "Got CS6, address = %lx\n", p->control_mem_base);

	if (!request_mem_region(p->control_mem_base, SZ_2K, "usrp_e_c")) {
		printk(KERN_ERR "Request_mem_region failed.\n");
		gpmc_cs_free(6);
		return -1;
	}

	p->ctl_addr = ioremap_nocache(p->control_mem_base, SZ_2K);


	/* Configure GPIO's */

	if (!(((gpio_request(TX_SPACE_AVAILABLE_GPIO,
				    "TX_SPACE_AVAILABLE_GPIO") == 0) &&
		(gpio_direction_input(TX_SPACE_AVAILABLE_GPIO) == 0)))) {
		printk(KERN_ERR "Could not claim GPIO for TX_SPACE_AVAILABLE_GPIO\n");
		return -1;
	}

	if (!(((gpio_request(RX_DATA_READY_GPIO, "RX_DATA_READY_GPIO") == 0) &&
		(gpio_direction_input(RX_DATA_READY_GPIO) == 0)))) {
		printk(KERN_ERR "Could not claim GPIO for RX_DATA_READY_GPIO\n");
		return -1;
	}

	rb_size.num_pages_rx_flags = NUM_PAGES_RX_FLAGS;
	rb_size.num_rx_frames = NUM_RX_FRAMES;
	rb_size.num_pages_tx_flags = NUM_PAGES_TX_FLAGS;
	rb_size.num_tx_frames = NUM_TX_FRAMES;

	ret = alloc_ring_buffers();
	if (ret < 0)
		return ret;

	/* Initialize various DMA related flags */
	rx_dma_active = 0;
	tx_dma_active = 0;
	shutting_down = 0;

	printk(KERN_DEBUG "usrp_e Driver Initialized.\n");

	return 0;
}

static void __exit
usrp_e_cleanup(void)
{
	struct usrp_e_dev *p = usrp_e_devp;

	unregister_chrdev_region(usrp_e_dev_number, 1);

	release_mem_region(p->mem_base, SZ_2K);
	release_mem_region(p->control_mem_base, SZ_2K);

	device_destroy(usrp_e_class, MKDEV(MAJOR(usrp_e_dev_number), 0));
	cdev_del(&p->cdev);

	class_destroy(usrp_e_class);

	iounmap(p->ioaddr);
	iounmap(p->ctl_addr);

	gpmc_cs_free(4);
	gpmc_cs_free(6);

	printk(KERN_DEBUG "Freeing gpios\n");

	gpio_free(TX_SPACE_AVAILABLE_GPIO);
	gpio_free(RX_DATA_READY_GPIO);

	delete_ring_buffer(&tx_rb, rb_size.num_tx_frames, DMA_TO_DEVICE);
	delete_ring_buffer(&rx_rb, rb_size.num_rx_frames, DMA_FROM_DEVICE);

	kfree(p);

	printk(KERN_DEBUG "Leaving cleanup\n");
}

static int
usrp_e_open(struct inode *inode, struct file *file)
{
	int ret;

	printk(KERN_DEBUG "usrp_e open called, use_count = %d\n",
		atomic_read(&use_count));
	if (atomic_add_return(1, &use_count) != 1) {
		printk(KERN_ERR "use_count = %d\n", atomic_read(&use_count));
		atomic_dec(&use_count);
		return -EBUSY;
	}

	ret = init_dma_controller();
	if (ret < 0)
		return ret;

	tx_rb_read = 0;
	rx_rb_write = 0;

	tx_dma_active = 0;
	rx_dma_active = 0;
	shutting_down = 0;

	init_ring_buffer(&rx_rb, rb_size.num_rx_frames, RB_KERNEL, DMA_FROM_DEVICE);
	init_ring_buffer(&tx_rb, rb_size.num_tx_frames, RB_KERNEL, DMA_TO_DEVICE);

	/* Configure interrupts for GPIO pins */

	ret = request_irq(gpio_to_irq(TX_SPACE_AVAILABLE_GPIO),
			space_available_irqhandler,
		IRQF_TRIGGER_RISING, "usrp_e_space_available", NULL);

	ret = request_irq(gpio_to_irq(RX_DATA_READY_GPIO),
			data_ready_irqhandler,
			IRQF_TRIGGER_RISING, "usrp_e_data_ready", NULL);

	printk(KERN_DEBUG "usrp: leaving open\n");
	return 0;
}

static int
usrp_e_release(struct inode *inode, struct file *file)
{
	struct usrp_e_dev *usrp_e_devp = file->private_data;

	printk(KERN_DEBUG "usrp_e release called\n");

	if (atomic_read(&use_count) != 1) {
		printk(KERN_ERR "Attempt to close usrp_e driver that is not open");
		return -ENOENT;
	}

	printk(KERN_DEBUG "Waiting for DMA to become inactive\n");
	shutting_down = 1;
	while (tx_dma_active || rx_dma_active)
		cpu_relax();

	/* Freeing gpio irq's */
	printk(KERN_DEBUG "Freeing gpio irq's\n");

	free_irq(gpio_to_irq(TX_SPACE_AVAILABLE_GPIO), NULL);
	free_irq(gpio_to_irq(RX_DATA_READY_GPIO), NULL);

	printk(KERN_DEBUG "Freeing DMA channels\n");

	release_dma_controller();

	usrp_e_devp = 0;

	atomic_dec(&use_count);

	return 0;
}

static ssize_t
usrp_e_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{

	return count;
}

static ssize_t
usrp_e_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{

	send_frame_to_fpga_start();

	return count;
}

static loff_t
usrp_e_llseek(struct file *file, loff_t offest, int orig)
{
	printk(KERN_DEBUG "usrp_e llseek called\n");

	return 0;
}

static int usrp_e_ctl16(unsigned long arg, int direction)
{
	struct usrp_e_ctl16 __user *argp = (struct usrp_e_ctl16 __user *) arg;
	int i;
	struct usrp_e_ctl16 ctl;

	if (copy_from_user(&ctl, argp, sizeof(struct usrp_e_ctl16)))
		return -EFAULT;

	if (ctl.count > 20)
		return -EINVAL;

	if ((ctl.offset >= SZ_2K) || ((ctl.offset + 2 * ctl.count) >= SZ_2K))
		return -EINVAL;

	if (direction == 0) {
		for (i = 0; i < ctl.count; i++)
			writew(ctl.buf[i], &usrp_e_devp->ctl_addr \
				[i*2 + ctl.offset]);
	} else if (direction == 1) {
		for (i = 0; i < ctl.count; i++)
			ctl.buf[i] = readw(&usrp_e_devp->ctl_addr \
				[i*2 + ctl.offset]);

		if (copy_to_user(argp, &ctl, sizeof(struct usrp_e_ctl16)))
			return -EFAULT;
	} else
		return -EFAULT;

	return 0;
}

static int usrp_e_ctl32(unsigned long arg, int direction)
{
	struct usrp_e_ctl32 __user *argp = (struct usrp_e_ctl32 __user *) arg;
	int i;
	struct usrp_e_ctl32 ctl;

	if (copy_from_user(&ctl, argp, sizeof(struct usrp_e_ctl32)))
		return -EFAULT;

	if (ctl.count > 10)
		return -EINVAL;

	if ((ctl.offset >= SZ_2K) || ((ctl.offset + 4 * ctl.count) >= SZ_2K))
		return -EINVAL;

	if (direction == 0) {
		for (i = 0; i < ctl.count; i++)
			writel(ctl.buf[i], &usrp_e_devp->ctl_addr \
				[i*4 + ctl.offset]);
	} else if (direction == 1) {
		for (i = 0; i < ctl.count; i++)
			ctl.buf[i] = readl(&usrp_e_devp->ctl_addr \
				[i*4 + ctl.offset]);

		if (copy_to_user(argp, &ctl, sizeof(struct usrp_e_ctl16)))
			return -EFAULT;

	} else
		return -EFAULT;

	return 0;
}

static int usrp_e_get_rb_info(unsigned long arg)
{
	struct usrp_e_ring_buffer_size_t __user *argp = (struct usrp_e_ring_buffer_size_t __user *) arg;

	if (copy_to_user(argp, &rb_size, sizeof(rb_size)))
		return -EFAULT;

	return 0;
}


static long usrp_e_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{

	switch (cmd) {
	case USRP_E_WRITE_CTL16:
		return usrp_e_ctl16(arg, 0);

	case USRP_E_READ_CTL16:
		return usrp_e_ctl16(arg, 1);

	case USRP_E_WRITE_CTL32:
		return usrp_e_ctl32(arg, 0);

	case USRP_E_READ_CTL32:
		return usrp_e_ctl32(arg, 1);

	case USRP_E_GET_RB_INFO:
		return usrp_e_get_rb_info(arg);

	case USRP_E_GET_COMPAT_NUMBER:
		return USRP_E_COMPAT_NUMBER;

	default:
		return -ENOTTY;
	}

	return 0;
}

static unsigned int usrp_e_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;
	unsigned long flags;

	poll_wait(filp, &data_received_queue, wait);
	poll_wait(filp, &tx_rb_space_available, wait);

	/* Make sure write is active (if needed) before sleeping */
	send_frame_to_fpga_start();

	/* Make sure to read in case the rx ring buffer is empty */
	get_frame_from_fpga_start();

	spin_lock_irqsave(&rx_rb_write_lock, flags);
	if (rx_rb_write == 0) {
		if ((*rx_rb.rbi)[rb_size.num_rx_frames - 1].flags & RB_USER)
			mask |= POLLIN | POLLRDNORM;
	} else {
		if ((*rx_rb.rbi)[rx_rb_write - 1].flags & RB_USER)
			mask |= POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&rx_rb_write_lock, flags);

	spin_lock_irqsave(&tx_rb_read_lock, flags);
	if (tx_rb_read == 0) {
		if ((*tx_rb.rbi)[rb_size.num_tx_frames - 1].flags & RB_KERNEL)
			mask |= POLLOUT | POLLWRNORM;
	} else {
		if ((*tx_rb.rbi)[tx_rb_read - 1].flags & RB_KERNEL)
			mask |= POLLOUT | POLLWRNORM;
	}
	spin_unlock_irqrestore(&tx_rb_read_lock, flags);

	return mask;

}

/* The mmap code is based on code in af_packet.c */

static void usrp_e_mm_open(struct vm_area_struct *vma)
{

	atomic_inc(&mapped);
}

static void usrp_e_mm_close(struct vm_area_struct *vma)
{

	atomic_dec(&mapped);
}

static const struct vm_operations_struct usrp_e_mmap_ops = {
	.open   = usrp_e_mm_open,
	.close  = usrp_e_mm_close,
};

static int usrp_e_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long size, expected_size;
	unsigned int i;
	unsigned long start;
	int err;
	struct page *page;

	if (vma->vm_pgoff)
		return -EINVAL;

	/* Verify the user will map the entire tx and rx ring buffer space */
	expected_size = (rb_size.num_rx_frames + rb_size.num_tx_frames) * (PAGE_SIZE >> 1)
		+ (rb_size.num_pages_rx_flags + rb_size.num_pages_tx_flags) * PAGE_SIZE;

	size = vma->vm_end - vma->vm_start;
	printk(KERN_DEBUG "Size = %ld, expected sixe = %ld\n", size, expected_size);

	if (size != expected_size)
		return -EINVAL;

	start = vma->vm_start;

	page = virt_to_page(rx_rb.rbi);
	err = vm_insert_page(vma, start, page);
	if (err)
		return -EINVAL;
	
	start += PAGE_SIZE;
	
	for (i = 0; i < rx_rb.num_pages; ++i) {
		struct page *page = virt_to_page((*rx_rb.pages)[i]);
		err = vm_insert_page(vma, start, page);
		if (err)
			return -EINVAL;

		start += PAGE_SIZE;
	}

	page = virt_to_page(tx_rb.rbi);
	err = vm_insert_page(vma, start, page);
	if (err)
		return -EINVAL;
	
	start += PAGE_SIZE;
	
	for (i = 0; i < tx_rb.num_pages; ++i) {
		struct page *page = virt_to_page((*tx_rb.pages)[i]);

		err = vm_insert_page(vma, start, page);
		if (err)
			return err;

		start += PAGE_SIZE;
	}

//	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	vma->vm_ops = &usrp_e_mmap_ops;

	return 0;
}

static const struct file_operations usrp_e_fops = {
	.owner		=	THIS_MODULE,
	.open		=	usrp_e_open,
	.release 	=	usrp_e_release,
	.read		=	usrp_e_read,
	.write		=	usrp_e_write,
	.llseek		=	usrp_e_llseek,
	.unlocked_ioctl	=	usrp_e_ioctl,
	.poll           =       usrp_e_poll,
	.mmap           =       usrp_e_mmap,
};

MODULE_VERSION("0.3");
MODULE_ALIAS(DEVICE_NAME);
MODULE_DESCRIPTION(DEVICE_NAME);
MODULE_AUTHOR("Philip Balister <philip@opensdr.com>");
MODULE_LICENSE("GPL v2");

module_init(usrp_e_init);
module_exit(usrp_e_cleanup);

static irqreturn_t space_available_irqhandler(int irq, void *dev_id)
{
	int serviced = IRQ_NONE;

	send_frame_to_fpga_start();

	serviced = IRQ_HANDLED;

	return serviced;
}

static void usrp_rx_dma_irq(int ch, u16 stat, void *data)
{

	rx_dma_active = 0;

	get_frame_from_fpga_finish();

}

static void usrp_tx_dma_irq(int ch, u16 stat, void *data)
{

	tx_dma_active = 0;

	send_frame_to_fpga_finish();

}

static irqreturn_t data_ready_irqhandler(int irq, void *dev_id)
{
	int serviced = IRQ_NONE;

	get_frame_from_fpga_start();

	serviced = IRQ_HANDLED;

	return serviced;
}

static int init_dma_controller()
{
	struct usrp_e_dev *p = usrp_e_devp;

	rx_dma = kzalloc(sizeof(struct dma_data), GFP_KERNEL);
	if (!rx_dma) {
		printk(KERN_ERR "Failed to allocate memory for rx_dma struct.");
		return -ENOMEM;
	}

	if (omap_request_dma(OMAP_DMA_NO_DEVICE, "usrp-e-rx",
			usrp_rx_dma_irq, (void *) rx_dma, &rx_dma->ch)) {
		printk(KERN_ERR "Could not get rx DMA channel for usrp_e\n");
		return -ENOMEM;
	}
	printk(KERN_DEBUG "rx_dma->ch %d\n", rx_dma->ch);

	rx_dma->phys_from = p->mem_base;

	memset(&rx_dma->params, 0, sizeof(rx_dma->params));
	rx_dma->params.data_type	= OMAP_DMA_DATA_TYPE_S16;

	rx_dma->params.src_amode	= OMAP_DMA_AMODE_POST_INC;
	rx_dma->params.dst_amode	= OMAP_DMA_AMODE_POST_INC;

	rx_dma->params.src_start	= p->mem_base;
	rx_dma->params.dst_start	= rx_dma->phys_to;

	rx_dma->params.src_ei		= 1;
	rx_dma->params.src_fi		= 1;
	rx_dma->params.dst_ei		= 1;
	rx_dma->params.dst_fi		= 1;

	rx_dma->params.elem_count	= 1024;
	rx_dma->params.frame_count	= 1;

	rx_dma->params.read_prio        = DMA_CH_PRIO_HIGH;
	rx_dma->params.write_prio       = DMA_CH_PRIO_LOW;

	omap_set_dma_params(rx_dma->ch, &rx_dma->params);

// Play with these with a real application
	omap_set_dma_src_burst_mode(rx_dma->ch, OMAP_DMA_DATA_BURST_16);
	omap_set_dma_dest_burst_mode(rx_dma->ch, OMAP_DMA_DATA_BURST_16);
	omap_set_dma_src_data_pack(rx_dma->ch, 1);
	omap_set_dma_dest_data_pack(rx_dma->ch, 1);

#if 0 // Need to find implentations of the endian calls
	omap_set_dma_src_endian_type(rx_dma->ch, OMAP_DMA_BIG_ENDIAN);
	omap_set_dma_dst_endian_type(rx_dma->ch, OMAP_DMA_LITTLE_ENDIAN);
#endif

	tx_dma = kzalloc(sizeof(struct dma_data), GFP_KERNEL);
	if (!tx_dma) {
		printk(KERN_ERR "Failed to allocate memory for tx_dma struct.");
		return -ENOMEM;
	}

	if (omap_request_dma(OMAP_DMA_NO_DEVICE, "usrp-e-tx",
			usrp_tx_dma_irq, (void *) tx_dma, &tx_dma->ch)) {
		printk(KERN_ERR "Could not get tx DMA channel for usrp_e\n");
		return -ENOMEM;
	}

	printk(KERN_DEBUG "tx_dma->ch %d\n", tx_dma->ch);

	tx_dma->phys_from = p->mem_base;

	memset(&tx_dma->params, 0, sizeof(tx_dma->params));
	tx_dma->params.data_type	= OMAP_DMA_DATA_TYPE_S16;

	tx_dma->params.src_amode	= OMAP_DMA_AMODE_POST_INC;
	tx_dma->params.dst_amode	= OMAP_DMA_AMODE_POST_INC;

	tx_dma->params.src_start	= tx_dma->phys_from;
	tx_dma->params.dst_start	= p->mem_base;

	tx_dma->params.src_ei		= 1;
	tx_dma->params.src_fi		= 1;
	tx_dma->params.dst_ei		= 1;
	tx_dma->params.dst_fi		= 1;

	tx_dma->params.elem_count	= 1024;
	tx_dma->params.frame_count	= 1;

	tx_dma->params.read_prio        = DMA_CH_PRIO_LOW;
	tx_dma->params.write_prio       = DMA_CH_PRIO_HIGH;

	omap_set_dma_params(tx_dma->ch, &tx_dma->params);

// Play with these with a real application
	omap_set_dma_src_burst_mode(tx_dma->ch, OMAP_DMA_DATA_BURST_16);
	omap_set_dma_dest_burst_mode(tx_dma->ch, OMAP_DMA_DATA_BURST_16);
	omap_set_dma_src_data_pack(tx_dma->ch, 1);
	omap_set_dma_dest_data_pack(tx_dma->ch, 1);

	return 0;
}

static void release_dma_controller()
{

	omap_free_dma(rx_dma->ch);
	omap_free_dma(tx_dma->ch);

	kfree(rx_dma);
	kfree(tx_dma);
}

static int get_frame_from_fpga_start()
{
	struct usrp_e_dev *p = usrp_e_devp;
	struct ring_buffer_info *rbi;
	struct ring_buffer_entry *rbe;
	u16 elements_to_read;
	unsigned long flags;

	spin_lock_irqsave(&rx_rb_write_lock, flags);
	rbi = &(*rx_rb.rbi)[rx_rb_write];
	rbe = &(*rx_rb.rbe)[rx_rb_write];

	/* Check for space available in the ring buffer */
	/* If no space, drop data. A read call will restart dma transfers. */
	if ((rbi->flags & RB_KERNEL) && (gpio_get_value(RX_DATA_READY_GPIO)) && !rx_dma_active  && !shutting_down) {

		rx_dma_active = 1;

		rbi->flags = RB_DMA_ACTIVE;
		
		spin_unlock_irqrestore(&rx_rb_write_lock, flags);

		elements_to_read = 2048;
		if (elements_to_read > 2048) {
			printk(KERN_ERR "usrp_e: FPGA has bad transfer size of %d\n", elements_to_read);
			goto out;
		}

// writew(1, p->ctl_addr + 54);
		rbi->len = elements_to_read;
		
// writew(2, p->ctl_addr + 54);
		omap_set_dma_dest_addr_size(rx_dma->ch, rbe->dma_addr,
					(elements_to_read >> 1));
		
// writew(3, p->ctl_addr + 54);
		omap_start_dma(rx_dma->ch);

// writew(4, p->ctl_addr + 54);
		dma_sync_single_for_device(NULL, rbe->dma_addr, SZ_2K, DMA_FROM_DEVICE);
	} else {
		spin_unlock_irqrestore(&rx_rb_write_lock, flags);
	}

out:
	return 0;

}


static int get_frame_from_fpga_finish()
{
	unsigned long flags;

	dma_sync_single_for_cpu(NULL, (*rx_rb.rbe)[rx_rb_write].dma_addr, SZ_2K, DMA_FROM_DEVICE);

	spin_lock_irqsave(&rx_rb_write_lock, flags);
	(*rx_rb.rbi)[rx_rb_write].flags = RB_USER;
	rx_rb_write++;
	if (rx_rb_write == rb_size.num_rx_frames)
		rx_rb_write = 0;

	rx_dma_active = 0;

	spin_unlock_irqrestore(&rx_rb_write_lock, flags);

	wake_up_interruptible(&data_received_queue);

	get_frame_from_fpga_start();

	return 0;
}

static int send_frame_to_fpga_start()
{
	struct usrp_e_dev *p = usrp_e_devp;
	struct ring_buffer_info *rbi;
	struct ring_buffer_entry *rbe;
	u16 elements_to_write;
	unsigned long flags;

//	printk("In send_frame_to_fpga_start.\n");

	/* Check if there is data to write to the FPGA, if so send it */
	/* Otherwise, do nothing. Process is restarted by calls to write */

	spin_lock_irqsave(&tx_rb_read_lock, flags);
	rbi = &(*tx_rb.rbi)[tx_rb_read];
	rbe = &(*tx_rb.rbe)[tx_rb_read];

	if ((rbi->flags & RB_USER) && !tx_dma_active && (gpio_get_value(TX_SPACE_AVAILABLE_GPIO)) && !shutting_down) {
//		printk("In send_frame_to_fpga_start, past if.\n");
		tx_dma_active = 1;

		rbi->flags = RB_DMA_ACTIVE;
		
		spin_unlock_irqrestore(&tx_rb_read_lock, flags);

		elements_to_write = ((rbi->len) >> 1);
		
// writew(1, p->ctl_addr + 54);
		omap_set_dma_src_addr_size(tx_dma->ch, rbe->dma_addr,
					elements_to_write);
// writew(2, p->ctl_addr + 54);
//		dma_sync_single_for_device(NULL, rbe->dma_addr, SZ_2K, DMA_TO_DEVICE);
		dsb();
		
// writew(3, p->ctl_addr + 54);
		omap_start_dma(tx_dma->ch);
	} else {
		spin_unlock_irqrestore(&tx_rb_read_lock, flags);
	}

	return 0;
}

static int send_frame_to_fpga_finish()
{
	unsigned long flags;

//	dma_sync_single_for_cpu(NULL, (*tx_rb.rbe)[tx_rb_read].dma_addr, SZ_2K, DMA_TO_DEVICE);
	
	spin_lock_irqsave(&tx_rb_read_lock, flags);
	(*tx_rb.rbi)[tx_rb_read].flags = RB_KERNEL;

	
	tx_rb_read++;
	if (tx_rb_read == rb_size.num_tx_frames)
		tx_rb_read = 0;
	
	tx_dma_active = 0;

	spin_unlock_irqrestore(&tx_rb_read_lock, flags);

	wake_up_interruptible(&tx_rb_space_available);

	send_frame_to_fpga_start();

	return 0;
}

static int alloc_ring_buffer(struct ring_buffer *rb,
			unsigned int num_bufs, enum dma_data_direction direction)
{
	int i;

	rb->rbi = (void *) __get_free_page(GFP_KERNEL | __GFP_COMP | __GFP_ZERO | __GFP_NOWARN);

	rb->rbe = kzalloc(sizeof(struct ring_buffer_entry) * num_bufs, GFP_KERNEL);
	if (!rb) {
		printk(KERN_ERR "Failed to allocate memory for rb entries\n");
		return -ENOMEM;
	}

	rb->num_pages = (num_bufs & 1) ? ((num_bufs + 1) / 2) : (num_bufs / 2);

	rb->pages = kzalloc(sizeof(unsigned long) * rb->num_pages, GFP_KERNEL);
	if (!(rb->pages)) {
		printk(KERN_ERR "Failed to allocate memory for rb page entries\n");
		return -ENOMEM;
	}

	for (i = 0; i < rb->num_pages; i++) {
		(*rb->pages)[i] =  __get_free_page(GFP_KERNEL | __GFP_DMA | __GFP_COMP | __GFP_ZERO | __GFP_NOWARN);

		(*(rb->rbe))[i*2].frame_addr =
			(void *) (*(rb->pages))[i];
		(*(rb->rbe))[i*2 + 1].frame_addr =
			(void *) ((*(rb->pages))[i] + SZ_2K);
		if (!(*(rb->rbe))[i*2].frame_addr || !(*(rb->rbe))[i*2 + 1].frame_addr) {
			printk(KERN_ERR "Failed to allocate memory dma buf\n");
			return -ENOMEM;
		}

		(*(rb->rbe))[i*2].dma_addr = dma_map_single(NULL, (*(rb->rbe))[i*2].frame_addr, SZ_2K, direction);
		(*(rb->rbe))[i*2 + 1].dma_addr = dma_map_single(NULL, (*(rb->rbe))[i*2 + 1].frame_addr, SZ_2K, direction);
		if (!(*(rb->rbe))[i*2].dma_addr || !(*(rb->rbe))[i*2 + 1].dma_addr) {
			printk(KERN_ERR "Failed to get physical address for dma buf\n");
			return -ENOMEM;
		}
	}

	return 0;
}

static void delete_ring_buffer(struct ring_buffer *rb,
			unsigned int num_bufs, enum dma_data_direction direction)
{
	unsigned int i;
	unsigned int num_pages;

	printk(KERN_DEBUG "Entering delete_ring_buffer\n");

	num_pages = (num_bufs & 1) ? ((num_bufs + 1) / 2) : (num_bufs / 2);

	for (i = 0; i < num_pages; i++) {
		dma_unmap_single(NULL, (*rb->rbe)[i*2].dma_addr, SZ_2K, direction);
		dma_unmap_single(NULL, (*rb->rbe)[i*2 + 1].dma_addr, SZ_2K, direction);
		free_page((*rb->pages)[i]);
	}

	free_page((unsigned long) rb->rbi);

	kfree(rb->pages);
	kfree(rb->rbe);

	printk(KERN_DEBUG "Leaving delete_ring_buffer\n");
}

static int alloc_ring_buffers()
{

	if (alloc_ring_buffer(&tx_rb, rb_size.num_rx_frames, DMA_TO_DEVICE) < 0)
		return -ENOMEM;
	if (alloc_ring_buffer(&rx_rb, rb_size.num_tx_frames, DMA_FROM_DEVICE) < 0)
		return -ENOMEM;

	return 0;
}

static void init_ring_buffer(struct ring_buffer *rb, int num_bufs,
			int initial_flags, enum dma_data_direction direction)
{
	int i;

	for (i = 0; i < num_bufs; i++) {
		dma_sync_single_for_device(NULL, (*rb->rbe)[i].dma_addr,
					SZ_2K, direction);
		dma_sync_single_for_cpu(NULL, (*rb->rbe)[i].dma_addr,
					SZ_2K, direction);
		(*rb->rbi)[i].flags = initial_flags;
	}

}

