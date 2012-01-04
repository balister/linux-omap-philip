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

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/dma-mapping.h>
#include <linux/semaphore.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include "plat/gpmc.h"
#include "plat/dma.h"

#include "asm/uaccess.h"
#include "asm/io.h"
#include "asm/atomic.h"

#include <linux/usrp_e.h>

#define X_TX_SPACE_AVAILABLE_GPIO	144
#define X_RX_DATA_READY_GPIO	146

struct dma_data {
	int ch;
	struct omap_dma_channel_params params;

	dma_addr_t phys_from;
	dma_addr_t phys_to;
};

struct ring_buffer_entry {
	dma_addr_t dma_addr;
	__u8 *frame_addr;
};

struct ring_buffer {
	struct ring_buffer_info (*rbi)[];
	struct ring_buffer_entry (*rbe)[];
	int num_pages;
	unsigned long (*pages)[];
};

static atomic_t use_count = ATOMIC_INIT(0);

struct usrp_e_drvdata {
	struct cdev cdev;

	atomic_t mapped;
	int shutting_down;

	unsigned long mem_base;
	unsigned long control_mem_base;
	u32 *ioaddr;
	u8 *ctl_addr;
	spinlock_t fpga_lock;

	atomic_t n_overruns;
	atomic_t n_underruns;

	struct dma_data *rx_dma;
	struct dma_data *tx_dma;
	int rx_dma_active;
	int tx_dma_active;

	struct ring_buffer tx_rb;
	struct ring_buffer rx_rb;
	struct usrp_e_ring_buffer_size_t rb_size;
	int tx_rb_read;
	int rx_rb_write;
};

struct usrp_e_devdata {
	struct usrp_e_pdata *pdata;
	struct usrp_e_drvdata *drvdata;
} dev_data;

static struct platform_driver usrp_e_driver;

#define NUM_PAGES_RX_FLAGS 1
#define NUM_RX_FRAMES 512
#define NUM_PAGES_TX_FLAGS 1
#define NUM_TX_FRAMES 512

static DEFINE_SPINLOCK(tx_rb_read_lock);
static DEFINE_SPINLOCK(rx_rb_write_lock);

static dev_t usrp_e_dev_number;
static struct class *usrp_e_class;

#define DEVICE_NAME	"usrp_e"
#define DRVNAME	"usrp_e"

static const struct file_operations usrp_e_fops;

static DECLARE_WAIT_QUEUE_HEAD(data_received_queue);
static DECLARE_WAIT_QUEUE_HEAD(tx_rb_space_available);


static int get_frame_from_fpga_start(void)
{
	struct usrp_e_drvdata *p = dev_data.drvdata;
	struct ring_buffer_info *rbi;
	struct ring_buffer_entry *rbe;
	u16 elements_to_read;
	unsigned long flags;

	spin_lock_irqsave(&rx_rb_write_lock, flags);
	rbi = &(*p->rx_rb.rbi)[p->rx_rb_write];
	rbe = &(*p->rx_rb.rbe)[p->rx_rb_write];

	/* Check for space available in the ring buffer */
	/* If no space, drop data. A read call will restart dma transfers. */
	if ((rbi->flags & RB_KERNEL) && (gpio_get_value(dev_data.pdata->data_ready_gpio)) && !p->rx_dma_active  && !p->shutting_down) {

		p->rx_dma_active = 1;

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
		omap_set_dma_dest_addr_size(p->rx_dma->ch, rbe->dma_addr,
					(elements_to_read >> 1));

// writew(3, p->ctl_addr + 54);
		omap_start_dma(p->rx_dma->ch);

// writew(4, p->ctl_addr + 54);
		dma_sync_single_for_device(NULL, rbe->dma_addr, SZ_2K, DMA_FROM_DEVICE);
	} else {
		spin_unlock_irqrestore(&rx_rb_write_lock, flags);
	}

out:
	return 0;

}


static int get_frame_from_fpga_finish(void)
{
	struct usrp_e_drvdata *p = dev_data.drvdata;
	unsigned long flags;

	dma_sync_single_for_cpu(NULL, (*p->rx_rb.rbe)[p->rx_rb_write].dma_addr, SZ_2K, DMA_FROM_DEVICE);

	spin_lock_irqsave(&rx_rb_write_lock, flags);
	(*p->rx_rb.rbi)[p->rx_rb_write].flags = RB_USER;
	p->rx_rb_write++;
	if (p->rx_rb_write == p->rb_size.num_rx_frames)
		p->rx_rb_write = 0;

	p->rx_dma_active = 0;

	spin_unlock_irqrestore(&rx_rb_write_lock, flags);

	wake_up_interruptible(&data_received_queue);

	get_frame_from_fpga_start();

	return 0;
}

static irqreturn_t data_ready_irqhandler(int irq, void *dev_id)
{
	int serviced = IRQ_NONE;

	get_frame_from_fpga_start();

	serviced = IRQ_HANDLED;

	return serviced;
}

static void usrp_rx_dma_irq(int ch, u16 stat, void *data)
{
	struct usrp_e_drvdata *p = dev_data.drvdata;

	p->rx_dma_active = 0;

	get_frame_from_fpga_finish();

}

static int send_frame_to_fpga_start(void)
{
	struct usrp_e_drvdata *p = dev_data.drvdata;
	struct ring_buffer_info *rbi;
	struct ring_buffer_entry *rbe;
	u16 elements_to_write;
	unsigned long flags;

//	printk("In send_frame_to_fpga_start.\n");

	/* Check if there is data to write to the FPGA, if so send it */
	/* Otherwise, do nothing. Process is restarted by calls to write */

	spin_lock_irqsave(&tx_rb_read_lock, flags);
	rbi = &(*p->tx_rb.rbi)[p->tx_rb_read];
	rbe = &(*p->tx_rb.rbe)[p->tx_rb_read];

	if ((rbi->flags & RB_USER) && !p->tx_dma_active && (gpio_get_value(dev_data.pdata->space_available_gpio)) && !p->shutting_down) {
//		printk("In send_frame_to_fpga_start, past if.\n");
		p->tx_dma_active = 1;

		rbi->flags = RB_DMA_ACTIVE;

		spin_unlock_irqrestore(&tx_rb_read_lock, flags);

		elements_to_write = ((rbi->len) >> 1);

// writew(1, p->ctl_addr + 54);
		omap_set_dma_src_addr_size(p->tx_dma->ch, rbe->dma_addr,
					elements_to_write);
// writew(2, p->ctl_addr + 54);
//		dma_sync_single_for_device(NULL, rbe->dma_addr, SZ_2K, DMA_TO_DEVICE);
		dsb();

// writew(3, p->ctl_addr + 54);
		omap_start_dma(p->tx_dma->ch);
	} else {
		spin_unlock_irqrestore(&tx_rb_read_lock, flags);
	}

	return 0;
}

static int send_frame_to_fpga_finish(void)
{
	struct usrp_e_drvdata *p = dev_data.drvdata;
	unsigned long flags;

//	dma_sync_single_for_cpu(NULL, (*tx_rb.rbe)[tx_rb_read].dma_addr, SZ_2K, DMA_TO_DEVICE);

	spin_lock_irqsave(&tx_rb_read_lock, flags);
	(*p->tx_rb.rbi)[p->tx_rb_read].flags = RB_KERNEL;


	p->tx_rb_read++;
	if (p->tx_rb_read == p->rb_size.num_tx_frames)
		p->tx_rb_read = 0;

	p->tx_dma_active = 0;

	spin_unlock_irqrestore(&tx_rb_read_lock, flags);

	wake_up_interruptible(&tx_rb_space_available);

	send_frame_to_fpga_start();

	return 0;
}

static irqreturn_t space_available_irqhandler(int irq, void *dev_id)
{
	int serviced = IRQ_NONE;

	send_frame_to_fpga_start();

	serviced = IRQ_HANDLED;

	return serviced;
}

static void usrp_tx_dma_irq(int ch, u16 stat, void *data)
{
	struct usrp_e_drvdata *p = dev_data.drvdata;

	p->tx_dma_active = 0;

	send_frame_to_fpga_finish();

}

static int init_dma_controller(void)
{
	struct usrp_e_drvdata *p = dev_data.drvdata;

	p->rx_dma = kzalloc(sizeof(struct dma_data), GFP_KERNEL);
	if (!p->rx_dma) {
		printk(KERN_ERR "Failed to allocate memory for rx_dma struct.");
		return -ENOMEM;
	}

	if (omap_request_dma(OMAP_DMA_NO_DEVICE, "usrp-e-rx",
			usrp_rx_dma_irq, (void *) p->rx_dma, &p->rx_dma->ch)) {
		printk(KERN_ERR "Could not get rx DMA channel for usrp_e\n");
		return -ENOMEM;
	}
	printk(KERN_DEBUG "rx_dma->ch %d\n", p->rx_dma->ch);

	p->rx_dma->phys_from = p->mem_base;

	memset(&p->rx_dma->params, 0, sizeof(p->rx_dma->params));
	p->rx_dma->params.data_type	= OMAP_DMA_DATA_TYPE_S16;

	p->rx_dma->params.src_amode	= OMAP_DMA_AMODE_POST_INC;
	p->rx_dma->params.dst_amode	= OMAP_DMA_AMODE_POST_INC;

	p->rx_dma->params.src_start	= p->mem_base;
	p->rx_dma->params.dst_start	= p->rx_dma->phys_to;

	p->rx_dma->params.src_ei		= 1;
	p->rx_dma->params.src_fi		= 1;
	p->rx_dma->params.dst_ei		= 1;
	p->rx_dma->params.dst_fi		= 1;

	p->rx_dma->params.elem_count	= 1024;
	p->rx_dma->params.frame_count	= 1;

	p->rx_dma->params.read_prio        = DMA_CH_PRIO_HIGH;
	p->rx_dma->params.write_prio       = DMA_CH_PRIO_LOW;

	omap_set_dma_params(p->rx_dma->ch, &p->rx_dma->params);

// Play with these with a real application
	omap_set_dma_src_burst_mode(p->rx_dma->ch, OMAP_DMA_DATA_BURST_16);
	omap_set_dma_dest_burst_mode(p->rx_dma->ch, OMAP_DMA_DATA_BURST_16);
	omap_set_dma_src_data_pack(p->rx_dma->ch, 1);
	omap_set_dma_dest_data_pack(p->rx_dma->ch, 1);

#if 0 // Need to find implentations of the endian calls
	omap_set_dma_src_endian_type(rx_dma->ch, OMAP_DMA_BIG_ENDIAN);
	omap_set_dma_dst_endian_type(rx_dma->ch, OMAP_DMA_LITTLE_ENDIAN);
#endif

	p->tx_dma = kzalloc(sizeof(struct dma_data), GFP_KERNEL);
	if (!p->tx_dma) {
		printk(KERN_ERR "Failed to allocate memory for tx_dma struct.");
		return -ENOMEM;
	}

	if (omap_request_dma(OMAP_DMA_NO_DEVICE, "usrp-e-tx",
			usrp_tx_dma_irq, (void *) p->tx_dma, &p->tx_dma->ch)) {
		printk(KERN_ERR "Could not get tx DMA channel for usrp_e\n");
		return -ENOMEM;
	}

	printk(KERN_DEBUG "tx_dma->ch %d\n", p->tx_dma->ch);

	p->tx_dma->phys_from = p->mem_base;

	memset(&p->tx_dma->params, 0, sizeof(p->tx_dma->params));
	p->tx_dma->params.data_type	= OMAP_DMA_DATA_TYPE_S16;

	p->tx_dma->params.src_amode	= OMAP_DMA_AMODE_POST_INC;
	p->tx_dma->params.dst_amode	= OMAP_DMA_AMODE_POST_INC;

	p->tx_dma->params.src_start	= p->tx_dma->phys_from;
	p->tx_dma->params.dst_start	= p->mem_base;

	p->tx_dma->params.src_ei		= 1;
	p->tx_dma->params.src_fi		= 1;
	p->tx_dma->params.dst_ei		= 1;
	p->tx_dma->params.dst_fi		= 1;

	p->tx_dma->params.elem_count	= 1024;
	p->tx_dma->params.frame_count	= 1;

	p->tx_dma->params.read_prio        = DMA_CH_PRIO_LOW;
	p->tx_dma->params.write_prio       = DMA_CH_PRIO_HIGH;

	omap_set_dma_params(p->tx_dma->ch, &p->tx_dma->params);

// Play with these with a real application
	omap_set_dma_src_burst_mode(p->tx_dma->ch, OMAP_DMA_DATA_BURST_16);
	omap_set_dma_dest_burst_mode(p->tx_dma->ch, OMAP_DMA_DATA_BURST_16);
	omap_set_dma_src_data_pack(p->tx_dma->ch, 1);
	omap_set_dma_dest_data_pack(p->tx_dma->ch, 1);

	return 0;
}

static void release_dma_controller(void)
{
	struct usrp_e_drvdata *p = dev_data.drvdata;

	omap_free_dma(p->rx_dma->ch);
	omap_free_dma(p->tx_dma->ch);

	kfree(p->rx_dma);
	kfree(p->tx_dma);
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

static int alloc_ring_buffers(void)
{
	struct usrp_e_drvdata *p = dev_data.drvdata;

	if (alloc_ring_buffer(&p->tx_rb, p->rb_size.num_rx_frames, DMA_TO_DEVICE) < 0)
		return -ENOMEM;
	if (alloc_ring_buffer(&p->rx_rb, p->rb_size.num_tx_frames, DMA_FROM_DEVICE) < 0)
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


static int usrp_e_open(struct inode *inode, struct file *file)
{
	struct usrp_e_drvdata *p = dev_data.drvdata;
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

	p->tx_rb_read = 0;
	p->rx_rb_write = 0;

	p->tx_dma_active = 0;
	p->rx_dma_active = 0;
	p->shutting_down = 0;

	init_ring_buffer(&p->rx_rb, p->rb_size.num_rx_frames, RB_KERNEL, DMA_FROM_DEVICE);
	init_ring_buffer(&p->tx_rb, p->rb_size.num_tx_frames, RB_KERNEL, DMA_TO_DEVICE);

	/* Configure interrupts for GPIO pins */

	ret = request_irq(gpio_to_irq(dev_data.pdata->space_available_gpio),
			space_available_irqhandler,
		IRQF_TRIGGER_RISING, "usrp_e_space_available", NULL);

	ret = request_irq(gpio_to_irq(dev_data.pdata->data_ready_gpio),
			data_ready_irqhandler,
			IRQF_TRIGGER_RISING, "usrp_e_data_ready", NULL);

	printk(KERN_DEBUG "usrp: leaving open\n");
	return 0;
}

static int usrp_e_release(struct inode *inode, struct file *file)
{
	struct usrp_e_drvdata *p = dev_data.drvdata;

	printk(KERN_DEBUG "usrp_e release called\n");

	if (atomic_read(&use_count) != 1) {
		printk(KERN_ERR "Attempt to close usrp_e driver that is not open");
		return -ENOENT;
	}

	printk(KERN_DEBUG "Waiting for DMA to become inactive\n");
	p->shutting_down = 1;
	while (p->tx_dma_active || p->rx_dma_active)
		cpu_relax();

	/* Freeing gpio irq's */
	printk(KERN_DEBUG "Freeing gpio irq's\n");

	free_irq(gpio_to_irq(dev_data.pdata->space_available_gpio), NULL);
	free_irq(gpio_to_irq(dev_data.pdata->data_ready_gpio), NULL);

	printk(KERN_DEBUG "Freeing DMA channels\n");

	release_dma_controller();

	atomic_dec(&use_count);

	return 0;
}

static ssize_t usrp_e_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{

	return count;
}

static ssize_t usrp_e_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{

	send_frame_to_fpga_start();

	return count;
}

static loff_t usrp_e_llseek(struct file *file, loff_t offest, int orig)
{
	printk(KERN_DEBUG "usrp_e llseek called\n");

	return 0;
}

static int usrp_e_ctl16(unsigned long arg, int direction)
{
	struct usrp_e_ctl16 __user *argp = (struct usrp_e_ctl16 __user *) arg;
	int i;
	struct usrp_e_ctl16 ctl;
	struct usrp_e_drvdata *p = dev_data.drvdata;

	if (copy_from_user(&ctl, argp, sizeof(struct usrp_e_ctl16)))
		return -EFAULT;

	if (ctl.count > 10)
		return -EINVAL;

	if (direction == 0) {
		for (i = 0; i < ctl.count; i++)
			writew(ctl.buf[i], &p->ctl_addr \
				[i*2 + ctl.offset]);
	} else if (direction == 1) {
		for (i = 0; i < ctl.count; i++)
			ctl.buf[i] = readw(&p->ctl_addr \
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
	struct usrp_e_drvdata *p = dev_data.drvdata;

	if (copy_from_user(&ctl, argp, sizeof(struct usrp_e_ctl32)))
		return -EFAULT;

	if (ctl.count > 20)
		return -EINVAL;

	if (direction == 0) {
		for (i = 0; i < ctl.count; i++)
			writel(ctl.buf[i], &p->ctl_addr \
				[i*4 + ctl.offset]);
	} else if (direction == 1) {
		for (i = 0; i < ctl.count; i++)
			ctl.buf[i] = readl(&p->ctl_addr \
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
	struct usrp_e_drvdata *p = dev_data.drvdata;

	if (copy_to_user(argp, &p->rb_size, sizeof(p->rb_size)))
		return -EFAULT;

	return 0;
}


static long usrp_e_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
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
	struct usrp_e_drvdata *p = dev_data.drvdata;
	unsigned int mask = 0;
	unsigned long flags;

	poll_wait(filp, &data_received_queue, wait);
	poll_wait(filp, &tx_rb_space_available, wait);

	/* Make sure write is active (if needed) before sleeping */
	send_frame_to_fpga_start();

	/* Make sure to read in case the rx ring buffer is empty */
	get_frame_from_fpga_start();

	spin_lock_irqsave(&rx_rb_write_lock, flags);
	if (p->rx_rb_write == 0) {
		if ((*p->rx_rb.rbi)[p->rb_size.num_rx_frames - 1].flags & RB_USER)
			mask |= POLLIN | POLLRDNORM;
	} else {
		if ((*p->rx_rb.rbi)[p->rx_rb_write - 1].flags & RB_USER)
			mask |= POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&rx_rb_write_lock, flags);

	spin_lock_irqsave(&tx_rb_read_lock, flags);
	if (p->tx_rb_read == 0) {
			if ((*p->tx_rb.rbi)[p->rb_size.num_tx_frames - 1].flags & RB_KERNEL)
			mask |= POLLOUT | POLLWRNORM;
	} else {
		if ((*p->tx_rb.rbi)[p->tx_rb_read - 1].flags & RB_KERNEL)
			mask |= POLLOUT | POLLWRNORM;
	}
	spin_unlock_irqrestore(&tx_rb_read_lock, flags);

	return mask;

}

/* The mmap code is based on code in af_packet.c */

static void usrp_e_mm_open(struct vm_area_struct *vma)
{
	struct usrp_e_drvdata *p = dev_data.drvdata;

	atomic_inc(&p->mapped);
}

static void usrp_e_mm_close(struct vm_area_struct *vma)
{
	struct usrp_e_drvdata *p = dev_data.drvdata;

	atomic_dec(&p->mapped);
}

static const struct vm_operations_struct usrp_e_mmap_ops = {
	.open   = usrp_e_mm_open,
	.close  = usrp_e_mm_close,
};

static int usrp_e_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct usrp_e_drvdata *p = dev_data.drvdata;
	unsigned long size, expected_size;
	unsigned int i;
	unsigned long start;
	int err;
	struct page *page;

	if (vma->vm_pgoff)
		return -EINVAL;

	/* Verify the user will map the entire tx and rx ring buffer space */
	expected_size = (p->rb_size.num_rx_frames + p->rb_size.num_tx_frames) * (PAGE_SIZE >> 1)
		+ (p->rb_size.num_pages_rx_flags + p->rb_size.num_pages_tx_flags) * PAGE_SIZE;

	size = vma->vm_end - vma->vm_start;
	printk(KERN_DEBUG "Size = %ld, expected sixe = %ld\n", size, expected_size);

	if (size != expected_size)
		return -EINVAL;

	start = vma->vm_start;

	page = virt_to_page(p->rx_rb.rbi);
	err = vm_insert_page(vma, start, page);
	if (err)
		return -EINVAL;

	start += PAGE_SIZE;

	for (i = 0; i < p->rx_rb.num_pages; ++i) {
		struct page *page = virt_to_page((*p->rx_rb.pages)[i]);
		err = vm_insert_page(vma, start, page);
		if (err)
			return -EINVAL;

		start += PAGE_SIZE;
	}

	page = virt_to_page(p->tx_rb.rbi);
	err = vm_insert_page(vma, start, page);
	if (err)
		return -EINVAL;

	start += PAGE_SIZE;

	for (i = 0; i < p->tx_rb.num_pages; ++i) {
		struct page *page = virt_to_page((*p->tx_rb.pages)[i]);

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

static int __init usrp_e_init(void)
{
	int ret;
	struct usrp_e_drvdata *p = dev_data.drvdata;

	printk(KERN_DEBUG "usrp_e entering driver initialization\n");

	ret = platform_driver_register(&usrp_e_driver);
	if (ret) {
		printk(KERN_ERR "Failed to register driver\n");
		return -1;
	}

	if (alloc_chrdev_region(&usrp_e_dev_number, 0, 1, DEVICE_NAME) < 0) {
		printk(KERN_DEBUG "Can't register device\n");
		return -1;
	}

	usrp_e_class = class_create(THIS_MODULE, DEVICE_NAME);

	atomic_set(&p->mapped, 0);
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


	p->rb_size.num_pages_rx_flags = NUM_PAGES_RX_FLAGS;
	p->rb_size.num_rx_frames = NUM_RX_FRAMES;
	p->rb_size.num_pages_tx_flags = NUM_PAGES_TX_FLAGS;
	p->rb_size.num_tx_frames = NUM_TX_FRAMES;

	ret = alloc_ring_buffers();
	if (ret < 0)
		return ret;

	/* Initialize various DMA related flags */
	p->rx_dma_active = 0;
	p->tx_dma_active = 0;
	p->shutting_down = 0;

	printk(KERN_DEBUG "usrp_e Driver Initialized.\n");

	return 0;
}

static void __exit usrp_e_cleanup(void)
{
	struct usrp_e_drvdata *p = dev_data.drvdata;


	delete_ring_buffer(&p->tx_rb, p->rb_size.num_tx_frames, DMA_TO_DEVICE);
	delete_ring_buffer(&p->rx_rb, p->rb_size.num_rx_frames, DMA_FROM_DEVICE);

	kfree(p);

	printk(KERN_DEBUG "Leaving cleanup\n");
}

static const struct file_operations usrp_e_fops = {
	.owner		=	THIS_MODULE,
	.open		=	usrp_e_open,
	.release	=	usrp_e_release,
	.read		=	usrp_e_read,
	.write		=	usrp_e_write,
	.llseek		=	usrp_e_llseek,
	.unlocked_ioctl	=	usrp_e_ioctl,
	.poll           =       usrp_e_poll,
	.mmap           =       usrp_e_mmap,
};

static int usrp_e_probe(struct platform_device *p)
{
	/* shorten some var names */
	struct usrp_e_pdata *pdata = dev_data.pdata;
	struct usrp_e_drvdata *drvdata = dev_data.drvdata;

	dev_data.pdata = devm_kzalloc(&p->dev, sizeof(struct usrp_e_pdata), GFP_KERNEL);
	dev_data.drvdata = devm_kzalloc(&p->dev, sizeof(struct usrp_e_drvdata), GFP_KERNEL);

	dev_data.pdata = p->dev.platform_data;

	printk(KERN_DEBUG "Getting Chip Select\n");
	if (gpmc_cs_request(pdata->data_CS, SZ_2K, &drvdata->mem_base) < 0) {
		printk(KERN_ERR "Failed request for GPMC mem for usrp_e\n");
		return -1;
	}
	printk(KERN_DEBUG "Got CS%d, address = %lx\n", pdata->data_CS, drvdata->mem_base);

	if (!request_mem_region(drvdata->mem_base, SZ_2K, "usrp_e")) {
		printk(KERN_ERR "Request_mem_region failed.\n");
		gpmc_cs_free(pdata->data_CS);
		return -1;
	}

	drvdata->ioaddr = ioremap(drvdata->mem_base, SZ_2K);
	spin_lock_init(&drvdata->fpga_lock);

	if (gpmc_cs_request(pdata->control_CS, SZ_2K, &drvdata->control_mem_base) < 0) {
		printk(KERN_ERR "Failed request for GPMC control mem for usrp_e\n");
		return -1;
	}
	printk(KERN_DEBUG "Got CS%d, address = %lx\n", pdata->control_CS, drvdata->control_mem_base);

	if (!request_mem_region(drvdata->control_mem_base, SZ_2K, "usrp_e_c")) {
		printk(KERN_ERR "Request_mem_region failed.\n");
		gpmc_cs_free(pdata->control_CS);
		return -1;
	}

	drvdata->ctl_addr = ioremap_nocache(drvdata->control_mem_base, SZ_2K);


	/* Configure GPIO's */

	if (!(((gpio_request(pdata->space_available_gpio,
				    "TX_SPACE_AVAILABLE_GPIO") == 0) &&
		(gpio_direction_input(pdata->space_available_gpio) == 0)))) {
		printk(KERN_ERR "Could not claim GPIO for TX_SPACE_AVAILABLE_GPIO\n");
		return -1;
	}

	if (!(((gpio_request(pdata->data_ready_gpio, "RX_DATA_READY_GPIO") == 0) &&
		(gpio_direction_input(pdata->data_ready_gpio) == 0)))) {
		printk(KERN_ERR "Could not claim GPIO for RX_DATA_READY_GPIO\n");
		return -1;
	}

	return 0;
}

static int usrp_e_remove(struct platform_device *p)
{

	printk(KERN_DEBUG "In usrp_e_remove\n");
#if 0
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
#endif
	return 0;
}

static struct platform_driver usrp_e_driver = {
	.probe = usrp_e_probe,
	.remove = usrp_e_remove,
	.driver = {
		.name = DRVNAME,
		.owner = THIS_MODULE,
		}
};

module_init(usrp_e_init);
module_exit(usrp_e_cleanup);

MODULE_VERSION("0.3");
MODULE_ALIAS(DEVICE_NAME);
MODULE_DESCRIPTION("Ettus Research E1XX memory mapped FPGA interface");
MODULE_AUTHOR("Philip Balister <philip@opensdr.com>");
MODULE_LICENSE("GPL v2");
