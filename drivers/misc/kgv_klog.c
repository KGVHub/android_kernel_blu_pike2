/*
 * kgv_klog - debug-only copy of the kernel log at /proc/kgv_klog
 *
 * On a "user" Android build SELinux does not let adb shell read the kernel
 * log (klogctl/syslog_read is denied), which makes driver bring-up blind.
 * Plain /proc files get the generic "proc" label, which adb shell may read,
 * so expose a snapshot of the log buffer there.
 *
 * This deliberately bypasses dmesg restrictions. Only enable it in debug
 * builds; never ship it.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kmsg_dump.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#define KGV_KLOG_SIZE	(1 << CONFIG_LOG_BUF_SHIFT)

struct kgv_klog_snap {
	size_t len;
	char buf[];
};

static int kgv_klog_open(struct inode *inode, struct file *file)
{
	struct kmsg_dumper dumper = { .active = true };
	struct kgv_klog_snap *snap;

	snap = vmalloc(sizeof(*snap) + KGV_KLOG_SIZE);
	if (!snap)
		return -ENOMEM;

	kmsg_dump_rewind(&dumper);
	kmsg_dump_get_buffer(&dumper, true, snap->buf, KGV_KLOG_SIZE, &snap->len);
	file->private_data = snap;
	return 0;
}

static ssize_t kgv_klog_read(struct file *file, char __user *ubuf,
			     size_t count, loff_t *ppos)
{
	struct kgv_klog_snap *snap = file->private_data;

	return simple_read_from_buffer(ubuf, count, ppos, snap->buf, snap->len);
}

static int kgv_klog_release(struct inode *inode, struct file *file)
{
	vfree(file->private_data);
	return 0;
}

static const struct file_operations kgv_klog_fops = {
	.open = kgv_klog_open,
	.read = kgv_klog_read,
	.llseek = default_llseek,
	.release = kgv_klog_release,
};

static int __init kgv_klog_init(void)
{
	if (!proc_create("kgv_klog", 0444, NULL, &kgv_klog_fops))
		return -ENOMEM;
	pr_warn("kgv_klog: kernel log exposed at /proc/kgv_klog (debug build)\n");
	return 0;
}
device_initcall(kgv_klog_init);
