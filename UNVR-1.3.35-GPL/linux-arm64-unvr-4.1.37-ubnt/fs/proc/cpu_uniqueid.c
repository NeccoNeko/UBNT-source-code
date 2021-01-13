#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/io.h>

extern const struct seq_operations cpu_uniqueid_op;

static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void c_stop(struct seq_file *m, void *v)
{
}

static int c_midr_show(struct seq_file *m, void *v)
{
    unsigned long *cpu_uniqueid1 = ioremap(0xfd896080, 32);
    unsigned long *cpu_uniqueid2 = ioremap(0xfd896084, 32);
    seq_printf(m, "0x%x%x\n", readl(cpu_uniqueid1), readl(cpu_uniqueid2));
    iounmap(cpu_uniqueid1);
    iounmap(cpu_uniqueid2);
    return 0;
}

const struct seq_operations cpu_uniqueid_op = {
	.start	= c_start,
	.next	= c_next,
	.stop	= c_stop,
	.show	= c_midr_show
};

static int cpu_uniqueid_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &cpu_uniqueid_op);
}

static const struct file_operations proc_cpu_uniqueid_operations = {
	.open		= cpu_uniqueid_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int __init proc_cpu_uniqueid_init(void)
{
	proc_create("cpu_uniqueid", 0, NULL, &proc_cpu_uniqueid_operations);
	return 0;
}
fs_initcall(proc_cpu_uniqueid_init);
