#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

extern const struct seq_operations cpumidr_op;
static int cpumidr_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &cpumidr_op);
}

static const struct file_operations proc_cpumidr_operations = {
	.open		= cpumidr_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int __init proc_cpumidr_init(void)
{
	proc_create("cpumidr", 0, NULL, &proc_cpumidr_operations);
	return 0;
}
fs_initcall(proc_cpumidr_init);
