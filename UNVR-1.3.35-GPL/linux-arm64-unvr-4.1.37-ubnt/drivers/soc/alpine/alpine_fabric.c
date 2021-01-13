/*
 * Annapurna labs fabric.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/irqchip/chained_irq.h>
#include <alpine_fabric.h>
#include <al_hal_nb_regs.h>

static void __iomem *nb_service_base_address;

static struct of_device_id of_nb_table[] = {
	{.compatible = "annapurna-labs,al-nb-service"},
	{ /* end of list */ },
};

struct sys_fabric_irq_struct {
	unsigned int		idx;
	void __iomem		*regs_base;
	unsigned int		irq_cause_base;
	struct irq_chip_generic	*irq_gc;
};

static struct sys_fabric_irq_struct	sf_irq_arr[AL_FABRIC_INSTANCE_N];

static void sf_irq_handler(unsigned irq, struct irq_desc *desc)
{
	unsigned long pending, mask;
	int offset;
	struct sys_fabric_irq_struct *chip = irq_desc_get_handler_data(desc);
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	struct al_nb_regs *nb_regs = chip->regs_base;

	chained_irq_enter(irqchip, desc);

	mask = chip->irq_gc->mask_cache;
	pending = readl(&nb_regs->global.nb_int_cause) & mask;

	/* deassert pending edge-triggered irqs */
	writel(~(pending & ~NB_GLOBAL_NB_INT_CAUSE_LEVEL_IRQ_MASK),
			&nb_regs->global.nb_int_cause);

	/* handle pending irqs */
	if (likely(pending)) {
		int fabric_irq_base = al_fabric_get_cause_irq(chip->idx, 0);
		for_each_set_bit(offset, &pending, AL_FABRIC_IRQ_N)
			generic_handle_irq(fabric_irq_base + offset);
	}

	/* deassert pending level-triggered irqs */
	writel(~(pending & NB_GLOBAL_NB_INT_CAUSE_LEVEL_IRQ_MASK),
			&nb_regs->global.nb_int_cause);

	chained_irq_exit(irqchip, desc);
}

static void init_sf_irq_gc(struct sys_fabric_irq_struct *sfi)
{
	struct irq_chip_type *ct;

	sfi->irq_gc = irq_alloc_generic_chip("alpine_sf_irq", 1,
			sfi->irq_cause_base, sfi->regs_base, handle_simple_irq);
	sfi->irq_gc->private = sfi;

	ct = sfi->irq_gc->chip_types;
	ct->chip.irq_mask = irq_gc_mask_clr_bit;
	ct->chip.irq_unmask = irq_gc_mask_set_bit;
	ct->regs.mask = offsetof(struct al_nb_regs, cpun_config_status[sfi->idx].local_cause_mask);

	/* clear the no request field so irq can be requested */
	irq_setup_generic_chip(sfi->irq_gc, IRQ_MSK(AL_FABRIC_IRQ_N),
			IRQ_GC_INIT_MASK_CACHE, IRQ_NOREQUEST, 0);
}

static int init_sf_irq_struct(struct sys_fabric_irq_struct *sfi_arr,
		unsigned int idx, void __iomem *regs_base)
{
	int err;

	pr_debug("[%s] entered with idx = %d, regs_base = %p\n",
			__func__, idx, regs_base);
	sfi_arr[idx].idx = idx;
	sfi_arr[idx].regs_base = regs_base;

	/* allocate irq descriptors for the cause interrupts */
	err = irq_alloc_descs(-1, 0, AL_FABRIC_IRQ_N, -1);
	if (err < 0) {
		pr_err("[%s] Failed to allocate IRQ descriptors\n", __func__);
		return err;
	}

	sfi_arr[idx].irq_cause_base = err;
	init_sf_irq_gc(&sfi_arr[idx]);

	return 0;
}

int al_fabric_get_cause_irq(unsigned int idx, int irq)
{
	return sf_irq_arr[idx].irq_cause_base + irq;
}

void __iomem *al_fabric_reg_base_get(void)
{
	return nb_service_base_address;
}

int __init al_fabric_init(void)
{
	struct device_node *nb_node;
	int nb_serv_irq[AL_FABRIC_INSTANCE_N];
	int i, err;

	pr_info("Initializing System Fabric\n");

	nb_node = of_find_matching_node(NULL, of_nb_table);

	nb_service_base_address = of_iomap(nb_node, 0);
	if (!nb_service_base_address)
		return 0;

	for (i = 0 ; i < AL_FABRIC_INSTANCE_N ; ++i) {
		err = init_sf_irq_struct(sf_irq_arr, i, nb_service_base_address);
		if (err < 0) {
			pr_err("[%s] Failed to initialize sys-fabric irq struct\n", __func__);
			return err;
		}
		nb_serv_irq[i] = irq_of_parse_and_map(nb_node, i);
		irq_set_chained_handler(nb_serv_irq[i], sf_irq_handler);
		err = irq_set_handler_data(nb_serv_irq[i], &sf_irq_arr[i]);
		if (err < 0) {
			pr_err("[%s] Failed to set irq handler data\n" , __func__);
			return err;
		}
	}

	return 0;
}
arch_initcall(al_fabric_init);

