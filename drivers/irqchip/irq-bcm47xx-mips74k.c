// SPDX-License-Identifier: GPL-2.0-only
/*
 * Broadcom BCM47xx MIPS74K core interrupt router
 *
 * On BCMA based BCM47xx SoCs, the MIPS74K core routes the backplane
 * interrupt flags of the other cores to the CPU interrupt lines. Each
 * INTMASK register selects the flags that raise one CPU line, and
 * INTSTATUS holds the state of all flags.
 *
 * The first parent line is shared by any number of flags. The others get
 * one flag each, handed out in the order the flags are mapped, so that the
 * busy cores do not have to share a line.
 */

#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#define INTSTATUS		0x10
#define INTMASK(line)		(0x14 + 4 * (line))

#define NR_FLAGS		32
/* INTMASK(5) is reserved for the CPU timer */
#define MAX_LINES		5
#define SHARED_LINE		0
#define NO_FLAG			-1

struct bcm47xx_intc;

struct bcm47xx_intc_line {
	struct bcm47xx_intc *intc;
	unsigned int index;
	/* flag owning a dedicated line, NO_FLAG if free */
	int flag;
	/* shadow of INTMASK(index) */
	u32 mask;
};

struct bcm47xx_intc {
	void __iomem *base;
	struct irq_domain *domain;
	raw_spinlock_t lock;
	unsigned int nr_lines;
	struct bcm47xx_intc_line lines[MAX_LINES];
	u8 flag_line[NR_FLAGS];
};

static void bcm47xx_intc_handle(struct irq_desc *desc)
{
	struct bcm47xx_intc_line *line = irq_desc_get_handler_data(desc);
	struct bcm47xx_intc *intc = line->intc;
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long pending;
	unsigned int hwirq;

	chained_irq_enter(chip, desc);

	if (line->index != SHARED_LINE) {
		/* only the owning flag can raise a dedicated line */
		if (line->flag != NO_FLAG)
			generic_handle_domain_irq(intc->domain, line->flag);
	} else {
		pending = readl(intc->base + INTSTATUS) & READ_ONCE(line->mask);
		for_each_set_bit(hwirq, &pending, NR_FLAGS)
			generic_handle_domain_irq(intc->domain, hwirq);
	}

	chained_irq_exit(chip, desc);
}

static void bcm47xx_intc_update(struct irq_data *d, bool enable)
{
	struct bcm47xx_intc *intc = irq_data_get_irq_chip_data(d);
	struct bcm47xx_intc_line *line = &intc->lines[intc->flag_line[d->hwirq]];

	guard(raw_spinlock)(&intc->lock);

	if (enable)
		line->mask |= BIT(d->hwirq);
	else
		line->mask &= ~BIT(d->hwirq);
	writel(line->mask, intc->base + INTMASK(line->index));
}

static void bcm47xx_intc_mask(struct irq_data *d)
{
	bcm47xx_intc_update(d, false);
}

static void bcm47xx_intc_unmask(struct irq_data *d)
{
	bcm47xx_intc_update(d, true);
}

static struct irq_chip bcm47xx_intc_chip = {
	.name		= "BCM47XX",
	.irq_mask	= bcm47xx_intc_mask,
	.irq_unmask	= bcm47xx_intc_unmask,
	.flags		= IRQCHIP_SKIP_SET_WAKE,
};

static int bcm47xx_intc_map(struct irq_domain *d, unsigned int virq,
			    irq_hw_number_t hwirq)
{
	struct bcm47xx_intc *intc = d->host_data;
	unsigned int i, line = SHARED_LINE;

	scoped_guard(raw_spinlock_irqsave, &intc->lock) {
		for (i = SHARED_LINE + 1; i < intc->nr_lines; i++) {
			if (intc->lines[i].flag == NO_FLAG) {
				intc->lines[i].flag = hwirq;
				line = i;
				break;
			}
		}
		intc->flag_line[hwirq] = line;
	}

	irq_set_chip_and_handler(virq, &bcm47xx_intc_chip, handle_level_irq);
	irq_set_chip_data(virq, intc);

	return 0;
}

static void bcm47xx_intc_unmap(struct irq_domain *d, unsigned int virq)
{
	struct irq_data *data = irq_get_irq_data(virq);
	struct bcm47xx_intc *intc = d->host_data;
	struct bcm47xx_intc_line *line;

	bcm47xx_intc_mask(data);

	guard(raw_spinlock_irqsave)(&intc->lock);

	line = &intc->lines[intc->flag_line[data->hwirq]];
	if (line->index != SHARED_LINE)
		line->flag = NO_FLAG;
	intc->flag_line[data->hwirq] = SHARED_LINE;
}

static const struct irq_domain_ops bcm47xx_intc_domain_ops = {
	.xlate	= irq_domain_xlate_onecell,
	.map	= bcm47xx_intc_map,
	.unmap	= bcm47xx_intc_unmap,
};

static int __init bcm47xx_intc_of_init(struct device_node *node,
				       struct device_node *parent)
{
	unsigned int parent_irq[MAX_LINES];
	struct bcm47xx_intc *intc;
	unsigned int i;
	int nr_lines, ret;

	nr_lines = of_irq_count(node);
	if (nr_lines < 1 || nr_lines > MAX_LINES) {
		pr_err("%pOF: invalid number of parent interrupts\n", node);
		return -EINVAL;
	}

	intc = kzalloc_obj(*intc);
	if (!intc)
		return -ENOMEM;

	raw_spin_lock_init(&intc->lock);
	intc->nr_lines = nr_lines;

	intc->base = of_iomap(node, 0);
	if (!intc->base) {
		pr_err("%pOF: failed to map registers\n", node);
		ret = -ENOMEM;
		goto err_free;
	}

	/* drop the routing left behind by the bootloader */
	for (i = 0; i < MAX_LINES; i++)
		writel(0, intc->base + INTMASK(i));

	for (i = 0; i < nr_lines; i++) {
		parent_irq[i] = irq_of_parse_and_map(node, i);
		if (!parent_irq[i]) {
			pr_err("%pOF: failed to map parent interrupt %u\n",
			       node, i);
			ret = -EINVAL;
			goto err_unmap;
		}

		intc->lines[i].intc = intc;
		intc->lines[i].index = i;
		intc->lines[i].flag = NO_FLAG;
	}

	intc->domain = irq_domain_create_linear(of_fwnode_handle(node), NR_FLAGS,
						&bcm47xx_intc_domain_ops, intc);
	if (!intc->domain) {
		ret = -ENOMEM;
		goto err_unmap;
	}

	for (i = 0; i < nr_lines; i++)
		irq_set_chained_handler_and_data(parent_irq[i],
						 bcm47xx_intc_handle,
						 &intc->lines[i]);

	return 0;

err_unmap:
	while (i--)
		irq_dispose_mapping(parent_irq[i]);
	iounmap(intc->base);
err_free:
	kfree(intc);
	return ret;
}
IRQCHIP_DECLARE(bcm47xx_mips74k_intc, "brcm,bcm4718-mips74k-intc",
		bcm47xx_intc_of_init);
