#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <asm-generic/io.h>
#include <asm/io.h>

#ifndef PCI_VENDOR_ID_HOLOPLOT
#define PCI_VENDOR_ID_HOLOPLOT 0x2081
#endif

#define PCI_DEVICE_ID_HOLOPLOT_JANUS 0x2401

struct holoplot_card {
	struct pci_dev *pci;

	void __iomem *bar0;
	void __iomem *bar4;

	void *dma_buf;

	struct miscdevice misc;

	int foo;
};

static irqreturn_t holoplot_pci_interrupt(int irq, void *dev_id)
{
	struct holoplot_card *card = dev_id;
	struct device *dev = &card->pci->dev;
	u32 status;

	status = readl(card->bar0 + 0x64);
	writel(status, card->bar0 + 0x64);

	if (status & 0x8) {
		dev_info(dev, "interrupt 0: %08x\n", status);


	}

	status = readl(card->bar0 + 0xe4);
	writel(0x8, card->bar0 + 0xe4);

	// if (status & 0x8)
	// 	dev_info(dev, "interrupt 1: %08x\n", status);

	return IRQ_HANDLED;
}

#define to_card(x) container_of(x, struct holoplot_card, misc)

static ssize_t janus_misc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct holoplot_card *card = to_card(file->private_data);

	dev_info(&card->pci->dev, "DMA buffer content 0x%llx\n", *(u64*) card->dma_buf);

	return 0;
}

static ssize_t janus_misc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	struct holoplot_card *card = to_card(file->private_data);
	u64 v;

	card->foo++;
	v = 0xdead0000 + card->foo;

	printk(KERN_ERR "XXXXXXXXX janus_misc_read -> %08llx\n", v);

	writeq(v, card->bar4);

	return count;
}

static const struct file_operations janus_misc_fops =
{
	.read = janus_misc_read,
	.write = janus_misc_write,
};

static void janus_misc_deregister(void *misc)
{
	misc_deregister(misc);
}

static int holoplot_pci_probe(struct pci_dev *pci,
			      const struct pci_device_id *pci_id)
{
	struct holoplot_card *card;
	struct device *dev = &pci->dev;
	dma_addr_t dma_addr;
	int ret;

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	card->pci = pci;

	ret = pcim_enable_device(pci);
	if (ret < 0)
		return ret;

	ret = pci_enable_msi(pci);
	if (ret < 0) {
		dev_err(dev, "error enabling MSI\n");
		return ret;
	}

	if (dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64))) {
		dev_err(dev, "error setting 64-bit DMA mask.\n");
		return -ENXIO;
	}

	ret = pcim_iomap_regions(pci, BIT(0)|BIT(2)|BIT(4), KBUILD_MODNAME);
	if (ret < 0) {
		dev_err(dev, "error mapping PCI resources.\n");
		return ret;
	}

	card->bar0 = pcim_iomap_table(pci)[0];
	card->bar4 = pcim_iomap_table(pci)[4];

	pci_set_master(pci);

	if (devm_request_irq(dev, pci->irq, holoplot_pci_interrupt,
			     IRQF_SHARED, KBUILD_MODNAME, card)) {
		dev_err(dev, "cannot obtain IRQ %d\n", pci->irq);
		return -EBUSY;
	}

	card->dma_buf = dmam_alloc_coherent(dev, SZ_16M, &dma_addr, GFP_KERNEL);
	if (!card->dma_buf) {
		dev_err(dev, "cannot allocate DMA buffer\n");
		return -ENOMEM;
	}

	// write the DMA buffer address to the scratch pad register
	writel(lower_32_bits(dma_addr), card->bar0 + 0xd0);
	writel(upper_32_bits(dma_addr), card->bar0 + 0xd4);

	wmb();

	writel(0xcccccccc, card->bar0 + 0xdc);

	card->misc.minor = MISC_DYNAMIC_MINOR;
	card->misc.fops = &janus_misc_fops;
	card->misc.name = KBUILD_MODNAME;

	ret = misc_register(&card->misc);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(dev, janus_misc_deregister, &card->misc);
	if (ret < 0)
		return ret;

	dev_info(&pci->dev, "Holoplot PCIe card probed. bar0=%px, bar4=%px irq=%d\n",
		 card->bar0, card->bar4, pci->irq);

	dev_info(&pci->dev, "DMA buffer at physical 0x%llx virtual 0x%px -- content 0x%llx\n", dma_addr, card->dma_buf, *(u64*) card->dma_buf);

	return 0;
}

static const struct pci_device_id holoplot_pci_ids[] = {
        { PCI_DEVICE(PCI_VENDOR_ID_HOLOPLOT, PCI_DEVICE_ID_HOLOPLOT_JANUS) },
        { 0, },
};
MODULE_DEVICE_TABLE(pci, holoplot_pci_ids);

static struct pci_driver holoplot_pci_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= holoplot_pci_ids,
	.probe		= holoplot_pci_probe,
};

module_pci_driver(holoplot_pci_driver);

MODULE_AUTHOR("Daniel Mack <daniel.mack@holoplot.com>");
MODULE_DESCRIPTION("Driver for Holoplot PCIe cards");
MODULE_LICENSE("GPL");