#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/io.h>

#ifndef PCI_VENDOR_ID_HOLOPLOT
#define PCI_VENDOR_ID_HOLOPLOT 0x2081
#endif

#define PCI_DEVICE_ID_HOLOPLOT_JANUS 0x1401

MODULE_AUTHOR("Daniel Mack <daniel.mack@holoplot.com>");
MODULE_DESCRIPTION("Driver for Holoplot PCIe cards");
MODULE_LICENSE("GPL");

struct holoplot_card {
	struct pci_dev *pci;

	unsigned long bar;
	void __iomem *iobase;
};

static irqreturn_t holoplot_pci_interrupt(int irq, void *dev_id)
{
	struct holoplot_card *card = dev_id;
	struct device *dev = &card->pci->dev;

	dev_info(dev, "interrupt\n");

	return IRQ_HANDLED;
}

static int holoplot_pci_probe(struct pci_dev *pci,
                              const struct pci_device_id *pci_id)
{
	struct holoplot_card *card;
	struct device *dev = &pci->dev;
	int err;

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	card->pci = pci;

	err = pcim_enable_device(pci);
	if (err < 0)
		return err;

	/* check PCI availability (32bit DMA) */
	if (dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32))) {
		dev_err(dev, "error setting 32-bit DMA mask.\n");
		return -ENXIO;
	}

	err = pcim_iomap_regions(pci, 1 << 0, KBUILD_MODNAME);
	if (err < 0) {
		dev_err(dev, "error mapping PCI resources.\n");
		return err;
	}

	card->bar = pci_resource_start(pci, 0);
	card->iobase = pcim_iomap_table(pci)[0];

	pci_set_master(pci);

	if (devm_request_irq(dev, pci->irq, holoplot_pci_interrupt,
			     IRQF_SHARED, KBUILD_MODNAME, card)) {
		dev_err(dev, "cannot obtain IRQ %d\n", pci->irq);
		return -EBUSY;
	}

	dev_info(&pci->dev, "Holoplot PCIe card probed. bar=%08lx iobase=%px, irq=%d\n",
		 card->bar, card->iobase, pci->irq);

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