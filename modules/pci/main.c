#include <linux/compiler.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/hwdep.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/pcm.h>

#include "holoplot_pci.h"

#ifndef PCI_VENDOR_ID_HOLOPLOT
#define PCI_VENDOR_ID_HOLOPLOT 0x2081
#endif

#define PCI_DEVICE_ID_holoplot_pci 0x2401

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;
module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "Index value for the Holoplut PCIe soundcard.");

static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;
module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string for the Holoplut PCIe soundcard.");

struct holoplot_pci_priv {
	struct pci_dev *pci;

	void __iomem *bar0; /* Xilinx DMA and PCIe bridge register spaces */
	void __iomem *bar2; /* Device info and control */
	void __iomem *bar4; /* Playback audio data */

	struct snd_pcm *pcm;
	struct snd_pcm_substream *playback;
	struct snd_pcm_substream *capture;

	int irq_count;
};

static u64 holoplot_pci_read_cc_reg(struct holoplot_pci_priv *priv, u64 reg)
{
	return readq(priv->bar4 + reg);
}

static void holoplot_pci_write_cc_reg(struct holoplot_pci_priv *priv, u64 reg, u64 val)
{
	writeq(val, priv->bar4 + reg);
}

static void holoplot_pci_write_reg(struct holoplot_pci_priv *priv, u32 reg, u32 val)
{
	writel(val, priv->bar0 + reg);
}

static u32 holoplot_pci_read_reg(struct holoplot_pci_priv *priv, u32 reg)
{
	return readl(priv->bar0 + reg);
}

static void holoplot_pci_write_ingress_reg(struct holoplot_pci_priv *priv, u32 reg, u32 val)
{
	u32 base = REG_AXIPCIE_INGRESS_BASE(2);

	holoplot_pci_write_reg(priv, base + reg, val);
}

static void holoplot_pci_write_egress_reg(struct holoplot_pci_priv *priv, u32 reg, u32 val)
{
	u32 base = REG_AXIPCIE_EGRESS_BASE(0);

	holoplot_pci_write_reg(priv, base + reg, val);
}

static irqreturn_t holoplot_pci_interrupt(int irq, void *dev_id)
{
	struct holoplot_pci_priv *priv = dev_id;
	struct device *dev = &priv->pci->dev;
	u32 status;

	status = holoplot_pci_read_reg(priv, REG_DMA_IRQ_0_STATUS);
	holoplot_pci_write_reg(priv, REG_DMA_IRQ_0_STATUS, status);

	if (status & 0x8) {
		if (priv->playback)
			snd_pcm_period_elapsed(priv->playback);

		if (priv->capture) {
			snd_pcm_period_elapsed(priv->capture);
		}

		priv->irq_count++;

		if (priv->irq_count % 256 == 0)
			dev_info(dev, "interrupt 0: status 0x%08x, counter %d\n", status, priv->irq_count);
	}

	status = holoplot_pci_read_reg(priv, REG_DMA_IRQ_1_STATUS);
	holoplot_pci_write_reg(priv, REG_DMA_IRQ_1_STATUS, status);

	if (status & 0x8) {
		if (priv->capture)
			snd_pcm_period_elapsed(priv->capture);
	}

	// if (status & 0x8)
	// 	dev_info(dev, "interrupt 1: %08x\n", status);

	return IRQ_HANDLED;
}

static const struct snd_pcm_hardware holoplot_pci_playback_hw = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats		= SNDRV_PCM_FMTBIT_S32_LE,
	.rates			= SNDRV_PCM_RATE_48000,
	.rate_min		= 48000,
	.rate_max		= 48000,
	.channels_min		= 256,
	.channels_max		= 256,
	.buffer_bytes_max	= BUFFER_BYTES_MAX,
	.period_bytes_min	= PERIOD_BYTES_MIN,
	.period_bytes_max	= PERIOD_BYTES_MAX,
	.periods_min		= PERIODS_MIN,
	.periods_max		= PERIODS_MAX,
};

static const struct snd_pcm_hardware holoplot_pci_capture_hw = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats		= SNDRV_PCM_FMTBIT_S32_LE,
	.rates			= SNDRV_PCM_RATE_48000,
	.rate_min		= 48000,
	.rate_max		= 48000,
	.channels_min		= 256,
	.channels_max		= 256,
	.buffer_bytes_max	= BUFFER_BYTES_MAX,
	.period_bytes_min	= PERIOD_BYTES_MIN,
	.period_bytes_max	= PERIOD_BYTES_MAX,
	.periods_min		= PERIODS_MIN,
	.periods_max		= PERIODS_MAX,
};

static int holoplot_pci_pcm_constraint(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	ret = snd_pcm_hw_constraint_step(runtime, 0,
					 SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 32);
	if (ret)
		return ret;

	ret = snd_pcm_hw_constraint_step(runtime, 0,
					 SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 32);
	if (ret)
		return ret;

	return snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
}

static int
holoplot_pci_playback_open(struct snd_pcm_substream *substream)
{
	struct holoplot_pci_priv *priv = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	priv->playback = substream;
	runtime->private_data = priv;
	runtime->hw = holoplot_pci_playback_hw;

	ret = holoplot_pci_pcm_constraint(substream);
	if (ret)
		return ret;

	return 0;
}

static int
holoplot_pci_capture_open(struct snd_pcm_substream *substream)
{
	struct holoplot_pci_priv *priv = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	priv->capture = substream;
	runtime->private_data = priv;
	runtime->hw = holoplot_pci_capture_hw;

	ret = holoplot_pci_pcm_constraint(substream);
	if (ret)
		return ret;

	return 0;
}

static int
holoplot_pci_playback_close(struct snd_pcm_substream *substream)
{
	struct holoplot_pci_priv *priv = snd_pcm_substream_chip(substream);

	priv->playback = NULL;

	return 0;
}

static int
holoplot_pci_capture_close(struct snd_pcm_substream *substream)
{
	struct holoplot_pci_priv *priv = snd_pcm_substream_chip(substream);

	priv->capture = NULL;

	return 0;
}

static int
holoplot_pci_playback_prepare(struct snd_pcm_substream *substream)
{
	struct holoplot_pci_priv *priv = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	holoplot_pci_write_cc_reg(priv, REG_CC_PLAYBACK_CONTROL,
				  REG_CC_PLAYBACK_CONTROL_RESET);

	holoplot_pci_write_ingress_reg(priv, REG_AXIPCIE_INGRESS_SRC_ADDR_LO,
				       lower_32_bits(runtime->dma_addr));
	holoplot_pci_write_ingress_reg(priv, REG_AXIPCIE_INGRESS_SRC_ADDR_HI,
				       upper_32_bits(runtime->dma_addr));

	return 0;
}

static int
holoplot_pci_capture_prepare(struct snd_pcm_substream *substream)
{
	struct holoplot_pci_priv *priv = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	holoplot_pci_write_cc_reg(priv, REG_CC_CAPTURE_CONTROL,
				  REG_CC_CAPTURE_CONTROL_RESET);

	holoplot_pci_write_egress_reg(priv, REG_AXIPCIE_EGRESS_DST_ADDR_LO,
				      lower_32_bits(runtime->dma_addr));
	holoplot_pci_write_egress_reg(priv, REG_AXIPCIE_EGRESS_DST_ADDR_HI,
				      upper_32_bits(runtime->dma_addr));

	return 0;
}

static int
holoplot_pci_playback_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct holoplot_pci_priv *priv = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	holoplot_pci_write_cc_reg(priv, REG_CC_PLAYBACK_PERIOD_SIZE,
				  params_period_bytes(params));
	holoplot_pci_write_cc_reg(priv, REG_CC_PLAYBACK_BUFFER_SIZE,
				  runtime->dma_bytes);

	return 0;
}

static int
holoplot_pci_capture_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params)
{
	struct holoplot_pci_priv *priv = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	holoplot_pci_write_cc_reg(priv, REG_CC_CAPTURE_PERIOD_SIZE,
				  params_period_bytes(params));
	holoplot_pci_write_cc_reg(priv, REG_CC_CAPTURE_BUFFER_SIZE,
				  runtime->dma_bytes);

	return 0;
}

static int
holoplot_pci_playback_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct holoplot_pci_priv *priv = snd_pcm_substream_chip(substream);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		holoplot_pci_write_cc_reg(priv, REG_CC_PLAYBACK_CONTROL,
					  REG_CC_PLAYBACK_CONTROL_RUN);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		holoplot_pci_write_cc_reg(priv, REG_CC_PLAYBACK_CONTROL, 0);
		break;

	default:
		return  -EINVAL;
	}

	return 0;
}

static int
holoplot_pci_capture_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct holoplot_pci_priv *priv = snd_pcm_substream_chip(substream);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		holoplot_pci_write_cc_reg(priv, REG_CC_CAPTURE_CONTROL,
					  REG_CC_CAPTURE_CONTROL_RUN);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		holoplot_pci_write_cc_reg(priv, REG_CC_CAPTURE_CONTROL, 0);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static snd_pcm_uframes_t
holoplot_pci_playback_pointer(struct snd_pcm_substream *substream)
{
	struct holoplot_pci_priv *priv = snd_pcm_substream_chip(substream);
	u64 pos = holoplot_pci_read_cc_reg(priv, REG_CC_PLAYBACK_POSITION);

	return bytes_to_frames(substream->runtime, pos);
}

static snd_pcm_uframes_t
holoplot_pci_capture_pointer(struct snd_pcm_substream *substream)
{
	struct holoplot_pci_priv *priv = snd_pcm_substream_chip(substream);
	u64 pos = holoplot_pci_read_cc_reg(priv, REG_CC_CAPTURE_POSITION);

	return bytes_to_frames(substream->runtime, pos);
}

static const struct snd_pcm_ops holoplot_pci_playback_ops = {
	.open		= holoplot_pci_playback_open,
	.close		= holoplot_pci_playback_close,
	.prepare	= holoplot_pci_playback_prepare,
	.hw_params	= holoplot_pci_playback_hw_params,
	.trigger	= holoplot_pci_playback_trigger,
	.pointer	= holoplot_pci_playback_pointer,
};

static const struct snd_pcm_ops holoplot_pci_capture_ops = {
	.open		= holoplot_pci_capture_open,
	.close		= holoplot_pci_capture_close,
	.prepare	= holoplot_pci_capture_prepare,
	.hw_params	= holoplot_pci_capture_hw_params,
	.trigger	= holoplot_pci_capture_trigger,
	.pointer	= holoplot_pci_capture_pointer,
};

/* Sysfs attributes*/

static ssize_t device_id_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct holoplot_pci_priv *priv = dev_get_drvdata(dev);
	u64 v;
	int i;

	v = holoplot_pci_read_cc_reg(priv, REG_BAR2_DEVICE_ID);

	for (i = 0; i < 8; i++)
		buf[i] = (int) (v >> (56 - i * 8)) & 0xff;

	return 8;
}
static DEVICE_ATTR_RO(device_id);

static ssize_t fpga_design_type_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct holoplot_pci_priv *priv = dev_get_drvdata(dev);
	u64 v;

	v = holoplot_pci_read_cc_reg(priv, REG_BAR2_DESIGN_TYPE_AND_VERSION);
	v >>= DESIGN_TYPE_SHIFT;
	v &= DESIGN_TYPE_MASK;

	return sysfs_emit(buf, "%08x\n", (u32) v);
}
static DEVICE_ATTR_RO(fpga_design_type);

static ssize_t fpga_design_version_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct holoplot_pci_priv *priv = dev_get_drvdata(dev);
	u64 v;

	v = holoplot_pci_read_cc_reg(priv, REG_BAR2_DESIGN_TYPE_AND_VERSION);
	v >>= DESIGN_VERSION_SHIFT;
	v &= DESIGN_VERSION_MASK;

	return sysfs_emit(buf, "%08x\n", (u32) v);
}
static DEVICE_ATTR_RO(fpga_design_version);

static struct attribute *holoplot_pci_dev_attrs[] = {
	&dev_attr_device_id.attr,
	&dev_attr_fpga_design_type.attr,
	&dev_attr_fpga_design_version.attr,
	NULL,
};

static const struct attribute_group holoplot_pci_dev_attr_group = {
	.attrs = holoplot_pci_dev_attrs,
};

static const struct attribute_group *holoplot_pci_dev_attr_groups[] = {
	&holoplot_pci_dev_attr_group,
	NULL,
};

static int holoplot_pci_probe(struct pci_dev *pci,
			      const struct pci_device_id *pci_id)
{
	struct holoplot_pci_priv *priv;
	struct device *dev = &pci->dev;
	struct snd_hwdep *hwdep;
	struct snd_card *card;
	dma_addr_t dma_addr;
	unsigned long bar;
	static int devno;
	int ret;

	ret = snd_devm_card_new(&pci->dev, index[devno], id[devno], THIS_MODULE,
				sizeof(*priv), &card);
	if (ret < 0)
		return ret;

	strcpy(card->driver, KBUILD_MODNAME);
	strcpy(card->shortname, "Holoplot PCIe");

	priv = card->private_data;

	priv->pci = pci;

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

	priv->bar0 = pcim_iomap_table(pci)[0];
	priv->bar2 = pcim_iomap_table(pci)[2];
	priv->bar4 = pcim_iomap_table(pci)[4];

	pci_set_master(pci);

	if (devm_request_irq(dev, pci->irq, holoplot_pci_interrupt,
			     IRQF_SHARED, KBUILD_MODNAME, priv)) {
		dev_err(dev, "cannot obtain IRQ %d\n", pci->irq);
		return -EBUSY;
	}



	writel(0xcccccccc, priv->bar0 + 0xdc);

	// write the DMA buffer address to the scratch pad register
	writel(lower_32_bits(dma_addr), priv->bar0 + 0xd0);
	writel(upper_32_bits(dma_addr), priv->bar0 + 0xd4);




	/*
	 * Ingress is used for playback
	 *
	 * We need to configure the source address of the DMA engine so it points
	 * to the hardware address that was assigned to BAR2.
	 */
	bar = pci_resource_start(pci, 4); // FIXME
	holoplot_pci_write_ingress_reg(priv, REG_AXIPCIE_INGRESS_SRC_ADDR_LO,
				       lower_32_bits(bar));
	holoplot_pci_write_ingress_reg(priv, REG_AXIPCIE_INGRESS_SRC_ADDR_HI,
				       upper_32_bits(bar));

	/*
	 * Egress is used for capture
	 *
	 * We need to configure the source address of the DMA engine so it points
	 * to the hardware address inside the card's memory space. The data will
	 * be written to the ALSA DMA buffer without a dedicated BAR.
	 */
	holoplot_pci_write_ingress_reg(priv, REG_AXIPCIE_EGRESS_SRC_ADDR_LO,
				       lower_32_bits(REMOTE_EGRESS_SOURCE));
	holoplot_pci_write_ingress_reg(priv, REG_AXIPCIE_EGRESS_SRC_ADDR_HI,
				       upper_32_bits(REMOTE_EGRESS_SOURCE));

	/* The internal capture source address is always the same */
	holoplot_pci_write_cc_reg(priv, REG_CC_CAPTURE_SRC_ADDR,
				  REMOTE_EGRESS_SOURCE);

	/* PCM */
	ret = snd_pcm_new(card, card->driver, 0, 1, 1, &priv->pcm);
	if (ret < 0)
		return ret;

	snd_pcm_set_ops(priv->pcm, SNDRV_PCM_STREAM_PLAYBACK,
			&holoplot_pci_playback_ops);
	snd_pcm_set_ops(priv->pcm, SNDRV_PCM_STREAM_CAPTURE,
			&holoplot_pci_capture_ops);

	priv->pcm->private_data = priv;
	priv->pcm->info_flags = 0;
	strcpy(priv->pcm->name, card->shortname);

	snd_pcm_set_managed_buffer_all(priv->pcm, SNDRV_DMA_TYPE_DEV, dev,
				       BUFFER_BYTES_MAX / 2, BUFFER_BYTES_MAX);

	/* HW dependent interface */
	ret = snd_hwdep_new(card, KBUILD_MODNAME, 0, &hwdep);
	if (ret < 0)
		return ret;

	hwdep->iface = SNDRV_HWDEP_IFACE_HOLOPLOT_PCI;
	hwdep->private_data = priv;

	/* Sysfs attributes, exposed through hwdep */
	hwdep->dev->groups = holoplot_pci_dev_attr_groups;
	dev_set_drvdata(hwdep->dev, priv);

	ret = snd_card_register(card);
	if (ret < 0)
		return ret;

	dev_info(dev, "Holoplot PCIe card probed. bar0=%px, bar4=%px irq=%d\n",
		 priv->bar0, priv->bar4, pci->irq);

	return 0;
}

static const struct pci_device_id holoplot_pci_ids[] = {
        { PCI_DEVICE(PCI_VENDOR_ID_HOLOPLOT, PCI_DEVICE_ID_holoplot_pci) },
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