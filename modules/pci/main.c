#include <linux/compiler.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <sound/core.h>
#include <sound/hwdep.h>
#include <sound/info.h>
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

	spinlock_t lock;

	void __iomem *bar0; /* Xilinx DMA register spaces */
	void __iomem *bar2; /* PCM transport control */

	int bar2_phys;

	struct snd_pcm *pcm;
	struct snd_pcm_substream *playback;
	struct snd_pcm_substream *capture;

	wait_queue_head_t misc_read_wait;
	wait_queue_head_t misc_write_wait;

	u32 misc_read_done;
	u32 misc_write_done;

	char device_id[sizeof(u64)+1];
	u32 fpga_design_type, fpga_design_version;

	bool card_registered;
	struct work_struct card_ready_work;
	struct snd_card *card;
};

/* DMA registers can be accessed directly through BAR 0 */

static void holoplot_pci_write_pcie_dma_reg(struct holoplot_pci_priv *priv, u32 channel, u32 reg, u32 val)
{
	writel(val, priv->bar0 + REG_DMA_CHANNEL_BASE(channel) + reg);
}

static u32 holoplot_pci_read_pcie_dma_reg(struct holoplot_pci_priv *priv, u32 channel, u32 reg)
{
	return readl(priv->bar0 + REG_DMA_CHANNEL_BASE(channel) + reg);
}

/* PCM transport control registers can be accessed directly through BAR 2 */

static u64 holoplot_pci_read_audio_dma_reg(struct holoplot_pci_priv *priv, u64 reg)
{
	return readq(priv->bar2 + reg);
}

static void holoplot_pci_write_audio_dma_reg(struct holoplot_pci_priv *priv, u64 reg, u64 val)
{
	writeq(val, priv->bar2 + reg);
}

/* Misc registers are accessed indirectly through BAR 0 and IRQs */

static void holoplot_pci_assert_irq(struct holoplot_pci_priv *priv, u32 channel, u32 cmd)
{
	holoplot_pci_write_pcie_dma_reg(priv, channel,
					REG_DMA_CHANNEL_SCRATCH_0, cmd);

	wmb();

	holoplot_pci_write_pcie_dma_reg(priv, channel,
					REG_DMA_CHANNEL_AXI_IRQ_CONTROL, 0x1);
	holoplot_pci_write_pcie_dma_reg(priv, channel,
					REG_DMA_CHANNEL_AXI_INTERRUPT_ASSERT, 0x8);
}

static int holoplot_pci_read_misc_reg(struct holoplot_pci_priv *priv, u32 reg, u64 *val)
{
	int ret;
	u32 hi, lo;

	holoplot_pci_write_pcie_dma_reg(priv, DMA_CHANNEL_MISC_READ,
					REG_DMA_CHANNEL_SCRATCH_1, reg);

	priv->misc_read_done = 0;

	holoplot_pci_assert_irq(priv, DMA_CHANNEL_MISC_READ,
				MISC_INTERRUPT_REQUEST);

	ret = wait_event_interruptible_timeout(priv->misc_read_wait,
					       priv->misc_read_done,
					       MISC_TRANSACTION_TIMEOUT);
	if (ret == 0)
		return -ETIMEDOUT;

	lo = holoplot_pci_read_pcie_dma_reg(priv, DMA_CHANNEL_MISC_READ,
					    REG_DMA_CHANNEL_SCRATCH_2);
	hi = holoplot_pci_read_pcie_dma_reg(priv, DMA_CHANNEL_MISC_READ,
					    REG_DMA_CHANNEL_SCRATCH_3);

	*val = ((u64) hi << 32) | lo;

	return 0;
}

static int holoplot_pci_write_misc_reg(struct holoplot_pci_priv *priv, u32 reg, u64 val)
{
	int ret;

	printk(KERN_ERR "XXX %s() %d\n", __func__, __LINE__);

	holoplot_pci_write_pcie_dma_reg(priv, DMA_CHANNEL_MISC_WRITE,
					REG_DMA_CHANNEL_SCRATCH_1, reg);
	holoplot_pci_write_pcie_dma_reg(priv, DMA_CHANNEL_MISC_WRITE,
					REG_DMA_CHANNEL_SCRATCH_2, lower_32_bits(val));
	holoplot_pci_write_pcie_dma_reg(priv, DMA_CHANNEL_MISC_WRITE,
					REG_DMA_CHANNEL_SCRATCH_3, upper_32_bits(val));

	priv->misc_write_done = 0;

	holoplot_pci_assert_irq(priv, DMA_CHANNEL_MISC_WRITE,
				MISC_INTERRUPT_REQUEST);

	ret = wait_event_interruptible_timeout(priv->misc_write_wait,
					       priv->misc_write_done,
					       MISC_TRANSACTION_TIMEOUT);
	if (ret == 0)
		return -ETIMEDOUT;

	return 0;
}

static u32 holoplot_pci_read_interrupt(struct holoplot_pci_priv *priv, u32 channel)
{
	u32 status;

	status = holoplot_pci_read_pcie_dma_reg(priv, channel,
						REG_DMA_CHANNEL_PCIE_INTERRUPT_STATUS);
	holoplot_pci_write_pcie_dma_reg(priv, channel,
					REG_DMA_CHANNEL_PCIE_INTERRUPT_STATUS, status);

	return status;
}

static irqreturn_t holoplot_pci_interrupt(int irq, void *dev_id)
{
	struct holoplot_pci_priv *priv = dev_id;
	struct device *dev = &priv->pci->dev;
	u32 status;

	status = holoplot_pci_read_interrupt(priv, DMA_CHANNEL_CAPTURE);
	if (status & 0x8) {
		if (priv->capture) {
			snd_pcm_period_elapsed(priv->capture);
		}
	}

	status = holoplot_pci_read_interrupt(priv, DMA_CHANNEL_PLAYBACK);
	if (status & 0x8) {
		if (priv->playback)
			snd_pcm_period_elapsed(priv->playback);
	}

	status = holoplot_pci_read_interrupt(priv, DMA_CHANNEL_MISC_READ);
	printk(KERN_ERR "XXX %s() %d status %08x\n", __func__, __LINE__, status);
	if (status & 0x8) {
		u32 cmd = holoplot_pci_read_pcie_dma_reg(priv, DMA_CHANNEL_MISC_READ,
							 REG_DMA_CHANNEL_SCRATCH_0);
		switch (cmd) {
		case MISC_INTERRUPT_INIT_DONE:
			printk(KERN_INFO "XXX %s() %d  INIT DONE\n", __func__, __LINE__);
			schedule_work(&priv->card_ready_work);
			break;

		case MISC_INTERRUPT_REPLY:
			printk(KERN_INFO "XXX %s() %d  REPLY\n", __func__, __LINE__);
			WRITE_ONCE(priv->misc_read_done, 1);
			wake_up(&priv->misc_read_wait);
			break;

		default:
			dev_err(dev, "unhandled misc interrupt %08x\n", cmd);
			break;
		}
	}

	status = holoplot_pci_read_interrupt(priv, DMA_CHANNEL_MISC_WRITE);
	printk(KERN_ERR "XXX %s() %d status %08x\n", __func__, __LINE__, status);
	if (status & 0x8) {
		u32 cmd = holoplot_pci_read_pcie_dma_reg(priv, DMA_CHANNEL_MISC_WRITE,
							 REG_DMA_CHANNEL_SCRATCH_0);
		switch (cmd) {
		case MISC_INTERRUPT_INIT_DONE:
			schedule_work(&priv->card_ready_work);
			break;

		case MISC_INTERRUPT_REPLY:
			WRITE_ONCE(priv->misc_write_done, 1);
			wake_up(&priv->misc_write_wait);
			break;

		default:
			dev_err(dev, "unhandled misc interrupt %08x\n", cmd);
			break;
		}
	}

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
	.channels_min		= 64,
	.channels_max		= 64,
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
	.channels_min		= 64,
	.channels_max		= 64,
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

	spin_lock_irq(&priv->lock);

	priv->playback = substream;
	runtime->private_data = priv;
	runtime->hw = holoplot_pci_playback_hw;

	spin_unlock_irq(&priv->lock);

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

	spin_lock_irq(&priv->lock);

	priv->capture = substream;
	runtime->private_data = priv;
	runtime->hw = holoplot_pci_capture_hw;

	spin_unlock_irq(&priv->lock);

	ret = holoplot_pci_pcm_constraint(substream);
	if (ret)
		return ret;

	return 0;
}

static int
holoplot_pci_playback_close(struct snd_pcm_substream *substream)
{
	struct holoplot_pci_priv *priv = snd_pcm_substream_chip(substream);

	spin_lock_irq(&priv->lock);
	priv->playback = NULL;
	spin_unlock_irq(&priv->lock);

	return 0;
}

static int
holoplot_pci_capture_close(struct snd_pcm_substream *substream)
{
	struct holoplot_pci_priv *priv = snd_pcm_substream_chip(substream);

	spin_lock_irq(&priv->lock);
	priv->capture = NULL;
	spin_unlock_irq(&priv->lock);

	return 0;
}

static int
holoplot_pci_playback_prepare(struct snd_pcm_substream *substream)
{
	struct holoplot_pci_priv *priv = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	spin_lock_irq(&priv->lock);

	holoplot_pci_write_audio_dma_reg(priv, REG_PCM_PLAYBACK_OFFSET +
					 REG_PCM_CONTROL,
					 REG_PCM_CONTROL_RESET);
	holoplot_pci_write_audio_dma_reg(priv, REG_PCM_PLAYBACK_OFFSET +
					 REG_PCM_BUFFER_SIZE,
					 runtime->dma_bytes);

	ret = holoplot_pci_write_misc_reg(priv, REG_MISC_PLAYBACK_SRC_ADDR,
					  runtime->dma_addr);

	spin_unlock_irq(&priv->lock);

	if (ret < 0) {
		dev_err(&priv->pci->dev, "error writing playback source address\n");
		return ret;
	}

	return 0;
}

static int
holoplot_pci_capture_prepare(struct snd_pcm_substream *substream)
{
	struct holoplot_pci_priv *priv = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	spin_lock_irq(&priv->lock);

	holoplot_pci_write_audio_dma_reg(priv, REG_PCM_CAPTURE_OFFSET +
					 REG_PCM_CONTROL,
					 REG_PCM_CONTROL_RESET);
	holoplot_pci_write_audio_dma_reg(priv, REG_PCM_CAPTURE_OFFSET +
					 REG_PCM_BUFFER_SIZE,
					 runtime->dma_bytes);

	ret = holoplot_pci_write_misc_reg(priv, REG_MISC_CAPTURE_DST_ADDR,
					  runtime->dma_addr);
	if (ret < 0) {
		dev_err(&priv->pci->dev, "error writing capture destination address\n");
		return ret;
	}

	spin_unlock_irq(&priv->lock);

	return 0;
}

static int
holoplot_pci_playback_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct holoplot_pci_priv *priv = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	holoplot_pci_write_audio_dma_reg(priv, REG_PCM_PLAYBACK_OFFSET +
					 REG_PCM_PERIOD_SIZE,
					 params_period_bytes(params));

	return 0;
}

static int
holoplot_pci_capture_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params)
{
	struct holoplot_pci_priv *priv = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	printk(KERN_INFO "holoplot_pci_capture_hw_params period bytes %d, buffer size %ld\n", params_period_bytes(params), runtime->dma_bytes);

	holoplot_pci_write_audio_dma_reg(priv, REG_PCM_CAPTURE_OFFSET +
					 REG_PCM_PERIOD_SIZE,
					 params_period_bytes(params));

	return 0;
}

static int
holoplot_pci_playback_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct holoplot_pci_priv *priv = snd_pcm_substream_chip(substream);
	u32 v;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		// holoplot_pci_update_egress_reg_bits(priv,
		// 				    REG_AXIPCIE_EGRESS_CONTROL,
		// 				    PCI_EXP_DPC_CAP_DL_ACTIVE,
		// 				    PCI_EXP_DPC_CAP_DL_ACTIVE);

		holoplot_pci_write_audio_dma_reg(priv,
						 REG_PCM_PLAYBACK_OFFSET +
						 REG_PCM_CONTROL,
						 REG_PCM_CONTROL_RUN);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		// holoplot_pci_update_egress_reg_bits(priv,
		// 				    REG_AXIPCIE_EGRESS_CONTROL,
		// 				    PCI_EXP_DPC_CAP_DL_ACTIVE, 0);

		holoplot_pci_write_audio_dma_reg(priv,
						 REG_PCM_PLAYBACK_OFFSET +
						 REG_PCM_CONTROL, 0);
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
		holoplot_pci_write_audio_dma_reg(priv,
						 REG_PCM_CAPTURE_OFFSET +
						 REG_PCM_CONTROL,
						 REG_PCM_CONTROL_RUN);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		holoplot_pci_write_audio_dma_reg(priv,
						 REG_PCM_CAPTURE_OFFSET +
						 REG_PCM_CONTROL, 0);
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
	u64 pos = holoplot_pci_read_audio_dma_reg(priv, REG_PCM_PLAYBACK_OFFSET +
						  REG_PCM_POSITION);

	return bytes_to_frames(substream->runtime, pos);
}

static snd_pcm_uframes_t
holoplot_pci_capture_pointer(struct snd_pcm_substream *substream)
{
	struct holoplot_pci_priv *priv = snd_pcm_substream_chip(substream);
	u64 pos = holoplot_pci_read_audio_dma_reg(priv, REG_PCM_CAPTURE_OFFSET +
						  REG_PCM_POSITION);

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
	int i;

	for (i = 0; i < sizeof(priv->device_id); i++) {
		buf[i] = priv->device_id[i];
	}

	buf[i++] = '\n';
	buf[i] = '\0';

	return i;
}
static DEVICE_ATTR_RO(device_id);

static ssize_t fpga_design_type_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct holoplot_pci_priv *priv = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%02x\n", priv->fpga_design_type);
}
static DEVICE_ATTR_RO(fpga_design_type);

static ssize_t fpga_design_version_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct holoplot_pci_priv *priv = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%02x\n", priv->fpga_design_version);
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

static void
holoplot_pci_proc_read(struct snd_info_entry *entry,
		       struct snd_info_buffer *buffer)
{
	struct holoplot_pci_priv *priv = entry->private_data;

	snd_iprintf(buffer, "PCI location: %s\n", dev_name(&priv->pci->dev));
	snd_iprintf(buffer, "Device ID: %s\n", priv->device_id);
	snd_iprintf(buffer, "FPGA design type: %02x\n", priv->fpga_design_type);
	snd_iprintf(buffer, "FPGA design version: %02x\n", priv->fpga_design_version);
}

static void holoplot_pci_card_ready(struct work_struct *work)
{
	struct holoplot_pci_priv *priv =
		container_of(work, struct holoplot_pci_priv, card_ready_work);
	unsigned long flags;
	int ret, i;
	u64 v;

	ret = holoplot_pci_write_misc_reg(priv, REG_MISC_PCM_CONTROL_BASE, priv->bar2_phys);
	if (ret < 0) {
		dev_err(&priv->pci->dev,
			"error writing PCM control base: %d\n", ret);
		return;
	}

	ret = holoplot_pci_read_misc_reg(priv, REG_MISC_DESIGN_TYPE_VERSION, &v);
	if (ret < 0) {
		dev_err(&priv->pci->dev,
			"error reading FPGA device info: %d\n", ret);
		return;
	}

	priv->fpga_design_type = (v >> 32) & 0xff;
	priv->fpga_design_version = v & 0xff;

	ret = holoplot_pci_read_misc_reg(priv, REG_MISC_DEVICE_ID, &v);
	if (ret < 0) {
		dev_err(&priv->pci->dev, "error reading device ID: %d\n", ret);
		return;
	}

	for (i = 0; i < 8; i++) {
		priv->device_id[i] = (v >> (i * 8)) & 0xff;

		if (priv->device_id[i] == 0 || priv->device_id[i] == 0xff)
			break;
	}

	sprintf(priv->card->longname,
		"Holoplot PCIe card at %s, irq %d, device ID %s",
		pci_name(priv->pci), priv->pci->irq, priv->device_id);

	if (!priv->card_registered) {
		ret = snd_card_register(priv->card);
		if (ret < 0) {
			dev_err(&priv->pci->dev, "error registering card\n");

			return;
		}

		priv->card_registered = true;

		dev_info(&priv->pci->dev, "card registered. Device ID %s n %llx\n", priv->device_id, v);
	}
}

static int __holoplot_pci_probe(struct pci_dev *pci,
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
	u32 v;
	u64 l;

	if (devno >= SNDRV_CARDS)
		return -ENODEV;

	ret = snd_devm_card_new(dev, index[devno], id[devno], THIS_MODULE,
				sizeof(*priv), &card);
	if (ret < 0)
		return ret;

	priv = card->private_data;
	priv->pci = pci;
	priv->card = card;

	spin_lock_init(&priv->lock);
	init_waitqueue_head(&priv->misc_read_wait);
	init_waitqueue_head(&priv->misc_write_wait);

	pci_set_drvdata(pci, priv);

	INIT_WORK(&priv->card_ready_work, holoplot_pci_card_ready);

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

	ret = pcim_iomap_regions(pci, BIT(0)|BIT(2), KBUILD_MODNAME);
	if (ret < 0) {
		dev_err(dev, "error mapping PCI resources.\n");
		return ret;
	}

	priv->bar0 = pcim_iomap_table(pci)[0];
	priv->bar2 = pcim_iomap_table(pci)[2];

	priv->bar2_phys = pci_resource_start(pci, 2);

	pci_set_master(pci);

	strcpy(card->driver, KBUILD_MODNAME);
	strcpy(card->shortname, "holoplot-pcie");

#if 0
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
	holoplot_pci_write_egress_reg(priv, REG_AXIPCIE_EGRESS_SRC_ADDR_LO,
				      lower_32_bits(REMOTE_EGRESS_SOURCE));
	holoplot_pci_write_egress_reg(priv, REG_AXIPCIE_EGRESS_SRC_ADDR_HI,
				      upper_32_bits(REMOTE_EGRESS_SOURCE));

	// FIX THIS MESS
	v = holoplot_pci_read_egress_reg(priv, REG_AXIPCIE_EGRESS_CONTROL);
	v &= ~0x001f0000;
	v |= BIT(19);
	v |= BIT(2);
	holoplot_pci_write_egress_reg(priv, REG_AXIPCIE_EGRESS_CONTROL, v);

	/* The internal capture source address is always the same */
	holoplot_pci_write_audio_dma_reg(priv, REG_PCM_CAPTURE_SRC_ADDR,
				  REMOTE_EGRESS_SOURCE);

#endif

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

	/* proc entry */
	snd_card_ro_proc_new(card, card->driver, priv, holoplot_pci_proc_read);

	/* HW dependent interface */
	ret = snd_hwdep_new(card, KBUILD_MODNAME, 0, &hwdep);
	if (ret < 0)
		return ret;

	hwdep->iface = SNDRV_HWDEP_IFACE_HOLOPLOT_PCI;
	hwdep->private_data = priv;

	/* Sysfs attributes, exposed through hwdep */
	hwdep->dev->groups = holoplot_pci_dev_attr_groups;
	dev_set_drvdata(hwdep->dev, priv);

	if (devm_request_irq(dev, pci->irq, holoplot_pci_interrupt,
			     IRQF_SHARED, KBUILD_MODNAME, priv)) {
		dev_err(dev, "cannot obtain IRQ %d\n", pci->irq);
		return -EBUSY;
	}

	ret = holoplot_pci_write_misc_reg(priv, REG_MISC_RESET, 1);
	if (ret < 0) {
		dev_err(dev, "error writing reset register\n");
	}

	dev_info(dev, "Holoplot PCIe card probed. bar0=%px, bar2=%px irq=%d\n",
		 priv->bar0, priv->bar2, pci->irq);

	devno++;

	return 0;
}

static int holoplot_pci_probe(struct pci_dev *pci,
			      const struct pci_device_id *pci_id)
{
	return snd_card_free_on_error(&pci->dev, __holoplot_pci_probe(pci, pci_id));
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