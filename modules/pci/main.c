#include <linux/compiler.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/workqueue.h>

#include <sound/core.h>
#include <sound/hwdep.h>
#include <sound/info.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/pcm.h>

#include "defs.h"

#ifndef PCI_VENDOR_ID_HOLOPLOT
#define PCI_VENDOR_ID_HOLOPLOT 0x2081
#endif

#define PCI_DEVICE_ID_AUDIO_BRIDE 0x2401

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;
module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "Index value for the Holoplot PCIe AudioBridge soundcard.");

static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;
module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string for the Holoplot PCIe AudioBridge soundcard.");

struct hab_priv {
	struct pci_dev *pci;

	spinlock_t lock;

	void __iomem *bar0; /* Xilinx DMA register spaces */
	void __iomem *bar2; /* PCM transport controls */

	u64 bar2_phys;

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

static void hab_write_pcie_dma_reg(struct hab_priv *priv,
				   u32 channel, u32 reg, u32 val)
{
	writel(val, priv->bar0 + REG_DMA_BASE(channel) + reg);
}

static u32 hab_read_pcie_dma_reg(struct hab_priv *priv, u32 channel, u32 reg)
{
	return readl(priv->bar0 + REG_DMA_BASE(channel) + reg);
}

/* PCM transport control registers can be accessed directly through BAR 2 */

static u64 hab_read_audio_dma_reg(struct hab_priv *priv, u64 reg)
{
	return readq(priv->bar2 + reg);
}

static void hab_write_audio_dma_reg(struct hab_priv *priv, u64 reg, u64 val)
{
	writeq(val, priv->bar2 + reg);
}

static void hab_write_audio_dma_reset(struct hab_priv *priv, u64 offset)
{
	hab_write_audio_dma_reg(priv, offset + REG_PCM_CONTROL,
				REG_PCM_CONTROL_RESET);
}

static int hab_write_audio_dma_run(struct hab_priv *priv, u64 offset, bool run)
{
	void __iomem *reg = priv->bar2 + offset + REG_PCM_CONTROL;
	u64 val = run ? REG_PCM_CONTROL_RUN : 0;

	writeq(val, reg);

	if (!run) {
		u64 r;

		/*
		 * The RUN bit will be cleared by the hardware only after the
		 * current DMA period has been processed.
		 * Wait for that to happen.
		 */
		return read_poll_timeout_atomic(readq, r,
						(r & REG_PCM_CONTROL_RUN) == 0,
						10, 1000, false, reg);
	}

	return 0;
}

/* Misc registers are accessed indirectly through BAR 0 and IRQs */

static void hab_assert_irq(struct hab_priv *priv, u32 channel, u32 cmd)
{
	hab_write_pcie_dma_reg(priv, channel, REG_DMA_SCRATCH_0, cmd);

	wmb();

	hab_write_pcie_dma_reg(priv, channel, REG_DMA_AXI_IRQ_CONTROL, 0x1);
	hab_write_pcie_dma_reg(priv, channel, REG_DMA_AXI_INTERRUPT_ASSERT,
			       REG_DMA_PCIE_SOFTWARE_INTERRUPT);
}

static int hab_read_misc_reg(struct hab_priv *priv, u32 reg, u64 *val)
{
	int ret;
	u32 hi, lo;

	hab_write_pcie_dma_reg(priv, DMA_CHANNEL_MISC_READ,
			       REG_DMA_SCRATCH_1, reg);

	priv->misc_read_done = 0;

	hab_assert_irq(priv, DMA_CHANNEL_MISC_READ, MISC_INTERRUPT_REQUEST);

	ret = wait_event_interruptible_timeout(priv->misc_read_wait,
					       priv->misc_read_done,
					       MISC_TRANSACTION_TIMEOUT);
	if (ret == 0)
		return -ETIMEDOUT;

	lo = hab_read_pcie_dma_reg(priv, DMA_CHANNEL_MISC_READ,
				   REG_DMA_SCRATCH_2);
	hi = hab_read_pcie_dma_reg(priv, DMA_CHANNEL_MISC_READ,
				   REG_DMA_SCRATCH_3);

	*val = ((u64) hi << 32) | lo;

	return 0;
}

static int hab_write_misc_reg(struct hab_priv *priv, u32 reg, u64 val)
{
	int ret;

	hab_write_pcie_dma_reg(priv, DMA_CHANNEL_MISC_WRITE,
			       REG_DMA_SCRATCH_1, reg);
	hab_write_pcie_dma_reg(priv, DMA_CHANNEL_MISC_WRITE,
			       REG_DMA_SCRATCH_2, lower_32_bits(val));
	hab_write_pcie_dma_reg(priv, DMA_CHANNEL_MISC_WRITE,
			       REG_DMA_SCRATCH_3, upper_32_bits(val));

	priv->misc_write_done = 0;

	hab_assert_irq(priv, DMA_CHANNEL_MISC_WRITE, MISC_INTERRUPT_REQUEST);

	ret = wait_event_interruptible_timeout(priv->misc_write_wait,
					       priv->misc_write_done,
					       MISC_TRANSACTION_TIMEOUT);
	if (ret == 0)
		return -ETIMEDOUT;

	return 0;
}

static u32 hab_read_interrupt(struct hab_priv *priv, u32 channel)
{
	u32 status;

	status = hab_read_pcie_dma_reg(priv, channel,
				       REG_DMA_PCIE_INTERRUPT_STATUS);
	status &= REG_DMA_PCIE_SOFTWARE_INTERRUPT;

	hab_write_pcie_dma_reg(priv, channel,
			       REG_DMA_PCIE_INTERRUPT_STATUS, status);

	return status;
}

static irqreturn_t hab_interrupt(int irq, void *dev_id)
{
	struct hab_priv *priv = dev_id;
	struct device *dev = &priv->pci->dev;
	u32 status;

	status = hab_read_interrupt(priv, DMA_CHANNEL_CAPTURE);
	if (status && priv->capture)
		snd_pcm_period_elapsed(priv->capture);

	status = hab_read_interrupt(priv, DMA_CHANNEL_PLAYBACK);
	if (status && priv->playback)
		snd_pcm_period_elapsed(priv->playback);

	status = hab_read_interrupt(priv, DMA_CHANNEL_MISC_READ);
	if (status) {
		u32 cmd = hab_read_pcie_dma_reg(priv, DMA_CHANNEL_MISC_READ,
						REG_DMA_SCRATCH_0);
		switch (cmd) {
		case MISC_INTERRUPT_INIT_DONE:
			schedule_work(&priv->card_ready_work);
			break;

		case MISC_INTERRUPT_REPLY:
			WRITE_ONCE(priv->misc_read_done, 1);
			wake_up(&priv->misc_read_wait);
			break;

		default:
			dev_err(dev, "unhandled misc read interrupt %08x\n", cmd);
			break;
		}
	}

	status = hab_read_interrupt(priv, DMA_CHANNEL_MISC_WRITE);
	if (status) {
		u32 cmd = hab_read_pcie_dma_reg(priv, DMA_CHANNEL_MISC_WRITE,
						REG_DMA_SCRATCH_0);
		switch (cmd) {
		case MISC_INTERRUPT_REPLY:
			WRITE_ONCE(priv->misc_write_done, 1);
			wake_up(&priv->misc_write_wait);
			break;

		default:
			dev_err(dev, "unhandled misc write interrupt %08x\n", cmd);
			break;
		}
	}

	return IRQ_HANDLED;
}

static const struct snd_pcm_hardware hab_playback_hw = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats		= SNDRV_PCM_FMTBIT_S32_LE,
	.rates			= SNDRV_PCM_RATE_48000,
	.rate_min		= 48000,
	.rate_max		= 48000,
	.channels_min		= CHANNELS_MIN,
	.channels_max		= CHANNELS_MAX,
	.buffer_bytes_max	= BUFFER_BYTES_MAX,
	.period_bytes_min	= PERIOD_BYTES_MIN,
	.period_bytes_max	= PERIOD_BYTES_MAX,
	.periods_min		= PERIODS_MIN,
	.periods_max		= PERIODS_MAX,
};

static const struct snd_pcm_hardware hab_capture_hw = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats		= SNDRV_PCM_FMTBIT_S32_LE,
	.rates			= SNDRV_PCM_RATE_48000,
	.rate_min		= 48000,
	.rate_max		= 48000,
	.channels_min		= CHANNELS_MIN,
	.channels_max		= CHANNELS_MAX,
	.buffer_bytes_max	= BUFFER_BYTES_MAX,
	.period_bytes_min	= PERIOD_BYTES_MIN,
	.period_bytes_max	= PERIOD_BYTES_MAX,
	.periods_min		= PERIODS_MIN,
	.periods_max		= PERIODS_MAX,
};

static int hab_pcm_constraint(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	ret = snd_pcm_hw_constraint_step(runtime, 0,
					 SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
					 BUFFER_ALIGNMENT);
	if (ret)
		return ret;

	ret = snd_pcm_hw_constraint_step(runtime, 0,
					 SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
					 BUFFER_ALIGNMENT);
	if (ret)
		return ret;

	return snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
}

static int
hab_playback_open(struct snd_pcm_substream *substream)
{
	struct hab_priv *priv = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	spin_lock_irq(&priv->lock);

	priv->playback = substream;
	runtime->private_data = priv;
	runtime->hw = hab_playback_hw;

	spin_unlock_irq(&priv->lock);

	ret = hab_pcm_constraint(substream);
	if (ret)
		return ret;

	return 0;
}

static int
hab_capture_open(struct snd_pcm_substream *substream)
{
	struct hab_priv *priv = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	spin_lock_irq(&priv->lock);

	priv->capture = substream;
	runtime->private_data = priv;
	runtime->hw = hab_capture_hw;

	spin_unlock_irq(&priv->lock);

	ret = hab_pcm_constraint(substream);
	if (ret)
		return ret;

	return 0;
}

static int hab_playback_close(struct snd_pcm_substream *substream)
{
	struct hab_priv *priv = snd_pcm_substream_chip(substream);

	hab_write_audio_dma_reset(priv, REG_PCM_PLAYBACK_OFFSET);

	spin_lock_irq(&priv->lock);
	priv->playback = NULL;
	spin_unlock_irq(&priv->lock);

	return 0;
}

static int hab_capture_close(struct snd_pcm_substream *substream)
{
	struct hab_priv *priv = snd_pcm_substream_chip(substream);

	hab_write_audio_dma_reset(priv, REG_PCM_CAPTURE_OFFSET);

	spin_lock_irq(&priv->lock);
	priv->capture = NULL;
	spin_unlock_irq(&priv->lock);

	return 0;
}

static int hab_playback_prepare(struct snd_pcm_substream *substream)
{
	struct hab_priv *priv = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct device *dev = &priv->pci->dev;
	int ret;

	hab_write_audio_dma_reset(priv, REG_PCM_PLAYBACK_OFFSET);

	hab_write_audio_dma_reg(priv,
				REG_PCM_PLAYBACK_OFFSET + REG_PCM_BUFFER_SIZE,
				runtime->dma_bytes);

	ret = hab_write_misc_reg(priv, REG_MISC_PLAYBACK_SRC_ADDR,
				 runtime->dma_addr);
	if (ret < 0) {
		dev_err(dev, "writing playback source address: %d\n", ret);
		return ret;
	}

	ret = hab_write_misc_reg(priv, REG_MISC_PLAYBACK_SIZE,
				 runtime->dma_bytes);
	if (ret < 0) {
		dev_err(dev, "writing playback size: %d\n", ret);
		return ret;
	}

	return ret;
}

static int hab_capture_prepare(struct snd_pcm_substream *substream)
{
	struct hab_priv *priv = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct device *dev = &priv->pci->dev;
	int ret;

	hab_write_audio_dma_reset(priv, REG_PCM_CAPTURE_OFFSET);

	hab_write_audio_dma_reg(priv,
				REG_PCM_CAPTURE_OFFSET + REG_PCM_BUFFER_SIZE,
				runtime->dma_bytes);

	ret = hab_write_misc_reg(priv, REG_MISC_CAPTURE_DST_ADDR,
				 runtime->dma_addr);
	if (ret < 0) {
		dev_err(dev, "writing capture destination address: %d\n", ret);
		return ret;
	}

	ret = hab_write_misc_reg(priv, REG_MISC_CAPTURE_SIZE,
				 runtime->dma_bytes);
	if (ret < 0) {
		dev_err(dev, "writing capture size: %d\n", ret);
		return ret;
	}

	return ret;
}

static int hab_playback_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params)
{
	struct hab_priv *priv = snd_pcm_substream_chip(substream);

	hab_write_audio_dma_reg(priv,
				REG_PCM_PLAYBACK_OFFSET + REG_PCM_PERIOD_SIZE,
				params_period_bytes(params));

	hab_write_audio_dma_reg(priv,
				REG_PCM_PLAYBACK_OFFSET + REG_PCM_CHANNEL_COUNT,
				params_channels(params));

	return 0;
}

static int hab_capture_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	struct hab_priv *priv = snd_pcm_substream_chip(substream);

	hab_write_audio_dma_reg(priv,
				REG_PCM_CAPTURE_OFFSET + REG_PCM_PERIOD_SIZE,
				params_period_bytes(params));

	hab_write_audio_dma_reg(priv,
				REG_PCM_CAPTURE_OFFSET + REG_PCM_CHANNEL_COUNT,
				params_channels(params));

	return 0;
}

static int hab_playback_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct hab_priv *priv = snd_pcm_substream_chip(substream);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		return hab_write_audio_dma_run(priv,
					       REG_PCM_PLAYBACK_OFFSET, true);

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		return hab_write_audio_dma_run(priv,
					       REG_PCM_PLAYBACK_OFFSET, false);
	}

	return  -EINVAL;
}

static int hab_capture_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct hab_priv *priv = snd_pcm_substream_chip(substream);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		return hab_write_audio_dma_run(priv,
					       REG_PCM_CAPTURE_OFFSET, true);

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		return hab_write_audio_dma_run(priv,
					       REG_PCM_CAPTURE_OFFSET, false);
	}

	return -EINVAL;
}

static snd_pcm_uframes_t
hab_playback_pointer(struct snd_pcm_substream *substream)
{
	struct hab_priv *priv = snd_pcm_substream_chip(substream);
	u64 pos = hab_read_audio_dma_reg(priv, REG_PCM_PLAYBACK_OFFSET +
					       REG_PCM_POSITION);

	return bytes_to_frames(substream->runtime, pos);
}

static snd_pcm_uframes_t
hab_capture_pointer(struct snd_pcm_substream *substream)
{
	struct hab_priv *priv = snd_pcm_substream_chip(substream);
	u64 pos = hab_read_audio_dma_reg(priv, REG_PCM_CAPTURE_OFFSET +
					       REG_PCM_POSITION);

	return bytes_to_frames(substream->runtime, pos);
}

static const struct snd_pcm_ops hab_playback_ops = {
	.open		= hab_playback_open,
	.close		= hab_playback_close,
	.prepare	= hab_playback_prepare,
	.hw_params	= hab_playback_hw_params,
	.trigger	= hab_playback_trigger,
	.pointer	= hab_playback_pointer,
};

static const struct snd_pcm_ops hab_capture_ops = {
	.open		= hab_capture_open,
	.close		= hab_capture_close,
	.prepare	= hab_capture_prepare,
	.hw_params	= hab_capture_hw_params,
	.trigger	= hab_capture_trigger,
	.pointer	= hab_capture_pointer,
};

/* Sysfs attributes*/

static ssize_t device_id_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct hab_priv *priv = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < sizeof(priv->device_id); i++)
		buf[i] = priv->device_id[i];

	buf[i++] = '\n';
	buf[i] = '\0';

	return i;
}
static DEVICE_ATTR_RO(device_id);

static ssize_t fpga_design_type_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct hab_priv *priv = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%02x\n", priv->fpga_design_type);
}
static DEVICE_ATTR_RO(fpga_design_type);

static ssize_t fpga_design_version_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct hab_priv *priv = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%02x\n", priv->fpga_design_version);
}
static DEVICE_ATTR_RO(fpga_design_version);

static ssize_t _ip_address_show(struct device *dev, char *buf, u32 reg)
{
	struct hab_priv *priv = dev_get_drvdata(dev);
	u32 ip, mask;
	int ret;
	u64 v;

	ret = hab_read_misc_reg(priv, reg, &v);
	if (ret < 0)
		return ret;

	if (v == 0)
		return 0;

	ip = lower_32_bits(v);
	mask = upper_32_bits(v);

	return sysfs_emit(buf, "%pI4/%pI4\n", &ip, &mask);
}

static ssize_t primary_ip_address_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	return _ip_address_show(dev, buf, REG_MISC_PRIMARY_IP);
}
static DEVICE_ATTR_RO(primary_ip_address);

static ssize_t secondary_ip_address_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	return _ip_address_show(dev, buf, REG_MISC_SECONDARY_IP);
}
static DEVICE_ATTR_RO(secondary_ip_address);

static ssize_t _mac_address_show(struct device *dev, char *buf, u32 reg)
{
	struct hab_priv *priv = dev_get_drvdata(dev);
	u64 v;
	int ret;

	ret = hab_read_misc_reg(priv, reg, &v);
	if (ret < 0)
		return ret;

	if (v == 0)
		return 0;

	return sysfs_emit(buf, "%pM\n", &v);
}

static ssize_t primary_mac_address_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return _mac_address_show(dev, buf, REG_MISC_PRIMARY_MAC);
}
static DEVICE_ATTR_RO(primary_mac_address);

static ssize_t secondary_mac_address_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	return _mac_address_show(dev, buf, REG_MISC_SECONDARY_MAC);
}
static DEVICE_ATTR_RO(secondary_mac_address);

static struct attribute *hab_dev_attrs[] = {
	&dev_attr_device_id.attr,
	&dev_attr_fpga_design_type.attr,
	&dev_attr_fpga_design_version.attr,
	&dev_attr_primary_ip_address.attr,
	&dev_attr_secondary_ip_address.attr,
	&dev_attr_primary_mac_address.attr,
	&dev_attr_secondary_mac_address.attr,
	NULL,
};

static const struct attribute_group hab_dev_attr_group = {
	.attrs = hab_dev_attrs,
};

static const struct attribute_group *hab_dev_attr_groups[] = {
	&hab_dev_attr_group,
	NULL,
};

static void
hab_proc_read(struct snd_info_entry *entry, struct snd_info_buffer *b)
{
	struct hab_priv *priv = entry->private_data;

	snd_iprintf(b, "PCI location: %s\n", dev_name(&priv->pci->dev));
	snd_iprintf(b, "Device ID: %s\n", priv->device_id);
	snd_iprintf(b, "FPGA design type: %02x\n", priv->fpga_design_type);
	snd_iprintf(b, "FPGA design version: %02x\n", priv->fpga_design_version);
}

static void hab_card_ready(struct work_struct *work)
{
	struct hab_priv *priv =
		container_of(work, struct hab_priv, card_ready_work);
	struct device *dev = &priv->pci->dev;
	int ret, i;
	u64 v;

	ret = hab_write_misc_reg(priv, REG_MISC_PCM_CONTROL_BASE, priv->bar2_phys);
	if (ret < 0) {
		dev_err(dev, "error writing PCM control base: %d\n", ret);
		return;
	}

	ret = hab_read_misc_reg(priv, REG_MISC_DESIGN_TYPE_VERSION, &v);
	if (ret < 0) {
		dev_err(dev, "error reading FPGA device info: %d\n", ret);
		return;
	}

	priv->fpga_design_type = (v >> 32) & 0xff;
	priv->fpga_design_version = v & 0xff;

	ret = hab_read_misc_reg(priv, REG_MISC_DEVICE_ID, &v);
	if (ret < 0) {
		dev_err(dev, "error reading device ID: %d\n", ret);
		return;
	}

	for (i = 0; i < 8; i++) {
		priv->device_id[i] = (v >> (i * 8)) & 0xff;

		if (priv->device_id[i] == 0 || priv->device_id[i] == 0xff)
			break;
	}

	scnprintf(priv->card->longname, sizeof(priv->card->longname),
		  "Holoplot PCIe AudioBridge card at %s, irq %d, device ID %s",
		  pci_name(priv->pci), priv->pci->irq, priv->device_id);

	if (!priv->card_registered) {
		ret = snd_card_register(priv->card);
		if (ret < 0) {
			dev_err(dev, "error registering card\n");

			return;
		}

		priv->card_registered = true;

		dev_info(dev, "Card registered. Device ID %s\n",
			 priv->device_id);
	}
}

static int __hab_probe(struct pci_dev *pci, const struct pci_device_id *pci_id)
{
	struct device *dev = &pci->dev;
	struct snd_hwdep *hwdep;
	struct hab_priv *priv;
	struct snd_card *card;
	static int devno;
	int ret;

	if (devno >= SNDRV_CARDS)
		return -ENODEV;

	ret = snd_devm_card_new(dev, index[devno], id[devno], THIS_MODULE,
				sizeof(*priv), &card);
	if (ret < 0)
		return ret;

	priv = card->private_data;
	priv->card = card;
	priv->pci = pci;

	spin_lock_init(&priv->lock);
	init_waitqueue_head(&priv->misc_read_wait);
	init_waitqueue_head(&priv->misc_write_wait);

	pci_set_drvdata(pci, priv);

	INIT_WORK(&priv->card_ready_work, hab_card_ready);

	ret = pcim_enable_device(pci);
	if (ret < 0) {
		dev_err(dev, "error enabling PCI device\n");
		return ret;
	}

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
	strcpy(card->shortname, "HAB");

	/* PCM */
	ret = snd_pcm_new(card, card->driver, 0, 1, 1, &priv->pcm);
	if (ret < 0)
		return ret;

	snd_pcm_set_ops(priv->pcm, SNDRV_PCM_STREAM_PLAYBACK, &hab_playback_ops);
	snd_pcm_set_ops(priv->pcm, SNDRV_PCM_STREAM_CAPTURE, &hab_capture_ops);

	priv->pcm->private_data = priv;
	priv->pcm->info_flags = 0;
	strcpy(priv->pcm->name, "Holoplot AudioBridge");

	snd_pcm_set_managed_buffer_all(priv->pcm, SNDRV_DMA_TYPE_DEV, dev,
				       BUFFER_BYTES_MAX / 2, BUFFER_BYTES_MAX);

	snd_card_ro_proc_new(card, card->driver, priv, hab_proc_read);

	/* HW dependent interface */
	ret = snd_hwdep_new(card, KBUILD_MODNAME, 0, &hwdep);
	if (ret < 0)
		return ret;

	hwdep->iface = SNDRV_HWDEP_IFACE_HOLOPLOT;
	hwdep->private_data = priv;

	/* Sysfs attributes, exposed through hwdep */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0)
	hwdep->dev->groups = hab_dev_attr_groups;
	dev_set_drvdata(hwdep->dev, priv);
#else
	hwdep->dev.groups = hab_dev_attr_groups;
	dev_set_drvdata(&hwdep->dev, priv);
#endif

	if (devm_request_irq(dev, pci->irq, hab_interrupt,
			     IRQF_SHARED, KBUILD_MODNAME, priv)) {
		dev_err(dev, "cannot obtain IRQ %d\n", pci->irq);
		return -EBUSY;
	}

	/* The card may not be up yet, so just ignore the error */
	hab_write_misc_reg(priv, REG_MISC_RESET, 1);

	dev_info(dev, "Holoplot PCIe card probed. bar0=%px, bar2=%px irq=%d\n",
		 priv->bar0, priv->bar2, pci->irq);

	devno++;

	return 0;
}

static int hab_probe(struct pci_dev *pci, const struct pci_device_id *pci_id)
{
	return snd_card_free_on_error(&pci->dev, __hab_probe(pci, pci_id));
}

static const struct pci_device_id hab_ids[] = {
        { PCI_DEVICE(PCI_VENDOR_ID_HOLOPLOT, PCI_DEVICE_ID_AUDIO_BRIDE) },
        { 0, },
};
MODULE_DEVICE_TABLE(pci, hab_ids);

static struct pci_driver hab_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= hab_ids,
	.probe		= hab_probe,
};

module_pci_driver(hab_driver);

MODULE_AUTHOR("Daniel Mack <daniel.mack@holoplot.com>");
MODULE_DESCRIPTION("Driver for Holoplot PCIe AudioBridge cards");
MODULE_LICENSE("GPL");