// SPDX-License-Identifier: GPL-2.0-only

#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/if_arp.h>
#include <linux/jiffies.h>
#include <linux/moduleparam.h>
#include <linux/workqueue.h>

#include <net/ip_tunnels.h>

#include "net.h"

#define RX_FULL_POLL_DELAY_MS 10

struct hab_net_priv {
	struct device		*dev;
	struct net_device	*ndev;
	struct napi_struct	napi;

	struct ring_buffer	*rx; // from host to device
	struct ring_buffer	*tx; // from device to host
	dma_addr_t		rx_phys;
	dma_addr_t		tx_phys;

	hab_net_write_cb	cb;
	void			*cb_arg;

	struct work_struct	rx_cb_work;
};

static netdev_tx_t hab_net_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct hab_net_priv *priv = netdev_priv(ndev);
	struct ring_buffer *rx = priv->rx;
	struct ring_buffer_packet *pkt;
	u32 write_pos, read_pos;
	int ret = NETDEV_TX_OK;

	rmb();

	write_pos = READ_ONCE(rx->write_pos);
	read_pos = READ_ONCE(rx->read_pos);

	if (write_pos >= RING_BUFFER_SIZE) {
		netdev_err(ndev, "invalid write_pos %u\n", write_pos);

		write_pos = 0;
		rx->write_pos = write_pos;
	}

	if (unlikely(skb->len > MAX_PACKET_SIZE)) {
		netdev_err(ndev, "packet too large (%u bytes), dropping\n",
			   skb->len);
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);

		return ret;
	}

	pkt = &rx->data[write_pos];

	// netdev_info(ndev, "transmitting packet: %u bytes index %d\n", skb->len, write_pos);

	write_pos++;
	write_pos %= RING_BUFFER_SIZE;

	if (unlikely(write_pos == read_pos)) {
		netdev_err(ndev, "RX buffer full\n");
		goto out;
	}

	memcpy(pkt->data, skb->data, skb->len);
	pkt->length = skb->len;
	wmb();

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;

	WRITE_ONCE(rx->write_pos, write_pos);

out:
	dev_kfree_skb_any(skb);

	/* Latch PCI interrupt to notify device */
	schedule_work(&priv->rx_cb_work);

	return ret;
}

static int hab_net_napi_poll(struct napi_struct *napi, int budget)
{
	struct hab_net_priv *priv =
		container_of(napi, struct hab_net_priv, napi);
	struct net_device *ndev = priv->ndev;
	struct ring_buffer *tx = priv->tx;
	u32 write_pos, read_pos;
	int count = 0;

	rmb();

	write_pos = READ_ONCE(tx->write_pos);
	read_pos = READ_ONCE(tx->read_pos);

	if (read_pos >= RING_BUFFER_SIZE) {
		netdev_err(ndev, "invalid read_pos %u\n", read_pos);

		read_pos = 0;
		tx->read_pos = read_pos;
	}

	// netdev_info(ndev, "reading TX buffer: read_pos=%u write_pos=%u\n", read_pos, write_pos);

	while (read_pos != write_pos && count < budget) {
		struct ring_buffer_packet *pkt = &tx->data[read_pos];
		size_t len = pkt->length;
		struct sk_buff *skb;
		u8 ip_version;

		// netdev_info(ndev, "receiving packet: %lu bytes index %d count %d\n", len, read_pos, count);

		read_pos++;
		read_pos %= RING_BUFFER_SIZE;

		count++;

		if (len > MAX_PACKET_SIZE || len < ETH_HLEN) {
			netdev_err(ndev, "invalid packet length %lu, dropping\n",
				   len);

			ndev->stats.rx_length_errors++;
			ndev->stats.rx_dropped++;
			continue;
		}

		skb = napi_alloc_skb(&priv->napi, len);
		if (!skb) {
			ndev->stats.rx_dropped++;
			continue;
		}

		skb_put_data(skb, pkt->data, len);

		ip_version = skb->len ? (skb->data[0] >> 4) : 0;

		switch (ip_version) {
		case 4:
			skb->protocol = htons(ETH_P_IP);
			break;
		case 6:
			skb->protocol = htons(ETH_P_IPV6);
			break;
		default:
			netdev_err(ndev, "unknown IP version %u, dropping\n",
				   ip_version);

			ndev->stats.rx_dropped++;
			dev_kfree_skb_any(skb);
			continue;
		}

		ndev->stats.rx_packets++;
		ndev->stats.rx_bytes += len;

		napi_gro_receive(&priv->napi, skb);

		wmb();
		WRITE_ONCE(tx->read_pos, read_pos);
	}

	if (count < budget)
		napi_complete_done(&priv->napi, count);

	return count;
}

void hab_net_read_tx(struct hab_net_priv *priv)
{
	napi_schedule(&priv->napi);
}

static int hab_net_net_open(struct net_device *ndev)
{
	struct hab_net_priv *priv = netdev_priv(ndev);

	WRITE_ONCE(priv->rx->write_pos, READ_ONCE(priv->rx->read_pos));
	WRITE_ONCE(priv->tx->read_pos, READ_ONCE(priv->tx->write_pos));

	napi_enable(&priv->napi);
	netif_start_queue(ndev);

	return 0;
}

static int hab_net_net_stop(struct net_device *ndev)
{
	struct hab_net_priv *priv = netdev_priv(ndev);

	netif_stop_queue(ndev);
	napi_disable(&priv->napi);

	return 0;
}

static const struct net_device_ops hab_net_netdev_ops = {
	.ndo_open	= hab_net_net_open,
	.ndo_stop	= hab_net_net_stop,
	.ndo_start_xmit	= hab_net_xmit,
};

static void hab_net_rx_cb_work(struct work_struct *work)
{
	struct hab_net_priv *priv =
		container_of(work, struct hab_net_priv, rx_cb_work);

	if (priv->cb)
		priv->cb(priv->cb_arg);
}

static void hab_net_cancel_work(void *arg)
{
	struct hab_net_priv *priv = arg;

	cancel_work_sync(&priv->rx_cb_work);
}

int hab_net_net_probe(struct device *dev,
		      struct hab_net_priv **priv_out)
{
	struct hab_net_priv *priv;
	struct net_device *ndev;
	int ret;

	ndev = devm_alloc_etherdev(dev, sizeof(*priv));
	if (!ndev)
		return -ENOMEM;

	priv = netdev_priv(ndev);
	priv->dev = dev;
	priv->ndev = ndev;

	snprintf(ndev->name, sizeof(ndev->name), "hab%%d");

	SET_NETDEV_DEV(ndev, dev);

	ndev->netdev_ops = &hab_net_netdev_ops;
	ndev->header_ops = &ip_tunnel_header_ops;

	ndev->mtu = 1500;
	ndev->hard_header_len = 0;
	ndev->addr_len = 0;
	ndev->type = ARPHRD_NONE;
	ndev->flags = IFF_POINTOPOINT | IFF_NOARP;
	ndev->features = 0;

	netif_napi_add(ndev, &priv->napi, hab_net_napi_poll);

	/* Host to device */
	priv->rx = dmam_alloc_coherent(dev, sizeof(*priv->rx),
				       &priv->rx_phys, GFP_KERNEL);
	if (!priv->rx) {
		dev_err(dev, "could not allocate RX buffer\n");
		return -ENOMEM;
	}

	/* Device to host */
	priv->tx = dmam_alloc_coherent(dev, sizeof(*priv->tx),
				       &priv->tx_phys, GFP_KERNEL);
	if (!priv->tx) {
		dev_err(dev, "could not allocate TX buffer\n");
		return -ENOMEM;
	}

	INIT_WORK(&priv->rx_cb_work, hab_net_rx_cb_work);

	ret = devm_add_action_or_reset(dev, hab_net_cancel_work, priv);
	if (ret < 0)
		return ret;

	ret = devm_register_netdev(dev, ndev);
	if (ret < 0) {
		dev_err(dev, "could not register network device: %d\n", ret);
		return ret;
	}

	dev_info(dev, "network device %s registered\n", ndev->name);
	dev_info(dev, "RX buffer DMA at %pad\n", &priv->rx_phys);
	dev_info(dev, "TX buffer DMA at %pad\n", &priv->tx_phys);

	*priv_out = priv;

	return 0;
}

void hab_net_set_write_callback(struct hab_net_priv *priv,
				hab_net_write_cb cb, void *arg)
{
	priv->cb = cb;
	priv->cb_arg = arg;
}

dma_addr_t hab_net_rx_buffer_phys(struct hab_net_priv *priv)
{
	return priv->rx_phys;
}

dma_addr_t hab_net_tx_buffer_phys(struct hab_net_priv *priv)
{
	return priv->tx_phys;
}
