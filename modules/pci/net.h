#pragma once

#include <linux/netdevice.h>

#define RING_BUFFER_SIZE 64
#define MAX_PACKET_SIZE 1540

struct ring_buffer_packet {
	u32 length;
	u8 data[MAX_PACKET_SIZE];
};

struct ring_buffer {
	u32 read_pos;
	u32 write_pos;
	struct ring_buffer_packet data[RING_BUFFER_SIZE];
};

struct hab_net_priv;

int hab_net_net_probe(struct device *dev, struct hab_net_priv **priv_out);

typedef void (*hab_net_write_cb)(void *);
void hab_net_set_write_callback(struct hab_net_priv *priv,
				hab_net_write_cb cb, void *arg);

/* RX buffer contains packets that flow from host to device */
dma_addr_t hab_net_rx_buffer_phys(struct hab_net_priv *priv);
/* TX buffer contains packets that flow from device to host */
dma_addr_t hab_net_tx_buffer_phys(struct hab_net_priv *priv);

void hab_net_read_tx(struct hab_net_priv *priv);