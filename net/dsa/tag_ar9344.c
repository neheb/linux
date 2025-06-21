// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 David Bauer <mail@david-bauer.net>
 */

#include <linux/bitfield.h>
#include <linux/etherdevice.h>

#include "dsa_priv.h"

#define AR9344_HDR_LEN		2
#define AR9344_HDR_VERSION	0x2

#define AR9344_HDR_VERSION_MASK		GENMASK(15, 14)
#define AR9344_HDR_PRIORITY_MASK	GENMASK(13, 12)
#define AR9344_HDR_ONTROL_MASK		GENMASK(11, 8)
#define AR9344_HDR_FROM_CPU		BIT(7)
#define AR9344_HDR_PORT_BIT_MASK	GENMASK(6, 0)

static struct sk_buff *ar9344_tag_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct dsa_port *dp = dsa_slave_to_port(dev);
	__be16 *phdr;
	u16 hdr;

	if (skb_cow_head(skb, AR9344_HDR_LEN) < 0)
		return NULL;

	skb_push(skb, AR9344_HDR_LEN);

	memmove(skb->data, skb->data + AR9344_HDR_LEN, 2 * ETH_ALEN);
	phdr = (__be16 *)(skb->data + 2 * ETH_ALEN);

	/* Set the version field, and set destination port information */
	hdr = FIELD_PREP(AR9344_HDR_VERSION_MASK, AR9344_HDR_VERSION) |
		AR9344_HDR_FROM_CPU | BIT(dp->index);

	*phdr = htons(hdr);

	return skb;
}

static struct sk_buff *ar9344_tag_rcv(struct sk_buff *skb, struct net_device *dev,
				      struct packet_type *pt)
{
	u8 ver;
	u16  hdr;
	int port;
	__be16 *phdr;

	if (unlikely(!pskb_may_pull(skb, AR9344_HDR_LEN)))
		return NULL;

	/* The QCA header is added by the switch between src addr and Ethertype
	 * At this point, skb->data points to ethertype so header should be
	 * right before
	 */
	phdr = (__be16 *)(skb->data - 2);
	hdr = ntohs(*phdr);

	/* Make sure the version is correct */
	ver = FIELD_GET(AR9344_HDR_VERSION_MASK, hdr);
	if (unlikely(ver != AR9344_HDR_VERSION))
		return NULL;

	/* Remove QCA tag and recalculate checksum */
	skb_pull_rcsum(skb, AR9344_HDR_LEN);
	memmove(skb->data - ETH_HLEN, skb->data - ETH_HLEN - AR9344_HDR_LEN,
		ETH_HLEN - AR9344_HDR_LEN);

	/* Get source port information */
	port = (hdr & AR9344_HDR_PORT_BIT_MASK);

	skb->dev = dsa_master_find_slave(dev, 0, port);
	if (!skb->dev)
		return NULL;

	return skb;
}

static const struct dsa_device_ops ar9344_netdev_ops = {
	.name	= "ar9344",
	.proto	= DSA_TAG_PROTO_AR9344,
	.xmit	= ar9344_tag_xmit,
	.rcv	= ar9344_tag_rcv,
	.overhead = AR9344_HDR_LEN,
};

MODULE_LICENSE("GPL");
MODULE_ALIAS_DSA_TAG_DRIVER(DSA_TAG_PROTO_AR9344);

module_dsa_tag_driver(ar9344_netdev_ops);
