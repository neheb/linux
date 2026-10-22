// SPDX-License-Identifier: GPL-2.0-or-later
/*

  Broadcom B43 wireless driver

  DMA ringbuffer and descriptor allocation/management

  Copyright (c) 2005, 2006 Michael Buesch <m@bues.ch>

  Some code in this file is derived from the b44.c driver
  Copyright (C) 2002 David S. Miller
  Copyright (C) Pekka Pietikainen


*/

#include "b43.h"
#include "dma.h"
#include "main.h"
#include "debugfs.h"
#include "xmit.h"

#include <linux/dma-mapping.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/skbuff.h>
#include <linux/etherdevice.h>
#include <linux/slab.h>
#include <asm/div64.h>
#include <net/page_pool/helpers.h>


/* Required number of TX DMA slots per TX frame.
 * This currently is 2, because we put the header and the ieee80211 frame
 * into separate slots. */
#define TX_SLOTS_PER_FRAME	2

static u32 b43_dma_address(struct b43_dma *dma, dma_addr_t dmaaddr,
			   enum b43_addrtype addrtype)
{
	u32 addr;

	switch (addrtype) {
	case B43_DMA_ADDR_LOW:
		addr = lower_32_bits(dmaaddr);
		if (dma->translation_in_low) {
			addr &= ~SSB_DMA_TRANSLATION_MASK;
			addr |= dma->translation;
		}
		break;
	case B43_DMA_ADDR_HIGH:
		addr = upper_32_bits(dmaaddr);
		if (!dma->translation_in_low) {
			addr &= ~SSB_DMA_TRANSLATION_MASK;
			addr |= dma->translation;
		}
		break;
	case B43_DMA_ADDR_EXT:
		if (dma->translation_in_low)
			addr = lower_32_bits(dmaaddr);
		else
			addr = upper_32_bits(dmaaddr);
		addr &= SSB_DMA_TRANSLATION_MASK;
		addr >>= SSB_DMA_TRANSLATION_SHIFT;
		break;
	}

	return addr;
}

/* 32bit DMA ops. */
static
struct b43_dmadesc_generic *op32_idx2desc(struct b43_dmaring *ring,
					  int slot,
					  struct b43_dmadesc_meta **meta)
{
	struct b43_dmadesc32 *desc;

	*meta = &(ring->meta[slot]);
	desc = ring->descbase;
	desc = &(desc[slot]);

	return (struct b43_dmadesc_generic *)desc;
}

static void op32_fill_descriptor(struct b43_dmaring *ring,
				 struct b43_dmadesc_generic *desc,
				 dma_addr_t dmaaddr, u16 bufsize,
				 int start, int end, int irq)
{
	struct b43_dmadesc32 *descbase = ring->descbase;
	int slot;
	u32 ctl;
	u32 addr;
	u32 addrext;

	slot = (int)(&(desc->dma32) - descbase);
	B43_WARN_ON(!(slot >= 0 && slot < ring->nr_slots));

	addr = b43_dma_address(&ring->dev->dma, dmaaddr, B43_DMA_ADDR_LOW);
	addrext = b43_dma_address(&ring->dev->dma, dmaaddr, B43_DMA_ADDR_EXT);

	ctl = bufsize & B43_DMA32_DCTL_BYTECNT;
	if (slot == ring->nr_slots - 1)
		ctl |= B43_DMA32_DCTL_DTABLEEND;
	if (start)
		ctl |= B43_DMA32_DCTL_FRAMESTART;
	if (end)
		ctl |= B43_DMA32_DCTL_FRAMEEND;
	if (irq)
		ctl |= B43_DMA32_DCTL_IRQ;
	ctl |= (addrext << B43_DMA32_DCTL_ADDREXT_SHIFT)
	    & B43_DMA32_DCTL_ADDREXT_MASK;

	desc->dma32.control = cpu_to_le32(ctl);
	desc->dma32.address = cpu_to_le32(addr);
}

static void op32_poke_tx(struct b43_dmaring *ring, int slot)
{
	b43_dma_write(ring, B43_DMA32_TXINDEX,
		      (u32) (slot * sizeof(struct b43_dmadesc32)));
}

static void op32_tx_suspend(struct b43_dmaring *ring)
{
	b43_dma_write(ring, B43_DMA32_TXCTL, b43_dma_read(ring, B43_DMA32_TXCTL)
		      | B43_DMA32_TXSUSPEND);
}

static void op32_tx_resume(struct b43_dmaring *ring)
{
	b43_dma_write(ring, B43_DMA32_TXCTL, b43_dma_read(ring, B43_DMA32_TXCTL)
		      & ~B43_DMA32_TXSUSPEND);
}

static int op32_get_current_rxslot(struct b43_dmaring *ring)
{
	u32 val;

	val = b43_dma_read(ring, B43_DMA32_RXSTATUS);
	val &= B43_DMA32_RXDPTR;

	return (val / sizeof(struct b43_dmadesc32));
}

static void op32_set_current_rxslot(struct b43_dmaring *ring, int slot)
{
	b43_dma_write(ring, B43_DMA32_RXINDEX,
		      (u32) (slot * sizeof(struct b43_dmadesc32)));
}

static const struct b43_dma_ops dma32_ops = {
	.idx2desc = op32_idx2desc,
	.fill_descriptor = op32_fill_descriptor,
	.poke_tx = op32_poke_tx,
	.tx_suspend = op32_tx_suspend,
	.tx_resume = op32_tx_resume,
	.get_current_rxslot = op32_get_current_rxslot,
	.set_current_rxslot = op32_set_current_rxslot,
};

/* 64bit DMA ops. */
static
struct b43_dmadesc_generic *op64_idx2desc(struct b43_dmaring *ring,
					  int slot,
					  struct b43_dmadesc_meta **meta)
{
	struct b43_dmadesc64 *desc;

	*meta = &(ring->meta[slot]);
	desc = ring->descbase;
	desc = &(desc[slot]);

	return (struct b43_dmadesc_generic *)desc;
}

static void op64_fill_descriptor(struct b43_dmaring *ring,
				 struct b43_dmadesc_generic *desc,
				 dma_addr_t dmaaddr, u16 bufsize,
				 int start, int end, int irq)
{
	struct b43_dmadesc64 *descbase = ring->descbase;
	int slot;
	u32 ctl0 = 0, ctl1 = 0;
	u32 addrlo, addrhi;
	u32 addrext;

	slot = (int)(&(desc->dma64) - descbase);
	B43_WARN_ON(!(slot >= 0 && slot < ring->nr_slots));

	addrlo = b43_dma_address(&ring->dev->dma, dmaaddr, B43_DMA_ADDR_LOW);
	addrhi = b43_dma_address(&ring->dev->dma, dmaaddr, B43_DMA_ADDR_HIGH);
	addrext = b43_dma_address(&ring->dev->dma, dmaaddr, B43_DMA_ADDR_EXT);

	if (slot == ring->nr_slots - 1)
		ctl0 |= B43_DMA64_DCTL0_DTABLEEND;
	if (start)
		ctl0 |= B43_DMA64_DCTL0_FRAMESTART;
	if (end)
		ctl0 |= B43_DMA64_DCTL0_FRAMEEND;
	if (irq)
		ctl0 |= B43_DMA64_DCTL0_IRQ;
	ctl1 |= bufsize & B43_DMA64_DCTL1_BYTECNT;
	ctl1 |= (addrext << B43_DMA64_DCTL1_ADDREXT_SHIFT)
	    & B43_DMA64_DCTL1_ADDREXT_MASK;

	desc->dma64.control0 = cpu_to_le32(ctl0);
	desc->dma64.control1 = cpu_to_le32(ctl1);
	desc->dma64.address_low = cpu_to_le32(addrlo);
	desc->dma64.address_high = cpu_to_le32(addrhi);
}

static void op64_poke_tx(struct b43_dmaring *ring, int slot)
{
	b43_dma_write(ring, B43_DMA64_TXINDEX,
		      (u32) (slot * sizeof(struct b43_dmadesc64)));
}

static void op64_tx_suspend(struct b43_dmaring *ring)
{
	b43_dma_write(ring, B43_DMA64_TXCTL, b43_dma_read(ring, B43_DMA64_TXCTL)
		      | B43_DMA64_TXSUSPEND);
}

static void op64_tx_resume(struct b43_dmaring *ring)
{
	b43_dma_write(ring, B43_DMA64_TXCTL, b43_dma_read(ring, B43_DMA64_TXCTL)
		      & ~B43_DMA64_TXSUSPEND);
}

static int op64_get_current_rxslot(struct b43_dmaring *ring)
{
	u32 val;

	val = b43_dma_read(ring, B43_DMA64_RXSTATUS);
	val &= B43_DMA64_RXSTATDPTR;

	return (val / sizeof(struct b43_dmadesc64));
}

static void op64_set_current_rxslot(struct b43_dmaring *ring, int slot)
{
	b43_dma_write(ring, B43_DMA64_RXINDEX,
		      (u32) (slot * sizeof(struct b43_dmadesc64)));
}

static const struct b43_dma_ops dma64_ops = {
	.idx2desc = op64_idx2desc,
	.fill_descriptor = op64_fill_descriptor,
	.poke_tx = op64_poke_tx,
	.tx_suspend = op64_tx_suspend,
	.tx_resume = op64_tx_resume,
	.get_current_rxslot = op64_get_current_rxslot,
	.set_current_rxslot = op64_set_current_rxslot,
};

static inline int free_slots(struct b43_dmaring *ring)
{
	return (ring->nr_slots - ring->used_slots);
}

static inline int next_slot(struct b43_dmaring *ring, int slot)
{
	B43_WARN_ON(!(slot >= -1 && slot <= ring->nr_slots - 1));
	if (slot == ring->nr_slots - 1)
		return 0;
	return slot + 1;
}

static inline int prev_slot(struct b43_dmaring *ring, int slot)
{
	B43_WARN_ON(!(slot >= 0 && slot <= ring->nr_slots - 1));
	if (slot == 0)
		return ring->nr_slots - 1;
	return slot - 1;
}

#ifdef CONFIG_B43_DEBUG
static void update_max_used_slots(struct b43_dmaring *ring,
				  int current_used_slots)
{
	if (current_used_slots <= ring->max_used_slots)
		return;
	ring->max_used_slots = current_used_slots;
	if (b43_debug(ring->dev, B43_DBG_DMAVERBOSE)) {
		b43dbg(ring->dev->wl,
		       "max_used_slots increased to %d on %s ring %d\n",
		       ring->max_used_slots,
		       ring->tx ? "TX" : "RX", ring->index);
	}
}
#else
static inline
    void update_max_used_slots(struct b43_dmaring *ring, int current_used_slots)
{
}
#endif /* DEBUG */

/* Request a slot for usage. */
static inline int request_slot(struct b43_dmaring *ring)
{
	int slot;

	B43_WARN_ON(!ring->tx);
	B43_WARN_ON(ring->stopped);
	B43_WARN_ON(free_slots(ring) == 0);

	slot = next_slot(ring, ring->current_slot);
	ring->current_slot = slot;
	ring->used_slots++;

	update_max_used_slots(ring, ring->used_slots);

	return slot;
}

static u16 b43_dmacontroller_base(enum b43_dmatype type, int controller_idx)
{
	static const u16 map64[] = {
		B43_MMIO_DMA64_BASE0,
		B43_MMIO_DMA64_BASE1,
		B43_MMIO_DMA64_BASE2,
		B43_MMIO_DMA64_BASE3,
		B43_MMIO_DMA64_BASE4,
		B43_MMIO_DMA64_BASE5,
	};
	static const u16 map32[] = {
		B43_MMIO_DMA32_BASE0,
		B43_MMIO_DMA32_BASE1,
		B43_MMIO_DMA32_BASE2,
		B43_MMIO_DMA32_BASE3,
		B43_MMIO_DMA32_BASE4,
		B43_MMIO_DMA32_BASE5,
	};

	if (type == B43_DMA_64BIT) {
		B43_WARN_ON(!(controller_idx >= 0 &&
			      controller_idx < ARRAY_SIZE(map64)));
		return map64[controller_idx];
	}
	B43_WARN_ON(!(controller_idx >= 0 &&
		      controller_idx < ARRAY_SIZE(map32)));
	return map32[controller_idx];
}

static inline
    dma_addr_t map_descbuffer(struct b43_dmaring *ring,
			      unsigned char *buf, size_t len, int tx)
{
	dma_addr_t dmaaddr;

	if (tx) {
		dmaaddr = dma_map_single(ring->dev->dev->dma_dev,
					 buf, len, DMA_TO_DEVICE);
	} else {
		dmaaddr = dma_map_single(ring->dev->dev->dma_dev,
					 buf, len, DMA_FROM_DEVICE);
	}

	return dmaaddr;
}

static inline
    void unmap_descbuffer(struct b43_dmaring *ring,
			  dma_addr_t addr, size_t len, int tx)
{
	if (tx) {
		dma_unmap_single(ring->dev->dev->dma_dev,
				 addr, len, DMA_TO_DEVICE);
	} else {
		dma_unmap_single(ring->dev->dev->dma_dev,
				 addr, len, DMA_FROM_DEVICE);
	}
}

static inline
    void sync_descbuffer_for_cpu(struct b43_dmaring *ring,
				 dma_addr_t addr, size_t len)
{
	B43_WARN_ON(ring->tx);
	dma_sync_single_for_cpu(ring->dev->dev->dma_dev,
				    addr, len, DMA_FROM_DEVICE);
}

static inline
    void sync_descbuffer_for_device(struct b43_dmaring *ring,
				    dma_addr_t addr, size_t len)
{
	B43_WARN_ON(ring->tx);
	dma_sync_single_for_device(ring->dev->dev->dma_dev,
				   addr, len, DMA_FROM_DEVICE);
}

static inline
    void free_descriptor_buffer(struct b43_dmaring *ring,
				struct b43_dmadesc_meta *meta)
{
	if (meta->skb) {
		if (ring->tx)
			ieee80211_free_txskb(ring->dev->wl->hw, meta->skb);
		else
			dev_kfree_skb_any(meta->skb);
		meta->skb = NULL;
	}
}

static int alloc_ringmemory(struct b43_dmaring *ring)
{
	/* The specs call for 4K buffers for 30- and 32-bit DMA with 4K
	 * alignment and 8K buffers for 64-bit DMA with 8K alignment.
	 * In practice we could use smaller buffers for the latter, but the
	 * alignment is really important because of the hardware bug. If bit
	 * 0x00001000 is used in DMA address, some hardware (like BCM4331)
	 * copies that bit into B43_DMA64_RXSTATUS and we get false values from
	 * B43_DMA64_RXSTATDPTR. Let's just use 8K buffers even if we don't use
	 * more than 256 slots for ring.
	 */
	u16 ring_mem_size = (ring->type == B43_DMA_64BIT) ?
				B43_DMA64_RINGMEMSIZE : B43_DMA32_RINGMEMSIZE;

	ring->descbase = dma_alloc_coherent(ring->dev->dev->dma_dev,
					    ring_mem_size, &(ring->dmabase),
					    GFP_KERNEL);
	if (!ring->descbase)
		return -ENOMEM;

	return 0;
}

static void free_ringmemory(struct b43_dmaring *ring)
{
	u16 ring_mem_size = (ring->type == B43_DMA_64BIT) ?
				B43_DMA64_RINGMEMSIZE : B43_DMA32_RINGMEMSIZE;
	dma_free_coherent(ring->dev->dev->dma_dev, ring_mem_size,
			  ring->descbase, ring->dmabase);
}

/* Reset the RX DMA channel */
static int b43_dmacontroller_rx_reset(struct b43_wldev *dev, u16 mmio_base,
				      enum b43_dmatype type)
{
	int i;
	u32 value;
	u16 offset;

	might_sleep();

	offset = (type == B43_DMA_64BIT) ? B43_DMA64_RXCTL : B43_DMA32_RXCTL;
	b43_write32(dev, mmio_base + offset, 0);
	for (i = 0; i < 10; i++) {
		offset = (type == B43_DMA_64BIT) ? B43_DMA64_RXSTATUS :
						   B43_DMA32_RXSTATUS;
		value = b43_read32(dev, mmio_base + offset);
		if (type == B43_DMA_64BIT) {
			value &= B43_DMA64_RXSTAT;
			if (value == B43_DMA64_RXSTAT_DISABLED) {
				i = -1;
				break;
			}
		} else {
			value &= B43_DMA32_RXSTATE;
			if (value == B43_DMA32_RXSTAT_DISABLED) {
				i = -1;
				break;
			}
		}
		msleep(1);
	}
	if (i != -1) {
		b43err(dev->wl, "DMA RX reset timed out\n");
		return -ENODEV;
	}

	return 0;
}

/* Reset the TX DMA channel */
static int b43_dmacontroller_tx_reset(struct b43_wldev *dev, u16 mmio_base,
				      enum b43_dmatype type)
{
	int i;
	u32 value;
	u16 offset;

	might_sleep();

	for (i = 0; i < 10; i++) {
		offset = (type == B43_DMA_64BIT) ? B43_DMA64_TXSTATUS :
						   B43_DMA32_TXSTATUS;
		value = b43_read32(dev, mmio_base + offset);
		if (type == B43_DMA_64BIT) {
			value &= B43_DMA64_TXSTAT;
			if (value == B43_DMA64_TXSTAT_DISABLED ||
			    value == B43_DMA64_TXSTAT_IDLEWAIT ||
			    value == B43_DMA64_TXSTAT_STOPPED)
				break;
		} else {
			value &= B43_DMA32_TXSTATE;
			if (value == B43_DMA32_TXSTAT_DISABLED ||
			    value == B43_DMA32_TXSTAT_IDLEWAIT ||
			    value == B43_DMA32_TXSTAT_STOPPED)
				break;
		}
		msleep(1);
	}
	offset = (type == B43_DMA_64BIT) ? B43_DMA64_TXCTL : B43_DMA32_TXCTL;
	b43_write32(dev, mmio_base + offset, 0);
	for (i = 0; i < 10; i++) {
		offset = (type == B43_DMA_64BIT) ? B43_DMA64_TXSTATUS :
						   B43_DMA32_TXSTATUS;
		value = b43_read32(dev, mmio_base + offset);
		if (type == B43_DMA_64BIT) {
			value &= B43_DMA64_TXSTAT;
			if (value == B43_DMA64_TXSTAT_DISABLED) {
				i = -1;
				break;
			}
		} else {
			value &= B43_DMA32_TXSTATE;
			if (value == B43_DMA32_TXSTAT_DISABLED) {
				i = -1;
				break;
			}
		}
		msleep(1);
	}
	if (i != -1) {
		b43err(dev->wl, "DMA TX reset timed out\n");
		return -ENODEV;
	}
	/* ensure the reset is completed. */
	msleep(1);

	return 0;
}

/* Check if a DMA mapping address is invalid. */
static bool b43_dma_mapping_error(struct b43_dmaring *ring,
				  dma_addr_t addr,
				  size_t buffersize, bool dma_to_device)
{
	if (unlikely(dma_mapping_error(ring->dev->dev->dma_dev, addr)))
		return true;

	switch (ring->type) {
	case B43_DMA_30BIT:
		if ((u64)addr + buffersize > (1ULL << 30))
			goto address_error;
		break;
	case B43_DMA_32BIT:
		if ((u64)addr + buffersize > (1ULL << 32))
			goto address_error;
		break;
	case B43_DMA_64BIT:
		/* Currently we can't have addresses beyond
		 * 64bit in the kernel. */
		break;
	}

	/* The address is OK. */
	return false;

address_error:
	/* We can't support this address. Unmap it again. */
	unmap_descbuffer(ring, addr, buffersize, dma_to_device);

	return true;
}

static bool b43_rx_buffer_is_poisoned(struct b43_dmaring *ring, struct sk_buff *skb)
{
	unsigned char *f = skb->data + ring->frameoffset;

	return ((f[0] & f[1] & f[2] & f[3] & f[4] & f[5] & f[6] & f[7]) == 0xFF);
}

static void b43_poison_rx_buffer(struct b43_dmaring *ring, struct sk_buff *skb)
{
	struct b43_rxhdr_fw4 *rxhdr;
	unsigned char *frame;

	/* This poisons the RX buffer to detect DMA failures. */

	rxhdr = (struct b43_rxhdr_fw4 *)(skb->data);
	rxhdr->frame_len = 0;

	B43_WARN_ON(ring->rx_buffersize < ring->frameoffset + sizeof(struct b43_plcp_hdr6) + 2);
	frame = skb->data + ring->frameoffset;
	memset(frame, 0xFF, sizeof(struct b43_plcp_hdr6) + 2 /* padding */);
}

static int setup_rx_descbuffer(struct b43_dmaring *ring,
			       struct b43_dmadesc_generic *desc,
			       struct b43_dmadesc_meta *meta, gfp_t gfp_flags)
{
	struct page_pool *pp = ring->page_pool;
	dma_addr_t dmaaddr;
	struct sk_buff *skb;
	unsigned int size;
	struct page *page;
	void *data;

	B43_WARN_ON(ring->tx);

	size = SKB_DATA_ALIGN(NET_SKB_PAD + ring->rx_buffersize) +
	       SKB_DATA_ALIGN(sizeof(struct skb_shared_info));

	data = page_pool_alloc_va(pp, &size, gfp_flags);
	if (unlikely(!data))
		return -ENOMEM;

	skb = build_skb(data, size);
	if (unlikely(!skb)) {
		page_pool_free_va(pp, data, false);
		return -ENOMEM;
	}
	skb_mark_for_recycle(skb);
	skb_reserve(skb, NET_SKB_PAD);

	page = virt_to_head_page(data);
	dmaaddr = page_pool_get_dma_addr(page) +
		  (skb->data - (u8 *)page_address(page));

	b43_poison_rx_buffer(ring, skb);
	sync_descbuffer_for_device(ring, dmaaddr, ring->rx_buffersize);

	meta->skb = skb;
	meta->dmaaddr = dmaaddr;
	ring->ops->fill_descriptor(ring, desc, dmaaddr,
				   ring->rx_buffersize, 0, 0, 0);

	return 0;
}

static int b43_rx_page_pool_create(struct b43_dmaring *ring)
{
	struct page_pool_params pp_params = {};
	struct page_pool *pp;

	pp_params.order = 0;
	pp_params.pool_size = ring->nr_slots;
	pp_params.nid = NUMA_NO_NODE;
	pp_params.dev = ring->dev->dev->dma_dev;
	pp_params.dma_dir = DMA_FROM_DEVICE;
	pp_params.flags = PP_FLAG_DMA_MAP;

	pp = page_pool_create(&pp_params);
	if (IS_ERR(pp)) {
		b43err(ring->dev->wl, "Failed to create the RX page pool\n");
		return PTR_ERR(pp);
	}

	ring->page_pool = pp;
	return 0;
}

/* Allocate the initial descbuffers.
 * This is used for an RX ring only.
 */
static int alloc_initial_descbuffers(struct b43_dmaring *ring)
{
	int i, err = -ENOMEM;
	struct b43_dmadesc_generic *desc;
	struct b43_dmadesc_meta *meta;

	for (i = 0; i < ring->nr_slots; i++) {
		desc = ring->ops->idx2desc(ring, i, &meta);

		err = setup_rx_descbuffer(ring, desc, meta, GFP_KERNEL);
		if (err) {
			b43err(ring->dev->wl,
			       "Failed to allocate initial descbuffers\n");
			goto err_unwind;
		}
	}
	mb();
	ring->used_slots = ring->nr_slots;
	err = 0;
      out:
	return err;

      err_unwind:
	for (i--; i >= 0; i--) {
		desc = ring->ops->idx2desc(ring, i, &meta);

		dev_kfree_skb(meta->skb);
	}
	goto out;
}

/* Do initial setup of the DMA controller.
 * Reset the controller, write the ring busaddress
 * and switch the "enable" bit on.
 */
static int dmacontroller_setup(struct b43_dmaring *ring)
{
	int err = 0;
	u32 value;
	u32 addrext;
	bool parity = ring->dev->dma.parity;
	u32 addrlo;
	u32 addrhi;

	if (ring->tx) {
		if (ring->type == B43_DMA_64BIT) {
			u64 ringbase = (u64) (ring->dmabase);
			addrext = b43_dma_address(&ring->dev->dma, ringbase, B43_DMA_ADDR_EXT);
			addrlo = b43_dma_address(&ring->dev->dma, ringbase, B43_DMA_ADDR_LOW);
			addrhi = b43_dma_address(&ring->dev->dma, ringbase, B43_DMA_ADDR_HIGH);

			value = B43_DMA64_TXENABLE;
			value |= (addrext << B43_DMA64_TXADDREXT_SHIFT)
			    & B43_DMA64_TXADDREXT_MASK;
			if (!parity)
				value |= B43_DMA64_TXPARITYDISABLE;
			b43_dma_write(ring, B43_DMA64_TXCTL, value);
			b43_dma_write(ring, B43_DMA64_TXRINGLO, addrlo);
			b43_dma_write(ring, B43_DMA64_TXRINGHI, addrhi);
		} else {
			u32 ringbase = (u32) (ring->dmabase);
			addrext = b43_dma_address(&ring->dev->dma, ringbase, B43_DMA_ADDR_EXT);
			addrlo = b43_dma_address(&ring->dev->dma, ringbase, B43_DMA_ADDR_LOW);

			value = B43_DMA32_TXENABLE;
			value |= (addrext << B43_DMA32_TXADDREXT_SHIFT)
			    & B43_DMA32_TXADDREXT_MASK;
			if (!parity)
				value |= B43_DMA32_TXPARITYDISABLE;
			b43_dma_write(ring, B43_DMA32_TXCTL, value);
			b43_dma_write(ring, B43_DMA32_TXRING, addrlo);
		}
	} else {
		err = alloc_initial_descbuffers(ring);
		if (err)
			goto out;
		if (ring->type == B43_DMA_64BIT) {
			u64 ringbase = (u64) (ring->dmabase);
			addrext = b43_dma_address(&ring->dev->dma, ringbase, B43_DMA_ADDR_EXT);
			addrlo = b43_dma_address(&ring->dev->dma, ringbase, B43_DMA_ADDR_LOW);
			addrhi = b43_dma_address(&ring->dev->dma, ringbase, B43_DMA_ADDR_HIGH);

			value = (ring->frameoffset << B43_DMA64_RXFROFF_SHIFT);
			value |= B43_DMA64_RXENABLE;
			value |= (addrext << B43_DMA64_RXADDREXT_SHIFT)
			    & B43_DMA64_RXADDREXT_MASK;
			if (!parity)
				value |= B43_DMA64_RXPARITYDISABLE;
			b43_dma_write(ring, B43_DMA64_RXCTL, value);
			b43_dma_write(ring, B43_DMA64_RXRINGLO, addrlo);
			b43_dma_write(ring, B43_DMA64_RXRINGHI, addrhi);
			b43_dma_write(ring, B43_DMA64_RXINDEX, ring->nr_slots *
				      sizeof(struct b43_dmadesc64));
		} else {
			u32 ringbase = (u32) (ring->dmabase);
			addrext = b43_dma_address(&ring->dev->dma, ringbase, B43_DMA_ADDR_EXT);
			addrlo = b43_dma_address(&ring->dev->dma, ringbase, B43_DMA_ADDR_LOW);

			value = (ring->frameoffset << B43_DMA32_RXFROFF_SHIFT);
			value |= B43_DMA32_RXENABLE;
			value |= (addrext << B43_DMA32_RXADDREXT_SHIFT)
			    & B43_DMA32_RXADDREXT_MASK;
			if (!parity)
				value |= B43_DMA32_RXPARITYDISABLE;
			b43_dma_write(ring, B43_DMA32_RXCTL, value);
			b43_dma_write(ring, B43_DMA32_RXRING, addrlo);
			b43_dma_write(ring, B43_DMA32_RXINDEX, ring->nr_slots *
				      sizeof(struct b43_dmadesc32));
		}
	}

out:
	return err;
}

/* Shutdown the DMA controller. */
static void dmacontroller_cleanup(struct b43_dmaring *ring)
{
	if (ring->tx) {
		b43_dmacontroller_tx_reset(ring->dev, ring->mmio_base,
					   ring->type);
		if (ring->type == B43_DMA_64BIT) {
			b43_dma_write(ring, B43_DMA64_TXRINGLO, 0);
			b43_dma_write(ring, B43_DMA64_TXRINGHI, 0);
		} else
			b43_dma_write(ring, B43_DMA32_TXRING, 0);
	} else {
		b43_dmacontroller_rx_reset(ring->dev, ring->mmio_base,
					   ring->type);
		if (ring->type == B43_DMA_64BIT) {
			b43_dma_write(ring, B43_DMA64_RXRINGLO, 0);
			b43_dma_write(ring, B43_DMA64_RXRINGHI, 0);
		} else
			b43_dma_write(ring, B43_DMA32_RXRING, 0);
	}
}

static void free_all_descbuffers(struct b43_dmaring *ring)
{
	struct b43_dmadesc_meta *meta;
	int i;

	if (!ring->used_slots)
		return;
	for (i = 0; i < ring->nr_slots; i++) {
		/* get meta - ignore returned value */
		ring->ops->idx2desc(ring, i, &meta);

		if (!meta->skb || b43_dma_ptr_is_poisoned(meta->skb)) {
			B43_WARN_ON(!ring->tx);
			continue;
		}
		if (ring->tx)
			unmap_descbuffer(ring, meta->dmaaddr,
					 meta->skb->len, 1);
		free_descriptor_buffer(ring, meta);
	}
}

static enum b43_dmatype b43_engine_type(struct b43_wldev *dev)
{
	u32 tmp;
	u16 mmio_base;

	switch (dev->dev->bus_type) {
#ifdef CONFIG_B43_BCMA
	case B43_BUS_BCMA:
		tmp = bcma_aread32(dev->dev->bdev, BCMA_IOST);
		if (tmp & BCMA_IOST_DMA64)
			return B43_DMA_64BIT;
		break;
#endif
#ifdef CONFIG_B43_SSB
	case B43_BUS_SSB:
		tmp = ssb_read32(dev->dev->sdev, SSB_TMSHIGH);
		if (tmp & SSB_TMSHIGH_DMA64)
			return B43_DMA_64BIT;
		break;
#endif
	}

	mmio_base = b43_dmacontroller_base(0, 0);
	b43_write32(dev, mmio_base + B43_DMA32_TXCTL, B43_DMA32_TXADDREXT_MASK);
	tmp = b43_read32(dev, mmio_base + B43_DMA32_TXCTL);
	if (tmp & B43_DMA32_TXADDREXT_MASK)
		return B43_DMA_32BIT;
	return B43_DMA_30BIT;
}

/* Main initialization function. */
static
struct b43_dmaring *b43_setup_dmaring(struct b43_wldev *dev,
				      int controller_index,
				      int for_tx,
				      enum b43_dmatype type)
{
	struct b43_dmaring *ring;
	int i, err;
	dma_addr_t dma_test;
	size_t nr_slots;

	if (for_tx)
		nr_slots = B43_TXRING_SLOTS;
	else
		nr_slots = B43_RXRING_SLOTS;

	ring = kzalloc_flex(*ring, meta, nr_slots);
	if (!ring)
		goto out;

	ring->nr_slots = nr_slots;

	for (i = 0; i < ring->nr_slots; i++)
		ring->meta->skb = B43_DMA_PTR_POISON;

	ring->type = type;
	ring->dev = dev;
	ring->mmio_base = b43_dmacontroller_base(type, controller_index);
	ring->index = controller_index;
	if (type == B43_DMA_64BIT)
		ring->ops = &dma64_ops;
	else
		ring->ops = &dma32_ops;
	if (for_tx) {
		ring->tx = true;
		ring->current_slot = -1;
	} else {
		if (ring->index == 0) {
			switch (dev->fw.hdr_format) {
			case B43_FW_HDR_598:
				ring->rx_buffersize = B43_DMA0_RX_FW598_BUFSIZE;
				ring->frameoffset = B43_DMA0_RX_FW598_FO;
				break;
			case B43_FW_HDR_410:
			case B43_FW_HDR_351:
				ring->rx_buffersize = B43_DMA0_RX_FW351_BUFSIZE;
				ring->frameoffset = B43_DMA0_RX_FW351_FO;
				break;
			}
		} else
			B43_WARN_ON(1);
	}
#ifdef CONFIG_B43_DEBUG
	ring->last_injected_overflow = jiffies;
#endif

	if (for_tx) {
		/* Assumption: B43_TXRING_SLOTS can be divided by TX_SLOTS_PER_FRAME */
		BUILD_BUG_ON(B43_TXRING_SLOTS % TX_SLOTS_PER_FRAME != 0);

		ring->txhdr_cache = kcalloc(ring->nr_slots / TX_SLOTS_PER_FRAME,
					    b43_txhdr_size(dev),
					    GFP_KERNEL);
		if (!ring->txhdr_cache)
			goto err_kfree_meta;

		/* test for ability to dma to txhdr_cache */
		dma_test = dma_map_single(dev->dev->dma_dev,
					  ring->txhdr_cache,
					  b43_txhdr_size(dev),
					  DMA_TO_DEVICE);

		if (b43_dma_mapping_error(ring, dma_test,
					  b43_txhdr_size(dev), 1)) {
			/* ugh realloc */
			kfree(ring->txhdr_cache);
			ring->txhdr_cache = kcalloc(ring->nr_slots / TX_SLOTS_PER_FRAME,
						    b43_txhdr_size(dev),
						    GFP_KERNEL | GFP_DMA);
			if (!ring->txhdr_cache)
				goto err_kfree_meta;

			dma_test = dma_map_single(dev->dev->dma_dev,
						  ring->txhdr_cache,
						  b43_txhdr_size(dev),
						  DMA_TO_DEVICE);

			if (b43_dma_mapping_error(ring, dma_test,
						  b43_txhdr_size(dev), 1)) {

				b43err(dev->wl,
				       "TXHDR DMA allocation failed\n");
				goto err_kfree_txhdr_cache;
			}
		}

		dma_unmap_single(dev->dev->dma_dev,
				 dma_test, b43_txhdr_size(dev),
				 DMA_TO_DEVICE);
	}

	if (!for_tx) {
		err = b43_rx_page_pool_create(ring);
		if (err)
			goto err_kfree_meta;
	}

	err = alloc_ringmemory(ring);
	if (err)
		goto err_destroy_page_pool;
	err = dmacontroller_setup(ring);
	if (err)
		goto err_free_ringmemory;

      out:
	return ring;

      err_free_ringmemory:
	free_ringmemory(ring);
      err_destroy_page_pool:
	if (ring->page_pool)
		page_pool_destroy(ring->page_pool);
      err_kfree_txhdr_cache:
	kfree(ring->txhdr_cache);
      err_kfree_meta:
	kfree(ring);
	ring = NULL;
	goto out;
}

#define divide(a, b)	({	\
	typeof(a) __a = a;	\
	do_div(__a, b);		\
	__a;			\
  })

#define modulo(a, b)	({	\
	typeof(a) __a = a;	\
	do_div(__a, b);		\
  })

/* Main cleanup function. */
static void b43_destroy_dmaring(struct b43_dmaring *ring,
				const char *ringname)
{
	if (!ring)
		return;

#ifdef CONFIG_B43_DEBUG
	{
		/* Print some statistics. */
		u64 failed_packets = ring->nr_failed_tx_packets;
		u64 succeed_packets = ring->nr_succeed_tx_packets;
		u64 nr_packets = failed_packets + succeed_packets;
		u64 permille_failed = 0, average_tries = 0;

		if (nr_packets)
			permille_failed = divide(failed_packets * 1000, nr_packets);
		if (nr_packets)
			average_tries = divide(ring->nr_total_packet_tries * 100, nr_packets);

		b43dbg(ring->dev->wl, "DMA-%u %s: "
		       "Used slots %d/%d, Failed frames %llu/%llu = %llu.%01llu%%, "
		       "Average tries %llu.%02llu\n",
		       (unsigned int)(ring->type), ringname,
		       ring->max_used_slots,
		       ring->nr_slots,
		       (unsigned long long)failed_packets,
		       (unsigned long long)nr_packets,
		       (unsigned long long)divide(permille_failed, 10),
		       (unsigned long long)modulo(permille_failed, 10),
		       (unsigned long long)divide(average_tries, 100),
		       (unsigned long long)modulo(average_tries, 100));
	}
#endif /* DEBUG */

	/* Device IRQs are disabled prior entering this function,
	 * so no need to take care of concurrency with rx handler stuff.
	 */
	dmacontroller_cleanup(ring);
	free_all_descbuffers(ring);
	free_ringmemory(ring);
	if (ring->page_pool)
		page_pool_destroy(ring->page_pool);

	kfree(ring->txhdr_cache);
	kfree(ring);
}

#define destroy_ring(dma, ring) do {				\
	b43_destroy_dmaring((dma)->ring, __stringify(ring));	\
	(dma)->ring = NULL;					\
    } while (0)

void b43_dma_free(struct b43_wldev *dev)
{
	struct b43_dma *dma;

	if (b43_using_pio_transfers(dev))
		return;
	dma = &dev->dma;

	destroy_ring(dma, rx_ring);
	destroy_ring(dma, tx_ring_AC_BK);
	destroy_ring(dma, tx_ring_AC_BE);
	destroy_ring(dma, tx_ring_AC_VI);
	destroy_ring(dma, tx_ring_AC_VO);
	destroy_ring(dma, tx_ring_mcast);
}

/* Some hardware with 64-bit DMA seems to be bugged and looks for translation
 * bit in low address word instead of high one.
 */
static bool b43_dma_translation_in_low_word(struct b43_wldev *dev,
					    enum b43_dmatype type)
{
	if (type != B43_DMA_64BIT)
		return true;

#ifdef CONFIG_B43_SSB
	if (dev->dev->bus_type == B43_BUS_SSB &&
	    dev->dev->sdev->bus->bustype == SSB_BUSTYPE_PCI &&
	    !(pci_is_pcie(dev->dev->sdev->bus->host_pci) &&
	      ssb_read32(dev->dev->sdev, SSB_TMSHIGH) & SSB_TMSHIGH_DMA64))
			return true;
#endif
	return false;
}

int b43_dma_init(struct b43_wldev *dev)
{
	struct b43_dma *dma = &dev->dma;
	enum b43_dmatype type = b43_engine_type(dev);
	int err;

	err = dma_set_mask_and_coherent(dev->dev->dma_dev, DMA_BIT_MASK(type));
	if (err) {
		b43err(dev->wl, "The machine/kernel does not support "
		       "the required %u-bit DMA mask\n", type);
		return err;
	}

	switch (dev->dev->bus_type) {
#ifdef CONFIG_B43_BCMA
	case B43_BUS_BCMA:
		dma->translation = bcma_core_dma_translation(dev->dev->bdev);
		break;
#endif
#ifdef CONFIG_B43_SSB
	case B43_BUS_SSB:
		dma->translation = ssb_dma_translation(dev->dev->sdev);
		break;
#endif
	}
	dma->translation_in_low = b43_dma_translation_in_low_word(dev, type);

	dma->parity = true;
#ifdef CONFIG_B43_BCMA
	/* TODO: find out which SSB devices need disabling parity */
	if (dev->dev->bus_type == B43_BUS_BCMA)
		dma->parity = false;
#endif

	err = -ENOMEM;
	/* setup TX DMA channels. */
	dma->tx_ring_AC_BK = b43_setup_dmaring(dev, 0, 1, type);
	if (!dma->tx_ring_AC_BK)
		goto out;

	dma->tx_ring_AC_BE = b43_setup_dmaring(dev, 1, 1, type);
	if (!dma->tx_ring_AC_BE)
		goto err_destroy_bk;

	dma->tx_ring_AC_VI = b43_setup_dmaring(dev, 2, 1, type);
	if (!dma->tx_ring_AC_VI)
		goto err_destroy_be;

	dma->tx_ring_AC_VO = b43_setup_dmaring(dev, 3, 1, type);
	if (!dma->tx_ring_AC_VO)
		goto err_destroy_vi;

	dma->tx_ring_mcast = b43_setup_dmaring(dev, 4, 1, type);
	if (!dma->tx_ring_mcast)
		goto err_destroy_vo;

	/* setup RX DMA channel. */
	dma->rx_ring = b43_setup_dmaring(dev, 0, 0, type);
	if (!dma->rx_ring)
		goto err_destroy_mcast;

	/* No support for the TX status DMA ring. */
	B43_WARN_ON(dev->dev->core_rev < 5);

	b43dbg(dev->wl, "%u-bit DMA initialized\n",
	       (unsigned int)type);
	err = 0;
out:
	return err;

err_destroy_mcast:
	destroy_ring(dma, tx_ring_mcast);
err_destroy_vo:
	destroy_ring(dma, tx_ring_AC_VO);
err_destroy_vi:
	destroy_ring(dma, tx_ring_AC_VI);
err_destroy_be:
	destroy_ring(dma, tx_ring_AC_BE);
err_destroy_bk:
	destroy_ring(dma, tx_ring_AC_BK);
	return err;
}

/* Generate a cookie for the TX header. */
static u16 generate_cookie(struct b43_dmaring *ring, int slot)
{
	u16 cookie;

	/* Use the upper 4 bits of the cookie as
	 * DMA controller ID and store the slot number
	 * in the lower 12 bits.
	 * Note that the cookie must never be 0, as this
	 * is a special value used in RX path.
	 * It can also not be 0xFFFF because that is special
	 * for multicast frames.
	 */
	cookie = (((u16)ring->index + 1) << 12);
	B43_WARN_ON(slot & ~0x0FFF);
	cookie |= (u16)slot;

	return cookie;
}

/* Inspect a cookie and find out to which controller/slot it belongs. */
static
struct b43_dmaring *parse_cookie(struct b43_wldev *dev, u16 cookie, int *slot)
{
	struct b43_dma *dma = &dev->dma;
	struct b43_dmaring *ring = NULL;

	switch (cookie & 0xF000) {
	case 0x1000:
		ring = dma->tx_ring_AC_BK;
		break;
	case 0x2000:
		ring = dma->tx_ring_AC_BE;
		break;
	case 0x3000:
		ring = dma->tx_ring_AC_VI;
		break;
	case 0x4000:
		ring = dma->tx_ring_AC_VO;
		break;
	case 0x5000:
		ring = dma->tx_ring_mcast;
		break;
	}
	*slot = (cookie & 0x0FFF);
	if (unlikely(!ring || *slot < 0 || *slot >= ring->nr_slots)) {
		b43dbg(dev->wl, "TX-status contains "
		       "invalid cookie: 0x%04X\n", cookie);
		return NULL;
	}

	return ring;
}

static int dma_tx_fragment(struct b43_dmaring *ring,
			   struct sk_buff *skb, bool poke)
{
	const struct b43_dma_ops *ops = ring->ops;
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct b43_private_tx_info *priv_info = b43_get_priv_tx_info(info);
	u8 *header;
	int slot, old_top_slot, old_used_slots;
	int err;
	struct b43_dmadesc_generic *desc;
	struct b43_dmadesc_meta *meta;
	struct b43_dmadesc_meta *meta_hdr;
	u16 cookie;
	size_t hdrsize = b43_txhdr_size(ring->dev);

	/* Important note: If the number of used DMA slots per TX frame
	 * is changed here, the TX_SLOTS_PER_FRAME definition at the top of
	 * the file has to be updated, too!
	 */

	old_top_slot = ring->current_slot;
	old_used_slots = ring->used_slots;

	/* Get a slot for the header. */
	slot = request_slot(ring);
	desc = ops->idx2desc(ring, slot, &meta_hdr);
	memset(meta_hdr, 0, sizeof(*meta_hdr));

	header = &(ring->txhdr_cache[(slot / TX_SLOTS_PER_FRAME) * hdrsize]);
	cookie = generate_cookie(ring, slot);
	err = b43_generate_txhdr(ring->dev, header,
				 skb, info, cookie);
	if (unlikely(err)) {
		ring->current_slot = old_top_slot;
		ring->used_slots = old_used_slots;
		return err;
	}

	meta_hdr->dmaaddr = map_descbuffer(ring, (unsigned char *)header,
					   hdrsize, 1);
	if (b43_dma_mapping_error(ring, meta_hdr->dmaaddr, hdrsize, 1)) {
		ring->current_slot = old_top_slot;
		ring->used_slots = old_used_slots;
		return -EIO;
	}
	ops->fill_descriptor(ring, desc, meta_hdr->dmaaddr,
			     hdrsize, 1, 0, 0);

	/* Get a slot for the payload. */
	slot = request_slot(ring);
	desc = ops->idx2desc(ring, slot, &meta);
	memset(meta, 0, sizeof(*meta));

	meta->skb = skb;
	meta->is_last_fragment = true;
	priv_info->bouncebuffer = NULL;

	meta->dmaaddr = map_descbuffer(ring, skb->data, skb->len, 1);
	/* create a bounce buffer in zone_dma on mapping failure. */
	if (b43_dma_mapping_error(ring, meta->dmaaddr, skb->len, 1)) {
		priv_info->bouncebuffer = kmemdup(skb->data, skb->len,
						  GFP_ATOMIC | GFP_DMA);
		if (!priv_info->bouncebuffer) {
			ring->current_slot = old_top_slot;
			ring->used_slots = old_used_slots;
			err = -ENOMEM;
			goto out_unmap_hdr;
		}

		meta->dmaaddr = map_descbuffer(ring, priv_info->bouncebuffer, skb->len, 1);
		if (b43_dma_mapping_error(ring, meta->dmaaddr, skb->len, 1)) {
			kfree(priv_info->bouncebuffer);
			priv_info->bouncebuffer = NULL;
			ring->current_slot = old_top_slot;
			ring->used_slots = old_used_slots;
			err = -EIO;
			goto out_unmap_hdr;
		}
	}

	ops->fill_descriptor(ring, desc, meta->dmaaddr, skb->len, 0, 1, 1);

	if (info->flags & IEEE80211_TX_CTL_SEND_AFTER_DTIM) {
		/* Tell the firmware about the cookie of the last
		 * mcast frame, so it can clear the more-data bit in it. */
		b43_shm_write16(ring->dev, B43_SHM_SHARED,
				B43_SHM_SH_MCASTCOOKIE, cookie);
	}
	/* Now transfer the whole frame, or the A-MPDU it ends. */
	if (poke) {
		/* Descriptors and buffers must be visible before the poke */
		wmb();
		ops->poke_tx(ring, next_slot(ring, slot));
	}
	return 0;

out_unmap_hdr:
	unmap_descbuffer(ring, meta_hdr->dmaaddr,
			 hdrsize, 1);
	return err;
}

static inline int should_inject_overflow(struct b43_dmaring *ring)
{
#ifdef CONFIG_B43_DEBUG
	if (unlikely(b43_debug(ring->dev, B43_DBG_DMAOVERFLOW))) {
		/* Check if we should inject another ringbuffer overflow
		 * to test handling of this situation in the stack. */
		unsigned long next_overflow;

		next_overflow = ring->last_injected_overflow + HZ;
		if (time_after(jiffies, next_overflow)) {
			ring->last_injected_overflow = jiffies;
			b43dbg(ring->dev->wl,
			       "Injecting TX ring overflow on "
			       "DMA controller %d\n", ring->index);
			return 1;
		}
	}
#endif /* CONFIG_B43_DEBUG */
	return 0;
}

/* Static mapping of mac80211's queues (priorities) to b43 DMA rings. */
static struct b43_dmaring *select_ring_by_priority(struct b43_wldev *dev,
						   u8 queue_prio)
{
	struct b43_dmaring *ring;

	if (dev->qos_enabled) {
		/* 0 = highest priority */
		switch (queue_prio) {
		default:
			B43_WARN_ON(1);
			fallthrough;
		case 0:
			ring = dev->dma.tx_ring_AC_VO;
			break;
		case 1:
			ring = dev->dma.tx_ring_AC_VI;
			break;
		case 2:
			ring = dev->dma.tx_ring_AC_BE;
			break;
		case 3:
			ring = dev->dma.tx_ring_AC_BK;
			break;
		}
	} else
		ring = dev->dma.tx_ring_AC_BE;

	return ring;
}

int b43_dma_tx(struct b43_wldev *dev, struct sk_buff *skb)
{
	struct b43_dmaring *ring;
	struct ieee80211_hdr *hdr;
	int err = 0;
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);

	hdr = (struct ieee80211_hdr *)skb->data;
	if (info->flags & IEEE80211_TX_CTL_SEND_AFTER_DTIM) {
		/* The multicast ring will be sent after the DTIM */
		ring = dev->dma.tx_ring_mcast;
		/* Set the more-data bit. Ucode will clear it on
		 * the last frame for us. */
		hdr->frame_control |= cpu_to_le16(IEEE80211_FCTL_MOREDATA);
	} else {
		/* Decide by priority where to put this frame. */
		ring = select_ring_by_priority(
			dev, skb_get_queue_mapping(skb));
	}

	B43_WARN_ON(!ring->tx);

	if (unlikely(ring->stopped)) {
		/* We get here only because of a bug in mac80211.
		 * Because of a race, one packet may be queued after
		 * the queue is stopped, thus we got called when we shouldn't.
		 * For now, just refuse the transmit. */
		if (b43_debug(dev, B43_DBG_DMAVERBOSE))
			b43err(dev->wl, "Packet after queue stopped\n");
		err = -ENOSPC;
		goto out;
	}

	if (WARN_ON(free_slots(ring) < TX_SLOTS_PER_FRAME)) {
		/* If we get here, we have a real error with the queue
		 * full, but queues not stopped. */
		b43err(dev->wl, "DMA queue overflow\n");
		err = -ENOSPC;
		goto out;
	}

	/* Assign the queue number to the ring (if not already done before)
	 * so TX status handling can use it. The queue to ring mapping is
	 * static, so we don't need to store it per frame. */
	ring->queue_prio = skb_get_queue_mapping(skb);

	err = dma_tx_fragment(ring, skb, true);
	if (unlikely(err == -ENOKEY)) {
		/* Drop this packet, as we don't have the encryption key
		 * anymore and must not transmit it unencrypted. */
		ieee80211_free_txskb(dev->wl->hw, skb);
		err = 0;
		goto out;
	}
	if (unlikely(err)) {
		b43err(dev->wl, "DMA tx mapping failure\n");
		goto out;
	}
	if ((free_slots(ring) < TX_SLOTS_PER_FRAME) ||
	    should_inject_overflow(ring)) {
		/* This TX ring is full. */
		unsigned int skb_mapping = skb_get_queue_mapping(skb);
		b43_stop_queue(dev, skb_mapping);
		dev->wl->tx_queue_stopped[skb_mapping] = true;
		ring->stopped = true;
		if (b43_debug(dev, B43_DBG_DMAVERBOSE)) {
			b43dbg(dev->wl, "Stopped TX ring %d\n", ring->index);
		}
	}
out:

	return err;
}

/* The cached TX header of the frame whose header is at @slot */
static struct b43_txhdr *b43_dma_txhdr(struct b43_dmaring *ring, int slot)
{
	size_t hdrsize = b43_txhdr_size(ring->dev);

	return (struct b43_txhdr *)
		&ring->txhdr_cache[(slot / TX_SLOTS_PER_FRAME) * hdrsize];
}

static void b43_dma_sync_txhdr(struct b43_dmaring *ring, int slot)
{
	struct b43_dmadesc_meta *meta;

	ring->ops->idx2desc(ring, slot, &meta);
	sync_descbuffer_for_device(ring, meta->dmaaddr,
				   b43_txhdr_size(ring->dev));
}

/*
 * Start the DMA engine on the A-MPDU queued last. If frames of it were
 * dropped, the frame queued last may not be marked last, and the length
 * in the header of the first frame, at @first_slot, must become @len.
 */
static void b43_dma_end_ampdu(struct b43_dmaring *ring, int first_slot,
			      u16 len, bool fixup)
{
	struct b43_txhdr *txhdr;
	int slot;

	if (fixup) {
		b43_ampdu_txhdr_set_len(b43_dma_txhdr(ring, first_slot), len);
		b43_dma_sync_txhdr(ring, first_slot);

		/* The header slot of the last frame */
		slot = prev_slot(ring, ring->current_slot);
		txhdr = b43_dma_txhdr(ring, slot);
		txhdr->mac_ctl = cpu_to_le32((le32_to_cpu(txhdr->mac_ctl) &
					      ~B43_TXH_MAC_AMPDU) |
					     B43_TXH_MAC_AMPDU_LAST);
		b43_dma_sync_txhdr(ring, slot);
	}

	/* Descriptors, buffers and headers must be visible before the poke */
	wmb();
	ring->ops->poke_tx(ring, next_slot(ring, ring->current_slot));
}

/* The b43 data of the receiver of @skb, NULL if it is gone */
static struct b43_sta *b43_dma_ampdu_sta(struct b43_wldev *dev,
					 struct sk_buff *skb)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;
	struct ieee80211_sta *sta;

	sta = ieee80211_find_sta_by_ifaddr(dev->wl->hw, hdr->addr1, NULL);

	return sta ? (struct b43_sta *)sta->drv_priv : NULL;
}

static u16 b43_dma_ampdu_seq(struct sk_buff *skb)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;

	return IEEE80211_SEQ_TO_SN(le16_to_cpu(hdr->seq_ctrl));
}

/*
 * The TX block ack window: the receiver flushes the MPDUs it waits for when
 * it gets one 64 or more beyond the oldest of them, so no MPDU may be sent
 * that far beyond the oldest one neither acked nor given up.
 */
static void b43_dma_baw_add(struct b43_sta *bsta, u8 tid, u16 seq)
{
	u16 idx = ieee80211_sn_sub(seq, bsta->baw_start[tid]);

	if (idx < IEEE80211_MAX_AMPDU_BUF_HT)
		bsta->baw_pending[tid] |= BIT_ULL(idx);
}

/* The MPDU @seq is done: move the window past the MPDUs that are done */
static void b43_dma_baw_complete(struct b43_sta *bsta, u8 tid, u16 seq)
{
	u16 idx = ieee80211_sn_sub(seq, bsta->baw_start[tid]);
	u16 next = ieee80211_sn_inc(bsta->ampdu_seq[tid]);

	if (idx < IEEE80211_MAX_AMPDU_BUF_HT)
		bsta->baw_pending[tid] &= ~BIT_ULL(idx);
	while (bsta->baw_start[tid] != next && !(bsta->baw_pending[tid] & 1)) {
		bsta->baw_pending[tid] >>= 1;
		bsta->baw_start[tid] = ieee80211_sn_inc(bsta->baw_start[tid]);
	}
}

/*
 * Where the MPDU in @skb is relative to the TX block ack window. The
 * receiver takes an MPDU more than 2048 behind its window for one ahead
 * of it, and moves its window there, so an MPDU behind the window must
 * never be sent. If the first MPDU of an A-MPDU is ahead of a full window
 * and nothing is on the ring anymore, no TX status will move the window,
 * so it moves to @skb.
 */
enum b43_ampdu_window b43_dma_ampdu_window(struct b43_wldev *dev,
					   struct sk_buff *skb, bool first)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;
	enum b43_ampdu_window ret = B43_AMPDU_IN_WINDOW;
	u16 seq = b43_dma_ampdu_seq(skb);
	u8 tid = ieee80211_get_tid(hdr);
	struct b43_dmaring *ring;
	struct b43_sta *bsta;

	rcu_read_lock();
	bsta = b43_dma_ampdu_sta(dev, skb);
	if (!bsta)
		goto out;
	if (ieee80211_sn_less(seq, bsta->baw_start[tid])) {
		ret = B43_AMPDU_STALE;
	} else if (ieee80211_sn_sub(seq, bsta->baw_start[tid]) >=
		   IEEE80211_MAX_AMPDU_BUF_HT) {
		ring = select_ring_by_priority(dev, skb_get_queue_mapping(skb));
		if (first && !ring->used_slots) {
			bsta->baw_start[tid] = seq;
			bsta->baw_pending[tid] = 0;
		} else {
			ret = B43_AMPDU_AHEAD;
		}
	}
out:
	rcu_read_unlock();

	return ret;
}

/*
 * Whether the block ack request in @skb moves the receiver's window to at
 * most just past the newest MPDU sent, and not further back than a window
 * behind the TX window, where the BAR for an MPDU given up after the ones
 * following it lands. A BAR anywhere else would make the receiver jump
 * its window there, and drop everything sent afterwards as out of range.
 */
bool b43_dma_ampdu_bar_valid(struct ieee80211_sta *sta, struct sk_buff *skb)
{
	const struct ieee80211_bar *bar = (const void *)skb->data;
	struct b43_sta *bsta = (struct b43_sta *)sta->drv_priv;
	u16 control = le16_to_cpu(bar->control);
	u8 tid = (control & IEEE80211_BAR_CTRL_TID_INFO_MASK) >>
		 IEEE80211_BAR_CTRL_TID_INFO_SHIFT;
	u16 ssn = IEEE80211_SEQ_TO_SN(le16_to_cpu(bar->start_seq_num));
	u16 next = ieee80211_sn_inc(bsta->ampdu_seq[tid]);

	if (control & IEEE80211_BAR_CTRL_MULTI_TID)
		return true;

	return ieee80211_sn_sub(next, ssn) <=
	       ieee80211_sn_sub(next, bsta->baw_start[tid]) +
	       IEEE80211_MAX_AMPDU_BUF_HT;
}

/* The MPDU in @skb is done without a TX status */
static void b43_dma_ampdu_dropped(struct b43_wldev *dev, struct sk_buff *skb)
{
	struct b43_sta *bsta;

	rcu_read_lock();
	bsta = b43_dma_ampdu_sta(dev, skb);
	if (bsta)
		b43_dma_baw_complete(bsta,
				     ieee80211_get_tid((struct ieee80211_hdr *)skb->data),
				     b43_dma_ampdu_seq(skb));
	rcu_read_unlock();
}

/* Whether the block ack in @status covers the MPDU in @skb */
static bool b43_dma_ampdu_frame_acked(const struct b43_txstatus *status,
				      struct sk_buff *skb)
{
	u16 idx = ieee80211_sn_sub(b43_dma_ampdu_seq(skb), status->ba_ssn);

	return status->ba_valid && idx < IEEE80211_MAX_AMPDU_BUF_HT &&
	       (status->ba_bitmap & BIT_ULL(idx));
}

/*
 * Whether the MPDU in @skb may be sent again. It must still be in the
 * block ack window, which follows the newest MPDU queued: once it falls
 * behind, the receiver takes it for an old frame, or for a new one once
 * the sequence numbers wrap.
 */
static bool b43_dma_ampdu_may_retry(struct b43_wldev *dev,
				    struct sk_buff *skb)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;
	u16 seq = b43_dma_ampdu_seq(skb);
	u8 tid = ieee80211_get_tid(hdr);
	struct b43_ampdu_retry *retry;
	struct b43_sta *bsta;

	bsta = b43_dma_ampdu_sta(dev, skb);
	if (!bsta || !dev->wl->vif)
		return false;

	if (ieee80211_sn_sub(bsta->ampdu_seq[tid], seq) >=
	    IEEE80211_MAX_AMPDU_BUF_HT)
		return false;

	retry = &bsta->ampdu_retry[tid][seq % IEEE80211_MAX_AMPDU_BUF_HT];
	if (retry->seq != seq) {
		retry->seq = seq;
		retry->count = 0;
	}
	if (retry->count >= B43_AMPDU_RETRY_LIMIT)
		return false;

	retry->count++;
	return true;
}

/*
 * Complete an MPDU of an A-MPDU, or put it on @retry to be sent again if
 * the block ack does not cover it. Must be called with the RCU read lock
 * held. Returns whether the block ack covered the MPDU.
 *
 * Rate control learns about the A-MPDU from b43_dma_ampdu_rate_status(),
 * so the status of the MPDU itself carries no A-MPDU information.
 */
static bool b43_dma_complete_ampdu_frame(struct b43_wldev *dev,
					 struct b43_dmaring *ring,
					 struct b43_dmadesc_meta *meta,
					 const struct b43_txstatus *status,
					 struct sk_buff_head *retry)
{
	struct sk_buff *skb = meta->skb;
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct b43_private_tx_info *priv_info = b43_get_priv_tx_info(info);
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)skb->data;
	bool acked = b43_dma_ampdu_frame_acked(status, skb);
	struct b43_sta *bsta;
	int i;

	unmap_descbuffer(ring, meta->dmaaddr, skb->len, 1);
	kfree(priv_info->bouncebuffer);
	priv_info->bouncebuffer = NULL;
	meta->skb = B43_DMA_PTR_POISON;

	if (!acked && b43_dma_ampdu_may_retry(dev, skb)) {
		hdr->frame_control |= cpu_to_le16(IEEE80211_FCTL_RETRY);
		/* The bounce buffer pointer took the place of the vif */
		info->control.vif = dev->wl->vif;
		/* Send it again at the rates rate control picks now */
		for (i = 0; i < ARRAY_SIZE(info->control.rates); i++) {
			info->control.rates[i].idx = -1;
			info->control.rates[i].count = 0;
		}
		info->flags &= ~IEEE80211_TX_CTL_RATE_CTRL_PROBE;
		ieee80211_get_tx_rates(info->control.vif,
				       ieee80211_find_sta_by_ifaddr(dev->wl->hw,
								    hdr->addr1,
								    NULL),
				       skb, info->control.rates,
				       ARRAY_SIZE(info->control.rates));
		__skb_queue_tail(retry, skb);
		return false;
	}

	bsta = b43_dma_ampdu_sta(dev, skb);
	if (bsta)
		b43_dma_baw_complete(bsta, ieee80211_get_tid(hdr),
				     b43_dma_ampdu_seq(skb));

	ieee80211_tx_info_clear_status(info);
	info->status.rates[0].count = 1;
	info->status.rates[1].idx = -1;
	if (acked)
		info->flags |= IEEE80211_TX_STAT_ACK;
	else
		/* Makes mac80211 send a block ack request */
		info->flags |= IEEE80211_TX_STAT_AMPDU_NO_BACK;

	ieee80211_tx_status_skb(dev->wl->hw, skb);
	return acked;
}

/*
 * Tell rate control how an A-MPDU of @len MPDUs, sent at the rate of its
 * first MPDU @first, went. This covers every transmission, including
 * those of MPDUs that are sent again, so that rate control notices a rate
 * that stopped working within a few A-MPDUs.
 */
static void b43_dma_ampdu_rate_status(struct b43_wldev *dev,
				      struct ieee80211_sta *sta,
				      const struct ieee80211_tx_info *first,
				      unsigned int len, unsigned int acked)
{
	struct ieee80211_tx_info info = {};
	struct ieee80211_tx_status status = {
		.sta = sta,
		.info = &info,
	};
	int i;

	info.band = first->band;
	info.flags = IEEE80211_TX_CTL_AMPDU | IEEE80211_TX_STAT_AMPDU |
		     (first->flags & IEEE80211_TX_CTL_RATE_CTRL_PROBE);
	if (acked)
		info.flags |= IEEE80211_TX_STAT_ACK;
	for (i = 0; i < ARRAY_SIZE(info.status.rates); i++)
		info.status.rates[i].idx = -1;
	info.status.rates[0] = first->control.rates[0];
	info.status.rates[0].count = 1;
	info.status.ampdu_len = len;
	info.status.ampdu_ack_len = acked;

	ieee80211_tx_status_ext(dev->wl->hw, &status);
}

/* Charge @sta with the airtime of an A-MPDU of @bytes sent at the rate of @first */
static void b43_dma_ampdu_airtime(struct b43_wldev *dev,
				  struct ieee80211_sta *sta,
				  const struct ieee80211_tx_info *first,
				  u8 tid, unsigned int bytes)
{
	struct ieee80211_tx_info info = {};
	int i;

	info.band = first->band;
	for (i = 0; i < ARRAY_SIZE(info.status.rates); i++)
		info.status.rates[i].idx = -1;
	info.status.rates[0] = first->control.rates[0];
	info.status.rates[0].count = 1;

	ieee80211_sta_register_airtime(sta, tid,
				       ieee80211_calc_tx_airtime(dev->wl->hw,
								 &info, bytes),
				       0);
}

/* A-MPDUs per TX ring waiting for their TX status */
#define B43_AMPDU_INFLIGHT	2

/*
 * Whether the ring for @skb has enough A-MPDUs on the way. Frames queued
 * meanwhile then go out together in the next one, instead of each in an
 * A-MPDU of its own as they arrive.
 */
bool b43_dma_ampdu_busy(struct b43_wldev *dev, struct sk_buff *skb)
{
	struct b43_dmaring *ring;

	ring = select_ring_by_priority(dev, skb_get_queue_mapping(skb));
	/* Nothing is on the way when the ring is empty */
	if (!ring->used_slots)
		ring->ampdu_inflight = 0;

	return ring->ampdu_inflight >= B43_AMPDU_INFLIGHT;
}

/*
 * Queue the MPDUs of an A-MPDU. The firmware sends frames marked first,
 * intermediate and last back to back as one PPDU, so the DMA engine is
 * only started once the last one is in the ring.
 */
int b43_dma_tx_ampdu(struct b43_wldev *dev, struct sk_buff **skbs,
		     unsigned int n, u16 ampdu_len)
{
	struct b43_ampdu_tx_ctx *ampdu = &dev->ampdu_tx;
	struct b43_dmaring *ring;
	struct b43_sta *bsta;
	unsigned int i, queued = 0, len = 0, mpdu_len;
	int first_slot = -1;
	int err;

	ring = select_ring_by_priority(dev, skb_get_queue_mapping(skbs[0]));
	if (unlikely(ring->stopped) ||
	    free_slots(ring) < n * TX_SLOTS_PER_FRAME)
		return -ENOSPC;

	ring->queue_prio = skb_get_queue_mapping(skbs[0]);

	rcu_read_lock();
	bsta = b43_dma_ampdu_sta(dev, skbs[0]);
	if (bsta) {
		u8 tid = ieee80211_get_tid((struct ieee80211_hdr *)skbs[0]->data);

		for (i = 0; i < n; i++) {
			u16 seq = b43_dma_ampdu_seq(skbs[i]);

			if (ieee80211_sn_less(bsta->ampdu_seq[tid], seq))
				bsta->ampdu_seq[tid] = seq;
			b43_dma_baw_add(bsta, tid, seq);
		}
	}
	rcu_read_unlock();

	ampdu->active = true;
	ampdu->len = ampdu_len;
	for (i = 0; i < n; i++) {
		ampdu->first = queued == 0;
		ampdu->last = i == n - 1;
		mpdu_len = b43_tx_mpdu_len(skbs[i]) + 4;

		err = dma_tx_fragment(ring, skbs[i], false);
		if (unlikely(err)) {
			if (!queued) {
				ampdu->active = false;
				/* The caller drops this frame */
				b43_dma_ampdu_dropped(dev, skbs[i]);
				return err;
			}
			/* Drop this frame, the A-MPDU goes on without it */
			b43_dma_ampdu_dropped(dev, skbs[i]);
			ieee80211_free_txskb(dev->wl->hw, skbs[i]);
			continue;
		}

		if (!queued)
			first_slot = prev_slot(ring, ring->current_slot);
		len = queued ? ALIGN(len, 4) + mpdu_len : mpdu_len;
		queued++;
	}
	ampdu->active = false;

	b43_dma_end_ampdu(ring, first_slot, len, queued < n);
	ring->ampdu_inflight++;

	if (free_slots(ring) < TX_SLOTS_PER_FRAME) {
		unsigned int skb_mapping = skb_get_queue_mapping(skbs[0]);

		b43_stop_queue(dev, skb_mapping);
		dev->wl->tx_queue_stopped[skb_mapping] = true;
		ring->stopped = true;
	}

	return 0;
}

/*
 * One TX status covers all MPDUs of an A-MPDU. Complete each of them with
 * its bit from the block ack, up to the one marked last.
 */
static void b43_dma_handle_ampdu_txstatus(struct b43_wldev *dev,
					  struct b43_dmaring *ring, int slot,
					  const struct b43_txstatus *status)
{
	const struct b43_dma_ops *ops = ring->ops;
	size_t hdrsize = b43_txhdr_size(dev);
	struct b43_dmadesc_meta *meta;
	struct sk_buff_head retry, *queue;
	struct ieee80211_tx_info first = {};
	struct ieee80211_sta *sta = NULL;
	unsigned int len = 0, acked = 0, bytes = 0;
	unsigned long flags;
	u8 tid = 0;
	bool last;

	__skb_queue_head_init(&retry);
	if (ring->ampdu_inflight)
		ring->ampdu_inflight--;
	rcu_read_lock();
	do {
		/* Header slot */
		ops->idx2desc(ring, slot, &meta);
		if (unlikely(meta->skb)) {
			b43dbg(dev->wl, "A-MPDU TX status: no header at slot %d on ring %d\n",
			       slot, ring->index);
			break;
		}
		last = (le32_to_cpu(b43_dma_txhdr(ring, slot)->mac_ctl) &
			B43_TXH_MAC_AMPDU) == B43_TXH_MAC_AMPDU_LAST;
		unmap_descbuffer(ring, meta->dmaaddr, hdrsize, 1);
		ring->used_slots--;

		/* Payload slot */
		slot = next_slot(ring, slot);
		ops->idx2desc(ring, slot, &meta);
		if (unlikely(!meta->skb || b43_dma_ptr_is_poisoned(meta->skb))) {
			b43dbg(dev->wl, "A-MPDU TX status: no frame at slot %d on ring %d\n",
			       slot, ring->index);
			break;
		}
		if (!len++) {
			struct ieee80211_hdr *hdr =
				(struct ieee80211_hdr *)meta->skb->data;

			first = *IEEE80211_SKB_CB(meta->skb);
			sta = ieee80211_find_sta_by_ifaddr(dev->wl->hw,
							   hdr->addr1, NULL);
			tid = b43_airtime_tid(hdr);
		}
		bytes += meta->skb->len;
		if (b43_dma_complete_ampdu_frame(dev, ring, meta, status,
						 &retry))
			acked++;
		ring->used_slots--;

		slot = next_slot(ring, slot);
	} while (!last && ring->used_slots);
	if (sta && first.control.rates[0].idx >= 0) {
		b43_dma_ampdu_rate_status(dev, sta, &first, len, acked);
		b43_dma_ampdu_airtime(dev, sta, &first, tid, bytes);
	}
	rcu_read_unlock();

	if (skb_queue_empty(&retry))
		return;

	/* Resend the MPDUs ahead of the frames queued since */
	queue = &dev->wl->tx_queue[skb_get_queue_mapping(retry.next)];
	spin_lock_irqsave(&queue->lock, flags);
	skb_queue_splice(&retry, queue);
	spin_unlock_irqrestore(&queue->lock, flags);
}

void b43_dma_handle_txstatus(struct b43_wldev *dev,
			     const struct b43_txstatus *status)
{
	const struct b43_dma_ops *ops;
	struct b43_dmaring *ring;
	struct b43_dmadesc_meta *meta;
	static const struct b43_txstatus fake; /* filled with 0 */
	const struct b43_txstatus *txstat;
	int slot, firstused;
	bool frame_succeed;
	int skip;
	static u8 err_out1;

	ring = parse_cookie(dev, status->cookie, &slot);
	if (unlikely(!ring))
		return;
	B43_WARN_ON(!ring->tx);

	/* Sanity check: TX packets are processed in-order on one ring.
	 * Check if the slot deduced from the cookie really is the first
	 * used slot. */
	firstused = ring->current_slot - ring->used_slots + 1;
	if (firstused < 0)
		firstused = ring->nr_slots + firstused;

	skip = 0;
	if (unlikely(slot != firstused)) {
		/* This possibly is a firmware bug and will result in
		 * malfunction, memory leaks and/or stall of DMA functionality.
		 */
		if (slot == next_slot(ring, next_slot(ring, firstused))) {
			/* If a single header/data pair was missed, skip over
			 * the first two slots in an attempt to recover.
			 */
			slot = firstused;
			skip = 2;
			if (!err_out1) {
				/* Report the error once. */
				b43dbg(dev->wl,
				       "Skip on DMA ring %d slot %d.\n",
				       ring->index, slot);
				err_out1 = 1;
			}
		} else {
			/* More than a single header/data pair were missed.
			 * Report this error. If running with open-source
			 * firmware, then reset the controller to
			 * revive operation.
			 */
			b43dbg(dev->wl,
			       "Out of order TX status report on DMA ring %d. Expected %d, but got %d\n",
			       ring->index, firstused, slot);
			if (dev->fw.opensource)
				b43_controller_restart(dev, "Out of order TX");
			return;
		}
	}

	if (!skip && b43_ampdu_tx_enabled(dev) &&
	    (le32_to_cpu(b43_dma_txhdr(ring, slot)->mac_ctl) &
	     B43_TXH_MAC_AMPDU)) {
		b43_dma_handle_ampdu_txstatus(dev, ring, slot, status);
		goto out_wake;
	}

	ops = ring->ops;
	while (1) {
		B43_WARN_ON(slot < 0 || slot >= ring->nr_slots);
		/* get meta - ignore returned value */
		ops->idx2desc(ring, slot, &meta);

		if (b43_dma_ptr_is_poisoned(meta->skb)) {
			b43dbg(dev->wl, "Poisoned TX slot %d (first=%d) "
			       "on ring %d\n",
			       slot, firstused, ring->index);
			break;
		}

		if (meta->skb) {
			struct b43_private_tx_info *priv_info =
			     b43_get_priv_tx_info(IEEE80211_SKB_CB(meta->skb));

			unmap_descbuffer(ring, meta->dmaaddr,
					 meta->skb->len, 1);
			kfree(priv_info->bouncebuffer);
			priv_info->bouncebuffer = NULL;
		} else {
			unmap_descbuffer(ring, meta->dmaaddr,
					 b43_txhdr_size(dev), 1);
		}

		if (meta->is_last_fragment) {
			struct ieee80211_tx_info *info;

			if (unlikely(!meta->skb)) {
				/* This is a scatter-gather fragment of a frame,
				 * so the skb pointer must not be NULL.
				 */
				b43dbg(dev->wl, "TX status unexpected NULL skb "
				       "at slot %d (first=%d) on ring %d\n",
				       slot, firstused, ring->index);
				break;
			}

			info = IEEE80211_SKB_CB(meta->skb);

			/*
			 * Call back to inform the ieee80211 subsystem about
			 * the status of the transmission. When skipping over
			 * a missed TX status report, use a status structure
			 * filled with zeros to indicate that the frame was not
			 * sent (frame_count 0) and not acknowledged
			 */
			if (unlikely(skip))
				txstat = &fake;
			else
				txstat = status;

			frame_succeed = b43_fill_txstatus_report(dev, info,
								 txstat);
			b43_tx_airtime(dev, meta->skb);
#ifdef CONFIG_B43_DEBUG
			if (frame_succeed)
				ring->nr_succeed_tx_packets++;
			else
				ring->nr_failed_tx_packets++;
			ring->nr_total_packet_tries += status->frame_count;
#endif /* DEBUG */
			ieee80211_tx_status_skb(dev->wl->hw, meta->skb);

			/* skb will be freed by ieee80211_tx_status_skb().
			 * Poison our pointer. */
			meta->skb = B43_DMA_PTR_POISON;
		} else {
			/* No need to call free_descriptor_buffer here, as
			 * this is only the txhdr, which is not allocated.
			 */
			if (unlikely(meta->skb)) {
				b43dbg(dev->wl, "TX status unexpected non-NULL skb "
				       "at slot %d (first=%d) on ring %d\n",
				       slot, firstused, ring->index);
				break;
			}
		}

		/* Everything unmapped and free'd. So it's not used anymore. */
		ring->used_slots--;

		if (meta->is_last_fragment && !skip) {
			/* This is the last scatter-gather
			 * fragment of the frame. We are done. */
			break;
		}
		slot = next_slot(ring, slot);
		if (skip > 0)
			--skip;
	}
out_wake:
	if (ring->stopped) {
		B43_WARN_ON(free_slots(ring) < TX_SLOTS_PER_FRAME);
		ring->stopped = false;
	}

	if (dev->wl->tx_queue_stopped[ring->queue_prio]) {
		dev->wl->tx_queue_stopped[ring->queue_prio] = false;
	} else if (!dev->wl->tx_queue_full[ring->queue_prio]) {
		/*
		 * If the driver queue is running and has room, wake the
		 * corresponding mac80211 queue.
		 */
		b43_wake_queue(dev, ring->queue_prio);
		if (b43_debug(dev, B43_DBG_DMAVERBOSE)) {
			b43dbg(dev->wl, "Woke up TX ring %d\n", ring->index);
		}
	}
	/* Have the TX status handler send more */
	b43_txq_pull_later(dev, ring->queue_prio);
}

static void dma_rx(struct b43_dmaring *ring, int *slot)
{
	const struct b43_dma_ops *ops = ring->ops;
	struct b43_dmadesc_generic *desc;
	struct b43_dmadesc_meta *meta;
	struct b43_rxhdr_fw4 *rxhdr;
	struct sk_buff *skb;
	u16 len;
	int err;
	dma_addr_t dmaaddr;

	desc = ops->idx2desc(ring, *slot, &meta);

	sync_descbuffer_for_cpu(ring, meta->dmaaddr, ring->rx_buffersize);
	skb = meta->skb;

	rxhdr = (struct b43_rxhdr_fw4 *)skb->data;
	len = le16_to_cpu(rxhdr->frame_len);
	if (len == 0) {
		int i = 0;

		do {
			udelay(2);
			barrier();
			len = le16_to_cpu(rxhdr->frame_len);
		} while (len == 0 && i++ < 5);
		if (unlikely(len == 0)) {
			dmaaddr = meta->dmaaddr;
			goto drop_recycle_buffer;
		}
	}
	if (unlikely(b43_rx_buffer_is_poisoned(ring, skb))) {
		/* Something went wrong with the DMA.
		 * The device did not touch the buffer and did not overwrite the poison. */
		b43dbg(ring->dev->wl, "DMA RX: Dropping poisoned buffer.\n");
		dmaaddr = meta->dmaaddr;
		goto drop_recycle_buffer;
	}
	if (unlikely(len + ring->frameoffset > ring->rx_buffersize)) {
		/* The data did not fit into one descriptor buffer
		 * and is split over multiple buffers.
		 * This should never happen, as we try to allocate buffers
		 * big enough. So simply ignore this packet.
		 */
		int cnt = 0;
		s32 tmp = len;

		while (1) {
			desc = ops->idx2desc(ring, *slot, &meta);
			/* recycle the descriptor buffer. */
			b43_poison_rx_buffer(ring, meta->skb);
			sync_descbuffer_for_device(ring, meta->dmaaddr,
						   ring->rx_buffersize);
			*slot = next_slot(ring, *slot);
			cnt++;
			tmp -= ring->rx_buffersize;
			if (tmp <= 0)
				break;
		}
		b43err(ring->dev->wl, "DMA RX buffer too small "
		       "(len: %u, buffer: %u, nr-dropped: %d)\n",
		       len, ring->rx_buffersize, cnt);
		goto drop;
	}

	dmaaddr = meta->dmaaddr;
	err = setup_rx_descbuffer(ring, desc, meta, GFP_ATOMIC);
	if (unlikely(err)) {
		b43dbg(ring->dev->wl, "DMA RX: setup_rx_descbuffer() failed\n");
		goto drop_recycle_buffer;
	}

	skb_put(skb, len + ring->frameoffset);
	skb_pull(skb, ring->frameoffset);

	b43_rx(ring->dev, skb, rxhdr);
drop:
	return;

drop_recycle_buffer:
	/* Poison and recycle the RX buffer. */
	b43_poison_rx_buffer(ring, skb);
	sync_descbuffer_for_device(ring, dmaaddr, ring->rx_buffersize);
}

void b43_dma_handle_rx_overflow(struct b43_dmaring *ring)
{
	int current_slot, previous_slot;

	B43_WARN_ON(ring->tx);

	/* Device has filled all buffers, drop all packets and let TCP
	 * decrease speed.
	 * Decrement RX index by one will let the device to see all slots
	 * as free again
	 */
	/*
	*TODO: How to increase rx_drop in mac80211?
	*/
	current_slot = ring->ops->get_current_rxslot(ring);
	previous_slot = prev_slot(ring, current_slot);
	ring->ops->set_current_rxslot(ring, previous_slot);
}

void b43_dma_rx(struct b43_dmaring *ring)
{
	const struct b43_dma_ops *ops = ring->ops;
	int slot, current_slot;
	int used_slots = 0;

	B43_WARN_ON(ring->tx);
	current_slot = ops->get_current_rxslot(ring);
	B43_WARN_ON(!(current_slot >= 0 && current_slot < ring->nr_slots));

	slot = ring->current_slot;
	for (; slot != current_slot; slot = next_slot(ring, slot)) {
		dma_rx(ring, &slot);
		update_max_used_slots(ring, ++used_slots);
	}
	wmb();
	ops->set_current_rxslot(ring, slot);
	ring->current_slot = slot;
}

static void b43_dma_tx_suspend_ring(struct b43_dmaring *ring)
{
	B43_WARN_ON(!ring->tx);
	ring->ops->tx_suspend(ring);
}

static void b43_dma_tx_resume_ring(struct b43_dmaring *ring)
{
	B43_WARN_ON(!ring->tx);
	ring->ops->tx_resume(ring);
}

void b43_dma_tx_suspend(struct b43_wldev *dev)
{
	b43_power_saving_ctl_bits(dev, B43_PS_AWAKE);
	b43_dma_tx_suspend_ring(dev->dma.tx_ring_AC_BK);
	b43_dma_tx_suspend_ring(dev->dma.tx_ring_AC_BE);
	b43_dma_tx_suspend_ring(dev->dma.tx_ring_AC_VI);
	b43_dma_tx_suspend_ring(dev->dma.tx_ring_AC_VO);
	b43_dma_tx_suspend_ring(dev->dma.tx_ring_mcast);
}

void b43_dma_tx_resume(struct b43_wldev *dev)
{
	b43_dma_tx_resume_ring(dev->dma.tx_ring_mcast);
	b43_dma_tx_resume_ring(dev->dma.tx_ring_AC_VO);
	b43_dma_tx_resume_ring(dev->dma.tx_ring_AC_VI);
	b43_dma_tx_resume_ring(dev->dma.tx_ring_AC_BE);
	b43_dma_tx_resume_ring(dev->dma.tx_ring_AC_BK);
	b43_power_saving_ctl_bits(dev, 0);
}

static void direct_fifo_rx(struct b43_wldev *dev, enum b43_dmatype type,
			   u16 mmio_base, bool enable)
{
	u32 ctl;

	if (type == B43_DMA_64BIT) {
		ctl = b43_read32(dev, mmio_base + B43_DMA64_RXCTL);
		ctl &= ~B43_DMA64_RXDIRECTFIFO;
		if (enable)
			ctl |= B43_DMA64_RXDIRECTFIFO;
		b43_write32(dev, mmio_base + B43_DMA64_RXCTL, ctl);
	} else {
		ctl = b43_read32(dev, mmio_base + B43_DMA32_RXCTL);
		ctl &= ~B43_DMA32_RXDIRECTFIFO;
		if (enable)
			ctl |= B43_DMA32_RXDIRECTFIFO;
		b43_write32(dev, mmio_base + B43_DMA32_RXCTL, ctl);
	}
}

/* Enable/Disable Direct FIFO Receive Mode (PIO) on a RX engine.
 * This is called from PIO code, so DMA structures are not available. */
void b43_dma_direct_fifo_rx(struct b43_wldev *dev,
			    unsigned int engine_index, bool enable)
{
	enum b43_dmatype type;
	u16 mmio_base;

	type = b43_engine_type(dev);

	mmio_base = b43_dmacontroller_base(type, engine_index);
	direct_fifo_rx(dev, type, mmio_base, enable);
}
