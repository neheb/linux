// SPDX-License-Identifier: GPL-2.0
/*
 * Exceptions for specific devices. Usually work-arounds for fatal design flaws.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/acpi.h>
#include <linux/dmi.h>
#include <linux/isa-dma.h> /* isa_dma_bridge_buggy */
#include <linux/pci.h>
#include <linux/platform_data/x86/apple.h>
#include <linux/suspend.h>
#include <linux/vgaarb.h>
#include <asm/amd/node.h>
#include <asm/hpet.h>
#include <asm/pci_x86.h>

/* Capability search loop limit (same value as drivers/pci/pci.h) */
#define PCI_FIND_CAP_TTL	48

void quirk_disable_msi(struct pci_dev *dev);
const char *pci_resource_name(struct pci_dev *dev, unsigned int i);

static void quirk_io(struct pci_dev *dev, int pos, unsigned int size,
		     const char *name)
{
	u32 region;
	struct pci_bus_region bus_region;
	struct resource *res = pci_resource_n(dev, pos);
	const char *res_name = pci_resource_name(dev, pos);

	pci_read_config_dword(dev, PCI_BASE_ADDRESS_0 + (pos << 2), &region);

	if (!region)
		return;

	res->name = pci_name(dev);
	res->flags = region & ~PCI_BASE_ADDRESS_IO_MASK;
	res->flags |=
		(IORESOURCE_IO | IORESOURCE_PCI_FIXED | IORESOURCE_SIZEALIGN);
	region &= ~(size - 1);

	/* Convert from PCI bus to resource space */
	bus_region.start = region;
	bus_region.end = region + size - 1;
	pcibios_bus_to_resource(dev->bus, res, &bus_region);

	pci_info(dev, FW_BUG "%s %pR: %s quirk\n", res_name, res, name);
}

static void quirk_io_region(struct pci_dev *dev, int port,
			    unsigned int size, int nr, const char *name)
{
	u16 region;
	struct pci_bus_region bus_region;
	struct resource *res = pci_resource_n(dev, nr);

	pci_read_config_word(dev, port, &region);
	region &= ~(size - 1);

	if (!region)
		return;

	res->name = pci_name(dev);
	res->flags = IORESOURCE_IO;

	/* Convert from PCI bus to resource space */
	bus_region.start = region;
	bus_region.end = region + size - 1;
	pcibios_bus_to_resource(dev->bus, res, &bus_region);

	/*
	 * "res" is typically a bridge window resource that's not being
	 * used for a bridge window, so it's just a place to stash this
	 * non-standard resource.  Printing "nr" or pci_resource_name() of
	 * it doesn't really make sense.
	 */
	if (!pci_claim_resource(dev, nr))
		pci_info(dev, "quirk: %pR claimed by %s\n", res, name);
}

static void piix4_io_quirk(struct pci_dev *dev, const char *name, unsigned int port, unsigned int enable)
{
	u32 devres;
	u32 mask, size, base;

	pci_read_config_dword(dev, port, &devres);
	if ((devres & enable) != enable)
		return;
	mask = (devres >> 16) & 15;
	base = devres & 0xffff;
	size = 16;
	for (;;) {
		unsigned int bit = size >> 1;
		if ((bit & mask) == bit)
			break;
		size = bit;
	}
	/*
	 * For now we only print it out. Eventually we'll want to
	 * reserve it (at least if it's in the 0x1000+ range), but
	 * let's get enough confirmation reports first.
	 */
	base &= -size;
	pci_info(dev, "%s PIO at %04x-%04x\n", name, base, base + size - 1);
}

static void piix4_mem_quirk(struct pci_dev *dev, const char *name, unsigned int port, unsigned int enable)
{
	u32 devres;
	u32 mask, size, base;

	pci_read_config_dword(dev, port, &devres);
	if ((devres & enable) != enable)
		return;
	base = devres & 0xffff0000;
	mask = (devres & 0x3f) << 16;
	size = 128 << 16;
	for (;;) {
		unsigned int bit = size >> 1;
		if ((bit & mask) == bit)
			break;
		size = bit;
	}

	/*
	 * For now we only print it out. Eventually we'll want to
	 * reserve it, but let's get enough confirmation reports first.
	 */
	base &= -size;
	pci_info(dev, "%s MMIO at %04x-%04x\n", name, base, base + size - 1);
}

static void pci_fixup_i450nx(struct pci_dev *d)
{
	/*
	 * i450NX -- Find and scan all secondary buses on all PXB's.
	 */
	int pxb, reg;
	u8 busno, suba, subb;

	dev_warn(&d->dev, "Searching for i450NX host bridges\n");
	reg = 0xd0;
	for(pxb = 0; pxb < 2; pxb++) {
		pci_read_config_byte(d, reg++, &busno);
		pci_read_config_byte(d, reg++, &suba);
		pci_read_config_byte(d, reg++, &subb);
		dev_dbg(&d->dev, "i450NX PXB %d: %02x/%02x/%02x\n", pxb, busno,
			suba, subb);
		if (busno)
			pcibios_scan_root(busno);	/* Bus A */
		if (suba < subb)
			pcibios_scan_root(suba+1);	/* Bus B */
	}
	pcibios_last_bus = -1;
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_82451NX, pci_fixup_i450nx);

static void pci_fixup_i450gx(struct pci_dev *d)
{
	/*
	 * i450GX and i450KX -- Find and scan all secondary buses.
	 * (called separately for each PCI bridge found)
	 */
	u8 busno;
	pci_read_config_byte(d, 0x4a, &busno);
	dev_info(&d->dev, "i440KX/GX host bridge; secondary bus %02x\n", busno);
	pcibios_scan_root(busno);
	pcibios_last_bus = -1;
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_82454GX, pci_fixup_i450gx);

static void pci_fixup_umc_ide(struct pci_dev *d)
{
	/*
	 * UM8886BF IDE controller sets region type bits incorrectly,
	 * therefore they look like memory despite of them being I/O.
	 */
	int i;

	dev_warn(&d->dev, "Fixing base address flags\n");
	for(i = 0; i < 4; i++)
		d->resource[i].flags |= PCI_BASE_ADDRESS_SPACE_IO;
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_UMC, PCI_DEVICE_ID_UMC_UM8886BF, pci_fixup_umc_ide);

static void pci_fixup_latency(struct pci_dev *d)
{
	/*
	 *  SiS 5597 and 5598 chipsets require latency timer set to
	 *  at most 32 to avoid lockups.
	 */
	dev_dbg(&d->dev, "Setting max latency to 32\n");
	pcibios_max_latency = 32;
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_SI, PCI_DEVICE_ID_SI_5597, pci_fixup_latency);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_SI, PCI_DEVICE_ID_SI_5598, pci_fixup_latency);

static void pci_fixup_piix4_acpi(struct pci_dev *d)
{
	/*
	 * PIIX4 ACPI device: hardwired IRQ9
	 */
	d->irq = 9;
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_82371AB_3, pci_fixup_piix4_acpi);

/*
 * Addresses issues with problems in the memory write queue timer in
 * certain VIA Northbridges.  This bugfix is per VIA's specifications,
 * except for the KL133/KM133: clearing bit 5 on those Northbridges seems
 * to trigger a bug in its integrated ProSavage video card, which
 * causes screen corruption.  We only clear bits 6 and 7 for that chipset,
 * until VIA can provide us with definitive information on why screen
 * corruption occurs, and what exactly those bits do.
 *
 * VIA 8363,8622,8361 Northbridges:
 *  - bits  5, 6, 7 at offset 0x55 need to be turned off
 * VIA 8367 (KT266x) Northbridges:
 *  - bits  5, 6, 7 at offset 0x95 need to be turned off
 * VIA 8363 rev 0x81/0x84 (KL133/KM133) Northbridges:
 *  - bits     6, 7 at offset 0x55 need to be turned off
 */

#define VIA_8363_KL133_REVISION_ID 0x81
#define VIA_8363_KM133_REVISION_ID 0x84

static void pci_fixup_via_northbridge_bug(struct pci_dev *d)
{
	u8 v;
	int where = 0x55;
	int mask = 0x1f; /* clear bits 5, 6, 7 by default */

	if (d->device == PCI_DEVICE_ID_VIA_8367_0) {
		/* fix pci bus latency issues resulted by NB bios error
		   it appears on bug free^Wreduced kt266x's bios forces
		   NB latency to zero */
		pci_write_config_byte(d, PCI_LATENCY_TIMER, 0);

		where = 0x95; /* the memory write queue timer register is
				different for the KT266x's: 0x95 not 0x55 */
	} else if (d->device == PCI_DEVICE_ID_VIA_8363_0 &&
			(d->revision == VIA_8363_KL133_REVISION_ID ||
			d->revision == VIA_8363_KM133_REVISION_ID)) {
			mask = 0x3f; /* clear only bits 6 and 7; clearing bit 5
					causes screen corruption on the KL133/KM133 */
	}

	pci_read_config_byte(d, where, &v);
	if (v & ~mask) {
		dev_warn(&d->dev, "Disabling VIA memory write queue (PCI ID %04x, rev %02x): [%02x] %02x & %02x -> %02x\n", \
			d->device, d->revision, where, v, mask, v & mask);
		v &= mask;
		pci_write_config_byte(d, where, v);
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_8363_0, pci_fixup_via_northbridge_bug);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_8622, pci_fixup_via_northbridge_bug);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_8361, pci_fixup_via_northbridge_bug);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_8367_0, pci_fixup_via_northbridge_bug);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_8363_0, pci_fixup_via_northbridge_bug);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_8622, pci_fixup_via_northbridge_bug);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_8361, pci_fixup_via_northbridge_bug);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_8367_0, pci_fixup_via_northbridge_bug);

/*
 * For some reasons Intel decided that certain parts of their
 * 815, 845 and some other chipsets must look like PCI-to-PCI bridges
 * while they are obviously not. The 82801 family (AA, AB, BAM/CAM,
 * BA/CA/DB and E) PCI bridges are actually HUB-to-PCI ones, according
 * to Intel terminology. These devices do forward all addresses from
 * system to PCI bus no matter what are their window settings, so they are
 * "transparent" (or subtractive decoding) from programmers point of view.
 */
static void pci_fixup_transparent_bridge(struct pci_dev *dev)
{
	if ((dev->device & 0xff00) == 0x2400)
		dev->transparent = 1;
}
DECLARE_PCI_FIXUP_CLASS_HEADER(PCI_VENDOR_ID_INTEL, PCI_ANY_ID,
			 PCI_CLASS_BRIDGE_PCI, 8, pci_fixup_transparent_bridge);

/*
 * Fixup for C1 Halt Disconnect problem on nForce2 systems.
 *
 * From information provided by "Allen Martin" <AMartin@nvidia.com>:
 *
 * A hang is caused when the CPU generates a very fast CONNECT/HALT cycle
 * sequence.  Workaround is to set the SYSTEM_IDLE_TIMEOUT to 80 ns.
 * This allows the state-machine and timer to return to a proper state within
 * 80 ns of the CONNECT and probe appearing together.  Since the CPU will not
 * issue another HALT within 80 ns of the initial HALT, the failure condition
 * is avoided.
 */
static void pci_fixup_nforce2(struct pci_dev *dev)
{
	u32 val;

	/*
	 * Chip  Old value   New value
	 * C17   0x1F0FFF01  0x1F01FF01
	 * C18D  0x9F0FFF01  0x9F01FF01
	 *
	 * Northbridge chip version may be determined by
	 * reading the PCI revision ID (0xC1 or greater is C18D).
	 */
	pci_read_config_dword(dev, 0x6c, &val);

	/*
	 * Apply fixup if needed, but don't touch disconnect state
	 */
	if ((val & 0x00FF0000) != 0x00010000) {
		dev_warn(&dev->dev, "nForce2 C1 Halt Disconnect fixup\n");
		pci_write_config_dword(dev, 0x6c, (val & 0xFF00FFFF) | 0x00010000);
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_NVIDIA, PCI_DEVICE_ID_NVIDIA_NFORCE2, pci_fixup_nforce2);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_NVIDIA, PCI_DEVICE_ID_NVIDIA_NFORCE2, pci_fixup_nforce2);

/* Max PCI Express root ports */
#define MAX_PCIEROOT	6
static int quirk_aspm_offset[MAX_PCIEROOT << 3];

#define GET_INDEX(a, b) ((((a) - PCI_DEVICE_ID_INTEL_MCH_PA) << 3) + ((b) & 7))

static int quirk_pcie_aspm_read(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 *value)
{
	return raw_pci_read(pci_domain_nr(bus), bus->number,
						devfn, where, size, value);
}

/*
 * Replace the original pci bus ops for write with a new one that will filter
 * the request to insure ASPM cannot be enabled.
 */
static int quirk_pcie_aspm_write(struct pci_bus *bus, unsigned int devfn, int where, int size, u32 value)
{
	u8 offset;

	offset = quirk_aspm_offset[GET_INDEX(bus->self->device, devfn)];

	if ((offset) && (where == offset))
		value = value & ~PCI_EXP_LNKCTL_ASPMC;

	return raw_pci_write(pci_domain_nr(bus), bus->number,
						devfn, where, size, value);
}

static struct pci_ops quirk_pcie_aspm_ops = {
	.read = quirk_pcie_aspm_read,
	.write = quirk_pcie_aspm_write,
};

/*
 * Prevents PCI Express ASPM (Active State Power Management) being enabled.
 *
 * Save the register offset, where the ASPM control bits are located,
 * for each PCI Express device that is in the device list of
 * the root port in an array for fast indexing. Replace the bus ops
 * with the modified one.
 */
static void pcie_rootport_aspm_quirk(struct pci_dev *pdev)
{
	int i;
	struct pci_bus  *pbus;
	struct pci_dev *dev;

	if ((pbus = pdev->subordinate) == NULL)
		return;

	/*
	 * Check if the DID of pdev matches one of the six root ports. This
	 * check is needed in the case this function is called directly by the
	 * hot-plug driver.
	 */
	if ((pdev->device < PCI_DEVICE_ID_INTEL_MCH_PA) ||
	    (pdev->device > PCI_DEVICE_ID_INTEL_MCH_PC1))
		return;

	if (list_empty(&pbus->devices)) {
		/*
		 * If no device is attached to the root port at power-up or
		 * after hot-remove, the pbus->devices is empty and this code
		 * will set the offsets to zero and the bus ops to parent's bus
		 * ops, which is unmodified.
		 */
		for (i = GET_INDEX(pdev->device, 0); i <= GET_INDEX(pdev->device, 7); ++i)
			quirk_aspm_offset[i] = 0;

		pci_bus_set_ops(pbus, pbus->parent->ops);
	} else {
		/*
		 * If devices are attached to the root port at power-up or
		 * after hot-add, the code loops through the device list of
		 * each root port to save the register offsets and replace the
		 * bus ops.
		 */
		list_for_each_entry(dev, &pbus->devices, bus_list)
			/* There are 0 to 8 devices attached to this bus */
			quirk_aspm_offset[GET_INDEX(pdev->device, dev->devfn)] =
				dev->pcie_cap + PCI_EXP_LNKCTL;

		pci_bus_set_ops(pbus, &quirk_pcie_aspm_ops);
		dev_info(&pbus->dev, "writes to ASPM control bits will be ignored\n");
	}

}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_MCH_PA,	pcie_rootport_aspm_quirk);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_MCH_PA1,	pcie_rootport_aspm_quirk);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_MCH_PB,	pcie_rootport_aspm_quirk);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_MCH_PB1,	pcie_rootport_aspm_quirk);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_MCH_PC,	pcie_rootport_aspm_quirk);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_MCH_PC1,	pcie_rootport_aspm_quirk);

/*
 * PCIe devices underneath Xeon 6 PCIe Root Port bifurcated to x2 have lower
 * performance with Extended Tags and MRRS > 128B. Work around the performance
 * problems by disabling Extended Tags and limiting MRRS to 128B.
 *
 * https://cdrdv2.intel.com/v1/dl/getContent/837176
 */
static int limit_mrrs_to_128(struct pci_host_bridge *b, struct pci_dev *pdev)
{
	int readrq = pcie_get_readrq(pdev);

	if (readrq > 128)
		pcie_set_readrq(pdev, 128);

	return 0;
}

static void pci_xeon_x2_bifurc_quirk(struct pci_dev *pdev)
{
	struct pci_host_bridge *bridge = pci_find_host_bridge(pdev->bus);
	u32 linkcap;

	pcie_capability_read_dword(pdev, PCI_EXP_LNKCAP, &linkcap);
	if (FIELD_GET(PCI_EXP_LNKCAP_MLW, linkcap) != 0x2)
		return;

	bridge->no_ext_tags = 1;
	bridge->enable_device = limit_mrrs_to_128;
	pci_info(pdev, "Disabling Extended Tags and limiting MRRS to 128B (performance reasons due to x2 PCIe link)\n");
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0x0db0, pci_xeon_x2_bifurc_quirk);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0x0db1, pci_xeon_x2_bifurc_quirk);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0x0db2, pci_xeon_x2_bifurc_quirk);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0x0db3, pci_xeon_x2_bifurc_quirk);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0x0db6, pci_xeon_x2_bifurc_quirk);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0x0db7, pci_xeon_x2_bifurc_quirk);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0x0db8, pci_xeon_x2_bifurc_quirk);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0x0db9, pci_xeon_x2_bifurc_quirk);

/*
 * Fixup to mark boot BIOS video selected by BIOS before it changes
 *
 * From information provided by "Jon Smirl" <jonsmirl@gmail.com>
 *
 * The standard boot ROM sequence for an x86 machine uses the BIOS
 * to select an initial video card for boot display. This boot video
 * card will have its BIOS copied to 0xC0000 in system RAM.
 * IORESOURCE_ROM_SHADOW is used to associate the boot video
 * card with this copy. On laptops this copy has to be used since
 * the main ROM may be compressed or combined with another image.
 * See pci_map_rom() for use of this flag. Before marking the device
 * with IORESOURCE_ROM_SHADOW check if a vga_default_device is already set
 * by either arch code or vga-arbitration; if so only apply the fixup to this
 * already-determined primary video card.
 */

static void pci_fixup_video(struct pci_dev *pdev)
{
	struct pci_dev *bridge;
	struct pci_bus *bus;
	u16 config;
	struct resource *res;

	/* Is VGA routed to us? */
	bus = pdev->bus;
	while (bus) {
		bridge = bus->self;

		/*
		 * From information provided by
		 * "David Miller" <davem@davemloft.net>
		 * The bridge control register is valid for PCI header
		 * type BRIDGE, or CARDBUS. Host to PCI controllers use
		 * PCI header type NORMAL.
		 */
		if (bridge && (pci_is_bridge(bridge))) {
			pci_read_config_word(bridge, PCI_BRIDGE_CONTROL,
						&config);
			if (!(config & PCI_BRIDGE_CTL_VGA))
				return;
		}
		bus = bus->parent;
	}
	if (!vga_default_device() || pdev == vga_default_device()) {
		pci_read_config_word(pdev, PCI_COMMAND, &config);
		if (config & (PCI_COMMAND_IO | PCI_COMMAND_MEMORY)) {
			res = &pdev->resource[PCI_ROM_RESOURCE];

			pci_disable_rom(pdev);
			if (res->parent)
				release_resource(res);

			res->start = 0xC0000;
			res->end = res->start + 0x20000 - 1;
			res->flags = IORESOURCE_MEM | IORESOURCE_ROM_SHADOW |
				     IORESOURCE_PCI_FIXED;
			dev_info(&pdev->dev, "Video device with shadowed ROM at %pR\n",
				 res);
		}
	}
}
DECLARE_PCI_FIXUP_CLASS_HEADER(PCI_ANY_ID, PCI_ANY_ID,
			       PCI_CLASS_DISPLAY_VGA, 8, pci_fixup_video);


static const struct dmi_system_id msi_k8t_dmi_table[] = {
	{
		.ident = "MSI-K8T-Neo2Fir",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "MSI"),
			DMI_MATCH(DMI_PRODUCT_NAME, "MS-6702E"),
		},
	},
	{}
};

/*
 * The AMD-Athlon64 board MSI "K8T Neo2-FIR" disables the onboard sound
 * card if a PCI-soundcard is added.
 *
 * The BIOS only gives options "DISABLED" and "AUTO". This code sets
 * the corresponding register-value to enable the soundcard.
 *
 * The soundcard is only enabled, if the mainboard is identified
 * via DMI-tables and the soundcard is detected to be off.
 */
static void pci_fixup_msi_k8t_onboard_sound(struct pci_dev *dev)
{
	unsigned char val;
	if (!dmi_check_system(msi_k8t_dmi_table))
		return; /* only applies to MSI K8T Neo2-FIR */

	pci_read_config_byte(dev, 0x50, &val);
	if (val & 0x40) {
		pci_write_config_byte(dev, 0x50, val & (~0x40));

		/* verify the change for status output */
		pci_read_config_byte(dev, 0x50, &val);
		if (val & 0x40)
			dev_info(&dev->dev, "Detected MSI K8T Neo2-FIR; "
					"can't enable onboard soundcard!\n");
		else
			dev_info(&dev->dev, "Detected MSI K8T Neo2-FIR; "
					"enabled onboard soundcard\n");
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_8237,
		pci_fixup_msi_k8t_onboard_sound);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_8237,
		pci_fixup_msi_k8t_onboard_sound);

/*
 * Some Toshiba laptops need extra code to enable their TI TSB43AB22/A.
 *
 * We pretend to bring them out of full D3 state, and restore the proper
 * IRQ, PCI cache line size, and BARs, otherwise the device won't function
 * properly.  In some cases, the device will generate an interrupt on
 * the wrong IRQ line, causing any devices sharing the line it's
 * *supposed* to use to be disabled by the kernel's IRQ debug code.
 */
static u16 toshiba_line_size;

static const struct dmi_system_id toshiba_ohci1394_dmi_table[] = {
	{
		.ident = "Toshiba PS5 based laptop",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "TOSHIBA"),
			DMI_MATCH(DMI_PRODUCT_VERSION, "PS5"),
		},
	},
	{
		.ident = "Toshiba PSM4 based laptop",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "TOSHIBA"),
			DMI_MATCH(DMI_PRODUCT_VERSION, "PSM4"),
		},
	},
	{
		.ident = "Toshiba A40 based laptop",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "TOSHIBA"),
			DMI_MATCH(DMI_PRODUCT_VERSION, "PSA40U"),
		},
	},
	{ }
};

static void pci_pre_fixup_toshiba_ohci1394(struct pci_dev *dev)
{
	if (!dmi_check_system(toshiba_ohci1394_dmi_table))
		return; /* only applies to certain Toshibas (so far) */

	dev->current_state = PCI_D3cold;
	pci_read_config_word(dev, PCI_CACHE_LINE_SIZE, &toshiba_line_size);
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_TI, 0x8032,
			 pci_pre_fixup_toshiba_ohci1394);

static void pci_post_fixup_toshiba_ohci1394(struct pci_dev *dev)
{
	if (!dmi_check_system(toshiba_ohci1394_dmi_table))
		return; /* only applies to certain Toshibas (so far) */

	/* Restore config space on Toshiba laptops */
	pci_write_config_word(dev, PCI_CACHE_LINE_SIZE, toshiba_line_size);
	pci_read_config_byte(dev, PCI_INTERRUPT_LINE, (u8 *)&dev->irq);
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_0,
			       pci_resource_start(dev, 0));
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_1,
			       pci_resource_start(dev, 1));
}
DECLARE_PCI_FIXUP_ENABLE(PCI_VENDOR_ID_TI, 0x8032,
			 pci_post_fixup_toshiba_ohci1394);


/*
 * Prevent the BIOS trapping accesses to the Cyrix CS5530A video device
 * configuration space.
 */
static void pci_early_fixup_cyrix_5530(struct pci_dev *dev)
{
	u8 r;
	/* clear 'F4 Video Configuration Trap' bit */
	pci_read_config_byte(dev, 0x42, &r);
	r &= 0xfd;
	pci_write_config_byte(dev, 0x42, r);
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_CYRIX, PCI_DEVICE_ID_CYRIX_5530_LEGACY,
			pci_early_fixup_cyrix_5530);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_CYRIX, PCI_DEVICE_ID_CYRIX_5530_LEGACY,
			pci_early_fixup_cyrix_5530);

/*
 * Siemens Nixdorf AG FSC Multiprocessor Interrupt Controller:
 * prevent update of the BAR0, which doesn't look like a normal BAR.
 */
static void pci_siemens_interrupt_controller(struct pci_dev *dev)
{
	dev->resource[0].flags |= IORESOURCE_PCI_FIXED;
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_SIEMENS, 0x0015,
			  pci_siemens_interrupt_controller);

/*
 * SB600: Disable BAR1 on device 14.0 to avoid HPET resources from
 * confusing the PCI engine:
 */
static void sb600_disable_hpet_bar(struct pci_dev *dev)
{
	u8 val;

	/*
	 * The SB600 and SB700 both share the same device
	 * ID, but the PM register 0x55 does something different
	 * for the SB700, so make sure we are dealing with the
	 * SB600 before touching the bit:
	 */

	pci_read_config_byte(dev, 0x08, &val);

	if (val < 0x2F) {
		outb(0x55, 0xCD6);
		val = inb(0xCD7);

		/* Set bit 7 in PM register 0x55 */
		outb(0x55, 0xCD6);
		outb(val | 0x80, 0xCD7);
	}
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_ATI, 0x4385, sb600_disable_hpet_bar);

#ifdef CONFIG_HPET_TIMER
static void sb600_hpet_quirk(struct pci_dev *dev)
{
	struct resource *r = &dev->resource[1];

	if (r->flags & IORESOURCE_MEM && r->start == hpet_address) {
		r->flags |= IORESOURCE_PCI_FIXED;
		dev_info(&dev->dev, "reg 0x14 contains HPET; making it immovable\n");
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_ATI, 0x4385, sb600_hpet_quirk);
#endif

/*
 * Twinhead H12Y needs us to block out a region otherwise we map devices
 * there and any access kills the box.
 *
 *   See: https://bugzilla.kernel.org/show_bug.cgi?id=10231
 *
 * Match off the LPC and svid/sdid (older kernels lose the bridge subvendor)
 */
static void twinhead_reserve_killing_zone(struct pci_dev *dev)
{
        if (dev->subsystem_vendor == 0x14FF && dev->subsystem_device == 0xA003) {
                pr_info("Reserving memory on Twinhead H12Y\n");
                request_mem_region(0xFFB00000, 0x100000, "twinhead");
        }
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x27B9, twinhead_reserve_killing_zone);

/*
 * Device [8086:2fc0]
 * Erratum HSE43
 * CONFIG_TDP_NOMINAL CSR Implemented at Incorrect Offset
 * https://www.intel.com/content/www/us/en/processors/xeon/xeon-e5-v3-spec-update.html
 *
 * Devices [8086:6f60,6fa0,6fc0]
 * Erratum BDF2
 * PCI BARs in the Home Agent Will Return Non-Zero Values During Enumeration
 * https://www.intel.com/content/www/us/en/processors/xeon/xeon-e5-v4-spec-update.html
 */
static void pci_invalid_bar(struct pci_dev *dev)
{
	dev->non_compliant_bars = 1;
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0x2fc0, pci_invalid_bar);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0x6f60, pci_invalid_bar);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0x6fa0, pci_invalid_bar);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0x6fc0, pci_invalid_bar);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0xa1ec, pci_invalid_bar);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0xa1ed, pci_invalid_bar);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0xa26c, pci_invalid_bar);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0xa26d, pci_invalid_bar);

/*
 * Device [1022:7808]
 * 23. USB Wake on Connect/Disconnect with Low Speed Devices
 * https://support.amd.com/TechDocs/46837.pdf
 * Appendix A2
 * https://support.amd.com/TechDocs/42413.pdf
 */
static void pci_fixup_amd_ehci_pme(struct pci_dev *dev)
{
	dev_info(&dev->dev, "PME# does not work under D3, disabling it\n");
	dev->pme_support &= ~((PCI_PM_CAP_PME_D3hot | PCI_PM_CAP_PME_D3cold)
		>> PCI_PM_CAP_PME_SHIFT);
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_AMD, 0x7808, pci_fixup_amd_ehci_pme);

/*
 * Device [1022:7914]
 * When in D0, PME# doesn't get asserted when plugging USB 2.0 device.
 */
static void pci_fixup_amd_fch_xhci_pme(struct pci_dev *dev)
{
	dev_info(&dev->dev, "PME# does not work under D0, disabling it\n");
	dev->pme_support &= ~(PCI_PM_CAP_PME_D0 >> PCI_PM_CAP_PME_SHIFT);
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_AMD, 0x7914, pci_fixup_amd_fch_xhci_pme);

/*
 * Apple MacBook Pro: Avoid [mem 0x7fa00000-0x7fbfffff]
 *
 * Using the [mem 0x7fa00000-0x7fbfffff] region, e.g., by assigning it to
 * the 00:1c.0 Root Port, causes a conflict with [io 0x1804], which is used
 * for soft poweroff and suspend-to-RAM.
 *
 * As far as we know, this is related to the address space, not to the Root
 * Port itself.  Attaching the quirk to the Root Port is a convenience, but
 * it could probably also be a standalone DMI quirk.
 *
 * https://bugzilla.kernel.org/show_bug.cgi?id=103211
 */
static void quirk_apple_mbp_poweroff(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;

	if ((!dmi_match(DMI_PRODUCT_NAME, "MacBookPro11,4") &&
	     !dmi_match(DMI_PRODUCT_NAME, "MacBookPro11,5")) ||
	    pdev->bus->number != 0 || pdev->devfn != PCI_DEVFN(0x1c, 0))
		return;

	res = request_mem_region(0x7fa00000, 0x200000,
				 "MacBook Pro poweroff workaround");
	if (res)
		dev_info(dev, "claimed %s %pR\n", res->name, res);
	else
		dev_info(dev, "can't work around MacBook Pro poweroff issue\n");
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x8c10, quirk_apple_mbp_poweroff);

/*
 * VMD-enabled root ports will change the source ID for all messages
 * to the VMD device. Rather than doing device matching with the source
 * ID, the AER driver should traverse the child device tree, reading
 * AER registers to find the faulting device.
 */
static void quirk_no_aersid(struct pci_dev *pdev)
{
	/* VMD Domain */
	if (is_vmd(pdev->bus) && pci_is_root_bus(pdev->bus))
		pdev->bus->bus_flags |= PCI_BUS_FLAGS_NO_AERSID;
}
DECLARE_PCI_FIXUP_CLASS_EARLY(PCI_VENDOR_ID_INTEL, PCI_ANY_ID,
			      PCI_CLASS_BRIDGE_PCI, 8, quirk_no_aersid);

static void quirk_intel_th_dnv(struct pci_dev *dev)
{
	struct resource *r = &dev->resource[4];

	/*
	 * Denverton reports 2k of RTIT_BAR (intel_th resource 4), which
	 * appears to be 4 MB in reality.
	 */
	if (r->end == r->start + 0x7ff) {
		r->start = 0;
		r->end   = 0x3fffff;
		r->flags |= IORESOURCE_UNSET;
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x19e1, quirk_intel_th_dnv);

#ifdef CONFIG_PHYS_ADDR_T_64BIT

#define AMD_141b_MMIO_BASE(x)	(0x80 + (x) * 0x8)
#define AMD_141b_MMIO_BASE_RE_MASK		BIT(0)
#define AMD_141b_MMIO_BASE_WE_MASK		BIT(1)
#define AMD_141b_MMIO_BASE_MMIOBASE_MASK	GENMASK(31,8)

#define AMD_141b_MMIO_LIMIT(x)	(0x84 + (x) * 0x8)
#define AMD_141b_MMIO_LIMIT_MMIOLIMIT_MASK	GENMASK(31,8)

#define AMD_141b_MMIO_HIGH(x)	(0x180 + (x) * 0x4)
#define AMD_141b_MMIO_HIGH_MMIOBASE_MASK	GENMASK(7,0)
#define AMD_141b_MMIO_HIGH_MMIOLIMIT_SHIFT	16
#define AMD_141b_MMIO_HIGH_MMIOLIMIT_MASK	GENMASK(23,16)

/*
 * The PCI Firmware Spec, rev 3.2, notes that ACPI should optionally allow
 * configuring host bridge windows using the _PRS and _SRS methods.
 *
 * But this is rarely implemented, so we manually enable a large 64bit BAR for
 * PCIe device on AMD Family 15h (Models 00h-1fh, 30h-3fh, 60h-7fh) Processors
 * here.
 */
static void pci_amd_enable_64bit_bar(struct pci_dev *dev)
{
	static const char *name = "PCI Bus 0000:00";
	struct resource *res, *conflict;
	u32 base, limit, high;
	struct pci_dev *other;
	unsigned i;

	if (!(pci_probe & PCI_BIG_ROOT_WINDOW))
		return;

	/* Check that we are the only device of that type */
	other = pci_get_device(dev->vendor, dev->device, NULL);
	if (other != dev ||
	    (other = pci_get_device(dev->vendor, dev->device, other))) {
		/* This is a multi-socket system, don't touch it for now */
		pci_dev_put(other);
		return;
	}

	for (i = 0; i < 8; i++) {
		pci_read_config_dword(dev, AMD_141b_MMIO_BASE(i), &base);
		pci_read_config_dword(dev, AMD_141b_MMIO_HIGH(i), &high);

		/* Is this slot free? */
		if (!(base & (AMD_141b_MMIO_BASE_RE_MASK |
			      AMD_141b_MMIO_BASE_WE_MASK)))
			break;

		base >>= 8;
		base |= high << 24;

		/* Abort if a slot already configures a 64bit BAR. */
		if (base > 0x10000)
			return;
	}
	if (i == 8)
		return;

	res = kzalloc_obj(*res);
	if (!res)
		return;

	/*
	 * Allocate a 256GB window directly below the 0xfd00000000 hardware
	 * limit (see AMD Family 15h Models 30h-3Fh BKDG, sec 2.4.6).
	 */
	res->name = name;
	res->flags = IORESOURCE_PREFETCH | IORESOURCE_MEM |
		IORESOURCE_MEM_64 | IORESOURCE_WINDOW;
	res->start = 0xbd00000000ull;
	res->end = 0xfd00000000ull - 1;

	conflict = request_resource_conflict(&iomem_resource, res);
	if (conflict) {
		kfree(res);
		if (conflict->name != name)
			return;

		/* We are resuming from suspend; just reenable the window */
		res = conflict;
	} else {
		dev_info(&dev->dev, "adding root bus resource %pR (tainting kernel)\n",
			 res);
		add_taint(TAINT_FIRMWARE_WORKAROUND, LOCKDEP_STILL_OK);
		pci_bus_add_resource(dev->bus, res);
	}

	base = ((res->start >> 8) & AMD_141b_MMIO_BASE_MMIOBASE_MASK) |
		AMD_141b_MMIO_BASE_RE_MASK | AMD_141b_MMIO_BASE_WE_MASK;
	limit = ((res->end + 1) >> 8) & AMD_141b_MMIO_LIMIT_MMIOLIMIT_MASK;
	high = ((res->start >> 40) & AMD_141b_MMIO_HIGH_MMIOBASE_MASK) |
		((((res->end + 1) >> 40) << AMD_141b_MMIO_HIGH_MMIOLIMIT_SHIFT)
		 & AMD_141b_MMIO_HIGH_MMIOLIMIT_MASK);

	pci_write_config_dword(dev, AMD_141b_MMIO_HIGH(i), high);
	pci_write_config_dword(dev, AMD_141b_MMIO_LIMIT(i), limit);
	pci_write_config_dword(dev, AMD_141b_MMIO_BASE(i), base);
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_AMD, 0x1401, pci_amd_enable_64bit_bar);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_AMD, 0x141b, pci_amd_enable_64bit_bar);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_AMD, 0x1571, pci_amd_enable_64bit_bar);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_AMD, 0x15b1, pci_amd_enable_64bit_bar);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_AMD, 0x1601, pci_amd_enable_64bit_bar);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_AMD, 0x1401, pci_amd_enable_64bit_bar);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_AMD, 0x141b, pci_amd_enable_64bit_bar);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_AMD, 0x1571, pci_amd_enable_64bit_bar);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_AMD, 0x15b1, pci_amd_enable_64bit_bar);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_AMD, 0x1601, pci_amd_enable_64bit_bar);

#define RS690_LOWER_TOP_OF_DRAM2	0x30
#define RS690_LOWER_TOP_OF_DRAM2_VALID	0x1
#define RS690_UPPER_TOP_OF_DRAM2	0x31
#define RS690_HTIU_NB_INDEX		0xA8
#define RS690_HTIU_NB_INDEX_WR_ENABLE	0x100
#define RS690_HTIU_NB_DATA		0xAC

/*
 * Some BIOS implementations support RAM above 4GB, but do not configure the
 * PCI host to respond to bus master accesses for these addresses. These
 * implementations set the TOP_OF_DRAM_SLOT1 register correctly, so PCI DMA
 * works as expected for addresses below 4GB.
 *
 * Reference: "AMD RS690 ASIC Family Register Reference Guide" (pg. 2-57)
 * https://www.amd.com/system/files/TechDocs/43372_rs690_rrg_3.00o.pdf
 */
static void rs690_fix_64bit_dma(struct pci_dev *pdev)
{
	u32 val = 0;
	phys_addr_t top_of_dram = __pa(high_memory - 1) + 1;

	if (top_of_dram <= (1ULL << 32))
		return;

	pci_write_config_dword(pdev, RS690_HTIU_NB_INDEX,
				RS690_LOWER_TOP_OF_DRAM2);
	pci_read_config_dword(pdev, RS690_HTIU_NB_DATA, &val);

	if (val)
		return;

	pci_info(pdev, "Adjusting top of DRAM to %pa for 64-bit DMA support\n", &top_of_dram);

	pci_write_config_dword(pdev, RS690_HTIU_NB_INDEX,
		RS690_UPPER_TOP_OF_DRAM2 | RS690_HTIU_NB_INDEX_WR_ENABLE);
	pci_write_config_dword(pdev, RS690_HTIU_NB_DATA, top_of_dram >> 32);

	pci_write_config_dword(pdev, RS690_HTIU_NB_INDEX,
		RS690_LOWER_TOP_OF_DRAM2 | RS690_HTIU_NB_INDEX_WR_ENABLE);
	pci_write_config_dword(pdev, RS690_HTIU_NB_DATA,
		top_of_dram | RS690_LOWER_TOP_OF_DRAM2_VALID);
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_ATI, 0x7910, rs690_fix_64bit_dma);

#endif

#ifdef CONFIG_AMD_NODE

#define AMD_15B8_RCC_DEV2_EPF0_STRAP2                                  0x10136008
#define AMD_15B8_RCC_DEV2_EPF0_STRAP2_NO_SOFT_RESET_DEV2_F0_MASK       0x00000080L

static void quirk_clear_strap_no_soft_reset_dev2_f0(struct pci_dev *dev)
{
	u32 data;

	if (!amd_smn_read(0, AMD_15B8_RCC_DEV2_EPF0_STRAP2, &data)) {
		data &= ~AMD_15B8_RCC_DEV2_EPF0_STRAP2_NO_SOFT_RESET_DEV2_F0_MASK;
		if (amd_smn_write(0, AMD_15B8_RCC_DEV2_EPF0_STRAP2, data))
			pci_err(dev, "Failed to write data 0x%x\n", data);
	} else {
		pci_err(dev, "Failed to read data\n");
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_AMD, 0x15b8, quirk_clear_strap_no_soft_reset_dev2_f0);
#endif

/*
 * When returning from D3cold to D0, firmware on some Google Coral and Reef
 * family Chromebooks with Intel Apollo Lake SoC clobbers the headers of
 * both the L1 PM Substates capability and the previous capability for the
 * "Celeron N3350/Pentium N4200/Atom E3900 Series PCI Express Port B #1".
 *
 * Save those values at enumeration-time and restore them at resume.
 */

static u16 prev_cap, l1ss_cap;
static u32 prev_header, l1ss_header;

static void chromeos_save_apl_pci_l1ss_capability(struct pci_dev *dev)
{
	int pos = PCI_CFG_SPACE_SIZE, prev = 0;
	u32 header, pheader = 0;

	while (pos) {
		pci_read_config_dword(dev, pos, &header);
		if (PCI_EXT_CAP_ID(header) == PCI_EXT_CAP_ID_L1SS) {
			prev_cap = prev;
			prev_header = pheader;
			l1ss_cap = pos;
			l1ss_header = header;
			return;
		}

		prev = pos;
		pheader = header;
		pos = PCI_EXT_CAP_NEXT(header);
	}
}

static void chromeos_fixup_apl_pci_l1ss_capability(struct pci_dev *dev)
{
	u32 header;

	if (!prev_cap || !prev_header || !l1ss_cap || !l1ss_header)
		return;

	/* Fixup the header of L1SS Capability if missing */
	pci_read_config_dword(dev, l1ss_cap, &header);
	if (header != l1ss_header) {
		pci_write_config_dword(dev, l1ss_cap, l1ss_header);
		pci_info(dev, "restore L1SS Capability header (was %#010x now %#010x)\n",
			 header, l1ss_header);
	}

	/* Fixup the link to L1SS Capability if missing */
	pci_read_config_dword(dev, prev_cap, &header);
	if (header != prev_header) {
		pci_write_config_dword(dev, prev_cap, prev_header);
		pci_info(dev, "restore previous Capability header (was %#010x now %#010x)\n",
			 header, prev_header);
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x5ad6, chromeos_save_apl_pci_l1ss_capability);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_INTEL, 0x5ad6, chromeos_fixup_apl_pci_l1ss_capability);

/*
 * Disable D3cold on Asus B1400 PCI-NVMe bridge
 *
 * On this platform with VMD off, the NVMe device cannot successfully power
 * back on from D3cold. This appears to be an untested transition by the
 * vendor: Windows leaves the NVMe and parent bridge in D0 during suspend.
 *
 * We disable D3cold on the parent bridge for simplicity, and the fact that
 * both parent bridge and NVMe device share the same power resource.
 *
 * This is only needed on BIOS versions before 308; the newer versions flip
 * StorageD3Enable from 1 to 0.
 */
static const struct dmi_system_id asus_nvme_broken_d3cold_table[] = {
	{
		.matches = {
				DMI_MATCH(DMI_SYS_VENDOR, "ASUSTeK COMPUTER INC."),
				DMI_MATCH(DMI_BIOS_VERSION, "B1400CEAE.304"),
		},
	},
	{
		.matches = {
				DMI_MATCH(DMI_SYS_VENDOR, "ASUSTeK COMPUTER INC."),
				DMI_MATCH(DMI_BIOS_VERSION, "B1400CEAE.305"),
		},
	},
	{
		.matches = {
				DMI_MATCH(DMI_SYS_VENDOR, "ASUSTeK COMPUTER INC."),
				DMI_MATCH(DMI_BIOS_VERSION, "B1400CEAE.306"),
		},
	},
	{
		.matches = {
				DMI_MATCH(DMI_SYS_VENDOR, "ASUSTeK COMPUTER INC."),
				DMI_MATCH(DMI_BIOS_VERSION, "B1400CEAE.307"),
		},
	},
	{}
};

static void asus_disable_nvme_d3cold(struct pci_dev *pdev)
{
	if (dmi_check_system(asus_nvme_broken_d3cold_table) > 0)
		pci_d3cold_disable(pdev);
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x9a09, asus_disable_nvme_d3cold);

#ifdef CONFIG_SUSPEND
/*
 * Root Ports on some AMD SoCs advertise PME_Support for D3hot and D3cold, but
 * if the SoC is put into a hardware sleep state by the amd-pmc driver, the
 * Root Ports don't generate wakeup interrupts for USB devices.
 *
 * When suspending, remove D3hot and D3cold from the PME_Support advertised
 * by the Root Port so we don't use those states if we're expecting wakeup
 * interrupts.  Restore the advertised PME_Support when resuming.
 */
static void amd_rp_pme_suspend(struct pci_dev *dev)
{
	struct pci_dev *rp;

	/*
	 * If system suspend is not in progress, we're doing runtime suspend, so
	 * amd-pmc will not be involved so PMEs during D3 work as advertised.
	 *
	 * The PMEs *do* work if amd-pmc doesn't put the SoC in the hardware
	 * sleep state, but we assume amd-pmc is always present.
	 */
	if (!pm_suspend_in_progress())
		return;

	rp = pcie_find_root_port(dev);
	if (!rp || !rp->pm_cap)
		return;

	rp->pme_support &= ~((PCI_PM_CAP_PME_D3hot|PCI_PM_CAP_PME_D3cold) >>
				    PCI_PM_CAP_PME_SHIFT);
	dev_info_once(&rp->dev, "quirk: disabling D3cold for suspend\n");
}

static void amd_rp_pme_resume(struct pci_dev *dev)
{
	struct pci_dev *rp;
	u16 pmc;

	rp = pcie_find_root_port(dev);
	if (!rp || !rp->pm_cap)
		return;

	pci_read_config_word(rp, rp->pm_cap + PCI_PM_PMC, &pmc);
	rp->pme_support = FIELD_GET(PCI_PM_CAP_PME_MASK, pmc);
}
/* Rembrandt (yellow_carp) */
DECLARE_PCI_FIXUP_SUSPEND(PCI_VENDOR_ID_AMD, 0x162e, amd_rp_pme_suspend);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_AMD, 0x162e, amd_rp_pme_resume);
DECLARE_PCI_FIXUP_SUSPEND(PCI_VENDOR_ID_AMD, 0x162f, amd_rp_pme_suspend);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_AMD, 0x162f, amd_rp_pme_resume);
/* Phoenix (pink_sardine) */
DECLARE_PCI_FIXUP_SUSPEND(PCI_VENDOR_ID_AMD, 0x1668, amd_rp_pme_suspend);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_AMD, 0x1668, amd_rp_pme_resume);
DECLARE_PCI_FIXUP_SUSPEND(PCI_VENDOR_ID_AMD, 0x1669, amd_rp_pme_suspend);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_AMD, 0x1669, amd_rp_pme_resume);

/*
 * Putting PCIe root ports on Ryzen SoCs with USB4 controllers into D3hot
 * may cause problems when the system attempts wake up from s2idle.
 *
 * On the TUXEDO Sirius 16 Gen 1 with a specific old BIOS this manifests as
 * a system hang.
 */
static const struct dmi_system_id quirk_tuxeo_rp_d3_dmi_table[] = {
	{
		.matches = {
			DMI_EXACT_MATCH(DMI_SYS_VENDOR, "TUXEDO"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "APX958"),
			DMI_EXACT_MATCH(DMI_BIOS_VERSION, "V1.00A00_20240108"),
		},
	},
	{}
};

static void quirk_tuxeo_rp_d3(struct pci_dev *pdev)
{
	struct pci_dev *root_pdev;

	if (dmi_check_system(quirk_tuxeo_rp_d3_dmi_table)) {
		root_pdev = pcie_find_root_port(pdev);
		if (root_pdev)
			root_pdev->dev_flags |= PCI_DEV_FLAGS_NO_D3;
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_AMD, 0x1502, quirk_tuxeo_rp_d3);
#endif /* CONFIG_SUSPEND */

/*
 * Deal with broken BIOSes that neglect to enable passive release,
 * which can cause problems in combination with the 82441FX/PPro MTRRs
 */
static void quirk_passive_release(struct pci_dev *dev)
{
	struct pci_dev *d = NULL;
	unsigned char dlc;

	/*
	 * We have to make sure a particular bit is set in the PIIX3
	 * ISA bridge, so we have to go out and find it.
	 */
	while ((d = pci_get_device(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_82371SB_0, d))) {
		pci_read_config_byte(d, 0x82, &dlc);
		if (!(dlc & 1<<1)) {
			pci_info(d, "PIIX3: Enabling Passive Release\n");
			dlc |= 1<<1;
			pci_write_config_byte(d, 0x82, dlc);
		}
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82441,	quirk_passive_release);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82441,	quirk_passive_release);

/* Triton requires workarounds to be used by the drivers */
static void quirk_triton(struct pci_dev *dev)
{
	if ((pci_pci_problems&PCIPCI_TRITON) == 0) {
		pci_info(dev, "Limiting direct PCI/PCI transfers\n");
		pci_pci_problems |= PCIPCI_TRITON;
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82437,	quirk_triton);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82437VX,	quirk_triton);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82439,	quirk_triton);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82439TX,	quirk_triton);

/* Natoma has some interesting boundary conditions with Zoran stuff at least */
static void quirk_natoma(struct pci_dev *dev)
{
	if ((pci_pci_problems&PCIPCI_NATOMA) == 0) {
		pci_info(dev, "Limiting direct PCI/PCI transfers\n");
		pci_pci_problems |= PCIPCI_NATOMA;
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82441,	quirk_natoma);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82443LX_0,	quirk_natoma);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82443LX_1,	quirk_natoma);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82443BX_0,	quirk_natoma);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82443BX_1,	quirk_natoma);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82443BX_2,	quirk_natoma);

/*
 * PIIX4 ACPI: Two IO regions pointed to by longwords at
 *	0x40 (64 bytes of ACPI registers)
 *	0x90 (16 bytes of SMB registers)
 * and a few strange programmable PIIX4 device resources.
 */
static void quirk_piix4_acpi(struct pci_dev *dev)
{
	u32 res_a;

	quirk_io_region(dev, 0x40, 64, PCI_BRIDGE_RESOURCES, "PIIX4 ACPI");
	quirk_io_region(dev, 0x90, 16, PCI_BRIDGE_RESOURCES+1, "PIIX4 SMB");

	/* Device resource A has enables for some of the other ones */
	pci_read_config_dword(dev, 0x5c, &res_a);

	piix4_io_quirk(dev, "PIIX4 devres B", 0x60, 3 << 21);
	piix4_io_quirk(dev, "PIIX4 devres C", 0x64, 3 << 21);

	/* Device resource D is just bitfields for static resources */

	/* Device 12 enabled? */
	if (res_a & (1 << 29)) {
		piix4_io_quirk(dev, "PIIX4 devres E", 0x68, 1 << 20);
		piix4_mem_quirk(dev, "PIIX4 devres F", 0x6c, 1 << 7);
	}
	/* Device 13 enabled? */
	if (res_a & (1 << 30)) {
		piix4_io_quirk(dev, "PIIX4 devres G", 0x70, 1 << 20);
		piix4_mem_quirk(dev, "PIIX4 devres H", 0x74, 1 << 7);
	}
	piix4_io_quirk(dev, "PIIX4 devres I", 0x78, 1 << 20);
	piix4_io_quirk(dev, "PIIX4 devres J", 0x7c, 1 << 20);
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82371AB_3,	quirk_piix4_acpi);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82443MX_3,	quirk_piix4_acpi);

#define ICH_PMBASE	0x40
#define ICH_ACPI_CNTL	0x44
#define  ICH4_ACPI_EN	0x10
#define  ICH6_ACPI_EN	0x80
#define ICH4_GPIOBASE	0x58
#define ICH4_GPIO_CNTL	0x5c
#define  ICH4_GPIO_EN	0x10
#define ICH6_GPIOBASE	0x48
#define ICH6_GPIO_CNTL	0x4c
#define  ICH6_GPIO_EN	0x10

/*
 * ICH4, ICH4-M, ICH5, ICH5-M ACPI: Three IO regions pointed to by longwords at
 *	0x40 (128 bytes of ACPI, GPIO & TCO registers)
 *	0x58 (64 bytes of GPIO I/O space)
 */
static void quirk_ich4_lpc_acpi(struct pci_dev *dev)
{
	u8 enable;

	/*
	 * The check for PCIBIOS_MIN_IO is to ensure we won't create a conflict
	 * with low legacy (and fixed) ports. We don't know the decoding
	 * priority and can't tell whether the legacy device or the one created
	 * here is really at that address.  This happens on boards with broken
	 * BIOSes.
	 */
	pci_read_config_byte(dev, ICH_ACPI_CNTL, &enable);
	if (enable & ICH4_ACPI_EN)
		quirk_io_region(dev, ICH_PMBASE, 128, PCI_BRIDGE_RESOURCES,
				 "ICH4 ACPI/GPIO/TCO");

	pci_read_config_byte(dev, ICH4_GPIO_CNTL, &enable);
	if (enable & ICH4_GPIO_EN)
		quirk_io_region(dev, ICH4_GPIOBASE, 64, PCI_BRIDGE_RESOURCES+1,
				"ICH4 GPIO");
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,    PCI_DEVICE_ID_INTEL_82801AA_0,		quirk_ich4_lpc_acpi);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,    PCI_DEVICE_ID_INTEL_82801AB_0,		quirk_ich4_lpc_acpi);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,    PCI_DEVICE_ID_INTEL_82801BA_0,		quirk_ich4_lpc_acpi);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,    PCI_DEVICE_ID_INTEL_82801BA_10,	quirk_ich4_lpc_acpi);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,    PCI_DEVICE_ID_INTEL_82801CA_0,		quirk_ich4_lpc_acpi);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,    PCI_DEVICE_ID_INTEL_82801CA_12,	quirk_ich4_lpc_acpi);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,    PCI_DEVICE_ID_INTEL_82801DB_0,		quirk_ich4_lpc_acpi);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,    PCI_DEVICE_ID_INTEL_82801DB_12,	quirk_ich4_lpc_acpi);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,    PCI_DEVICE_ID_INTEL_82801EB_0,		quirk_ich4_lpc_acpi);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,    PCI_DEVICE_ID_INTEL_ESB_1,		quirk_ich4_lpc_acpi);

static void ich6_lpc_acpi_gpio(struct pci_dev *dev)
{
	u8 enable;

	pci_read_config_byte(dev, ICH_ACPI_CNTL, &enable);
	if (enable & ICH6_ACPI_EN)
		quirk_io_region(dev, ICH_PMBASE, 128, PCI_BRIDGE_RESOURCES,
				 "ICH6 ACPI/GPIO/TCO");

	pci_read_config_byte(dev, ICH6_GPIO_CNTL, &enable);
	if (enable & ICH6_GPIO_EN)
		quirk_io_region(dev, ICH6_GPIOBASE, 64, PCI_BRIDGE_RESOURCES+1,
				"ICH6 GPIO");
}

static void ich6_lpc_generic_decode(struct pci_dev *dev, unsigned int reg,
				    const char *name, int dynsize)
{
	u32 val;
	u32 size, base;

	pci_read_config_dword(dev, reg, &val);

	/* Enabled? */
	if (!(val & 1))
		return;
	base = val & 0xfffc;
	if (dynsize) {
		/*
		 * This is not correct. It is 16, 32 or 64 bytes depending on
		 * register D31:F0:ADh bits 5:4.
		 *
		 * But this gets us at least _part_ of it.
		 */
		size = 16;
	} else {
		size = 128;
	}
	base &= ~(size-1);

	/*
	 * Just print it out for now. We should reserve it after more
	 * debugging.
	 */
	pci_info(dev, "%s PIO at %04x-%04x\n", name, base, base+size-1);
}

static void quirk_ich6_lpc(struct pci_dev *dev)
{
	/* Shared ACPI/GPIO decode with all ICH6+ */
	ich6_lpc_acpi_gpio(dev);

	/* ICH6-specific generic IO decode */
	ich6_lpc_generic_decode(dev, 0x84, "LPC Generic IO decode 1", 0);
	ich6_lpc_generic_decode(dev, 0x88, "LPC Generic IO decode 2", 1);
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_ICH6_0, quirk_ich6_lpc);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_ICH6_1, quirk_ich6_lpc);

static void ich7_lpc_generic_decode(struct pci_dev *dev, unsigned int reg,
				    const char *name)
{
	u32 val;
	u32 mask, base;

	pci_read_config_dword(dev, reg, &val);

	/* Enabled? */
	if (!(val & 1))
		return;

	/* IO base in bits 15:2, mask in bits 23:18, both are dword-based */
	base = val & 0xfffc;
	mask = (val >> 16) & 0xfc;
	mask |= 3;

	/*
	 * Just print it out for now. We should reserve it after more
	 * debugging.
	 */
	pci_info(dev, "%s PIO at %04x (mask %04x)\n", name, base, mask);
}

/* ICH7-10 has the same common LPC generic IO decode registers */
static void quirk_ich7_lpc(struct pci_dev *dev)
{
	/* We share the common ACPI/GPIO decode with ICH6 */
	ich6_lpc_acpi_gpio(dev);

	/* And have 4 ICH7+ generic decodes */
	ich7_lpc_generic_decode(dev, 0x84, "ICH7 LPC Generic IO decode 1");
	ich7_lpc_generic_decode(dev, 0x88, "ICH7 LPC Generic IO decode 2");
	ich7_lpc_generic_decode(dev, 0x8c, "ICH7 LPC Generic IO decode 3");
	ich7_lpc_generic_decode(dev, 0x90, "ICH7 LPC Generic IO decode 4");
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_ICH7_0, quirk_ich7_lpc);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_ICH7_1, quirk_ich7_lpc);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_ICH7_31, quirk_ich7_lpc);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_ICH8_0, quirk_ich7_lpc);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_ICH8_2, quirk_ich7_lpc);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_ICH8_3, quirk_ich7_lpc);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_ICH8_1, quirk_ich7_lpc);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_ICH8_4, quirk_ich7_lpc);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_ICH9_2, quirk_ich7_lpc);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_ICH9_4, quirk_ich7_lpc);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_ICH9_7, quirk_ich7_lpc);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,   PCI_DEVICE_ID_INTEL_ICH9_8, quirk_ich7_lpc);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,   PCI_DEVICE_ID_INTEL_ICH10_1, quirk_ich7_lpc);

static void quirk_transparent_bridge(struct pci_dev *dev)
{
	dev->transparent = 1;
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82380FB,	quirk_transparent_bridge);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_TOSHIBA,	0x605,	quirk_transparent_bridge);

/*
 * Enabling Link Bandwidth Management Interrupts (BW notifications) can cause
 * boot hangs on P45.
 */
static void quirk_p45_bw_notifications(struct pci_dev *dev)
{
	dev->no_bw_notif = 1;
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x2e21, quirk_p45_bw_notifications);

/*
 * Ensure C0 rev restreaming is off. This is normally done by the BIOS but
 * in the odd case it is not the results are corruption hence the presence
 * of a Linux check.
 */
static void quirk_disable_pxb(struct pci_dev *pdev)
{
	u16 config;

	if (pdev->revision != 0x04)		/* Only C0 requires this */
		return;
	pci_read_config_word(pdev, 0x40, &config);
	if (config & (1<<6)) {
		config &= ~(1<<6);
		pci_write_config_word(pdev, 0x40, config);
		pci_info(pdev, "C0 revision 450NX. Disabling PCI restreaming\n");
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82454NX,	quirk_disable_pxb);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82454NX,	quirk_disable_pxb);

/* Intel 82801CAM ICH3-M datasheet says IDE modes must be the same */
static void quirk_ide_samemode(struct pci_dev *pdev)
{
	u8 prog;

	pci_read_config_byte(pdev, PCI_CLASS_PROG, &prog);

	if (((prog & 1) && !(prog & 4)) || ((prog & 4) && !(prog & 1))) {
		pci_info(pdev, "IDE mode mismatch; forcing legacy mode\n");
		prog &= ~5;
		pdev->class &= ~5;
		pci_write_config_byte(pdev, PCI_CLASS_PROG, prog);
	}
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_82801CA_10, quirk_ide_samemode);

/*
 * This was originally an Alpha-specific thing, but it really fits here.
 * The i82375 PCI/EISA bridge appears as non-classified. Fix that.
 */
static void quirk_eisa_bridge(struct pci_dev *dev)
{
	dev->class = PCI_CLASS_BRIDGE_EISA << 8;
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82375,	quirk_eisa_bridge);

/* Enable 1k I/O space granularity on the Intel P64H2 */
static void quirk_p64h2_1k_io(struct pci_dev *dev)
{
	u16 en1k;

	pci_read_config_word(dev, 0x40, &en1k);

	if (en1k & 0x200) {
		pci_info(dev, "Enable I/O Space to 1KB granularity\n");
		dev->io_window_1k = 1;
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x1460, quirk_p64h2_1k_io);

/*
 * Originally in EDAC sources for i82875P: Intel tells BIOS developers to
 * hide device 6 which configures the overflow device access containing the
 * DRBs - this is where we expose device 6.
 * http://www.x86-secret.com/articles/tweak/pat/patsecrets-2.htm
 */
static void quirk_unhide_mch_dev6(struct pci_dev *dev)
{
	u8 reg;

	if (pci_read_config_byte(dev, 0xF4, &reg) == 0 && !(reg & 0x02)) {
		pci_info(dev, "Enabling MCH 'Overflow' Device\n");
		pci_write_config_byte(dev, 0xF4, reg | 0x02);
	}
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_82865_HB,
			quirk_unhide_mch_dev6);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_82875_HB,
			quirk_unhide_mch_dev6);

/*
 * Intel 5000 and 5100 Memory controllers have an erratum with read completion
 * coalescing (which is enabled by default on some BIOSes) and MPS of 256B.
 * Since there is no way of knowing what the PCIe MPS on each fabric will be
 * until all of the devices are discovered and buses walked, read completion
 * coalescing must be disabled.  Unfortunately, it cannot be re-enabled because
 * it is possible to hotplug a device with MPS of 256B.
 */
static void quirk_intel_mc_errata(struct pci_dev *dev)
{
	int err;
	u16 rcc;

	if (pcie_bus_config == PCIE_BUS_TUNE_OFF ||
	    pcie_bus_config == PCIE_BUS_DEFAULT)
		return;

	/*
	 * Intel erratum specifies bits to change but does not say what
	 * they are.  Keeping them magical until such time as the registers
	 * and values can be explained.
	 */
	err = pci_read_config_word(dev, 0x48, &rcc);
	if (err) {
		pci_err(dev, "Error attempting to read the read completion coalescing register\n");
		return;
	}

	if (!(rcc & (1 << 10)))
		return;

	rcc &= ~(1 << 10);

	err = pci_write_config_word(dev, 0x48, rcc);
	if (err) {
		pci_err(dev, "Error attempting to write the read completion coalescing register\n");
		return;
	}

	pr_info_once("Read completion coalescing disabled due to hardware erratum relating to 256B MPS\n");
}
/* Intel 5000 series memory controllers and ports 2-7 */
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x25c0, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x25d0, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x25d4, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x25d8, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x25e2, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x25e3, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x25e4, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x25e5, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x25e6, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x25e7, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x25f7, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x25f8, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x25f9, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x25fa, quirk_intel_mc_errata);
/* Intel 5100 series memory controllers and ports 2-7 */
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x65c0, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x65e2, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x65e3, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x65e4, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x65e5, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x65e6, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x65e7, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x65f7, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x65f8, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x65f9, quirk_intel_mc_errata);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x65fa, quirk_intel_mc_errata);

/*
 * Ivytown NTB BAR sizes are misreported by the hardware due to an erratum.
 * To work around this, query the size it should be configured to by the
 * device and modify the resource end to correspond to this new size.
 */
static void quirk_intel_ntb(struct pci_dev *dev)
{
	int rc;
	u8 val;

	rc = pci_read_config_byte(dev, 0x00D0, &val);
	if (rc)
		return;

	resource_set_size(&dev->resource[2], (resource_size_t)1 << val);

	rc = pci_read_config_byte(dev, 0x00D1, &val);
	if (rc)
		return;

	resource_set_size(&dev->resource[4], (resource_size_t)1 << val);
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x0e08, quirk_intel_ntb);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, 0x0e0d, quirk_intel_ntb);

/*
 * Some BIOS implementations leave the Intel GPU interrupts enabled, even
 * though no one is handling them (e.g., if the i915 driver is never
 * loaded).  Additionally the interrupt destination is not set up properly
 * and the interrupt ends up -somewhere-.
 *
 * These spurious interrupts are "sticky" and the kernel disables the
 * (shared) interrupt line after 100,000+ generated interrupts.
 *
 * Fix it by disabling the still enabled interrupts.  This resolves crashes
 * often seen on monitor unplug.
 */
#define I915_DEIER_REG 0x4400c
static void disable_igfx_irq(struct pci_dev *dev)
{
	void __iomem *regs = pci_iomap(dev, 0, 0);
	if (regs == NULL) {
		pci_warn(dev, "igfx quirk: Can't iomap PCI device\n");
		return;
	}

	/* Check if any interrupt line is still enabled */
	if (readl(regs + I915_DEIER_REG) != 0) {
		pci_warn(dev, "BIOS left Intel GPU interrupts enabled; disabling\n");

		writel(0, regs + I915_DEIER_REG);
	}

	pci_iounmap(dev, regs);
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x0042, disable_igfx_irq);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x0046, disable_igfx_irq);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x004a, disable_igfx_irq);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x0102, disable_igfx_irq);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x0106, disable_igfx_irq);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x010a, disable_igfx_irq);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x0152, disable_igfx_irq);

/*
 * PCI devices which are on Intel chips can skip the 10ms delay
 * before entering D3 mode.
 */
static void quirk_remove_d3hot_delay(struct pci_dev *dev)
{
	dev->d3hot_delay = 0;
}
/* C600 Series devices do not need 10ms d3hot_delay */
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x0412, quirk_remove_d3hot_delay);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x0c00, quirk_remove_d3hot_delay);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x0c0c, quirk_remove_d3hot_delay);
/* Lynxpoint-H PCH devices do not need 10ms d3hot_delay */
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x8c02, quirk_remove_d3hot_delay);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x8c18, quirk_remove_d3hot_delay);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x8c1c, quirk_remove_d3hot_delay);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x8c20, quirk_remove_d3hot_delay);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x8c22, quirk_remove_d3hot_delay);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x8c26, quirk_remove_d3hot_delay);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x8c2d, quirk_remove_d3hot_delay);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x8c31, quirk_remove_d3hot_delay);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x8c3a, quirk_remove_d3hot_delay);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x8c3d, quirk_remove_d3hot_delay);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x8c4e, quirk_remove_d3hot_delay);
/* Intel Cherrytrail devices do not need 10ms d3hot_delay */
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x2280, quirk_remove_d3hot_delay);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x2298, quirk_remove_d3hot_delay);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x229c, quirk_remove_d3hot_delay);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x22b0, quirk_remove_d3hot_delay);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x22b5, quirk_remove_d3hot_delay);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x22b7, quirk_remove_d3hot_delay);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x22b8, quirk_remove_d3hot_delay);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x22d8, quirk_remove_d3hot_delay);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL, 0x22dc, quirk_remove_d3hot_delay);

static void rom_bar_overlap_defect(struct pci_dev *dev)
{
	pci_info(dev, "working around ROM BAR overlap defect\n");
	dev->rom_bar_overlap = 1;
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0x1533, rom_bar_overlap_defect);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0x1536, rom_bar_overlap_defect);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0x1537, rom_bar_overlap_defect);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0x1538, rom_bar_overlap_defect);

/*
 * VIA Apollo KT133 needs PCI latency patch
 * Made according to a Windows driver-based patch by George E. Breese;
 * see PCI Latency Adjust on http://www.viahardware.com/download/viatweak.shtm
 * Also see http://www.au-ja.org/review-kt133a-1-en.phtml for the info on
 * which Mr Breese based his work.
 *
 * Updated based on further information from the site and also on
 * information provided by VIA
 */
static void quirk_vialatency(struct pci_dev *dev)
{
	struct pci_dev *p;
	u8 busarb;

	/*
	 * Ok, we have a potential problem chipset here. Now see if we have
	 * a buggy southbridge.
	 */
	p = pci_get_device(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_82C686, NULL);
	if (p != NULL) {

		/*
		 * 0x40 - 0x4f == 686B, 0x10 - 0x2f == 686A;
		 * thanks Dan Hollis.
		 * Check for buggy part revisions
		 */
		if (p->revision < 0x40 || p->revision > 0x42)
			goto exit;
	} else {
		p = pci_get_device(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_8231, NULL);
		if (p == NULL)	/* No problem parts */
			goto exit;

		/* Check for buggy part revisions */
		if (p->revision < 0x10 || p->revision > 0x12)
			goto exit;
	}

	/*
	 * Ok we have the problem. Now set the PCI master grant to occur
	 * every master grant. The apparent bug is that under high PCI load
	 * (quite common in Linux of course) you can get data loss when the
	 * CPU is held off the bus for 3 bus master requests.  This happens
	 * to include the IDE controllers....
	 *
	 * VIA only apply this fix when an SB Live! is present but under
	 * both Linux and Windows this isn't enough, and we have seen
	 * corruption without SB Live! but with things like 3 UDMA IDE
	 * controllers. So we ignore that bit of the VIA recommendation..
	 */
	pci_read_config_byte(dev, 0x76, &busarb);

	/*
	 * Set bit 4 and bit 5 of byte 76 to 0x01
	 * "Master priority rotation on every PCI master grant"
	 */
	busarb &= ~(1<<5);
	busarb |= (1<<4);
	pci_write_config_byte(dev, 0x76, busarb);
	pci_info(dev, "Applying VIA southbridge workaround\n");
exit:
	pci_dev_put(p);
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_8363_0,	quirk_vialatency);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_8371_1,	quirk_vialatency);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_8361,		quirk_vialatency);
/* Must restore this on a resume from RAM */
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_8363_0,	quirk_vialatency);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_8371_1,	quirk_vialatency);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_8361,		quirk_vialatency);

/* VIA Apollo VP3 needs ETBF on BT848/878 */
static void quirk_viaetbf(struct pci_dev *dev)
{
	if ((pci_pci_problems&PCIPCI_VIAETBF) == 0) {
		pci_info(dev, "Limiting direct PCI/PCI transfers\n");
		pci_pci_problems |= PCIPCI_VIAETBF;
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_82C597_0,	quirk_viaetbf);

static void quirk_vsfx(struct pci_dev *dev)
{
	if ((pci_pci_problems&PCIPCI_VSFX) == 0) {
		pci_info(dev, "Limiting direct PCI/PCI transfers\n");
		pci_pci_problems |= PCIPCI_VSFX;
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_82C576,	quirk_vsfx);

/*
 * VIA ACPI: One IO region pointed to by longword at
 *	0x48 or 0x20 (256 bytes of ACPI registers)
 */
static void quirk_vt82c586_acpi(struct pci_dev *dev)
{
	if (dev->revision & 0x10)
		quirk_io_region(dev, 0x48, 256, PCI_BRIDGE_RESOURCES,
				"vt82c586 ACPI");
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_82C586_3,	quirk_vt82c586_acpi);

/*
 * VIA VT82C686 ACPI: Three IO region pointed to by (long)words at
 *	0x48 (256 bytes of ACPI registers)
 *	0x70 (128 bytes of hardware monitoring register)
 *	0x90 (16 bytes of SMB registers)
 */
static void quirk_vt82c686_acpi(struct pci_dev *dev)
{
	quirk_vt82c586_acpi(dev);

	quirk_io_region(dev, 0x70, 128, PCI_BRIDGE_RESOURCES+1,
				 "vt82c686 HW-mon");

	quirk_io_region(dev, 0x90, 16, PCI_BRIDGE_RESOURCES+2, "vt82c686 SMB");
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_82C686_4,	quirk_vt82c686_acpi);

/*
 * VIA VT8235 ISA Bridge: Two IO regions pointed to by words at
 *	0x88 (128 bytes of power management registers)
 *	0xd0 (16 bytes of SMB registers)
 */
static void quirk_vt8235_acpi(struct pci_dev *dev)
{
	quirk_io_region(dev, 0x88, 128, PCI_BRIDGE_RESOURCES, "vt8235 PM");
	quirk_io_region(dev, 0xd0, 16, PCI_BRIDGE_RESOURCES+1, "vt8235 SMB");
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_8235,	quirk_vt8235_acpi);

/*
 * FIXME: it is questionable that quirk_via_acpi() is needed.  It shows up
 * as an ISA bridge, and does not support the PCI_INTERRUPT_LINE register
 * at all.  Therefore it seems like setting the pci_dev's IRQ to the value
 * of the ACPI SCI interrupt is only done for convenience.
 *	-jgarzik
 */
static void quirk_via_acpi(struct pci_dev *d)
{
	u8 irq;

	/* VIA ACPI device: SCI IRQ line in PCI config byte 0x42 */
	pci_read_config_byte(d, 0x42, &irq);
	irq &= 0xf;
	if (irq && (irq != 2))
		d->irq = irq;
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_82C586_3,	quirk_via_acpi);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_82C686_4,	quirk_via_acpi);

/* VIA bridges which have VLink */
static int via_vlink_dev_lo = -1, via_vlink_dev_hi = 18;

static void quirk_via_bridge(struct pci_dev *dev)
{
	/* See what bridge we have and find the device ranges */
	switch (dev->device) {
	case PCI_DEVICE_ID_VIA_82C686:
		/*
		 * The VT82C686 is special; it attaches to PCI and can have
		 * any device number. All its subdevices are functions of
		 * that single device.
		 */
		via_vlink_dev_lo = PCI_SLOT(dev->devfn);
		via_vlink_dev_hi = PCI_SLOT(dev->devfn);
		break;
	case PCI_DEVICE_ID_VIA_8237:
	case PCI_DEVICE_ID_VIA_8237A:
		via_vlink_dev_lo = 15;
		break;
	case PCI_DEVICE_ID_VIA_8235:
		via_vlink_dev_lo = 16;
		break;
	case PCI_DEVICE_ID_VIA_8231:
	case PCI_DEVICE_ID_VIA_8233_0:
	case PCI_DEVICE_ID_VIA_8233A:
	case PCI_DEVICE_ID_VIA_8233C_0:
		via_vlink_dev_lo = 17;
		break;
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_82C686,	quirk_via_bridge);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_8231,		quirk_via_bridge);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_8233_0,	quirk_via_bridge);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_8233A,	quirk_via_bridge);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_8233C_0,	quirk_via_bridge);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_8235,		quirk_via_bridge);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_8237,		quirk_via_bridge);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_8237A,	quirk_via_bridge);

/*
 * quirk_via_vlink		-	VIA VLink IRQ number update
 * @dev: PCI device
 *
 * If the device we are dealing with is on a PIC IRQ we need to ensure that
 * the IRQ line register which usually is not relevant for PCI cards, is
 * actually written so that interrupts get sent to the right place.
 *
 * We only do this on systems where a VIA south bridge was detected, and
 * only for VIA devices on the motherboard (see quirk_via_bridge above).
 */
static void quirk_via_vlink(struct pci_dev *dev)
{
	u8 irq, new_irq;

	/* Check if we have VLink at all */
	if (via_vlink_dev_lo == -1)
		return;

	new_irq = dev->irq;

	/* Don't quirk interrupts outside the legacy IRQ range */
	if (!new_irq || new_irq > 15)
		return;

	/* Internal device ? */
	if (dev->bus->number != 0 || PCI_SLOT(dev->devfn) > via_vlink_dev_hi ||
	    PCI_SLOT(dev->devfn) < via_vlink_dev_lo)
		return;

	/*
	 * This is an internal VLink device on a PIC interrupt. The BIOS
	 * ought to have set this but may not have, so we redo it.
	 */
	pci_read_config_byte(dev, PCI_INTERRUPT_LINE, &irq);
	if (new_irq != irq) {
		pci_info(dev, "VIA VLink IRQ fixup, from %d to %d\n",
			irq, new_irq);
		udelay(15);	/* unknown if delay really needed */
		pci_write_config_byte(dev, PCI_INTERRUPT_LINE, new_irq);
	}
}
DECLARE_PCI_FIXUP_ENABLE(PCI_VENDOR_ID_VIA, PCI_ANY_ID, quirk_via_vlink);

/*
 * VIA VT82C598 has its device ID settable and many BIOSes set it to the ID
 * of VT82C597 for backward compatibility.  We need to switch it off to be
 * able to recognize the real type of the chip.
 */
static void quirk_vt82c598_id(struct pci_dev *dev)
{
	pci_write_config_byte(dev, 0xfc, 0);
	pci_read_config_word(dev, PCI_DEVICE_ID, &dev->device);
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_82C597_0,	quirk_vt82c598_id);

/*
 * On ASUS A8V and A8V Deluxe boards, the onboard AC97 audio controller
 * and MC97 modem controller are disabled when a second PCI soundcard is
 * present. This patch, tweaking the VT8237 ISA bridge, enables them.
 * -- bjd
 */
static void asus_hides_ac97_lpc(struct pci_dev *dev)
{
	u8 val;
	int asus_hides_ac97 = 0;

	if (likely(dev->subsystem_vendor == PCI_VENDOR_ID_ASUSTEK)) {
		if (dev->device == PCI_DEVICE_ID_VIA_8237)
			asus_hides_ac97 = 1;
	}

	if (!asus_hides_ac97)
		return;

	pci_read_config_byte(dev, 0x50, &val);
	if (val & 0xc0) {
		pci_write_config_byte(dev, 0x50, val & (~0xc0));
		pci_read_config_byte(dev, 0x50, &val);
		if (val & 0xc0)
			pci_info(dev, "Onboard AC97/MC97 devices continue to play 'hide and seek'! 0x%x\n",
				 val);
		else
			pci_info(dev, "Enabled onboard AC97/MC97 devices\n");
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_8237, asus_hides_ac97_lpc);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_8237, asus_hides_ac97_lpc);

static void quirk_via_cx700_pci_parking_caching(struct pci_dev *dev)
{
	/*
	 * Disable PCI Bus Parking and PCI Master read caching on CX700
	 * which causes unspecified timing errors with a VT6212L on the PCI
	 * bus leading to USB2.0 packet loss.
	 *
	 * This quirk is only enabled if a second (on the external PCI bus)
	 * VT6212L is found -- the CX700 core itself also contains a USB
	 * host controller with the same PCI ID as the VT6212L.
	 */

	/* Count VT6212L instances */
	struct pci_dev *p = pci_get_device(PCI_VENDOR_ID_VIA,
		PCI_DEVICE_ID_VIA_8235_USB_2, NULL);
	uint8_t b;

	/*
	 * p should contain the first (internal) VT6212L -- see if we have
	 * an external one by searching again.
	 */
	p = pci_get_device(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_8235_USB_2, p);
	if (!p)
		return;
	pci_dev_put(p);

	if (pci_read_config_byte(dev, 0x76, &b) == 0) {
		if (b & 0x40) {
			/* Turn off PCI Bus Parking */
			pci_write_config_byte(dev, 0x76, b ^ 0x40);

			pci_info(dev, "Disabling VIA CX700 PCI parking\n");
		}
	}

	if (pci_read_config_byte(dev, 0x72, &b) == 0) {
		if (b != 0) {
			/* Turn off PCI Master read caching */
			pci_write_config_byte(dev, 0x72, 0x0);

			/* Set PCI Master Bus time-out to "1x16 PCLK" */
			pci_write_config_byte(dev, 0x75, 0x1);

			/* Disable "Read FIFO Timer" */
			pci_write_config_byte(dev, 0x77, 0x0);

			pci_info(dev, "Disabling VIA CX700 PCI caching\n");
		}
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_VIA, 0x324e, quirk_via_cx700_pci_parking_caching);

/* Chipsets where PCI->PCI transfers vanish or hang */
static void quirk_nopcipci(struct pci_dev *dev)
{
	if ((pci_pci_problems & PCIPCI_FAIL) == 0) {
		pci_info(dev, "Disabling direct PCI/PCI transfers\n");
		pci_pci_problems |= PCIPCI_FAIL;
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_SI,	PCI_DEVICE_ID_SI_5597,		quirk_nopcipci);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_SI,	PCI_DEVICE_ID_SI_496,		quirk_nopcipci);

/* SiS 96x south bridge: BIOS typically hides SMBus device...  */
static void quirk_sis_96x_smbus(struct pci_dev *dev)
{
	u8 val = 0;
	pci_read_config_byte(dev, 0x77, &val);
	if (val & 0x10) {
		pci_info(dev, "Enabling SiS 96x SMBus\n");
		pci_write_config_byte(dev, 0x77, val & ~0x10);
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_SI,	PCI_DEVICE_ID_SI_961,		quirk_sis_96x_smbus);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_SI,	PCI_DEVICE_ID_SI_962,		quirk_sis_96x_smbus);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_SI,	PCI_DEVICE_ID_SI_963,		quirk_sis_96x_smbus);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_SI,	PCI_DEVICE_ID_SI_LPC,		quirk_sis_96x_smbus);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_SI,	PCI_DEVICE_ID_SI_961,		quirk_sis_96x_smbus);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_SI,	PCI_DEVICE_ID_SI_962,		quirk_sis_96x_smbus);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_SI,	PCI_DEVICE_ID_SI_963,		quirk_sis_96x_smbus);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_SI,	PCI_DEVICE_ID_SI_LPC,		quirk_sis_96x_smbus);

/*
 * ... This is further complicated by the fact that some SiS96x south
 * bridges pretend to be 85C503/5513 instead.  In that case see if we
 * spotted a compatible north bridge to make sure.
 * (pci_find_device() doesn't work yet)
 *
 * We can also enable the sis96x bit in the discovery register..
 */
#define SIS_DETECT_REGISTER 0x40

static void quirk_sis_503(struct pci_dev *dev)
{
	u8 reg;
	u16 devid;

	pci_read_config_byte(dev, SIS_DETECT_REGISTER, &reg);
	pci_write_config_byte(dev, SIS_DETECT_REGISTER, reg | (1 << 6));
	pci_read_config_word(dev, PCI_DEVICE_ID, &devid);
	if (((devid & 0xfff0) != 0x0960) && (devid != 0x0018)) {
		pci_write_config_byte(dev, SIS_DETECT_REGISTER, reg);
		return;
	}

	/*
	 * Ok, it now shows up as a 96x.  Run the 96x quirk by hand in case
	 * it has already been processed.  (Depends on link order, which is
	 * apparently not guaranteed)
	 */
	dev->device = devid;
	quirk_sis_96x_smbus(dev);
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_SI,	PCI_DEVICE_ID_SI_503,		quirk_sis_503);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_SI,	PCI_DEVICE_ID_SI_503,		quirk_sis_503);

/*
 * ALi Magik requires workarounds to be used by the drivers that DMA to AGP
 * space. Latency must be set to 0xA and Triton workaround applied too.
 * [Info kindly provided by ALi]
 */
static void quirk_alimagik(struct pci_dev *dev)
{
	if ((pci_pci_problems&PCIPCI_ALIMAGIK) == 0) {
		pci_info(dev, "Limiting direct PCI/PCI transfers\n");
		pci_pci_problems |= PCIPCI_ALIMAGIK|PCIPCI_TRITON;
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_AL,	PCI_DEVICE_ID_AL_M1647,		quirk_alimagik);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_AL,	PCI_DEVICE_ID_AL_M1651,		quirk_alimagik);

/*
 * Let's make the southbridge information explicit instead of having to
 * worry about people probing the ACPI areas, for example.. (Yes, it
 * happens, and if you read the wrong ACPI register it will put the machine
 * to sleep with no way of waking it up again. Bummer).
 *
 * ALI M7101: Two IO regions pointed to by words at
 *	0xE0 (64 bytes of ACPI registers)
 *	0xE2 (32 bytes of SMB registers)
 */
static void quirk_ali7101_acpi(struct pci_dev *dev)
{
	quirk_io_region(dev, 0xE0, 64, PCI_BRIDGE_RESOURCES, "ali7101 ACPI");
	quirk_io_region(dev, 0xE2, 32, PCI_BRIDGE_RESOURCES+1, "ali7101 SMB");
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_AL,	PCI_DEVICE_ID_AL_M7101,		quirk_ali7101_acpi);

static void quirk_nopciamd(struct pci_dev *dev)
{
	u8 rev;
	pci_read_config_byte(dev, 0x08, &rev);
	if (rev == 0x13) {
		/* Erratum 24 */
		pci_info(dev, "Chipset erratum: Disabling direct PCI/AGP transfers\n");
		pci_pci_problems |= PCIAGP_FAIL;
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_AMD,	PCI_DEVICE_ID_AMD_8151_0,	quirk_nopciamd);

/*
 * Some CS5536 BIOSes (for example, the Soekris NET5501 board w/ comBIOS
 * ver. 1.33  20070103) don't set the correct ISA PCI region header info.
 * BAR0 should be 8 bytes; instead, it may be set to something like 8k
 * (which conflicts w/ BAR1's memory range).
 *
 * CS553x's ISA PCI BARs may also be read-only (ref:
 * https://bugzilla.kernel.org/show_bug.cgi?id=85991 - Comment #4 forward).
 */
static void quirk_cs5536_vsa(struct pci_dev *dev)
{
	static char *name = "CS5536 ISA bridge";

	if (pci_resource_len(dev, 0) != 8) {
		quirk_io(dev, 0,   8, name);	/* SMB */
		quirk_io(dev, 1, 256, name);	/* GPIO */
		quirk_io(dev, 2,  64, name);	/* MFGPT */
		pci_info(dev, "%s bug detected (incorrect header); workaround applied\n",
			 name);
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_CS5536_ISA, quirk_cs5536_vsa);

/*
 * Following the PCI ordering rules is optional on the AMD762. I'm not sure
 * what the designers were smoking but let's not inhale...
 *
 * To be fair to AMD, it follows the spec by default, it's BIOS people who
 * turn it off!
 */
static void quirk_amd_ordering(struct pci_dev *dev)
{
	u32 pcic;
	pci_read_config_dword(dev, 0x4C, &pcic);
	if ((pcic & 6) != 6) {
		pcic |= 6;
		pci_warn(dev, "BIOS failed to enable PCI standards compliance; fixing this error\n");
		pci_write_config_dword(dev, 0x4C, pcic);
		pci_read_config_dword(dev, 0x84, &pcic);
		pcic |= (1 << 23);	/* Required in this mode */
		pci_write_config_dword(dev, 0x84, pcic);
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_AMD,	PCI_DEVICE_ID_AMD_FE_GATE_700C, quirk_amd_ordering);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_AMD,	PCI_DEVICE_ID_AMD_FE_GATE_700C, quirk_amd_ordering);

static void quirk_amd_ide_mode(struct pci_dev *pdev)
{
	/* set SBX00/Hudson-2 SATA in IDE mode to AHCI mode */
	u8 tmp;

	pci_read_config_byte(pdev, PCI_CLASS_DEVICE, &tmp);
	if (tmp == 0x01) {
		pci_read_config_byte(pdev, 0x40, &tmp);
		pci_write_config_byte(pdev, 0x40, tmp|1);
		pci_write_config_byte(pdev, 0x9, 1);
		pci_write_config_byte(pdev, 0xa, 6);
		pci_write_config_byte(pdev, 0x40, tmp);

		pdev->class = PCI_CLASS_STORAGE_SATA_AHCI;
		pci_info(pdev, "set SATA to AHCI mode\n");
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_ATI, PCI_DEVICE_ID_ATI_IXP600_SATA, quirk_amd_ide_mode);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_ATI, PCI_DEVICE_ID_ATI_IXP600_SATA, quirk_amd_ide_mode);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_ATI, PCI_DEVICE_ID_ATI_IXP700_SATA, quirk_amd_ide_mode);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_ATI, PCI_DEVICE_ID_ATI_IXP700_SATA, quirk_amd_ide_mode);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_HUDSON2_SATA_IDE, quirk_amd_ide_mode);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_HUDSON2_SATA_IDE, quirk_amd_ide_mode);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_AMD, 0x7900, quirk_amd_ide_mode);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_AMD, 0x7900, quirk_amd_ide_mode);

#ifdef CONFIG_X86_32
/*
 * The VIA VP2/VP3/MVP3 seem to have some 'features'. There may be a
 * workaround but VIA don't answer queries. If you happen to have good
 * contacts at VIA ask them for me please -- Alan
 *
 * This appears to be BIOS not version dependent. So presumably there is a
 * chipset level fix.
 */
static void quirk_isa_dma_hangs(struct pci_dev *dev)
{
	if (!isa_dma_bridge_buggy) {
		isa_dma_bridge_buggy = 1;
		pci_info(dev, "Activating ISA DMA hang workarounds\n");
	}
}
/*
 * It's not totally clear which chipsets are the problematic ones.  We know
 * 82C586 and 82C596 variants are affected.
 */
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_82C586_0,	quirk_isa_dma_hangs);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_82C596,	quirk_isa_dma_hangs);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,    PCI_DEVICE_ID_INTEL_82371SB_0,  quirk_isa_dma_hangs);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_AL,	PCI_DEVICE_ID_AL_M1533,		quirk_isa_dma_hangs);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_NEC,	PCI_DEVICE_ID_NEC_CBUS_1,	quirk_isa_dma_hangs);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_NEC,	PCI_DEVICE_ID_NEC_CBUS_2,	quirk_isa_dma_hangs);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_NEC,	PCI_DEVICE_ID_NEC_CBUS_3,	quirk_isa_dma_hangs);
#endif

#ifdef CONFIG_X86_IO_APIC

#include <asm/io_apic.h>

/*
 * VIA 686A/B: If an IO-APIC is active, we need to route all on-chip
 * devices to the external APIC.
 *
 * TODO: When we have device-specific interrupt routers, this code will go
 * away from quirks.
 */
static void quirk_via_ioapic(struct pci_dev *dev)
{
	u8 tmp;

	if (nr_ioapics < 1)
		tmp = 0;    /* nothing routed to external APIC */
	else
		tmp = 0x1f; /* all known bits (4-0) routed to external APIC */

	pci_info(dev, "%s VIA external APIC routing\n",
		 tmp ? "Enabling" : "Disabling");

	/* Offset 0x58: External APIC IRQ output control */
	pci_write_config_byte(dev, 0x58, tmp);
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_82C686,	quirk_via_ioapic);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_82C686,	quirk_via_ioapic);

/*
 * VIA 8237: Some BIOSes don't set the 'Bypass APIC De-Assert Message' Bit.
 * This leads to doubled level interrupt rates.
 * Set this bit to get rid of cycle wastage.
 * Otherwise uncritical.
 */
static void quirk_via_vt8237_bypass_apic_deassert(struct pci_dev *dev)
{
	u8 misc_control2;
#define BYPASS_APIC_DEASSERT 8

	pci_read_config_byte(dev, 0x5B, &misc_control2);
	if (!(misc_control2 & BYPASS_APIC_DEASSERT)) {
		pci_info(dev, "Bypassing VIA 8237 APIC De-Assert Message\n");
		pci_write_config_byte(dev, 0x5B, misc_control2|BYPASS_APIC_DEASSERT);
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_8237,		quirk_via_vt8237_bypass_apic_deassert);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_VIA,	PCI_DEVICE_ID_VIA_8237,		quirk_via_vt8237_bypass_apic_deassert);

/*
 * The AMD IO-APIC can hang the box when an APIC IRQ is masked.
 * We check all revs >= B0 (yet not in the pre production!) as the bug
 * is currently marked NoFix
 *
 * We have multiple reports of hangs with this chipset that went away with
 * noapic specified. For the moment we assume it's the erratum. We may be wrong
 * of course. However the advice is demonstrably good even if so.
 */
static void quirk_amd_ioapic(struct pci_dev *dev)
{
	if (dev->revision >= 0x02) {
		pci_warn(dev, "I/O APIC: AMD Erratum #22 may be present. In the event of instability try\n");
		pci_warn(dev, "        : booting with the \"noapic\" option\n");
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_AMD,	PCI_DEVICE_ID_AMD_VIPER_7410,	quirk_amd_ioapic);

/*
 * Intel EESSC IO-APIC devices have bogus BARs, fix them up.
 */
static void quirk_alder_ioapic(struct pci_dev *pdev)
{
	int i;

	if ((pdev->class >> 8) != 0xff00)
		return;

	/*
	 * The first BAR is the location of the IO-APIC... we must
	 * not touch this (and it's already covered by the fixmap), so
	 * forcibly insert it into the resource tree.
	 */
	if (pci_resource_start(pdev, 0) && pci_resource_len(pdev, 0))
		insert_resource(&iomem_resource, &pdev->resource[0]);

	/*
	 * The next five BARs all seem to be rubbish, so just clean
	 * them out.
	 */
	for (i = 1; i < PCI_STD_NUM_BARS; i++)
		memset(&pdev->resource[i], 0, sizeof(pdev->resource[i]));
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_EESSC,	quirk_alder_ioapic);

static int dmi_disable_ioapicreroute(const struct dmi_system_id *d)
{
	noioapicreroute = 1;
	pr_info("%s detected: disable boot interrupt reroute\n", d->ident);

	return 0;
}

static const struct dmi_system_id boot_interrupt_dmi_table[] = {
	/*
	 * Systems to exclude from boot interrupt reroute quirks
	 */
	{
		.callback = dmi_disable_ioapicreroute,
		.ident = "ASUSTek Computer INC. M2N-LR",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "ASUSTek Computer INC."),
			DMI_MATCH(DMI_PRODUCT_NAME, "M2N-LR"),
		},
	},
	{}
};

/*
 * Boot interrupts on some chipsets cannot be turned off. For these chipsets,
 * remap the original interrupt in the Linux kernel to the boot interrupt, so
 * that a PCI device's interrupt handler is installed on the boot interrupt
 * line instead.
 */
static void quirk_reroute_to_boot_interrupts_intel(struct pci_dev *dev)
{
	dmi_check_system(boot_interrupt_dmi_table);
	if (noioapicquirk || noioapicreroute)
		return;

	dev->irq_reroute_variant = INTEL_IRQ_REROUTE_VARIANT;
	pci_info(dev, "rerouting interrupts for [%04x:%04x]\n",
		 dev->vendor, dev->device);
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_80333_0,	quirk_reroute_to_boot_interrupts_intel);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_80333_1,	quirk_reroute_to_boot_interrupts_intel);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_ESB2_0,	quirk_reroute_to_boot_interrupts_intel);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_PXH_0,	quirk_reroute_to_boot_interrupts_intel);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_PXH_1,	quirk_reroute_to_boot_interrupts_intel);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_PXHV,	quirk_reroute_to_boot_interrupts_intel);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_80332_0,	quirk_reroute_to_boot_interrupts_intel);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_80332_1,	quirk_reroute_to_boot_interrupts_intel);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_80333_0,	quirk_reroute_to_boot_interrupts_intel);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_80333_1,	quirk_reroute_to_boot_interrupts_intel);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_ESB2_0,	quirk_reroute_to_boot_interrupts_intel);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_PXH_0,	quirk_reroute_to_boot_interrupts_intel);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_PXH_1,	quirk_reroute_to_boot_interrupts_intel);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_PXHV,	quirk_reroute_to_boot_interrupts_intel);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_80332_0,	quirk_reroute_to_boot_interrupts_intel);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_80332_1,	quirk_reroute_to_boot_interrupts_intel);

/*
 * On some chipsets we can disable the generation of legacy INTx boot
 * interrupts.
 */

/*
 * IO-APIC1 on 6300ESB generates boot interrupts, see Intel order no
 * 300641-004US, section 5.7.3.
 *
 * Core IO on Xeon E5 1600/2600/4600, see Intel order no 326509-003.
 * Core IO on Xeon E5 v2, see Intel order no 329188-003.
 * Core IO on Xeon E7 v2, see Intel order no 329595-002.
 * Core IO on Xeon E5 v3, see Intel order no 330784-003.
 * Core IO on Xeon E7 v3, see Intel order no 332315-001US.
 * Core IO on Xeon E5 v4, see Intel order no 333810-002US.
 * Core IO on Xeon E7 v4, see Intel order no 332315-001US.
 * Core IO on Xeon D-1500, see Intel order no 332051-001.
 * Core IO on Xeon Scalable, see Intel order no 610950.
 */
#define INTEL_6300_IOAPIC_ABAR		0x40	/* Bus 0, Dev 29, Func 5 */
#define INTEL_6300_DISABLE_BOOT_IRQ	(1<<14)

#define INTEL_CIPINTRC_CFG_OFFSET	0x14C	/* Bus 0, Dev 5, Func 0 */
#define INTEL_CIPINTRC_DIS_INTX_ICH	(1<<25)

static void quirk_disable_intel_boot_interrupt(struct pci_dev *dev)
{
	u16 pci_config_word;
	u32 pci_config_dword;

	if (noioapicquirk)
		return;

	switch (dev->device) {
	case PCI_DEVICE_ID_INTEL_ESB_10:
		pci_read_config_word(dev, INTEL_6300_IOAPIC_ABAR,
				     &pci_config_word);
		pci_config_word |= INTEL_6300_DISABLE_BOOT_IRQ;
		pci_write_config_word(dev, INTEL_6300_IOAPIC_ABAR,
				      pci_config_word);
		break;
	case 0x3c28:	/* Xeon E5 1600/2600/4600	*/
	case 0x0e28:	/* Xeon E5/E7 V2		*/
	case 0x2f28:	/* Xeon E5/E7 V3,V4		*/
	case 0x6f28:	/* Xeon D-1500			*/
	case 0x2034:	/* Xeon Scalable Family		*/
		pci_read_config_dword(dev, INTEL_CIPINTRC_CFG_OFFSET,
				      &pci_config_dword);
		pci_config_dword |= INTEL_CIPINTRC_DIS_INTX_ICH;
		pci_write_config_dword(dev, INTEL_CIPINTRC_CFG_OFFSET,
				       pci_config_dword);
		break;
	default:
		return;
	}
	pci_info(dev, "disabled boot interrupts on device [%04x:%04x]\n",
		 dev->vendor, dev->device);
}
/*
 * Device 29 Func 5 Device IDs of IO-APIC
 * containing ABAR—APIC1 Alternate Base Address Register
 */
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_ESB_10,
		quirk_disable_intel_boot_interrupt);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_ESB_10,
		quirk_disable_intel_boot_interrupt);

/*
 * Device 5 Func 0 Device IDs of Core IO modules/hubs
 * containing Coherent Interface Protocol Interrupt Control
 *
 * Device IDs obtained from volume 2 datasheets of commented
 * families above.
 */
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	0x3c28,
		quirk_disable_intel_boot_interrupt);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	0x0e28,
		quirk_disable_intel_boot_interrupt);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	0x2f28,
		quirk_disable_intel_boot_interrupt);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	0x6f28,
		quirk_disable_intel_boot_interrupt);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_INTEL,	0x2034,
		quirk_disable_intel_boot_interrupt);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_INTEL,	0x3c28,
		quirk_disable_intel_boot_interrupt);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_INTEL,	0x0e28,
		quirk_disable_intel_boot_interrupt);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_INTEL,	0x2f28,
		quirk_disable_intel_boot_interrupt);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_INTEL,	0x6f28,
		quirk_disable_intel_boot_interrupt);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_INTEL,	0x2034,
		quirk_disable_intel_boot_interrupt);

/* Disable boot interrupts on HT-1000 */
#define BC_HT1000_FEATURE_REG		0x64
#define BC_HT1000_PIC_REGS_ENABLE	(1<<0)
#define BC_HT1000_MAP_IDX		0xC00
#define BC_HT1000_MAP_DATA		0xC01

static void quirk_disable_broadcom_boot_interrupt(struct pci_dev *dev)
{
	u32 pci_config_dword;
	u8 irq;

	if (noioapicquirk)
		return;

	pci_read_config_dword(dev, BC_HT1000_FEATURE_REG, &pci_config_dword);
	pci_write_config_dword(dev, BC_HT1000_FEATURE_REG, pci_config_dword |
			BC_HT1000_PIC_REGS_ENABLE);

	for (irq = 0x10; irq < 0x10 + 32; irq++) {
		outb(irq, BC_HT1000_MAP_IDX);
		outb(0x00, BC_HT1000_MAP_DATA);
	}

	pci_write_config_dword(dev, BC_HT1000_FEATURE_REG, pci_config_dword);

	pci_info(dev, "disabled boot interrupts on device [%04x:%04x]\n",
		 dev->vendor, dev->device);
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_SERVERWORKS,   PCI_DEVICE_ID_SERVERWORKS_HT1000SB,	quirk_disable_broadcom_boot_interrupt);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_SERVERWORKS,   PCI_DEVICE_ID_SERVERWORKS_HT1000SB,	quirk_disable_broadcom_boot_interrupt);

/* Disable boot interrupts on AMD and ATI chipsets */

/*
 * NOIOAMODE needs to be disabled to disable "boot interrupts". For AMD 8131
 * rev. A0 and B0, NOIOAMODE needs to be disabled anyway to fix IO-APIC mode
 * (due to an erratum).
 */
#define AMD_813X_MISC			0x40
#define AMD_813X_NOIOAMODE		(1<<0)
#define AMD_813X_REV_B1			0x12
#define AMD_813X_REV_B2			0x13

static void quirk_disable_amd_813x_boot_interrupt(struct pci_dev *dev)
{
	u32 pci_config_dword;

	if (noioapicquirk)
		return;
	if ((dev->revision == AMD_813X_REV_B1) ||
	    (dev->revision == AMD_813X_REV_B2))
		return;

	pci_read_config_dword(dev, AMD_813X_MISC, &pci_config_dword);
	pci_config_dword &= ~AMD_813X_NOIOAMODE;
	pci_write_config_dword(dev, AMD_813X_MISC, pci_config_dword);

	pci_info(dev, "disabled boot interrupts on device [%04x:%04x]\n",
		 dev->vendor, dev->device);
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_AMD,	PCI_DEVICE_ID_AMD_8131_BRIDGE,	quirk_disable_amd_813x_boot_interrupt);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_AMD,	PCI_DEVICE_ID_AMD_8131_BRIDGE,	quirk_disable_amd_813x_boot_interrupt);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_AMD,	PCI_DEVICE_ID_AMD_8132_BRIDGE,	quirk_disable_amd_813x_boot_interrupt);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_AMD,	PCI_DEVICE_ID_AMD_8132_BRIDGE,	quirk_disable_amd_813x_boot_interrupt);

#define AMD_8111_PCI_IRQ_ROUTING	0x56

static void quirk_disable_amd_8111_boot_interrupt(struct pci_dev *dev)
{
	u16 pci_config_word;

	if (noioapicquirk)
		return;

	pci_read_config_word(dev, AMD_8111_PCI_IRQ_ROUTING, &pci_config_word);
	if (!pci_config_word) {
		pci_info(dev, "boot interrupts on device [%04x:%04x] already disabled\n",
			 dev->vendor, dev->device);
		return;
	}
	pci_write_config_word(dev, AMD_8111_PCI_IRQ_ROUTING, 0);
	pci_info(dev, "disabled boot interrupts on device [%04x:%04x]\n",
		 dev->vendor, dev->device);
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_AMD,   PCI_DEVICE_ID_AMD_8111_SMBUS,	quirk_disable_amd_8111_boot_interrupt);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_AMD,   PCI_DEVICE_ID_AMD_8111_SMBUS,	quirk_disable_amd_8111_boot_interrupt);
#endif /* CONFIG_X86_IO_APIC */

/*
 * ATI Northbridge setups MCE the processor if you even read somewhere
 * between 0x3b0->0x3bb or read 0x3d3
 */
static void quirk_ati_exploding_mce(struct pci_dev *dev)
{
	pci_info(dev, "ATI Northbridge, reserving I/O ports 0x3b0 to 0x3bb\n");
	/* Mae rhaid i ni beidio ag edrych ar y lleoliadiau I/O hyn */
	request_region(0x3b0, 0x0C, "RadeonIGP");
	request_region(0x3d3, 0x01, "RadeonIGP");
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_ATI,	PCI_DEVICE_ID_ATI_RS100,   quirk_ati_exploding_mce);

/*
 * Accessing the device between 0x3b0 to 0x3bb or reading 0x3d3 may MCE
 * on ATI Northbridges. Cyrix MediaGX/Geode has a Slave Disconnect
 * Boundary issue.
 */
static void quirk_mediagx_master(struct pci_dev *dev)
{
	u8 reg;

	pci_read_config_byte(dev, 0x41, &reg);
	if (reg & 2) {
		reg &= ~2;
		pci_info(dev, "Fixup for MediaGX/Geode Slave Disconnect Boundary (0x41=0x%02x)\n",
			 reg);
		pci_write_config_byte(dev, 0x41, reg);
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_CYRIX,	PCI_DEVICE_ID_CYRIX_PCI_MASTER, quirk_mediagx_master);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_CYRIX,	PCI_DEVICE_ID_CYRIX_PCI_MASTER, quirk_mediagx_master);

/* Serverworks CSB5 IDE does not fully support native mode */
static void quirk_svwks_csb5ide(struct pci_dev *pdev)
{
	u8 prog;
	pci_read_config_byte(pdev, PCI_CLASS_PROG, &prog);
	if (prog & 5) {
		prog &= ~5;
		pdev->class &= ~5;
		pci_write_config_byte(pdev, PCI_CLASS_PROG, prog);
		/* PCI layer will sort out resources */
	}
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_SERVERWORKS, PCI_DEVICE_ID_SERVERWORKS_CSB5IDE, quirk_svwks_csb5ide);

/*
 * Under some circumstances, AER is not linked with extended capabilities.
 * Force it to be linked by setting the corresponding control bit in the
 * config space.
 */
static void quirk_nvidia_ck804_pcie_aer_ext_cap(struct pci_dev *dev)
{
	uint8_t b;

	if (pci_read_config_byte(dev, 0xf41, &b) == 0) {
		if (!(b & 0x20)) {
			pci_write_config_byte(dev, 0xf41, b | 0x20);
			pci_info(dev, "Linking AER extended capability\n");
		}
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_NVIDIA,  PCI_DEVICE_ID_NVIDIA_CK804_PCIE,
			quirk_nvidia_ck804_pcie_aer_ext_cap);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_NVIDIA,  PCI_DEVICE_ID_NVIDIA_CK804_PCIE,
			quirk_nvidia_ck804_pcie_aer_ext_cap);

/*
 * On ASUS P4B boards, the SMBus PCI Device within the ICH2/4 southbridge
 * is not activated. The myth is that Asus said that they do not want the
 * users to be irritated by just another PCI Device in the Win98 device
 * manager. (see the file prog/hotplug/README.p4b in the lm_sensors
 * package 2.7.0 for details)
 *
 * The SMBus PCI Device can be activated by setting a bit in the ICH LPC
 * bridge. Unfortunately, this device has no subvendor/subdevice ID. So it
 * becomes necessary to do this tweak in two steps -- the chosen trigger
 * is either the Host bridge (preferred) or on-board VGA controller.
 *
 * Note that we used to unhide the SMBus that way on Toshiba laptops
 * (Satellite A40 and Tecra M2) but then found that the thermal management
 * was done by SMM code, which could cause unsynchronized concurrent
 * accesses to the SMBus registers, with potentially bad effects. Thus you
 * should be very careful when adding new entries: if SMM is accessing the
 * Intel SMBus, this is a very good reason to leave it hidden.
 *
 * Likewise, many recent laptops use ACPI for thermal management. If the
 * ACPI DSDT code accesses the SMBus, then Linux should not access it
 * natively, and keeping the SMBus hidden is the right thing to do. If you
 * are about to add an entry in the table below, please first disassemble
 * the DSDT and double-check that there is no code accessing the SMBus.
 */
static int asus_hides_smbus;

static void asus_hides_smbus_hostbridge(struct pci_dev *dev)
{
	if (unlikely(dev->subsystem_vendor == PCI_VENDOR_ID_ASUSTEK)) {
		if (dev->device == PCI_DEVICE_ID_INTEL_82845_HB)
			switch (dev->subsystem_device) {
			case 0x8025: /* P4B-LX */
			case 0x8070: /* P4B */
			case 0x8088: /* P4B533 */
			case 0x1626: /* L3C notebook */
				asus_hides_smbus = 1;
			}
		else if (dev->device == PCI_DEVICE_ID_INTEL_82845G_HB)
			switch (dev->subsystem_device) {
			case 0x80b1: /* P4GE-V */
			case 0x80b2: /* P4PE */
			case 0x8093: /* P4B533-V */
				asus_hides_smbus = 1;
			}
		else if (dev->device == PCI_DEVICE_ID_INTEL_82850_HB)
			switch (dev->subsystem_device) {
			case 0x8030: /* P4T533 */
				asus_hides_smbus = 1;
			}
		else if (dev->device == PCI_DEVICE_ID_INTEL_7205_0)
			switch (dev->subsystem_device) {
			case 0x8070: /* P4G8X Deluxe */
				asus_hides_smbus = 1;
			}
		else if (dev->device == PCI_DEVICE_ID_INTEL_E7501_MCH)
			switch (dev->subsystem_device) {
			case 0x80c9: /* PU-DLS */
				asus_hides_smbus = 1;
			}
		else if (dev->device == PCI_DEVICE_ID_INTEL_82855GM_HB)
			switch (dev->subsystem_device) {
			case 0x1751: /* M2N notebook */
			case 0x1821: /* M5N notebook */
			case 0x1897: /* A6L notebook */
				asus_hides_smbus = 1;
			}
		else if (dev->device == PCI_DEVICE_ID_INTEL_82855PM_HB)
			switch (dev->subsystem_device) {
			case 0x184b: /* W1N notebook */
			case 0x186a: /* M6Ne notebook */
				asus_hides_smbus = 1;
			}
		else if (dev->device == PCI_DEVICE_ID_INTEL_82865_HB)
			switch (dev->subsystem_device) {
			case 0x80f2: /* P4P800-X */
				asus_hides_smbus = 1;
			}
		else if (dev->device == PCI_DEVICE_ID_INTEL_82915GM_HB)
			switch (dev->subsystem_device) {
			case 0x1882: /* M6V notebook */
			case 0x1977: /* A6VA notebook */
				asus_hides_smbus = 1;
			}
	} else if (unlikely(dev->subsystem_vendor == PCI_VENDOR_ID_HP)) {
		if (dev->device ==  PCI_DEVICE_ID_INTEL_82855PM_HB)
			switch (dev->subsystem_device) {
			case 0x088C: /* HP Compaq nc8000 */
			case 0x0890: /* HP Compaq nc6000 */
				asus_hides_smbus = 1;
			}
		else if (dev->device == PCI_DEVICE_ID_INTEL_82865_HB)
			switch (dev->subsystem_device) {
			case 0x12bc: /* HP D330L */
			case 0x12bd: /* HP D530 */
			case 0x006a: /* HP Compaq nx9500 */
				asus_hides_smbus = 1;
			}
		else if (dev->device == PCI_DEVICE_ID_INTEL_82875_HB)
			switch (dev->subsystem_device) {
			case 0x12bf: /* HP xw4100 */
				asus_hides_smbus = 1;
			}
	} else if (unlikely(dev->subsystem_vendor == PCI_VENDOR_ID_SAMSUNG)) {
		if (dev->device ==  PCI_DEVICE_ID_INTEL_82855PM_HB)
			switch (dev->subsystem_device) {
			case 0xC00C: /* Samsung P35 notebook */
				asus_hides_smbus = 1;
		}
	} else if (unlikely(dev->subsystem_vendor == PCI_VENDOR_ID_COMPAQ)) {
		if (dev->device == PCI_DEVICE_ID_INTEL_82855PM_HB)
			switch (dev->subsystem_device) {
			case 0x0058: /* Compaq Evo N620c */
				asus_hides_smbus = 1;
			}
		else if (dev->device == PCI_DEVICE_ID_INTEL_82810_IG3)
			switch (dev->subsystem_device) {
			case 0xB16C: /* Compaq Deskpro EP 401963-001 (PCA# 010174) */
				/* Motherboard doesn't have Host bridge
				 * subvendor/subdevice IDs, therefore checking
				 * its on-board VGA controller */
				asus_hides_smbus = 1;
			}
		else if (dev->device == PCI_DEVICE_ID_INTEL_82801DB_2)
			switch (dev->subsystem_device) {
			case 0x00b8: /* Compaq Evo D510 CMT */
			case 0x00b9: /* Compaq Evo D510 SFF */
			case 0x00ba: /* Compaq Evo D510 USDT */
				/* Motherboard doesn't have Host bridge
				 * subvendor/subdevice IDs and on-board VGA
				 * controller is disabled if an AGP card is
				 * inserted, therefore checking USB UHCI
				 * Controller #1 */
				asus_hides_smbus = 1;
			}
		else if (dev->device == PCI_DEVICE_ID_INTEL_82815_CGC)
			switch (dev->subsystem_device) {
			case 0x001A: /* Compaq Deskpro EN SSF P667 815E */
				/* Motherboard doesn't have host bridge
				 * subvendor/subdevice IDs, therefore checking
				 * its on-board VGA controller */
				asus_hides_smbus = 1;
			}
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82845_HB,	asus_hides_smbus_hostbridge);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82845G_HB,	asus_hides_smbus_hostbridge);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82850_HB,	asus_hides_smbus_hostbridge);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82865_HB,	asus_hides_smbus_hostbridge);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82875_HB,	asus_hides_smbus_hostbridge);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_7205_0,	asus_hides_smbus_hostbridge);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_E7501_MCH,	asus_hides_smbus_hostbridge);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82855PM_HB,	asus_hides_smbus_hostbridge);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82855GM_HB,	asus_hides_smbus_hostbridge);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82915GM_HB, asus_hides_smbus_hostbridge);

DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82810_IG3,	asus_hides_smbus_hostbridge);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82801DB_2,	asus_hides_smbus_hostbridge);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82815_CGC,	asus_hides_smbus_hostbridge);

static void asus_hides_smbus_lpc(struct pci_dev *dev)
{
	u16 val;

	if (likely(!asus_hides_smbus))
		return;

	pci_read_config_word(dev, 0xF2, &val);
	if (val & 0x8) {
		pci_write_config_word(dev, 0xF2, val & (~0x8));
		pci_read_config_word(dev, 0xF2, &val);
		if (val & 0x8)
			pci_info(dev, "i801 SMBus device continues to play 'hide and seek'! 0x%x\n",
				 val);
		else
			pci_info(dev, "Enabled i801 SMBus device\n");
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82801AA_0,	asus_hides_smbus_lpc);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82801DB_0,	asus_hides_smbus_lpc);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82801BA_0,	asus_hides_smbus_lpc);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82801CA_0,	asus_hides_smbus_lpc);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82801CA_12,	asus_hides_smbus_lpc);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82801DB_12,	asus_hides_smbus_lpc);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82801EB_0,	asus_hides_smbus_lpc);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82801AA_0,	asus_hides_smbus_lpc);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82801DB_0,	asus_hides_smbus_lpc);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82801BA_0,	asus_hides_smbus_lpc);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82801CA_0,	asus_hides_smbus_lpc);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82801CA_12,	asus_hides_smbus_lpc);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82801DB_12,	asus_hides_smbus_lpc);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_82801EB_0,	asus_hides_smbus_lpc);

/* It appears we just have one such device. If not, we have a warning */
static void __iomem *asus_rcba_base;
static void asus_hides_smbus_lpc_ich6_suspend(struct pci_dev *dev)
{
	u32 rcba;

	if (likely(!asus_hides_smbus))
		return;
	WARN_ON(asus_rcba_base);

	pci_read_config_dword(dev, 0xF0, &rcba);
	/* use bits 31:14, 16 kB aligned */
	asus_rcba_base = ioremap(rcba & 0xFFFFC000, 0x4000);
	if (asus_rcba_base == NULL)
		return;
}

static void asus_hides_smbus_lpc_ich6_resume_early(struct pci_dev *dev)
{
	u32 val;

	if (likely(!asus_hides_smbus || !asus_rcba_base))
		return;

	/* read the Function Disable register, dword mode only */
	val = readl(asus_rcba_base + 0x3418);

	/* enable the SMBus device */
	writel(val & 0xFFFFFFF7, asus_rcba_base + 0x3418);
}

static void asus_hides_smbus_lpc_ich6_resume(struct pci_dev *dev)
{
	if (likely(!asus_hides_smbus || !asus_rcba_base))
		return;

	iounmap(asus_rcba_base);
	asus_rcba_base = NULL;
	pci_info(dev, "Enabled ICH6/i801 SMBus device\n");
}

static void asus_hides_smbus_lpc_ich6(struct pci_dev *dev)
{
	asus_hides_smbus_lpc_ich6_suspend(dev);
	asus_hides_smbus_lpc_ich6_resume_early(dev);
	asus_hides_smbus_lpc_ich6_resume(dev);
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_ICH6_1,	asus_hides_smbus_lpc_ich6);
DECLARE_PCI_FIXUP_SUSPEND(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_ICH6_1,	asus_hides_smbus_lpc_ich6_suspend);
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_ICH6_1,	asus_hides_smbus_lpc_ich6_resume);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_INTEL,	PCI_DEVICE_ID_INTEL_ICH6_1,	asus_hides_smbus_lpc_ich6_resume_early);

/*
 * Go through the list of HyperTransport capabilities and return 1 if a HT
 * MSI capability is found and enabled.
 */
static int msi_ht_cap_enabled(struct pci_dev *dev)
{
	int pos, ttl = PCI_FIND_CAP_TTL;

	pos = pci_find_ht_capability(dev, HT_CAPTYPE_MSI_MAPPING);
	while (pos && ttl--) {
		u8 flags;

		if (pci_read_config_byte(dev, pos + HT_MSI_FLAGS,
					 &flags) == 0) {
			pci_info(dev, "Found %s HT MSI Mapping\n",
				flags & HT_MSI_FLAGS_ENABLE ?
				"enabled" : "disabled");
			return (flags & HT_MSI_FLAGS_ENABLE) != 0;
		}

		pos = pci_find_next_ht_capability(dev, pos,
						  HT_CAPTYPE_MSI_MAPPING);
	}
	return 0;
}

/* Check the HyperTransport MSI mapping to know whether MSI is enabled or not */
static void quirk_msi_ht_cap(struct pci_dev *dev)
{
	if (!msi_ht_cap_enabled(dev))
		quirk_disable_msi(dev);
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_SERVERWORKS, PCI_DEVICE_ID_SERVERWORKS_HT2000_PCIE,
			quirk_msi_ht_cap);

/*
 * The nVidia CK804 chipset may have 2 HT MSI mappings.  MSI is supported
 * if the MSI capability is set in any of these mappings.
 */
static void quirk_nvidia_ck804_msi_ht_cap(struct pci_dev *dev)
{
	struct pci_dev *pdev;

	/*
	 * Check HT MSI cap on this chipset and the root one.  A single one
	 * having MSI is enough to be sure that MSI is supported.
	 */
	pdev = pci_get_slot(dev->bus, 0);
	if (!pdev)
		return;
	if (!msi_ht_cap_enabled(pdev))
		quirk_msi_ht_cap(dev);
	pci_dev_put(pdev);
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_NVIDIA, PCI_DEVICE_ID_NVIDIA_CK804_PCIE,
			quirk_nvidia_ck804_msi_ht_cap);

/* Force enable MSI mapping capability on HT bridges */
static void ht_enable_msi_mapping(struct pci_dev *dev)
{
	int pos, ttl = PCI_FIND_CAP_TTL;

	pos = pci_find_ht_capability(dev, HT_CAPTYPE_MSI_MAPPING);
	while (pos && ttl--) {
		u8 flags;

		if (pci_read_config_byte(dev, pos + HT_MSI_FLAGS,
					 &flags) == 0) {
			pci_info(dev, "Enabling HT MSI Mapping\n");

			pci_write_config_byte(dev, pos + HT_MSI_FLAGS,
					      flags | HT_MSI_FLAGS_ENABLE);
		}
		pos = pci_find_next_ht_capability(dev, pos,
						  HT_CAPTYPE_MSI_MAPPING);
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_SERVERWORKS,
			 PCI_DEVICE_ID_SERVERWORKS_HT1000_PXB,
			 ht_enable_msi_mapping);
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_8132_BRIDGE,
			 ht_enable_msi_mapping);

/*
 * The P5N32-SLI motherboards from Asus have a problem with MSI
 * for the MCP55 NIC. It is not yet determined whether the MSI problem
 * also affects other devices. As for now, turn off MSI for this device.
 */
static void nvenet_msi_disable(struct pci_dev *dev)
{
	const char *board_name = dmi_get_system_info(DMI_BOARD_NAME);

	if (board_name &&
	    (strstr(board_name, "P5N32-SLI PREMIUM") ||
	     strstr(board_name, "P5N32-E SLI"))) {
		pci_info(dev, "Disabling MSI for MCP55 NIC on P5N32-SLI\n");
		dev->no_msi = 1;
	}
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_NVIDIA,
			PCI_DEVICE_ID_NVIDIA_NVENET_15,
			nvenet_msi_disable);

/*
 * Some versions of the MCP55 bridge from Nvidia have a legacy IRQ routing
 * config register.  This register controls the routing of legacy
 * interrupts from devices that route through the MCP55.  If this register
 * is misprogrammed, interrupts are only sent to the BSP, unlike
 * conventional systems where the IRQ is broadcast to all online CPUs.  Not
 * having this register set properly prevents kdump from booting up
 * properly, so let's make sure that we have it set correctly.
 * Note that this is an undocumented register.
 */
static void nvbridge_check_legacy_irq_routing(struct pci_dev *dev)
{
	u32 cfg;

	if (!pci_find_capability(dev, PCI_CAP_ID_HT))
		return;

	pci_read_config_dword(dev, 0x74, &cfg);

	if (cfg & ((1 << 2) | (1 << 15))) {
		pr_info("Rewriting IRQ routing register on MCP55\n");
		cfg &= ~((1 << 2) | (1 << 15));
		pci_write_config_dword(dev, 0x74, cfg);
	}
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_NVIDIA,
			PCI_DEVICE_ID_NVIDIA_MCP55_BRIDGE_V0,
			nvbridge_check_legacy_irq_routing);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_NVIDIA,
			PCI_DEVICE_ID_NVIDIA_MCP55_BRIDGE_V4,
			nvbridge_check_legacy_irq_routing);

static int ht_check_msi_mapping(struct pci_dev *dev)
{
	int pos, ttl = PCI_FIND_CAP_TTL;
	int found = 0;

	/* Check if there is HT MSI cap or enabled on this device */
	pos = pci_find_ht_capability(dev, HT_CAPTYPE_MSI_MAPPING);
	while (pos && ttl--) {
		u8 flags;

		if (found < 1)
			found = 1;
		if (pci_read_config_byte(dev, pos + HT_MSI_FLAGS,
					 &flags) == 0) {
			if (flags & HT_MSI_FLAGS_ENABLE) {
				if (found < 2) {
					found = 2;
					break;
				}
			}
		}
		pos = pci_find_next_ht_capability(dev, pos,
						  HT_CAPTYPE_MSI_MAPPING);
	}

	return found;
}

static int host_bridge_with_leaf(struct pci_dev *host_bridge)
{
	struct pci_dev *dev;
	int pos;
	int i, dev_no;
	int found = 0;

	dev_no = host_bridge->devfn >> 3;
	for (i = dev_no + 1; i < 0x20; i++) {
		dev = pci_get_slot(host_bridge->bus, PCI_DEVFN(i, 0));
		if (!dev)
			continue;

		/* found next host bridge? */
		pos = pci_find_ht_capability(dev, HT_CAPTYPE_SLAVE);
		if (pos != 0) {
			pci_dev_put(dev);
			break;
		}

		if (ht_check_msi_mapping(dev)) {
			found = 1;
			pci_dev_put(dev);
			break;
		}
		pci_dev_put(dev);
	}

	return found;
}

#define PCI_HT_CAP_SLAVE_CTRL0     4    /* link control */
#define PCI_HT_CAP_SLAVE_CTRL1     8    /* link control to */

static int is_end_of_ht_chain(struct pci_dev *dev)
{
	int pos, ctrl_off;
	int end = 0;
	u16 flags, ctrl;

	pos = pci_find_ht_capability(dev, HT_CAPTYPE_SLAVE);

	if (!pos)
		goto out;

	pci_read_config_word(dev, pos + PCI_CAP_FLAGS, &flags);

	ctrl_off = ((flags >> 10) & 1) ?
			PCI_HT_CAP_SLAVE_CTRL0 : PCI_HT_CAP_SLAVE_CTRL1;
	pci_read_config_word(dev, pos + ctrl_off, &ctrl);

	if (ctrl & (1 << 6))
		end = 1;

out:
	return end;
}

static void nv_ht_enable_msi_mapping(struct pci_dev *dev)
{
	struct pci_dev *host_bridge;
	int pos;
	int i, dev_no;
	int found = 0;

	dev_no = dev->devfn >> 3;
	for (i = dev_no; i >= 0; i--) {
		host_bridge = pci_get_slot(dev->bus, PCI_DEVFN(i, 0));
		if (!host_bridge)
			continue;

		pos = pci_find_ht_capability(host_bridge, HT_CAPTYPE_SLAVE);
		if (pos != 0) {
			found = 1;
			break;
		}
		pci_dev_put(host_bridge);
	}

	if (!found)
		return;

	/* don't enable end_device/host_bridge with leaf directly here */
	if (host_bridge == dev && is_end_of_ht_chain(host_bridge) &&
	    host_bridge_with_leaf(host_bridge))
		goto out;

	/* root did that ! */
	if (msi_ht_cap_enabled(host_bridge))
		goto out;

	ht_enable_msi_mapping(dev);

out:
	pci_dev_put(host_bridge);
}

static void ht_disable_msi_mapping(struct pci_dev *dev)
{
	int pos, ttl = PCI_FIND_CAP_TTL;

	pos = pci_find_ht_capability(dev, HT_CAPTYPE_MSI_MAPPING);
	while (pos && ttl--) {
		u8 flags;

		if (pci_read_config_byte(dev, pos + HT_MSI_FLAGS,
					 &flags) == 0) {
			pci_info(dev, "Disabling HT MSI Mapping\n");

			pci_write_config_byte(dev, pos + HT_MSI_FLAGS,
					      flags & ~HT_MSI_FLAGS_ENABLE);
		}
		pos = pci_find_next_ht_capability(dev, pos,
						  HT_CAPTYPE_MSI_MAPPING);
	}
}

static void __nv_msi_ht_cap_quirk(struct pci_dev *dev, int all)
{
	struct pci_dev *host_bridge;
	int pos;
	int found;

	if (!pci_msi_enabled())
		return;

	/* check if there is HT MSI cap or enabled on this device */
	found = ht_check_msi_mapping(dev);

	/* no HT MSI CAP */
	if (found == 0)
		return;

	/*
	 * HT MSI mapping should be disabled on devices that are below
	 * a non-HyperTransport host bridge. Locate the host bridge.
	 */
	host_bridge = pci_get_domain_bus_and_slot(pci_domain_nr(dev->bus), 0,
						  PCI_DEVFN(0, 0));
	if (host_bridge == NULL) {
		pci_warn(dev, "nv_msi_ht_cap_quirk didn't locate host bridge\n");
		return;
	}

	pos = pci_find_ht_capability(host_bridge, HT_CAPTYPE_SLAVE);
	if (pos != 0) {
		/* Host bridge is to HT */
		if (found == 1) {
			/* it is not enabled, try to enable it */
			if (all)
				ht_enable_msi_mapping(dev);
			else
				nv_ht_enable_msi_mapping(dev);
		}
		goto out;
	}

	/* HT MSI is not enabled */
	if (found == 1)
		goto out;

	/* Host bridge is not to HT, disable HT MSI mapping on this device */
	ht_disable_msi_mapping(dev);

out:
	pci_dev_put(host_bridge);
}

static void nv_msi_ht_cap_quirk_all(struct pci_dev *dev)
{
	return __nv_msi_ht_cap_quirk(dev, 1);
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_AL, PCI_ANY_ID, nv_msi_ht_cap_quirk_all);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_AL, PCI_ANY_ID, nv_msi_ht_cap_quirk_all);

static void nv_msi_ht_cap_quirk_leaf(struct pci_dev *dev)
{
	return __nv_msi_ht_cap_quirk(dev, 0);
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_NVIDIA, PCI_ANY_ID, nv_msi_ht_cap_quirk_leaf);
DECLARE_PCI_FIXUP_RESUME_EARLY(PCI_VENDOR_ID_NVIDIA, PCI_ANY_ID, nv_msi_ht_cap_quirk_leaf);

#ifdef CONFIG_HAS_IOPORT
/*
 * Intel NM10 "Tiger Point" LPC PM1a_STS.BM_STS must be clear
 * for some HT machines to use C4 w/o hanging.
 */
static void quirk_tigerpoint_bm_sts(struct pci_dev *dev)
{
	u32 pmbase;
	u16 pm1a;

	pci_read_config_dword(dev, 0x40, &pmbase);
	pmbase = pmbase & 0xff80;
	pm1a = inw(pmbase);

	if (pm1a & 0x10) {
		pci_info(dev, FW_BUG "Tiger Point LPC.BM_STS cleared\n");
		outw(0x10, pmbase);
	}
}
DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_TGP_LPC, quirk_tigerpoint_bm_sts);
#endif

#ifdef CONFIG_DMAR_TABLE
#define VTUNCERRMSK_REG	0x1ac
#define VTD_MSK_SPEC_ERRORS	(1 << 31)
/*
 * This is a quirk for masking VT-d spec-defined errors to platform error
 * handling logic. Without this, platforms using Intel 7500, 5500 chipsets
 * (and the derivative chipsets like X58 etc) seem to generate NMI/SMI (based
 * on the RAS config settings of the platform) when a VT-d fault happens.
 * The resulting SMI caused the system to hang.
 *
 * VT-d spec-related errors are already handled by the VT-d OS code, so no
 * need to report the same error through other channels.
 */
static void vtd_mask_spec_errors(struct pci_dev *dev)
{
	u32 word;

	pci_read_config_dword(dev, VTUNCERRMSK_REG, &word);
	pci_write_config_dword(dev, VTUNCERRMSK_REG, word | VTD_MSK_SPEC_ERRORS);
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0x342e, vtd_mask_spec_errors);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0x3c28, vtd_mask_spec_errors);
#endif

#ifdef CONFIG_ACPI
/*
 * Apple: Shutdown Cactus Ridge Thunderbolt controller.
 *
 * On Apple hardware the Cactus Ridge Thunderbolt controller needs to be
 * shutdown before suspend. Otherwise the native host interface (NHI) will not
 * be present after resume if a device was plugged in before suspend.
 *
 * The Thunderbolt controller consists of a PCIe switch with downstream
 * bridges leading to the NHI and to the tunnel PCI bridges.
 *
 * This quirk cuts power to the whole chip. Therefore we have to apply it
 * during suspend_noirq of the upstream bridge.
 *
 * Power is automagically restored before resume. No action is needed.
 */
static void quirk_apple_poweroff_thunderbolt(struct pci_dev *dev)
{
	acpi_handle bridge, SXIO, SXFP, SXLV;

	if (!x86_apple_machine)
		return;
	if (pci_pcie_type(dev) != PCI_EXP_TYPE_UPSTREAM)
		return;

	/*
	 * SXIO/SXFP/SXLF turns off power to the Thunderbolt controller.
	 * We don't know how to turn it back on again, but firmware does,
	 * so we can only use SXIO/SXFP/SXLF if we're suspending via
	 * firmware.
	 */
	if (!pm_suspend_via_firmware())
		return;

	bridge = ACPI_HANDLE(&dev->dev);
	if (!bridge)
		return;

	/*
	 * SXIO and SXLV are present only on machines requiring this quirk.
	 * Thunderbolt bridges in external devices might have the same
	 * device ID as those on the host, but they will not have the
	 * associated ACPI methods. This implicitly checks that we are at
	 * the right bridge.
	 */
	if (ACPI_FAILURE(acpi_get_handle(bridge, "DSB0.NHI0.SXIO", &SXIO))
	    || ACPI_FAILURE(acpi_get_handle(bridge, "DSB0.NHI0.SXFP", &SXFP))
	    || ACPI_FAILURE(acpi_get_handle(bridge, "DSB0.NHI0.SXLV", &SXLV)))
		return;
	pci_info(dev, "quirk: cutting power to Thunderbolt controller...\n");

	/* magic sequence */
	acpi_execute_simple_method(SXIO, NULL, 1);
	acpi_execute_simple_method(SXFP, NULL, 0);
	msleep(300);
	acpi_execute_simple_method(SXLV, NULL, 0);
	acpi_execute_simple_method(SXIO, NULL, 0);
	acpi_execute_simple_method(SXLV, NULL, 0);
}
DECLARE_PCI_FIXUP_SUSPEND_LATE(PCI_VENDOR_ID_INTEL,
			       PCI_DEVICE_ID_INTEL_CACTUS_RIDGE_4C,
			       quirk_apple_poweroff_thunderbolt);
#endif
