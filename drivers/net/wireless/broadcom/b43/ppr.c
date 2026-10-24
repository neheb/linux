// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Broadcom B43 wireless driver
 * PPR (Power Per Rate) management
 *
 * Copyright (c) 2014 Rafał Miłecki <zajec5@gmail.com>
 */

#include "ppr.h"
#include "b43.h"

#define ppr_for_each_entry(ppr, i, entry)				\
	for (i = 0, entry = &(ppr)->__all_rates[i];			\
	     i < B43_PPR_RATES_NUM;					\
	     i++, entry++)

void b43_ppr_clear(struct b43_wldev *dev, struct b43_ppr *ppr)
{
	memset(ppr, 0, sizeof(*ppr));

	/* Compile-time PPR check */
	BUILD_BUG_ON(sizeof(struct b43_ppr) != B43_PPR_RATES_NUM * sizeof(u8));
}

void b43_ppr_add(struct b43_wldev *dev, struct b43_ppr *ppr, int diff)
{
	int i;
	u8 *rate;

	ppr_for_each_entry(ppr, i, rate) {
		*rate = clamp_val(*rate + diff, 0, 127);
	}
}

void b43_ppr_apply_max(struct b43_wldev *dev, struct b43_ppr *ppr, u8 max)
{
	int i;
	u8 *rate;

	ppr_for_each_entry(ppr, i, rate) {
		*rate = min(*rate, max);
	}
}

void b43_ppr_apply_min(struct b43_wldev *dev, struct b43_ppr *ppr, u8 min)
{
	int i;
	u8 *rate;

	ppr_for_each_entry(ppr, i, rate) {
		*rate = max(*rate, min);
	}
}

u8 b43_ppr_get_max(struct b43_wldev *dev, struct b43_ppr *ppr)
{
	u8 res = 0;
	int i;
	u8 *rate;

	ppr_for_each_entry(ppr, i, rate) {
		res = max(*rate, res);
	}

	return res;
}

/*
 * SPROM rev 9 has an offset for each CCK, legacy OFDM and MCS 0-7 rate in
 * each band, in 0.5 dB steps below the maximum power. The MCS offsets
 * apply to every way of sending the MCS (SISO, CDD, STBC and SDM).
 */
static bool b43_ppr_load_max_from_sprom_r9(struct b43_wldev *dev,
					   struct b43_ppr *ppr,
					   enum b43_band band, u8 maxpwr)
{
	struct b43_ppr_rates *rates = &ppr->rates;
	struct ssb_sprom *sprom = dev->dev->bus_sprom;
	u32 ofdm_po, mcs_po;
	int i;

	switch (band) {
	case B43_BAND_2G:
		ofdm_po = sprom->legofdmbw202gpo;
		mcs_po = sprom->mcsbw202gpo;
		break;
	case B43_BAND_5G_LO:
		ofdm_po = sprom->legofdmbw205glpo;
		mcs_po = sprom->mcsbw205glpo;
		break;
	case B43_BAND_5G_MI:
		ofdm_po = sprom->legofdmbw205gmpo;
		mcs_po = sprom->mcsbw205gmpo;
		break;
	case B43_BAND_5G_HI:
		ofdm_po = sprom->legofdmbw205ghpo;
		mcs_po = sprom->mcsbw205ghpo;
		break;
	default:
		return false;
	}

	if (band == B43_BAND_2G) {
		for (i = 0; i < 4; i++)
			rates->cck[i] = maxpwr -
					((sprom->cckbw202gpo >> (i * 4)) & 0xf) * 2;
	}

	for (i = 0; i < 8; i++) {
		u8 ofdm = maxpwr - ((ofdm_po >> (i * 4)) & 0xf) * 2;
		u8 mcs = maxpwr - ((mcs_po >> (i * 4)) & 0xf) * 2;

		rates->ofdm[i] = ofdm;
		rates->ofdm_20_cdd[i] = ofdm;
		rates->mcs_20[i] = mcs;
		rates->mcs_20_cdd[i] = mcs;
		rates->mcs_20_stbc[i] = mcs;
		rates->mcs_20_sdm[i] = mcs;
	}

	return true;
}

bool b43_ppr_load_max_from_sprom(struct b43_wldev *dev, struct b43_ppr *ppr,
				 enum b43_band band)
{
	struct b43_ppr_rates *rates = &ppr->rates;
	struct ssb_sprom *sprom = dev->dev->bus_sprom;
	struct b43_phy *phy = &dev->phy;
	u8 maxpwr, off;
	u32 sprom_ofdm_po;
	u16 *sprom_mcs_po;
	u8 extra_cdd_po, extra_stbc_po;
	int i;

	switch (band) {
	case B43_BAND_2G:
		maxpwr = min(sprom->core_pwr_info[0].maxpwr_2g,
			     sprom->core_pwr_info[1].maxpwr_2g);
		sprom_ofdm_po = sprom->ofdm2gpo;
		sprom_mcs_po = sprom->mcs2gpo;
		extra_cdd_po = (sprom->cddpo >> 0) & 0xf;
		extra_stbc_po = (sprom->stbcpo >> 0) & 0xf;
		break;
	case B43_BAND_5G_LO:
		maxpwr = min(sprom->core_pwr_info[0].maxpwr_5gl,
			     sprom->core_pwr_info[1].maxpwr_5gl);
		sprom_ofdm_po = sprom->ofdm5glpo;
		sprom_mcs_po = sprom->mcs5glpo;
		extra_cdd_po = (sprom->cddpo >> 8) & 0xf;
		extra_stbc_po = (sprom->stbcpo >> 8) & 0xf;
		break;
	case B43_BAND_5G_MI:
		maxpwr = min(sprom->core_pwr_info[0].maxpwr_5g,
			     sprom->core_pwr_info[1].maxpwr_5g);
		sprom_ofdm_po = sprom->ofdm5gpo;
		sprom_mcs_po = sprom->mcs5gpo;
		extra_cdd_po = (sprom->cddpo >> 4) & 0xf;
		extra_stbc_po = (sprom->stbcpo >> 4) & 0xf;
		break;
	case B43_BAND_5G_HI:
		maxpwr = min(sprom->core_pwr_info[0].maxpwr_5gh,
			     sprom->core_pwr_info[1].maxpwr_5gh);
		sprom_ofdm_po = sprom->ofdm5ghpo;
		sprom_mcs_po = sprom->mcs5ghpo;
		extra_cdd_po = (sprom->cddpo >> 12) & 0xf;
		extra_stbc_po = (sprom->stbcpo >> 12) & 0xf;
		break;
	default:
		WARN_ON_ONCE(1);
		return false;
	}

	/* Rev 9 replaced the offsets read above by per rate ones */
	if (sprom->revision >= 9)
		return b43_ppr_load_max_from_sprom_r9(dev, ppr, band, maxpwr);

	if (band == B43_BAND_2G) {
		for (i = 0; i < 4; i++) {
			off = ((sprom->cck2gpo >> (i * 4)) & 0xf) * 2;
			rates->cck[i] = maxpwr - off;
		}
	}

	/* OFDM */
	for (i = 0; i < 8; i++) {
		off = ((sprom_ofdm_po >> (i * 4)) & 0xf) * 2;
		rates->ofdm[i] = maxpwr - off;
	}

	/* MCS 20 SISO */
	rates->mcs_20[0] = rates->ofdm[0];
	rates->mcs_20[1] = rates->ofdm[2];
	rates->mcs_20[2] = rates->ofdm[3];
	rates->mcs_20[3] = rates->ofdm[4];
	rates->mcs_20[4] = rates->ofdm[5];
	rates->mcs_20[5] = rates->ofdm[6];
	rates->mcs_20[6] = rates->ofdm[7];
	rates->mcs_20[7] = rates->ofdm[7];

	/* MCS 20 CDD */
	for (i = 0; i < 4; i++) {
		off = ((sprom_mcs_po[0] >> (i * 4)) & 0xf) * 2;
		rates->mcs_20_cdd[i] = maxpwr - off;
		if (phy->type == B43_PHYTYPE_N && phy->rev >= 3)
			rates->mcs_20_cdd[i] -= extra_cdd_po;
	}
	for (i = 0; i < 4; i++) {
		off = ((sprom_mcs_po[1] >> (i * 4)) & 0xf) * 2;
		rates->mcs_20_cdd[4 + i] = maxpwr - off;
		if (phy->type == B43_PHYTYPE_N && phy->rev >= 3)
			rates->mcs_20_cdd[4 + i] -= extra_cdd_po;
	}

	/* OFDM 20 CDD */
	rates->ofdm_20_cdd[0] = rates->mcs_20_cdd[0];
	rates->ofdm_20_cdd[1] = rates->mcs_20_cdd[0];
	rates->ofdm_20_cdd[2] = rates->mcs_20_cdd[1];
	rates->ofdm_20_cdd[3] = rates->mcs_20_cdd[2];
	rates->ofdm_20_cdd[4] = rates->mcs_20_cdd[3];
	rates->ofdm_20_cdd[5] = rates->mcs_20_cdd[4];
	rates->ofdm_20_cdd[6] = rates->mcs_20_cdd[5];
	rates->ofdm_20_cdd[7] = rates->mcs_20_cdd[6];

	/* MCS 20 STBC */
	for (i = 0; i < 4; i++) {
		off = ((sprom_mcs_po[0] >> (i * 4)) & 0xf) * 2;
		rates->mcs_20_stbc[i] = maxpwr - off;
		if (phy->type == B43_PHYTYPE_N && phy->rev >= 3)
			rates->mcs_20_stbc[i] -= extra_stbc_po;
	}
	for (i = 0; i < 4; i++) {
		off = ((sprom_mcs_po[1] >> (i * 4)) & 0xf) * 2;
		rates->mcs_20_stbc[4 + i] = maxpwr - off;
		if (phy->type == B43_PHYTYPE_N && phy->rev >= 3)
			rates->mcs_20_stbc[4 + i] -= extra_stbc_po;
	}

	/* MCS 20 SDM */
	for (i = 0; i < 4; i++) {
		off = ((sprom_mcs_po[2] >> (i * 4)) & 0xf) * 2;
		rates->mcs_20_sdm[i] = maxpwr - off;
	}
	for (i = 0; i < 4; i++) {
		off = ((sprom_mcs_po[3] >> (i * 4)) & 0xf) * 2;
		rates->mcs_20_sdm[4 + i] = maxpwr - off;
	}

	return true;
}
