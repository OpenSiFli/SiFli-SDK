/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ****************************************************************************************
 *
 * @file rwnx_ieee80211.h
 *
 * @brief ieee80211 struct description
 *
 * @data 2023
 *
 ****************************************************************************************
 */

#ifndef _RWNX_IEEE80211_H_
#define _RWNX_IEEE80211_H_

//nl80211_channel_type
enum aic_channel_type {
	_80211_CHAN_NO_HT,
	_80211_CHAN_HT20,
	_80211_CHAN_HT40MINUS,
	_80211_CHAN_HT40PLUS
};

//nl80211_band
enum aic_80211_band {
	_80211_BAND_2GHZ,
	_80211_BAND_5GHZ,
	_80211_BAND_60GHZ,
	_80211_BAND_6GHZ,
	NUM_80211_BANDS,
};

//ieee80211_channel
struct ieee80211_channel {
	enum aic_80211_band band;
	u32 center_freq;
	u16 hw_value;
	u32 flags;
	int max_antenna_gain;
	int max_power;
	int max_reg_power;
	bool beacon_found;
	u32 orig_flags;
	int orig_mag, orig_mpwr;
	//enum nl80211_dfs_state dfs_state;
	unsigned long dfs_state_entered;
	unsigned int dfs_cac_ms;
};

//nl80211_chan_width
enum aic_80211_chan_width {
	_80211_CHAN_WIDTH_20_NOHT,
	_80211_CHAN_WIDTH_20,
	_80211_CHAN_WIDTH_40,
	_80211_CHAN_WIDTH_80,
	_80211_CHAN_WIDTH_80P80,
	_80211_CHAN_WIDTH_160,
	_80211_CHAN_WIDTH_5,
	_80211_CHAN_WIDTH_10,
};

//cfg80211_chan_def
struct aic_80211_chan_def {
	struct ieee80211_channel *chan;
	enum aic_80211_chan_width width;
	u32 center_freq1;
	u32 center_freq2;
};

void aic_80211_chandef_create(struct aic_80211_chan_def *chandef,
                             struct ieee80211_channel *chan,
                             enum aic_channel_type chan_type);


#endif /* _RWNX_IEEE80211_H_ */

