/*-
 * GPL LICENSE SUMMARY
 *
 *   Copyright(c) 2017 Intel Corporation. All rights reserved.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of version 2 of the GNU General Public License as
 *   published by the Free Software Foundation.
 *
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program;
 *
 *   The full GNU General Public License is included in this distribution
 *   in the file called LICENSE.GPL.
 *
 *   Contact Information:
 *   Intel Corporation
 */

#include "unci_dev.h"

#define ETHTOOL_GEEPROM_LEN 99
#define ETHTOOL_GREGS_LEN 98
#define ETHTOOL_GSSET_COUNT 97

static int unci_check_if_running(struct net_device *dev)
{
	return 0;
}

static void unci_get_drvinfo(struct net_device *dev,
		struct ethtool_drvinfo *info)
{
	int ret;

	ret = unci_nl_exec(info->cmd, dev, NULL, 0,
			info, sizeof(struct ethtool_drvinfo));
	if (ret < 0)
		memset(info, 0, sizeof(struct ethtool_drvinfo));
}

static int unci_get_settings(struct net_device *dev, struct ethtool_cmd *ecmd)
{
	return unci_nl_exec(ecmd->cmd, dev, NULL, 0,
			ecmd, sizeof(struct ethtool_cmd));
}

static int unci_set_settings(struct net_device *dev, struct ethtool_cmd *ecmd)
{
	return unci_nl_exec(ecmd->cmd, dev, ecmd, sizeof(struct ethtool_cmd),
			NULL, 0);
}

static void unci_get_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
	int ret;

	ret = unci_nl_exec(wol->cmd, dev, NULL, 0,
			wol, sizeof(struct ethtool_wolinfo));
	if (ret < 0)
		memset(wol, 0, sizeof(struct ethtool_wolinfo));
}

static int unci_set_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
	return unci_nl_exec(wol->cmd, dev, wol, sizeof(struct ethtool_wolinfo),
			NULL, 0);
}

static int unci_nway_reset(struct net_device *dev)
{
	return unci_nl_exec(ETHTOOL_NWAY_RST, dev, NULL, 0, NULL, 0);
}

static u32 unci_get_link(struct net_device *dev)
{
	u32 data;
	int ret;

	ret = unci_nl_exec(ETHTOOL_GLINK, dev, NULL, 0, &data, sizeof(u32));
	if (ret < 0)
		return 0;

	return data;
}

static int unci_get_eeprom_len(struct net_device *dev)
{
	int data;
	int ret;

	ret = unci_nl_exec(ETHTOOL_GEEPROM_LEN, dev, NULL, 0,
			&data, sizeof(int));
	if (ret < 0)
		return 0;

	return data;
}

static int unci_get_eeprom(struct net_device *dev,
		struct ethtool_eeprom *eeprom, u8 *data)
{
	struct ethtool_eeprom eeprom_tmp;
	int ret = 0;
	int remaining;
	u32 offset = 0;

	eeprom_tmp = *eeprom;

	remaining = eeprom_tmp.len;
	while (remaining > 0 && ret == 0) {
		eeprom_tmp.len = min(remaining, UNCI_NL_MSG_LEN);

		ret = unci_nl_exec(eeprom_tmp.cmd, dev,
				&eeprom_tmp, sizeof(struct ethtool_eeprom),
				data + offset, eeprom_tmp.len);
		eeprom_tmp.offset += eeprom_tmp.len;
		offset += eeprom_tmp.len;
		remaining -= eeprom_tmp.len;
	}

	return ret;
}

static int unci_set_eeprom(struct net_device *dev,
		struct ethtool_eeprom *eeprom, u8 *data)
{
	struct ethtool_eeprom *eeprom_tmp;
	int ret = 0;
	u32 remaining;
	u32 offset = 0;
	u32 payload;

	if (sizeof(struct ethtool_eeprom) > UNCI_NL_MSG_LEN)
		return -1;

	eeprom_tmp = kmalloc(UNCI_NL_MSG_LEN, GFP_KERNEL);
	payload = UNCI_NL_MSG_LEN - sizeof(struct ethtool_eeprom);

	*eeprom_tmp = *eeprom;
	remaining = eeprom->len;

	while (remaining > 0 && ret == 0) {
		eeprom_tmp->len = min(remaining, payload);

		memcpy(eeprom_tmp->data, data + offset, payload);

		ret = unci_nl_exec(eeprom->cmd, dev, eeprom,
				UNCI_NL_MSG_LEN, NULL, 0);

		eeprom_tmp->offset += eeprom_tmp->len;
		offset += eeprom_tmp->len;
		remaining -= eeprom_tmp->len;
	}

	kfree(eeprom_tmp);

	return ret;
}

static void unci_get_ringparam(struct net_device *dev,
		struct ethtool_ringparam *ring)
{
	unci_nl_exec(ring->cmd, dev, NULL, 0,
			ring, sizeof(struct ethtool_ringparam));
}

static int unci_set_ringparam(struct net_device *dev,
		struct ethtool_ringparam *ring)
{
	return unci_nl_exec(ring->cmd, dev, ring,
			sizeof(struct ethtool_ringparam), NULL, 0);
}

static void unci_get_pauseparam(struct net_device *dev,
		struct ethtool_pauseparam *pause)
{
	unci_nl_exec(pause->cmd, dev, NULL, 0,
			pause, sizeof(struct ethtool_pauseparam));
}

static int unci_set_pauseparam(struct net_device *dev,
		struct ethtool_pauseparam *pause)
{
	return unci_nl_exec(pause->cmd, dev, pause,
			sizeof(struct ethtool_pauseparam), NULL, 0);
}

static u32 unci_get_msglevel(struct net_device *dev)
{
	u32 data;
	int ret;

	ret = unci_nl_exec(ETHTOOL_GMSGLVL, dev, NULL, 0, &data, sizeof(u32));
	if (ret < 0)
		return 0;

	return data;
}

static void unci_set_msglevel(struct net_device *dev, u32 data)
{
	unci_nl_exec(ETHTOOL_SMSGLVL, dev, &data, sizeof(u32), NULL, 0);
}

static int unci_get_regs_len(struct net_device *dev)
{
	int data;
	int ret;

	ret = unci_nl_exec(ETHTOOL_GREGS_LEN, dev, NULL, 0, &data, sizeof(int));
	if (ret < 0)
		return 0;

	return data;
}

static void unci_get_regs(struct net_device *dev, struct ethtool_regs *regs,
		void *p)
{
	struct ethtool_regs regs_tmp;
	u32 len = regs->len;

	regs_tmp = *regs;

	if (len > UNCI_NL_MSG_LEN) {
		len = UNCI_NL_MSG_LEN;
		regs_tmp.len = len;
	}

	unci_nl_exec(regs->cmd, dev, &regs_tmp, sizeof(struct ethtool_regs),
			p, len);
}

static void unci_get_strings(struct net_device *dev, u32 stringset, u8 *data)
{
	unci_nl_exec(ETHTOOL_GSTRINGS, dev, &stringset, sizeof(u32), data, 0);
}

static int unci_get_sset_count(struct net_device *dev, int sset)
{
	int data;
	int ret;

	ret = unci_nl_exec(ETHTOOL_GSSET_COUNT, dev, &sset, sizeof(int),
			&data, sizeof(int));
	if (ret < 0)
		return 0;

	return data;
}

static void unci_get_ethtool_stats(struct net_device *dev,
		struct ethtool_stats *stats, u64 *data)
{
	unci_nl_exec(stats->cmd, dev, stats, sizeof(struct ethtool_stats),
			data, stats->n_stats);
}

static const struct ethtool_ops unci_ethtool_ops = {
	.begin			= unci_check_if_running,
	.get_drvinfo		= unci_get_drvinfo,
	.get_settings		= unci_get_settings,
	.set_settings		= unci_set_settings,
	.get_regs_len		= unci_get_regs_len,
	.get_regs		= unci_get_regs,
	.get_wol		= unci_get_wol,
	.set_wol		= unci_set_wol,
	.nway_reset		= unci_nway_reset,
	.get_link		= unci_get_link,
	.get_eeprom_len		= unci_get_eeprom_len,
	.get_eeprom		= unci_get_eeprom,
	.set_eeprom		= unci_set_eeprom,
	.get_ringparam		= unci_get_ringparam,
	.set_ringparam		= unci_set_ringparam,
	.get_pauseparam		= unci_get_pauseparam,
	.set_pauseparam		= unci_set_pauseparam,
	.get_msglevel		= unci_get_msglevel,
	.set_msglevel		= unci_set_msglevel,
	.get_strings		= unci_get_strings,
	.get_sset_count		= unci_get_sset_count,
	.get_ethtool_stats	= unci_get_ethtool_stats,
};

void unci_set_ethtool_ops(struct net_device *netdev)
{
	netdev->ethtool_ops = &unci_ethtool_ops;
}
