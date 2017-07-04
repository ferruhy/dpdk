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

#include <linux/etherdevice.h>
#include <linux/module.h>
#include <linux/version.h>
#include <net/rtnetlink.h>

#include "unci_dev.h"

static int unci_net_init(struct net_device *dev)
{
	u8 mac[ETH_ALEN] = {0};

	unci_nl_exec(UNCI_REQ_GET_MAC, dev, NULL, 0, mac, ETH_ALEN);
	memcpy(dev->dev_addr, mac, dev->addr_len);
	return 0;
}

static int unci_net_open(struct net_device *dev)
{
	/* DPDK port already started, stop it first */
	unci_nl_exec(UNCI_REQ_STOP_PORT, dev, NULL, 0, NULL, 0);
	unci_nl_exec(UNCI_REQ_START_PORT, dev, NULL, 0, NULL, 0);
	netif_start_queue(dev);
	return 0;
}

static int unci_net_close(struct net_device *dev)
{
	unci_nl_exec(UNCI_REQ_STOP_PORT, dev, NULL, 0, NULL, 0);
	netif_stop_queue(dev);
	return 0;
}

static int unci_net_xmit(struct sk_buff *skb, struct net_device *dev)
{
	dev_kfree_skb(skb);
	return NETDEV_TX_OK;
}

static void unci_net_change_rx_flags(struct net_device *dev, int flags)
{
	u32 on = 1;
	u32 off = 0;

	if (flags & IFF_PROMISC)
		unci_nl_exec(UNCI_REQ_SET_PROMISC, dev,
				dev->flags & IFF_PROMISC ?  &on : &off,
				sizeof(u32), NULL, 0);

	if (flags & IFF_ALLMULTI)
		unci_nl_exec(UNCI_REQ_SET_ALLMULTI, dev,
				dev->flags & IFF_ALLMULTI ?  &on : &off,
				sizeof(u32), NULL, 0);
}

static int unci_net_set_mac(struct net_device *dev, void *p)
{
	struct sockaddr *addr = p;
	int err;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	err = unci_nl_exec(UNCI_REQ_SET_MAC, dev, addr->sa_data,
			dev->addr_len, NULL, 0);
	if (err < 0)
		return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

	return 0;
}

static int unci_net_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	return -EOPNOTSUPP;
}

/*
 * Configuration changes (passed on by ifconfig)
 */
static int unci_net_config(struct net_device *dev, struct ifmap *map)
{
	if (dev->flags & IFF_UP)
		return -EBUSY;

	return -EOPNOTSUPP;
}

static int unci_net_change_mtu(struct net_device *dev, int new_mtu)
{
	int err = 0;

	err = unci_nl_exec(UNCI_REQ_CHANGE_MTU, dev, &new_mtu, sizeof(int),
			NULL, 0);

	if (err == 0)
		dev->mtu = new_mtu;

	return err;
}

static void unci_net_stats64(struct net_device *dev,
		struct rtnl_link_stats64 *stats)
{
	int err;

	err = unci_nl_exec(UNCI_REQ_GET_STATS, dev, NULL, 0,
			stats, sizeof(struct rtnl_link_stats64));
}

#if (KERNEL_VERSION(3, 9, 0) <= LINUX_VERSION_CODE)
static int unci_net_change_carrier(struct net_device *dev, bool new_carrier)
{
	if (new_carrier)
		netif_carrier_on(dev);
	else
		netif_carrier_off(dev);
	return 0;
}
#endif

static const struct net_device_ops unci_net_netdev_ops = {
	.ndo_init = unci_net_init,
	.ndo_open = unci_net_open,
	.ndo_stop = unci_net_close,
	.ndo_start_xmit = unci_net_xmit,
	.ndo_change_rx_flags = unci_net_change_rx_flags,
	.ndo_set_mac_address = unci_net_set_mac,
	.ndo_do_ioctl = unci_net_ioctl,
	.ndo_set_config = unci_net_config,
	.ndo_change_mtu = unci_net_change_mtu,
	.ndo_get_stats64 = unci_net_stats64,
#if (KERNEL_VERSION(3, 9, 0) <= LINUX_VERSION_CODE)
	.ndo_change_carrier = unci_net_change_carrier,
#endif
};

static void unci_net_setup(struct net_device *dev)
{
	struct unci_dev *unci;

	ether_setup(dev);
	dev->netdev_ops = &unci_net_netdev_ops;

	unci = netdev_priv(dev);
	init_completion(&unci->msg_received);

	unci_set_ethtool_ops(dev);
}

static int unci_net_newlink(struct net *net, struct net_device *dev,
		struct nlattr *tb[], struct nlattr *data[])
{
	struct unci_dev *unci = netdev_priv(dev);

	unci->port_id = nla_get_u32(data[IFLA_UNCI_PORTID]);
	unci->pid = nla_get_u32(data[IFLA_UNCI_PID]);

	return register_netdevice(dev);
}

static const struct nla_policy unci_policy[IFLA_UNCI_MAX + 1] = {
	[IFLA_UNCI_PORTID] = { .type = NLA_U32 },
	[IFLA_UNCI_PID]    = { .type = NLA_U32 },
};

static int unci_validate(struct nlattr *tb[], struct nlattr *data[])
{
	if (!data)
		return -EINVAL;

	if (!data[IFLA_UNCI_PID] || !data[IFLA_UNCI_PORTID])
		return -EINVAL;

	return 0;
}

static struct rtnl_link_ops unci_link_ops __read_mostly = {
	.kind = UNCI_DEVICE,
	.priv_size = sizeof(struct unci_dev),
	.maxtype = IFLA_UNCI_MAX,
	.setup = unci_net_setup,
	.newlink = unci_net_newlink,
	.policy = unci_policy,
	.validate = unci_validate,
};

static int __init unci_init(void)
{
	int ret;

	ret = unci_nl_init();
	if (ret)
		return ret;

	return rtnl_link_register(&unci_link_ops);
}
module_init(unci_init);

static void __exit unci_exit(void)
{
	rtnl_link_unregister(&unci_link_ops);
	unci_nl_release();
}
module_exit(unci_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Kernel Module for managing unci devices");
MODULE_ALIAS_RTNL_LINK(UNCI_DEVICE);
