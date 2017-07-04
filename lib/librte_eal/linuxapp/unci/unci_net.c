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

#include <linux/module.h>
#include <net/rtnetlink.h>

#include "unci_dev.h"

static const struct net_device_ops unci_net_netdev_ops = { 0 };

static void unci_net_setup(struct net_device *dev)
{
	struct unci_dev *unci;

	ether_setup(dev);
	dev->netdev_ops = &unci_net_netdev_ops;

	unci = netdev_priv(dev);
	init_completion(&unci->msg_received);
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
