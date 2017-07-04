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

#ifndef _UNCI_DEV_H_
#define _UNCI_DEV_H_

#include <linux/netdevice.h>

#include <exec-env/unci.h>

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

struct unci_dev {
	__u32 port_id;
	__u32 pid;
	struct completion msg_received;
	u32 nb_timedout_msg;
};

int unci_nl_init(void);
void unci_nl_release(void);
int unci_nl_exec(u32 cmd, struct net_device *dev, void *in_data,
		size_t in_len, void *out_data, size_t out_len);

void unci_set_ethtool_ops(struct net_device *netdev);

#endif /* _UNCI_DEV_H_ */
