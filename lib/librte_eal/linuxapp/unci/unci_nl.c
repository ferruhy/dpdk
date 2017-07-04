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
 *   The full GNU General Public License is included in this distribution
 *   in the file called LICENSE.GPL.
 *
 *   Contact Information:
 *   Intel Corporation
 */

#include <net/genetlink.h>
#include <net/sock.h>

#include "unci_dev.h"

static int unci_genl_process(struct sk_buff *skb, struct genl_info *info)
{
	struct nlattr **attrs = info->attrs;
	struct unci_nl_msg nl_msg;

	if (!attrs[UNCI_ATTR_MSG])
		return -EINVAL;

	nla_memcpy(&nl_msg, attrs[UNCI_ATTR_MSG], sizeof(struct unci_nl_msg));
	pr_debug("cmd: %u\n", nl_msg.cmd_id);

	return 0;
}

static struct nla_policy unci_genl_policy[UNCI_ATTR_MAX + 1] = {
	[UNCI_ATTR_MSG] = { .type = NLA_BINARY, .len = UNCI_GENL_MSG_LEN },
};

static const struct genl_ops unci_ops[] = {
	{
		.cmd = UNCI_CMD_MSG,
		.doit = unci_genl_process,
		.policy = unci_genl_policy,
	},
};

static struct genl_family unci_genl_family __ro_after_init = {
	.module = THIS_MODULE,
	.name = UNCI_DEVICE,
	.version = UNCI_GENL_VERSION,
	.maxattr = UNCI_ATTR_MAX,
	.ops = unci_ops,
	.n_ops = ARRAY_SIZE(unci_ops),
};

int unci_nl_init(void)
{
	return genl_register_family(&unci_genl_family);
}

void unci_nl_release(void)
{
	genl_unregister_family(&unci_genl_family);
}
