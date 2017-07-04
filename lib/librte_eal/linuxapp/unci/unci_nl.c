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

#define UNCI_GENL_MSG_LEN 1536

#define UNCI_CMD_TIMEOUT 500 /* ms */

static struct response_buffer {
	int magic; /* for sanity check */
	void *buffer;
	size_t length;
	struct completion *msg_received;
	int *err;
	u32 in_use;
} response_buffer;

static DEFINE_MUTEX(sync_lock);

static int unci_response_buffer_register(int magic, void *buffer, size_t length,
		struct completion *msg_received, int *err)
{
	if (!response_buffer.in_use) {
		response_buffer.magic = magic;
		response_buffer.buffer = buffer;
		response_buffer.length = length;
		response_buffer.msg_received = msg_received;
		response_buffer.err = err;
		response_buffer.in_use = 1;
		return 0;
	}

	return 1;
}

static void unci_response_buffer_unregister(int magic)
{
	if (response_buffer.in_use) {
		if (magic == response_buffer.magic) {
			response_buffer.magic = -1;
			response_buffer.buffer = NULL;
			response_buffer.length = 0;
			response_buffer.msg_received = NULL;
			response_buffer.err = NULL;
			response_buffer.in_use = 0;
		} else {
			pr_err("Unregister magic mismatch\n");
		}
	}
}

static void nl_recv_user_request(struct unci_nl_msg *nl_msg)
{
	/* Userspace requests not supported yet */
	pr_debug("Request from userspace received\n");
}

static void nl_recv_user_response(struct unci_nl_msg *nl_msg)
{
	struct completion *msg_received;
	size_t recv_len;
	size_t expected_len;

	if (response_buffer.in_use) {
		if (response_buffer.buffer != NULL) {
			recv_len = nl_msg->output_buffer_len;
			expected_len = response_buffer.length;

			memcpy(response_buffer.buffer,
					nl_msg->output_buffer,
					response_buffer.length);

			if (nl_msg->err == 0 && recv_len != expected_len)
				pr_info("Expected and received len not match "
					"%zu - %zu\n", recv_len, expected_len);
		}

		*response_buffer.err = nl_msg->err;
		msg_received = response_buffer.msg_received;
		unci_response_buffer_unregister(response_buffer.magic);
		complete(msg_received);
	}
}

static struct genl_family unci_genl_family;

static int unci_nl_send(u32 cmd_id, u32 port_id, u32 pid, void *in_data,
		size_t in_data_len)
{
	struct unci_nl_msg nl_msg;
	struct sk_buff *skb;
	void *payload;
	u32 size = 0;

	if (pid == 0)
		return -1;

	memset(&nl_msg, 0, sizeof(struct unci_nl_msg));
	nl_msg.cmd_id = cmd_id;
	nl_msg.port_id = port_id;

	if (in_data) {
		if (in_data_len == 0 || in_data_len > UNCI_NL_MSG_LEN)
			return -EINVAL;
		nl_msg.input_buffer_len = in_data_len;
		memcpy(nl_msg.input_buffer, in_data, in_data_len);
	}

	skb = genlmsg_new(UNCI_GENL_MSG_LEN, GFP_ATOMIC);
	if (!skb)
		return -ENOMEM;

	size = sizeof(struct unci_nl_msg) + GENL_HDRLEN +
		unci_genl_family.hdrsize;

	payload = genlmsg_put(skb, 0, 0, &unci_genl_family, 0, UNCI_CMD_MSG);
	if (payload == NULL) {
		nlmsg_free(skb);
		return -EMSGSIZE;
	}

	nla_put(skb, UNCI_ATTR_MSG, sizeof(struct unci_nl_msg), &nl_msg);

	genlmsg_end(skb, payload);

	genlmsg_unicast(&init_net, skb, pid);
	pr_debug("Sent cmd:%u port:%u pid:%u\n", cmd_id, port_id, pid);

	return 0;
}

int unci_nl_exec(u32 cmd, struct net_device *dev, void *in_data,
		size_t in_data_len, void *out_data, size_t out_data_len)
{
	struct unci_dev *unci = netdev_priv(dev);
	int err = -EINVAL;
	int ret;

	if (out_data_len > UNCI_NL_MSG_LEN) {
		pr_err("Message is too big to receive:%zu\n", out_data_len);
		return err;
	}

	mutex_lock(&sync_lock);
	ret = unci_response_buffer_register(cmd, out_data, out_data_len,
			&unci->msg_received, &err);
	if (ret) {
		mutex_unlock(&sync_lock);
		return -EINVAL;
	}

	ret = unci_nl_send(cmd, unci->port_id, unci->pid, in_data, in_data_len);
	if (ret) {
		unci_response_buffer_unregister(response_buffer.magic);
		mutex_unlock(&sync_lock);
		return ret;
	}

	ret = wait_for_completion_interruptible_timeout(&unci->msg_received,
			 msecs_to_jiffies(UNCI_CMD_TIMEOUT));
	if (ret == 0 || err < 0) {
		unci_response_buffer_unregister(response_buffer.magic);
		mutex_unlock(&sync_lock);
		if (ret == 0) { /* timeout */
			unci->nb_timedout_msg++;
			pr_info("Command timed-out for port:%u cmd:%u (%u)\n",
				unci->port_id, cmd, unci->nb_timedout_msg);
			return -EINVAL;
		}
		pr_debug("Command return error for port:%d cmd:%d err:%d\n",
				unci->port_id, cmd, err);
		return err;
	}
	mutex_unlock(&sync_lock);

	return 0;
}

static int unci_genl_process(struct sk_buff *skb, struct genl_info *info)
{
	struct nlattr **attrs = info->attrs;
	struct unci_nl_msg nl_msg;

	if (!attrs[UNCI_ATTR_MSG])
		return -EINVAL;

	nla_memcpy(&nl_msg, attrs[UNCI_ATTR_MSG], sizeof(struct unci_nl_msg));
	pr_debug("cmd: %u\n", nl_msg.cmd_id);

	if (nl_msg.flag & UNCI_MSG_FLAG_REQUEST) {
		nl_recv_user_request(&nl_msg);
		return 0;
	}

	nl_recv_user_response(&nl_msg);
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
