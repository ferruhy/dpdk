/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2017 Intel Corporation. All rights reserved.
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <pthread.h>
#include <string.h>
#include <unistd.h>

#include <sys/socket.h>
#include <linux/netlink.h>
#include <linux/genetlink.h>

#include <rte_log.h>
#include <rte_lcore.h>
#include "rte_ctrl_process.h"
#include "rte_nl.h"
#include "rte_ctrl_if.h"

#define GENLMSG_DATA(glh) (((char *)(NLMSG_DATA(glh)) + GENL_HDRLEN))
#define NLA_DATA(na) ((void *)((char *)(na) + NLA_HDRLEN))

struct ctrl_if_nl {
	union {
		struct hdr {
			struct nlmsghdr nlh;
			struct genlmsghdr genlh;
		} h;
		uint8_t nlmsg[UNCI_GENL_MSG_LEN];
	};
	struct msghdr msg;
	struct iovec iov;
	struct sockaddr_nl dest_addr;
};

struct ctrl_if_msg_sync {
	struct unci_nl_msg msg_storage;
	pthread_mutex_t msg_lock;
	uint32_t pending_process;
};


/**
 * Flags values for rte_eth_control_interface_process_msg() API
 */
enum control_interface_process_flag {
	/**< Process if msg available. */
	RTE_ETHTOOL_CTRL_IF_PROCESS_MSG,

	/**< Discard msg if available, respond with a error value. */
	RTE_ETHTOOL_CTRL_IF_DISCARD_MSG,
};

static int sock_fd = -1;
static pthread_t thread_id;

static struct ctrl_if_nl nl_s;
static struct ctrl_if_nl nl_r;

static int nl_family_id;

static struct ctrl_if_msg_sync ctrl_if_sync = {
	.msg_lock = PTHREAD_MUTEX_INITIALIZER,
};

static int
nl_send(void *buf, size_t len)
{
	struct nlattr *nl_na;
	int ret;

	nl_s.h.nlh.nlmsg_len = NLMSG_LENGTH(GENL_HDRLEN);
	nl_s.h.nlh.nlmsg_type = nl_family_id;
	nl_s.h.nlh.nlmsg_flags = NLM_F_REQUEST;
	nl_s.h.nlh.nlmsg_seq = 0;
	nl_s.h.nlh.nlmsg_pid = getpid();

	nl_s.h.genlh.cmd = UNCI_CMD_MSG;
	nl_s.h.genlh.version = UNCI_GENL_VERSION;
	nl_s.h.genlh.reserved = 0;

	nl_na = (struct nlattr *) GENLMSG_DATA(nl_s.nlmsg);
	nl_na->nla_type = UNCI_ATTR_MSG;
	nl_na->nla_len = len + NLA_HDRLEN;

	nl_s.h.nlh.nlmsg_len += NLMSG_ALIGN(nl_na->nla_len);

	if (nl_s.h.nlh.nlmsg_len > UNCI_GENL_MSG_LEN) {
		RTE_LOG(ERR, CTRL_IF, "Message is too big, len:%zu\n", len);
		return -1;
	}

	/* Fill in the netlink message payload */
	memcpy(NLA_DATA(nl_na), buf, len);

	ret = sendmsg(sock_fd, &nl_s.msg, 0);
	if (ret < 0)
		RTE_LOG(ERR, CTRL_IF, "Failed nl msg send. ret:%d, err:%d\n",
				ret, errno);
	return ret;
}

/* each request sent expects a reply */
static int
nl_reply(struct unci_nl_msg *msg)
{
	return nl_send((void *)msg, sizeof(struct unci_nl_msg));
}

static void
process_msg(struct unci_nl_msg *msg)
{
	if (msg->cmd_id > UNCI_REQ_UNKNOWN) {
		msg->err = rte_eth_dev_control_process(msg->cmd_id,
				msg->port_id, msg->input_buffer,
				msg->output_buffer, &msg->output_buffer_len);
	}

	if (msg->err)
		memset(msg->output_buffer, 0, msg->output_buffer_len);

	nl_reply(msg);
}

static int
control_interface_msg_process(uint32_t flag)
{
	struct unci_nl_msg msg_storage;
	int ret = 0;

	pthread_mutex_lock(&ctrl_if_sync.msg_lock);
	if (ctrl_if_sync.pending_process == 0) {
		pthread_mutex_unlock(&ctrl_if_sync.msg_lock);
		return 0;
	}

	memcpy(&msg_storage, &ctrl_if_sync.msg_storage,
			sizeof(struct unci_nl_msg));
	ctrl_if_sync.pending_process = 0;
	pthread_mutex_unlock(&ctrl_if_sync.msg_lock);

	switch (flag) {
	case RTE_ETHTOOL_CTRL_IF_PROCESS_MSG:
		process_msg(&msg_storage);
		break;

	case RTE_ETHTOOL_CTRL_IF_DISCARD_MSG:
		msg_storage.err = -1;
		nl_reply(&msg_storage);
		break;

	default:
		ret = -1;
		break;
	}

	return ret;
}

static int
msg_add_and_process(struct nlmsghdr *nlh)
{
	struct nlattr *nl_na;
	char *genlh;

	pthread_mutex_lock(&ctrl_if_sync.msg_lock);

	if (ctrl_if_sync.pending_process) {
		pthread_mutex_unlock(&ctrl_if_sync.msg_lock);
		return -1;
	}

	genlh = NLMSG_DATA(nlh);
	nl_na = (struct nlattr *) (genlh + GENL_HDRLEN);

	if (nl_na->nla_type != UNCI_ATTR_MSG) {
		pthread_mutex_unlock(&ctrl_if_sync.msg_lock);
		return -1;
	}

	memcpy(&ctrl_if_sync.msg_storage, NLA_DATA(nl_na),
			sizeof(struct unci_nl_msg));
	ctrl_if_sync.msg_storage.flag = UNCI_MSG_FLAG_RESPONSE;
	ctrl_if_sync.pending_process = 1;

	pthread_mutex_unlock(&ctrl_if_sync.msg_lock);

	control_interface_msg_process(RTE_ETHTOOL_CTRL_IF_PROCESS_MSG);

	return 0;
}

static void *
nl_recv(void *arg)
{
	int ret;

	for (;;) {
		ret = recvmsg(sock_fd, &nl_r.msg, 0);
		if (ret < 0)
			continue;

		if (nl_r.h.nlh.nlmsg_type == NLMSG_ERROR)
			continue;

		if ((unsigned int)ret < sizeof(struct unci_nl_msg)) {
			RTE_LOG(WARNING, CTRL_IF,
					"Received %d bytes, payload %zu\n",
					ret, sizeof(struct unci_nl_msg));
			continue;
		}

		msg_add_and_process(&nl_r.h.nlh);
	}

	return arg;
}

static void
nl_setup_header(struct ctrl_if_nl *nl)
{
	memset(nl, 0, sizeof(struct ctrl_if_nl));

	nl->dest_addr.nl_family = AF_NETLINK;

	/* Fill the netlink message header */
	nl->h.nlh.nlmsg_len = UNCI_GENL_MSG_LEN;
	nl->h.nlh.nlmsg_pid = getpid();  /* self pid */
	nl->h.nlh.nlmsg_flags = NLM_F_REQUEST;
	nl->h.nlh.nlmsg_type = nl_family_id;

	nl->h.genlh.cmd = UNCI_CMD_MSG;
	nl->h.genlh.version = UNCI_GENL_VERSION;

	nl->iov.iov_base = (void *)nl->nlmsg;
	nl->iov.iov_len = nl->h.nlh.nlmsg_len;

	nl->msg.msg_name = (void *)&nl->dest_addr;
	nl->msg.msg_namelen = sizeof(struct sockaddr_nl);
	nl->msg.msg_iov = &nl->iov;
	nl->msg.msg_iovlen = 1;
}

static int
unci_genl_family_get(int fd)
{
	struct nlattr *nl_na;
	int id = -1;
	int ret;

	nl_setup_header(&nl_s);
	nl_setup_header(&nl_r);

	nl_s.h.nlh.nlmsg_type = GENL_ID_CTRL;
	nl_s.h.nlh.nlmsg_flags = NLM_F_REQUEST;
	nl_s.h.nlh.nlmsg_len = NLMSG_LENGTH(GENL_HDRLEN);

	nl_s.h.genlh.cmd = CTRL_CMD_GETFAMILY;

	nl_na = (struct nlattr *) GENLMSG_DATA(&nl_s);
	nl_na->nla_type = CTRL_ATTR_FAMILY_NAME;
	nl_na->nla_len = strlen(UNCI_DEVICE) + 1 + NLA_HDRLEN;
	strcpy(NLA_DATA(nl_na), UNCI_DEVICE);

	nl_s.h.nlh.nlmsg_len += NLMSG_ALIGN(nl_na->nla_len);

	nl_s.iov.iov_len = nl_s.h.nlh.nlmsg_len;

	ret = sendmsg(fd, &nl_s.msg, 0);

	if ((uint32_t)ret != nl_s.h.nlh.nlmsg_len)
		return -1;

	ret = recvmsg(fd, &nl_r.msg, 0);
	if (ret < 0)
		return -1;

	if (!NLMSG_OK((&nl_r.h.nlh), (uint32_t)ret))
		return -1;

	if (nl_r.h.nlh.nlmsg_type == NLMSG_ERROR)
		return -1;

	nl_na = (struct nlattr *) GENLMSG_DATA(&nl_r);
	nl_na = (struct nlattr *) ((char *) nl_na + NLA_ALIGN(nl_na->nla_len));
	if (nl_na->nla_type == CTRL_ATTR_FAMILY_ID)
		id = *(__u16 *) NLA_DATA(nl_na);

	return id;
}

static int
nl_socket_init(void)
{
	struct sockaddr_nl src_addr;
	int fd;
	int ret;

	fd = socket(AF_NETLINK, SOCK_RAW, NETLINK_GENERIC);
	if (fd < 0)
		return -1;

	memset(&src_addr, 0, sizeof(struct sockaddr_nl));
	src_addr.nl_family = AF_NETLINK;
	src_addr.nl_pid = getpid();

	ret = bind(fd, (struct sockaddr *)&src_addr, sizeof(src_addr));
	if (ret) {
		close(fd);
		return -1;
	}

	nl_family_id = unci_genl_family_get(fd);
	if (!nl_family_id) {
		close(fd);
		return -1;
	}

	nl_setup_header(&nl_s);
	nl_setup_header(&nl_r);

	return fd;
}

int
control_interface_nl_init(void)
{
	int ret;

	sock_fd = nl_socket_init();
	if (sock_fd < 0) {
		RTE_LOG(ERR, CTRL_IF, "Failed to initialize netlink socket\n");
		return -1;
	}

	ret = pthread_create(&thread_id, NULL, nl_recv, NULL);
	if (ret != 0) {
		RTE_LOG(ERR, CTRL_IF, "Failed to create receive thread\n");
		return -1;
	}

	ret = rte_thread_setname(thread_id, "netlink-recv");
	if (ret != 0)
		RTE_LOG(ERR, CTRL_IF, "Failed to set thread name.\n");

	return 0;
}

void
control_interface_nl_release(void)
{
	pthread_cancel(thread_id);
	pthread_join(thread_id, NULL);
	close(sock_fd);
}
