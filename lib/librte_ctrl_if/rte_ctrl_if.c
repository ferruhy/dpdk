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

#include <string.h>
#include <unistd.h>

#include <sys/socket.h>
#include <linux/rtnetlink.h>

#include <rte_log.h>
#include "rte_ctrl_if.h"
#include "rte_nl.h"

#define NAMESZ 32
#define IFNAME "dpdk"
#define BUFSZ 1024

static int unci_rtnl_fd = -1;
static uint32_t unci_ref_cnt;

struct unci_request {
	struct nlmsghdr nlmsg;
	uint8_t buf[BUFSZ];
};

static int
control_interface_rtnl_init(void)
{
	struct sockaddr_nl src;
	int ret;

	unci_rtnl_fd = socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
	if (unci_rtnl_fd < 0) {
		RTE_LOG(ERR, CTRL_IF, "Socket create failed\n");
		return -1;
	}

	memset(&src, 0, sizeof(struct sockaddr_nl));

	src.nl_family = AF_NETLINK;
	src.nl_pid = getpid();

	ret = bind(unci_rtnl_fd, (struct sockaddr *)&src,
			sizeof(struct sockaddr_nl));
	if (ret < 0) {
		RTE_LOG(ERR, CTRL_IF, "Bind to socket failed\n");
		return -1;
	}

	return 0;
}

static int
control_interface_init(void)
{
	int ret;

	ret = control_interface_rtnl_init();
	if (ret < 0) {
		RTE_LOG(ERR, CTRL_IF, "Failed to initialize rtnetlink\n");
		return -1;
	}

	ret = control_interface_nl_init();
	if (ret < 0) {
		RTE_LOG(ERR, CTRL_IF, "Failed to initialize netlink\n");
		close(unci_rtnl_fd);
		unci_rtnl_fd = -1;
	}

	return ret;
}

static int
control_interface_ref_get(void)
{
	int ret = 0;

	if (unci_ref_cnt == 0)
		ret = control_interface_init();

	if (ret == 0)
		unci_ref_cnt++;
	else
		RTE_LOG(ERR, CTRL_IF,
				"Failed to initialize control interface\n");

	return unci_ref_cnt;
}

static void
control_interface_release(void)
{
	close(unci_rtnl_fd);
	unci_rtnl_fd = -1;

	control_interface_nl_release();
}

static int
control_interface_ref_put(void)
{
	if (unci_ref_cnt == 0)
		return 0;

	unci_ref_cnt--;

	if (unci_ref_cnt == 0)
		control_interface_release();

	return unci_ref_cnt;
}

static int
add_attr(struct unci_request *req, uint16_t type, void *buf, size_t len)
{
	struct rtattr *rta;
	int nlmsg_len;

	nlmsg_len = NLMSG_ALIGN(req->nlmsg.nlmsg_len);
	rta = (struct rtattr *)((char *)&req->nlmsg + nlmsg_len);
	if (nlmsg_len + RTA_LENGTH(len) > sizeof(struct unci_request))
		return -1;
	rta->rta_type = type;
	rta->rta_len = RTA_LENGTH(len);
	memcpy(RTA_DATA(rta), buf, len);
	req->nlmsg.nlmsg_len = nlmsg_len + RTA_LENGTH(len);

	return 0;
}

static struct
rtattr *add_attr_nested(struct unci_request *req, unsigned short type)
{
	struct rtattr *rta;
	uint32_t nlmsg_len;

	nlmsg_len = NLMSG_ALIGN(req->nlmsg.nlmsg_len);
	rta = (struct rtattr *)((uint8_t *)&req->nlmsg + nlmsg_len);
	if (nlmsg_len + RTA_LENGTH(0) > sizeof(struct unci_request))
		return NULL;
	rta->rta_type = type;
	rta->rta_len = nlmsg_len;
	req->nlmsg.nlmsg_len = nlmsg_len + RTA_LENGTH(0);

	return rta;
}

static void
end_attr_nested(struct unci_request *req, struct rtattr *rta)
{
	rta->rta_len = req->nlmsg.nlmsg_len - rta->rta_len;
}

static int
rte_eth_rtnl_create(uint8_t port_id)
{
	struct unci_request req;
	struct ifinfomsg *info;
	struct rtattr *rta1;
	struct rtattr *rta2;
	uint32_t pid = getpid();
	char name[NAMESZ];
	char type[NAMESZ];
	struct iovec iov;
	struct msghdr msg;
	struct sockaddr_nl nladdr;
	uint8_t buf[BUFSZ];
	uint32_t port_id_local = port_id;
	int ret;

	memset(&req, 0, sizeof(struct unci_request));

	req.nlmsg.nlmsg_len = NLMSG_LENGTH(sizeof(struct ifinfomsg));
	req.nlmsg.nlmsg_flags = NLM_F_REQUEST | NLM_F_CREATE | NLM_F_EXCL;
	req.nlmsg.nlmsg_flags |= NLM_F_ACK;
	req.nlmsg.nlmsg_type = RTM_NEWLINK;

	info = NLMSG_DATA(&req.nlmsg);

	info->ifi_family = AF_UNSPEC;
	info->ifi_index = 0;

	snprintf(name, NAMESZ, IFNAME"%u", port_id);
	ret = add_attr(&req, IFLA_IFNAME, name, strlen(name) + 1);
	if (ret < 0)
		return -1;

	rta1 = add_attr_nested(&req, IFLA_LINKINFO);
	if (rta1 == NULL)
		return -1;

	snprintf(type, NAMESZ, UNCI_DEVICE);
	ret = add_attr(&req, IFLA_INFO_KIND, type, strlen(type) + 1);
	if (ret < 0)
		return -1;

	rta2 = add_attr_nested(&req, IFLA_INFO_DATA);
	if (rta2 == NULL)
		return -1;

	ret = add_attr(&req, IFLA_UNCI_PORTID, &port_id_local,
			sizeof(uint32_t));
	if (ret < 0)
		return -1;

	ret = add_attr(&req, IFLA_UNCI_PID, &pid, sizeof(uint32_t));
	if (ret < 0)
		return -1;

	end_attr_nested(&req, rta2);
	end_attr_nested(&req, rta1);

	memset(&nladdr, 0, sizeof(nladdr));
	nladdr.nl_family = AF_NETLINK;

	iov.iov_base = (void *)&req.nlmsg;
	iov.iov_len = req.nlmsg.nlmsg_len;

	memset(&msg, 0, sizeof(struct msghdr));
	msg.msg_name = &nladdr;
	msg.msg_namelen = sizeof(nladdr);
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;

	ret = sendmsg(unci_rtnl_fd, &msg, 0);
	if (ret < 0) {
		RTE_LOG(ERR, CTRL_IF, "Send for create failed %d.\n", errno);
		return -1;
	}

	memset(buf, 0, sizeof(buf));
	iov.iov_base = buf;
	iov.iov_len = sizeof(buf);

	ret = recvmsg(unci_rtnl_fd, &msg, 0);
	if (ret < 0) {
		RTE_LOG(ERR, CTRL_IF, "Recv for create failed\n");
		return -1;
	}

	return 0;
}

int
rte_eth_control_interface_create(uint8_t port_id)
{
	int ret;

	if (control_interface_ref_get() != 0) {
		ret = rte_eth_rtnl_create(port_id);
		RTE_LOG(DEBUG, CTRL_IF,
			"Control interface %s for port:%u\n",
			ret < 0 ? "failed" : "created", port_id);
	}

	return 0;
}

static int
rte_eth_rtnl_destroy(uint8_t port_id)
{
	struct unci_request req;
	struct ifinfomsg *info;
	char name[NAMESZ];
	struct iovec iov;
	struct msghdr msg;
	struct sockaddr_nl nladdr;
	int ret;

	memset(&req, 0, sizeof(struct unci_request));

	req.nlmsg.nlmsg_len = NLMSG_LENGTH(sizeof(struct ifinfomsg));
	req.nlmsg.nlmsg_flags = NLM_F_REQUEST;
	req.nlmsg.nlmsg_type = RTM_DELLINK;

	info = NLMSG_DATA(&req.nlmsg);

	info->ifi_family = AF_UNSPEC;
	info->ifi_index = 0;

	snprintf(name, NAMESZ, IFNAME"%u", port_id);
	ret = add_attr(&req, IFLA_IFNAME, name, strlen(name) + 1);
	if (ret < 0)
		return -1;

	memset(&nladdr, 0, sizeof(nladdr));
	nladdr.nl_family = AF_NETLINK;

	iov.iov_base = (void *)&req.nlmsg;
	iov.iov_len = req.nlmsg.nlmsg_len;

	memset(&msg, 0, sizeof(struct msghdr));
	msg.msg_name = &nladdr;
	msg.msg_namelen = sizeof(nladdr);
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;

	ret = sendmsg(unci_rtnl_fd, &msg, 0);
	if (ret < 0) {
		RTE_LOG(ERR, CTRL_IF, "Send for destroy failed\n");
		return -1;
	}
	return 0;
}

int
rte_eth_control_interface_destroy(uint8_t port_id)
{
	rte_eth_rtnl_destroy(port_id);
	control_interface_ref_put();
	RTE_LOG(DEBUG, CTRL_IF, "Control interface destroyed for port:%u\n",
			port_id);

	return 0;
}
