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

#include <stdio.h>
#include <error.h>

#include <linux/if_link.h>

#include <rte_version.h>
#include <rte_ethdev.h>
#include "rte_ctrl_process.h"

static int
set_mtu(uint8_t port_id, void *in_data)
{
	int *mtu = in_data;

	return rte_eth_dev_set_mtu(port_id, *mtu);
}

static int
get_stats(uint8_t port_id, void *data, size_t *data_len)
{
	struct rte_eth_stats stats;
	struct rtnl_link_stats64 *if_stats = data;
	int ret;

	ret = rte_eth_stats_get(port_id, &stats);
	if (ret < 0)
		return -EOPNOTSUPP;

	if_stats->rx_packets = stats.ipackets;
	if_stats->tx_packets = stats.opackets;
	if_stats->rx_bytes = stats.ibytes;
	if_stats->tx_bytes = stats.obytes;
	if_stats->rx_errors = stats.ierrors;
	if_stats->tx_errors = stats.oerrors;
	if_stats->rx_dropped = stats.imissed;

	*data_len = sizeof(struct rtnl_link_stats64);

	return 0;
}

static int
get_mac(uint8_t port_id, void *data, size_t *data_len)
{
	struct ether_addr addr;

	rte_eth_macaddr_get(port_id, &addr);
	memcpy(data, &addr, sizeof(struct ether_addr));

	*data_len = sizeof(struct ether_addr);

	return 0;
}

static int
set_mac(uint8_t port_id, void *in_data)
{
	struct ether_addr addr;

	memcpy(&addr, in_data, ETHER_ADDR_LEN);

	return rte_eth_dev_default_mac_addr_set(port_id, &addr);
}

static int
start_port(uint8_t port_id)
{
	rte_eth_dev_stop(port_id);
	return rte_eth_dev_start(port_id);
}

static int
stop_port(uint8_t port_id)
{
	rte_eth_dev_stop(port_id);
	return 0;
}

static int
set_promisc(uint8_t port_id, void *in_data)
{
	int *promisc = in_data;

	if (*promisc)
		rte_eth_promiscuous_enable(port_id);
	else
		rte_eth_promiscuous_disable(port_id);

	return 0;
}

static int
set_allmulti(uint8_t port_id, void *in_data)
{
	int *allmulti = in_data;

	if (*allmulti)
		rte_eth_allmulticast_enable(port_id);
	else
		rte_eth_allmulticast_disable(port_id);

	return 0;
}

int
rte_eth_dev_control_process(uint32_t cmd_id, uint8_t port_id, void *in_data,
		void *out_data, size_t *out_data_len)
{
	if (!rte_eth_dev_is_valid_port(port_id))
		return -ENODEV;

	switch (cmd_id) {
	case UNCI_REQ_CHANGE_MTU:
		return set_mtu(port_id, in_data);
	case UNCI_REQ_GET_STATS:
		return get_stats(port_id, out_data, out_data_len);
	case UNCI_REQ_GET_MAC:
		return get_mac(port_id, out_data, out_data_len);
	case UNCI_REQ_SET_MAC:
		return set_mac(port_id, in_data);
	case UNCI_REQ_START_PORT:
		return start_port(port_id);
	case UNCI_REQ_STOP_PORT:
		return stop_port(port_id);
	case UNCI_REQ_SET_PROMISC:
		return set_promisc(port_id, in_data);
	case UNCI_REQ_SET_ALLMULTI:
		return set_allmulti(port_id, in_data);
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}
