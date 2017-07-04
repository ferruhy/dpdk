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

#ifndef _RTE_CTRL_IF_H_
#define _RTE_CTRL_IF_H_

/**
 * @file
 *
 * Control Interface Library for RTE
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#include <linux/types.h>

#include <exec-env/unci.h>

/**
 * Creates control interfaces (Linux virtual network interface)for
 * given ethdev port.
 *
 * This API opens device created by supportive kernel module and initializes
 * kernel communication interface.
 *
 * With first interface created, a pthread created to receive the control
 * messages.
 *
 * If supportive kernel module is not inserted this API will return
 * an error.
 *
 * @param port_id
 *  port id to create virtual interface
 * @return
 *  0 on success.
 *  Negative value on error.
 */
int rte_eth_control_interface_create(uint8_t port_id);

/**
 * Destroys control interfaces.
 *
 * This API close device created by supportive kernel module and release
 * underlying communication interface.
 *
 * @return
 * @param port_id
 *  port id to destroy virtual interface
 *  0 on success.
 *  Negative value on error.
 */
int rte_eth_control_interface_destroy(uint8_t port_id);

#ifdef __cplusplus
}
#endif

#endif /* _RTE_CTRL_IF_H_ */
