..  BSD LICENSE
    Copyright(c) 2017 Intel Corporation. All rights reserved.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.
    * Neither the name of Intel Corporation nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
    A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

.. _Ethtool_Library:

Ethtool Library
===============

Ethtool interface
-----------------

The Ethtool interface is built as a separate library, and implements
the following functions:

- ``rte_ethtool_get_drvinfo()``
- ``rte_ethtool_get_regs_len()``
- ``rte_ethtool_get_regs()``
- ``rte_ethtool_get_link()``
- ``rte_ethtool_get_eeprom_len()``
- ``rte_ethtool_get_eeprom()``
- ``rte_ethtool_set_eeprom()``
- ``rte_ethtool_get_pauseparam()``
- ``rte_ethtool_set_pauseparam()``
- ``rte_ethtool_net_open()``
- ``rte_ethtool_net_stop()``
- ``rte_ethtool_net_get_mac_addr()``
- ``rte_ethtool_net_set_mac_addr()``
- ``rte_ethtool_net_validate_addr()``
- ``rte_ethtool_net_change_mtu()``
- ``rte_ethtool_net_get_stats64()``
- ``rte_ethtool_net_vlan_rx_add_vid()``
- ``rte_ethtool_net_vlan_rx_kill_vid()``
- ``rte_ethtool_net_set_rx_mode()``
- ``rte_ethtool_get_ringparam()``
- ``rte_ethtool_set_ringparam()``
