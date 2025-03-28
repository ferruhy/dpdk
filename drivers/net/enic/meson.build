# SPDX-License-Identifier: BSD-3-Clause
# Copyright(c) 2018 Cisco Systems, Inc.

if is_windows
    build = false
    reason = 'not supported on Windows'
    subdir_done()
endif

if dpdk_conf.get('RTE_IOVA_IN_MBUF') == 0
    subdir_done()
endif

sources = files(
        'base/vnic_cq.c',
        'base/vnic_dev.c',
        'base/vnic_intr.c',
        'base/vnic_rq.c',
        'base/vnic_wq.c',
        'enic_ethdev.c',
        'enic_flow.c',
        'enic_fm_flow.c',
        'enic_main.c',
        'enic_res.c',
        'enic_rxtx.c',
        'enic_sriov.c',
        'enic_vf_representor.c',
)
deps += ['hash']
includes += include_directories('base')

# Build the avx2 handler for 64-bit X86 targets, even though 'machine'
# may not. This is to support users who build for the min supported machine
# and need to run the binary on newer CPUs too.
if dpdk_conf.has('RTE_ARCH_X86_64')
    cflags += '-DENIC_RXTX_VEC'
    enic_avx2_lib = static_library('enic_avx2_lib',
            'enic_rxtx_vec_avx2.c',
            dependencies: [static_rte_ethdev, static_rte_bus_pci],
            include_directories: includes,
            c_args: [cflags, cc_avx2_flags])
    objs += enic_avx2_lib.extract_objects('enic_rxtx_vec_avx2.c')
endif

annotate_locks = false
