/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2021 6WIND S.A.
 * Copyright 2021 Mellanox Technologies, Ltd
 */

#include "mlx5_tx.h"

/* Generate routines without Enhanced Multi-Packet Write support. */
MLX5_TXOFF_DECL(full,
		MLX5_TXOFF_CONFIG_FULL)

MLX5_TXOFF_DECL(none,
		MLX5_TXOFF_CONFIG_NONE)

MLX5_TXOFF_DECL(md,
		MLX5_TXOFF_CONFIG_METADATA)

MLX5_TXOFF_DECL(mt,
		MLX5_TXOFF_CONFIG_MULTI | MLX5_TXOFF_CONFIG_TSO |
		MLX5_TXOFF_CONFIG_METADATA)

MLX5_TXOFF_DECL(mtsc,
		MLX5_TXOFF_CONFIG_MULTI | MLX5_TXOFF_CONFIG_TSO |
		MLX5_TXOFF_CONFIG_SWP |	MLX5_TXOFF_CONFIG_CSUM |
		MLX5_TXOFF_CONFIG_METADATA)

MLX5_TXOFF_DECL(mti,
		MLX5_TXOFF_CONFIG_MULTI | MLX5_TXOFF_CONFIG_TSO |
		MLX5_TXOFF_CONFIG_INLINE |
		MLX5_TXOFF_CONFIG_METADATA)

MLX5_TXOFF_DECL(mtv,
		MLX5_TXOFF_CONFIG_MULTI | MLX5_TXOFF_CONFIG_TSO |
		MLX5_TXOFF_CONFIG_VLAN |
		MLX5_TXOFF_CONFIG_METADATA)

MLX5_TXOFF_DECL(mtiv,
		MLX5_TXOFF_CONFIG_MULTI | MLX5_TXOFF_CONFIG_TSO |
		MLX5_TXOFF_CONFIG_INLINE | MLX5_TXOFF_CONFIG_VLAN |
		MLX5_TXOFF_CONFIG_METADATA)

MLX5_TXOFF_DECL(sc,
		MLX5_TXOFF_CONFIG_SWP |	MLX5_TXOFF_CONFIG_CSUM |
		MLX5_TXOFF_CONFIG_METADATA)

MLX5_TXOFF_DECL(sci,
		MLX5_TXOFF_CONFIG_SWP |	MLX5_TXOFF_CONFIG_CSUM |
		MLX5_TXOFF_CONFIG_INLINE |
		MLX5_TXOFF_CONFIG_METADATA)

MLX5_TXOFF_DECL(scv,
		MLX5_TXOFF_CONFIG_SWP |	MLX5_TXOFF_CONFIG_CSUM |
		MLX5_TXOFF_CONFIG_VLAN |
		MLX5_TXOFF_CONFIG_METADATA)

MLX5_TXOFF_DECL(sciv,
		MLX5_TXOFF_CONFIG_SWP |	MLX5_TXOFF_CONFIG_CSUM |
		MLX5_TXOFF_CONFIG_INLINE | MLX5_TXOFF_CONFIG_VLAN |
		MLX5_TXOFF_CONFIG_METADATA)

MLX5_TXOFF_DECL(i,
		MLX5_TXOFF_CONFIG_INLINE |
		MLX5_TXOFF_CONFIG_METADATA)

MLX5_TXOFF_DECL(v,
		MLX5_TXOFF_CONFIG_VLAN |
		MLX5_TXOFF_CONFIG_METADATA)

MLX5_TXOFF_DECL(iv,
		MLX5_TXOFF_CONFIG_INLINE | MLX5_TXOFF_CONFIG_VLAN |
		MLX5_TXOFF_CONFIG_METADATA)
