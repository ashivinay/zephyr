# Copyright 2023 NXP
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_QDEC_NXP_S32
  qdec_nxp_s32.c
)
