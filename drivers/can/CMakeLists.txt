# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_CAN              can_common.c)
zephyr_sources_ifdef(CONFIG_CAN_LOOPBACK     can_loopback.c)
zephyr_sources_ifdef(CONFIG_CAN_MCP2515      can_mcp2515.c)
zephyr_sources_ifdef(CONFIG_CAN_STM32        can_stm32.c)
zephyr_sources_ifdef(CONFIG_CAN_STM32FD      can_stm32fd.c)
zephyr_sources_ifdef(CONFIG_CAN_MCAN         can_mcan.c)
zephyr_sources_ifdef(CONFIG_CAN_SAM          can_sam.c)
zephyr_sources_ifdef(CONFIG_CAN_SAM0         can_sam0.c)
zephyr_linker_sources_ifdef(CONFIG_CAN_SAM RAM_SECTIONS can_sam0.ld)
zephyr_linker_sources_ifdef(CONFIG_CAN_SAM0 RAM_SECTIONS can_sam0.ld)
zephyr_sources_ifdef(CONFIG_CAN_MCUX_FLEXCAN can_mcux_flexcan.c)

zephyr_sources_ifdef(CONFIG_USERSPACE        can_handlers.c)
zephyr_sources_ifdef(CONFIG_CAN_SHELL        can_shell.c)
zephyr_sources_ifdef(CONFIG_CAN_NET          can_net.c)
