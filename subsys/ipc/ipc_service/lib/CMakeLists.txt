# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_IPC_SERVICE_ICMSG			icmsg.c)
zephyr_sources_ifdef(CONFIG_IPC_SERVICE_ICMSG_ME		icmsg_me.c)
zephyr_sources_ifdef(CONFIG_IPC_SERVICE_RPMSG			ipc_rpmsg.c)
zephyr_sources_ifdef(CONFIG_IPC_SERVICE_STATIC_VRINGS		ipc_static_vrings.c)
zephyr_sources_ifdef(CONFIG_PBUF				pbuf.c)
