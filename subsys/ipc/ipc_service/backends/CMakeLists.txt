# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_IPC_SERVICE_BACKEND_ICMSG			ipc_icmsg.c)
zephyr_sources_ifdef(CONFIG_IPC_SERVICE_BACKEND_ICMSG_ME_INITIATOR	ipc_icmsg_me_initiator.c)
zephyr_sources_ifdef(CONFIG_IPC_SERVICE_BACKEND_ICMSG_ME_FOLLOWER	ipc_icmsg_me_follower.c)
zephyr_sources_ifdef(CONFIG_IPC_SERVICE_BACKEND_ICBMSG			ipc_icbmsg.c)
zephyr_sources_ifdef(CONFIG_IPC_SERVICE_BACKEND_RPMSG			ipc_rpmsg_static_vrings.c)
