.. _can_shell:

CAN Shell
#########

.. contents::
    :local:
    :depth: 1

Overview
********

The CAN shell provides a ``can`` command with a set of subcommands for the :ref:`shell <shell_api>`
module. It allows for testing and exploring the :ref:`can_api` driver API through an interactive
interface without having to write a dedicated application. The CAN shell can also be enabled in
existing applications to aid in interactive debugging of CAN issues.

The CAN shell provides access to most CAN controller features, including inspection, configuration,
sending and receiving of CAN frames, and bus recovery.

In order to enable the CAN shell, the following :ref:`Kconfig <kconfig>` options must be enabled:

* :kconfig:option:`CONFIG_SHELL`
* :kconfig:option:`CONFIG_CAN`
* :kconfig:option:`CONFIG_CAN_SHELL`

The following :ref:`Kconfig <kconfig>` options enable additional subcommands and features of the
``can`` command:

* :kconfig:option:`CONFIG_CAN_FD_MODE` enables CAN FD specific subcommands (e.g. for setting the
  timing for the CAN FD data phase).
* :kconfig:option:`CONFIG_CAN_RX_TIMESTAMP` enables printing of timestamps for received CAN frames.
* :kconfig:option:`CONFIG_CAN_STATS` enables printing of various statistics for the CAN controller
  in the ``can show`` subcommand. This depends on :kconfig:option:`CONFIG_STATS` being enabled as
  well.
* :kconfig:option:`CONFIG_CAN_AUTO_BUS_OFF_RECOVERY` enables the ``can recover`` subcommand when
  disabled.

For example, building the :ref:`hello_world` sample for the :ref:`frdm_k64f` with the CAN shell and
CAN statistics enabled:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: frdm_k64f
   :gen-args: -DCONFIG_SHELL=y -DCONFIG_CAN=y -DCONFIG_CAN_SHELL=y -DCONFIG_STATS=y -DCONFIG_CAN_STATS=y
   :goals: build

See the :ref:`shell <shell_api>` documentation for general instructions on how to connect and
interact with the shell. The CAN shell comes with built-in help (unless
:kconfig:option:`CONFIG_SHELL_HELP` is disabled). The built-in help messages can be printed by
passing ``-h`` or ``--help`` to the ``can`` command or any of its subcommands. All subcommands also
support tab-completion of their arguments.

.. tip::
   All of the CAN shell subcommands take the name to a CAN controller as their first argument. A
   list of all devices available can be obtained using the ``device list`` shell command when
   :kconfig:option:`CONFIG_DEVICE_SHELL` is enabled.

Inspection
**********

The properties of a given CAN controller can be inspected using the ``can show`` subcommand as shown
below. The properties include the core CAN clock rate, the maximum supported bitrate, the number of
RX filters supported, capabilities, current state, error counters, timing limits, and more:

.. code-block:: console

   uart:~$ can show can@0
   core clock:      144000000 Hz
   max bitrate:     5000000 bps
   max std filters: 15
   max ext filters: 15
   capabilities:    normal loopback listen-only fd
   state:           stopped
   rx errors:       0
   tx errors:       0
   timing:          sjw 1..128, prop_seg 0..0, phase_seg1 2..256, phase_seg2 2..128, prescaler 1..512
   timing data:     sjw 1..16, prop_seg 0..0, phase_seg1 1..32, phase_seg2 1..16, prescaler 1..32
   statistics:
     bit errors:    0
       bit0 errors: 0
       bit1 errors: 0
     stuff errors:  0
     crc errors:    0
     form errors:   0
     ack errors:    0
     rx overruns:   0

.. note::
   The statistics are only printed if :kconfig:option:`CONFIG_CAN_STATS` is enabled.

Configuration
*************

The CAN shell allows for configuring the CAN controller mode and timing, along with starting and
stopping the processing of CAN frames.

.. note::
   The CAN controller mode and timing can only be changed while the CAN controller is stopped, which
   is the initial setting upon boot-up. The initial CAN controller mode is set to ``normal`` and the
   initial timing is set according to the ``bus-speed``, ``sample-point``, ``bus-speed-data``, and
   ``sample-point-data`` :ref:`devicetree` properties:

Timing
======

The classical CAN bitrate/CAN FD arbitration phase bitrate can be configured using the ``can
bitrate`` subcommand as shown below. The bitrate is specified in bits per second.

.. code-block:: console

   uart:~$ can bitrate can@0 125000
   setting bitrate to 125000 bps

If using CAN FD mode, the data phase bitrate can be configured using the ``can dbitrate`` subcommand
as shown below. The bitrate is specified in bits per second.

.. code-block:: console

   uart:~$ can dbitrate can@0 1000000
   setting data bitrate to 1000000 bps

Both of these subcommands allow specifying an optional sample point in per mille and a
(Re)Synchronization Jump Width (SJW) in Time Quanta as positional arguments. Refer to the
interactive help of the subcommands for more details.

It is also possible to configure the raw bit timing using the ``can timing`` and ``can dtiming``
subcommands. Refer to the interactive help output for these subcommands for details on the required
arguments.

Mode
====

The CAN shell allows for setting the mode of the CAN controller using the ``can mode``
subcommand. An example for enabling CAN FD mode is shown below.

.. code-block:: console

   uart:~$ can mode can@0 fd
   setting mode 0x00000004

The subcommand accepts multiple modes given on the same command line (e.g. ``can mode can@0 fd
loopback`` for setting CAN FD and loopback mode). Vendor-specific modes can be specified in
hexadecimal.

Starting and Stopping
=====================

After the timing and mode has been configured, the CAN controller can be started using the ``can
start`` subcommand as shown below. This will enable reception and transmission of CAN frames.

.. code-block:: console

   uart:~$ can start can@0
   starting can@0

Prior to reconfiguring the timing or mode, the CAN controller needs to be stopped using the ``can
stop`` subcommand as shown below:

.. code-block:: console

   uart:~$ can stop can@0
   stopping can@0

Receiving
*********

TODO

.. code-block:: console

   uart:~$ can filter add can@0 010
   adding filter with standard (11-bit) CAN ID 0x010, CAN ID mask 0x7ff, data frames 1, RTR frames 0, CAN FD frames 0
   filter ID: 0

.. code-block:: console

   --       010   [8]  01 02 03 04 05 06 07 08
   --       010   [1]  01
   --       010   [1]  20

.. code-block:: console

   uart:~$ can filter remove can@0 0
   removing filter with ID 0

Sending
*******

TODO

.. code-block:: console

   uart:~$ can send can@0 010 1 2 3 4 5 6 7 8
   enqueuing CAN frame #2 with standard (11-bit) CAN ID 0x010, RTR 0, CAN FD 0, BRS 0, DLC 8
   CAN frame #0 successfully sent

   uart:~$ can send can@0 010 1
   enqueuing CAN frame #3 with standard (11-bit) CAN ID 0x010, RTR 0, CAN FD 0, BRS 0, DLC 1
   CAN frame #1 successfully sent

   uart:~$ can send can@0 010 20
   enqueuing CAN frame #4 with standard (11-bit) CAN ID 0x010, RTR 0, CAN FD 0, BRS 0, DLC 1
   CAN frame #2 successfully sent

Bus Recovery
************

TODO

.. code-block:: console

   uart:~$ can recover can@0
   recovering, no timeout

.. note::
   The ``recover`` subcommand is only available if
   :kconfig:option:`CONFIG_CAN_AUTO_BUS_OFF_RECOVERY` is disabled.
