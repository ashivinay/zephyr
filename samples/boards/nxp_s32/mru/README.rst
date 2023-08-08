.. _nxp_s32_mru-samples:

NXP S32 MRU Sample Application
##############################

Overview
********

This sample application demonstrates how to use the NXP S32 MRU through the
:ref:`MBOX API <mbox_api>`. This application runs in a single core to simplify
the setup, but normally you would want to use the MRU as a mean of messaging
between two or more cores.

The source code for this sample application can be found at:
:zephyr_file:`samples/boards/nxp_s32/mru`.

Building and Running
********************

Build and run the sample application:

.. zephyr-app-commands::
   :zephyr-app: samples/boards/nxp_s32/mru
   :board: s32z270dc2_rtu0_r52
   :goals: build flash

Once started, you should see in the serial console an output similar as below.

.. code-block:: console

   *** Booting Zephyr OS build zephyr-v3.4.0-2086-g8c1b1112f8ef ***
   Ping on channel 0, tx data is 9831024
   Pong on channel 0, rx data is 9831024

   Ping on channel 0, tx data is 33832020
   Pong on channel 0, rx data is 33832020

   Ping on channel 0, tx data is 57840018
   Pong on channel 0, rx data is 57840018
