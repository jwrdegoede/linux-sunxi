.. SPDX-License-Identifier: GPL-2.0

.. include:: <isonum.txt>

===============================================================
Intel Image Processing Unit 6 (IPU6) Input System driver
===============================================================

Copyright |copy| 2023 Intel Corporation

Introduction
============

This file documents the Intel IPU6 (6th generation Image Processing Unit)
Input System (MIPI CSI2 receiver) drivers located under
drivers/media/pci/intel/ipu6.

The Intel IPU6 can be found in certain Intel Chipsets but not in all SKUs:

* TigerLake
* JasperLake
* AlderLake
* RaptorLake
* MeteorLake

Intel IPU6 is made up of two components - Input System (ISYS) and Processing
System (PSYS).

The Input System mainly works as MIPI CSI2 receiver which receives and
processes the imaging data from the sensors and outputs the frames to memory.

There are 2 driver modules - intel_ipu6 and intel_ipu6_isys. intel_ipu6 is an
IPU6 common driver which does PCI configuration, firmware loading and parsing,
firmware authentication, DMA mapping and IPU-MMU (internal Memory mapping Unit)
configuration. intel_ipu6_isys implements V4L2, Media Controller and V4L2
sub-device interfaces. The IPU6 ISYS driver supports camera sensors connected
to the IPU6 ISYS through V4L2 sub-device sensor drivers.

.. Note:: See Documentation/driver-api/media/drivers/ipu6.rst for more
	  information about the IPU6 hardware.


Input system driver
===================

The input System driver mainly configures CSI2 DPHY, constructs the firmware
stream configuration, sends commands to firmware, gets response from hardware
and firmware and then returns buffers to user.
The ISYS is represented as several V4L2 sub-devices - 'Intel IPU6 CSI2 $port',
which provide V4L2 subdev interfaces to the user space, there are also several
video nodes for each CSI-2 stream capture - 'Intel IPU6 ISYS capture $num' which
provide interface to user to set formats, queue buffers and streaming.

.. kernel-figure::  ipu6_isys_graph.svg
   :alt: ipu6 isys media graph without multiple streams support

   ipu6 isys media graph without multiple streams support

.. kernel-figure::  ipu6_isys_multi.svg
   :alt: ipu6 isys media graph with multiple streams support

   ipu6 isys media graph with multiple streams support

Capturing frames by IPU6 ISYS
------------------------------------

IPU6 ISYS is used to capture frames from the camera sensors connected to the
CSI2 ports. The supported input formats of ISYS are listed in table below:

.. tabularcolumns:: |p{0.8cm}|p{4.0cm}|p{4.0cm}|

.. flat-table::
    :header-rows: 1

    * - IPU6 ISYS supported input formats

    * - RGB565, RGB888

    * - UYVY8, YUYV8

    * - RAW8, RAW10, RAW12

Here is an example of IPU6 ISYS raw capture on Dell XPS 9315 laptop. On this
machine, ov01a10 sensor is connected to IPU ISYS CSI2 port 2, which can
generate images at sBGGR10 with resolution 1280x800.

Using the media controller APIs, we can configure ov01a10 sensor by
media-ctl [#f1]_ and yavta [#f2]_ to transmit frames to IPU6 ISYS.

.. code-block:: none

    # This example assumes /dev/media0 as the IPU ISYS media device
    export MDEV=/dev/media0

    # Establish the link for the media devices using media-ctl
    media-ctl -d $MDEV -l "\"ov01a10 3-0036\":0 -> \"Intel IPU6 CSI2 2\":0[1]"

    # Set the format for the media devices
    media-ctl -d $MDEV -V "ov01a10:0 [fmt:SBGGR10/1280x800]"
    media-ctl -d $MDEV -V "Intel IPU6 CSI2 2:0 [fmt:SBGGR10/1280x800]"
    media-ctl -d $MDEV -V "Intel IPU6 CSI2 2:1 [fmt:SBGGR10/1280x800]"

    # Establish the link for the media devices using media-ctl
    media-ctl -d $MDEV -l "\"ov01a10 3-0036\":0 -> \"Intel IPU6 CSI2 2\":0[1]"
    media-ctl -d $MDEV -l "\"Intel IPU6 CSI2 2\":1 ->\"Intel IPU6 ISYS Capture 0\":0[5]"

Once the media pipeline is configured, desired sensor specific settings
(such as exposure and gain settings) can be set, using the yavta tool.

e.g

.. code-block:: none

    # and that ov01a10 sensor is connected to i2c bus 3 with address 0x36
    export SDEV=$(media-ctl -d $MDEV -e "ov01a10 3-0036")

    yavta -w 0x009e0903 400 $SDEV
    yavta -w 0x009e0913 1000 $SDEV
    yavta -w 0x009e0911 2000 $SDEV

Once the desired sensor settings are set, frame captures can be done as below.

e.g

.. code-block:: none

    yavta --data-prefix -u -c10 -n5 -I -s 1280x800 --file=/tmp/frame-#.bin \
          -f SBGGR10 $(media-ctl -d $MDEV -e "Intel IPU6 ISYS Capture 0")

With the above command, 10 frames are captured at 1280x800 resolution with
sBGGR10 format. The captured frames are available as /tmp/frame-#.bin files.


References
==========

.. [#f1] https://git.ideasonboard.org/?p=media-ctl.git;a=summary
.. [#f2] https://git.ideasonboard.org/yavta.git
