====================
request_firmware API
====================

You would typically load firmware and then load it into your device somehow.
The typical firmware work flow is reflected below::

	 if(request_firmware(&fw_entry, $FIRMWARE, device) == 0)
                copy_fw_to_device(fw_entry->data, fw_entry->size);
	 release_firmware(fw_entry);

Synchronous firmware requests
=============================

Synchronous firmware requests will wait until the firmware is found or until
an error is returned.

request_firmware
----------------
.. kernel-doc:: drivers/base/firmware_loader/main.c
   :functions: request_firmware

firmware_request_nowarn
-----------------------
.. kernel-doc:: drivers/base/firmware_loader/main.c
   :functions: firmware_request_nowarn

request_firmware_direct
-----------------------
.. kernel-doc:: drivers/base/firmware_loader/main.c
   :functions: request_firmware_direct

request_firmware_into_buf
-------------------------
.. kernel-doc:: drivers/base/firmware_loader/main.c
   :functions: request_firmware_into_buf

Asynchronous firmware requests
==============================

Asynchronous firmware requests allow driver code to not have to wait
until the firmware or an error is returned. Function callbacks are
provided so that when the firmware or an error is found the driver is
informed through the callback. request_firmware_nowait() cannot be called
in atomic contexts.

request_firmware_nowait
-----------------------
.. kernel-doc:: drivers/base/firmware_loader/main.c
   :functions: request_firmware_nowait

Special optimizations on reboot
===============================

Some devices have an optimization in place to enable the firmware to be
retained during system reboot. When such optimizations are used the driver
author must ensure the firmware is still available on resume from suspend,
this can be done with firmware_request_cache() instead of requesting for the
firmware to be loaded.

firmware_request_cache()
------------------------
.. kernel-doc:: drivers/base/firmware_loader/main.c
   :functions: firmware_request_cache

request firmware API expected driver use
========================================

Once an API call returns you process the firmware and then release the
firmware. For example if you used request_firmware() and it returns,
the driver has the firmware image accessible in fw_entry->{data,size}.
If something went wrong request_firmware() returns non-zero and fw_entry
is set to NULL. Once your driver is done with processing the firmware it
can call call release_firmware(fw_entry) to release the firmware image
and any related resource.

EFI embedded firmware support
=============================

On some devices the system's EFI code / ROM may contain an embedded copy
of firmware for some of the system's integrated peripheral devices and
the peripheral's Linux device-driver needs to access this firmware.

A device driver which needs this can describe the firmware it needs
using an efi_embedded_fw_desc struct:

.. kernel-doc:: include/linux/efi_embedded_fw.h
   :functions: efi_embedded_fw_desc

The EFI embedded-fw code works by scanning all EFI_BOOT_SERVICES_CODE memory
segments for an eight byte sequence matching prefix, if the prefix is found it
then does a crc32 over length bytes and if that matches makes a copy of length
bytes and adds that to its list with found firmwares.

To avoid doing this somewhat expensive scan on all systems, dmi matching is
used. Drivers are expected to export a dmi_system_id array, with each entries'
driver_data pointing to an efi_embedded_fw_desc.

To register this array with the efi-embedded-fw code, a driver needs to:

1. Always be builtin to the kernel or store the dmi_system_id array in a
   separate object file which always gets builtin.

2. Add an extern declaration for the dmi_system_id array to
   include/linux/efi_embedded_fw.h.

3. Add the dmi_system_id array to the embedded_fw_table in
   drivers/firmware/efi/embedded-firmware.c wrapped in a #ifdef testing that
   the driver is being builtin.

4. Add "select EFI_EMBEDDED_FIRMWARE if EFI_STUB" to its Kconfig entry.

The request_firmware() function will always first try to load firmware with
the specified name directly from the disk, so the EFI embedded-fw can always
be overridden by placing a file under /lib/firmare.

To make request_firmware() fallback to trying EFI embedded firmwares after this,
the driver must set a boolean "efi-embedded-firmware" device-property on the
device before passing it to request_firmware(). Note that this disables the
usual usermodehelper fallback, so you may want to only set this on systems
which match your dmi_system_id array.

Once the device-property is set, the driver can use the regular
request_firmware() function to get the firmware, using the name filled in
in the efi_embedded_fw_desc.

Note that:

1. The code scanning for EFI embbedded-firmware runs near the end
   of start_kernel(), just before calling rest_init(). For normal drivers and
   subsystems using subsys_initcall() to register themselves this does not
   matter. This means that code running earlier cannot use EFI
   embbedded-firmware.

2. ATM the EFI embedded-fw code assumes that firmwares always start at an offset
   which is a multiple of 8 bytes, if this is not true for your case send in
   a patch to fix this.

3. ATM the EFI embedded-fw code only works on x86 because other archs free
   EFI_BOOT_SERVICES_CODE before the EFI embedded-fw code gets a chance to
   scan it.

4. The current brute-force scanning of EFI_BOOT_SERVICES_CODE is an ad-hoc
   brute-force solution. There has been discussion to use the PI spec's
   Firmware Volume protocol. This has been rejected because the FV Protocol
   relies on *internal* interfaces of PI spec, and:
   1. The The PI spec does not define firmware at all
   2. The internal interfaces of PI Spec does not guarantee any backward
      compatibility. Any implementation details in FV may be subject to change,
      and may vary system to system. Supporting the FV Protocol would be
      difficult as it is purposely ambiguous.
