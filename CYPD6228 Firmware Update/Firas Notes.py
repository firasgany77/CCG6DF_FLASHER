
"""
Notes:
1. Any data memory access operation (valid or invalid) does not result in response  or event generation.

2. CCG provides a response through the RESPONSE or PD_RESPONSE register for all writes into locations other
than the flash read/write memory and the write data memory regions.

3. A “Success” (0x02) response will be provided if the address and the data written to it are valid,
and appropriate error responses are provided if not.

4. CCG1 and CCG2 devices (HPIv1) make use of a single application firmware binary which can be updated
through the bootloader. The application firmware does not provide (?), and flash read/write functionality and
firmware updates can only be performed through the bootloader. Since the bootloader does not have any
USB-PD support, such designs will require the user to disable USB-PD operation as part of the firmware
update sequence.

5. CCG devices that implement (HPIv2) make use of dual application binaries with a bootloader that selects the
latest application binary for loading at start-up. The two application binaries (called FW1 and FW2
henceforth) can have similar functionality and will be capable of reading/updating each other. This
architecture allows users to perform firmware updates while the USB-PD port is operational and in contract.
PD port disable is only required when the device is being reset to activate the new firmware; and can be
deferred until the next system power cycle.

5. Boot Wait window (HPIv2): 50 ms: Interface is enabled, and EC can access HPI registers. EC needs to send
ENTER_FLASHING_MODE command in this window to force CCG to stay in bootloader mode. If timeout
expires and ENTER_FLASHING_MODE command is not received, CCG switches operation to FW mode.

6. EC can update the firmware running on the CCG device using the FLASH_ROW_READ_WRITE commands.
In the case of HPIv1, firmware update is only supported by the Bootloader.In the case of HPIv2, firmware
update is supported by the application firmware; with the constraint that none of the flash rows corresponding
to the active firmware binary can be changed.
The bootloader flash rows are maintained as read-only in all cases and can only be updated through SWD
access.

7. The FW1 and FW2 addresses are not fixed and need to be determined using the metadata.
each binary always starts with 256 bytes (64 Bytes * 4) of vector table and start-up code and the configuration table will be located
immediately after this region. vector table and start-up code are at Rows 21-24.

8. CCG6DF and CCG6SF have only 64 KB of flash and make use of code that has been stored into device ROM
during manufacturing. These devices have a reduced bootloader which is 1.25 KB (0x0500) in size:

1 KB (kilobyte) = 1024 bytes. So, 1.25 KB = 1.25 × 1024 bytes
1.25 KB = 1280 bytes
Converting 1280 to hexadecimal
1280 in decimal = 0x0500 in hexadecimal.
1280 / 64 = 20 rows of 64 Bytes. --> first 20 rows in FW files are for Notebook Bootloader.

"""
"""