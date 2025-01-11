#!/usr/bin/env python3

import os
import smbus2
from intelhex import IntelHex


# -------------------------------------------------------------------
# I2C Bus / Address / Constants
# -------------------------------------------------------------------
I2C_BUS                = 2        # Example: I2C bus number
I2C_SLAVE_ADDR         = 0x40     # The CCG6DF device's I2C address
FLASH_ROW_SIZE_BYTES   = 64      # Flash row size for CCG6DF
SUCCESS_CODE           = 0x02     # Example "command success" code

# -------------------------------------------------------------------
# Offsets & Opcodes
# (For demonstration; in real usage verify if HPv1 vs. HPv2 addresses)
# -------------------------------------------------------------------
DEVICE_MODE_OFFSET           = 0x0000
ENTER_FLASHING_MODE_OFFSET   = 0x000A
JUMP_TO_BOOT_OFFSET          = 0x0007
FLASH_ROW_READ_WRITE_OFFSET  = 0x000C
RESET_OFFSET                 = 0x0800
PDPORT_ENABLE_OFFSET         = 0x002C

PORT_DISABLE_OPCODE          = 0x11
PD_CONTROL_OFFSET_PORT0      = 0x1006
PD_CONTROL_OFFSET_PORT1      = 0x2006
RESPONSE_OFFSET_PORT0        = 0x1400
RESPONSE_OFFSET_PORT1        = 0x2400

FW1_METADATA_ROW             = 0xFF80
FW2_METADATA_ROW             = 0xFF00

# -------------------------------------------------------------------
# Helper Functions for Double-Byte Offset (LSB first, MSB second)
# -------------------------------------------------------------------
def i2c_write_block(bus, dev_addr, register_offset_16b, data_bytes):
    """
    Write 'data_bytes' to a 16-bit register offset on the device at 'dev_addr'.
    For offset 0xABCD, we send [0xCD, 0xAB] as the first two bytes.
    Then we send 'data_bytes'.

    Example:
      MSB = (register_offset_16b >> 8) & 0xFF   # 0xAB
      LSB = register_offset_16b & 0xFF         # 0xCD
    -> outbuf = [LSB, MSB] + data_bytes
    """
    ls_byte = register_offset_16b & 0xFF
    ms_byte = (register_offset_16b >> 8) & 0xFF
    outbuf  = [ls_byte, ms_byte] + list(data_bytes)
    bus.write_i2c_block_data(dev_addr, outbuf[0], outbuf[1:])


def i2c_read_block(bus, dev_addr, register_offset_16b, num_bytes):
    """
    Read 'num_bytes' from a 16-bit register offset.
    1) Write [LSB, MSB]
    2) Repeated START + read 'num_bytes'

    We call write_byte_data or write_i2c_block_data in a way that sends the 2-byte offset first.
    Then we do read_i2c_block_data to read the data out.
    """
    ls_byte = register_offset_16b & 0xFF
    ms_byte = (register_offset_16b >> 8) & 0xFF

    # First, we send the 2-byte offset. For smbus2, we can do this in two steps or one block:
    bus.write_i2c_block_data(dev_addr, ls_byte, [ms_byte])

    # Now, read the requested number of bytes from the device
    return bus.read_i2c_block_data(dev_addr, 0, num_bytes)


# -------------------------------------------------------------------
# Device-Specific Commands
# -------------------------------------------------------------------
def read_device_mode(bus):
    """
    Example: Reads a single byte from the DEVICE_MODE_OFFSET (0x0000).
    Returns the raw byte encoding the current device mode.
    """
    data = i2c_read_block(bus, I2C_SLAVE_ADDR, DEVICE_MODE_OFFSET, 1)
    return data[0]

def enter_flashing_mode(bus):
    """
    Writes the signature 'P' (0x50) to ENTER_FLASHING_MODE_OFFSET (0x000A).
    """
    signature = 0x50  # 'P'
    i2c_write_block(bus, I2C_SLAVE_ADDR, ENTER_FLASHING_MODE_OFFSET, [signature])

def jump_to_boot(bus):
    """
    Writes the signature 'J' (0x4A) to JUMP_TO_BOOT_OFFSET (0x0007).
    """
    signature = 0x4A  # 'J'
    i2c_write_block(bus, I2C_SLAVE_ADDR, JUMP_TO_BOOT_OFFSET, [signature])

def reset_device(bus, reset_type=1):
    """
    Writes:
      Byte[0] = 'R' (0x52) as signature,
      Byte[1] = reset_type (0 => I2C reset, 1 => Device reset)
    to RESET_OFFSET (0x0008).
    """
    signature = 0x52  # 'R'
    i2c_write_block(bus, I2C_SLAVE_ADDR, RESET_OFFSET, [signature, reset_type & 0xFF])

def flash_row_read_write(bus, row_number, data_to_write=None):
    """
    Demonstration of a single command that triggers read/write of a Flash row
    (128 bytes for CCG6DF):
      Byte[0]: 'F' => 0x46 (signature)
      Byte[1]: command => 0 for Read, 1 for Write
      Byte[2..3]: row_number (LSB, MSB)
      If writing, append 128 bytes of data.

    NOTE: In real HPv2 or HPv1 flows, you typically write the row data to a data-memory region
    and then trigger the row write via this register. This simplified function lumps them.
    """
    # Signature 'F' => 0x46
    if data_to_write is not None:
        if len(data_to_write) != FLASH_ROW_SIZE_BYTES:
            raise ValueError("Data to write must match the FLASH_ROW_SIZE_BYTES.")
        cmd_buf = [
            0x46,          # 'F'
            0x01,          # Flash Row Write
            row_number & 0xFF,
            (row_number >> 8) & 0xFF
        ] + list(data_to_write)
        i2c_write_block(bus, I2C_SLAVE_ADDR, FLASH_ROW_READ_WRITE_OFFSET, cmd_buf)
    else:
        # Read
        cmd_buf = [
            0x46,
            0x00,         # 0 => Flash Row Read
            row_number & 0xFF,
            (row_number >> 8) & 0xFF
        ]
        i2c_write_block(bus, I2C_SLAVE_ADDR, FLASH_ROW_READ_WRITE_OFFSET, cmd_buf)
        # Possibly wait for "flash data available" response,
        # then read from data memory or from the same offset if doc says so.
        return i2c_read_block(bus, I2C_SLAVE_ADDR, FLASH_ROW_READ_WRITE_OFFSET, FLASH_ROW_SIZE_BYTES)

def disable_pd_ports(bus):
    """
    Example: Write PORT_DISABLE_OPCODE (0x11) to the PD_CONTROL_OFFSET_PORT0 (0x1006).
    If you have two ports, repeat for PORT1.
    Check your device doc for actual procedure to disable PD ports.
    """
    i2c_write_block(bus, I2C_SLAVE_ADDR, PD_CONTROL_OFFSET_PORT0, [PORT_DISABLE_OPCODE])
    # Potentially do same for port1:
    # i2c_write_block(bus, I2C_SLAVE_ADDR, PD_CONTROL_OFFSET_PORT1, [PORT_DISABLE_OPCODE])

def validate_firmware(bus):
    """
    If your device requires writing 0x01 to 0x0B (or some offset) to validate FW, do that here.
    We'll leave it as a placeholder.
    """
    pass


# -------------------------------------------------------------------
# Main Firmware Update Flow
# -------------------------------------------------------------------
def update_firmware_ccg6df_example(hex_file_path):
    """
    This function shows a sequence to:
      1. Check device mode; if in FW mode, disable PD ports, jump to boot.
      2. Enter flashing mode
      3. Clear metadata rows
      4. Write all rows from the IntelHex
      5. Validate
      6. Reset
    Using a 16-bit offset approach: we always send LS byte before MS byte.

    This code is for illustration. Real usage must confirm register addresses
    and step details from your device documentation.
    """
    print("Opening I2C bus:", I2C_BUS, "Device address:", hex(I2C_SLAVE_ADDR))
    bus = smbus2.SMBus(I2C_BUS)

    try:
        # 1. Read device mode
        mode_before = read_device_mode(bus)
        print("Current device mode (raw):", hex(mode_before))

        # 2. If in FW mode, disable PD ports, jump to boot
        print("Disabling PD ports...")
        disable_pd_ports(bus)

        print("Jumping to bootloader mode...")
        jump_to_boot(bus)

        # In real code, wait for a "Reset Complete" event or poll. We'll do a short sleep:
        import time
        time.sleep(0.2)

        # 3. Now in bootloader mode?
        mode_boot = read_device_mode(bus)
        print("Device mode after jump:", hex(mode_boot))

        # 4. Enter flashing mode
        print("Entering flashing mode...")
        enter_flashing_mode(bus)
        time.sleep(0.05)


    finally:
        bus.close()


# -------------------------------------------------------------------
# Example usage if run directly
# -------------------------------------------------------------------
if __name__ == "__main__":
    firmware_hex_path = "/home/firas/Documents/CYPD6228/CYPD6228-96BZXI_notebook_dualapp_usb4_228_2.hex"
    print("Starting firmware update for CCG6DF device (two-byte offset, LS byte first).")
    update_firmware_ccg6df_example(firmware_hex_path)
    print("Done.")
