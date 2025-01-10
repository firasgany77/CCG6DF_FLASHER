#!/usr/bin/env python3
"""
Example code to update the firmware on a CCG6DF device, using a two-byte register offset
(LS byte first, MS byte second) and i2c_rdwr to handle writes bigger than 32 bytes.

It uses:
- smbus2 for I2C transactions
- intelhex for parsing the HEX file

All addresses, constants, and partial opcodes are from the original request.
"""

import os
import smbus2
from smbus2 import i2c_msg
from intelhex import IntelHex

# -------------------------------------------------------------------
# I2C Bus / Address / Constants
# -------------------------------------------------------------------
I2C_BUS                = 2        # Example: I2C bus number
I2C_SLAVE_ADDR         = 0x40     # The CCG6DF device's I2C address
FLASH_ROW_SIZE_BYTES   = 128      # Flash row size for CCG6DF
SUCCESS_CODE           = 0x02     # Example "command success" code

# -------------------------------------------------------------------
# Offsets & Opcodes
# (For demonstration; in real usage verify if HPv1 vs. HPv2 addresses)
# -------------------------------------------------------------------
DEVICE_MODE_OFFSET           = 0x0000
ENTER_FLASHING_MODE_OFFSET   = 0x000A
JUMP_TO_BOOT_OFFSET          = 0x0007
FLASH_ROW_READ_WRITE_OFFSET  = 0x000C
RESET_OFFSET                 = 0x0008
PDPORT_ENABLE_OFFSET         = 0x002C

PORT_DISABLE_OPCODE          = 0x11
PD_CONTROL_OFFSET_PORT0      = 0x1006
PD_CONTROL_OFFSET_PORT1      = 0x2006
RESPONSE_OFFSET_PORT0        = 0x1400
RESPONSE_OFFSET_PORT1        = 0x2400

FW1_METADATA_ROW             = 0xFF80
FW2_METADATA_ROW             = 0xFF00

# -------------------------------------------------------------------
# Helper functions using i2c_rdwr (no 32-byte limit)
# -------------------------------------------------------------------
def i2c_write_block_16b_offset(bus, dev_addr, register_offset_16b, data_bytes):
    """
    Write `data_bytes` to the 16-bit register offset in `register_offset_16b`.
    For offset 0xABCD, we send [0xCD, 0xAB] as the first two bytes (LS byte first).
    Then the payload (data_bytes) follows.
    We create a single i2c_msg.write() message and pass it to bus.i2c_rdwr(),
    which avoids the 32-byte limit of write_i2c_block_data.
    """
    ls_byte = register_offset_16b & 0xFF
    ms_byte = (register_offset_16b >> 8) & 0xFF

    outbuf = [ls_byte, ms_byte] + list(data_bytes)
    write_msg = i2c_msg.write(dev_addr, outbuf)
    bus.i2c_rdwr(write_msg)


def i2c_read_block_16b_offset(bus, dev_addr, register_offset_16b, num_bytes):
    """
    Read `num_bytes` from the 16-bit register offset in `register_offset_16b`.
    We do:
      1) A 'write' message of [LSB, MSB] to set the offset.
      2) A 'read' message to read back `num_bytes` bytes of data.

    We pass both messages to bus.i2c_rdwr() in order, which simulates
    a repeated start condition with no intervening stop.
    """
    ls_byte = register_offset_16b & 0xFF
    ms_byte = (register_offset_16b >> 8) & 0xFF

    # First message: write offset
    write_msg = i2c_msg.write(dev_addr, [ls_byte, ms_byte])
    # Second message: read data
    read_msg = i2c_msg.read(dev_addr, num_bytes)

    bus.i2c_rdwr(write_msg, read_msg)
    return list(read_msg)


# -------------------------------------------------------------------
# Device-Specific Commands
# -------------------------------------------------------------------
def read_device_mode(bus):
    """
    Example: read 1 byte from DEVICE_MODE_OFFSET (0x0000).
    Return the raw byte, which encodes the current device mode.
    """
    data = i2c_read_block_16b_offset(bus, I2C_SLAVE_ADDR, DEVICE_MODE_OFFSET, 1)
    return data[0]

def enter_flashing_mode(bus):
    """
    Write signature 'P' (0x50) to ENTER_FLASHING_MODE_OFFSET (0x000A).
    """
    signature = 0x50  # 'P'
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, ENTER_FLASHING_MODE_OFFSET, [signature])

def jump_to_boot(bus):
    """
    Write signature 'J' (0x4A) to JUMP_TO_BOOT_OFFSET (0x0007).
    """
    signature = 0x4A  # 'J'
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, JUMP_TO_BOOT_OFFSET, [signature])

def reset_device(bus, reset_type=1):
    """
    Write:
      Byte[0] = 'R' => 0x52
      Byte[1] = reset_type (0 => I2C reset, 1 => Device reset)
    to RESET_OFFSET (0x0008).
    """
    signature = 0x52  # 'R'
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, RESET_OFFSET, [signature, reset_type & 0xFF])

def flash_row_read_write(bus, row_number, data_to_write=None):
    """
    Demonstrate a "Flash row read/write" command:
      Byte[0]: 'F'(0x46) => signature
      Byte[1]: command => (0=read, 1=write)
      Byte[2..3]: row_number (LSB, MSB)
      If writing, append the entire row data (128 bytes).
    """
    if data_to_write is not None:
        if len(data_to_write) != FLASH_ROW_SIZE_BYTES:
            raise ValueError("Data must match the FLASH_ROW_SIZE_BYTES.")
        # Compose the command buffer
        cmd_buf = [
            0x46,            # 'F'
            0x01,            # Write
            row_number & 0xFF,
            (row_number >> 8) & 0xFF
        ] + list(data_to_write)

        i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, FLASH_ROW_READ_WRITE_OFFSET, cmd_buf)

    else:
        # Read
        cmd_buf = [
            0x46,
            0x00,            # 0 => Read
            row_number & 0xFF,
            (row_number >> 8) & 0xFF
        ]
        i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, FLASH_ROW_READ_WRITE_OFFSET, cmd_buf)
        # Next, read the data from the data memory or from the same offset if doc says so.
        # We'll try to read 128 bytes from the same offset:
        return i2c_read_block_16b_offset(bus, I2C_SLAVE_ADDR, FLASH_ROW_READ_WRITE_OFFSET, FLASH_ROW_SIZE_BYTES)

def disable_pd_ports(bus):
    """
    Example: Write PORT_DISABLE_OPCODE (0x11) to PD_CONTROL_OFFSET_PORT0 (0x1006).
    Check your device doc for actual procedure to disable PD ports.
    """
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, PD_CONTROL_OFFSET_PORT0, [PORT_DISABLE_OPCODE])
    # Potentially do the same for port1 if needed:
    # i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, PD_CONTROL_OFFSET_PORT1, [PORT_DISABLE_OPCODE])

def validate_firmware(bus):
    """
    If your device requires writing something to a Validate FW register, do so here.
    Placeholder.
    """
    pass


# -------------------------------------------------------------------
# Main Firmware Update Flow
# -------------------------------------------------------------------
def update_firmware_ccg6df_example(hex_file_path):
    """
    Example:
      1. Check device mode; if in FW mode => disable PD ports, jump to boot
      2. Enter flashing mode
      3. Clear FW metadata row
      4. Program each row from the IntelHex
      5. Validate (optional)
      6. Reset

    Using i2c_rdwr to allow writes > 32 bytes in a single transaction.
    """
    print("Opening I2C bus:", I2C_BUS, "Device address:", hex(I2C_SLAVE_ADDR))
    bus = smbus2.SMBus(I2C_BUS)

    try:
        # 1. Read device mode
        mode_before = read_device_mode(bus)
        print("Current device mode (raw):", hex(mode_before))

        # If in FW mode => disable PD ports => jump to boot
        print("Disabling PD ports...")
        disable_pd_ports(bus)

        print("Jumping to bootloader mode...")
        jump_to_boot(bus)

        # Wait/poll for boot or "Reset Complete" event in real usage
        import time
        time.sleep(0.2)

        # 2. Check if in boot mode now
        mode_boot = read_device_mode(bus)
        print("Device mode after jump:", hex(mode_boot))

        # 3. Enter flashing mode
        print("Entering flashing mode...")
        enter_flashing_mode(bus)
        time.sleep(0.05)

        # 4. Clear FW1 metadata row
        print(f"Clearing FW1 metadata row at 0x{FW1_METADATA_ROW:04X} ...")
        zero_row = [0x00] * FLASH_ROW_SIZE_BYTES
        flash_row_read_write(bus, FW1_METADATA_ROW, zero_row)
        time.sleep(0.05)

        # 5. Program each row from the IntelHex file
        print(f"Parsing HEX file: {hex_file_path}")
        ih = IntelHex(hex_file_path)

        start_addr = ih.minaddr()
        end_addr   = ih.maxaddr()

        print("Min address in HEX:", hex(ih.minaddr()))
        print("Max address in HEX:", hex(ih.maxaddr()))

        # Enforce a maximum address limit to match device flash size (e.g., 64 KB)
        MAX_FLASH_ADDRESS = 0xFFFF  # 64 KB for CCG6DF
        if end_addr > MAX_FLASH_ADDRESS:
            print(
                f"Warning: HEX file contains addresses beyond device's flash range. Limiting to 0x{MAX_FLASH_ADDRESS:04X}.")
            end_addr = MAX_FLASH_ADDRESS

        # Round up to next multiple of FLASH_ROW_SIZE_BYTES
        end_aligned = ((end_addr + FLASH_ROW_SIZE_BYTES) // FLASH_ROW_SIZE_BYTES) * FLASH_ROW_SIZE_BYTES

        print(f"Programming from 0x{start_addr:04X} to 0x{end_addr:04X} in {FLASH_ROW_SIZE_BYTES}-byte rows.")
        for base_addr in range(start_addr, end_aligned, FLASH_ROW_SIZE_BYTES):
            row_data = []
            for offset in range(FLASH_ROW_SIZE_BYTES):
                addr = base_addr + offset
                if addr > end_addr:
                    row_data.append(0xFF)  # fill up
                else:
                    row_data.append(ih[addr] & 0xFF)

            row_num = base_addr // FLASH_ROW_SIZE_BYTES
            print(f"Writing row #{row_num}, flash offset: 0x{base_addr:04X}")
            flash_row_read_write(bus, row_num, row_data)
            time.sleep(0.01)

        # 6. Validate firmware if needed
        print("Validating firmware (placeholder)...")
        validate_firmware(bus)
        time.sleep(0.05)

        # 7. Reset to run new firmware
        print("Resetting device to run new firmware...")
        reset_device(bus, 1)
        print("Firmware update sequence complete.")

    finally:
        bus.close()


# -------------------------------------------------------------------
# Example usage if run directly
# -------------------------------------------------------------------
if __name__ == "__main__":
    firmware_hex_path = "/home/firas/Documents/CYPD6228/NEW.hex"
    print("Starting firmware update for CCG6DF device (two-byte offset, i2c_rdwr for >32 bytes).")
    update_firmware_ccg6df_example(firmware_hex_path)
    print("Done.")
