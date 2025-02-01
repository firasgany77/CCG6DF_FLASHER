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
import time
from smbus2 import i2c_msg
from intelhex import IntelHex
import random

# How to enable:
# 1. Use 0x42 Slave Address.
# 2. Connect Mini-Prog4.
# 3. Patch.
# 4. Run.

# -------------------------------------------------------------------
# I2C Bus / Address / Constants
# -------------------------------------------------------------------
I2C_BUS                = 2       # Example: I2C bus number
I2C_SLAVE_ADDR         = 0x40    # The CCG6DF device's I2C address
FLASH_ROW_SIZE_BYTES   = 64      # Flash row size for CCG6DF
SUCCESS_CODE           = 0x02     # Example "command success" code

# -------------------------------------------------------------------
# Offsets & Opcodes
# -------------------------------------------------------------------
DEVICE_MODE_OFFSET           = 0x0000
ENTER_FLASHING_MODE_OFFSET   = 0x000A
JUMP_TO_BOOT_OFFSET          = 0x0007
FLASH_ROW_READ_WRITE_OFFSET  = 0x000C
RESET_OFFSET                 = 0x0800 # 0x0800 the one that works
PDPORT_ENABLE_OFFSET         = 0x002C
VALIDATE_FW_OFFSET           = 0x000B
FIRMWARE1_START              = 0x0500

PORT_DISABLE_OPCODE          = 0x11
PORT_ENABLE_OPCODE           = 0x10
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

    0xABCD = 1010 1011 1100 1101
    0xFF   = 0000 0000 1111 1111

    0xABCD & 0xFF = 1100 1101 = 0xCD = LS_BYTE
    (0xABCD >> 8) ---> move all bit 8 times to the right =  0000 0000 1010 1011
    (0xABCD >> 8) & 0xFF  = 1010 1011 = 0xAB = MS_BYTE

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
    """Write:
      Byte[0] = 'R' => 0x52
      Byte[1] = reset_type (0 => I2C reset, 1 => Device reset)
    to RESET_OFFSET (0x0008)."""

    signature = 0x52  # 'R'
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, RESET_OFFSET, [signature, reset_type & 0xFF])
    time.sleep(0.5)

def flash_row_read_write(bus, row_number, data_to_write=None):
    """
    Flash row read/write command:
      Byte[0]: 'F'(0x46) => signature
      Byte[1]: command => (0=read, 1=write)
      Byte[2..3]: row_number (LSB, MSB)
      If writing, append the entire row data (64 bytes).
    """
    if data_to_write is not None:
        if len(data_to_write) != FLASH_ROW_SIZE_BYTES:
            raise ValueError(f"Data must match the FLASH_ROW_SIZE_BYTES ({FLASH_ROW_SIZE_BYTES}).")
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
        # Read 64 bytes of data
        return i2c_read_block_16b_offset(bus, I2C_SLAVE_ADDR, FLASH_ROW_READ_WRITE_OFFSET, FLASH_ROW_SIZE_BYTES)

def disable_pd_ports(bus):
    """
    Example: Write PORT_DISABLE_OPCODE (0x11) to PD_CONTROL_OFFSET_PORT0 (0x1006).
    Check your device doc for actual procedure to disable PD ports.
    """
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, PD_CONTROL_OFFSET_PORT0, [PORT_DISABLE_OPCODE])
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, PD_CONTROL_OFFSET_PORT1, [PORT_DISABLE_OPCODE])


def enable_pd_ports(bus):
    """
    Example: Write PORT_DISABLE_OPCODE (0x11) to PD_CONTROL_OFFSET_PORT0 (0x1006).
    Check your device doc for actual procedure to disable PD ports.
    """
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, PDPORT_ENABLE_OFFSET, [0x01])
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, PDPORT_ENABLE_OFFSET, [0x11])
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, PDPORT_ENABLE_OFFSET, [0x03])
def validate_firmware(bus):
    """
    If your device requires writing something to a Validate FW register, do so here.
    Placeholder.
    """
    pass


def read_response_register_port0(bus):
    """
    Read the response register at RESPONSE_OFFSET_PORT0 (0x1400).
    Returns the data read from the register.
    """
    print("Reading response register at RESPONSE_OFFSET_PORT0...")
    data = i2c_read_block_16b_offset(bus, I2C_SLAVE_ADDR, RESPONSE_OFFSET_PORT0, 4)  # Example: read 4 bytes
    return data[0]

def check_for_success_response(bus, operation_description):
    """
    Reads the response register at RESPONSE_OFFSET_PORT0.
    If the response equals SUCCESS_CODE, prints success;
    otherwise, prints an error.
    """
    print(f"Checking response for operation: {operation_description}")
    response = i2c_read_block_16b_offset(bus,I2C_SLAVE_ADDR, RESPONSE_OFFSET_PORT0, 1)
    if not response:
        print("ERROR: No data read from response register.")
        return False

    resp_val = response[0]
    print(f"Response register value: 0x{resp_val:02X}")

    if resp_val == SUCCESS_CODE:
        print(f"Operation '{operation_description}' succeeded (0x{resp_val:02X}).")
        return True
    else:
        print(f"Operation '{operation_description}' failed or returned unexpected code (0x{resp_val:02X}).")
        return False

# -------------------------------------------------------------------
# Main Firmware Update Flow
# -------------------------------------------------------------------
def update_firmware_ccg6df_example(hex_file_path):
    print("Opening I2C bus:", I2C_BUS, "Device address:", hex(I2C_SLAVE_ADDR))
    bus = smbus2.SMBus(I2C_BUS)

    try:
        # 1. Read device mode
        mode_before = read_device_mode(bus)
        print("Current device mode (raw):", hex(mode_before))

        # Disable PD ports, jump to boot, etc.
        print("Disabling PD ports...")
        disable_pd_ports(bus)
        time.sleep(0.6)
        if not check_for_success_response(bus, f"Disabling PD Port #{0}"):
            print("Aborting due to error writing flash row.")
            return

        print("Jumping to bootloader mode...")
        jump_to_boot(bus)
        time.sleep(0.2)

        # Confirm mode=0x84
        if read_device_mode(bus) != 0x84:
            print("Error: not in boot mode.")
            return

        # Enter flashing mode
        enter_flashing_mode(bus)
        time.sleep(0.1)

        # 4) Clear FW1 metadata row => write 64 bytes of 0x00
        print(f"Clearing FW1 metadata row at 0x{FW1_METADATA_ROW:04X} ...")
        zero_row = [0x00] * FLASH_ROW_SIZE_BYTES
        # The row index is FW1_METADATA_ROW / 64
        meta_row_num = FW1_METADATA_ROW // 64
        flash_row_read_write(bus, meta_row_num, zero_row)
        time.sleep(0.1)
        if not check_for_success_response(bus, "Clearing FW1 metadata row"):
            print("Warning: Clearing metadata row did not return success. Continuing anyway.")

        # Parse the HEX file
        ih = IntelHex()
        ih.loadhex(firmware_hex_path)
        start_addr = FIRMWARE1_START
        end_addr = 0x2840 + (64-1) # = 0x287F
        # Processes the entire row from 0x2840 through 0x287F (64 bytes total),

        row_size = 64

        for base_addr in range(start_addr, end_addr + 1, row_size):
            # Build 64-byte block
            block_data = []
            for offset in range(row_size):
                addr = base_addr + offset
                if addr > end_addr:
                    block_data.append(0xFF)
                else:
                    block_data.append(ih[addr])

            row_num = (base_addr - FIRMWARE1_START) // row_size

            # NEW: Print the 64 bytes in hex
            row_hex = [f"0x{b:02X}" for b in block_data]
            print(f"Programming row #{row_num} (base=0x{base_addr:04X}) with data:\n", row_hex)
            # End new line

            flash_row_read_write(bus, row_num, block_data)
            # Optionally wait/check success here

        # (Optional) Validate FW if needed

        # reset, check mode, etc.
        reset_device(bus, 1)
        time.sleep(0.3)
        mode = read_device_mode(bus)
        print(f"Device mode after reset: 0x{mode:02X}")

    finally:
        bus.close()
# -------------------------------------------------------------------
# Example usage if run directly
# -------------------------------------------------------------------
if __name__ == "__main__":
    firmware_hex_path = "/home/firas/Documents/CYPD6228/CYPD6228-96BZXI_notebook_dualapp_usb4_228_2.hex"
    bus = smbus2.SMBus(I2C_BUS)

    #reset_device(bus, 1)
    #print("Reset device from update_firmware_ccg6df_ex")

    time.sleep(0.6)

    print("Starting firmware update for CCG6DF device (two-byte offset, i2c_rdwr for >32 bytes).")
    update_firmware_ccg6df_example(firmware_hex_path)

    time.sleep(0.6)

    #print("Enable PD ports...")
    #enable_pd_ports(bus)

    #reset_device(bus, 1)
    #print("Device Reset")

    time.sleep(1)

    mode = read_device_mode(bus)
    print(f"Post-reset device mode: 0x{mode:02X}")

    #i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, PD_CONTROL_OFFSET_PORT0, [0x11])
    #i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, PD_CONTROL_OFFSET_PORT0, [0x10])
    #i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, PD_CONTROL_OFFSET_PORT0, [0x01])

    # Hard Reset and Soft Reset through PD_CONTROL:
    #i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, PD_CONTROL_OFFSET_PORT0, [0x0D]) #HARD RESET
    #i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, PD_CONTROL_OFFSET_PORT0, [0x0E]) #SOFT RESET

    print("Done.")