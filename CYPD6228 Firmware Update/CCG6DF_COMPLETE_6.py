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
import time
import random
import smbus2
from smbus2 import i2c_msg
from intelhex import IntelHex

# -------------------------------------------------------------------
# I2C Bus / Address / Constants
# -------------------------------------------------------------------
I2C_BUS                = 2        # Example: I2C bus number
I2C_SLAVE_ADDR         = 0x40     # The CCG6DF device's I2C address
FLASH_ROW_SIZE_BYTES   = 64       # Flash row size for CCG6DF
SUCCESS_CODE           = 0x02     # Example "command success" code

# -------------------------------------------------------------------
# Offsets & Opcodes
# -------------------------------------------------------------------
DEVICE_MODE_OFFSET           = 0x0000
ENTER_FLASHING_MODE_OFFSET   = 0x000A
JUMP_TO_BOOT_OFFSET          = 0x0007
FLASH_ROW_READ_WRITE_OFFSET  = 0x000C
RESET_OFFSET                 = 0x0800  # The reset offset that works
PDPORT_ENABLE_OFFSET         = 0x002C
VALIDATE_FW_OFFSET           = 0x000B  # Validate FW register offset
FIRMWARE1_START              = 0x0500  # Start of FW region in flash
INTR_REG                     = 0x0006  # Where interrupts are cleared

PORT_DISABLE_OPCODE          = 0x11
PORT_ENABLE_OPCODE           = 0x10
PD_CONTROL_OFFSET_PORT0      = 0x1006
PD_CONTROL_OFFSET_PORT1      = 0x2006
RESPONSE_OFFSET_PORT0        = 0x1400
RESPONSE_OFFSET_PORT1        = 0x2400

FW1_METADATA_ROW             = 0xFFC0
FW2_METADATA_ROW             = 0xFF40

# -------------------------------------------------------------------
# Low-level I2C read/write helpers
# -------------------------------------------------------------------
def i2c_write_block_16b_offset(bus, dev_addr, register_offset_16b, data_bytes):
    """Write `data_bytes` to the 16-bit register offset in `register_offset_16b`."""
    ls_byte = register_offset_16b & 0xFF
    ms_byte = (register_offset_16b >> 8) & 0xFF
    outbuf  = [ls_byte, ms_byte] + list(data_bytes)
    write_msg = i2c_msg.write(dev_addr, outbuf)
    bus.i2c_rdwr(write_msg)

def i2c_read_block_16b_offset(bus, dev_addr, register_offset_16b, num_bytes):
    """Read `num_bytes` from the 16-bit register offset in `register_offset_16b`."""
    ls_byte = register_offset_16b & 0xFF
    ms_byte = (register_offset_16b >> 8) & 0xFF
    write_msg = i2c_msg.write(dev_addr, [ls_byte, ms_byte])
    read_msg  = i2c_msg.read(dev_addr, num_bytes)
    bus.i2c_rdwr(write_msg, read_msg)
    return list(read_msg)

# -------------------------------------------------------------------
# HPI / Bootloader commands
# -------------------------------------------------------------------
def read_device_mode(bus):
    data = i2c_read_block_16b_offset(bus, I2C_SLAVE_ADDR, DEVICE_MODE_OFFSET, 1)
    return data[0]

def enter_flashing_mode(bus):
    signature = 0x50  # 'P'
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, ENTER_FLASHING_MODE_OFFSET, [signature])

def jump_to_boot(bus):
    signature = 0x4A  # 'J'
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, JUMP_TO_BOOT_OFFSET, [signature])

def reset_device(bus, reset_type=1):
    signature = 0x52  # 'R'
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, RESET_OFFSET, [signature, reset_type & 0xFF])
    time.sleep(0.5)

def flash_row_read_write(bus, row_number, data_to_write=None):
    if data_to_write is not None:
        if len(data_to_write) != FLASH_ROW_SIZE_BYTES:
            raise ValueError(f"Data must be {FLASH_ROW_SIZE_BYTES} bytes.")
        cmd_buf = [0x46, 0x01,
                   row_number & 0xFF,
                   (row_number >> 8) & 0xFF] + list(data_to_write)
        i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, FLASH_ROW_READ_WRITE_OFFSET, cmd_buf)
    else:
        cmd_buf = [0x46, 0x00,
                   row_number & 0xFF,
                   (row_number >> 8) & 0xFF]
        i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, FLASH_ROW_READ_WRITE_OFFSET, cmd_buf)
        return i2c_read_block_16b_offset(bus, I2C_SLAVE_ADDR, FLASH_ROW_READ_WRITE_OFFSET, FLASH_ROW_SIZE_BYTES)

def disable_pd_ports(bus):
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, PD_CONTROL_OFFSET_PORT0, [PORT_DISABLE_OPCODE])
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, PD_CONTROL_OFFSET_PORT1, [PORT_DISABLE_OPCODE])

def validate_firmware(bus, which_fw=1):
    print(f"Validating firmware slot {which_fw} via VALIDATE_FW_OFFSET (0x000B)...")
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, VALIDATE_FW_OFFSET, [which_fw & 0xFF])
    time.sleep(0.1)

# -------------------------------------------------------------------
# NEW: Read & Clear INTR_REG to properly de-assert interrupt
# -------------------------------------------------------------------
def read_and_clear_intr_reg(bus):
    """Reads INTR_REG (1 byte) and writes back any set bits to clear them."""
    intr_val_list = i2c_read_block_16b_offset(bus, I2C_SLAVE_ADDR, INTR_REG, 1)
    if intr_val_list:
        intr_val = intr_val_list[0]
        if intr_val != 0:
            print(f"[Interrupt] INTR_REG read => 0x{intr_val:02X}, clearing it...")
            # Write back the same bits to clear them
            i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, INTR_REG, [intr_val])
            time.sleep(0.01)
        else:
            print("[Interrupt] INTR_REG read => 0x00, no interrupt bits set.")
    else:
        print("[Interrupt] Could not read INTR_REG (no data returned).")

# -------------------------------------------------------------------
# Checking response with interrupt clearing
# -------------------------------------------------------------------
def check_for_success_response(bus, operation_description):
    """
    1) Read & clear INTR_REG first to ensure we are in de-asserted state.
    2) Read the response register at RESPONSE_OFFSET_PORT0 to see if it is SUCCESS_CODE.
    3) Clear any new interrupt bits that appear.
    """
    print(f"Checking response for operation: {operation_description}")

    # NEW: Clear any stale interrupt bits from a previous operation
    read_and_clear_intr_reg(bus)

    # Now read the response register
    resp = i2c_read_block_16b_offset(bus, I2C_SLAVE_ADDR, RESPONSE_OFFSET_PORT0, 1)
    if not resp:
        print("ERROR: No data read from response register.")
        return False

    val = resp[0]
    print(f"Response register value: 0x{val:02X}")

    # NEW: Clear the interrupt after reading the response
    read_and_clear_intr_reg(bus)

    if val == SUCCESS_CODE:
        print(f"Operation '{operation_description}' succeeded (0x{val:02X}).")
        return True
    else:
        print(f"Operation '{operation_description}' failed/unexpected code (0x{val:02X}).")
        return False

# -------------------------------------------------------------------
# Main Firmware Update Flow
# -------------------------------------------------------------------
def update_firmware_ccg6df_example(hex_file_path):
    print("Opening I2C bus:", I2C_BUS, "Device address:", hex(I2C_SLAVE_ADDR))
    bus = smbus2.SMBus(I2C_BUS)

    try:
        # 1) Read device mode
        mode_before = read_device_mode(bus)
        print("Current device mode (raw):", hex(mode_before))

        # 2) If in FW mode => disable PD ports => jump to boot
        print("Disabling PD ports...")
        disable_pd_ports(bus)
        time.sleep(0.3)

        # Check success
        if not check_for_success_response(bus, "Disabling PD Port"):
            print("Aborting. Could not disable PD ports.")
            return

        print("Jumping to bootloader mode...")
        jump_to_boot(bus)
        time.sleep(0.3)

        mode_boot = read_device_mode(bus)
        print("Device mode after jump:", hex(mode_boot))
        if mode_boot != 0x84:
            print("ERROR: not in bootloader mode. Aborting.")
            return

        # 3) Enter flashing mode
        print("Entering flashing mode...")
        enter_flashing_mode(bus)
        time.sleep(0.3)
        # (Optionally) check success
        # check_for_success_response(bus, "Enter Flashing Mode")

        # 4) Clear FW1 metadata row => row # = 0xFFC0 // 64 = 0x3FF
        print(f"Clearing FW1 metadata row at 0x{FW1_METADATA_ROW:04X} ...")
        zero_row = [0x00] * FLASH_ROW_SIZE_BYTES
        meta_row_num = FW1_METADATA_ROW // 64
        flash_row_read_write(bus, meta_row_num, zero_row)
        time.sleep(0.1)

        if not check_for_success_response(bus, "Clearing FW1 metadata row"):
            print("Warning: Could not clear FW1 metadata row or no success code returned.")

        # 5) Program each row from the IntelHex file in [0x0500..0x287F]
        ih = IntelHex(hex_file_path)
        start_addr = FIRMWARE1_START
        end_addr   = 0x2840 + 0x3F  # 0x287F

        row_size = 64
        for base_addr in range(start_addr, end_addr+1, row_size):
            block_data = []
            for offset in range(row_size):
                addr = base_addr + offset
                if addr > end_addr:
                    block_data.append(0xFF)
                else:
                    block_data.append(ih[addr])

            row_num = (base_addr - FIRMWARE1_START) // row_size
            row_hex = [f"0x{b:02X}" for b in block_data]
            print(f"\nProgramming row #{row_num} at base=0x{base_addr:04X} with data:\n{row_hex}")

            flash_row_read_write(bus, row_num, block_data)
            time.sleep(0.05)

            # (Optional) check success
            if not check_for_success_response(bus, f"Programming row #{row_num}"):
                print("Aborting. Row write didn't succeed.")
                return

        # 6) Validate firmware (FW1 => 0x01)
        validate_firmware(bus, which_fw=1)
        if not check_for_success_response(bus, "Firmware Validation"):
            print("Warning: Firmware validation not acknowledged as success...")

        # 7) Reset device
        print("\nResetting device to run new firmware...")
        reset_device(bus, 1)
        time.sleep(0.3)

        mode_after = read_device_mode(bus)
        print(f"Device mode after reset: 0x{mode_after:02X}")
        if mode_after == 0x85:
            print("Device is now in FW1 mode (0x85). Success!")
        elif mode_after == 0x86:
            print("Device is now in FW2 mode (0x86). Possibly a dual-image scenario.")
        else:
            print("Device is not in FW mode. Possibly still in bootloader or an error occurred.")

    finally:
        bus.close()

# -------------------------------------------------------------------
# If run as main
# -------------------------------------------------------------------
if __name__ == "__main__":
    firmware_hex_path = "/home/firas/Documents/CYPD6228/CYPD6228-96BZXI_notebook_dualapp_usb4_228_2.hex"
    print("Starting firmware update for CCG6DF device.")
    update_firmware_ccg6df_example(firmware_hex_path)
    print("Done.")
