#!/usr/bin/env python3
"""
Example code to update the firmware on a CCG6DF device, using a two-byte register offset
(LS byte first, MS byte second) and i2c_rdwr to handle writes bigger than 32 bytes.

It uses:
- smbus2 for I2C transactions
- intelhex for parsing the HEX file
...
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
I2C_SLAVE_ADDR         = 0x42     # The CCG6DF device's I2C address
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

PORT_DISABLE_OPCODE          = 0x11
PORT_ENABLE_OPCODE           = 0x10
PD_CONTROL_OFFSET_PORT0      = 0x1006
PD_CONTROL_OFFSET_PORT1      = 0x2006
RESPONSE_OFFSET_PORT0        = 0x1400
RESPONSE_OFFSET_PORT1        = 0x2400

FW1_METADATA_ROW             = 0xFFC0
FW2_METADATA_ROW             = 0xFF40

# -------------------------------------------------------------------
# Helper functions (same as your code) ...
# -------------------------------------------------------------------
def i2c_write_block_16b_offset(bus, dev_addr, register_offset_16b, data_bytes):
    ls_byte = register_offset_16b & 0xFF
    ms_byte = (register_offset_16b >> 8) & 0xFF
    outbuf = [ls_byte, ms_byte] + list(data_bytes)
    write_msg = i2c_msg.write(dev_addr, outbuf)
    bus.i2c_rdwr(write_msg)

def i2c_read_block_16b_offset(bus, dev_addr, register_offset_16b, num_bytes):
    ls_byte = register_offset_16b & 0xFF
    ms_byte = (register_offset_16b >> 8) & 0xFF
    write_msg = i2c_msg.write(dev_addr, [ls_byte, ms_byte])
    read_msg  = i2c_msg.read(dev_addr, num_bytes)
    bus.i2c_rdwr(write_msg, read_msg)
    return list(read_msg)

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
    """
    Example: Write `which_fw` to VALIDATE_FW_OFFSET (0x000B).
    If FW1 => 1, FW2 => 2, etc. (Check docs for your device.)
    """
    # Some devices accept 0x01 => FW1, 0x02 => FW2. Implementation may vary.
    print(f"Validating firmware slot {which_fw} via VALIDATE_FW_OFFSET (0x000B)...")
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, VALIDATE_FW_OFFSET, [which_fw & 0xFF])
    time.sleep(0.1)

def check_for_success_response(bus, operation_description):
    print(f"Checking response for operation: {operation_description}")
    resp = i2c_read_block_16b_offset(bus, I2C_SLAVE_ADDR, RESPONSE_OFFSET_PORT0, 1)
    if not resp:
        print("ERROR: No data read from response register.")
        return False
    val = resp[0]
    print(f"Response register value: 0x{val:02X}")
    if val == SUCCESS_CODE:
        print(f"Operation '{operation_description}' succeeded (0x{val:02X}).")
        return True
    else:
        print(f"Operation '{operation_description}' failed or returned unexpected code (0x{val:02X}).")
        return False

def check_response_code(bus, ccg_slave_address):
    """
    Reads the response register at RESPONSE_OFFSET_PORT0 and returns the numeric code.
    Always prints the code, but does NOT decide success/failure anymore.
    """
    response_bytes = i2c_read_block_16b_offset(bus, ccg_slave_address, PD_CONTROL_OFFSET_PORT0, 1)
    if not response_bytes:
        print("WARNING: No data read from response register. Returning 0xFF.")
        return 0xFF

    resp_val = response_bytes[0]
    return resp_val
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

        # 2) Disable PD ports, jump to boot
        print("Disabling PD ports...")
        disable_pd_ports(bus)
        time.sleep(0.6)
        if not check_for_success_response(bus, "Disabling PD Port #0"):
            print("Aborting. Could not disable PD ports.")
            return

        print("Jumping to bootloader mode...")
        jump_to_boot(bus)
        time.sleep(0.2)
        mode_boot = read_device_mode(bus)
        print("Device mode after jump:", hex(mode_boot))

        print("Jumping to Alt-FW...")
        i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, JUMP_TO_BOOT_OFFSET, [0x41]) # Signature: A
        time.sleep(0.2)

        # Confirm bootloader mode = 0x84
        mode_boot = read_device_mode(bus)
        print("Device mode after jump:", hex(mode_boot))


        # 3) Enter flashing mode
        print("Entering flashing mode...")
        enter_flashing_mode(bus)
        time.sleep(0.1)
        # (Optionally check for success response here if needed)

        # 4) Clear FW1 metadata row => write 64 bytes of 0x00
        print(f"Clearing FW1 metadata row at 0x{FW2_METADATA_ROW:04X} ...")
        zero_row = [0x00] * FLASH_ROW_SIZE_BYTES
        # The row index is FW1_METADATA_ROW / 64
        meta_row_num = FW2_METADATA_ROW // 64
        flash_row_read_write(bus, meta_row_num, zero_row)
        time.sleep(0.1)
        #response_code = check_response_code(bus, I2C_SLAVE_ADDR)
        #print(f"Response code after zeroing FW1_METADATA_ROW = 0x{response_code:02X}")


        # 5) Program each row from the IntelHex file
        ih = IntelHex()
        ih.loadhex(hex_file_path)

        start_addr = FIRMWARE1_START
        end_addr   = 0x2840 + (64 - 1)  # = 0x287F
        # Processes the entire row from 0x2840 through 0x287F (64 bytes total),
        row_size   = 64

        for base_addr in range(start_addr, end_addr + 1, row_size):
            block_data = []
            for offset in range(row_size):
                addr = base_addr + offset
                if addr > end_addr:
                    block_data.append(0xFF)
                else:
                    block_data.append(ih[addr])

            row_num = (base_addr - FIRMWARE1_START) // row_size

            # Print the 64 bytes in hex
            row_hex = [f"0x{b:02X}" for b in block_data]
            print(f"\nProgramming row #{row_num} at base=0x{base_addr:04X} with data:\n", row_hex)

            flash_row_read_write(bus, row_num, block_data)
            time.sleep(0.01)  # minimal delay
            # Optionally check response

        # 6) Validate firmware (assuming FW1 => pass 1)
        validate_firmware(bus, which_fw=1)
        #if not check_for_success_response(bus, "Firmware Validation"):
        #    print("Warning: Firmware validation did not succeed. Device may not boot from FW1.")

        # 7) Reset device
        print("\nResetting device to run the new firmware...")
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


if __name__ == "__main__":
    firmware_hex_path = "/home/firas/Documents/CYPD6228/CYPD6228-96BZXI_notebook_dualapp_usb4_228_2.hex"

    print("Starting firmware update for CCG6DF device.")
    update_firmware_ccg6df_example(firmware_hex_path)

    print("Done.")
