#!/usr/bin/env python3
"""
Example script to update the firmware on an Infineon CCG6DF device via I2C (HPI).
It:
  1) Checks current DEVICE_MODE (FW vs. Bootloader).
  2) If in firmware mode, disables the PD port and jumps to bootloader.
  3) Enters flashing mode.
  4) Parses a provided .hex firmware file using intelhex.
  5) Erases metadata (optional step).
  6) Writes each flash row from the .hex file.
  7) (Optional) read-verifies each row.
  8) Validates and resets to load new firmware.

Requires:
  pip install smbus2 intelhex
"""

import time
import argparse
import smbus2

from intelhex import IntelHex

# -------------------------
# I2C/HPI Configuration
# -------------------------
I2C_BUS = 2  # e.g., /dev/i2c-2
CCG_I2C_SLAVE_ADDR = 0x40  # CCG6DF address on that bus

bus = smbus2.SMBus(I2C_BUS)

# -------------------------
# Register Offsets & Values
# (Example placeholders!)
# -------------------------
DEVICE_MODE_OFFSET = 0x0000  # Where to read device mode: FW vs Bootloader - O.K.
PD_CONTROL_OFFSET = 0x1006  # Port-0 PD_CONTROL register (HPIv2) - O.K.
RESPONSE_OFFSET = 0x1400  # Where CCG places the response code - O.K.
EVENT_OFFSET = 0x1401  # Where CCG places asynchronous event codes (example!)
RESET_COMPLETE_EVENT = 0x80  # Example event code meaning "Reset Complete" - O.K.
SUCCESS_RESPONSE = 0x02  # "Command handled successfully" - O.K.
PORT_DISABLE_OPCODE = 0x11  # PD_CONTROL command for "Port Disable" - O.K.
JUMP_TO_BOOT_OFFSET = 0x0007  # Where to write the JUMP_TO_BOOT command - O.K
JUMP_TO_BOOT_SIGNATURE = 0xA5  # Often the signature or value needed for jump
ENTER_FLASHING_MODE_OFFSET = 0x000A  # Register to enter "Flashing Mode" - O.K.
FLASH_ROW_READ_WRITE_OFFSET = 0x000C  # Register to trigger flash read/write - O.K.
FLASH_DATA_AVAILABLE = 0x03  # Response code for "Flash Data Available" - O.K.
VALIDATE_FW_OFFSET = 0x000B  # Register to request FW validation - O.K.
RESET_OFFSET = 0x0008 # Register to force a soft reset - O.K.

DEVICE_MODE_FW = 0x01  # Example: "firmware mode" value
DEVICE_MODE_BOOTLOADER = 0x00  # Example: "bootloader mode" value

FLASH_ROW_SIZE_BYTES = 256  # Typical CCG flash row size
METADATA_ROW_NUMBER = 0xFF80 # Address of FW1 Metadata Row


# -------------------------
# Low-level I2C Helpers
# -------------------------

def i2c_write_8bit(offset_16b, data_byte):
    """
    Write a single data byte to a 16-bit offset register in big-endian order.
    """
    high = (offset_16b >> 8) & 0xFF
    low = offset_16b & 0xFF
    write_data = [high, low, data_byte]
    bus.i2c_rdwr(smbus2.i2c_msg.write(CCG_I2C_SLAVE_ADDR, write_data))


def i2c_write_multi(offset_16b, data_bytes):
    """
    Write multiple data bytes to a 16-bit offset register in big-endian order.
    E.g., used for writing entire flash-row data to "Data Memory".
    """
    high = (offset_16b >> 8) & 0xFF
    low = offset_16b & 0xFF
    write_data = [high, low] + list(data_bytes)
    bus.i2c_rdwr(smbus2.i2c_msg.write(CCG_I2C_SLAVE_ADDR, write_data))


def i2c_read(offset_16b, num_bytes=1):
    """
    Read from a 16-bit offset register in big-endian order, returning `num_bytes`.
    """
    high = (offset_16b >> 8) & 0xFF
    low = offset_16b & 0xFF

    # Write phase (set register offset)
    bus.i2c_rdwr(smbus2.i2c_msg.write(CCG_I2C_SLAVE_ADDR, [high, low]))
    # Read phase
    read_msg = smbus2.i2c_msg.read(CCG_I2C_SLAVE_ADDR, num_bytes)
    bus.i2c_rdwr(read_msg)

    return list(read_msg)


def read_response():
    """
    Read the single-byte 'response code' from the RESPONSE_OFFSET register.
    """
    resp = i2c_read(RESPONSE_OFFSET, 1)
    if resp:
        return resp[0]
    return None


def read_event():
    """
    Read the single-byte 'event code' from the EVENT_OFFSET (example usage).
    Some designs store asynchronous events in a queue, adapt as needed.
    """
    ev = i2c_read(EVENT_OFFSET, 1)
    if ev:
        return ev[0]
    return None


# -------------------------
# High-level Steps
# -------------------------

def check_device_mode():
    """Return the current device mode read from DEVICE_MODE_OFFSET."""
    val = i2c_read(DEVICE_MODE_OFFSET, 1)
    if val:
        return val[0]
    return None


def disable_port():
    """
    Disable the PD port using PD_CONTROL (opcode=0x11).
    """
    i2c_write_8bit(PD_CONTROL_OFFSET, PORT_DISABLE_OPCODE)
    time.sleep(0.01)
    return read_response()


def jump_to_bootloader():
    """
    Trigger JUMP_TO_BOOT by writing a signature to JUMP_TO_BOOT_OFFSET.
    Then wait for RESET_COMPLETE or ~10ms.
    """
    i2c_write_8bit(JUMP_TO_BOOT_OFFSET, JUMP_TO_BOOT_SIGNATURE)
    for _ in range(10):
        ev_code = read_event()
        if ev_code == RESET_COMPLETE_EVENT:
            break
        time.sleep(0.001)
    else:
        # If event not found, forcibly wait ~10ms
        time.sleep(0.01)


def enter_flashing_mode():
    """
    Example of writing a byte to enable 'Flashing Mode'.
    """
    i2c_write_8bit(ENTER_FLASHING_MODE_OFFSET, 0x01)  # Hypothetical 'enable' value
    time.sleep(0.01)


def clear_firmware_metadata():
    """
    Clears the metadata row in flash memory:
    1. Fill data memory with zeros.
    2. Use FLASH_ROW_READ_WRITE to write that buffer.
    3. Wait for success.
    """
    zero_buffer = [0x00] * FLASH_ROW_SIZE_BYTES

    # 1) Write zero-buffer into data memory (assume offset 0x2000 for data memory).
    i2c_write_multi(0x2000, zero_buffer)

    # 2) Trigger flash write to the metadata row
    row_low = (METADATA_ROW_NUMBER & 0xFF)
    row_high = (METADATA_ROW_NUMBER >> 8) & 0xFF
    write_cmd = [0x01, row_low, row_high]  # "0x01" = 'WRITE' (example)
    i2c_write_multi(FLASH_ROW_READ_WRITE_OFFSET, write_cmd)

    # 3) Wait for success
    wait_for_success("Clearing firmware metadata")


def wait_for_success(action_label=""):
    """
    Poll the response register for SUCCESS or an error code.
    Add a time limit to avoid infinite loops.
    """
    for _ in range(100):
        resp = read_response()
        if resp == SUCCESS_RESPONSE:
            print(f"{action_label}: SUCCESS (0x02).")
            return
        elif resp not in (None, 0x00):
            print(f"{action_label}: Error response 0x{resp:02X}")
            raise RuntimeError("Command failed.")
        time.sleep(0.01)
    raise TimeoutError(f"{action_label}: No success response within timeout.")


def program_flash_row(row_number, data_array):
    """
    Writes (and optionally verifies) one row’s worth of data to flash.
    """
    # 1) Copy data to data memory
    i2c_write_multi(0x2000, data_array)  # Example offset for data memory

    # 2) Trigger write
    row_low = (row_number & 0xFF)
    row_high = (row_number >> 8) & 0xFF
    write_cmd = [0x01, row_low, row_high]  # "0x01" = 'WRITE'
    i2c_write_multi(FLASH_ROW_READ_WRITE_OFFSET, write_cmd)

    # 3) Wait for success
    wait_for_success(f"Programming row 0x{row_number:04X}")

    # (Optional) read-verify:
    # 4a) Trigger read
    read_cmd = [0x02, row_low, row_high]  # "0x02" = 'READ'
    i2c_write_multi(FLASH_ROW_READ_WRITE_OFFSET, read_cmd)

    # 4b) Wait for 'Flash Data Available'
    for _ in range(100):
        resp = read_response()
        if resp == FLASH_DATA_AVAILABLE:
            break
        elif resp not in (None, 0x00):
            print(f"Read-verify error: 0x{resp:02X}")
            raise RuntimeError("Flash read request failed.")
        time.sleep(0.01)
    else:
        raise TimeoutError("Did not get FLASH_DATA_AVAILABLE in time.")

    # 4c) Read back from data memory
    read_back = i2c_read(0x2000, len(data_array))

    # 4d) Verify
    if read_back != data_array:
        raise RuntimeError(f"Flash verify mismatch on row 0x{row_number:04X}")
    print(f"Row 0x{row_number:04X} verified OK.")


def validate_firmware():
    """
    Request that the new firmware be validated using VALIDATE_FW register.
    """
    i2c_write_8bit(VALIDATE_FW_OFFSET, 0x01)  # e.g., 'start validation'
    wait_for_success("Firmware Validate")


def reset_device():
    """
    Trigger a reset so that the new firmware will load.
    """
    i2c_write_8bit(RESET_OFFSET, 0x01)  # e.g., 'reset'
    time.sleep(0.1)


# -------------------------
# Firmware Parsing
# -------------------------

def parse_intel_hex_file(hex_path):
    """
    Parse the .hex file using the 'IntelHex' library.
    Returns a dict { row_number: [256 bytes] } for each row that needs programming.

    Adjust the row size as appropriate for your device (256 is typical for CCG).
    Also adjust the row numbering vs. the actual memory map in the device.
    """
    ih = IntelHex(hex_path)

    # Find min & max addresses
    min_addr, max_addr = ih.minaddr(), ih.maxaddr()

    # Build a dictionary mapping row_number -> data (256 bytes each)
    row_data_map = {}

    for addr in range(min_addr, max_addr + 1):
        byte_value = ih[addr]

        # Example: row_number is address // 256
        row_number = addr // FLASH_ROW_SIZE_BYTES
        offset_in_row = addr % FLASH_ROW_SIZE_BYTES

        if row_number not in row_data_map:
            # Initialize 256 bytes with 0xFF (typical default for empty flash)
            row_data_map[row_number] = [0xFF] * FLASH_ROW_SIZE_BYTES

        row_data_map[row_number][offset_in_row] = byte_value

    return row_data_map


# -------------------------
# Main Flow
# -------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Update firmware on CCG6DF device over I2C (HPI)."
    )
    parser.add_argument(
        "--firmware",
        type=str,
        required=True,
        help="Path to the Intel HEX firmware file (e.g., .hex).",
    )
    args = parser.parse_args()

    firmware_path = args.firmware
    print(f"Firmware File: {firmware_path}")

    # 1) Check current DEVICE_MODE
    print("1) Check DEVICE_MODE register...")
    mode = check_device_mode()
    print(f"   Current mode = 0x{mode:02X}")

    # 2) If in FW mode, disable port + jump to boot
    if mode == DEVICE_MODE_FW:
        print("   Device in FIRMWARE mode.")

        print("2a) Disable PD port (opcode 0x11).")
        resp = disable_port()
        if resp != SUCCESS_RESPONSE:
            raise RuntimeError(f"Port disable failed with code 0x{resp:02X}")

        print("2c) Initiate JUMP_TO_BOOT command...")
        jump_to_bootloader()

        print("2d) Wait for RESET_COMPLETE event or ~10 ms.")
        time.sleep(0.01)

    else:
        print("   Device NOT in FIRMWARE mode (already in bootloader?). Skipping steps 2a-2d.")

    # 3) Verify device is in bootloader mode
    print("3) Verify device is in bootloader mode...")
    mode = check_device_mode()
    if mode != DEVICE_MODE_BOOTLOADER:
        raise RuntimeError(f"Device did not enter bootloader mode (got 0x{mode:02X}).")
    print("   Device is in bootloader mode.")

    # 4) Enter flashing mode
    print("4) Enter flashing mode...")
    enter_flashing_mode()

    # 5) Clear firmware metadata
    print("5) Clear firmware metadata...")
    clear_firmware_metadata()

    # 6) Parse the firmware file and program rows
    print("6) Parsing firmware file...")
    row_data_map = parse_intel_hex_file("/home/firas/Documents/CYPD6228/CYPD6228-96BZXI_notebook_dualapp_usb4_3_5_1_4_0_0_1_nb.hex")
    print(f"   Parsed {len(row_data_map)} flash rows from {firmware_path}.")

    print("   Programming new firmware rows...")
    for row_num in sorted(row_data_map.keys()):
        data_array = row_data_map[row_num]
        program_flash_row(row_num, data_array)

    # 7) Validate new firmware
    print("7) Validate new firmware...")
    validate_firmware()

    # 8) Reset device
    print("8) Reset device to load new firmware...")
    reset_device()
    print("Done. Device should now boot into the new firmware.")


if __name__ == "__main__":
    try:
        main()
    finally:
        bus.close()
