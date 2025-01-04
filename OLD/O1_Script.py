#!/usr/bin/env python3
"""
Example script to update the firmware on an Infineon CCG6DF device via I2C (HPI).
Requires:
  pip install smbus2
"""

import time
import smbus2

# -------------------------
# I2C/HPI Configuration
# -------------------------
I2C_BUS = 2  # e.g., /dev/i2c-1
CCG_I2C_SLAVE_ADDR = 0x40  # Example I2C slave address for the CCG6DF
bus = smbus2.SMBus(I2C_BUS)

# -------------------------
# Register Offsets & Values
# (Example placeholders!)
# -------------------------
DEVICE_MODE_OFFSET = 0x0002  # Where to read if device is in FW or Bootloader mode
PD_CONTROL_OFFSET = 0x1006  # Port-0 PD_CONTROL register (HPIv2)
RESPONSE_OFFSET = 0x1400  # Where the CCG places the response code
EVENT_OFFSET = 0x1401  # Where the CCG places asynchronous event codes (example!)
RESET_COMPLETE_EVENT = 0x80  # Example event code meaning "Reset Complete"
SUCCESS_RESPONSE = 0x02  # "Command handled successfully"
PORT_DISABLE_OPCODE = 0x11  # PD_CONTROL command for "Port Disable"
JUMP_TO_BOOT_OFFSET = 0x0008  # Where to write the JUMP_TO_BOOT command
JUMP_TO_BOOT_SIGNATURE = 0xA5  # Often the signature or value needed for jump
ENTER_FLASHING_MODE_OFFSET = 0x0010  # Example register to enter "Flashing Mode"
FLASH_ROW_READ_WRITE_OFFSET = 0x0012  # Example register to trigger a flash read or write
FLASH_DATA_AVAILABLE = 0x03  # Example response code for "Flash Data Available"
VALIDATE_FW_OFFSET = 0x0016  # Example register to request FW validation
RESET_OFFSET = 0x0018  # Example register to force a soft reset

DEVICE_MODE_FW = 0x01  # Example "firmware mode" value
DEVICE_MODE_BOOTLOADER = 0x00  # Example "bootloader mode" value

# Example metadata row boundaries
METADATA_ROW_NUMBER = 0x7FFE  # Example row for metadata
FLASH_ROW_SIZE_BYTES = 256  # Example row size


# -------------------------
# Low-level I2C Helpers
# -------------------------

def i2c_write_8bit(offset_16b, data_byte):
    """
    Write a single data byte to a 16-bit offset register in big-endian order.
    """
    # offset_16b => two bytes: high, low
    high = (offset_16b >> 8) & 0xFF
    low = offset_16b & 0xFF

    # We send 3 bytes total: [high_offset, low_offset, data_byte]
    write_data = [high, low, data_byte]
    bus.i2c_rdwr(
        smbus2.i2c_msg.write(CCG_I2C_SLAVE_ADDR, write_data)
    )


def i2c_write_multi(offset_16b, data_bytes):
    """
    Write multiple data bytes to a 16-bit offset register in big-endian order.
    E.g., used for writing entire flash-row data into "Data Memory".
    """
    high = (offset_16b >> 8) & 0xFF
    low = offset_16b & 0xFF

    write_data = [high, low] + list(data_bytes)
    bus.i2c_rdwr(
        smbus2.i2c_msg.write(CCG_I2C_SLAVE_ADDR, write_data)
    )


def i2c_read(offset_16b, num_bytes=1):
    """
    Read from a 16-bit offset register in big-endian order, returning `num_bytes`.
    """
    high = (offset_16b >> 8) & 0xFF
    low = offset_16b & 0xFF

    # Write phase (set register offset)
    bus.i2c_rdwr(
        smbus2.i2c_msg.write(CCG_I2C_SLAVE_ADDR, [high, low])
    )
    # Read phase
    read_msg = smbus2.i2c_msg.read(CCG_I2C_SLAVE_ADDR, num_bytes)
    bus.i2c_rdwr(read_msg)

    return list(read_msg)


def read_response():
    """
    Read the single-byte 'response code' from the known RESPONSE_OFFSET register.
    """
    resp = i2c_read(RESPONSE_OFFSET, 1)
    if resp:
        return resp[0]
    return None


def read_event():
    """
    Read the single-byte 'event code' from the known EVENT_OFFSET register (example).
    Some designs store asynchronous events in the same response register or a queue.
    Adapt as needed.
    """
    ev = i2c_read(EVENT_OFFSET, 1)
    if ev:
        return ev[0]
    return None


# -------------------------
# High-level Steps
# -------------------------

def check_device_mode():
    """Return the current device mode read from the DEVICE_MODE register."""
    val = i2c_read(DEVICE_MODE_OFFSET, 1)
    if val:
        return val[0]
    return None


def disable_port():
    """Disable the PD port using PD_CONTROL (opcode 0x11)."""
    i2c_write_8bit(PD_CONTROL_OFFSET, PORT_DISABLE_OPCODE)
    time.sleep(0.01)  # small delay
    resp_code = read_response()
    return resp_code


def jump_to_bootloader():
    """
    Trigger JUMP_TO_BOOT by writing the special signature or command
    to the JUMP_TO_BOOT_OFFSET. Then wait for RESET_COMPLETE or ~10 ms
    """
    i2c_write_8bit(JUMP_TO_BOOT_OFFSET, JUMP_TO_BOOT_SIGNATURE)
    # Wait a short time for the device to reset
    # Either poll for an event or just do a fixed delay
    for _ in range(10):
        ev_code = read_event()
        if ev_code == RESET_COMPLETE_EVENT:
            break
        time.sleep(0.001)
    else:
        # If not found event, just forcibly wait 10 ms
        time.sleep(0.01)


def enter_flashing_mode():
    """Example of writing a byte to a register that puts the device into Flashing Mode."""
    i2c_write_8bit(ENTER_FLASHING_MODE_OFFSET, 0x01)  # hypothetical 'enable' value
    time.sleep(0.01)  # wait, if necessary


def clear_firmware_metadata():
    """
    Clears the metadata row in flash memory:
    1. Fill data memory with zeros
    2. Use FLASH_ROW_READ_WRITE register to trigger the write
    3. Wait for SUCCESS
    """
    zero_buffer = [0x00] * FLASH_ROW_SIZE_BYTES
    # 1) Write the zero-buffer into data memory (assume data memory at offset 0x2000, for example).
    i2c_write_multi(0x2000, zero_buffer)

    # 2) Trigger the flash write operation by specifying the row number, direction, etc.
    #    Hypothetical format: [Command=WRITE, row_LSB, row_MSB], or something like that.
    #    We'll just guess we need to write 3 bytes: <WRITE_CMD>, <row_low>, <row_high>
    row_low = (METADATA_ROW_NUMBER & 0xFF)
    row_high = (METADATA_ROW_NUMBER >> 8) & 0xFF
    write_cmd = [0x01, row_low, row_high]  # "0x01" = 'WRITE', placeholder
    i2c_write_multi(FLASH_ROW_READ_WRITE_OFFSET, write_cmd)

    # 3) Wait for success
    wait_for_success("Clearing firmware metadata")


def wait_for_success(action_label=""):
    """
    Repeatedly read the response register until SUCCESS or error code is encountered.
    Add a timeout if desired.
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
    Writes (and optionally verifies) one row’s worth of data into flash.
    """
    # 1) Copy data to data memory
    i2c_write_multi(0x2000, data_array)  # Example data memory offset

    # 2) Trigger write
    row_low = (row_number & 0xFF)
    row_high = (row_number >> 8) & 0xFF
    write_cmd = [0x01, row_low, row_high]  # "0x01" = 'WRITE', placeholder
    i2c_write_multi(FLASH_ROW_READ_WRITE_OFFSET, write_cmd)

    # 3) Wait for success
    wait_for_success(f"Programming row 0x{row_number:04X}")

    # (Optional) read-verify
    # 4a) Trigger read
    read_cmd = [0x02, row_low, row_high]  # "0x02" = 'READ'
    i2c_write_multi(FLASH_ROW_READ_WRITE_OFFSET, read_cmd)

    # 4b) Wait for 'Flash Data Available' response code
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

    # 4c) Read the data back from data memory
    read_back = i2c_read(0x2000, len(data_array))
    # 4d) Verify
    if read_back != data_array:
        raise RuntimeError(f"Flash verify mismatch on row 0x{row_number:04X}")
    print(f"Row 0x{row_number:04X} verified OK.")


def validate_firmware():
    """
    Request the new firmware be validated using VALIDATE_FW register.
    """
    i2c_write_8bit(VALIDATE_FW_OFFSET, 0x01)  # e.g., "start validation"
    wait_for_success("Firmware Validate")


def reset_device():
    """
    Trigger a reset so that the new firmware will load.
    """
    i2c_write_8bit(RESET_OFFSET, 0x01)  # e.g., "reset"
    # Possibly wait for reset if needed
    time.sleep(0.1)


# -------------------------
# Main Flow
# -------------------------

def main():
    print("1) Check DEVICE_MODE register...")
    mode = check_device_mode()
    print(f"   Current mode = 0x{mode:02X}")

    if mode == DEVICE_MODE_FW:
        print("   Device in FIRMWARE mode.")

        print("2a) Disable the PD port (opcode 0x11).")
        resp = disable_port()
        if resp != SUCCESS_RESPONSE:
            raise RuntimeError(f"Port disable failed with code 0x{resp:02X}")

        print("2c) Initiate JUMP_TO_BOOT command...")
        jump_to_bootloader()

        print("2d) Wait for RESET_COMPLETE event or ~10 ms done.")
        time.sleep(0.01)

    else:
        print("   Device NOT in FIRMWARE mode (already in bootloader?). Skipping steps 2a-2d.")

    print("3) Verify device is in bootloader mode...")
    mode = check_device_mode()
    if mode != DEVICE_MODE_BOOTLOADER:
        raise RuntimeError(f"Device did not enter bootloader mode (got 0x{mode:02X}).")
    print("   Device is in bootloader mode.")

    print("4) Initiate flashing mode...")
    enter_flashing_mode()

    print("5) Clear the firmware metadata in flash memory...")
    clear_firmware_metadata()

    print("6) Program new firmware rows...")
    # You would typically parse a hex file or otherwise get your flash data.
    # For example, assume 'firmware_rows' is a list of (row_number, row_data):
    # This is just a mock example of two rows:
    firmware_rows = [
        (0x5000, [0x01, 0x02, 0x03] + [0xFF] * (FLASH_ROW_SIZE_BYTES - 3)),
        (0x5001, [0x10, 0x20, 0x30] + [0xFF] * (FLASH_ROW_SIZE_BYTES - 3)),
    ]
    for row_num, row_data in firmware_rows:
        program_flash_row(row_num, row_data)

    print("7) Validate new firmware...")
    validate_firmware()

    print("8) Reset device to load new firmware...")
    reset_device()
    print("Done. Device should now boot the new firmware.")


if __name__ == "__main__":
    try:
        main()
    finally:
        bus.close()
