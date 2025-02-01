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
I2C_SLAVE_ADDR         = 0x42    # The CCG6DF device's I2C address
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

def validate_firmware(bus):
    """
    If your device requires writing something to a Validate FW register, do so here.
    Placeholder.
    """
    pass

def reset_device_startup(bus):
    """
    Resets the device at the start of the script to ensure it's in a known state.
    """
    print("Resetting device...")
    reset_command = [0x52, 0x01]  # 'R' + reset_type (1 = Device Reset)
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, RESET_OFFSET, reset_command)
    print("Device reset command sent.")

    # Wait for the device to stabilize
    import time
    time.sleep(0.5)

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
        # 0. Reset device at startup
        reset_device_startup(bus)
        print("Reset device from update_firmware_ccg6df_ex")

        # (Existing code follows...)

        # 1. Read device mode
        mode_before = read_device_mode(bus)
        print("Current device mode (raw):", hex(mode_before))

        # If in FW mode => disable PD ports => jump to boot
        print("Disabling PD ports...")
        disable_pd_ports(bus)
        time.sleep(0.6) # disabling this timer will prevent entering boot mode
        # on the other hand adding it stops successful write using flash_row_read_write function. 
        if not check_for_success_response(bus,f"Disabling PD Port #{0}"):
            print("Aborting due to error writing flash row.")
            return

        print("Jumping to bootloader mode...")
        jump_to_boot(bus)
        # can't enable_jump_to_boot after flashing mode is enabled.

        # Wait/poll for boot or "Reset Complete" event in real usage
        time.sleep(0.5)

        # 2. Check if in boot mode now
        mode_boot = read_device_mode(bus)
        print("Device mode after jump:", hex(mode_boot))

        # 3. Enter flashing mode
        print("Entering flashing mode...")
        enter_flashing_mode(bus)
        time.sleep(0.1)

        #if not check_for_success_response(bus, f"Entering Flashing Mode"):
        #    print("Aborting due to error Entering Flashing Mode.")
        #    return

        # 4. Clear FW1 metadata row
        print(f"Clearing FW1 metadata row at 0x{FW1_METADATA_ROW:04X} ...")
        zero_row = [0x00] * FLASH_ROW_SIZE_BYTES
        flash_row_read_write(bus, FW2_METADATA_ROW, zero_row)

        rowNumLSB = 0x34
        rowNumMSB = 0x12
        row_data = [random.randrange(256) for _ in range(64)]
        cmd_buf = [0x46, 0x00, rowNumLSB, rowNumMSB]


        i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, FLASH_ROW_READ_WRITE_OFFSET, [0x01])

        row_number = 0x0D40
        row_data = flash_row_read_write(bus, row_number, None)

        if row_data is None:
            print("Failed to read row data.")
        else:
            # Format the bytes for easier reading
            row_hex = [f"0x{b:02X}" for b in row_data]
            print(f"64-byte read from 0x{row_number:04X} memory address:\n", row_hex)

        time.sleep(0.1)

        """
        # 5. Program each row from the IntelHex file
        print(f"Parsing HEX file: {hex_file_path}")
        ih = IntelHex(hex_file_path)

        start_addr = ih.minaddr()
        end_addr = ih.maxaddr()

        print(f"start_addr: {start_addr}")
        print(f"end_addr: {end_addr}")

        # Enforce a maximum address limit to match device flash size (e.g., 64 KB)
        MAX_FLASH_ADDRESS = 0xFFFF  # 64 KB for CCG6DF
        if end_addr > MAX_FLASH_ADDRESS:
            print(
                f"Warning: HEX file contains addresses beyond device's flash range. Limiting to 0x{MAX_FLASH_ADDRESS:04X}.")
            end_addr = MAX_FLASH_ADDRESS

        # Ensure the jumps are aligned to increments of 0x40
        INCREMENT = 0x40  # Increment size for each row
        print(f"Programming from 0x{start_addr:04X} to 0x{end_addr:04X} in increments of 0x{INCREMENT:02X}.")

        for base_addr in range(start_addr, end_addr + 1, INCREMENT):
            row_data = []
            is_empty = True  # Flag to detect empty rows

            for offset in range(INCREMENT):
                addr = base_addr + offset
                if addr > end_addr:
                    row_data.append(0xFF)  # Fill with padding if beyond end address
                else:
                    value = ih[addr] & 0xFF
                    row_data.append(value)
                    if value != 0x00:  # Check if row has meaningful data
                        is_empty = False

            if is_empty:
                # Skip flashing this row since it's empty
                print(f"Skipping empty row at flash offset: 0x{base_addr:04X}")
                continue

            # Write the row to flash
            row_num = base_addr // INCREMENT
            print(f"Writing row #{row_num}, flash offset: 0x{base_addr:04X}, data: {row_data[:8]}...")

            flash_row_read_write(bus, row_num, row_data)
            time.sleep(0.01)  # Minimal delay per row
            # Step 5a: Wait for "SUCCESS" response after each row write
            if not check_for_success_response(bus, f"write row #{row_num}"):
                print("Aborting due to error writing flash row.")
                return

        """
        # 6. Validate firmware if needed
        print("Validating firmware (placeholder)...")
        validate_firmware(bus)
        time.sleep(0.1)

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
    firmware_hex_path = "/home/firas/Documents/CYPD6228/CYPD6228-96BZXI_notebook_dualapp_usb4_228_2.hex"

    #print("Resetting device at startup...")
    #bus = smbus2.SMBus(I2C_BUS)
    #reset_device_startup(bus)
    #bus.close()

    print("Starting firmware update for CCG6DF device (two-byte offset, i2c_rdwr for >32 bytes).")
    update_firmware_ccg6df_example(firmware_hex_path)

    print("Done.")