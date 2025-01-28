#!/usr/bin/env python3

import os
import smbus2
import time
from smbus2 import i2c_msg
from intelhex import IntelHex

# -------------------------------------------------------------------
# I2C Bus / Address / Constants
# -------------------------------------------------------------------
I2C_BUS                = 2       # Example: I2C bus number
#I2C_SLAVE_ADDR         = 0x42    # The CCG6DF device's I2C address

# if SWD Clock IO's state is HIGH Slave Address = 0x42
# if SWD Clock IO's state is LOW, Slave Address = 0x40
# if SWD Clock IO is floating, Slave Address = 0x08

FLASH_ROW_SIZE_BYTES   = 64      # Flash row size for CCG6DF
SUCCESS_CODE           = 0x02    # Example "command success" code

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

FW1_METADATA_ROW             = 0xFF80 # Row 1023 in Hex File
FW2_METADATA_ROW             = 0xFF00 # Row 1021 in Hex File


"""
HPIv1 Method:

1. Check the DEVICE_MODE register.
2. If CCG device is in FIRMWARE mode:
    a. Disable the PD port using the Port Disable command 
    b. Wait for SUCCESS response 
    c. Initiate a JUMP_TO_BOOT command 
    d. Wait for a RESET_COMPLETE event (or ~10 ms delay). 
3. Read DEVICE_MODE register and verify that device is in bootloader mode. 
4. Initiate flashing mode entry using ENTER_FLASHING_MODE register.
5. Clear the firmware metadata in flash memory
    a. Fill the data memory with zeros.
    b. Use the FLASH_ROW_READ_WRITE register to trigger a write of the “zero” buffer into the metadata flash row.
    c. Wait for a SUCCESS response.
6. For each flash row to be updated:
    a. Copy the data into the data memory.
    b. Use the FLASH_ROW_READ_WRITE register to trigger writing of data to the desired flash row.
    c. Wait for a SUCCESS response.
    d. If read-verify is required:
   	 i. Use the FLASH_ROW_READ_WRITE register to trigger reading of data from the desired flash row.
   	 ii. Wait for a FLASH_DATA_AVAILABLE response from CCG.
   	 iii. Read the data from the data memory and verify.
7. Use VALIDATE_FW register to request the new firmware to be validated.
Use the RESET register to go through a fresh start-up cycle which will load the new firmware binary.
"""

# -------------------------------------------------------------------
# Helper functions using i2c_rdwr (no 32-byte limit)
# -------------------------------------------------------------------
def i2c_write_block_16b_offset(bus, ccg_slave_address, register_offset_16b, data_bytes):
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
    write_msg = i2c_msg.write(ccg_slave_address, outbuf)
    bus.i2c_rdwr(write_msg)


def i2c_read_block_16b_offset(bus, ccg_slave_address, register_offset_16b, num_bytes):
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
    write_msg = i2c_msg.write(ccg_slave_address, [ls_byte, ms_byte])
    # Second message: read data
    read_msg = i2c_msg.read(ccg_slave_address, num_bytes)

    bus.i2c_rdwr(write_msg, read_msg)
    return list(read_msg)


# -------------------------------------------------------------------
# Device-Specific Commands
# -------------------------------------------------------------------
def read_device_mode(bus, ccg_slave_address):
    """
    Example: read 1 byte from DEVICE_MODE_OFFSET (0x0000).
    Return the raw byte, which encodes the current device mode.
    """
    data = i2c_read_block_16b_offset(bus, ccg_slave_address, DEVICE_MODE_OFFSET, 1)
    return data[0]

def enter_flashing_mode(bus, ccg_slave_address):
    """
    Write signature 'P' (0x50) to ENTER_FLASHING_MODE_OFFSET (0x000A).
    """
    signature = 0x50  # 'P'
    i2c_write_block_16b_offset(bus, ccg_slave_address, ENTER_FLASHING_MODE_OFFSET, [signature])

def jump_to_boot(bus, ccg_slave_address):
    """
    Write signature 'J' (0x4A) to JUMP_TO_BOOT_OFFSET (0x0007).
    """
    signature = 0x4A  # 'J'
    i2c_write_block_16b_offset(bus, ccg_slave_address, JUMP_TO_BOOT_OFFSET, [signature])

def reset_device(bus, reset_type=1, ccg_slave_address=None):
    """
    Write:
      Byte[0] = 'R' => 0x52
      Byte[1] = reset_type (0 => I2C reset, 1 => Device reset)
    to RESET_OFFSET (0x0008).
    """
    signature = 0x52  # 'R'
    i2c_write_block_16b_offset(bus, ccg_slave_address, RESET_OFFSET, [signature, reset_type & 0xFF])

def flash_row_read_write(bus, row_number, data_to_write=None, ccg_slave_address=None):
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

        i2c_write_block_16b_offset(bus, ccg_slave_address, FLASH_ROW_READ_WRITE_OFFSET, cmd_buf)

    else:
        # Read
        cmd_buf = [
            0x46,
            0x00,            # 0 => Read
            row_number & 0xFF,
            (row_number >> 8) & 0xFF
        ]
        i2c_write_block_16b_offset(bus, ccg_slave_address, FLASH_ROW_READ_WRITE_OFFSET, cmd_buf)
        # Read 64 bytes of data
        return i2c_read_block_16b_offset(bus, ccg_slave_address, FLASH_ROW_READ_WRITE_OFFSET, FLASH_ROW_SIZE_BYTES)

def disable_pd_ports(bus, ccg_slave_address):
    """
    Example: Write PORT_DISABLE_OPCODE (0x11) to PD_CONTROL_OFFSET_PORT0 (0x1006).
    Check your device doc for actual procedure to disable PD ports.
    """
    i2c_write_block_16b_offset(bus, ccg_slave_address, PD_CONTROL_OFFSET_PORT0, [PORT_DISABLE_OPCODE])
    i2c_write_block_16b_offset(bus, ccg_slave_address, PD_CONTROL_OFFSET_PORT1, [PORT_DISABLE_OPCODE])

def validate_firmware(bus):
    """
    If your device requires writing something to a Validate FW register, do so here.
    Placeholder.
    """
    pass

def reset_device_startup(bus, ccg_slave_address):
    """
    Resets the device at the start of the script to ensure it's in a known state.
    """
    print("Resetting device...")
    reset_command = [0x52, 0x01]  # 'R' + reset_type (1 = Device Reset)
    i2c_write_block_16b_offset(bus, ccg_slave_address, RESET_OFFSET, reset_command)
    print("Device reset command sent.")

    # Wait for the device to stabilize
    import time
    time.sleep(0.5)

def read_response_register_port0(bus, ccg_slave_address):
    """
    Read the response register at RESPONSE_OFFSET_PORT0 (0x1400).
    Returns the data read from the register.
    """
    print("Reading response register at RESPONSE_OFFSET_PORT0...")
    data = i2c_read_block_16b_offset(bus, ccg_slave_address, RESPONSE_OFFSET_PORT0, 4)  # Example: read 4 bytes
    return data[0]

def check_for_success_response(bus, ccg_slave_address, operation_description):
    """
    Reads the response register at RESPONSE_OFFSET_PORT0.
    If the response equals SUCCESS_CODE, prints success;
    otherwise, prints an error.

    :param ccg_slave_address:
    :param bus: The smbus2.SMBus instance
    :param operation_description: String describing the operation we are verifying
    :return: True if success, False otherwise
    """
    print(f"Checking response for operation: {operation_description}")
    response = i2c_read_block_16b_offset(bus, ccg_slave_address, RESPONSE_OFFSET_PORT0, 1)
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
def update_firmware_ccg6df_example(hex_file_path, ccg_slave_address):

    print("Opening I2C bus:", I2C_BUS, "Device address:", hex(ccg_slave_address))
    bus = smbus2.SMBus(I2C_BUS)

    try:
        # 1. Read DEVICE_MODE Register:
        # if b1-b0 in pre_boot_device_mode are 10 then we are in FW2 and register value will be = 0x86 = 1000 0110
        # if b1-b0 in pre_boot_device_mode are 01 then we are in FW1 and register value will be = 0x85 = 1000 0101
        # if b1-b0 in pre_boot_device_mode are 00 then we are in BTL and register value will be = 0x84 = 1000 0100
        pre_boot_device_mode = read_device_mode(bus, ccg_slave_address)

        if pre_boot_device_mode == 0x85:
            print("Current device mode: 0x85 => FW1")
        elif pre_boot_device_mode == 0x86:
            print("Current device mode: 0x86 => FW2")
        else:
            print(f"ERROR: Device mode 0x{pre_boot_device_mode:02X} is not FW1 or FW2. Aborting.")
            return

        # 2. If CCG device is in FIRMWARE Mode:
        # 2a. Disable THE PD port using the Port-Disable Command
        print("Disabling PD ports...")
        disable_pd_ports(bus, ccg_slave_address)

        time.sleep(0.2)

        # 2.b: Wait for "SUCCESS" response:
        if not check_for_success_response(bus, ccg_slave_address,"Disable PD ports"):
            print("ERROR: Disabling PD ports did not return success. Aborting update.")
            return
        else:
            print("PD ports disabled successfully. SUCCESS Response Received")

        # 2.c Initiate JUMP_TO_BOOT command
        jump_to_boot(bus, ccg_slave_address)

        # 2.d Wait for REST_COMPLETE event (or ~10ms Delay)
        time.sleep(0.2)

        # 3. Read DEVICE_MODE register and verify that device is in bootloader mode.
        post_boot_device_mode = read_device_mode(bus, ccg_slave_address)
        if post_boot_device_mode == 0x84:
            print("Current device mode: 0x84 => Bootloader")
        else:
            print(f"ERROR: Device mode 0x{post_boot_device_mode:02X} is not Bootloader. Aborting.")
            return

        # 4.Initiate flashing mode entry using ENTER_FLASHING_MODE register.
        #print("Entering flashing mode...")
        enter_flashing_mode(bus, ccg_slave_address)
        time.sleep(0.1)

        # 5. Clear the firmware metadata in flash memory
        # 5.a Fill the data memory with zeros.
        # 5.b Use the FLASH_ROW_READ_WRITE register to trigger a write of the “zero” buffer into the metadata flash row.
        if pre_boot_device_mode == 0x86:
            # 0x86 => FW2
            print(f"Clearing FW2 metadata row at 0x{FW2_METADATA_ROW:04X} ...")
            zero_row = [0x00] * FLASH_ROW_SIZE_BYTES
            flash_row_read_write(bus, FW2_METADATA_ROW, zero_row, ccg_slave_address) # Row 1021 is zeroes (0xFF00)
            time.sleep(0.1)
            #flash_row_read_write(bus, FW2_METADATA_ROW + 1, zero_row, ccg_slave_address) # Row 1022 has data (0xFF40)
            time.sleep(0.1)
            if not check_for_success_response(bus, ccg_slave_address, "Clearing FW Metadata"):
                print("ERROR: Clearing FW Metadata did not return success. Aborting update.")
                return
            else:
                print("FW Metadata cleared successfully. SUCCESS Response Received")


        elif pre_boot_device_mode == 0x85:
            # 0x85 => FW1
            print(f"Clearing FW1 metadata row at 0x{FW1_METADATA_ROW:04X} ...")
            zero_row = [0x00] * FLASH_ROW_SIZE_BYTES
            flash_row_read_write(bus, FW1_METADATA_ROW, zero_row, ccg_slave_address) # Row 1023 is zeroes (0xFF80)
            time.sleep(0.1)
            #flash_row_read_write(bus, FW1_METADATA_ROW + 1, zero_row, ccg_slave_address) # Row 1024 has data (0xFFC0)
            time.sleep(0.1)
            if not check_for_success_response(bus, ccg_slave_address, "Clearing FW Metadata"):
                print("ERROR: Clearing FW Metadata did not return success. Aborting update.")
                return
            else:
                print("FW Metadata cleared successfully. SUCCESS Response Received")


        else:
            print(f"ERROR: pre_boot_device_mode = 0x{pre_boot_device_mode:02X} is not FW1 or FW2, cannot clear metadata.")
            return

        # 5.c Wait for a SUCCESS response after clearing FW Metadata.

        """
        time.sleep(0.1)
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


        # 6. Validate firmware if needed
        print("Validating firmware (placeholder)...")
        validate_firmware(bus)
        time.sleep(0.1)

        # 7. Reset to run new firmware
        print("Resetting device to run new firmware...")
        reset_device(bus, 1)
        print("Firmware update sequence complete.")
    """
    finally:
        bus.close()


# -------------------------------------------------------------------
# Example usage if run directly
# -------------------------------------------------------------------
if __name__ == "__main__":
    firmware_hex_path = "/home/firas/Documents/CYPD6228/CYPD6228-96BZXI_notebook_dualapp_usb4_228_2.hex"

    # We'll try CCG6DF I2C_SLAVE_ADDRESS 0x40 first, then 0x42 if 0x40 fails.
    possible_addresses = [0x42, 0x40]

    for addr in possible_addresses:
        print(f"\nAttempting firmware update for CCG6DF at address 0x{addr:02X}...")
        try:
            # Call our function *with* the chosen address:
            update_firmware_ccg6df_example(firmware_hex_path, ccg_slave_address=addr)

            # If we got this far, let's assume success or normal completion:
            print("Firmware update attempt completed (check logs for success/failure).")
            # If you wish to break on success, you can check some success condition or just break:
            break

        except Exception as e:
            # If an uncaught error/exception occurs, we print and move on to the next address
            print(f"Error with address 0x{addr:02X}: {e}. Trying next address...\n")

            time.sleep(0.2)

    print("Done.")
