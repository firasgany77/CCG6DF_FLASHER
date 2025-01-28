#!/usr/bin/env python3
"""
Example code to update the firmware on a CCG6DF device, using a two-byte register offset
(LS byte first, MS byte second) and i2c_rdwr to handle writes bigger than 32 bytes.

This version has been enhanced to demonstrate each step of the required HPIv2 procedure
for updating the firmware, including strict commenting and detailed print statements
to show exactly what is happening at each stage.

Firmware Update Steps for HPIv2:
--------------------------------
1. Check the Device Mode
2. Select the Correct Firmware Binary (for demonstration, we use a given HEX file)
3. Enter Flashing Mode
4. Clear Metadata for the Selected Firmware Slot
5. Update the Firmware
   5a. (Optional) Verification after each row write
6. Validate the Firmware
7. Activate the New Firmware
   (Disable PD Ports -> Wait for Success -> Reset the device -> Wait for RESET_COMPLETE)

All addresses, constants, and partial opcodes are from the original request.
"""

import os
import time
import smbus2
from smbus2 import i2c_msg
from intelhex import IntelHex

# -------------------------------------------------------------------
# I2C Bus / Address / Constants
# -------------------------------------------------------------------
I2C_BUS = 2  # Example: I2C bus number
I2C_SLAVE_ADDR = 0x40  # The CCG6DF device's I2C address
FLASH_ROW_SIZE_BYTES = 64  # Flash row size for CCG6DF
SUCCESS_CODE = 0x02  # Example "command success" code

# -------------------------------------------------------------------
# Offsets & Opcodes
# -------------------------------------------------------------------
DEVICE_MODE_OFFSET = 0x0000
ENTER_FLASHING_MODE_OFFSET = 0x000A
JUMP_TO_BOOT_OFFSET = 0x0007
FLASH_ROW_READ_WRITE_OFFSET = 0x000C
RESET_OFFSET = 0x0800
PDPORT_ENABLE_OFFSET = 0x002C

PORT_DISABLE_OPCODE = 0x11
PD_CONTROL_OFFSET_PORT0 = 0x1006
PD_CONTROL_OFFSET_PORT1    = 0x2006  # Not used, per request
RESPONSE_OFFSET_PORT0 = 0x1400
RESPONSE_OFFSET_PORT1      = 0x2400  # Not used, per request

FW1_METADATA_ROW = 0xFF80
FW2_METADATA_ROW = 0xFF00  # Example second metadata row


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
# Utility to check for SUCCESS response after a write operation
# -------------------------------------------------------------------
def check_for_success_response(bus, operation_description):
    """
    Reads the response register at RESPONSE_OFFSET_PORT0.
    If the response equals SUCCESS_CODE, prints success;
    otherwise, prints an error.

    :param bus: The smbus2.SMBus instance
    :param operation_description: String describing the operation we are verifying
    :return: True if success, False otherwise
    """
    print(f"Checking response for operation: {operation_description}")
    response = i2c_read_block_16b_offset(bus, I2C_SLAVE_ADDR, RESPONSE_OFFSET_PORT0, 1)
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
# Device-Specific Commands
# -------------------------------------------------------------------
def read_device_mode(bus):
    """
    Read 1 byte from DEVICE_MODE_OFFSET (0x0000).
    Return the raw byte, which encodes the current device mode.
    """
    data = i2c_read_block_16b_offset(bus, I2C_SLAVE_ADDR, DEVICE_MODE_OFFSET, 1)
    return data[0] if data else None


def enter_flashing_mode(bus):
    """
    Write signature 'P' (0x50) to ENTER_FLASHING_MODE_OFFSET (0x000A).
    """
    signature = 0x50  # 'P'
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, ENTER_FLASHING_MODE_OFFSET, [signature])


def jump_to_boot(bus):
    """
    Write signature 'J' (0x4A) to JUMP_TO_BOOT_OFFSET (0x0007).
    This instructs the device to jump to bootloader mode.
    """
    signature = 0x4A  # 'J'
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, JUMP_TO_BOOT_OFFSET, [signature])


def reset_device(bus, reset_type=1):
    """
    Write:
      Byte[0] = 'R' => 0x52
      Byte[1] = reset_type (0 => I2C reset, 1 => Device reset)
    to RESET_OFFSET (0x0800).

    This triggers a device reset.
    """
    signature = 0x52  # 'R'
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, RESET_OFFSET, [signature, reset_type & 0xFF])


def flash_row_read_write(bus, row_number, data_to_write=None):
    """
    Flash row read/write command:
      Byte[0]: 'F'(0x46) => signature
      Byte[1]: command => (0=read, 1=write)
      Byte[2..3]: row_number (LSB, MSB)
      If writing, append the entire row data (64 bytes) to the command buffer.

    If data_to_write is None, we perform a read operation and return the 64 bytes read.
    """
    if data_to_write is not None:
        # Perform a write
        if len(data_to_write) != FLASH_ROW_SIZE_BYTES:
            raise ValueError(
                f"Data length {len(data_to_write)} must match FLASH_ROW_SIZE_BYTES ({FLASH_ROW_SIZE_BYTES})."
            )

        cmd_buf = [
                      0x46,  # 'F' signature
                      0x01,  # 1 => Write
                      row_number & 0xFF,
                      (row_number >> 8) & 0xFF
                  ] + list(data_to_write)

        i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, FLASH_ROW_READ_WRITE_OFFSET, cmd_buf)
        return None
    else:
        # Perform a read
        cmd_buf = [
            0x46,  # 'F' signature
            0x00,  # 0 => Read
            row_number & 0xFF,
            (row_number >> 8) & 0xFF
        ]
        i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, FLASH_ROW_READ_WRITE_OFFSET, cmd_buf)

        # After issuing the read command, we read back 64 bytes of data from the same offset.
        data_read = i2c_read_block_16b_offset(bus, I2C_SLAVE_ADDR, FLASH_ROW_READ_WRITE_OFFSET, FLASH_ROW_SIZE_BYTES)
        return data_read


def disable_pd_ports(bus):
    """
    Write PORT_DISABLE_OPCODE (0x11) to PD_CONTROL_OFFSET_PORT0 (0x1006)
    to disable PD ports on the device. Only port0 is relevant here.
    """
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, PD_CONTROL_OFFSET_PORT0, [PORT_DISABLE_OPCODE])
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, PD_CONTROL_OFFSET_PORT1, [PORT_DISABLE_OPCODE])

def validate_firmware(bus):
    """
    If your device requires writing something to a 'Validate FW' register, do so here.
    Currently a placeholder for demonstration.
    """
    print("Placeholder: Validate firmware (if needed).")


def reset_device_startup(bus):
    """
    Resets the device at the start of the script to ensure it's in a known state.
    """
    print("Performing device reset at script startup...")
    reset_command = [0x52, 0x01]  # 'R' + reset_type (1 = Device Reset)
    i2c_write_block_16b_offset(bus, I2C_SLAVE_ADDR, RESET_OFFSET, reset_command)
    print("Device reset command sent. Waiting 0.5s for device to stabilize...")
    time.sleep(0.5)


# -------------------------------------------------------------------
# Main Firmware Update Flow
# -------------------------------------------------------------------
def update_firmware_ccg6df_example(hex_file_path):
    """
    Demonstrates a step-by-step firmware update procedure for the CCG6DF device using HPIv2.

    Steps:
    1. Check the device mode (DEVICE_MODE register).
    2. Select the correct firmware (in this example, we assume the provided HEX is correct).
    3. Enter Flashing Mode (ENTER_FLASHING_MODE register).
    4. Clear metadata for the selected firmware slot.
    5. Update the firmware by writing each row from the HEX file.
       - Optionally read back each row for verification.
       - Always wait for "SUCCESS" after each row write.
    6. Validate the firmware (VALIDATE_FW register) - placeholder.
    7. Activate the new firmware:
       a. Disable the PD ports,
       b. Wait for "SUCCESS",
       c. Perform a device reset (RESET register),
       d. Wait for a reset complete event (not explicitly shown here; you can poll the device_mode).
    """
    print(f"\n--- Starting firmware update procedure for CCG6DF ---\n")
    print(f"Using I2C bus #{I2C_BUS}, device address 0x{I2C_SLAVE_ADDR:02X}")

    bus = smbus2.SMBus(I2C_BUS)

    try:
        # Step 0: (Optional) Reset device to ensure known state
        reset_device_startup(bus)

        # Step 1: Check the current device mode
        device_mode_value = read_device_mode(bus)
        if device_mode_value is None:
            print("ERROR: Could not read device mode. Aborting update.")
            return
        print(f"Current device mode (raw byte): 0x{device_mode_value:02X}")

        # (Example) If the device is in FW mode, we disable PD ports and jump to boot
        # In a real situation, you might check a specific bit or value that indicates "FW mode"
        print("Disabling PD ports...")
        disable_pd_ports(bus)
        # Wait for success response from disabling PD ports
        if not check_for_success_response(bus, "disable_pd_ports"):
            print("Aborting due to error disabling PD ports.")
            return

        print("Jumping to bootloader mode...")
        jump_to_boot(bus)
        # Wait for success response from jump to boot
        if not check_for_success_response(bus, "jump_to_boot"):
            print("Aborting due to error jumping to bootloader.")
            return

        # Wait a short time for the device to actually enter bootloader mode
        time.sleep(0.2)

        # Read device mode again to confirm (optional)
        mode_boot = read_device_mode(bus)
        print(f"Device mode after jump (raw byte): 0x{mode_boot:02X}")

        # Step 3: Enter flashing mode
        print("Entering flashing mode...")
        enter_flashing_mode(bus)
        if not check_for_success_response(bus, "enter_flashing_mode"):
            print("Aborting due to error entering flashing mode.")
            return
        time.sleep(0.1)

        # Step 4: Clear metadata for the selected firmware slot
        # (Here we demonstrate clearing FW2_METADATA_ROW or FW1_METADATA_ROW as needed.)
        # For demonstration, let's assume we are updating the "FW2" slot and want to clear FW2's metadata:
        selected_metadata_row = FW2_METADATA_ROW
        print(f"Clearing metadata row at 0x{selected_metadata_row:04X} by writing 64 bytes of 0x00...")
        zero_row = [0x00] * FLASH_ROW_SIZE_BYTES
        flash_row_read_write(bus, selected_metadata_row, zero_row)

        # Wait for success response after clearing metadata
        if not check_for_success_response(bus, "clear_metadata"):
            print("Aborting due to error clearing metadata.")
            return

        # (Optional) We can read back the row to verify it is indeed cleared (commented out):
        """
        print("Reading back metadata row to verify zeroed data...")
        read_back = flash_row_read_write(bus, selected_metadata_row, None)
        if read_back:
            first_8_bytes = read_back[:8]
            print(f"First 8 bytes read back: {first_8_bytes}")
        """

        # Step 5: Update the firmware
        print(f"\n--- Updating firmware from HEX file: {hex_file_path} ---")
        ih = IntelHex(hex_file_path)
        start_addr = ih.minaddr()
        end_addr = ih.maxaddr()

        print(f"HEX file address range: 0x{start_addr:04X} to 0x{end_addr:04X}")

        # Enforce a maximum address limit to match device's flash size, e.g. 64KB
        MAX_FLASH_ADDRESS = 0xFFFF
        if end_addr > MAX_FLASH_ADDRESS:
            print(
                f"Warning: HEX file contains addresses beyond 0x{MAX_FLASH_ADDRESS:04X}. "
                "Truncating to device's max flash range."
            )
            end_addr = MAX_FLASH_ADDRESS

        INCREMENT = FLASH_ROW_SIZE_BYTES  # 64
        print(f"Programming in increments of 0x{INCREMENT:02X} bytes per flash row.\n")

        for base_addr in range(start_addr, end_addr + 1, INCREMENT):
            row_data = []
            is_empty = True  # Detect if this row is fully 0xFF or 0x00 and skip if truly empty

            for offset in range(INCREMENT):
                addr = base_addr + offset
                if addr > end_addr:
                    # If beyond the file's end, pad with 0xFF or 0x00 as needed
                    row_data.append(0xFF)
                else:
                    val = ih[addr] & 0xFF
                    row_data.append(val)
                    if val != 0x00:
                        # Mark row as non-empty if we find any non-zero byte
                        is_empty = False

            # Optionally skip truly empty rows
            # (If your device's default state is 0xFF in flash, skip only if all 0xFF)
            # Here, let's skip if all bytes are 0xFF:
            if all(b == 0xFF for b in row_data):
                print(f"Skipping empty row at flash offset: 0x{base_addr:04X} (all 0xFF).")
                continue

            # Each row in flash is identified by row_number = (base_addr // 0x40)
            row_num = base_addr // INCREMENT
            print(f"Writing row #{row_num} (flash offset: 0x{base_addr:04X}).")
            flash_row_read_write(bus, row_num, row_data)

            # Step 5a: Wait for "SUCCESS" response after each row write
            if not check_for_success_response(bus, f"write row #{row_num}"):
                print("Aborting due to error writing flash row.")
                return

            # (Optional) Verification step: read back and compare with the original data
            """
            print(f"Reading back row #{row_num} for verification...")
            verified_data = flash_row_read_write(bus, row_num, None)
            # Wait for success response from read operation
            if not check_for_success_response(bus, f"read row #{row_num}"):
                print("Aborting due to error reading back flash row.")
                return

            if verified_data != row_data:
                print(f"ERROR: Verification failed for row #{row_num}!")
                return
            else:
                print(f"Row #{row_num} verification PASSED.")
            """

            # Brief delay (if needed)
            time.sleep(0.01)

        # Step 6: Validate the firmware (placeholder)
        print("\nValidating firmware (if required by device)...")
        validate_firmware(bus)
        # If there's a success response for validate, check it:
        # For demonstration, we assume a success response is not mandatory here.

        # Step 7: Activate the new firmware
        # 7a. Disable PD ports
        print("\nDisabling PD ports prior to reset (activate new firmware)...")
        disable_pd_ports(bus)
        if not check_for_success_response(bus, "disable_pd_ports"):
            print("Aborting due to error disabling PD ports before final reset.")
            return

        # 7b. Perform a device reset to start running new firmware
        print("Resetting device to activate new firmware...")
        reset_device(bus, 1)
        # If there's a success response for reset, check it:
        # Some devices respond before actually resetting, so the check might be optional.
        # We'll do it for demonstration:
        check_for_success_response(bus, "device_reset")

        print("\n--- Firmware update sequence complete. Device should now be running the new firmware. ---\n")

    finally:
        # Always close the bus
        bus.close()


# -------------------------------------------------------------------
# Example usage if run directly
# -------------------------------------------------------------------
if __name__ == "__main__":
    firmware_hex_path = "/home/firas/Documents/CYPD6228/CYPD6228-96BZXI_notebook_dualapp_usb4_228_2.hex"

    # Perform an initial reset outside the update logic, purely as an example
    print("Performing initial reset before starting firmware update...")
    bus = smbus2.SMBus(I2C_BUS)
    reset_device_startup(bus)
    bus.close()

    print("\nNow starting firmware update for CCG6DF device.")
    update_firmware_ccg6df_example(firmware_hex_path)

    print("\nDone.\n")
