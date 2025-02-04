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

"""
# NEW: Define the lower and upper flashing limits for partial firmware updates
LOWER_FLASHING_LIMIT   = 0x0A00  # Row 41
UPPER_FLASHING_LIMIT   = 0x1000  # Row 65  
                                 # 65 - 41 +1 = 25 
                                 # 132 - 83 + 1 = 50"""


# NEW: Define the lower and upper flashing limits for partial firmware updates
LOWER_FLASHING_LIMIT   = 0x0A00  # FW1 0x0523
UPPER_FLASHING_LIMIT   = 0x5AC0  # Row 65
                                 # 65 - 41 +1 = 25
                                 # 132 - 83 + 1 = 50  S


FLASH_ROW_SIZE_BYTES   = 64      # Flash row size for CCG6DF
SUCCESS_CODE           = 0x02    # Example "command success" code
FLASH_DATA_AVAILABLE   = 0x03    # Flash data available, flash row data requested by EC is available in data memory.


# for (5.a Fill data memory with zeroes)
# 0x0500 - 0xFEC0 covers FW1-FW2 Memory data, including Vectors and Startup-Code, Configuration Table.
#DATA_MEM_LOWER_LIMIT   = 0x0600 # FW1 START
#DATA_MEM_UPPER_LIMIT   = 0x2880 # FW2 START (End of FW1 change-allowed region)

# -------------------------------------------------------------------
# Offsets & Opcodes
# -------------------------------------------------------------------
DEVICE_MODE_OFFSET           = 0x0000
ENTER_FLASHING_MODE_OFFSET   = 0x000A
JUMP_TO_BOOT_OFFSET          = 0x0007
FLASH_ROW_READ_WRITE_OFFSET  = 0x000C
# “INVALID COMMAND = 0x05 ” if not in flashing mode or if signature or command field is invalid.
# “INVALID ARGUMENT = 0x09 " if flash row specified is invalid.
# "SUCCESS = 0x02 " if flash write operation is successful.
# "FLASH DATA AVAILABLE = 0x03" if flash read operation is successful.



FlASH_UPDATE_FAILED          = 0x07 # response code
BOOT_LOADER_LAST_ROW         = 0x0004 # number of last flash row occupied by Bootloader.
RESET_OFFSET                 = 0x0800 # 0x0800 the one that works
PDPORT_ENABLE_OFFSET         = 0x002C
FIRMWARE_BINARY_LOCATION_OFFSET = 0x0028

PORT_DISABLE_OPCODE          = 0x11
READ_WRITE_I2C_DEVICE_REGISTER = 0x3C # Opcode for HPIv2 port-0 address: 0x1006

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

"""
HPIv2 Method:

1. Check the DEVICE_MODE register.
2. Identify the firmware binary corresponding to the inactive firmware application (FW2 if FW1 is running and
vice versa).
3. Initiate flashing mode entry using ENTER_FLASHING_MODE register.
4. Clear the metadata corresponding to the firmware being updated:
   4.a Fill the data memory with zeroes.
   4.b Use the FLASH_ROW_READ_WRITE register to trigger a write of the “zero” buffer into the metadata flash row.
   4.c Wait for a SUCCESS response.
5. For each flash row to be updated:
   5.a Copy the data into the flash read/write memory.
   5.b Use the FLASH_ROW_READ_WRITE register to trigger writing of data to the desired flash row.
   5.c Wait for a SUCCESS response.
   5.d If read-verify is required:
   5.d.i Use the FLASH_ROW_READ_WRITE register to trigger reading of data from the desired flash row.
   5.d.ii. Wait for a FLASH_DATA_AVAILABLE response from CCG.
   5.d.iii Read the data from the flash read/write memory and verify.
6. Use VALIDATE_FW register to request the new firmware to be validated.
7. If new firmware has to be activated:
   7.a Use PDPORT_ENABLE register to disable PD ports.
   7.b Wait for SUCCESS response.
   7.c Use RESET register to go through a fresh start-up cycle which will load the new firmware binary.
   7.d Wait for RESET_COMPLETE event."""
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

def flash_row_read_write(bus, row_number, data_to_write=None, ccg_slave_address=None, bytes_to_read=None):
    """
    Flash row read/write command:
      Byte[0]: 'F'(0x46) => signature
      Byte[1]: command => (0=read, 1=write)
      Byte[2..3]: row_number (LSB, MSB)
      If writing, append the entire row data (64 bytes).

      In Dual Firmware Mode, flash read/writes can only be done on the inactive firmware copy.
      If Firmware1 is active, flash access is limited to rows above FW2_START.
      If Firmware2 is active, flash access is limited rows between FW1_START and FW2_START.
    """

    if data_to_write is not None:
        if len(data_to_write) != FLASH_ROW_SIZE_BYTES:
            raise ValueError(f"Data must match the FLASH_ROW_SIZE_BYTES ({FLASH_ROW_SIZE_BYTES}).")
        # Compose the command buffer
        cmd_buf = [
            0x46,            # 'F' Signature
            0x01,            # Write
            row_number & 0xFF,
            (row_number >> 8) & 0xFF
        ] + list(data_to_write)

        i2c_write_block_16b_offset(bus, ccg_slave_address, FLASH_ROW_READ_WRITE_OFFSET, cmd_buf)

    else:
        # Read
        cmd_buf = [
            0x46,            # "F' Signature
            0x00,            # 0 => Read
            row_number & 0xFF,
            (row_number >> 8) & 0xFF
        ]
        # write-phase:
        i2c_write_block_16b_offset(bus, ccg_slave_address, FLASH_ROW_READ_WRITE_OFFSET, cmd_buf)
        # read-phase: Read 64 bytes of data
        return i2c_read_block_16b_offset(bus, ccg_slave_address, FLASH_ROW_READ_WRITE_OFFSET, bytes_to_read)

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
def update_firmware_ccg6df_example(hex_file_path, ccg_slave_address):
    print("Opening I2C bus:", I2C_BUS, "Device address:", hex(ccg_slave_address))
    bus = smbus2.SMBus(I2C_BUS)

    try:

        reset_device_startup(bus, ccg_slave_address)
        print("Reset device from update_firmware_ccg6df_ex")

        # 1. Check DEVICE_MODE Register:
        pre_boot_device_mode = read_device_mode(bus, ccg_slave_address)
        print(f"Initial Device Mode: 0x{pre_boot_device_mode:02X}")
        # 2. Identify the firmware binary corresponding to the inactive firmware application (FW2 if FW1 is running and
        # vice versa).
        if pre_boot_device_mode == 0x85:
            print("Current device mode: 0x85 => FW1")
        elif pre_boot_device_mode == 0x86:
            print("Current device mode: 0x86 => FW2")
        else:
            print(f"ERROR: Device mode 0x{pre_boot_device_mode:02X} is not FW1 or FW2. Aborting.")
            return

        # (only for HPIv1)
        # 2. If CCG device is in FIRMWARE Mode:
        # 2a. Disable THE PD port using the Port-Disable Command
        print("Disabling PD ports...")
        disable_pd_ports(bus, ccg_slave_address)
        time.sleep(1)  # Increased delay

        # (only for HPIv1)
        # 2.b: Wait for "SUCCESS" response:
        response_code = check_response_code(bus, ccg_slave_address)
        print(f"Response code after disabling PD Ports = 0x{response_code:02X}")

        #(only for HPIv1)
        # 2.c Initiate JUMP_TO_BOOT command
        print("Jumping to bootloader mode...")
        jump_to_boot(bus, ccg_slave_address)

        # (only for HPIv1)
        # 2.d Wait for REST_COMPLETE event (or ~10ms Delay)
        time.sleep(0.5)  # Increased delay

        # (only for HPIv1)
        # 3. Read DEVICE_MODE register and verify what is the current mode:
        post_boot_device_mode = read_device_mode(bus, ccg_slave_address)
        print(f"Post Boot Device Mode: 0x{post_boot_device_mode:02X}")
        if post_boot_device_mode == 0x84:
            print("Current device mode: 0x84 => Bootloader Mode")
        elif post_boot_device_mode == 0x86:
            print("Current device mode: 0x86 => Firmware Mode: FW2")
        elif post_boot_device_mode == 0x85:
            print("Current device mode: 0x85 => Firmware Mode: FW1")
        else:
            print(f"ERROR: Device mode 0x{post_boot_device_mode:02X} is not in BTL, FW1, FW2 Mode Aborting.")
            return

        # 4. Initiate flashing mode entry using ENTER_FLASHING_MODE register.
        print("Entering flashing mode...")
        enter_flashing_mode(bus, ccg_slave_address)
        time.sleep(1)  # Increased delay

        response_code = check_response_code(bus, ccg_slave_address)
        print(f"Response code after Entering Flashing Mode = 0x{response_code:02X}")

        # 5. Clear the firmware metadata in flash memory
        # 5.a Fill data memory with zeroes
        # 5.b Use the FLASH_ROW_READ_WRITE register to trigger a write of the “zero” buffer into the metadata flash row.

        i2c_write_block_16b_offset(bus, ccg_slave_address, FLASH_ROW_READ_WRITE_OFFSET, [0x10])
        # 0x01: Flash Write
        # 0x10: Flash Read
        time.sleep(0.2)
        response_code = check_response_code(bus, ccg_slave_address)
        print(f"Response code for initiating Flash Read Command is = 0x{response_code:02X}")

        zero_row = [0x00] * FLASH_ROW_SIZE_BYTES
        flash_row_read_write(bus, 0xFF80, zero_row, ccg_slave_address) # FW1
        #row_data = flash_row_read_write(bus, 0xFF00, zero_row, ccg_slave_address) # FW2

        time.sleep(0.2)

        row_data = flash_row_read_write(bus, 0x0A00, None ,ccg_slave_address, 64)

        if row_data is None:
            print("Failed to read row data.")
        else:
            # Format the bytes for easier reading
            row_hex = [f"0x{b:02X}" for b in row_data]
            print("64-byte read from 0x0A00 memory address:\n", row_hex)


        # 5.b Use the FLASH_ROW_READ_WRITE register to trigger a write of the “zero” buffer into the metadata flash row.
        if pre_boot_device_mode == 0x86:
            # 0x86 => FW2
            print(f"Clearing FW2 metadata row at 0x{FW2_METADATA_ROW:04X} ...")
            zero_row = [0x00] * FLASH_ROW_SIZE_BYTES
            flash_row_read_write(bus, FW2_METADATA_ROW, zero_row, ccg_slave_address)

        # elif pre_boot_device_mode == 0x85:
            # 0x85 => FW1
            # print(f"Clearing FW1 metadata row at 0x{FW1_METADATA_ROW:04X} ...")
            # zero_row = [0x00] * FLASH_ROW_SIZE_BYTES
            # flash_row_read_write(bus, FW1_METADATA_ROW + 1, zero_row, ccg_slave_address)

        else:
            print(f"ERROR: pre_boot_device_mode = 0x{pre_boot_device_mode:02X} is not FW1 or FW2, cannot clear metadata.")
            return


        """ 
        # 6. Copy the data into the data memory.
        time.sleep(0.1)
        # Program each row from the IntelHex file
        print(f"Parsing HEX file: {hex_file_path}")
        ih = IntelHex(hex_file_path)

        start_addr = LOWER_FLASHING_LIMIT
        end_addr = UPPER_FLASHING_LIMIT

        print(f"start_addr: {start_addr:04x}")
        print(f"end_addr: {end_addr:04x}")

        # Display final range after bounding
        print(f"Programming from 0x{start_addr:04X} to 0x{end_addr:04X} in increments of 0x40.")

        increment = FLASH_ROW_SIZE_BYTES # 64 bytes per row

        for base_addr in range(start_addr, end_addr + 1, increment):
            row_data = []
            is_empty = True  # Flag to detect empty rows

            for offset in range(increment):
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
                print(f"Skipping empty row #{base_addr // increment + 1} at flash offset: 0x{base_addr:04X}")
                continue

            row_num = base_addr // increment + 1
            print(f"Writing row #{row_num}, flash offset: 0x{base_addr:04X}, data: {row_data[:8]}...")

            flash_row_read_write(bus, row_num, row_data, ccg_slave_address=ccg_slave_address)
            time.sleep(0.01)  # Minimal delay per row
            # Step 5a: Wait for "SUCCESS" response after each row write
            if not check_for_success_response(bus, ccg_slave_address, f"write row #{row_num}"):
                print("Aborting due to error writing flash row.")
                return
        """

        # Read
        cmd_buf = [
            0x46,  # Byte 0 : 'F' signature
            0x00,  # Byte 1 : 0 => Read
            0x40,  # byte 2 : Row Number MSB
            0x0A,  # byte 3 : Row Number LSB
                   # 0x09C0 is a row of zeroes after bootloader end
                   # FIRMWARE_BINARY_LOCATION = 0x0028

        ]

        # Write phase: send the "read" command
        i2c_write_block_16b_offset(bus, ccg_slave_address, FLASH_ROW_READ_WRITE_OFFSET, cmd_buf)

        if not check_response_code(bus, ccg_slave_address):
            print("Aborting due to error writing flash row.")

        # Read phase: read 64 bytes of data from the flash row
        read_row_val = i2c_read_block_16b_offset(
            bus,
            ccg_slave_address,
            0x400A,
            64,
        )



        # 'read_row_val' is now a list of 64 bytes from the specified flash row
        # Print the read values in hex form, e.g. 0x01, 0xFF, ...
        hex_values = [f"0x{byte:02X}" for byte in read_row_val]
        print(f"Data read from row #{0x2300} in hex:")
        print(hex_values)





        # 6. Validate firmware if needed
        #print("Validating firmware (placeholder)...")
        #validate_firmware(bus)
        #time.sleep(0.1)

        # 7. Reset to run new firmware
        #print("Resetting device to run new firmware...")
        #reset_device(bus, 1, ccg_slave_address=ccg_slave_address)
        #print("Firmware update sequence complete.")

    except Exception as e:
        print(f"Exception occurred: {e}")
        raise  # Re-throw the exception, so the outer loop sees it


    finally:
        bus.close()

# -------------------------------------------------------------------
# Example usage if run directly
# -------------------------------------------------------------------
if __name__ == "__main__":
    #firmware_hex_path = "/home/firas/Documents/CYPD6228/CYPD6228-96BZXI_notebook_dualapp_usb4_228_2.hex"
    firmware_hex_path = "/home/firas/Documents/CYPD6228/CYPD6228-96BZXI_notebook_dualapp_usb4_3_5_1_4_0_0_1_nb_FW1_EDITED.hex"



    # We'll try CCG6DF I2C_SLAVE_ADDRESS 0x42 first, then 0x40 if 0x42 fails.
    possible_addresses = [0x40, 0x42]

    # the 40 works fine with the following order:
    # 1. Turn OFF EE-3200 (Disconnect Power)
    # 2. Erase All Flash Using PsoC Programmer + Miniprog4 Adapter
    # 3. Patch Image (228-2) Using PsoC Programmer + Miniprog4 Adapter
    # 4. Disconnect Type-C Cable for MiniProg4
    # 5. Power ON EE-3200
    # 6. Run CCG6DF_COMPLETE_2

    for addr in possible_addresses:
        print(f"\nAttempting firmware update for CCG6DF at address 0x{addr:02X}...")
        try:

            print("Resetting device at startup...")
            bus = smbus2.SMBus(I2C_BUS)
            reset_device_startup(bus, addr)
            bus.close()

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
