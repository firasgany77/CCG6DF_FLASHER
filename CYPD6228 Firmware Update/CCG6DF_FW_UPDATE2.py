#!/usr/bin/env python3
"""
Minimal test code to send the "Port Disable" command (0x11) to a CCG device
and read back the response code.

Requirements:
  - pip install smbus2
  - Adjust offsets (PD_CONTROL_OFFSET_PORT0, RESPONSE_OFFSET) and I2C address as needed.
"""

"""
Steps for Firmware Update:
. Check the DEVICE_MODE register.[Done]
2. If CCG device is in FIRMWARE mode:
    a. Disable the PD port using the Port Disable command [Done]
    b. Wait for SUCCESS response [Done]
    c. Initiate a JUMP_TO_BOOT command [Done]
    d. Wait for a RESET_COMPLETE event (or ~10 ms delay). [Done] - How to read the event?
3. Read DEVICE_MODE register and verify that device is in bootloader mode. [Fail]
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

import time
import os
import smbus2

# I2C Bus / Address Configuration
I2C_BUS = 2  # e.g., /dev/i2c-2
I2C_SLAVE_ADDR = 0x40  # The CCG device's I2C address
FLASH_ROW_SIZE_BYTES = 128  # flash size for CCG6DF

# Offsets & Opcodes
PD_CONTROL_OFFSET_PORT0 = 0x1006  # (Port-0)
PD_CONTROL_OFFSET_PORT1 = 0x2006  # (Port-1)
PORT_DISABLE_OPCODE = 0x11  # Command for "Port Disable"
RESPONSE_OFFSET_PORT0 = 0x1400  # (Port-0)
RESPONSE_OFFSET_PORT1 = 0x2400 # (Port-1)
SUCCESS_CODE = 0x02  # Example "command success" code
RESET_OFFSET = 0x0800 # HPIv2 Address: 0x0008, Byte[1]: 00: i2c rest, 01: device reset.
DEVICE_MODE_OFFSET = 0x0000
ENTER_FLASHING_MODE_OFFSET = 0x000A
READ_SILICON_ID = 0x0002 # Read Returns A0, Address: 0x0002 , Silicon Revision Major.Minor field 1.1 means A0 silicon. Likewise, 1.2 is A1 silicon.
READ_DIE_INFO = 0x0033 # Address 0x0033
JUMP_TO_BOOT_OFFSET = 0x0007 # read 4.2.3.6.2 HPIv2

FLASH_ROW_READ_WRITE = 0x000C
FW1_METADATA_ROW = 0xFF80  # Address of FW1 Metadata Row
TBOOTWAIT_OFFSET = 0x14  # Offset for tBootWait parameter in the metadata


PDPORT_ENABLE = 0x002C
# Commands like DEVICE RESET or JUMP_TO_BOOT should only be initiated after the SUCCESS response for the
# PDPORT_ENABLE command has been received.

# Response Codes:
# 0x00: No response. No outstanding command, event, or asynchronous message in the system.
# 0x01: Reserved
# 0x02: SUCCESS
# 0x03: Flash Data Available, Flash row data requested by EC is available in Data Memory.
# 0x05: Invalid Command. Partial, unaligned register write or invalid command.
# 0x06: Invalid State: Command failed because it is not supported in the current state of the firmware.
# 0x07: Flash Update Failed. Flash write operation failed.
# 0x08: Invalid FW. FW validity check failed. See VALIDATE_FW command.
# 0x09: Invalid Arguments. Command handling failed due to invalid arguments.
# 0x0A: Not Supported. Command not supported in the current mode.


# In addition to the RESET command, CCG also supports a JUMP mechanism which can be used to request the
# device to revert to Bootloader mode. In the case of a JUMP_TO_BOOT command, CCG stays in bootloader
# mode so that the EC can perform a firmware update operation.

# Since two different application firmware binaries (FW1 and FW2) are present under HPIv2, a special command
# is provided to request transfer to the alternate firmware binary. The JUMP_TO_ALT_FW operation is requested
# by writing a special signature into the JUMP_TO_BOOT register. This command can be useful in cases where
# the functionality provided by each of the firmware binaries is different, and the user wants to select a specific
# feature set at runtime.

# Initialize the bus
bus = smbus2.SMBus(I2C_BUS)


def i2c_write_8bit(offset_16b, data_byte):
    """
    Write one byte (data_byte) to a 16-bit offset in little-endian order.
    """
    # 0xABCD
    low = (offset_16b >> 8) & 0xFF # 0xAB
    high = offset_16b & 0xFF # 0xCD

    write_data = [high, low, data_byte]
    bus.i2c_rdwr(
        smbus2.i2c_msg.write(I2C_SLAVE_ADDR, write_data)
    )


def i2c_read(offset_16b, num_bytes=1):
    """
    Read num_bytes from a 16-bit offset in little-endian order.
    """
    # 0xABCD
    low = (offset_16b >> 8) & 0xFF # 0xAB
    high = offset_16b & 0xFF # 0xCD

    bus.i2c_rdwr(
        smbus2.i2c_msg.write(I2C_SLAVE_ADDR, [high, low])
    )
    # Read phase
    read_msg = smbus2.i2c_msg.read(I2C_SLAVE_ADDR, num_bytes)
    bus.i2c_rdwr(read_msg)

    return list(read_msg)


def read_response():
    """
    Reads a single byte response code from RESPONSE_OFFSET.
    Returns None if nothing read.
    """
    resp = i2c_read(RESPONSE_OFFSET_PORT0, 1)
    if resp:
        return resp[0]
    return None



def write_register(bus, address, register, data):
    """
    Write data to a specific register using i2ctransfer.
    """
    # Correctly order the register as high byte first, then low byte
    low_byte = (register >> 8) & 0xFF
    high_byte = register & 0xFF

    # Ensure data is a list
    if isinstance(data, int):
        data = [data]

    # Construct the command with proper formatting
    command = f"sudo i2ctransfer -y {bus} w{len(data) + 2}@0x{address:02X} 0x{high_byte:02X} 0x{low_byte:02X} " + " ".join(f"0x{byte:02X}" for byte in data)
    print(f"Executing command: {command}")

    # Execute the command
    result = os.system(command)

    # Check for errors
    if result != 0:
        raise Exception(f"Error writing to register 0x{register:04X} with data {data}. Command: {command}")

def flash_row(bus, address, row_address, data):
    """
    Flash a single row to the device.
    """
    # this version of the script uses high byte first, then low byte.
    row_low_byte = row_address & 0xFF
    row_high_byte = (row_address >> 8) & 0xFF

    # Write to the Flash Row Write Register with the row address followed by data
    # for address 0xFE40 for example, high-byte: 0xFE , low-byte: 0x40
    write_register(bus, address, 0x000C, [row_high_byte, row_low_byte] + list(data))


def read_row(bus, address, row_address, row_size):
    """
    Read a single row from the device.
    """
    print(f"Reading row at address 0x{row_address:04X}...")

    # Split the row address into high and low bytes
    row_low_byte = row_address & 0xFF
    row_high_byte = (row_address >> 8) & 0xFF

    # Initiate a read operation by writing the row address to the FLASH_ROW_READ_WRITE register
    write_register(bus, address, 0x000C, [row_high_byte, row_low_byte])

    # Allow time for the flash data to become available
    time.sleep(0.05)  # 50 ms delay to ensure data is ready

    # Read the row data (128 bytes for CCG6DF)
    command = f"sudo i2ctransfer -y {bus} w2@0x{address:02X} 0x00 0x0C r{row_size}"
    print(f"Executing command: {command}")

    result = os.popen(command).read().strip()
    if not result:
        raise Exception("Failed to read row data.")

    # Convert the result into a list of bytes
    data = [int(byte, 16) for byte in result.split()]
    if len(data) != row_size:
        raise Exception(f"Unexpected row size. Expected: {row_size}, Received: {len(data)}")

    print(f"Read data: {data}")
    return data


def main():
    print("Read tBootTime value from Metadata")
    read_row(I2C_BUS, 0xFF80, 0x1004, 128)
    read_row(I2C_BUS, 0xFF80, 0x1004, 128)


    #print("Sending 'Port-0 Disable' command (opcode=0x11)...")
    #i2c_write_8bit(PD_CONTROL_OFFSET_PORT0, PORT_DISABLE_OPCODE)
    #print("Sending 'Port-1 Disable' command (opcode=0x11)...")
    #i2c_write_8bit(PD_CONTROL_OFFSET_PORT1, PORT_DISABLE_OPCODE)

    time.sleep(1)  # Minimum delay needed to catch the response
    resp_code = read_response()
    if resp_code is None:
        print("No response received (None).")
    else:
        print(f"Response code for PORT DISABLE CMD: 0x{resp_code:02X}")
        if resp_code == SUCCESS_CODE:
            print("Command succeeded!")
        else:
            print("Command returned an error or unexpected code.")

    resp_code = read_response()
    if resp_code is None:
        print("No response received (None).")
    else:
        print(f"Response code for JUMP TO BOOT CMD: 0x{resp_code:02X}")
        if resp_code == SUCCESS_CODE:
            print("Command succeeded!")
        else:
            print("Command returned an error or unexpected code.")

    time.sleep(3)

    print("Reading Device Mode Register")
    device_mode_reg_val = i2c_read(DEVICE_MODE_OFFSET, 1)
    if device_mode_reg_val:
        print(f"device_mode_reg_val: 0x{device_mode_reg_val[0]:02X}")
    else:
        print("Failed to read device_mode_reg_val.")

    print("Reading Silicon ID")
    silicon_id_val = i2c_read(READ_SILICON_ID, 1)
    if silicon_id_val:
        print(f"silicon_id_val: 0x{silicon_id_val[0]:02X}")
    else:
        print("Failed to read silicon_id_val.")

    time.sleep(0.8)

if __name__ == "__main__":
    try:
        main()
    finally:
        bus.close()
