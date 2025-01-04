#!/usr/bin/env python3
"""
Minimal test code to send the "Port Disable" command (0x11) to a CCG device
and read back the response code.

Requirements:
  - pip install smbus2
  - Adjust offsets (PD_CONTROL_OFFSET_PORT0, RESPONSE_OFFSET) and I2C address as needed.
"""

import time
import smbus2

# I2C Bus / Address Configuration
I2C_BUS = 2  # e.g., /dev/i2c-2
I2C_SLAVE_ADDR = 0x40  # The CCG device's I2C address

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


def main():

    #print("Sending 'RESET' command (opcode=0x00)...") # NO RESPONSE for RESET command (0x00 is returned as response code).
    #i2c_write_8bit(RESET_OFFSET, 0x00)

    print("Sending 'Port-0 Disable' command (opcode=0x11)...") # responses with 0x02 (SUCCESS). // Working One
    i2c_write_8bit(PD_CONTROL_OFFSET_PORT0, PORT_DISABLE_OPCODE)
    #print("Sending 'Port-1 Disable' command (opcode=0x11)...") # responses with 0x02 (SUCCESS).
    i2c_write_8bit(PD_CONTROL_OFFSET_PORT1, PORT_DISABLE_OPCODE)

    time.sleep(1) # minimum delay is needed to catch the response after write command
    # Read the response code of Reset Event
    resp_code = read_response()
    if resp_code is None:
        print("No response received (None).")
    else:
        print(f"Response code: 0x{resp_code:02X}")
        if resp_code == SUCCESS_CODE:
            print("Command succeeded!")
        else:
            print("Command returned an error or unexpected code.")


    time.sleep(0.5)
    print("Sending 'Port Enable' command (opcode=0x03)...")
    i2c_write_8bit(PDPORT_ENABLE, 0x00)
    i2c_write_8bit(PDPORT_ENABLE, 0x01)



    #time.sleep(3)

    #print("Entering Flashing Mode Command (opcode=0x00)...")
    #i2c_write_8bit(ENTER_FLASHING_MODE_OFFSET, 0x01)

    print("Initiating JUMP TO BOOT Command")
    i2c_write_8bit(JUMP_TO_BOOT_OFFSET, 0x01)

    time.sleep(1)

    print("Reading Device Mode Register")
    device_mode_reg_val = i2c_read(DEVICE_MODE_OFFSET, 1)
    if device_mode_reg_val:
         #device_mode_reg_val[0] contains the first byte of the response
        print(f"device_mode_reg_val: 0x{device_mode_reg_val[0]:02X}")
    else:
        print("Failed to read device_mode_reg_val.")

    print("Reading Silicon ID")
    silicon_id_val = i2c_read(READ_SILICON_ID, 1)
    if silicon_id_val:
         #silicon_id_val[0] contains the first byte of the response
        print(f"silicon_id_val: 0x{silicon_id_val[0]:02X}")
    else:
        print("Failed to read silicon_id_val.")

#    print("Reading Die Info") # Returns Byte 10: 0x7F, byte 11: 0x09, why not A0? (check HPIv2 (CCGx) Address: 0x0033)
#    die_info_val = i2c_read(READ_DIE_INFO, 32)  # Assume this returns a list of 32 bytes
#    if die_info_val:
#        print("die_info_val:")
#        for i, byte in enumerate(die_info_val):
#            print(f"Byte {i}: 0x{byte:02X}")
#    else:
#        print("Failed to read die_info_val")


    # Small delay to let the device process
    time.sleep(0.8)




if __name__ == "__main__":
    try:
        main()
    finally:
        bus.close()
