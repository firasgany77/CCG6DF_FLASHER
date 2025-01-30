import smbus2
from smbus2 import i2c_msg

I2C_BUS                = 2
CCG_SLAVE_ADDRESS      = 0x40
FLASH_ROW_READ_WRITE_OFFSET  = 0x000C
FLASH_ROW_SIZE_BYTES         = 64

HPI_RESPONSE_I2C_REG = 0x1D
"""0x1D: I2C_REG (HPIv2 only)
This message returns the I2C device register values from the I2C
device."""

FW1_START_ADDR = 0x0500 # FW START
TARGET_ADDR    = 0x0A80 # ROW NUMBER 43 in hex file


def i2c_write_block_16b_offset(bus, dev_addr, register_offset_16b, data_bytes):
    ls_byte = register_offset_16b & 0xFF
    ms_byte = (register_offset_16b >> 8) & 0xFF
    outbuf  = [ls_byte, ms_byte] + list(data_bytes)
    write_msg = i2c_msg.write(dev_addr, outbuf)
    bus.i2c_rdwr(write_msg)

def i2c_read_block_16b_offset(bus, dev_addr, register_offset_16b, num_bytes):
    ls_byte = register_offset_16b & 0xFF
    ms_byte = (register_offset_16b >> 8) & 0xFF
    write_msg = i2c_msg.write(dev_addr, [ls_byte, ms_byte])
    read_msg  = i2c_msg.read(dev_addr, num_bytes)
    bus.i2c_rdwr(write_msg, read_msg)
    return list(read_msg)

def read_64_bytes_for_address(bus, addr):
    """
    Reads the entire 64-byte row that contains 'addr' (within FW1).
    Returns the row data (list of 64 bytes).
    """
    # 1. Calculate offset in FW1:
    offset_in_fw1 = addr - FW1_START_ADDR
    if offset_in_fw1 < 0:
        raise ValueError(f"Address 0x{addr:04X} is below FW1 start (0x{FW1_START_ADDR:04X}).")

    # 2. Determine the row number
    row_number = offset_in_fw1 // FLASH_ROW_SIZE_BYTES
    # First 21 rows in HEX File are for Bootloader - so row_number must be 43 - 21 = 22 (for sanity_check)

    # 3. Prepare the read command
    cmd_buf = [
        0x46,  # 'F' signature
        0x00,  # 0 => read
        row_number & 0xFF,
        (row_number >> 8) & 0xFF
    ]

    # 4. Write phase: issue the read command
    i2c_write_block_16b_offset(bus, CCG_SLAVE_ADDRESS, FLASH_ROW_READ_WRITE_OFFSET, cmd_buf)

    # 5. Read phase: read 64 bytes from that row
    row_data = i2c_read_block_16b_offset(bus, CCG_SLAVE_ADDRESS,
                                         FLASH_ROW_READ_WRITE_OFFSET,
                                         FLASH_ROW_SIZE_BYTES)

    return row_data

if __name__ == "__main__":
    with smbus2.SMBus(I2C_BUS) as bus:
        # Read the entire 64-byte row containing TARGET_ADDR
        row_contents = read_64_bytes_for_address(bus, TARGET_ADDR)

        # Print row number + data in hex
        offset_in_fw1 = TARGET_ADDR - FW1_START_ADDR
        row_number    = offset_in_fw1 // FLASH_ROW_SIZE_BYTES
        print(f"Reading row #{row_number} for target address 0x{TARGET_ADDR:04X}...\n")

        # Format the bytes for easier reading
        row_hex = [f"0x{b:02X}" for b in row_contents]
        print("64-byte row data:\n", row_hex)
