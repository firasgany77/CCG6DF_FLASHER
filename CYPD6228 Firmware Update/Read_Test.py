import smbus2
from smbus2 import i2c_msg

from OLD.CCG6DF_Flasher_O1 import PD_CONTROL_OFFSET

# I2C Bus / Address / Constants
I2C_BUS                 = 2
CCG_SLAVE_ADDRESS       = 0x42
FLASH_ROW_READ_WRITE_OFFSET = 0x000C
FLASH_ROW_SIZE_BYTES         = 64

# Define needed constants (previously missing):
RESPONSE_OFFSET_PORT0   = 0x1400
PD_CONTROL_OFFSET_PORT0 = 0x1006 # HPIv2 Port-0
READ_WRITE_I2C_DEVICE_REGISTER_OPCODE = 0x3C # Opcode for HPIv2 port-0 address: 0x1006
SUCCESS_CODE            = 0x02  # Example: "command success" code for HPI
FIRMWARE_BINARY_LOCATION = 0x0028

FW1_START_ADDR = 0x0500  # FW START
TARGET_ADDR    = 0x0A80  # e.g., in the FW1 area

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

def check_response(bus, ccg_slave_address):
    """
    Reads the response register at RESPONSE_OFFSET_PORT0 and returns the numeric code.
    Always prints the code, but does NOT decide success/failure anymore.
    """
    response_bytes = i2c_read_block_16b_offset(bus, ccg_slave_address, PD_CONTROL_OFFSET, 1)
    if not response_bytes:
        print("WARNING: No data read from response register. Returning 0xFF.")
        return 0xFF

    resp_val = response_bytes[0]
    return resp_val



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
    # Example comment: first 21 rows in the HEX file might be for the bootloader,
    # so row_number could be offset by 21. (Just a comment or 'sanity check')

    # 3. Prepare the read command
    cmd_buf = [
        0x46,  # 'F' signature
        0x00,  # 0 => read
        (row_number & 0xFF),
        ((row_number >> 8) & 0xFF)
    ]



    response_code = check_response(bus, CCG_SLAVE_ADDRESS)
    print(f"Response code for reading 0x{FIRMWARE_BINARY_LOCATION:04X} is = 0x{response_code:02X}")

    return row_data


if __name__ == "__main__":
    with smbus2.SMBus(I2C_BUS) as bus:

        # HPIv2 register address space: Page 13 in CCG Host Processor Interface.
        # 1. Read register with 2-byte address in range (0x0000 - 0x0040): FW information registers:
        # Read + Write phases inside i2c_read_block_16b_offset.
        row_data = i2c_read_block_16b_offset(bus, CCG_SLAVE_ADDRESS, FIRMWARE_BINARY_LOCATION, 4)
        # Byte[0] to Byte[1]: FW1_START = 0x0A00
        # Byte[0] to Byte[1]: FW2_START = 0x7000
        if row_data is None:
            print("Failed to read row data.")
        else:
            # Format the bytes for easier reading
            row_hex = [f"0x{b:02X}" for b in row_data]
            print("4-byte read from FIRMWARE_BINARY_LOCATION:\n", row_hex)
        response_code = check_response(bus, CCG_SLAVE_ADDRESS)
        print(f"Response code for reading 0x{FIRMWARE_BINARY_LOCATION:04X} is = 0x{response_code:02X}")
        ####################################################################################################

        i2c_write_block_16b_offset(bus, CCG_SLAVE_ADDRESS, PD_CONTROL_OFFSET_PORT0, [READ_WRITE_I2C_DEVICE_REGISTER_OPCODE])
        response_code = check_response(bus, CCG_SLAVE_ADDRESS)
        print(f"Response code for writing 0x3C to PD_CONTROL to read/update device register = 0x{response_code:02X}")

        # 1. Read register with 2-byte address in range (0x1400 - 0x1600): Read Data Memory (508 bytes).
        # Read + Write phases inside i2c_read_block_16b_offset.
        row_data2 = i2c_read_block_16b_offset(bus, CCG_SLAVE_ADDRESS, 0x2000, 2)
        if row_data2 is None:
            print("Failed to read row data.")
        else:
            # Format the bytes for easier reading
            row_hex = [f"0x{b:02X}" for b in row_data2]
            print("4-byte read from FIRMWARE_BINARY_LOCATION:\n", row_hex)
        response_code = check_response(bus, CCG_SLAVE_ADDRESS)
        print(f"Response code for reading 0x{FIRMWARE_BINARY_LOCATION:04X} is = 0x{response_code:02X}")