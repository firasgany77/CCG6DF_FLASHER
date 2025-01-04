import time
import os
import smbus2
from intelhex import IntelHex


# I2C Bus / Address Configuration
I2C_BUS = 2  # e.g., /dev/i2c-2
I2C_SLAVE_ADDR = 0x40  # The CCG device's I2C address
FLASH_ROW_SIZE_BYTES = 128  # flash size for CCG6DF
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
FIRMWARE_BINARY_LOCATION = 0x0028

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

    bus.i2c_rdwr(smbus2.i2c_msg.write(I2C_SLAVE_ADDR, [high, low]))
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


def execute_i2c_command(command):
    """
    Executes a given i2ctransfer command and handles errors.
    """
    print(f"Executing command: {command}")
    result = os.system(command)
    if result != 0:
        raise Exception(f"Command failed: {command}")

def write_multi_byte(bus, address, register, data):
    """
    Writes data to a specific register using i2ctransfer.
    """
    #Reg_Address: 0xABCD
    high_byte = (register >> 8) & 0xFF # high: 0xAB
    low_byte = register & 0xFF # low :0xCD
    data = [data] if isinstance(data, int) else data
    command = f"sudo i2ctransfer -y {bus} w{len(data) + 2}@0x{address:02X} 0x{high_byte:02X} 0x{low_byte:02X} "
    command += " ".join(f"0x{byte:02X}" for byte in data)
    execute_i2c_command(command)

def erase_firmware(bus, address):
    """Erases the existing firmware from the device."""
    print("Erasing previous firmware...")
    write_multi_byte(bus, address, 0x000E, [0x01])

def flash_row(bus, address, row_address, data):
    """Flashes a single row to the device."""
    row_high_byte = (row_address >> 8) & 0xFF # row high byte
    row_low_byte = row_address & 0xFF # row low byte
    write_multi_byte(bus, address, FLASH_ROW_READ_WRITE, [row_high_byte, row_low_byte] + list(data))

def disable_pd_ports(bus, address):
    """Disables PD ports on the device."""
    print("Disabling PD ports...")
    write_multi_byte(bus, address, 0x002C, [0x00])

def reset_device(bus, address):
    """Resets the device."""
    print("Resetting the device...")
    write_multi_byte(bus, address, 0x0008, [0x01])

def validate_firmware(bus, address):
    """Validates the newly flashed firmware."""
    print("Validating firmware...")
    write_multi_byte(bus, address, 0x000B, [0x01])

def activate_new_firmware(bus, address):
    """Activates the new firmware by resetting the device."""
    print("Activating new firmware...")
    disable_pd_ports(bus, address)
    reset_device(bus, address)

def flash_firmware(bus, address, firmware_file):
    """
    Parses the firmware file and flashes it to the device.
    """
    if not os.path.exists(firmware_file):
        raise FileNotFoundError(f"Firmware file {firmware_file} not found.")

    print("Parsing Intel HEX file...")
    hex_data = IntelHex(firmware_file)

    print("Entering flashing mode...")
    write_multi_byte(bus, address, 0x000A, [0x01])

    erase_firmware(bus, address)

    print("Flashing firmware rows...")
    for start_address, end_address in hex_data.segments():
        for row_address in range(start_address, end_address, 64):
            row_data = hex_data.tobinarray(start=row_address, size=64)
            flash_row(bus, address, row_address, row_data)
            time.sleep(0.05)  # Small delay for stability
            print(f"Flashed row at address 0x{row_address:04X}")

    validate_firmware(bus, address)
    activate_new_firmware(bus, address)

    print("Firmware flashing and activation complete.")

def main():
    firmware_file = "/home/firas/Documents/CYPD6228/CYPD6228-96BZXI_notebook_dualapp_usb4_3_5_1_4_0_0_1_nb.hex"

    # Fixed structure / array for the data to send
    data = [
        0xA8, 0x42, 0x30, 0x46, 0x03, 0xD8, 0x00, 0xF0, 0xA5, 0xF8, 0x01, 0x20,
        0x02, 0xE0, 0x00, 0xF0, 0x95, 0xF8, 0x00, 0x20, 0x04, 0x49, 0x88, 0x55,
        0x38, 0x46, 0xFE, 0xF7, 0x8D, 0xFC, 0xF8, 0xBD, 0x80, 0x20, 0x00, 0x10,
        0x88, 0x13, 0x00, 0x00, 0xF8, 0x20, 0x00, 0x20, 0x00, 0x20, 0x70, 0x47,
        0xF8, 0xB5, 0x0D, 0x46, 0x07, 0x46, 0xFE, 0xF7, 0x81, 0xFF, 0x06, 0x46,
        0x0A, 0x48, 0x01, 0x68
    ] # size = 64 Bytes, data for row address: 0x2140

    # Execute the command for debug purposes
    print("Executing debug command with fixed data...")
    try:
        #write_multi_byte(I2C_BUS, I2C_SLAVE_ADDR, FLASH_ROW_READ_WRITE, data)
        flash_row(I2C_BUS, I2C_SLAVE_ADDR, 0x2140, data)
        print("Command executed successfully.")
    except Exception as e:
        print(f"Error executing debug command: {e}")

    resp_code = read_response()
    if resp_code is None:
        print("No response received (None).")
    else:
        print(f"Response code for Write Command: 0x{resp_code:02X}")
        if resp_code == SUCCESS_CODE:
            print("Command succeeded!")
        else:
            print("Command returned an error or unexpected code.")


    print("Read FW1_START and FW2_START")
    try:
        # Read 4 bytes from the FIRMWARE_BINARY_LOCATION register
        firmware_location = i2c_read(FIRMWARE_BINARY_LOCATION, 4)

        if firmware_location:
            fw1_start = (firmware_location[0] << 8) | (firmware_location[1])  # FW1_START: Combine low and high bytes
            fw2_start = (firmware_location[2] << 8) | (firmware_location[3])   # FW2_START: Combine low and high bytes

            print(f"FW1_START: 0x{fw1_start:04X}")
            print(f"FW2_START: 0x{fw2_start:04X}")
        else:
            print("Failed to read firmware location.")

    except Exception as e:
        print(f"Error: {e}")

   # try:
   #     flash_firmware(bus, address, firmware_file)
   # except Exception as e:
   #     print(f"Error during firmware flashing: {e}")

if __name__ == "__main__":
    main()
