#Row at address 0xD140 flashed successfully.
# Executing command: sudo i2ctransfer -y 2 w68@0x40 0x00 0x0C 0xD1 0x40 0x00 0x82 0x51 0x8A 0x49 0x38 0x59 0x8A 0x4D 0x37 0x49 0x38
# 0xD0 0x10 0x28 0x00 0x91 0x01 0x18 0x71 0x49 0x3C 0x91 0x02 0x18 0x71 0x5D 0x00 0x49 0x3D 0x48 0x3C 0x43 0x4E 0x46 0x26 0x21 0x1C
# 0xBD 0xFE 0xFD 0xF4 0xF7 0xFE 0x46 0x20 0x21 0x00 0xD1 0xE1 0x28 0x08 0xFE 0xF4 0xF7 0xFB 0x46 0x20 0xD0 0x04 0x28 0x07 0xFE 0xF9

import time
import os
from intelhex import IntelHex

def write_register(bus, address, register, data):
    """
    Write data to a specific register using i2ctransfer.
    """
    # Correctly order the register as high byte first, then low byte
    high_byte = (register >> 8) & 0xFF
    low_byte = register & 0xFF

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
    Flash a single row to the device with reversed data bytes.
    """
    # Correctly order the row address as high byte first, then low byte
    row_high_byte = (row_address >> 8) & 0xFF
    row_low_byte = row_address & 0xFF

    # Reverse the data bytes for flashing
    reversed_data = list(data[::-1])

    # Write to the Flash Row Write Register with the row address followed by reversed data
    write_register(bus, address, 0x000C, [row_high_byte, row_low_byte] + reversed_data)

def main():
    # Path to the firmware file
    firmware_file = "/home/firas/Documents/CYPD6228/CYPD6228-96BZXI_notebook_dualapp_usb4_3_5_1_4_0_0_1_nb.hex"

    if not os.path.exists(firmware_file):
        print(f"Error: Firmware file {firmware_file} not found.")
        return

    print("Parsing Intel HEX file...")
    hex_data = IntelHex(firmware_file)

    bus = 2  # Replace with the appropriate I2C bus number
    address = 0x40  # I2C address of the CYPD6228 device

    print("Entering flashing mode...")
    try:
        write_register(bus, address, 0x000A, [0x01])  # Enter flashing mode
    except Exception as e:
        print(f"Failed to enter flashing mode: {e}")
        return

    print("Flashing firmware rows...")
    try:
        for start_address in hex_data.segments():
            for row_address in range(start_address[0], start_address[1], 64):
                row_data = hex_data.tobinarray(start=row_address, size=64)
                flash_row(bus, address, row_address, row_data)
                time.sleep(0.05)  # 50 ms delay
                print(f"Row at address 0x{row_address:04X} flashed successfully.")
    except Exception as e:
        print(f"Failed during firmware flashing: {e}")
        return

    print("Validating firmware...")
    try:
        write_register(bus, address, 0x000B, [0x01])  # Validate firmware
    except Exception as e:
        print(f"Failed to validate firmware: {e}")
        return

    print("Activating new firmware...")
    try:
        write_register(bus, address, 0x002C, [0x00])  # Disable PD ports
        write_register(bus, address, 0x0008, [0x01])  # Reset the device
    except Exception as e:
        print(f"Failed to activate new firmware: {e}")
        return

    print("Firmware flashing and activation complete.")

if __name__ == "__main__":
    main()
