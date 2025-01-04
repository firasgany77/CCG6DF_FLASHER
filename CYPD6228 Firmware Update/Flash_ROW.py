import time
import os
from intelhex import IntelHex

def execute_i2c_command(command):
    """
    Executes a given i2ctransfer command and handles errors.
    """
    print(f"Executing command: {command}")
    result = os.system(command)
    if result != 0:
        raise Exception(f"Command failed: {command}")

def write_register(bus, address, register, data):
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
    write_register(bus, address, 0x000E, [0x01])

def flash_row(bus, address, row_address, data):
    """Flashes a single row to the device."""
    row_high_byte = (row_address >> 8) & 0xFF
    row_low_byte = row_address & 0xFF
    write_register(bus, address, 0x000C, [row_low_byte, row_high_byte] + list(data))

def disable_pd_ports(bus, address):
    """Disables PD ports on the device."""
    print("Disabling PD ports...")
    write_register(bus, address, 0x002C, [0x00])

def reset_device(bus, address):
    """Resets the device."""
    print("Resetting the device...")
    write_register(bus, address, 0x0008, [0x01])

def validate_firmware(bus, address):
    """Validates the newly flashed firmware."""
    print("Validating firmware...")
    write_register(bus, address, 0x000B, [0x01])

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
    write_register(bus, address, 0x000A, [0x01])

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
    bus = 2  # I2C bus number
    address = 0x40  # I2C address of the device

    try:
        flash_firmware(bus, address, firmware_file)
    except Exception as e:
        print(f"Error during firmware flashing: {e}")

if __name__ == "__main__":
    main()
