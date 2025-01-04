import os
import time
from intelhex import IntelHex


def write_register(bus, address, register, data):
    """
    Write data to a specific register using i2ctransfer.
    """
    high_byte = (register >> 8) & 0xFF
    low_byte = register & 0xFF

    if isinstance(data, int):
        data = [data]

    command = f"sudo i2ctransfer -y {bus} w{len(data) + 2}@0x{address:02X} 0x{high_byte:02X} 0x{low_byte:02X} " + " ".join(f"0x{byte:02X}" for byte in data)
    print(f"Executing command: {command}")
    result = os.system(command)
    if result != 0:
        raise Exception(f"Error writing to register 0x{register:04X} with data {data}. Command: {command}")


def read_register(bus, address, register, length=1):
    """
    Read data from a specific register using i2ctransfer.
    """
    high_byte = (register >> 8) & 0xFF
    low_byte = register & 0xFF

    command = f"sudo i2ctransfer -y {bus} w2@0x{address:02X} 0x{high_byte:02X} 0x{low_byte:02X} r{length}"
    print(f"Executing command: {command}")
    result = os.popen(command).read().strip()
    if not result:
        raise Exception(f"Error reading register 0x{register:04X}. Command: {command}")
    return [int(byte, 16) for byte in result.split()]


def wait_for_response(bus, address, response_expected):
    """
    Wait for a specific response from the device.
    """
    while True:
        response = read_register(bus, address, 0x000D, 1)[0]  # Status register
        if response == response_expected:
            print(f"Received response: 0x{response_expected:02X}")
            break
        time.sleep(0.05)


def clear_metadata(bus, address, metadata_address):
    """
    Clear the metadata of the firmware being updated.
    """
    print(f"Clearing metadata at address 0x{metadata_address:04X}...")
    zero_buffer = [0x00] * 64
    write_register(bus, address, 0x000C, [metadata_address >> 8, metadata_address & 0xFF] + zero_buffer)
    wait_for_response(bus, address, 0x01)  # SUCCESS
    print(f"Metadata at 0x{metadata_address:04X} cleared.")


def flash_row(bus, address, row_address, data):
    """
    Flash a single row to the device.
    """
    row_low_byte = row_address & 0xFF
    row_high_byte = (row_address >> 8) & 0xFF
    reversed_data = list(data[::-1])  # Reverse data
    write_register(bus, address, 0x000C, [row_high_byte, row_low_byte] + reversed_data)
    wait_for_response(bus, address, 0x01)  # SUCCESS
    time.sleep(0.05)  # Add 50ms delay


def verify_row(bus, address, row_address, expected_data):
    """
    Read and verify the data from the flash memory row.
    """
    print(f"Verifying row at address 0x{row_address:04X}...")
    write_register(bus, address, 0x000C, [row_address >> 8, row_address & 0xFF])  # Trigger read
    wait_for_response(bus, address, 0x06)  # FLASH_DATA_AVAILABLE
    read_data = read_register(bus, address, 0x0010, 64)  # Read back data
    if list(expected_data) != list(read_data):
        raise Exception(f"Row verification failed at 0x{row_address:04X}.")
    print(f"Row verified at 0x{row_address:04X}.")


#def erase_firmware(bus, address):
   # """
   # Erase the existing firmware from the device.
    #"""
 #   print("Erasing previous firmware...")
  #  write_register(bus, address, 0x000E, [0x01])  # Send erase firmware command
   # wait_for_response(bus, address, 0x01)  # SUCCESS


def main():
    firmware_file = "/home/firas/Downloads/CYPD6228-96BZXI_notebook_dualapp_usb4_3_5_1_4_0_0_1_nb.hex"

    if not os.path.exists(firmware_file):
        print(f"Error: Firmware file {firmware_file} not found.")
        return

    print("Parsing Intel HEX file...")
    hex_data = IntelHex(firmware_file)

    bus = 2
    address = 0x40

    print("Entering flashing mode...")
    try:
        write_register(bus, address, 0x000A, [0x01])  # Enter flashing mode
    except Exception as e:
        print(f"Failed to enter flashing mode: {e}")
        return

    #print("Erasing previous firmware...")
    #try:
     #   erase_firmware(bus, address)
    #except Exception as e:
     #   print(f"Failed to erase previous firmware: {e}")
      #  return

    print("Clearing metadata...")
    try:
        clear_metadata(bus, address, 0xFF00)  # Update for FW2 as an example
        #clear_metadata(bus, address, 0xFF80)  # Update for FW1 as an example
    except Exception as e:
        print(f"Failed to clear metadata: {e}")
        return

    print("Flashing firmware rows...")
    try:
        for start_address in hex_data.segments():
            for row_address in range(start_address[0], start_address[1], 64):
                row_data = hex_data.tobinarray(start=row_address, size=64)
                flash_row(bus, address, row_address, row_data)
                verify_row(bus, address, row_address, row_data)
                print(f"Row at address 0x{row_address:04X} flashed and verified successfully.")
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
        wait_for_response(bus, address, 0x01)  # SUCCESS
        write_register(bus, address, 0x0008, [0x01])  # Reset the device
        wait_for_response(bus, address, 0x05)  # RESET_COMPLETE
    except Exception as e:
        print(f"Failed to activate new firmware: {e}")
        return

    print("Firmware flashing and activation complete.")


if __name__ == "__main__":
    main()
