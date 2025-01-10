import time
import os
import smbus2
from intelhex import IntelHex

##############################################################################
#                           CONSTANTS & CONFIGURATION
##############################################################################
I2C_BUS = 2  # e.g., "/dev/i2c-2" in Linux
I2C_SLAVE_ADDR = 0x40  # The CCG6DF device's I2C address
FLASH_ROW_SIZE_BYTES = 128  # flash size for CCG6DF

# Offsets & Opcodes
PD_CONTROL_OFFSET_PORT0 = 0x1006  # (Port-0)
PD_CONTROL_OFFSET_PORT1 = 0x2006  # (Port-1)
PORT_DISABLE_OPCODE = 0x11        # Command for "Port Disable"
RESPONSE_OFFSET_PORT0 = 0x1400    # (Port-0)
RESPONSE_OFFSET_PORT1 = 0x2400    # (Port-1)
SUCCESS_CODE = 0x02               # Example "command success" code
RESET_OFFSET = 0x0008             # HPIv2 Address: 0x0008
DEVICE_MODE_OFFSET = 0x0000
ENTER_FLASHING_MODE_OFFSET = 0x000A
READ_SILICON_ID = 0x0002          # Read Returns A0, Address: 0x0002
READ_DIE_INFO = 0x0033            # Address 0x0033
JUMP_TO_BOOT_OFFSET = 0x0007      # Read 4.2.3.6.2 HPIv2
FLASH_ROW_READ_WRITE = 0x000C
FW1_METADATA_ROW = 0xFF80         # Address of FW1 Metadata Row
FW2_METADATA_ROW = 0xFF00         # Address of FW2 Metadata Row
TBOOTWAIT_OFFSET = 0x14           # Offset for tBootWait parameter
PDPORT_ENABLE = 0x002C            # Register to enable/disable PD ports

##############################################################################
#                           I2C HELPER FUNCTIONS
##############################################################################

# Initialize the I2C bus
bus = smbus2.SMBus(I2C_BUS)

def i2c_write_8bit(offset_16b, data_byte):
    """
    Write an 8-bit value to a 16-bit register offset in HPIv2 format:
      - LS-byte (offset_16b & 0xFF)
      - MS-byte ((offset_16b >> 8) & 0xFF)
      - data_byte
    """
    MS_Byte = (offset_16b >> 8) & 0xFF  # e.g., 0xAB
    LS_Byte = offset_16b & 0xFF        # e.g., 0xCD
    write_data = [LS_Byte, MS_Byte, data_byte]
    bus.i2c_rdwr(smbus2.i2c_msg.write(I2C_SLAVE_ADDR, write_data))

def i2c_read(offset_16b, num_bytes=1):
    """
    Read 'num_bytes' from a 16-bit register offset in HPIv2 format.
    """
    MSB_Byte = (offset_16b >> 8) & 0xFF  # 0xAB
    LSB_Byte = offset_16b & 0xFF        # 0xCD
    # Write phase: specify the offset we want to read from
    bus.i2c_rdwr(smbus2.i2c_msg.write(I2C_SLAVE_ADDR, [LSB_Byte, MSB_Byte]))
    # Read phase
    read_msg = smbus2.i2c_msg.read(I2C_SLAVE_ADDR, num_bytes)
    bus.i2c_rdwr(read_msg)
    return list(read_msg)

def read_response():
    """
    Reads a single byte from RESPONSE_OFFSET_PORT0 (0x1400).
    Returns the response code if available, else None.
    """
    resp = i2c_read(RESPONSE_OFFSET_PORT0, 1)
    if resp:
        return resp[0]
    return None

def check_response():
    """
    Reads the response code (1 byte) from the device.
    Prints success message if it matches SUCCESS_CODE (0x02).
    Otherwise, prints an error message.
    Returns True if success, False if error or no response.
    """
    resp_code = read_response()
    if resp_code is None:
        print("No response received (None).")
        return False
    else:
        print(f"Response code for Command: 0x{resp_code:02X}")
        if resp_code == SUCCESS_CODE:
            print("Command succeeded!")
            return True
        else:
            print("Command returned an error or unexpected code.")
            return False

##############################################################################
#                     STEPS 1..5: Enter Flash Mode Function
##############################################################################

def enter_flash_mode_steps_1_to_5():
    """
    Performs the following steps:
      1) Read DEVICE_MODE register
      2) Identify the inactive firmware application (example logic)
      3) Disable Port 0 and Port 1
      4) Wait/check for SUCCESS response
      5) Enter flashing mode via ENTER_FLASHING_MODE (send ASCII 'P' = 0x50).
         Then re-read DEVICE_MODE register to confirm we are in flashing mode.
    """
    print("Step 1) Read DEVICE_MODE register...")
    device_mode = i2c_read(DEVICE_MODE_OFFSET, 1)
    if not device_mode:
        print("Failed to read DEVICE_MODE.")
        return
    current_mode_value = device_mode[0]
    print(f"  Current DEVICE_MODE value: 0x{current_mode_value:02X}")

    # Step 2) Identify the active/inactive FW (example logic).
    # Adjust to your actual register definitions or metadata checks.
    # Placeholder assumption:
    if (current_mode_value & 0x01) == 1:
        active_fw = "FW1"
        inactive_fw = "FW2"
    else:
        active_fw = "FW2"
        inactive_fw = "FW1"

    print(f"  Active firmware: {active_fw}")
    print(f"  Inactive firmware to be updated: {inactive_fw}")

    # Step 3) Disable Port-0 and Port-1
    print("\nStep 3) Disabling PD Port-0...")
    i2c_write_8bit(PD_CONTROL_OFFSET_PORT0, PORT_DISABLE_OPCODE)
    time.sleep(0.1)
    check_response()

    print("Disabling PD Port-1...")
    i2c_write_8bit(PD_CONTROL_OFFSET_PORT1, PORT_DISABLE_OPCODE)
    time.sleep(0.1)
    check_response()

    # Step 4) Wait for success response
    # We already called check_response() above right after each write,
    # so consider that complete for demonstration purposes.

    # Step 5) Enter flashing mode by sending ASCII 'P' (0x50)
    print("\nStep 5) Sending ENTER_FLASHING_MODE command (ASCII 'P'=0x50)...")
    i2c_write_8bit(ENTER_FLASHING_MODE_OFFSET, 0x50)
    time.sleep(0.2)

    # Check the response (if needed)
    check_response()

    # Re-read DEVICE_MODE to confirm new mode
    new_device_mode = i2c_read(DEVICE_MODE_OFFSET, 1)
    if new_device_mode:
        print(f"  New DEVICE_MODE value: 0x{new_device_mode[0]:02X}")
        # Example check to see if device is in "Flashing Mode".
        # You need the real bit definitions for your chip.
        # e.g., if 0x08 indicates flashing mode:
        # if (new_device_mode[0] & 0x08):
        #     print("Device is now in Flashing Mode!")
        # else:
        #     print("Device did NOT enter flashing mode as expected.")
    else:
        print("Could not read DEVICE_MODE after entering flashing mode.")

##############################################################################
#                                MAIN
##############################################################################

if __name__ == "__main__":
    try:
        print("=== CCG6DF FW Update Steps 1..5 (Enter Flash Mode) ===\n")
        enter_flash_mode_steps_1_to_5()
    finally:
        # Close the I2C bus on exit
        bus.close()
        print("\nI2C bus closed. Exiting.")
