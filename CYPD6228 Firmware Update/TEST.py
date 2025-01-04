import time
import os
import smbus2
from intelhex import IntelHex

# I2C Bus Configuration
I2C_BUS = 2
I2C_SLAVE_ADDR = 0x40
FIRMWARE_BINARY_LOCATION = 0x0028

bus = smbus2.SMBus(I2C_BUS)

def i2c_read(offset_16b, num_bytes=1):
    """
    Reads num_bytes from a 16-bit offset in little-endian order.
    """
    high = (offset_16b >> 8) & 0xFF
    low = offset_16b & 0xFF

    # Write phase: specify the register to read
    bus.i2c_rdwr(smbus2.i2c_msg.write(I2C_SLAVE_ADDR, [high, low]))

    # Read phase
    read_msg = smbus2.i2c_msg.read(I2C_SLAVE_ADDR, num_bytes)
    bus.i2c_rdwr(read_msg)

    return list(read_msg)

def main():
    print("Read FW1_START and FW2_START")
    try:
        # Read 4 bytes from the FIRMWARE_BINARY_LOCATION register
        firmware_location = i2c_read(FIRMWARE_BINARY_LOCATION, 4)

        if firmware_location:
            fw1_start = (firmware_location[1] << 8) | firmware_location[0]  # FW1_START: Combine low and high bytes
            fw2_start = (firmware_location[3] << 8) | firmware_location[2]  # FW2_START: Combine low and high bytes

            print(f"FW1_START: 0x{fw1_start:04X}")
            print(f"FW2_START: 0x{fw2_start:04X}")
        else:
            print("Failed to read firmware location.")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
