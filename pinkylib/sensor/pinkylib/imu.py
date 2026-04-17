import smbus2
import time
import struct

class IMU:
    # I2C Address
    BNO055_ADDRESS = 0x28

    # Registers
    BNO055_CHIP_ID_ADDR = 0x00
    BNO055_OPR_MODE_ADDR = 0x3D
    BNO055_PWR_MODE_ADDR = 0x3E
    BNO055_ACC_DATA_X_LSB_ADDR = 0x08

    # Operation Modes
    OPERATION_MODE_CONFIG = 0x00
    OPERATION_MODE_IMU = 0x08    # 6-DOF
    OPERATION_MODE_NDOF = 0x0C   # 9-DOF
    POWER_MODE_NORMAL = 0x00

    def __init__(self, i2c_bus=0, mode='imu'):
        try:
            self.bus = smbus2.SMBus(i2c_bus)
        except FileNotFoundError:
            raise RuntimeError(f"Cannot find I2C bus {i2c_bus}.")

        try:
            chip_id = self.bus.read_byte_data(self.BNO055_ADDRESS, self.BNO055_CHIP_ID_ADDR)
            if chip_id != 0xA0:
                raise RuntimeError("BNO055 sensor not found.")
        except IOError:
            print("I2C communication error: Check wiring.")

        # Select Mode
        if mode.lower() in ['ndof', '9dof', '9']:
            self.target_mode = self.OPERATION_MODE_NDOF
            print("Initializing sensor... [9-DOF NDOF Mode: Mag ON]")
        else:
            self.target_mode = self.OPERATION_MODE_IMU
            print("Initializing sensor... [6-DOF IMU Mode: Mag OFF]")

        # 1. Config Mode (Neutral)
        self.bus.write_byte_data(self.BNO055_ADDRESS, self.BNO055_OPR_MODE_ADDR, self.OPERATION_MODE_CONFIG)
        time.sleep(0.05)

        # 2. Power Mode
        self.bus.write_byte_data(self.BNO055_ADDRESS, self.BNO055_PWR_MODE_ADDR, self.POWER_MODE_NORMAL)
        time.sleep(0.05)
        
        # 3. Target Mode
        self.bus.write_byte_data(self.BNO055_ADDRESS, self.BNO055_OPR_MODE_ADDR, self.target_mode)
        time.sleep(0.1) 
        
        print("Initialization complete! Ready to read data.")

    def _bytes_to_int16(self, lsb, msb):
        return struct.unpack('<h', bytes([lsb, msb]))[0]

    def read_imu_data(self):
        try:
            # Read 24 bytes starting from ACC_X_LSB
            data = self.bus.read_i2c_block_data(self.BNO055_ADDRESS, self.BNO055_ACC_DATA_X_LSB_ADDR, 24)
        except OSError:
            return None

        # Acceleration (m/s^2)
        acc_x = self._bytes_to_int16(data[0], data[1]) / 100.0
        acc_y = self._bytes_to_int16(data[2], data[3]) / 100.0
        acc_z = self._bytes_to_int16(data[4], data[5]) / 100.0

        # Magnetometer (uT)
        mag_x = self._bytes_to_int16(data[6], data[7]) / 16.0
        mag_y = self._bytes_to_int16(data[8], data[9]) / 16.0
        mag_z = self._bytes_to_int16(data[10], data[11]) / 16.0

        # Gyroscope (deg/s)
        gyro_x = self._bytes_to_int16(data[12], data[13]) / 16.0
        gyro_y = self._bytes_to_int16(data[14], data[15]) / 16.0
        gyro_z = self._bytes_to_int16(data[16], data[17]) / 16.0

        # Euler Angles (Degrees)
        yaw = self._bytes_to_int16(data[18], data[19]) / 16.0
        roll = self._bytes_to_int16(data[20], data[21]) / 16.0
        pitch = self._bytes_to_int16(data[22], data[23]) / 16.0
        
        return {
            'acc': (acc_x, acc_y, acc_z),
            'mag': (mag_x, mag_y, mag_z),
            'gyro': (gyro_x, gyro_y, gyro_z),
            'euler': (roll, pitch, yaw) 
        }

    def close(self):
        self.bus.close()
        print("I2C bus closed.")