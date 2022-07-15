from MPU6050 import MPU6050
from math import sqrt, asin, atan, degrees

i2c_bus = 1
device_address = 0x68
# The offsets are different for each device and should be changed
# accordingly using a calibration procedure
x_accel_offset = -0
y_accel_offset = -0
z_accel_offset = 0

# Gyro offsets: [181, 32, 6] 2022-07-13
x_gyro_offset = 181
y_gyro_offset = 32
z_gyro_offset = 6

enable_debug_output = True

mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset,
              z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,
              sample_rate_divider=1, dlpf=0x04,
              a_debug=enable_debug_output)

count = 0

while count < 10000:
    
    accel = mpu.get_acceleration()
    
    pitch = asin(accel[0]/sqrt(accel[0]*accel[0] + accel[1]*accel[1]+ accel[2]*accel[2] ))
    roll = atan(accel[1]/accel[2])
    print('{:03d} - {}  :: {:7.2f}  {:7.2f}'.format( FIFO_count, accel,  
                                                    round(degrees(roll),2), round(degrees(pitch),2) ))

    count += 1
