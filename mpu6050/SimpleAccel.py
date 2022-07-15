from MPU6050_no_fifo import MPU6050

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

gyro_offset = [0]*3

print('Gyro offsets before dmp: [181, 32, 6] 2022-07-13 :')
gyro_offset[0] = mpu.get_x_gyro_offset()
gyro_offset[1] = mpu.get_y_gyro_offset()
gyro_offset[2] = mpu.get_z_gyro_offset()
print(gyro_offset)

mpu.dmp_initialize()
mpu.set_DMP_enabled(True)
mpu_int_status = mpu.get_int_status()
print(hex(mpu_int_status))

packet_size = mpu.DMP_get_FIFO_packet_size()
print(packet_size)
FIFO_count = mpu.get_FIFO_count()
print(FIFO_count)

count = 0
FIFO_buffer = [0]*64

FIFO_count_list = list()

gyro_offset = [0]*3

mpu.set_x_gyro_offset(x_gyro_offset)
mpu.set_y_gyro_offset(y_gyro_offset)
mpu.set_z_gyro_offset(z_gyro_offset)

print('Gyro offsets: [181, 32, 6] 2022-07-13 :')
gyro_offset[0] = mpu.get_x_gyro_offset()
gyro_offset[1] = mpu.get_y_gyro_offset()
gyro_offset[2] = mpu.get_z_gyro_offset()
print(gyro_offset)

while count < 10000:
    
    #mpu_int_status = mpu.get_int_status()
    mpu.reset_FIFO()

    accel = mpu.get_acceleration()
    
    FIFO_count = mpu.get_FIFO_count()
    while (FIFO_count < packet_size):
        FIFO_count = mpu.get_FIFO_count()
    
    FIFO_buffer = mpu.get_FIFO_bytes(packet_size)

    dmp_accel = mpu.DMP_get_acceleration_int16(FIFO_buffer)
    FIFO_count = mpu.get_FIFO_count()
    print(FIFO_count, accel, dmp_accel.x, dmp_accel.y, dmp_accel.z)

    count += 1
