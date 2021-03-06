from MPU6050 import MPU6050

def int_to_bytes(i: int, *, signed: bool = False) -> bytes:

    length = ((i + ((i * signed) < 0)).bit_length() + 7 + signed) // 8
    return i.to_bytes(length, byteorder='big', signed=signed)

i2c_bus = 1
device_address = 0x68
# The offsets are different for each device and should be changed
# accordingly using a calibration procedure
x_accel_offset = 0
y_accel_offset = 0
z_accel_offset = 0

x_gyro_offset = 0
y_gyro_offset = 0
z_gyro_offset = 0

enable_debug_output = True

mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset,
              z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,
              enable_debug_output)

accel_reading = mpu.get_acceleration()

x_accel_reading = accel_reading[0]
y_accel_reading = accel_reading[1]
z_accel_reading = accel_reading[2]

x_accel_avg = [0]*100
y_accel_avg = [0]*100
z_accel_avg = [0]*100

axindex = 0
ayindex = 0
azindex = 0

gyro_reading = mpu.get_rotation()

print('Gyro rotation: {}'.format(gyro_reading))


x_gyro_reading = gyro_reading[0]
y_gyro_reading = gyro_reading[1]
z_gyro_reading = gyro_reading[2]

x_gyro_avg = 0.0
y_gyro_avg = 0.0
z_gyro_avg = 0.0

x_gyro_offset_avg = [0]*100
y_gyro_offset_avg = [0]*100
z_gyro_offset_avg = [0]*100

gxindex = 0
gyindex = 0
gzindex = 0

alpha = 0.1
one_minus_alpha = 1.0 - alpha

gyro_offset = [0]*3

gyro_offset[0] = mpu.get_x_gyro_offset_TC()
gyro_offset[1] = mpu.get_y_gyro_offset_TC()
gyro_offset[2] = mpu.get_z_gyro_offset_TC()

print('Gyro offsets: {}'.format(gyro_offset))

mpu.set_x_gyro_offset_TC(0)
mpu.set_y_gyro_offset_TC(117)
mpu.set_z_gyro_offset_TC(0)

gyro_offset[0] = mpu.get_x_gyro_offset_TC()
gyro_offset[1] = mpu.get_y_gyro_offset_TC()
gyro_offset[2] = mpu.get_z_gyro_offset_TC()

print('Gyro offsets: {}'.format(gyro_offset))


z_offset = 0
y_offset = 0

try:
    for i in range(15):
        for j in range(100):
            accel_reading = mpu.get_acceleration()
            x_accel_reading = accel_reading[0]
            y_accel_reading = accel_reading[1]
            z_accel_reading = accel_reading[2]

            gyro_reading = mpu.get_rotation()
            x_gyro_avg = alpha*gyro_reading[0] + one_minus_alpha*x_gyro_avg
            y_gyro_avg = alpha*gyro_reading[1] + one_minus_alpha*y_gyro_avg
            z_gyro_avg = alpha*gyro_reading[2] + one_minus_alpha*z_gyro_avg
            

        print('RawG x: {}  y: {}  z:{}'.format(gyro_reading[0], gyro_reading[1], gyro_reading[2]))
        print('Gyro x: {}  y: {}  z:{}'.format(x_gyro_avg, y_gyro_avg, z_gyro_avg))
        print('Accl x: {}  y: {}  z:{}'.format(x_accel_reading, y_accel_reading, z_accel_reading))

        z_offset += int(z_gyro_avg/20)
        y_offset += int(10)

        print('#### z_offset: {}'.format(z_offset))
        mpu.set_x_gyro_offset_TC(0)
        mpu.set_y_gyro_offset_TC(y_offset)
        mpu.set_z_gyro_offset_TC(z_offset)

        gyro_offset[0] = mpu.get_x_gyro_offset_TC()
        gyro_offset[1] = mpu.get_y_gyro_offset_TC()
        gyro_offset[2] = mpu.get_z_gyro_offset_TC()

        print('Gyro offsets: {}'.format(gyro_offset))


except KeyboardInterrupt:
    pass
