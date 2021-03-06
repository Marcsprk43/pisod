from MPU6050 import MPU6050


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

alpha = 0.05
one_minus_alpha = 1.0 - alpha

gyro_offset = [0]*3

print('Before offset changes:')
gyro_offset[0] = mpu.get_x_gyro_offset()
gyro_offset[1] = mpu.get_y_gyro_offset()
gyro_offset[2] = mpu.get_z_gyro_offset()
print(gyro_offset)


# supply your own gyro offsets here, scaled for min sensitivity
#mpu.set_x_gyro_offset_TC(int(756/4))
#mpu.set_y_gyro_offset_TC(int(108/4))
#mpu.set_z_gyro_offset_TC(int(28/4))
#Gyro offsets: [181, 31, 6]


mpu.set_x_gyro_offset(int(181))
mpu.set_y_gyro_offset(int(31))
mpu.set_z_gyro_offset(int(6))

mpu.set_x_gyro_offset(0)
mpu.set_y_gyro_offset(0)
mpu.set_z_gyro_offset(0)


print('After offset changes:')
gyro_offset[0] = mpu.get_x_gyro_offset()
gyro_offset[1] = mpu.get_y_gyro_offset()
gyro_offset[2] = mpu.get_z_gyro_offset()
print(gyro_offset)





try:
    for i in range(5000):
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

    mpu.set_x_gyro_offset(int(-x_gyro_avg/4))
    mpu.set_y_gyro_offset(int(-y_gyro_avg/4))
    mpu.set_z_gyro_offset(int(-z_gyro_avg/4))

    saved_offsets = [int(-x_gyro_avg/4),int(-y_gyro_avg/4),int(-z_gyro_avg/4)]

    gyro_offset[0] = mpu.get_x_gyro_offset()
    gyro_offset[1] = mpu.get_y_gyro_offset()
    gyro_offset[2] = mpu.get_z_gyro_offset()

    print('Gyro offsets: {}'.format(gyro_offset))


    for i in range(1000):
        accel_reading = mpu.get_acceleration()
        x_accel_reading = accel_reading[0]
        y_accel_reading = accel_reading[1]
        z_accel_reading = accel_reading[2]

        gyro_reading = mpu.get_rotation()
        x_gyro_avg = alpha*gyro_reading[0] + one_minus_alpha*x_gyro_avg
        y_gyro_avg = alpha*gyro_reading[1] + one_minus_alpha*y_gyro_avg
        z_gyro_avg = alpha*gyro_reading[2] + one_minus_alpha*z_gyro_avg
        

        print('**RawG x: {}  y: {}  z:{}'.format(gyro_reading[0], gyro_reading[1], gyro_reading[2]))
        print('**Gyro x: {}  y: {}  z:{}'.format(x_gyro_avg, y_gyro_avg, z_gyro_avg))
        print('**Accl x: {}  y: {}  z:{}'.format(x_accel_reading, y_accel_reading, z_accel_reading))


    print('Gyro offsets: {}'.format(saved_offsets))
    
except KeyboardInterrupt:
    pass
