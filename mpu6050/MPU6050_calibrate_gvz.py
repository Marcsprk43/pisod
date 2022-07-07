from MPU6050 import MPU6050


i2c_bus = 1
device_address = 0x68
# The offsets are different for each device and should be changed
# accordingly using a calibration procedure
x_accel_offset = 0
y_accel_offset = 0
z_accel_offset = 0
x_gyro_offset = 834
y_gyro_offset = 100
z_gyro_offset = 40
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

try:
    for i in range(1000):
        accel_reading = mpu.get_acceleration()
        x_accel_reading = accel_reading[0]
        y_accel_reading = accel_reading[1]
        z_accel_reading = accel_reading[2]

        gyro_reading = mpu.get_rotation()
        x_gyro_avg = alpha*gyro_reading[0] + one_minus_alpha*x_gyro_avg
        y_gyro_avg = alpha*gyro_reading[1] + one_minus_alpha*y_gyro_avg
        z_gyro_avg = alpha*gyro_reading[2] + one_minus_alpha*z_gyro_avg
        

        print('Gyro x: {}  y: {}  z:{}'.format(x_gyro_avg, y_gyro_avg, z_gyro_avg))


except KeyboardInterrupt:
    pass
