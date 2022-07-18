from pymavlink import mavutil
import time

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('/dev/ttyAMA0', 57600)

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

count = 0 
while ( count < 100 ):
    try:
       print(the_connection.recv_match(blocking=False).to_dict())
    except:
        pass
    time.sleep(0.1)
    count += 1

timestamp = 1000.0

while ( 1 ):
    # Once connected, use 'the_connection' to get and send messages
    try: 
        the_connection.recv_match(blocking=False)
        lat = the_connection.messages['AHRS2'].lat  # Note, you can access message fields as attributes!
        lng = the_connection.messages['AHRS2'].lng  # Note, you can access message fields as attributes!
        altitude = the_connection.messages['AHRS2'].altitude  # Note, you can access message fields as attributes!
        voltage = the_connection.messages['SYS_STATUS'].voltage_battery  # Note, you can access message fields as attributes!
        #battery_remaining = the_connection.messages['SYS_STATUS'].battery_remaining
        
        if (the_connection.time_since('AHRS2') < timestamp):   # this means a new message has been received
            print('Alt:  {} - Time since last read: {}'.format(altitude, timestamp))

        timestamp = the_connection.time_since('AHRS2')

    except Exception as e:
        print(e)
