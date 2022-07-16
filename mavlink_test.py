from pymavlink import mavutil

# Start a connection listening on a UDP port
mv_con = mavutil.mavlink_connection('/dev/ttyAMA0',57600)

print('Connecting to /dev/ttyAMA0')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
mv_con.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (mv_con.target_system, mv_con.target_component))

# Once connected, use 'the_connection' to get and send messages

while(1):
    try: 
        altitude = mv_con.messages['GPS_RAW_INT'].alt  # Note, you can access message fields as attributes!
        timestamp = mv_con.time_since('GPS_RAW_INT')
        firmware_version = mv_con.messages['FIRMWARE_VERSION_TYPE']

        print('Altitude: {} - {} firmware: {}'.format(altitude,timestamp,firmware_version))
    except:
        print('No GPS_RAW_INT message received')