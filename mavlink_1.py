from matplotlib import blocking_input
from pymavlink import mavutil
import time

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('/dev/ttyAMA0', 57600)

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

count = 0 
while ( count < 30 ):
    try:
       the_connection.recv_match(blocking_input=False)
    except:
        pass
    time.sleep(0.1)
    count += 1



while ( 1 ):
    # Once connected, use 'the_connection' to get and send messages
    try: 
        the_connection.recv_match(blocking_input=False)
        altitude = the_connection.messages['AHRS2'].pitch  # Note, you can access message fields as attributes!
        timestamp = the_connection.time_since('AHRS2')
        print('Alt:  {} - Time since last read: {}'.format(altitude, timestamp))
    except Exception as e:
        print(e)

    time.sleep(0.2)