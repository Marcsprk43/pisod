from pymavlink import mavutil
import time

def wait_conn(conn):
    """
    Sends a ping to stabilish the UDP communication and awaits for a response
    """
    msg = None
    while not msg:
        conn.mav.ping_send(
            int(time.time() * 1e6), # Unix time in microseconds
            0, # Ping number
            0, # Request ping of all systems
            0 # Request ping of all components
        )
        msg = conn.recv_match()
        time.sleep(0.5)

# Start a connection listening on a UDP port
mv_con = mavutil.mavlink_connection('/dev/ttyAMA0',57600)

print('Connecting to /dev/ttyAMA0')



# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
mv_con.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (mv_con.target_system, mv_con.target_component))

# Once connected, use 'the_connection' to get and send messages

# Send a ping to start connection and wait for any reply.
#  This function is necessary when using 'udpout',
#  as described before, 'udpout' connects to 'udpin',
#  and needs to send something to allow 'udpin' to start
#  sending data.
wait_conn(mv_con)

# Get some information !
while True:
    try:
        print(mv_con.recv_match().to_dict())
    except:
        pass
    time.sleep(0.1)

    try: 
        altitude = mv_con.messages['GPS_RAW_INT'].alt  # Note, you can access message fields as attributes!
        timestamp = mv_con.time_since('GPS_RAW_INT')
        print('Alt: {} - {}'.format(altitude, timestamp))
    except:
        print('No GPS_RAW_INT message received')



# Output:
# {'mavpackettype': 'AHRS2', 'roll': -0.11364290863275528, 'pitch': -0.02841472253203392, 'yaw': 2.0993032455444336, 'altitude': 0.0, 'lat': 0, 'lng': 0}
# {'mavpackettype': 'AHRS3', 'roll': 0.025831475853919983, 'pitch': 0.006112074479460716, 'yaw': 2.1514968872070312, 'altitude': 0.0, 'lat': 0, 'lng': 0, 'v1': 0.0, 'v2': 0.0, 'v3': 0.0, 'v4': 0.0}
# {'mavpackettype': 'VFR_HUD', 'airspeed': 0.0, 'groundspeed': 0.0, 'heading': 123, 'throttle': 0, 'alt': 3.129999876022339, 'climb': 3.2699999809265137}
# {'mavpackettype': 'AHRS', 'omegaIx': 0.0014122836291790009, 'omegaIy': -0.022567369043827057, 'omegaIz': 0.02394154854118824, 'accel_weight': 0.0, 'renorm_val': 0.0, 'error_rp': 0.08894175291061401, 'error_yaw': 0.0990816056728363}