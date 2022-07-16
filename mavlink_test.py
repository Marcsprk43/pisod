from pymavlink import mavutil
import time

class SimpleMavlink:

    def __init__(self, dev, baud, verbose=False):

        self.device = dev
        self.baud = baud
        self.verbose = verbose

        self.conn = mavutil.mavlink_connection(self.device, self.baud)

        if (verbose):
            print('Connecting to : {}'.format(self.device))
            print('Waiting for heartbeat')
        

    def wait_heartbeat(self):
        self.conn.wait_heartbeat()
        if (self.verbose):
            print("Heartbeat from system received")


    def ping_conn(self):
        """
        Sends a ping to stabilish the UDP communication and awaits for a response
        """
        msg = None
        while not msg:
            self.conn.mav.ping_send(
                int(time.time() * 1e6), # Unix time in microseconds
                0, # Ping number
                0, # Request ping of all systems
                0 # Request ping of all components
            )
            msg = self.conn.recv_match()
            time.sleep(0.5)
        if (self.verbose):
            print('Response from ping....{}'.format(msg))


# Start a connection listening on a UDP port
mv_con = SimpleMavlink('/dev/ttyAMA0', 57600, verbose=True)

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
mv_con.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (mv_con.conn.target_system, mv_con.conn.target_component))

# Once connected, use 'the_connection' to get and send messages

# Send a ping to start connection and wait for any reply.
#  This function is necessary when using 'udpout',
#  as described before, 'udpout' connects to 'udpin',
#  and needs to send something to allow 'udpin' to start
#  sending data.

mv_con.ping_conn()

# Get some information !
while True:

    try: 
        altitude = mv_con.conn.messages['VIBRATION'].vibration_x  # Note, you can access message fields as attributes!
        timestamp = mv_con.conn.time_since('GPS_RAW_INT')
        print('Alt: {} - {}'.format(altitude, timestamp))
    except Exception as e:
        print(e)

    time.sleep(0.1)



# Output:
# {'mavpackettype': 'AHRS2', 'roll': -0.11364290863275528, 'pitch': -0.02841472253203392, 'yaw': 2.0993032455444336, 'altitude': 0.0, 'lat': 0, 'lng': 0}
# {'mavpackettype': 'AHRS3', 'roll': 0.025831475853919983, 'pitch': 0.006112074479460716, 'yaw': 2.1514968872070312, 'altitude': 0.0, 'lat': 0, 'lng': 0, 'v1': 0.0, 'v2': 0.0, 'v3': 0.0, 'v4': 0.0}
# {'mavpackettype': 'VFR_HUD', 'airspeed': 0.0, 'groundspeed': 0.0, 'heading': 123, 'throttle': 0, 'alt': 3.129999876022339, 'climb': 3.2699999809265137}
# {'mavpackettype': 'AHRS', 'omegaIx': 0.0014122836291790009, 'omegaIy': -0.022567369043827057, 'omegaIz': 0.02394154854118824, 'accel_weight': 0.0, 'renorm_val': 0.0, 'error_rp': 0.08894175291061401, 'error_yaw': 0.0990816056728363}