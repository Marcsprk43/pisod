from pymavlink import mavutil
import time

def request_message_interval(master, message_id: int, frequency_hz: float):
    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
    """
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0, # Unused parameters
        0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )


# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600,
                                            dialect='ardupilotmega', autoreconnect=True)

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

the_connection.mav.system_time_send(0, 0)
mavutil.mavlink.MAVLINK_MSG_ID_AHRS2
print('sending sys_status update rate to 30ms')
request_message_interval(the_connection, 1, 10)
request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 10)

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
