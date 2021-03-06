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
    
    if (frequency_hz > 0):
        message_freq_us = 1e6 / frequency_hz
    elif (frequency_hz == 0):
        message_freq_us = 0   # leave default
    else:
        message_freq_us = -1   # disable

    print('Requesting message frequency: {}  -  {}hz  {}us'.format(message_id, frequency_hz, message_freq_us))

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        message_freq_us, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0, # Unused parameters
        0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )


# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600,
                                            dialect='ardupilotmega', autoreconnect=True)

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1)
request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 50)
request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT  , 1)
request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_POWER_STATUS  , 1)
request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_MEMINFO  , 1)
request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT  , 1)
request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT  , 1)
request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU  , 1)
request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2  , 1)
request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE  , 1)
request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT  , 1)
request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME  , 1)
request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_AHRS  , 1)
request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_HWSTATUS  , 1)
request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_TERRAIN_REPORT  , 1)
request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT  , 1)
request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_VIBRATION  , 1)
request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE  , 1)
request_message_interval(the_connection, mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD  , 1)



time.sleep(5)

count = 0 
while ( count < 100 ):
    try:
       print(the_connection.recv_match(blocking=False).to_dict())
    except:
        pass
    time.sleep(0.1)
    count += 1

timestamp = 1000.0

count = 0

message_dict = {}

while ( count < 10000 ):
    # Once connected, use 'the_connection' to get and send messages
    time.sleep(0.03)  # this is approimately 30 fps
    
    try: 
        msg = the_connection.recv_match(blocking=True).to_dict()
        if (msg):
            
            count += 1

            if msg['mavpackettype'] in message_dict.keys():
                message_dict[msg['mavpackettype']] += 1
            else:
                message_dict[msg['mavpackettype']] = 1

            #the_connection.recv_match(blocking=False)
            lat = the_connection.messages['AHRS2'].lat  # Note, you can access message fields as attributes!
            lng = the_connection.messages['AHRS2'].lng  # Note, you can access message fields as attributes!
            altitude = the_connection.messages['AHRS2'].altitude  # Note, you can access message fields as attributes!
            voltage = the_connection.messages['SYS_STATUS'].voltage_battery  # Note, you can access message fields as attributes!
            #battery_remaining = the_connection.messages['SYS_STATUS'].battery_remaining
            
            if (the_connection.time_since('AHRS2') < timestamp):   # this means a new message has been received
                print('{} Alt:  {} - Time since last read: {}'.format(count, altitude, timestamp))

            timestamp = the_connection.time_since('AHRS2')

    except Exception as e:
        print(e)


print(message_dict)