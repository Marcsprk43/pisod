from pymavlink import mavutil

# Start a connection listening on a UDP port
mv_con = mavutil.mavlink_connection('/dev/ttyAMA0')

print('Connecting to /dev/ttyAMA0')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
mv_con.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (mv_con.target_system, mv_con.target_component))

# Once connected, use 'the_connection' to get and send messages