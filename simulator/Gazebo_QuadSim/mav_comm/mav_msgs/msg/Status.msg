Header header

# If values are not known / available, set to -1 or empty string.
string      vehicle_name
string      vehicle_type                  # E.g. firefly, pelican ...
float32     battery_voltage               # Battery voltage in V.
string      rc_command_mode               # Command mode set on the 3 position switch on the rc.
bool        command_interface_enabled     # Reports whether the serial command interface is enabled.
float32     flight_time                   # Flight time in s.
float32     system_uptime									# MAV uptime in s.
float32     cpu_load                      # MAV CPU load: 0.0 ... 1.0

string      motor_status                  # Current motor status: running, stopped, starting, stopping.
bool        in_air                        # True if vehicle is actually in air, false otherwise

string      gps_status                    # GPS status: lock, no_lock
int32       gps_num_satellites            # Number of visible satellites

string RC_COMMAND_ATTITUDE="attitude_thrust"
string RC_COMMAND_ATTITUDE_HEIGHT="attitude_height"
string RC_COMMAND_POSITION="position"

string MOTOR_STATUS_RUNNING="running"
string MOTOR_STATUS_STOPPED="stopped"
string MOTOR_STATUS_STARTING="starting"
string MOTOR_STATUS_STOPPING="stopping"

string GPS_STATUS_LOCK="lock"
string GPS_STATUS_NO_LOCK="no_lock"
