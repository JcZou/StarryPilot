Header header

# We use the coordinate frames with the following convention:
#   x: forward
#   y: left
#   z: up

# rotation convention (z-y'-x''):
# yaw rotates around fixed frame's z axis
# pitch rotates around new y-axis (y')
# roll rotates around new x-axis (x'')

# This is a convenience-message to support that low-level (microcontroller-based) state
# estimators may not have knowledge about the absolute yaw.
# Roll- and pitch-angle should be specified in the header/frame_id frame
float64 roll                   # Roll angle [rad]
float64 pitch                  # Pitch angle  [rad]
float64 yaw_rate               # Yaw rate around z-axis [rad/s]

geometry_msgs/Vector3 thrust   # Thrust [N] expressed in the body frame.
                               # For a fixed-wing, usually the x-component is used.
                               # For a multi-rotor, usually the z-component is used.
                               # Set all un-used components to 0.
