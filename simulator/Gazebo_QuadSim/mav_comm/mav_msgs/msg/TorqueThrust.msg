Header header

# We use the coordinate frames with the following convention:
#   x: forward
#   y: left
#   z: up

geometry_msgs/Vector3 torque  # Torque expressed in the body frame [Nm].
geometry_msgs/Vector3 thrust  # Thrust [N] expressed in the body frame. 
                              # For a fixed-wing, usually the x-component is used.
                              # For a multi-rotor, usually the z-component is used. 
                              # Set all un-used components to 0.  
